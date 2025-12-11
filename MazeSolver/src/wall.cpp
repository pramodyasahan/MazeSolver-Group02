#include "wall.h"
#include "line.h" // for stopMotors prototype if needed (stopMotors implemented here too)
#include <Arduino.h>
#include "common.h"

// Implement motor helpers and wall functions (moved from original main file)

// Move forward with given wheel PWM values (right, left)
void moveForward(int pwmValR, int pwmValL) {
  analogWrite(R_RPWM, pwmValR); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);       analogWrite(L_LPWM, pwmValL);
}

void moveBackward(int pwmVal) {
  analogWrite(R_RPWM, 0);       analogWrite(R_LPWM, pwmVal);
  analogWrite(L_RPWM, pwmVal);  analogWrite(L_LPWM, 0);
}

void pivotLeft(int pwmVal) {
  analogWrite(R_RPWM, pwmVal); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, pwmVal); analogWrite(L_LPWM, 0);
}

void pivotRight(int pwmVal) {
  analogWrite(L_RPWM, 0);      analogWrite(L_LPWM, pwmVal);
  analogWrite(R_RPWM, 0);      analogWrite(R_LPWM, pwmVal);
}

void stopMotors() {
  stopLeft();
  stopRight();
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return -1;
  long distance = duration * 0.034 / 2;
  return distance;
}

void countEncoderL() {
  int A = digitalRead(L_ENC_A);
  int B = digitalRead(L_ENC_B);
  if (A == B) { encoderCountL++; }
  else        { encoderCountL--; }
}

void countEncoderR() {
  int A = digitalRead(R_ENC_A);
  int B = digitalRead(R_ENC_B);
  if (A == B) { encoderCountR--; } 
  else        { encoderCountR++;}
}

// ===============================================================
//             180 Degree Encoder-Based Pivot
// ===============================================================
void pivot180(int pwmVal) {
  const int targetCounts = 550; // Approximate value for 180-degree pivot

  encoderCountL = 0;
  encoderCountR = 0;

  pivotRight(pwmVal);

  while(absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    if (absl(encoderCountL) >= targetCounts) { stopLeft(); }
    if (absl(encoderCountR) >= targetCounts) { stopRight(); }
    delay(5);
  }

  stopMotors();
}

// ===============================================================
//                  TURNING (Arc with sync + stop-on-reach)
// ===============================================================
void pivotTurn90(bool leftTurn, int pwmOuterMax) {
  const long TICKS_90_DEG = 560;   
  const int pwmMin = 40;

  int pwm = constrain(pwmOuterMax, pwmMin, 255);

  // Reset encoders
  encoderCountL = 0;
  encoderCountR = 0;

  // --- Motor commands for pivot ---
  if (leftTurn) {
    // Left turn: left wheel backward, right wheel forward
    analogWrite(L_RPWM, 0);  analogWrite(L_LPWM, 0);
    analogWrite(R_RPWM, pwm);    analogWrite(R_LPWM, 0);
  } else {
    // Right turn: right wheel backward, left wheel forward
    analogWrite(L_RPWM, 0);    analogWrite(L_LPWM, pwm);
    analogWrite(R_RPWM, 0);  analogWrite(R_LPWM, 0);
  }

  // --- Run until encoder reaches tick target ---
  while (true) {
    long aL = absl(encoderCountL);
    long aR = absl(encoderCountR);

    if (aL >= TICKS_90_DEG || aR >= TICKS_90_DEG) {
      break;
    }
    delay(5);
  }

  // Stop both wheels
  stopLeft();
  stopRight();
}

void smoothTurnLeft()  { pivotTurn90(true, TURN_PWM); }
void smoothTurnRight() { pivotTurn90(false, TURN_PWM); }

// ===============================================================
//             NEW: Encoder-Based Forward Movement
// ===============================================================
void moveForwardWallFollow(int distance_cm, int basePWM) {
  if (distance_cm <= 0) return;

  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const long targetCounts = (long)(((float)distance_cm / wheelCircumference) * countsPerRev);

  // Reset encoders
  noInterrupts();
  encoderCountL = 0;
  encoderCountR = 0;
  interrupts();

  unsigned long tStart = millis();
  const unsigned long TIMEOUT_MS = max(5000UL, (unsigned long)distance_cm * 60UL); // conservative timeout

  while (true) {
    // Safety timeout
    if (millis() - tStart > TIMEOUT_MS) {
      stopMotors();
      // Serial.println("moveForwardWallFollow: timeout");
      break;
    }

    // Encoder check
    noInterrupts();
    long aL = absl(encoderCountL);
    long aR = absl(encoderCountR);
    interrupts();

    if ((aL + aR) / 2 >= targetCounts) {
      // reached distance
      stopMotors();
      break;
    }

    // Read ultrasonics (non-blocking-ish; pulseIn still blocks)
    long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
    long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

    bool frontValid = dFront > 0;
    bool leftValid  = dLeft  > 0;
    bool rightValid = dRight > 0;

    // Obstacle immediately ahead: stop and choose avoidance
    if (frontValid && dFront < OBSTACLE_THRESHOLD) {
      stopMotors();
      delay(50);
      // Align / decide turn like wallFollowingMode
      bool turnLeft = (dLeft > dRight);
      bool isUTurn = false;
      if (turnLeft && dLeft > OPEN_SPACE_THRESHOLD_CM)   isUTurn = true;
      if (!turnLeft && dRight > OPEN_SPACE_THRESHOLD_CM) isUTurn = true;

      if (!isUTurn && leftValid && rightValid && (abs(dLeft - dRight) > ALIGN_TOLERANCE_CM)) {
        if (dLeft < dRight) { pivotLeft(ALIGN_PWM); }
        else                { pivotRight(ALIGN_PWM); }
        delay(ALIGN_DURATION_MS);
        stopMotors();
        delay(50);
      }

      if (turnLeft) smoothTurnLeft();
      else          smoothTurnRight();

      // after avoidance, continue outer loop which will re-evaluate encoders/timeouts
      continue;
    }

    // WALL FOLLOW PID (dual-wall, left-wall or right-wall fallback)
    int leftPWM = basePWM;
    int rightPWM = basePWM;

    if (leftValid && rightValid && dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
      int we = (int)(dLeft - dRight);
      int corr = (int)(Kp_Wall * we);
      corr = constrain(corr, -MAX_CORRECTION, MAX_CORRECTION);
      leftPWM  = constrain(basePWM - corr, 0, 255);
      rightPWM = constrain(basePWM + corr, 0, 255);
    } else if (leftValid && dLeft < 10) {
      int targetDist = 6;
      int we = (int)(dLeft - targetDist);
      int corr = (int)(Kp_Wall * we);
      corr = constrain(corr, -MAX_CORRECTION, MAX_CORRECTION);
      leftPWM  = constrain(basePWM + corr, 0, 255);
      rightPWM = constrain(basePWM - corr, 0, 255);
    } else if (rightValid && dRight < 10) {
      int targetDist = 6;
      int we = (int)(targetDist - dRight);
      int corr = (int)(Kp_Wall * we);
      corr = constrain(corr, -MAX_CORRECTION, MAX_CORRECTION);
      leftPWM  = constrain(basePWM - corr, 0, 255);
      rightPWM = constrain(basePWM + corr, 0, 255);
    } else {
      // no good wall info — drive straight at basePWM
      leftPWM = basePWM;
      rightPWM = basePWM;
    }

    // Apply PWMs (rightSpeed, leftSpeed ordering in your helpers)
    analogWrite(R_RPWM, rightPWM); analogWrite(R_LPWM, 0);
    analogWrite(L_RPWM, 0); analogWrite(L_LPWM, leftPWM);

    // small delay for loop timing
    delay(8);
  }

  // ensure stop & short settle
  stopMotors();
  delay(40);
}

// ===============================================================
//                 WALL FOLLOWING MODE FUNCTION
// ===============================================================
void wallFollowingMode() {
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool frontValid = dFront > 0;
  bool leftValid  = dLeft  > 0;
  bool rightValid = dRight > 0;

  Serial.print("F:"); Serial.print(dFront);
  Serial.print(" L:"); Serial.print(dLeft);
  Serial.print(" R:"); Serial.println(dRight);

  // Mirror to Bluetooth
  Serial3.print("F:"); Serial3.print(dFront);
  Serial3.print(" L:"); Serial3.print(dLeft);
  Serial3.print(" R:"); Serial3.println(dRight);

  // ---------- STATE 1: Dead End ----------
  if (frontValid && dFront < OBSTACLE_THRESHOLD &&
      leftValid  && dLeft  < DEAD_END_THRESHOLD &&
      rightValid && dRight < DEAD_END_THRESHOLD)
  {
      stopMotors();
      delay(100);
      pivot180(PIVOT_180_PWM);
      stopMotors();
      delay(100);
  }
  // ---------- STATE 2: Obstacle Ahead ----------
  else if (frontValid && dFront < OBSTACLE_THRESHOLD) {
    stopMotors();
    delay(100);

    bool turnLeft = (dLeft > dRight);
    bool isUTurn = false;
    if (turnLeft && dLeft > OPEN_SPACE_THRESHOLD_CM)   isUTurn = true;
    if (!turnLeft && dRight > OPEN_SPACE_THRESHOLD_CM) isUTurn = true;

    if (!isUTurn && leftValid && rightValid && (abs(dLeft - dRight) > ALIGN_TOLERANCE_CM)) {
      Serial.println("--- Aligning ---");
      Serial3.println("BT: Aligning");
      if (dLeft < dRight) { pivotLeft(ALIGN_PWM); }
      else                { pivotRight(ALIGN_PWM); }
      delay(ALIGN_DURATION_MS);
      stopMotors();
      delay(100);
    }

    if (turnLeft) smoothTurnLeft();
    else          smoothTurnRight();

    stopMotors();
    delay(100);
  }
  // ---------- STATE 3: No Wall on Left ----------
  else if (leftValid && dLeft > NO_WALL_THRESHOLD) {
    moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
    stopMotors();
    delay(10);
    smoothTurnLeft();
    stopMotors();
    delay(150);
  }
  // ---------- STATE 4: Path Clear (Follow Wall) ----------
  else {
    int leftSpeed, rightSpeed;

    if (leftValid && rightValid && dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
      int error = (int)(dLeft - dRight);
      int correction = (int)(Kp_Wall * error);
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
      leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
      rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
    }
    else if (leftValid && dLeft < 10) {
      int targetDist = 6;
      int error = (int)(dLeft - targetDist);
      int correction = (int)(Kp_Wall * error);
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
      leftSpeed  = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
      rightSpeed = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
    }
    else if (rightValid && dRight < 10) {
      int targetDist = 6;
      int error = (int)(targetDist - dRight);
      int correction = (int)(Kp_Wall * error);
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
      leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
      rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
    }
    else {
      leftSpeed = BASE_PWM_STRAIGHT;
      rightSpeed = BASE_PWM_STRAIGHT;
    }
    moveForward(rightSpeed, leftSpeed);
  }

  delay(10);
}