#include "wall.h"
#include "line.h" 
#include <Arduino.h>
#include "common.h"

// Basic Motor Control
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
  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout
  if (duration == 0) return -1;
  long distance = duration * 0.034 / 2;
  return distance;
}

// Encoder ISRs
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

// 180 Pivot
void pivot180(int pwmVal) {
  const int targetCounts = 550; 
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

// 90 Degree Turn
void pivotTurn90(bool leftTurn, int pwmOuterMax) {
  const long TICKS_90_DEG = 560;   
  const int pwmMin = 40;
  int pwm = constrain(pwmOuterMax, pwmMin, 255);

  encoderCountL = 0;
  encoderCountR = 0;

  if (leftTurn) {
    analogWrite(L_RPWM, 0);  analogWrite(L_LPWM, 0);
    analogWrite(R_RPWM, pwm);    analogWrite(R_LPWM, 0);
  } else {
    analogWrite(L_RPWM, 0);    analogWrite(L_LPWM, pwm);
    analogWrite(R_RPWM, 0);  analogWrite(R_LPWM, 0);
  }

  while (true) {
    long aL = absl(encoderCountL);
    long aR = absl(encoderCountR);
    if (aL >= TICKS_90_DEG || aR >= TICKS_90_DEG) break;
    delay(5);
  }
  stopLeft();
  stopRight();
}

void smoothTurnLeft()  { pivotTurn90(true, TURN_PWM); }
void smoothTurnRight() { pivotTurn90(false, TURN_PWM); }

// ===============================================================
// FIXED: moveForwardWallFollow returns BOOL (true=success)
// ===============================================================
bool moveForwardWallFollow(int distance_cm, int basePWM) {
  
  const int targetCounts = 525;

  // Reset encoders
  noInterrupts();
  encoderCountL = 0;
  encoderCountR = 0;
  interrupts();

  unsigned long tStart = millis();
  // Timeout: if we don't move in time, we are likely stuck
  const unsigned long TIMEOUT_MS = max(5000UL, (unsigned long)distance_cm * 60UL); 

  bool success = false;

  while (true) {
    // 1. TIMEOUT CHECK
    if (millis() - tStart > TIMEOUT_MS) {
      stopMotors();
      success = false; // Failed to reach target
      break;
    }

    // 2. ENCODER CHECK
    noInterrupts();
    long aL = absl(encoderCountL);
    long aR = absl(encoderCountR);
    interrupts();

    if ((aL + aR) / 2 >= targetCounts) {
      stopMotors();
      success = true; // SUCCESS
      break;
    }

    // 3. SENSOR READ
    long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
    long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

    bool frontValid = dFront > 0;
    bool leftValid  = dLeft  > 0;
    bool rightValid = dRight > 0;

    // 5. WALL FOLLOW PID
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
      int we = (int)(targetDist - dLeft);
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
    }

    analogWrite(R_RPWM, rightPWM); analogWrite(R_LPWM, 0);
    analogWrite(L_RPWM, 0); analogWrite(L_LPWM, leftPWM);
    delay(8);
  }

  stopMotors();
  delay(500);
  return success;
}

bool moveBackwardWallFollow(int basePWM) {
  
  const int targetCounts = 280;

  // Reset encoders
  noInterrupts();
  encoderCountL = 0;
  encoderCountR = 0;
  interrupts();

  unsigned long tStart = millis();
  // Timeout: if we don't move in time, we are likely stuck
  const unsigned long TIMEOUT_MS = 1500UL; 

  bool success = false;

  while (true) {
    // 1. TIMEOUT CHECK
    if (millis() - tStart > TIMEOUT_MS) {
      stopMotors();
      success = false; // Failed to reach target
      break;
    }

    // 2. ENCODER CHECK
    noInterrupts();
    long aL = absl(encoderCountL);
    long aR = absl(encoderCountR);
    interrupts();

    if ((aL + aR) / 2 >= targetCounts) {
      stopMotors();
      success = true; // SUCCESS
      break;
    }

    // 3. SENSOR READ
    long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
    long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

    bool frontValid = dFront > 0;
    bool leftValid  = dLeft  > 0;
    bool rightValid = dRight > 0;

    // 5. WALL FOLLOW PID
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
    }

    analogWrite(R_RPWM, 0); analogWrite(R_LPWM, leftPWM);
    analogWrite(L_RPWM, rightPWM); analogWrite(L_LPWM, 0);
    delay(8);
  }

  stopMotors();
  delay(500);
  return success;
}
// General Wall Following (Infinite Mode)
void wallFollowingMode() {
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool frontValid = dFront > 0;
  bool leftValid  = dLeft  > 0;
  bool rightValid = dRight > 0;

  // Debug output
  Serial.print("F:"); Serial.print(dFront);
  Serial.print(" L:"); Serial.print(dLeft);
  Serial.print(" R:"); Serial.println(dRight);

  // Dead End
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
  // Obstacle Ahead
  else if (frontValid && dFront < OBSTACLE_THRESHOLD) {
    stopMotors();
    delay(100);
    bool turnLeft = (dLeft > dRight);
    
    // Check for open space
    bool isUTurn = false;
    if (turnLeft && dLeft > OPEN_SPACE_THRESHOLD_CM)   isUTurn = true;
    if (!turnLeft && dRight > OPEN_SPACE_THRESHOLD_CM) isUTurn = true;

    // Align if stuck in corner
    if (!isUTurn && leftValid && rightValid && (abs(dLeft - dRight) > ALIGN_TOLERANCE_CM)) {
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
  // Left Wall Gap
  else if (leftValid && dLeft > NO_WALL_THRESHOLD) {
    moveForwardWallFollow(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
    stopMotors();
    delay(10);
    smoothTurnLeft();
    stopMotors();
    delay(150);
  }
  // Follow Wall
  else {
    int leftSpeed = BASE_PWM_STRAIGHT, rightSpeed = BASE_PWM_STRAIGHT;
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
    moveForward(rightSpeed, leftSpeed);
  }
  delay(10);
}