// src/WallFollower.cpp
#include <Arduino.h>
#include "RobotConfig.h"
#include "RobotState.h"

// ===================================================================================
// Constants & Tuning (UNCHANGED)
// ===================================================================================
static const float WHEEL_DIAMETER_CM = 6.5f;
static const int pulsesPerMotorRev   = 11;
static const int gearRatio           = 20;
static const int countsPerRev        = pulsesPerMotorRev * gearRatio * 2;

static const int   TURN_PWM_MAZE  = 50;
static const int   PIVOT_180_PWM  = 55;
static const int   TICKS_90_DEG   = 560;

static const int OBSTACLE_THRESHOLD      = 8;
static const int OPEN_SPACE_THRESHOLD_CM = 30;
static const int DEAD_END_THRESHOLD      = 10;
static const int NO_WALL_THRESHOLD       = 25;
static const int CORNER_CLEARANCE_CM     = 10;

static const int   BASE_PWM_STRAIGHT = 60;
static const float Kp_Wall           = 3.5f;
static const float Kd_Wall           = 10.0f; // (not used in original PID_Logic_Wall, kept)
static const int   MAX_CORRECTION    = 20;
static const int   WALL_DETECT_RANGE = 20;

static const int ALIGN_PWM          = 45;
static const int ALIGN_DURATION_MS  = 50;
static const int ALIGN_TOLERANCE_CM = 1;

// ===================================================================================
// Finish / White box detection (FIXED)
// ===================================================================================
// Old value 2 was too low and can trigger randomly inside black maze.
// Require strong white (white box) + debounce.
static const int      FINISH_LINE_SENSITIVITY = 6;     // TODO tune: 6 or 7
static const uint16_t FINISH_CONFIRM_MS       = 120;   // TODO tune: 80-150
static unsigned long  finishWhiteStart        = 0;

// ===================================================================================
// Encoder Globals (UNCHANGED behavior)
// ===================================================================================
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

// ===================================================================================
// Small helpers
// ===================================================================================
static inline long absl(long x){ return (x < 0) ? -x : x; }
static inline void stopRight(){ analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0); }
static inline void stopLeft() { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0); }

// ===================================================================================
// Local sensor buffer (for finish detection same as original)
// ===================================================================================
static uint8_t sensorVal[8];
static uint8_t sensorBits = 0;

// ===================================================================================
// Prototypes (same as original)
// ===================================================================================
static void readLineSensors();
static long readUltrasonic(int trigPin, int echoPin);

static void moveForward(int pwmValR, int pwmValL);
static void moveForwardDistance(int distance_cm, int pwmVal);
static void stopMotors();

static void pivotTurn90_Maze(bool leftTurn, int pwmOuterMax);
static void pivot180(int pwmVal);
static void smoothTurnLeft();
static void smoothTurnRight();
static void pivotLeft_Maze(int pwmVal);
static void pivotRight_Maze(int pwmVal);

static void PID_Logic_Wall(long dLeft, long dRight);

// ===================================================================================
// Public begin
// ===================================================================================
void wallFollowerBegin() {
  // Ultrasonics
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // Motors
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);

  // Encoders
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);

  // Line sensors (used for finish detection)
  for (int i=0;i<8;i++) pinMode(LINE_SENSOR_PINS[i], INPUT);

  finishWhiteStart = 0;
  stopMotors();
}

// ===================================================================================
// Public: Maze Mapping Update (records path via mazeSolverRecord)
// ===================================================================================
void wallFollowerMappingUpdate() {
  readLineSensors();

  int whiteCount = 0;
  for (int i=0;i<8;i++) if (sensorVal[i] == 0) whiteCount++;

  // -------- FIXED finish detection (debounced, strong white) --------
  if (whiteCount >= FINISH_LINE_SENSITIVITY) {
    if (finishWhiteStart == 0) finishWhiteStart = millis();
    if (millis() - finishWhiteStart >= FINISH_CONFIRM_MS) {
      stopMotors();
      mazeSolverRecord('S');     // same behavior as your code
      finishWhiteStart = 0;
      return;                   // main.cpp decides state transition
    }
  } else {
    finishWhiteStart = 0;
  }

  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool frontValid = (dFront > 0);
  bool leftValid  = (dLeft  > 0);
  bool rightValid = (dRight > 0);

  // Dead end
  if (frontValid && dFront < OBSTACLE_THRESHOLD &&
      leftValid  && dLeft  < DEAD_END_THRESHOLD &&
      rightValid && dRight < DEAD_END_THRESHOLD) {

    stopMotors(); delay(100);
    pivot180(PIVOT_180_PWM);
    mazeSolverRecord('B');
    stopMotors(); delay(100);
    return;
  }

  // Front blocked
  if (frontValid && dFront < OBSTACLE_THRESHOLD) {
    stopMotors(); delay(100);

    if (leftValid && rightValid && (abs(dLeft - dRight) > ALIGN_TOLERANCE_CM)) {
      if (dLeft < dRight) pivotLeft_Maze(ALIGN_PWM);
      else                pivotRight_Maze(ALIGN_PWM);
      delay(ALIGN_DURATION_MS);
      stopMotors(); delay(100);
    }

    if (leftValid && dLeft > NO_WALL_THRESHOLD) {
      smoothTurnLeft();
      mazeSolverRecord('L');
    } else {
      smoothTurnRight();
      mazeSolverRecord('R');
    }

    stopMotors(); delay(100);
    return;
  }

  // Left open corner
  if (leftValid && dLeft > NO_WALL_THRESHOLD) {
    moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
    stopMotors(); delay(10);

    smoothTurnLeft();
    mazeSolverRecord('L');

    stopMotors(); delay(150);
    return;
  }

  // Straight preference
  if (rightValid && dRight > NO_WALL_THRESHOLD) {
    if (dFront > OPEN_SPACE_THRESHOLD_CM) {
      mazeSolverRecord('S');
      moveForwardDistance(15, BASE_PWM_STRAIGHT);
      return;
    }
  }

  // Wall follow PID
  PID_Logic_Wall(dLeft, dRight);
}

// ===================================================================================
// Public: Maze Solving Update (replays simplified path)
// ===================================================================================
void wallFollowerSolvingUpdate() {
  readLineSensors();

  int whiteCount = 0;
  for (int i=0;i<8;i++) if (sensorVal[i] == 0) whiteCount++;

  // -------- FIXED finish detection (debounced, strong white) --------
  if (whiteCount >= FINISH_LINE_SENSITIVITY) {
    if (finishWhiteStart == 0) finishWhiteStart = millis();
    if (millis() - finishWhiteStart >= FINISH_CONFIRM_MS) {
      stopMotors();
      finishWhiteStart = 0;
      return; // main.cpp handles state transition
    }
  } else {
    finishWhiteStart = 0;
  }

  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool leftOpen     = (dLeft  > NO_WALL_THRESHOLD);
  bool rightOpen    = (dRight > NO_WALL_THRESHOLD);
  bool frontBlocked = (dFront > 0 && dFront < OBSTACLE_THRESHOLD);

  if (leftOpen || rightOpen || frontBlocked) {
    stopMotors(); delay(100);

    if (frontBlocked && dLeft > 0 && dRight > 0 && abs(dLeft - dRight) > ALIGN_TOLERANCE_CM) {
      if (dLeft < dRight) pivotLeft_Maze(ALIGN_PWM);
      else                pivotRight_Maze(ALIGN_PWM);
      delay(ALIGN_DURATION_MS);
      stopMotors();
    }

    bool hasMove=false;
    char move = mazeSolverGetNextMove(&hasMove);
    if (hasMove) {
      DBG_PORT.print("BT: Doing -> "); DBG_PORT.println(move);

      if (move == 'L') {
        moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
        stopMotors(); delay(10);
        smoothTurnLeft();
        moveForward(BASE_PWM_STRAIGHT, BASE_PWM_STRAIGHT); delay(100);
        stopMotors(); delay(100);
      } else if (move == 'R') {
        moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
        stopMotors(); delay(10);
        smoothTurnRight();
        moveForward(BASE_PWM_STRAIGHT, BASE_PWM_STRAIGHT); delay(100);
        stopMotors(); delay(100);
      } else if (move == 'S') {
        moveForwardDistance(15, BASE_PWM_STRAIGHT);
      } else {
        PID_Logic_Wall(dLeft, dRight);
      }
    } else {
      PID_Logic_Wall(dLeft, dRight);
    }

  } else {
    PID_Logic_Wall(dLeft, dRight);
  }
}

// ===================================================================================
// Helpers (UNCHANGED behavior)
// ===================================================================================
static void PID_Logic_Wall(long dLeft, long dRight) {
  int leftSpeed, rightSpeed;
  bool leftValid = (dLeft > 0);
  bool rightValid = (dRight > 0);

  if (leftValid && rightValid && dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
    int error = (int)(dLeft - dRight);
    int correction = (int)(Kp_Wall * error);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
    rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
  } else if (leftValid && dLeft < 10) {
    int targetDist = 6;
    int error = (int)(targetDist - dLeft);
    int correction = (int)(Kp_Wall * error);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    leftSpeed  = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
    rightSpeed = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
  } else if (rightValid && dRight < 10) {
    int targetDist = 6;
    int error = (int)(targetDist - dRight);
    int correction = (int)(Kp_Wall * error);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
    rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
  } else {
    leftSpeed = BASE_PWM_STRAIGHT;
    rightSpeed = BASE_PWM_STRAIGHT;
  }

  moveForward(rightSpeed, leftSpeed);
  delay(10);
}

static void pivotTurn90_Maze(bool leftTurn, int pwmOuterMax) {
  int pwm = constrain(pwmOuterMax, 40, 255);
  encoderCountL = 0; encoderCountR = 0;

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

  moveForward(BASE_PWM_STRAIGHT, BASE_PWM_STRAIGHT);
  delay(50);
  stopMotors();
}

static void pivot180(int pwmVal) {
  const int targetCounts = 550;
  encoderCountL = 0; encoderCountR = 0;

  pivotRight_Maze(pwmVal);
  while (absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    if (absl(encoderCountL) >= targetCounts) stopLeft();
    if (absl(encoderCountR) >= targetCounts) stopRight();
    delay(5);
  }
  stopMotors();
}

static void smoothTurnLeft()  { pivotTurn90_Maze(true, TURN_PWM_MAZE); }
static void smoothTurnRight() { pivotTurn90_Maze(false, TURN_PWM_MAZE); }

static void pivotLeft_Maze(int pwmVal) {
  analogWrite(R_RPWM, pwmVal); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, pwmVal); analogWrite(L_LPWM, 0);
}

static void pivotRight_Maze(int pwmVal){
  analogWrite(L_RPWM, 0);      analogWrite(L_LPWM, pwmVal);
  analogWrite(R_RPWM, 0);      analogWrite(R_LPWM, pwmVal);
}

static void moveForward(int pwmValR, int pwmValL) {
  analogWrite(R_RPWM, pwmValR); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);       analogWrite(L_LPWM, pwmValL);
}

static void moveForwardDistance(int distance_cm, int pwmVal) {
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const long targetCounts = (long)(((float)distance_cm / wheelCircumference) * countsPerRev);

  noInterrupts(); encoderCountL = 0; encoderCountR = 0; interrupts();
  moveForward(pwmVal, pwmVal);

  while (true) {
    noInterrupts(); long cL = encoderCountL; long cR = encoderCountR; interrupts();
    if ((absl(cL) + absl(cR)) / 2 >= targetCounts) break;
    delay(10);
  }
  stopMotors();
}

static void readLineSensors() {
  sensorBits = 0;
  for (int i=0;i<8;i++) {
    int raw = digitalRead(LINE_SENSOR_PINS[i]);
    bool isBlack = BLACK_IS_HIGH ? (raw == HIGH) : (raw == LOW);
    sensorVal[i] = isBlack ? 1 : 0;
    if (isBlack) sensorBits |= (1u << i);
  }
}

static long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}

static void stopMotors() { stopLeft(); stopRight(); }

// ===== Encoder ISRs (must be global linkage, so keep here) =====
void countEncoderL();
void countEncoderR();

void countEncoderL() {
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) encoderCountL++;
  else encoderCountL--;
}

void countEncoderR() {
  if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) encoderCountR--;
  else encoderCountR++;
}
