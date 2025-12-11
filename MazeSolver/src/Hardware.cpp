// Hardware.cpp
#include "Hardware.h"

volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

// ============================================================================
// Internal helpers
// ============================================================================
inline void stopRightMotor() { analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0); }
inline void stopLeftMotor()  { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0); }

long absl(long x) { return (x < 0) ? -x : x; }

// ============================================================================
// Motors
// ============================================================================
void initMotors() {
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);

  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);
}

void stopMotors() {
  stopLeftMotor();
  stopRightMotor();
}

void moveForward(int pwmRight, int pwmLeft) {
  analogWrite(R_RPWM, pwmRight); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);        analogWrite(L_LPWM, pwmLeft);
}

void pivotLeft(int pwmVal) {
  // both wheels forward for left pivot (depending on mechanical orientation)
  analogWrite(R_RPWM, pwmVal); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, pwmVal); analogWrite(L_LPWM, 0);
}

void pivotRight(int pwmVal) {
  // both wheels backward (or opposite) for right pivot
  analogWrite(L_RPWM, 0);      analogWrite(L_LPWM, pwmVal);
  analogWrite(R_RPWM, 0);      analogWrite(R_LPWM, pwmVal);
}

void moveForwardDistance(int distance_cm, int pwmVal) {
  const float wheelCirc = PI * WHEEL_DIAMETER_CM;
  const long targetCounts = (long)((distance_cm / wheelCirc) * countsPerRev);

  resetEncoders();
  moveForward(pwmVal, pwmVal);

  while (true) {
    if (getEncoderAvg() >= targetCounts) break;
    delay(5);
  }
  stopMotors();
}

void pivot180(int pwmVal) {
  const long targetCounts = 550; // TODO: tune
  resetEncoders();
  pivotRight(pwmVal);

  while (absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    if (absl(encoderCountL) >= targetCounts) stopLeftMotor();
    if (absl(encoderCountR) >= targetCounts) stopRightMotor();
    delay(5);
  }
  stopMotors();
}

void pivotTurn90(bool leftTurn, int pwmOuterMax) {
  const long TICKS_90_DEG = 560;  // TODO: tune based on your robot
  int pwm = constrain(pwmOuterMax, 40, 255);

  resetEncoders();

  if (leftTurn) {
    // Right wheel forward, left stopped
    analogWrite(L_RPWM, 0);  analogWrite(L_LPWM, 0);
    analogWrite(R_RPWM, pwm); analogWrite(R_LPWM, 0);
  } else {
    // Left wheel backward (or forward depending on wiring), right stopped
    analogWrite(L_RPWM, 0);   analogWrite(L_LPWM, pwm);
    analogWrite(R_RPWM, 0);   analogWrite(R_LPWM, 0);
  }

  while (true) {
    if (absl(encoderCountL) >= TICKS_90_DEG || absl(encoderCountR) >= TICKS_90_DEG) break;
    delay(5);
  }
  stopMotors();
}

void smoothTurnLeft()  { pivotTurn90(true,  TURN_PWM); }
void smoothTurnRight() { pivotTurn90(false, TURN_PWM); }

// ============================================================================
// Encoders
// ============================================================================
static void countEncoderL_ISR() {
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) encoderCountL++;
  else encoderCountL--;
}

static void countEncoderR_ISR() {
  if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) encoderCountR--;
  else encoderCountR++;
}

void initEncoders() {
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), countEncoderL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), countEncoderR_ISR, CHANGE);
}

void resetEncoders() {
  noInterrupts();
  encoderCountL = 0;
  encoderCountR = 0;
  interrupts();
}

long getEncoderAvg() {
  noInterrupts();
  long cL = encoderCountL;
  long cR = encoderCountR;
  interrupts();
  return (absl(cL) + absl(cR)) / 2;
}

// ============================================================================
// Ultrasonic
// ============================================================================
void initUltrasonic() {
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000); // timeout ~25ms
  if (duration == 0) return 999;  // no echo, treat as far
  return (long)(duration * 0.034 / 2.0); // microseconds to cm
}

long readFrontDistance() { return readUltrasonic(TRIG_FRONT, ECHO_FRONT); }
long readLeftDistance()  { return readUltrasonic(TRIG_LEFT,  ECHO_LEFT); }
long readRightDistance() { return readUltrasonic(TRIG_RIGHT, ECHO_RIGHT); }
