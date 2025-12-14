#include "line.h"
#include "wall.h" // for motor prototypes if needed
#include <Arduino.h>
#include <common.h>

// Read line sensors and populate sensorVal and sensorBits
void readSensors() {
  sensorBits = 0;
  for (int i = 0; i < 8; i++) {
    int raw = digitalRead(sensorPins[i]);
    bool isBlack = BLACK_IS_HIGH ? (raw == HIGH) : (raw == LOW);
    sensorVal[i] = isBlack ? 1 : 0;
    if (isBlack) sensorBits |= (1u << i);
  }
}

float getLineError() {
  static const float weight[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
  float sumW = 0.0f, sum = 0.0f;

  for (int i = 0; i < 8; i++) {
    if (sensorVal[i]) { sumW += weight[i]; sum += 1.0f; }
  }

  if (sum == 0.0f) return lastError;
  return sumW / sum;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  analogWrite(R_RPWM, rightSpeed);
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, leftSpeed);
}

bool anyOnLine() { return sensorBits != 0; }
bool centerOnLine() { return sensorVal[CENTER_IDX[0]] || sensorVal[CENTER_IDX[1]]; }

uint8_t countBlk(const uint8_t idxs[], uint8_t n) {
  uint8_t c = 0;
  for (uint8_t k = 0; k < n; k++) c += sensorVal[idxs[k]] ? 1 : 0;
  return c;
}

bool strongLeftFeature() {
  uint8_t L = countBlk(LEFT_IDX, 3);
  uint8_t R = countBlk(RIGHT_IDX, 3);
  return (L >= LEFT_STRONG_MIN) && (R <= 1);
}

bool strongRightFeature() {
  uint8_t L = countBlk(LEFT_IDX, 3);
  uint8_t R = countBlk(RIGHT_IDX, 3);
  return (R >= RIGHT_STRONG_MIN) && (L <= 1);
}

void brake(uint16_t ms) { stopMotors(); delay(ms); }

void pivotLeftUntilCenter() {
  Serial3.println("BT: Pivot Left (Center)");
  unsigned long t0 = millis();
  while (millis() - t0 < PIVOT_TIMEOUT_MS) {
    analogWrite(R_RPWM, TURN_PWM_LINE);
    analogWrite(R_LPWM, 0);
    analogWrite(L_RPWM, TURN_PWM_LINE);
    analogWrite(L_LPWM, 0);
    readSensors();
    if (centerOnLine()) break;
    delay(4);
  }
}

void pivotRightUntilCenter() {
  Serial3.println("BT: Pivot Right (Center)");
  unsigned long t0 = millis();
  while (millis() - t0 < PIVOT_TIMEOUT_MS) {
    analogWrite(R_RPWM, 0);
    analogWrite(R_LPWM, TURN_PWM_LINE);
    analogWrite(L_RPWM, 0);
    analogWrite(L_LPWM, TURN_PWM_LINE);
    readSensors();
    if (centerOnLine()) break;
    delay(4);
  }
}

void pivotLeft90UntilLine(int pwmVal) {
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const float turnDistance = (PI * WHEEL_BASE_CM) / 4.0f; // 1/4 of the full circle circumference
  const long targetCounts = (long)((turnDistance / wheelCircumference) * countsPerRev);

  Serial.println("Pivoting left (max 90 deg) until line is found...");
  Serial3.println("BT: Search Left");

  noInterrupts();
  encoderCountL = 0;
  encoderCountR = 0;
  interrupts();

  pivotLeft(pwmVal);

  bool lineWasFound = false;

  while (absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    readSensors();
    if (anyOnLine()) {
      lineWasFound = true;
      break;
    }
    if (absl(encoderCountL) >= targetCounts) { stopLeft(); }
    if (absl(encoderCountR) >= targetCounts) { stopRight(); }
    delay(5);
  }

  stopMotors();
}

void pivotRight90UntilLine(int pwmVal) {
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const float turnDistance = (PI * WHEEL_BASE_CM) / 4.0f;
  const long targetCounts = (long)((turnDistance / wheelCircumference) * countsPerRev);

  Serial.println("Pivoting right (max 90 deg) until line is found...");
  Serial3.println("BT: Search Right");

  noInterrupts();
  encoderCountL = 0;
  encoderCountR = 0;
  interrupts();

  pivotRight(pwmVal);

  bool lineWasFound = false;

  while (absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    readSensors();
    if (anyOnLine()) {
      lineWasFound = true;
      break;
    }
    if (absl(encoderCountL) >= targetCounts) { stopLeft(); }
    if (absl(encoderCountR) >= targetCounts) { stopRight(); }
    delay(5);
  }

  stopMotors();

  if (lineWasFound) {
    Serial.println("Line found! Stopping pivot.");
  } else {
    Serial.println("90-degree pivot completed, line was not found.");
  }
}

void searchWhenLost() {
  Serial3.println("BT: Lost! Searching...");
  pivotLeft90UntilLine(50);
  pivotRight90UntilLine(50);
  pivotRight90UntilLine(50);
}

void lineFollowingMode() {
  readSensors();

  // 1) If we see a strong corner on one side and center is not on line, snap pivot
  if (strongLeftFeature() && !centerOnLine()) {
    brake(BRAKE_MS);
    pivotLeftUntilCenter();
    brake(BRAKE_MS);
  } 
  else if (strongRightFeature() && !centerOnLine()) {
    brake(BRAKE_MS);
    pivotRightUntilCenter();
    brake(BRAKE_MS);
  } 
  else {
    // 2) Normal PID tracking (with smooth recovery when line is momentarily lost)
    if (!anyOnLine()) {
      searchWhenLost();
      return;
    }

    error = getLineError();
    integral += error;
    derivative = error - lastError;
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    int leftSpeed  = constrain((int)(baseSpeed + correction),  0, maxPWM);
    int rightSpeed = constrain((int)(baseSpeed - correction), 0, maxPWM);

    setMotorSpeeds(leftSpeed, rightSpeed);
  }

  delay(5);
}
