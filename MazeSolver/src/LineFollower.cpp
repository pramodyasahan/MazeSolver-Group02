// src/LineFollower.cpp
#include <Arduino.h>
#include "RobotConfig.h"
#include "RobotState.h"

// =======================================================
// Line Sensors
// =======================================================
// After readSensors(): sensorVal[i] == 1 means "black", 0 means "white"
static uint8_t sensorVal[8];
static uint8_t sensorBits = 0; // bit i = 1 if sensor i sees black

// Convenience groupings (0 = far-left, 7 = far-right)
static const uint8_t CENTER_IDX[2] = {3,4};
static const uint8_t LEFT_IDX[3]   = {0,1,2};
static const uint8_t RIGHT_IDX[3]  = {5,6,7};

// How strong a “left/right feature” must be to treat as a corner
static const uint8_t LEFT_STRONG_MIN  = 2;
static const uint8_t RIGHT_STRONG_MIN = 2;

// Pivot behavior at corners
static const int      TURN_PWM          = 140;  // strong pivot power
static const uint16_t PIVOT_TIMEOUT_MS  = 600;  // safety stop if something goes wrong
static const uint16_t BRAKE_MS          = 30;

// ====================== PID Parameters ======================
static float Kp = 40.0;
static float Ki = 0.0;
static float Kd = 4.0;

static int baseSpeed = 50;   // cruise speed
static int maxPWM    = 100;  // clamps for smoothness

// ====================== Variables ======================
static float error = 0, lastError = 0;
static float integral = 0, derivative = 0, correction = 0;

// ====================== Function Prototypes ======================
static void readSensors();
static float getLineError();               // returns ~[-3.5..+3.5]; uses lastError if no blacks
static void setMotorSpeeds(int leftSpeed, int rightSpeed);
static void stopMotors();

static bool anyOnLine();
static bool centerOnLine();
static uint8_t countBlk(const uint8_t idxs[], uint8_t n);
static bool strongLeftFeature();
static bool strongRightFeature();

static void brake(uint16_t ms);
static void pivotLeftUntilCenter();
static void pivotRightUntilCenter();
static void searchWhenLost();

// =======================================================
// Public: begin
// =======================================================
void lineFollowerBegin() {
  // IMPORTANT: DBG_PORT/Serial3 is initialized in main.cpp (do NOT begin it here)

  DBG_PORT.println("========================================");
  DBG_PORT.println("LineFollower (integrated) - PID + hard 90deg corner snap");
  DBG_PORT.println("Corner: strong edge + center off-line -> pivot until center re-locks");
  DBG_PORT.println("========================================");

  // Motors (kept consistent with your wiring)
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);

  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);

  // Line sensors
  for (int i = 0; i < 8; i++) pinMode(LINE_SENSOR_PINS[i], INPUT);

  stopMotors();
  delay(250);

  DBG_PORT.print("CFG Kp="); DBG_PORT.print(Kp);
  DBG_PORT.print(" Ki="); DBG_PORT.print(Ki);
  DBG_PORT.print(" Kd="); DBG_PORT.print(Kd);
  DBG_PORT.print(" base="); DBG_PORT.print(baseSpeed);
  DBG_PORT.print(" maxPWM="); DBG_PORT.println(maxPWM);

  // Reset runtime state
  sensorBits = 0;
  for (int i = 0; i < 8; i++) sensorVal[i] = 0;

  error = 0;
  lastError = 0;
  integral = 0;
  derivative = 0;
  correction = 0;
}

// =======================================================
// Public: update
// =======================================================
void lineFollowerUpdate(bool /*isMappingPhase*/) {
  readSensors();

  // 1) If we see a strong corner on one side and center is not on line, snap pivot
  if (strongLeftFeature() && !centerOnLine()) {
    brake(BRAKE_MS);
    pivotLeftUntilCenter();
    brake(BRAKE_MS);
  } else if (strongRightFeature() && !centerOnLine()) {
    brake(BRAKE_MS);
    pivotRightUntilCenter();
    brake(BRAKE_MS);
  } else {
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

    int leftSpeed  = constrain((int)(baseSpeed + correction), 0, maxPWM);
    int rightSpeed = constrain((int)(baseSpeed - correction), 0, maxPWM);

    setMotorSpeeds(leftSpeed, rightSpeed);
  }

  // Small loop delay for stability (preserved)
  delay(5);
}

// ===============================================================
//                  Function Definitions
// ===============================================================
static void readSensors() {
  sensorBits = 0;
  for (int i = 0; i < 8; i++) {
    int raw = digitalRead(LINE_SENSOR_PINS[i]);
    bool isBlack = BLACK_IS_HIGH ? (raw == HIGH) : (raw == LOW);
    sensorVal[i] = isBlack ? 1 : 0;
    if (isBlack) sensorBits |= (1u << i);
  }
}

// Weighted error: outer sensors have larger magnitude so turns are detected earlier
static float getLineError() {
  static const float weight[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
  float sumW = 0.0f, sum = 0.0f;

  for (int i = 0; i < 8; i++) {
    if (sensorVal[i]) { sumW += weight[i]; sum += 1.0f; }
  }

  if (sum == 0.0f) {
    // No line currently under any sensor → keep prior heading
    return lastError;
  }

  return sumW / sum; // normalized error roughly in [-3.5, +3.5]
}

static void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Your wiring: Right forward uses R_RPWM, Left forward uses L_LPWM
  analogWrite(R_RPWM, rightSpeed);
  analogWrite(R_LPWM, 0);

  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, leftSpeed);
}

static void stopMotors() {
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
}

// ====================== Corner / Recovery helpers ======================
static bool anyOnLine() { return sensorBits != 0; }
static bool centerOnLine() { return sensorVal[CENTER_IDX[0]] || sensorVal[CENTER_IDX[1]]; }

static uint8_t countBlk(const uint8_t idxs[], uint8_t n) {
  uint8_t c = 0;
  for (uint8_t k = 0; k < n; k++) c += sensorVal[idxs[k]] ? 1 : 0;
  return c;
}

// “Strong” side feature = ≥2 blacks on that edge while the opposite edge is mostly white.
static bool strongLeftFeature() {
  uint8_t L = countBlk(LEFT_IDX, 3);
  uint8_t R = countBlk(RIGHT_IDX, 3);
  return (L >= LEFT_STRONG_MIN) && (R <= 1);
}
static bool strongRightFeature() {
  uint8_t L = countBlk(LEFT_IDX, 3);
  uint8_t R = countBlk(RIGHT_IDX, 3);
  return (R >= RIGHT_STRONG_MIN) && (L <= 1);
}

static void brake(uint16_t ms) { stopMotors(); delay(ms); }

// In-place left pivot until a center sensor re-locks the line (or timeout)
static void pivotLeftUntilCenter() {
  unsigned long t0 = millis();
  while (millis() - t0 < PIVOT_TIMEOUT_MS) {
    // Right forward, Left backward
    analogWrite(R_RPWM, TURN_PWM); analogWrite(R_LPWM, 0);
    analogWrite(L_RPWM, TURN_PWM); analogWrite(L_LPWM, 0);

    readSensors();
    if (centerOnLine()) break;
    delay(4);
  }
}

// In-place right pivot until center sensors see the line
static void pivotRightUntilCenter() {
  unsigned long t0 = millis();
  while (millis() - t0 < PIVOT_TIMEOUT_MS) {
    // Right backward, Left forward
    analogWrite(R_RPWM, 0);        analogWrite(R_LPWM, TURN_PWM);
    analogWrite(L_RPWM, 0);        analogWrite(L_LPWM, TURN_PWM);

    readSensors();
    if (centerOnLine()) break;
    delay(4);
  }
}

// When no sensors see the line, rotate toward the last error direction to re-acquire quickly
static void searchWhenLost() {
  if (lastError >= 0) {
    // rotate right gently
    analogWrite(R_RPWM, 0);        analogWrite(R_LPWM, baseSpeed);
    analogWrite(L_RPWM, 0);        analogWrite(L_LPWM, baseSpeed);
  } else {
    // rotate left gently
    analogWrite(R_RPWM, baseSpeed); analogWrite(R_LPWM, 0);
    analogWrite(L_RPWM, baseSpeed); analogWrite(L_LPWM, 0);
  }

  // brief search window, rechecking sensors
  unsigned long t0 = millis();
  while (millis() - t0 < 150) {
    readSensors();
    if (anyOnLine()) break;
    delay(4);
  }
}
