// LineFollow.cpp
#include "LineFollow.h"

// sensor readings
static int sensorVal[8];
static int sensorBits[8];   // 1 = line, 0 = background

// PID state
static float lineError = 0;
static float lastError = 0;
static float integral  = 0;

// flags
static bool BLACK_IS_HIGH = true;  // set false if your board is opposite
static bool lineEndFlag   = false;

static void readSensors();
static float getLineError();
static bool anyOnLine();
static bool strongLeftFeature();
static bool strongRightFeature();
static bool detectLineWhiteBoxInternal();

// ============================================================================

void initLineFollow() {
  for (int i = 0; i < 8; i++) {
    pinMode(LINE_SENSOR_PINS[i], INPUT);
  }
  integral = 0;
  lastError = 0;
  lineEndFlag = false;
}

bool lineReachedEnd() {
  if (lineEndFlag) {
    lineEndFlag = false;
    return true;
  }
  return false;
}

// ============================================================================
// Core update
// ============================================================================
void lineFollowUpdate() {
  // Detect large white box at end of line
  if (detectLineWhiteBoxInternal()) {
    stopMotors();
    lineEndFlag = true;
    return;
  }

  readSensors();

  if (!anyOnLine()) {
    // Lost line: simple search – pivot slowly until we find it again
    // You can replace with more advanced search later
    pivotLeft(40);
    delay(50);
    stopMotors();
    return;
  }

  float error = getLineError();

  // PID
  float P = error;
  integral += error;
  float D = error - lastError;
  lastError = error;

  float correction = Kp_Line * P + Ki_Line * integral + Kd_Line * D;
  correction = constrain(correction, -MAX_PWM_LINE, MAX_PWM_LINE);

  int pwmR = BASE_SPEED_LINE + (int)correction;
  int pwmL = BASE_SPEED_LINE - (int)correction;

  pwmR = constrain(pwmR, 0, 255);
  pwmL = constrain(pwmL, 0, 255);

  moveForward(pwmR, pwmL);
}

// ============================================================================
// Sensor helpers
// ============================================================================
static void readSensors() {
  for (int i = 0; i < 8; i++) {
    sensorVal[i] = digitalRead(LINE_SENSOR_PINS[i]);
    bool onLine;
    if (BLACK_IS_HIGH) {
      onLine = (sensorVal[i] == HIGH);
    } else {
      onLine = (sensorVal[i] == LOW);
    }
    sensorBits[i] = onLine ? 1 : 0;
  }
}

// Weighted error: negative = line to left, positive = line to right
static float getLineError() {
  // Simple weighting: positions -3, -2, -1, 0, 0, +1, +2, +3
  int weights[8] = {-3, -2, -1, 0, 0, 1, 2, 3};

  int sum = 0;
  int count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorBits[i]) {
      sum += weights[i];
      count++;
    }
  }
  if (count == 0) return 0;  // shouldn't happen if anyOnLine() is true

  return (float)sum / (float)count;
}

static bool anyOnLine() {
  for (int i = 0; i < 8; i++) {
    if (sensorBits[i]) return true;
  }
  return false;
}

// These can be customized to detect sharp left/right markers, if needed
static bool strongLeftFeature() {
  // e.g., left-most few sensors see line strongly
  return (sensorBits[0] || sensorBits[1]) && !sensorBits[6] && !sensorBits[7];
}

static bool strongRightFeature() {
  return (sensorBits[7] || sensorBits[6]) && !sensorBits[0] && !sensorBits[1];
}

static bool detectLineWhiteBoxInternal() {
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) {
    int val = digitalRead(LINE_SENSOR_PINS[i]);
    // assuming LOW = white
    if (val == LOW) whiteCount++;
  }
  return (whiteCount >= WHITE_COUNT_THRESHOLD);
}
