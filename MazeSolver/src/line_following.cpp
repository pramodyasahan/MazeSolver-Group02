#include <Arduino.h>

// ====================== Motor Pins ======================
#define L_RPWM 4
#define L_LPWM 5
#define L_REN 6
#define L_LEN 7

#define R_RPWM 2
#define R_LPWM 3
#define R_REN 9
#define R_LEN 8

// ====================== Line Sensors ======================
const uint8_t sensorPins[8] = {22, 23, 24, 25, 26, 27, 28, 29};
// After readSensors(): sensorVal[i] == 1 means "black", 0 means "white"
uint8_t sensorVal[8];
uint8_t sensorBits = 0; // bit i = 1 if sensor i sees black

// Set this to false if your board reads LOW on black.
const bool BLACK_IS_HIGH = true;

// Convenience groupings (0 = far-left, 7 = far-right)
const uint8_t CENTER_IDX[2] = {3,4};
const uint8_t LEFT_IDX[3]   = {0,1,2};
const uint8_t RIGHT_IDX[3]  = {5,6,7};

// How strong a “left/right feature” must be to treat as a corner
const uint8_t LEFT_STRONG_MIN  = 2;
const uint8_t RIGHT_STRONG_MIN = 2;

// Pivot behavior at corners
const int TURN_PWM = 140;               // strong pivot power
const uint16_t PIVOT_TIMEOUT_MS = 600;  // safety stop if something goes wrong
const uint16_t BRAKE_MS = 30;

// ====================== PID Parameters ======================
float Kp = 40.0;
float Ki = 0.0;
float Kd = 4.0;

int baseSpeed = 50;   // cruise speed
int maxPWM    = 100;  // clamps for smoothness

// ====================== Variables ======================
float error = 0, lastError = 0;
float integral = 0, derivative = 0, correction = 0;

// ====================== Function Prototypes ======================
void readSensors();
float getLineError();               // returns ~[-3.5..+3.5]; uses lastError if no blacks
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();

bool anyOnLine();
bool centerOnLine();
uint8_t countBlk(const uint8_t idxs[], uint8_t n);
bool strongLeftFeature();
bool strongRightFeature();

void brake(uint16_t ms);
void pivotLeftUntilCenter();
void pivotRightUntilCenter();
void searchWhenLost();

// ====================== Setup ======================
void setup() {
  Serial.begin(9600);

  // --- Motor setup ---
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);

  pinMode(R_REN, OUTPUT);
  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);
  pinMode(L_LEN, OUTPUT);

  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);

  // --- Sensor setup ---
  for (int i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

  Serial.println("PID Line Following Robot (with hard 90° corner handling)");
}

// ====================== Main Loop ======================
void loop() {
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

    int leftSpeed  = constrain((int)(baseSpeed + correction),  0, maxPWM);
    int rightSpeed = constrain((int)(baseSpeed - correction),  0, maxPWM);

    setMotorSpeeds(leftSpeed, rightSpeed);
  }

  // Small loop delay for stability
  delay(5);
}

// ===============================================================
//                  Function Definitions
// ===============================================================

void readSensors() {
  sensorBits = 0;
  for (int i = 0; i < 8; i++) {
    int raw = digitalRead(sensorPins[i]);
    bool isBlack = BLACK_IS_HIGH ? (raw == HIGH) : (raw == LOW);
    sensorVal[i] = isBlack ? 1 : 0;
    if (isBlack) sensorBits |= (1u << i);
  }
}

// Weighted error: outer sensors have larger magnitude so turns are detected earlier
float getLineError() {
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

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Your original wiring: Right forward uses R_RPWM, Left forward uses L_LPWM
  analogWrite(R_RPWM, rightSpeed);
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, leftSpeed);
}

void stopMotors() {
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
}

// ====================== Corner / Recovery helpers ======================
bool anyOnLine() { return sensorBits != 0; }
bool centerOnLine() { return sensorVal[CENTER_IDX[0]] || sensorVal[CENTER_IDX[1]]; }

uint8_t countBlk(const uint8_t idxs[], uint8_t n) {
  uint8_t c = 0;
  for (uint8_t k = 0; k < n; k++) c += sensorVal[idxs[k]] ? 1 : 0;
  return c;
}

// “Strong” side feature = ≥2 blacks on that edge while the opposite edge is mostly white.
// This triggers a decisive pivot rather than letting PID average the corner away.
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

// In-place left pivot until a center sensor re-locks the line (or timeout)
void pivotLeftUntilCenter() {
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
void pivotRightUntilCenter() {
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
void searchWhenLost() {
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
