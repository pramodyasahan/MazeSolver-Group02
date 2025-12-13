#include <Arduino.h>

// =======================================================
// Motor Pins (UNCHANGED)
// =======================================================
#define L_RPWM 4
#define L_LPWM 5
#define L_REN  6
#define L_LEN  7

#define R_RPWM 2
#define R_LPWM 3
#define R_REN  9
#define R_LEN  8

// =======================================================
// Line Sensors (UNCHANGED)
// =======================================================
const uint8_t sensorPins[8] = {22,23,24,25,26,27,28,29};
uint8_t s[8];
uint8_t bits = 0;
const bool BLACK_IS_HIGH = true;

// Sensor groups
const uint8_t LEFT_IDX[3]   = {0,1,2};
const uint8_t CENTER_IDX[2] = {3,4};
const uint8_t RIGHT_IDX[3]  = {5,6,7};

// =======================================================
// PID (TUNABLE)
// =======================================================
float Kp = 40.0;
float Ki = 0.0;
float Kd = 4.0;

int baseSpeed = 55;
int maxPWM    = 130;
int CORR_CLAMP = 70;

// =======================================================
// Turn & search (TUNABLE)
// =======================================================
const int TURN_PWM = 150;
const uint16_t TURN_TIMEOUT_MS = 900;
const uint8_t CENTER_STABLE_N = 4;
const uint16_t TURN_COOLDOWN_MS = 450;
unsigned long lastTurnTime = 0;

const int SEARCH_PWM = 80;
const uint16_t SEARCH_MS = 450;

// =======================================================
// ALL BLACK handling (junction / wide tape)
// =======================================================
const uint8_t ALL_BLACK_MIN = 7;
const int COAST_PWM = 45;
const uint16_t ALL_BLACK_COAST_MS = 250;
unsigned long allBlackStart = 0;

// =======================================================
// PID state
// =======================================================
float error = 0, lastError = 0;
float integral = 0, derivative = 0, corr = 0;

// =======================================================
// Debug timing
// =======================================================
unsigned long lastPidPrint = 0;
const unsigned long PID_PRINT_MS = 120;     // periodic PID log
const unsigned long EVENT_GAP_MS = 40;      // avoid event spam
unsigned long lastEventPrint = 0;

// =======================================================
// Prototypes
// =======================================================
void readSensors();
uint8_t countIdx(const uint8_t* idx, uint8_t n);
uint8_t countAll();
bool anyOnLine();
float lineError();

void setMotor(int left, int right);
void stopMotors();

void pivotLeft();
void pivotRight();
void searchWhenLost();

// Debug helpers
void printBits(uint8_t b);
void debugEvent(const char* tag, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc);
void debugPid(const char* mode, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc, int mL, int mR);

// =======================================================
// Setup
// =======================================================
void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("========================================");
  Serial.println("LineFollower DEBUG BUILD (USB Serial)");
  Serial.println("Logs: EVT=events, PID=periodic, PIV=live pivot");
  Serial.println("========================================");

  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);

  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);

  for (int i=0;i<8;i++) pinMode(sensorPins[i], INPUT);

  stopMotors();
  delay(300);

  Serial.print("CFG Kp="); Serial.print(Kp);
  Serial.print(" Ki="); Serial.print(Ki);
  Serial.print(" Kd="); Serial.print(Kd);
  Serial.print(" base="); Serial.print(baseSpeed);
  Serial.print(" max="); Serial.print(maxPWM);
  Serial.print(" clamp="); Serial.println(CORR_CLAMP);
}

// =======================================================
// Main Loop
// =======================================================
void loop() {
  readSensors();

  uint8_t Lc = countIdx(LEFT_IDX,3);
  uint8_t Cc = countIdx(CENTER_IDX,2);
  uint8_t Rc = countIdx(RIGHT_IDX,3);
  uint8_t Bc = countAll();

  bool inCooldown = (millis() - lastTurnTime < TURN_COOLDOWN_MS);

  // =====================================================
  // 90° CORNER DETECTION (FIXED)
  // Rule: forward lost (center=0) + strong evidence on one side
  // Typical L-turn: center disappears, new line appears on left side.
  // =====================================================
  bool forwardLost = (Cc == 0);
  bool strongLeft  = (Lc >= 2 && Rc == 0);
  bool strongRight = (Rc >= 2 && Lc == 0);
  bool bothSides   = (Lc >= 2 && Rc >= 2);

  bool left90  = (!inCooldown && forwardLost && strongLeft);
  bool right90 = (!inCooldown && forwardLost && strongRight);

  if (left90) {
    debugEvent("TURN_90_LEFT", Lc, Cc, Rc, Bc);
    pivotLeft();
    lastTurnTime = millis();
    integral = 0;
    lastError = -2.5;
    return;
  }

  if (right90) {
    debugEvent("TURN_90_RIGHT", Lc, Cc, Rc, Bc);
    pivotRight();
    lastTurnTime = millis();
    integral = 0;
    lastError = 2.5;
    return;
  }

  if (!inCooldown && forwardLost && bothSides) {
    debugEvent("TURN_BOTH_SIDES", Lc, Cc, Rc, Bc);
    if (Lc > Rc) pivotLeft();
    else if (Rc > Lc) pivotRight();
    else {
      if (lastError >= 0) pivotRight();
      else pivotLeft();
    }
    lastTurnTime = millis();
    integral = 0;
    return;
  }

  // =====================================================
  // ALL BLACK (junction / wide tape)
  // =====================================================
  if (Bc >= ALL_BLACK_MIN) {
    if (allBlackStart == 0) {
      allBlackStart = millis();
      debugEvent("ALL_BLACK_ENTER", Lc, Cc, Rc, Bc);
    }

    setMotor(COAST_PWM, COAST_PWM);
    debugPid("ALL_BLACK_COAST", Lc, Cc, Rc, Bc, COAST_PWM, COAST_PWM);

    if (millis() - allBlackStart > ALL_BLACK_COAST_MS && !inCooldown) {
      debugEvent("ALL_BLACK_DECIDE_TURN", Lc, Cc, Rc, Bc);
      if (lastError >= 0) pivotRight();
      else pivotLeft();
      lastTurnTime = millis();
      allBlackStart = 0;
    }
    return;
  } else {
    if (allBlackStart != 0) debugEvent("ALL_BLACK_EXIT", Lc, Cc, Rc, Bc);
    allBlackStart = 0;
  }

  // =====================================================
  // LOST LINE
  // =====================================================
  if (!anyOnLine()) {
    debugEvent("LOST_LINE", Lc, Cc, Rc, Bc);
    searchWhenLost();
    return;
  }

  // =====================================================
  // PID FOLLOW
  // =====================================================
  error = lineError();
  integral += error;
  derivative = error - lastError;
  corr = (Kp * error) + (Ki * integral) + (Kd * derivative);
  corr = constrain(corr, -CORR_CLAMP, CORR_CLAMP);
  lastError = error;

  int mL = constrain((int)(baseSpeed + corr), 0, maxPWM);
  int mR = constrain((int)(baseSpeed - corr), 0, maxPWM);

  setMotor(mL, mR);
  debugPid("PID", Lc, Cc, Rc, Bc, mL, mR);

  delay(5);
}

// =======================================================
// Sensor Helpers
// =======================================================
void readSensors() {
  bits = 0;
  for (int i=0;i<8;i++) {
    bool black = BLACK_IS_HIGH ? digitalRead(sensorPins[i]) : !digitalRead(sensorPins[i]);
    s[i] = black ? 1 : 0;
    if (black) bits |= (1 << i);
  }
}

uint8_t countIdx(const uint8_t* idx, uint8_t n) {
  uint8_t c=0;
  for(uint8_t i=0;i<n;i++) c += s[idx[i]];
  return c;
}

uint8_t countAll() {
  uint8_t c=0;
  for(int i=0;i<8;i++) c += s[i];
  return c;
}

bool anyOnLine() { return bits != 0; }

float lineError() {
  static const float w[8] = {-3.5,-2.5,-1.5,-0.5,0.5,1.5,2.5,3.5};
  float sw=0, sn=0;
  for(int i=0;i<8;i++) {
    if(s[i]) { sw += w[i]; sn += 1.0; }
  }
  return sn ? (sw/sn) : lastError;
}

// =======================================================
// Motor (UNCHANGED)
// =======================================================
void setMotor(int left, int right) {
  analogWrite(R_RPWM, right);
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, left);
}

void stopMotors() {
  analogWrite(R_RPWM,0);
  analogWrite(R_LPWM,0);
  analogWrite(L_RPWM,0);
  analogWrite(L_LPWM,0);
}

// =======================================================
// Pivot (with live logs)
// =======================================================
void pivotLeft() {
  Serial.println("EVT pivotLeft start");

  unsigned long t0 = millis();
  unsigned long lastDbg = 0;
  uint8_t stable = 0;

  while (millis() - t0 < TURN_TIMEOUT_MS) {
    // Your original pivotLeft behavior (UNCHANGED)
    analogWrite(R_RPWM, TURN_PWM);
    analogWrite(L_RPWM, TURN_PWM);

    readSensors();

    if (s[3] || s[4]) stable++;
    else stable = 0;

    if (millis() - lastDbg > 80) {
      lastDbg = millis();
      Serial.print("PIV_L dt="); Serial.print(millis() - t0);
      Serial.print(" stable="); Serial.print(stable);
      Serial.print(" bits="); printBits(bits);
      Serial.print(" C="); Serial.print((int)(s[3] || s[4]));
      Serial.print(" S="); for(int i=0;i<8;i++) Serial.print(s[i]);
      Serial.println();
    }

    if (stable >= CENTER_STABLE_N) break;
    delay(4);
  }

  stopMotors();
  Serial.print("EVT pivotLeft done dt="); Serial.println(millis() - t0);
}

void pivotRight() {
  Serial.println("EVT pivotRight start");

  unsigned long t0 = millis();
  unsigned long lastDbg = 0;
  uint8_t stable = 0;

  while (millis() - t0 < TURN_TIMEOUT_MS) {
    // Your original pivotRight behavior (UNCHANGED)
    analogWrite(R_LPWM, TURN_PWM);
    analogWrite(L_LPWM, TURN_PWM);

    readSensors();

    if (s[3] || s[4]) stable++;
    else stable = 0;

    if (millis() - lastDbg > 80) {
      lastDbg = millis();
      Serial.print("PIV_R dt="); Serial.print(millis() - t0);
      Serial.print(" stable="); Serial.print(stable);
      Serial.print(" bits="); printBits(bits);
      Serial.print(" C="); Serial.print((int)(s[3] || s[4]));
      Serial.print(" S="); for(int i=0;i<8;i++) Serial.print(s[i]);
      Serial.println();
    }

    if (stable >= CENTER_STABLE_N) break;
    delay(4);
  }

  stopMotors();
  Serial.print("EVT pivotRight done dt="); Serial.println(millis() - t0);
}

// =======================================================
// Lost search (with logs)
// =======================================================
void searchWhenLost() {
  Serial.print("EVT search start dir=");
  Serial.println(lastError >= 0 ? "RIGHT" : "LEFT");

  if (lastError >= 0) {
    analogWrite(R_LPWM, SEARCH_PWM);
    analogWrite(L_LPWM, SEARCH_PWM);
  } else {
    analogWrite(R_RPWM, SEARCH_PWM);
    analogWrite(L_RPWM, SEARCH_PWM);
  }

  unsigned long t0 = millis();
  unsigned long lastDbg = 0;

  while (millis() - t0 < SEARCH_MS) {
    readSensors();
    if (anyOnLine()) {
      stopMotors();
      Serial.print("EVT search reacquired dt="); Serial.println(millis() - t0);
      return;
    }

    if (millis() - lastDbg > 80) {
      lastDbg = millis();
      Serial.print("SRCH dt="); Serial.print(millis() - t0);
      Serial.print(" bits="); printBits(bits);
      Serial.print(" S="); for(int i=0;i<8;i++) Serial.print(s[i]);
      Serial.println();
    }

    delay(4);
  }

  stopMotors();
  Serial.println("EVT search fail stop");
}

// =======================================================
// Debug Printers
// =======================================================
void printBits(uint8_t b) {
  for (int i = 7; i >= 0; i--) Serial.print((b >> i) & 1);
}

void debugEvent(const char* tag, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc) {
  if (millis() - lastEventPrint < EVENT_GAP_MS) return;
  lastEventPrint = millis();

  Serial.print("EVT t="); Serial.print(millis());
  Serial.print(" "); Serial.print(tag);
  Serial.print(" bits="); printBits(bits);
  Serial.print(" (0x"); Serial.print(bits, HEX); Serial.print(")");
  Serial.print(" S="); for(int i=0;i<8;i++) Serial.print(s[i]);
  Serial.print(" L="); Serial.print(Lc);
  Serial.print(" C="); Serial.print(Cc);
  Serial.print(" R="); Serial.print(Rc);
  Serial.print(" B="); Serial.println(Bc);
}

void debugPid(const char* mode, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc, int mL, int mR) {
  if (millis() - lastPidPrint < PID_PRINT_MS) return;
  lastPidPrint = millis();

  Serial.print("PID t="); Serial.print(millis());
  Serial.print(" MODE="); Serial.print(mode);
  Serial.print(" bits="); printBits(bits);
  Serial.print(" S="); for(int i=0;i<8;i++) Serial.print(s[i]);
  Serial.print(" L="); Serial.print(Lc);
  Serial.print(" C="); Serial.print(Cc);
  Serial.print(" R="); Serial.print(Rc);
  Serial.print(" B="); Serial.print(Bc);
  Serial.print(" err="); Serial.print(error,2);
  Serial.print(" corr="); Serial.print(corr,2);
  Serial.print(" mL="); Serial.print(mL);
  Serial.print(" mR="); Serial.println(mR);
}