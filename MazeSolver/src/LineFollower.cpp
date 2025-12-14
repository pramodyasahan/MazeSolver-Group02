// src/LineFollower.cpp
#include <Arduino.h>
#include "RobotConfig.h"
#include "RobotState.h"

// =======================================================
// Line Sensors
// =======================================================
static uint8_t s[8];
static uint8_t bits = 0;

// Sensor groups
static const uint8_t LEFT_IDX[3]   = {0,1,2};
static const uint8_t CENTER_IDX[2] = {3,4};
static const uint8_t RIGHT_IDX[3]  = {5,6,7};

// =======================================================
// PID (TUNABLE)
// =======================================================
static float Kp = 45.0;
static float Ki = 0.0;
static float Kd = 4.0;

static int baseSpeed  = 60;
static int maxPWM     = 130;
static int CORR_CLAMP = 70;

// =======================================================
// Wide-curve handling (TUNABLE)
// =======================================================
static int WIDE_BASE  = 40; // TODO tune: 35-55
static int WIDE_CLAMP = 55; // TODO tune: 40-70

// =======================================================
// Turn & search (TUNABLE)
// =======================================================
static const int      TURN_PWM          = 150;
static const uint16_t TURN_TIMEOUT_MS   = 900;
static const uint8_t  CENTER_STABLE_N   = 4;
static const uint16_t TURN_COOLDOWN_MS  = 450;
static unsigned long  lastTurnTime      = 0;

static const int      SEARCH_PWM = 80;
static const uint16_t SEARCH_MS  = 450;

// =======================================================
// ALL BLACK handling (junction / wide tape)
// =======================================================
static const uint8_t  ALL_BLACK_MIN       = 6;
static const int      COAST_PWM           = 45;
static const uint16_t ALL_BLACK_COAST_MS  = 250;
static unsigned long  allBlackStart       = 0;

// =======================================================
// PID state
// =======================================================
static float error = 0, lastError = 0;
static float integral = 0, derivative = 0, corr = 0;

// =======================================================
// Debug timing
// =======================================================
static unsigned long lastPidPrint = 0;
static const unsigned long PID_PRINT_MS = 120;
static const unsigned long EVENT_GAP_MS = 40;
static unsigned long lastEventPrint = 0;

// =======================================================
// Prototypes
// =======================================================
static void readSensors();
static uint8_t countIdx(const uint8_t* idx, uint8_t n);
static uint8_t countAll();
static bool anyOnLine();
static float lineError();

static void setMotor(int left, int right);
static void stopMotors();

static void pivotLeft();
static void searchWhenLost();

static void printBits(uint8_t b);
static void debugEvent(const char* tag, const char* reason, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc);
static void debugPid(const char* mode, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc, int mL, int mR);

// =======================================================
// Public: begin
// =======================================================
void lineFollowerBegin() {
  // IMPORTANT: Serial3 is initialized in main.cpp (do NOT begin it here)

  DBG_PORT.println("========================================");
  DBG_PORT.println("LineFollower DEBUG BUILD (Serial3)");
  DBG_PORT.println("EVT=events  PID=periodic  PIV=live pivot  SRCH=lost search");
  DBG_PORT.println("LEFT-ONLY 90deg turns enabled");
  DBG_PORT.println("========================================");

  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);

  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);

  for (int i=0;i<8;i++) pinMode(LINE_SENSOR_PINS[i], INPUT);

  stopMotors();
  delay(250);

  DBG_PORT.print("CFG Kp="); DBG_PORT.print(Kp);
  DBG_PORT.print(" Ki="); DBG_PORT.print(Ki);
  DBG_PORT.print(" Kd="); DBG_PORT.print(Kd);
  DBG_PORT.print(" base="); DBG_PORT.print(baseSpeed);
  DBG_PORT.print(" max="); DBG_PORT.print(maxPWM);
  DBG_PORT.print(" clamp="); DBG_PORT.print(CORR_CLAMP);
  DBG_PORT.print(" wideBase="); DBG_PORT.print(WIDE_BASE);
  DBG_PORT.print(" wideClamp="); DBG_PORT.println(WIDE_CLAMP);

  // Reset runtime state (important when switching states)
  bits = 0;
  for (int i=0;i<8;i++) s[i] = 0;

  error = 0;
  lastError = 0;
  integral = 0;
  derivative = 0;
  corr = 0;

  lastTurnTime = 0;
  allBlackStart = 0;
  lastPidPrint = 0;
  lastEventPrint = 0;
}

// =======================================================
// Public: update
// =======================================================
void lineFollowerUpdate(bool /*isMappingPhase*/) {
  readSensors();

  uint8_t Lc = countIdx(LEFT_IDX,3);
  uint8_t Cc = countIdx(CENTER_IDX,2);
  uint8_t Rc = countIdx(RIGHT_IDX,3);
  uint8_t Bc = countAll();

  bool inCooldown = (millis() - lastTurnTime < TURN_COOLDOWN_MS);

  // LEFT 90° CORNER DETECTION (LEFT-ONLY MAP)
  bool forwardLost = (Cc == 0);
  bool strongLeft  = (Lc >= 2);
  bool rightClear  = (Rc == 0);
  bool left90 = (!inCooldown && forwardLost && strongLeft && rightClear);

  if (left90) {
    debugEvent("TURN_90_LEFT", "Cc==0 && Lc>=2 && Rc==0", Lc, Cc, Rc, Bc);
    pivotLeft();
    lastTurnTime = millis();
    integral = 0;
    lastError = -2.5;
    return;
  }

  // ALL BLACK / WIDE LINE HANDLING
  bool centerOn = (Cc > 0);

  if (Bc >= ALL_BLACK_MIN) {
    // CASE A: WIDE CURVE (center still on line) -> no pivot
    if (centerOn) {
      float e = lineError();
      float d = e - lastError;
      float c = (Kp * e) + (Kd * d);
      c = constrain(c, -WIDE_CLAMP, WIDE_CLAMP);
      lastError = e;

      int mL = constrain((int)(WIDE_BASE + c), 0, maxPWM);
      int mR = constrain((int)(WIDE_BASE - c), 0, maxPWM);

      setMotor(mL, mR);
      debugEvent("WIDE_LINE", "Bc high but centerOn -> slow PID", Lc, Cc, Rc, Bc);
      debugPid("WIDE_LINE_PID", Lc, Cc, Rc, Bc, mL, mR);
      return;
    }

    // CASE B: TRUE JUNCTION (center lost) -> coast then LEFT decide
    if (allBlackStart == 0) {
      allBlackStart = millis();
      debugEvent("ALL_BLACK_ENTER", "Bc>=min && Cc==0 (junction)", Lc, Cc, Rc, Bc);
    }

    setMotor(COAST_PWM, COAST_PWM);
    debugPid("ALL_BLACK_COAST", Lc, Cc, Rc, Bc, COAST_PWM, COAST_PWM);

    if ((millis() - allBlackStart > ALL_BLACK_COAST_MS) && !inCooldown) {
      debugEvent("ALL_BLACK_LEFT_DECIDE", "junction -> pivotLeft()", Lc, Cc, Rc, Bc);
      pivotLeft();
      lastTurnTime = millis();
      allBlackStart = 0;
    }
    return;

  } else {
    if (allBlackStart != 0) debugEvent("ALL_BLACK_EXIT", "Bc dropped", Lc, Cc, Rc, Bc);
    allBlackStart = 0;
  }

  // LOST LINE
  if (!anyOnLine()) {
    debugEvent("LOST_LINE", "bits==0 -> search", Lc, Cc, Rc, Bc);
    searchWhenLost();
    return;
  }

  // PID FOLLOW
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

  delay(5); // preserved
}

// =======================================================
// Sensor Helpers
// =======================================================
static void readSensors() {
  bits = 0;
  for (int i=0;i<8;i++) {
    bool raw = digitalRead(LINE_SENSOR_PINS[i]);
    bool black = BLACK_IS_HIGH ? raw : !raw;
    s[i] = black ? 1 : 0;
    if (black) bits |= (1u << i);
  }
}

static uint8_t countIdx(const uint8_t* idx, uint8_t n) {
  uint8_t c=0;
  for(uint8_t i=0;i<n;i++) c += s[idx[i]];
  return c;
}

static uint8_t countAll() {
  uint8_t c=0;
  for(int i=0;i<8;i++) c += s[i];
  return c;
}

static bool anyOnLine() { return bits != 0; }

static float lineError() {
  static const float w[8] = {-3.5,-2.5,-1.5,-0.5,0.5,1.5,2.5,3.5};
  float sw=0, sn=0;
  for(int i=0;i<8;i++) {
    if(s[i]) { sw += w[i]; sn += 1.0; }
  }
  return sn ? (sw/sn) : lastError;
}

// =======================================================
// Motor (mapping preserved)
// =======================================================
static void setMotor(int left, int right) {
  // Right forward
  analogWrite(R_RPWM, right);
  analogWrite(R_LPWM, 0);

  // Left forward
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, left);
}

static void stopMotors() {
  analogWrite(R_RPWM,0);
  analogWrite(R_LPWM,0);
  analogWrite(L_RPWM,0);
  analogWrite(L_LPWM,0);
}

// =======================================================
// Pivot LEFT (fixed pivot)
// Right forward + Left backward
// =======================================================
static void pivotLeft() {
  DBG_PORT.println("EVT pivotLeft start (R fwd, L back)");

  unsigned long t0 = millis();
  unsigned long lastDbg = 0;
  uint8_t stable = 0;

  while (millis() - t0 < TURN_TIMEOUT_MS) {
    // Right forward
    analogWrite(R_RPWM, TURN_PWM);
    analogWrite(R_LPWM, 0);

    // Left backward
    analogWrite(L_RPWM, TURN_PWM);
    analogWrite(L_LPWM, 0);

    readSensors();

    if (s[3] || s[4]) stable++;
    else stable = 0;

    if (millis() - lastDbg > 80) {
      lastDbg = millis();
      DBG_PORT.print("PIV_L dt="); DBG_PORT.print(millis() - t0);
      DBG_PORT.print(" stable="); DBG_PORT.print(stable);
      DBG_PORT.print(" bits="); printBits(bits);
      DBG_PORT.print(" C="); DBG_PORT.print((int)(s[3] || s[4]));
      DBG_PORT.print(" S="); for(int i=0;i<8;i++) DBG_PORT.print(s[i]);
      DBG_PORT.println();
    }

    if (stable >= CENTER_STABLE_N) break;
    delay(4);
  }

  stopMotors();
  DBG_PORT.print("EVT pivotLeft done dt="); DBG_PORT.println(millis() - t0);
}

// =======================================================
// Lost search
// =======================================================
static void searchWhenLost() {
  const bool searchRight = (lastError >= 0);

  DBG_PORT.print("EVT search start dir=");
  DBG_PORT.println(searchRight ? "RIGHT" : "LEFT");

  unsigned long t0 = millis();
  unsigned long lastDbg = 0;

  while (millis() - t0 < SEARCH_MS) {
    if (searchRight) {
      // rotate right: right back + left fwd
      analogWrite(R_RPWM, 0);
      analogWrite(R_LPWM, SEARCH_PWM);
      analogWrite(L_RPWM, 0);
      analogWrite(L_LPWM, SEARCH_PWM);
    } else {
      // rotate left: right fwd + left back
      analogWrite(R_RPWM, SEARCH_PWM);
      analogWrite(R_LPWM, 0);
      analogWrite(L_RPWM, SEARCH_PWM);
      analogWrite(L_LPWM, 0);
    }

    readSensors();
    if (anyOnLine()) {
      stopMotors();
      DBG_PORT.print("EVT search reacquired dt="); DBG_PORT.println(millis() - t0);
      return;
    }

    if (millis() - lastDbg > 80) {
      lastDbg = millis();
      DBG_PORT.print("SRCH dt="); DBG_PORT.print(millis() - t0);
      DBG_PORT.print(" bits="); printBits(bits);
      DBG_PORT.print(" S="); for(int i=0;i<8;i++) DBG_PORT.print(s[i]);
      DBG_PORT.println();
    }

    delay(4);
  }

  stopMotors();
  DBG_PORT.println("EVT search fail stop");
}

// =======================================================
// Debug Printers (Serial3)
// =======================================================
static void printBits(uint8_t b) {
  for (int i = 7; i >= 0; i--) DBG_PORT.print((b >> i) & 1);
}

static void debugEvent(const char* tag, const char* reason, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc) {
  if (millis() - lastEventPrint < EVENT_GAP_MS) return;
  lastEventPrint = millis();

  DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
  DBG_PORT.print(" "); DBG_PORT.print(tag);
  DBG_PORT.print(" why="); DBG_PORT.print(reason);
  DBG_PORT.print(" bits="); printBits(bits);
  DBG_PORT.print(" (0x"); DBG_PORT.print(bits, HEX); DBG_PORT.print(")");
  DBG_PORT.print(" S="); for(int i=0;i<8;i++) DBG_PORT.print(s[i]);
  DBG_PORT.print(" L="); DBG_PORT.print(Lc);
  DBG_PORT.print(" C="); DBG_PORT.print(Cc);
  DBG_PORT.print(" R="); DBG_PORT.print(Rc);
  DBG_PORT.print(" B="); DBG_PORT.println(Bc);
}

static void debugPid(const char* mode, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc, int mL, int mR) {
  if (millis() - lastPidPrint < PID_PRINT_MS) return;
  lastPidPrint = millis();

  DBG_PORT.print("PID t="); DBG_PORT.print(millis());
  DBG_PORT.print(" MODE="); DBG_PORT.print(mode);
  DBG_PORT.print(" bits="); printBits(bits);
  DBG_PORT.print(" S="); for(int i=0;i<8;i++) DBG_PORT.print(s[i]);
  DBG_PORT.print(" L="); DBG_PORT.print(Lc);
  DBG_PORT.print(" C="); DBG_PORT.print(Cc);
  DBG_PORT.print(" R="); DBG_PORT.print(Rc);
  DBG_PORT.print(" B="); DBG_PORT.print(Bc);
  DBG_PORT.print(" err="); DBG_PORT.print(error,2);
  DBG_PORT.print(" corr="); DBG_PORT.print(corr,2);
  DBG_PORT.print(" mL="); DBG_PORT.print(mL);
  DBG_PORT.print(" mR="); DBG_PORT.println(mR);
}
