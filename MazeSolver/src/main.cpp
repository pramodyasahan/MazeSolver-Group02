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
float Kp = 30.0;
float Ki = 0.0;
float Kd = 4.0;

int baseSpeed = 62;
int maxPWM    = 130;
int CORR_CLAMP = 70;

// =======================================================
// Wide-curve handling (TUNABLE)
// If many sensors see black but center still sees line,
// treat as WIDE_LINE and slow down (no pivot).
// =======================================================
int WIDE_BASE = 40;          // TODO tune: 35-55
int WIDE_CLAMP = 55;         // TODO tune: 40-70

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
// IMPORTANT: with the "centerOn" gate below, this can stay 7.
// If you still get false triggers on curves, try 8.
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
void searchWhenLost();

// Debug helpers
void printBits(uint8_t b);
void debugEvent(const char* tag, const char* reason, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc);
void debugPid(const char* mode, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc, int mL, int mR);

// =======================================================
// Setup
// =======================================================
void setup() {
  Serial3.begin(9600);
  delay(150);

  Serial3.println("========================================");
  Serial3.println("LineFollower DEBUG BUILD (Serial3)");
  Serial3.println("EVT=events  PID=periodic  PIV=live pivot  SRCH=lost search");
  Serial3.println("LEFT-ONLY 90deg turns enabled");
  Serial3.println("========================================");

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
  delay(250);

  Serial3.print("CFG Kp="); Serial3.print(Kp);
  Serial3.print(" Ki="); Serial3.print(Ki);
  Serial3.print(" Kd="); Serial3.print(Kd);
  Serial3.print(" base="); Serial3.print(baseSpeed);
  Serial3.print(" max="); Serial3.print(maxPWM);
  Serial3.print(" clamp="); Serial3.print(CORR_CLAMP);
  Serial3.print(" wideBase="); Serial3.print(WIDE_BASE);
  Serial3.print(" wideClamp="); Serial3.println(WIDE_CLAMP);
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
  // LEFT 90° CORNER DETECTION (LEFT-ONLY MAP)
  // Trigger when center is lost and left is strong & right is clear.
  // =====================================================
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

  // =====================================================
  // ALL BLACK / WIDE LINE HANDLING
  // Key fix: only treat ALL_BLACK as junction if CENTER is LOST.
  // If center still sees line -> it's a wide curve/wide tape: slow PID.
  // =====================================================
  bool centerOn = (Cc > 0);

  if (Bc >= ALL_BLACK_MIN) {
    // CASE A: WIDE CURVE (center still on line) -> no pivot
    if (centerOn) {
      float e = lineError();
      float d = e - lastError;
      float c = (Kp * e) + (Kd * d);      // keep it responsive, no integral here
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

  // =====================================================
  // LOST LINE
  // =====================================================
  if (!anyOnLine()) {
    debugEvent("LOST_LINE", "bits==0 -> search", Lc, Cc, Rc, Bc);
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
    bool raw = digitalRead(sensorPins[i]);
    bool black = BLACK_IS_HIGH ? raw : !raw;
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
// Motor (mapping preserved)
// =======================================================
void setMotor(int left, int right) {
  // Right forward
  analogWrite(R_RPWM, right);
  analogWrite(R_LPWM, 0);

  // Left forward
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
// Pivot LEFT (fixed pivot)
// Right forward + Left backward
// =======================================================
void pivotLeft() {
  Serial3.println("EVT pivotLeft start (R fwd, L back)");

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
      Serial3.print("PIV_L dt="); Serial3.print(millis() - t0);
      Serial3.print(" stable="); Serial3.print(stable);
      Serial3.print(" bits="); printBits(bits);
      Serial3.print(" C="); Serial3.print((int)(s[3] || s[4]));
      Serial3.print(" S="); for(int i=0;i<8;i++) Serial3.print(s[i]);
      Serial3.println();
    }

    if (stable >= CENTER_STABLE_N) break;
    delay(4);
  }

  stopMotors();
  Serial3.print("EVT pivotLeft done dt="); Serial3.println(millis() - t0);
}

// =======================================================
// Lost search (kept as before)
// =======================================================
void searchWhenLost() {
  const bool searchRight = (lastError >= 0);

  Serial3.print("EVT search start dir=");
  Serial3.println(searchRight ? "RIGHT" : "LEFT");

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
      Serial3.print("EVT search reacquired dt="); Serial3.println(millis() - t0);
      return;
    }

    if (millis() - lastDbg > 80) {
      lastDbg = millis();
      Serial3.print("SRCH dt="); Serial3.print(millis() - t0);
      Serial3.print(" bits="); printBits(bits);
      Serial3.print(" S="); for(int i=0;i<8;i++) Serial3.print(s[i]);
      Serial3.println();
    }

    delay(4);
  }

  stopMotors();
  Serial3.println("EVT search fail stop");
}

// =======================================================
// Debug Printers (Serial3)
// =======================================================
void printBits(uint8_t b) {
  for (int i = 7; i >= 0; i--) Serial3.print((b >> i) & 1);
}

void debugEvent(const char* tag, const char* reason, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc) {
  if (millis() - lastEventPrint < EVENT_GAP_MS) return;
  lastEventPrint = millis();

  Serial3.print("EVT t="); Serial3.print(millis());
  Serial3.print(" "); Serial3.print(tag);
  Serial3.print(" why="); Serial3.print(reason);
  Serial3.print(" bits="); printBits(bits);
  Serial3.print(" (0x"); Serial3.print(bits, HEX); Serial3.print(")");
  Serial3.print(" S="); for(int i=0;i<8;i++) Serial3.print(s[i]);
  Serial3.print(" L="); Serial3.print(Lc);
  Serial3.print(" C="); Serial3.print(Cc);
  Serial3.print(" R="); Serial3.print(Rc);
  Serial3.print(" B="); Serial3.println(Bc);
}

void debugPid(const char* mode, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc, int mL, int mR) {
  if (millis() - lastPidPrint < PID_PRINT_MS) return;
  lastPidPrint = millis();

  Serial3.print("PID t="); Serial3.print(millis());
  Serial3.print(" MODE="); Serial3.print(mode);
  Serial3.print(" bits="); printBits(bits);
  Serial3.print(" S="); for(int i=0;i<8;i++) Serial3.print(s[i]);
  Serial3.print(" L="); Serial3.print(Lc);
  Serial3.print(" C="); Serial3.print(Cc);
  Serial3.print(" R="); Serial3.print(Rc);
  Serial3.print(" B="); Serial3.print(Bc);
  Serial3.print(" err="); Serial3.print(error,2);
  Serial3.print(" corr="); Serial3.print(corr,2);
  Serial3.print(" mL="); Serial3.print(mL);
  Serial3.print(" mR="); Serial3.println(mR);
}
