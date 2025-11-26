#include <Arduino.h>
#include <EEPROM.h>

// ==================== Bluetooth Configuration ====================
// Using Serial3 (Pin 14 TX, Pin 15 RX) on Arduino Mega
#define BTSerial Serial3 

// ==================== Pin Definitions ====================
#define TRIG_FRONT 48
#define ECHO_FRONT 49
#define TRIG_RIGHT 50
#define ECHO_RIGHT 51
#define TRIG_LEFT  52
#define ECHO_LEFT  53

#define L_RPWM 4
#define L_LPWM 5
#define L_REN  6
#define L_LEN  7

#define R_RPWM 2
#define R_LPWM 3
#define R_REN  9
#define R_LEN  8

#define L_ENC_A 18
#define L_ENC_B 19
#define R_ENC_A 20
#define R_ENC_B 21

// ==================== Robot Physical Constants ====================
const float WHEEL_DIAMETER_CM = 6.5f;
const float WHEEL_BASE_CM     = 17.0f;
const int pulsesPerMotorRev = 11;
const int gearRatio         = 20;
const int countsPerRev      = pulsesPerMotorRev * gearRatio * 2;
// Distance per tick in cm
const float CM_PER_TICK = (PI * WHEEL_DIAMETER_CM) / countsPerRev;

// ==================== Turning Configuration ====================
const int   TURN_PWM       = 50;
const float TURN_RADIUS_CM = 10.0f;
const int   PIVOT_180_PWM  = 55;

// ==================== Navigation Thresholds ====================
const int OBSTACLE_THRESHOLD      = 12;
const int OPEN_SPACE_THRESHOLD_CM = 1000;
const int DEAD_END_THRESHOLD      = 12;
const int NO_WALL_THRESHOLD       = 25;
const int CORNER_CLEARANCE_CM     = 6;

// ==================== Wall Following ====================
const int   BASE_PWM_STRAIGHT = 60;
const float Kp_Wall                = 4.0f;
const int   MAX_CORRECTION    = 20;
const int   WALL_DETECT_RANGE = 20;

// ==================== Alignment ====================
const int ALIGN_PWM         = 45;
const int ALIGN_DURATION_MS = 50;
const int ALIGN_TOLERANCE_CM= 1;

// ==================== Line Sensors ====================
const uint8_t sensorPins[8] = {22, 23, 24, 25, 26, 27, 28, 29};
uint8_t sensorVal[8];
uint8_t sensorBits = 0; 
const bool BLACK_IS_HIGH = true;

const uint8_t CENTER_IDX[2] = {3, 4};
const uint8_t LEFT_IDX[3]   = {0, 1, 2};
const uint8_t RIGHT_IDX[3]  = {5, 6, 7};

const uint8_t LEFT_STRONG_MIN  = 2;
const uint8_t RIGHT_STRONG_MIN = 2;
const int TURN_PWM_LINE = 140; 
const uint16_t PIVOT_TIMEOUT_MS = 600;
const uint16_t BRAKE_MS = 30;

// ==================== PID Parameters ====================
float Kp = 50.0;
float Ki = 0.0;
float Kd = 4.0;
int baseSpeed = 50; 
int maxPWM    = 100;

// ==================== Global Variables ====================
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

float error = 0, lastError = 0;
float integral = 0, derivative = 0, correction = 0;
bool wallMode = true;

// Odometry Variables
double robotX = 0.0;
double robotY = 0.0;
double robotTheta = 0.0; 
long lastEncL = 0;
long lastEncR = 0;
int eepromAddr = 0;
const int SAVE_INTERVAL_CM = 10;
double lastSavedX = 0.0;
double lastSavedY = 0.0;

// ==================== Function Prototypes ====================
// (This tells the compiler these functions exist later in the file)
void updateOdometry();
void stopMotors();
void stopRight();
void stopLeft();
long absl(long x);
long readUltrasonic(int trigPin, int echoPin);
void countEncoderL();
void countEncoderR();
void smoothTurnLeft();
void smoothTurnRight();
void moveForward(int pwmValR, int pwmValL);
void moveBackward(int pwmVal);
void pivotLeft(int pwmVal);
void pivotRight(int pwmVal);
void pivot180(int pwmVal);
void moveForwardDistance(int distance_cm, int pwmVal);
void wallFollowingMode();
void readSensors();
float getLineError();               
void setMotorSpeeds(int leftSpeed, int rightSpeed);
bool anyOnLine();
bool centerOnLine();
uint8_t countBlk(const uint8_t idxs[], uint8_t n);
bool strongLeftFeature();
bool strongRightFeature();
void brake(uint16_t ms);
void pivotLeftUntilCenter();
void pivotRightUntilCenter();
void pivotLeft90UntilLine(int pwmVal);
void pivotRight90UntilLine(int pwmVal);
void searchWhenLost();
void lineFollowingMode();

// ==================== Inline Helpers ====================
inline void stopRight(){ analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0); }
inline void stopLeft() { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0); }
inline long absl(long x){ return (x < 0) ? -x : x; }

// ==================== Setup ====================
void setup() {
  BTSerial.begin(9600); 
  
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);

  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), countEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), countEncoderR, CHANGE);

  BTSerial.println("Robot Initialized... Recording Path.");
  
  // Save start point
  int startX = 0; int startY = 0;
  EEPROM.put(eepromAddr, startX); eepromAddr += sizeof(int);
  EEPROM.put(eepromAddr, startY); eepromAddr += sizeof(int);
}

// ==================== Main Loop ====================
void loop() {
    updateOdometry();

    if (wallMode) {
        wallFollowingMode();

        // Check for line
        int whiteCount = 0;
        for (int i = 0; i < 8; i++) {
            if (digitalRead(sensorPins[i]) == LOW) whiteCount++;
        }

        if (whiteCount >= 7) { 
            BTSerial.println("All white detected! Switching to LINE FOLLOWING...");
            stopMotors();
            delay(1000);
            moveForward(BASE_PWM_STRAIGHT, BASE_PWM_STRAIGHT);
            delay(1000);
            wallMode = false;
        }
    } else {
        lineFollowingMode();
    }
}

// ==================== Odometry Logic ====================
void updateOdometry() {
  noInterrupts();
  long curL = encoderCountL;
  long curR = encoderCountR;
  interrupts();

  long dL_ticks = curL - lastEncL;
  long dR_ticks = curR - lastEncR;

  if (dL_ticks == 0 && dR_ticks == 0) return;

  lastEncL = curL;
  lastEncR = curR;

  double distL = dL_ticks * CM_PER_TICK;
  double distR = dR_ticks * CM_PER_TICK;
  double distCenter = (distL + distR) / 2.0;
  double dTheta = (distR - distL) / WHEEL_BASE_CM;

  robotTheta += dTheta;
  if (robotTheta > PI)  robotTheta -= TWO_PI;
  if (robotTheta < -PI) robotTheta += TWO_PI;

  robotX += distCenter * cos(robotTheta);
  robotY += distCenter * sin(robotTheta);

  BTSerial.print("POS:");
  BTSerial.print((int)robotX);
  BTSerial.print(",");
  BTSerial.println((int)robotY);

  double distFromLastSave = sqrt(pow(robotX - lastSavedX, 2) + pow(robotY - lastSavedY, 2));

  if (distFromLastSave >= SAVE_INTERVAL_CM) {
    if (eepromAddr < 4090) { 
      int xStore = (int)robotX;
      int yStore = (int)robotY;
      EEPROM.put(eepromAddr, xStore); eepromAddr += sizeof(int);
      EEPROM.put(eepromAddr, yStore); eepromAddr += sizeof(int);
      lastSavedX = robotX;
      lastSavedY = robotY;
      BTSerial.print(">> Saved: "); BTSerial.print(xStore); BTSerial.print(","); BTSerial.println(yStore);
    }
  }
}

// ==================== Wall Following Logic ====================
void wallFollowingMode() {
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool frontValid = dFront > 0;
  bool leftValid  = dLeft  > 0;
  bool rightValid = dRight > 0;

  if (frontValid && dFront < DEAD_END_THRESHOLD &&
      leftValid  && dLeft  < DEAD_END_THRESHOLD &&
      rightValid && dRight < DEAD_END_THRESHOLD) {
      BTSerial.println("--- DEAD END ---");
      stopMotors(); delay(100); pivot180(PIVOT_180_PWM); stopMotors(); delay(100);
  }
  else if (frontValid && dFront < OBSTACLE_THRESHOLD) {
    stopMotors(); delay(100);
    bool turnLeft = (dLeft > dRight);
    bool isUTurn = false;
    if (turnLeft && dLeft > OPEN_SPACE_THRESHOLD_CM)   isUTurn = true;
    if (!turnLeft && dRight > OPEN_SPACE_THRESHOLD_CM) isUTurn = true;

    if (!isUTurn && leftValid && rightValid && (abs(dLeft - dRight) > ALIGN_TOLERANCE_CM)) {
      if (dLeft < dRight) pivotLeft(ALIGN_PWM); else pivotRight(ALIGN_PWM);
      delay(ALIGN_DURATION_MS); stopMotors(); delay(100);
    }
    if (turnLeft) smoothTurnLeft(); else smoothTurnRight();
    stopMotors(); delay(100);
  }
  else if (leftValid && dLeft > NO_WALL_THRESHOLD) {
    BTSerial.println("--- No left wall ---");
    moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
    stopMotors(); delay(10); smoothTurnLeft(); stopMotors(); delay(150);
  }
  else {
    int leftSpeed, rightSpeed;
    if (leftValid && rightValid && dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
      int err = (int)(dLeft - dRight);
      int corr = constrain((int)(Kp_Wall * err), -MAX_CORRECTION, MAX_CORRECTION);
      leftSpeed = constrain(BASE_PWM_STRAIGHT - corr, 0, 100);
      rightSpeed = constrain(BASE_PWM_STRAIGHT + corr, 0, 100);
    } else if (leftValid && dLeft < 10) {
      int err = (int)(dLeft - 6);
      int corr = constrain((int)(Kp_Wall * err), -MAX_CORRECTION, MAX_CORRECTION);
      leftSpeed = constrain(BASE_PWM_STRAIGHT + corr, 0, 100);
      rightSpeed = constrain(BASE_PWM_STRAIGHT - corr, 0, 100);
    } else if (rightValid && dRight < 10) {
      int err = (int)(6 - dRight);
      int corr = constrain((int)(Kp_Wall * err), -MAX_CORRECTION, MAX_CORRECTION);
      leftSpeed = constrain(BASE_PWM_STRAIGHT - corr, 0, 100);
      rightSpeed = constrain(BASE_PWM_STRAIGHT + corr, 0, 100);
    } else {
      leftSpeed = BASE_PWM_STRAIGHT; rightSpeed = BASE_PWM_STRAIGHT;
    }
    moveForward(rightSpeed, leftSpeed);
  }
  delay(10);
}

// ==================== Line Following Logic ====================
void lineFollowingMode() {
  readSensors();
  if (strongLeftFeature() && !centerOnLine()) {
    brake(BRAKE_MS); pivotLeftUntilCenter(); brake(BRAKE_MS);
  } 
  else if (strongRightFeature() && !centerOnLine()) {
    brake(BRAKE_MS); pivotRightUntilCenter(); brake(BRAKE_MS);
  } 
  else {
    if (!anyOnLine()) { searchWhenLost(); return; }
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

// ==================== Motion Functions ====================
void moveForwardDistance(int distance_cm, int pwmVal) {
  const long targetCounts = (long)(((float)distance_cm / (PI * WHEEL_DIAMETER_CM)) * countsPerRev);
  BTSerial.print("Moving FWD: "); BTSerial.println(distance_cm);
  noInterrupts(); encoderCountL = 0; encoderCountR = 0; interrupts();
  moveForward(pwmVal, pwmVal);
  while (true) {
    updateOdometry();
    noInterrupts(); long cL = encoderCountL; long cR = encoderCountR; interrupts();
    if ((absl(cL) + absl(cR)) / 2 >= targetCounts) break;
    delay(10);
  }
  stopMotors();
}

void pivot180(int pwmVal) {
  const long targetCounts = (long)((PI * (WHEEL_BASE_CM / 2.0f) / (PI * WHEEL_DIAMETER_CM)) * countsPerRev);
  BTSerial.println("Pivot 180");
  encoderCountL = 0; encoderCountR = 0;
  pivotRight(pwmVal);
  while(absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    updateOdometry();
    if (absl(encoderCountL) >= targetCounts) stopLeft();
    if (absl(encoderCountR) >= targetCounts) stopRight();
    delay(5);
  }
  stopMotors();
}

void pivotTurn90(bool leftTurn, int pwmOuterMax) {
  const long TICKS_90_DEG = 560; 
  encoderCountL = 0; encoderCountR = 0;
  if (leftTurn) { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0); analogWrite(R_RPWM, pwmOuterMax); analogWrite(R_LPWM, 0); }
  else { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, pwmOuterMax); analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0); }
  while (true) {
    updateOdometry();
    if (absl(encoderCountL) >= TICKS_90_DEG || absl(encoderCountR) >= TICKS_90_DEG) break;
    delay(5);
  }
  stopMotors();
}

void smoothTurnLeft()  { pivotTurn90(true, TURN_PWM); }
void smoothTurnRight() { pivotTurn90(false, TURN_PWM); }

void pivotLeft90UntilLine(int pwmVal) {
  const long targetCounts = (long)(((PI * WHEEL_BASE_CM / 4.0f) / (PI * WHEEL_DIAMETER_CM)) * countsPerRev);
  noInterrupts(); encoderCountL = 0; encoderCountR = 0; interrupts();
  pivotLeft(pwmVal);
  while (absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    updateOdometry();
    readSensors();
    if (anyOnLine()) break;
    if (absl(encoderCountL) >= targetCounts) stopLeft();
    if (absl(encoderCountR) >= targetCounts) stopRight();
    delay(5);
  }
  stopMotors();
}

void pivotRight90UntilLine(int pwmVal) {
  const long targetCounts = (long)(((PI * WHEEL_BASE_CM / 4.0f) / (PI * WHEEL_DIAMETER_CM)) * countsPerRev);
  noInterrupts(); encoderCountL = 0; encoderCountR = 0; interrupts();
  pivotRight(pwmVal);
  while (absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    updateOdometry();
    readSensors();
    if (anyOnLine()) break;
    if (absl(encoderCountL) >= targetCounts) stopLeft();
    if (absl(encoderCountR) >= targetCounts) stopRight();
    delay(5);
  }
  stopMotors();
}

void searchWhenLost() {
  BTSerial.println("Lost Line! Searching...");
  pivotLeft90UntilLine(50);
  pivotRight90UntilLine(50);
  pivotRight90UntilLine(50);
}

// ==================== Basic Hardware Helpers ====================
void moveForward(int pwmValR, int pwmValL) { analogWrite(R_RPWM, pwmValR); analogWrite(R_LPWM, 0); analogWrite(L_RPWM, 0); analogWrite(L_LPWM, pwmValL); }
void pivotLeft(int pwmVal) { analogWrite(R_RPWM, pwmVal); analogWrite(R_LPWM, 0); analogWrite(L_RPWM, pwmVal); analogWrite(L_LPWM, 0); }
void pivotRight(int pwmVal) { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, pwmVal); analogWrite(R_RPWM, 0); analogWrite(R_LPWM, pwmVal); }
void stopMotors() { stopLeft(); stopRight(); }
void brake(uint16_t ms) { stopMotors(); delay(ms); }

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 25000);
  return (dur == 0) ? -1 : dur * 0.034 / 2;
}

void countEncoderL() { if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) encoderCountL++; else encoderCountL--; }
void countEncoderR() { if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) encoderCountR--; else encoderCountR++; }

void readSensors() {
  sensorBits = 0;
  for (int i = 0; i < 8; i++) {
    int raw = digitalRead(sensorPins[i]);
    sensorVal[i] = (BLACK_IS_HIGH ? (raw == HIGH) : (raw == LOW)) ? 1 : 0;
    if (sensorVal[i]) sensorBits |= (1u << i);
  }
}

float getLineError() {
  static const float weight[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
  float sumW = 0.0f, sum = 0.0f;
  for (int i = 0; i < 8; i++) { if (sensorVal[i]) { sumW += weight[i]; sum += 1.0f; } }
  return (sum == 0.0f) ? lastError : sumW / sum;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  analogWrite(R_RPWM, rightSpeed); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0); analogWrite(L_LPWM, leftSpeed);
}

bool anyOnLine() { return sensorBits != 0; }
bool centerOnLine() { return sensorVal[CENTER_IDX[0]] || sensorVal[CENTER_IDX[1]]; }
uint8_t countBlk(const uint8_t idxs[], uint8_t n) { uint8_t c = 0; for (uint8_t k = 0; k < n; k++) c += sensorVal[idxs[k]]; return c; }
bool strongLeftFeature() { return (countBlk(LEFT_IDX, 3) >= LEFT_STRONG_MIN && countBlk(RIGHT_IDX, 3) <= 1); }
bool strongRightFeature() { return (countBlk(RIGHT_IDX, 3) >= RIGHT_STRONG_MIN && countBlk(LEFT_IDX, 3) <= 1); }

void pivotLeftUntilCenter() {
  unsigned long t0 = millis();
  while (millis() - t0 < PIVOT_TIMEOUT_MS) {
    updateOdometry(); analogWrite(R_RPWM, TURN_PWM_LINE); analogWrite(R_LPWM, 0); analogWrite(L_RPWM, TURN_PWM_LINE); analogWrite(L_LPWM, 0);
    readSensors(); if (centerOnLine()) break; delay(4);
  }
}
void pivotRightUntilCenter() {
  unsigned long t0 = millis();
  while (millis() - t0 < PIVOT_TIMEOUT_MS) {
    updateOdometry(); analogWrite(R_RPWM, 0); analogWrite(R_LPWM, TURN_PWM_LINE); analogWrite(L_RPWM, 0); analogWrite(L_LPWM, TURN_PWM_LINE);
    readSensors(); if (centerOnLine()) break; delay(4);
  }
}