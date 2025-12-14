#include <Arduino.h>

// ===================================================================================
// 1. PIN DEFINITIONS
// ===================================================================================
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

#define SEARCH_MODE 33 

// Line Sensor Settings
const uint8_t sensorPins[8] = {22, 23, 24, 25, 26, 27, 28, 29};
const bool BLACK_IS_HIGH = true; 

// ===================================================================================
// 2. CONSTANTS & TUNING
// ===================================================================================
const float WHEEL_DIAMETER_CM = 6.5f;
const int pulsesPerMotorRev   = 11;
const int gearRatio           = 20;
const int countsPerRev        = pulsesPerMotorRev * gearRatio * 2;

// --- Turning ---
const int   TURN_PWM_MAZE  = 50;
const int   PIVOT_180_PWM  = 55;
const int   TICKS_90_DEG   = 560; 

// --- Navigation ---
const int OBSTACLE_THRESHOLD      = 8;
const int OPEN_SPACE_THRESHOLD_CM = 30;
const int DEAD_END_THRESHOLD      = 10;
const int NO_WALL_THRESHOLD       = 25;
const int CORNER_CLEARANCE_CM     = 10; 

// --- Wall Following ---
const int   BASE_PWM_STRAIGHT = 60;
const float Kp_Wall           = 3.5f;
const float Kd_Wall           = 10.0f;
const int   MAX_CORRECTION    = 20;
const int   WALL_DETECT_RANGE = 20;

// --- Alignment ---
const int ALIGN_PWM         = 45;
const int ALIGN_DURATION_MS = 50;
const int ALIGN_TOLERANCE_CM= 1;

// --- Finish / Exit Settings ---
const int FINISH_LINE_SENSITIVITY = 2; 

// --- Line Following PID ---
float Kp_Line = 40.0;
float Ki_Line = 0.0;
float Kd_Line = 4.0;
int baseSpeedLine = 55;
int maxPWMLine    = 130;
int CORR_CLAMP    = 70;

// --- Line Turn & Search ---
const int TURN_PWM_LINE        = 150;
const uint16_t TURN_TIMEOUT_MS = 900;
const uint8_t CENTER_STABLE_N  = 4;
const uint16_t TURN_COOLDOWN_MS= 450;
const int SEARCH_PWM           = 80;
const uint16_t SEARCH_MS       = 450;
const uint8_t ALL_BLACK_MIN    = 7;
const int COAST_PWM            = 45;
const uint16_t ALL_BLACK_COAST_MS = 250;

const uint8_t LEFT_IDX[3]   = {0,1,2};
const uint8_t CENTER_IDX[2] = {3,4};
const uint8_t RIGHT_IDX[3]  = {5,6,7};

// ===================================================================================
// 3. GLOBALS & STATE MACHINE
// ===================================================================================
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;
unsigned long lastDebugTime = 0; 

char pathSmall[100];       
int pathLenSmall = 0;   
char pathBig[100];
int pathLenBig = 0;

char* currentPathArr; 
int*  currentPathLenPtr;
int   readIndex = 0;    

uint8_t sensorVal[8];
uint8_t sensorBits = 0;

float lastErrorLine = 0;
float integralLine = 0;
unsigned long lastTurnTime = 0;
unsigned long lastEventPrint = 0;
unsigned long lastPidPrint = 0;
const unsigned long PID_PRINT_MS = 120;
const unsigned long EVENT_GAP_MS = 40;

unsigned long modeStartTime = 0;

enum RobotState {
  STATE_MAPPING_SMALL,   
  STATE_TRANS_TO_LINE,   
  STATE_LINE_FOLLOW,     
  STATE_MAPPING_BIG,     
  STATE_WAIT_FOR_RUN2,   
  STATE_SOLVING_SMALL,   
  STATE_SOLVING_LINE,    
  STATE_SOLVING_BIG,     
  STATE_DONE             
};

RobotState currentState = STATE_MAPPING_SMALL;

// ===================================================================================
// 4. FUNCTION PROTOTYPES
// ===================================================================================
void stopMotors(); 
long readUltrasonic(int trigPin, int echoPin);
void readLineSensors();
void countEncoderL(); void countEncoderR();
void moveForward(int pwmValR, int pwmValL);
void moveForwardDistance(int distance_cm, int pwmVal);
void pivotTurn90_Maze(bool leftTurn, int pwmOuterMax);
void pivot180(int pwmVal);
void smoothTurnLeft(); void smoothTurnRight();
void pivotLeft_Maze(int pwmVal); void pivotRight_Maze(int pwmVal);
void logicLineFollow(bool isMappingPhase);
// Line Follow Helpers
void linePivotLeft(); void linePivotRight(); void searchWhenLost();
uint8_t countIdx(const uint8_t* idx, uint8_t n);
uint8_t countAll(); bool anyOnLine(); float getLineError();
void debugEvent(const char* tag, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc);
void debugPid(const char* mode, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc, int mL, int mR);
void printBits(uint8_t b);
// Logic
void logicMapping();
void logicSolving();
void PID_Logic_Wall(long dLeft, long dRight);
void recordPath(char direction);
void simplifyPath();
void printPaths();
void printDebugDashboard();
inline long absl(long x){ return (x < 0) ? -x : x; }
inline void stopRight(){ analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0); }
inline void stopLeft() { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0); }

// ===================================================================================
// 5. SETUP
// ===================================================================================
void setup() {
  Serial.begin(9600);
  Serial3.begin(9600); 
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
  pinMode(SEARCH_MODE, INPUT_PULLUP); 
  for(int i=0; i<8; i++) pinMode(sensorPins[i], INPUT);
  currentPathArr = pathSmall;
  currentPathLenPtr = &pathLenSmall;
  stopMotors();
  Serial3.println("BT: Robot Ready. Sustained Black Logic.");
  modeStartTime = millis();
}

// ===================================================================================
// 6. MAIN LOOP
// ===================================================================================
void loop() {
  printDebugDashboard(); 
  switch (currentState) {
    case STATE_MAPPING_SMALL:
      logicMapping(); 
      break;

    case STATE_TRANS_TO_LINE:
      // Immediate transition, no delay.
      currentState = STATE_LINE_FOLLOW;
      break;

    case STATE_LINE_FOLLOW:
      logicLineFollow(true); 
      break;

    case STATE_MAPPING_BIG:
      logicMapping();
      break;

    case STATE_WAIT_FOR_RUN2:
      stopMotors();
      if (millis() - lastDebugTime > 2000) {
        Serial3.println("BT: Waiting for Pin 33 (LOW)...");
        printPaths();
        lastDebugTime = millis();
      }
      if (digitalRead(SEARCH_MODE) == LOW) {
        delay(1000);
        Serial3.println("BT: STARTING SOLVE RUN!");
        currentPathArr = pathSmall;
        currentPathLenPtr = &pathLenSmall;
        readIndex = 0;
        modeStartTime = millis();
        currentState = STATE_SOLVING_SMALL;
      }
      break;

    case STATE_SOLVING_SMALL:
      logicSolving();
      break;

    case STATE_SOLVING_LINE:
      logicLineFollow(false); 
      break;

    case STATE_SOLVING_BIG:
      logicSolving();
      break;

    case STATE_DONE:
      stopMotors();
      break;
  }
}

// ===================================================================================
// 7. LOGIC: LINE FOLLOWING (SUSTAINED BLACK -> BIG MAZE)
// ===================================================================================
void logicLineFollow(bool isMappingPhase) {
  readLineSensors(); 
  uint8_t Lc = countIdx(LEFT_IDX, 3);
  uint8_t Cc = countIdx(CENTER_IDX, 2);
  uint8_t Rc = countIdx(RIGHT_IDX, 3);
  uint8_t Bc = countAll();
  bool inCooldown = (millis() - lastTurnTime < TURN_COOLDOWN_MS);

  // Variable to track how long we have seen All Black
  static unsigned long blackDetectionStartTime = 0;
  const int BLACK_DURATION_THRESHOLD = 400; // Milliseconds

  // ==========================================================
  // 1. DETECT SUSTAINED ALL BLACK
  // ==========================================================
  if (Bc >= ALL_BLACK_MIN) {
    // Start timer if not already started
    if (blackDetectionStartTime == 0) {
      blackDetectionStartTime = millis();
    }

    // Check duration
    if (millis() - blackDetectionStartTime > BLACK_DURATION_THRESHOLD) {
        stopMotors();
        Serial3.println("BT: >>> TRANSITION: SUSTAINED BLACK -> BIG MAZE!");
        
        moveForwardDistance(15, BASE_PWM_STRAIGHT); 
        
        // Reset flags
        blackDetectionStartTime = 0;
        modeStartTime = millis(); 

        if (isMappingPhase) {
            currentPathArr = pathBig;
            currentPathLenPtr = &pathLenBig;
            currentState = STATE_MAPPING_BIG;
        } else {
            currentPathArr = pathBig;
            currentPathLenPtr = &pathLenBig;
            readIndex = 0;
            currentState = STATE_SOLVING_BIG;
        }
        return; // EXIT LINE FOLLOWER
    }

    // While waiting, coast forward
    moveForward(baseSpeedLine, baseSpeedLine);
    return; 
  } 
  else {
    // Reset timer if we see white
    blackDetectionStartTime = 0;
  }

  // ==========================================================
  // 2. 90° CORNER DETECTION
  // ==========================================================
  bool forwardLost = (Cc == 0);
  bool strongLeft  = (Lc >= 2 && Rc == 0);
  bool strongRight = (Rc >= 2 && Lc == 0);
  bool bothSides   = (Lc >= 2 && Rc >= 2);
  bool left90  = (!inCooldown && forwardLost && strongLeft);
  bool right90 = (!inCooldown && forwardLost && strongRight);

  if (left90) {
    debugEvent("TURN_90_LEFT", Lc, Cc, Rc, Bc);
    linePivotLeft(); lastTurnTime = millis(); integralLine = 0; lastErrorLine = -2.5; return;
  }
  if (right90) {
    debugEvent("TURN_90_RIGHT", Lc, Cc, Rc, Bc);
    linePivotRight(); lastTurnTime = millis(); integralLine = 0; lastErrorLine = 2.5; return;
  }
  if (!inCooldown && forwardLost && bothSides) {
    debugEvent("TURN_BOTH_SIDES", Lc, Cc, Rc, Bc);
    if (Lc > Rc) linePivotLeft(); else if (Rc > Lc) linePivotRight();
    else { if (lastErrorLine >= 0) linePivotRight(); else linePivotLeft(); }
    lastTurnTime = millis(); integralLine = 0; return;
  }

  // ==========================================================
  // 3. LOST LINE SEARCH
  // ==========================================================
  if (!anyOnLine()) {
     debugEvent("LOST_LINE", Lc, Cc, Rc, Bc);
     searchWhenLost(); 
     return;
  }

  // ==========================================================
  // 4. PID CONTROL
  // ==========================================================
  float error = getLineError();
  integralLine += error;
  float derivative = error - lastErrorLine;
  float corr = (Kp_Line * error) + (Ki_Line * integralLine) + (Kd_Line * derivative);
  corr = constrain(corr, -CORR_CLAMP, CORR_CLAMP);
  lastErrorLine = error;
  int mL = constrain((int)(baseSpeedLine + corr), 0, maxPWMLine);
  int mR = constrain((int)(baseSpeedLine - corr), 0, maxPWMLine);
  moveForward(mR, mL); 
  debugPid("PID", Lc, Cc, Rc, Bc, mL, mR);
  delay(5);
}

// ===================================================================================
// 8. MAZE MAPPING
// ===================================================================================
void logicMapping() {
  readLineSensors();
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) if (sensorVal[i] == 0) whiteCount++; 

  if (whiteCount >= FINISH_LINE_SENSITIVITY) {
    stopMotors(); 
    if (currentState == STATE_MAPPING_SMALL) {
       Serial3.println("BT: >>> TRANSITION: MAP SMALL -> LINE FOLLOW");
       recordPath('S');
       moveForwardDistance(10, BASE_PWM_STRAIGHT); 
       currentState = STATE_TRANS_TO_LINE;
    } 
    else if (currentState == STATE_MAPPING_BIG) {
       Serial3.println("BT: >>> TRANSITION: MAP BIG -> WAIT");
       recordPath('S'); 
       currentState = STATE_WAIT_FOR_RUN2;
    }
    return;
  }

  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  bool frontValid = (dFront > 0); bool leftValid  = (dLeft  > 0); bool rightValid = (dRight > 0);

  if (frontValid && dFront < OBSTACLE_THRESHOLD && leftValid  && dLeft  < DEAD_END_THRESHOLD && rightValid && dRight < DEAD_END_THRESHOLD) {
      stopMotors(); delay(100); pivot180(PIVOT_180_PWM); recordPath('B'); stopMotors(); delay(100); return;
  } else if (frontValid && dFront < OBSTACLE_THRESHOLD) {
    stopMotors(); delay(100);
    if (leftValid && rightValid && (abs(dLeft - dRight) > ALIGN_TOLERANCE_CM)) {
      if (dLeft < dRight) pivotLeft_Maze(ALIGN_PWM); else pivotRight_Maze(ALIGN_PWM);
      delay(ALIGN_DURATION_MS); stopMotors(); delay(100);
    }
    if (leftValid && dLeft > NO_WALL_THRESHOLD) { smoothTurnLeft(); recordPath('L'); } 
    else { smoothTurnRight(); recordPath('R'); }
    stopMotors(); delay(100); return;
  } else if (leftValid && dLeft > NO_WALL_THRESHOLD) {
    moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
    stopMotors(); delay(10); smoothTurnLeft(); recordPath('L'); stopMotors(); delay(150); return;
  } else {
    if (rightValid && dRight > NO_WALL_THRESHOLD) {
        if (dFront > OPEN_SPACE_THRESHOLD_CM) { recordPath('S'); moveForwardDistance(15, BASE_PWM_STRAIGHT); return; }
    }
    PID_Logic_Wall(dLeft, dRight);
  }
}

// ===================================================================================
// 9. MAZE SOLVING
// ===================================================================================
void logicSolving() {
  readLineSensors();
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) if (sensorVal[i] == 0) whiteCount++;

  if (whiteCount >= FINISH_LINE_SENSITIVITY) {
    stopMotors(); 
    if (currentState == STATE_SOLVING_SMALL) {
      Serial3.println("BT: Small Solved -> Line Follow");
      moveForwardDistance(10, BASE_PWM_STRAIGHT);
      currentState = STATE_SOLVING_LINE;
    } else if (currentState == STATE_SOLVING_BIG) {
      Serial3.println("BT: Big Solved -> DONE"); currentState = STATE_DONE;
    }
    return;
  }

  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  bool leftOpen = (dLeft > NO_WALL_THRESHOLD); bool rightOpen = (dRight > NO_WALL_THRESHOLD);
  bool frontBlocked = (dFront > 0 && dFront < OBSTACLE_THRESHOLD);

  if (leftOpen || rightOpen || frontBlocked) {
    stopMotors(); delay(100);
    if (frontBlocked && dLeft > 0 && dRight > 0 && abs(dLeft - dRight) > ALIGN_TOLERANCE_CM) {
       if (dLeft < dRight) pivotLeft_Maze(ALIGN_PWM); else pivotRight_Maze(ALIGN_PWM);
       delay(ALIGN_DURATION_MS); stopMotors();
    }
    if (readIndex < *currentPathLenPtr) {
      char move = currentPathArr[readIndex]; readIndex++;
      Serial3.print("BT: Doing -> "); Serial3.println(move);
      if (move == 'L') { moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT); stopMotors(); delay(10); smoothTurnLeft(); stopMotors(); delay(100); } 
      else if (move == 'R') { moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT); stopMotors(); delay(10); smoothTurnRight(); stopMotors(); delay(100); } 
      else if (move == 'S') { moveForwardDistance(15, BASE_PWM_STRAIGHT); }
    } else { PID_Logic_Wall(dLeft, dRight); }
  } else { PID_Logic_Wall(dLeft, dRight); }
}

// ===================================================================================
// 10. GENERAL HELPERS
// ===================================================================================
void PID_Logic_Wall(long dLeft, long dRight) {
  int leftSpeed, rightSpeed;
  bool leftValid = (dLeft > 0); bool rightValid = (dRight > 0);
  if (leftValid && rightValid && dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
    int error = (int)(dLeft - dRight); int correction = (int)(Kp_Wall * error);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    leftSpeed = constrain(BASE_PWM_STRAIGHT - correction, 0, 100); rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
  } else if (leftValid && dLeft < 10) {
    int error = (int)(6 - dLeft); int correction = (int)(Kp_Wall * error);
    leftSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100); rightSpeed = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
  } else if (rightValid && dRight < 10) {
    int error = (int)(6 - dRight); int correction = (int)(Kp_Wall * error);
    leftSpeed = constrain(BASE_PWM_STRAIGHT - correction, 0, 100); rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
  } else { leftSpeed = BASE_PWM_STRAIGHT; rightSpeed = BASE_PWM_STRAIGHT; }
  moveForward(rightSpeed, leftSpeed); delay(10);
}
void recordPath(char direction) { currentPathArr[*currentPathLenPtr] = direction; (*currentPathLenPtr)++; simplifyPath(); }
void simplifyPath() {
  int len = *currentPathLenPtr; if (len < 3 || currentPathArr[len-2] != 'B') return;
  char prev = currentPathArr[len-3]; char curr = currentPathArr[len-1]; char newMove = '?';
  if (prev == 'L' && curr == 'L') newMove = 'S'; else if (prev == 'L' && curr == 'S') newMove = 'R';
  else if (prev == 'R' && curr == 'L') newMove = 'B'; else if (prev == 'S' && curr == 'L') newMove = 'R';
  else if (prev == 'S' && curr == 'S') newMove = 'B'; else if (prev == 'L' && curr == 'R') newMove = 'B';
  if (newMove != '?') { currentPathArr[len-3] = newMove; *currentPathLenPtr -= 2; }
}
void printPaths() {
  Serial3.print("Path Small: "); for(int i=0; i<pathLenSmall; i++) Serial3.print(pathSmall[i]); Serial3.println();
  Serial3.print("Path Big: "); for(int i=0; i<pathLenBig; i++) Serial3.print(pathBig[i]); Serial3.println();
}
void pivotTurn90_Maze(bool leftTurn, int pwmOuterMax) {
  int pwm = constrain(pwmOuterMax, 40, 255); encoderCountL = 0; encoderCountR = 0;
  if (leftTurn) { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0); analogWrite(R_RPWM, pwm); analogWrite(R_LPWM, 0); } 
  else { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, pwm); analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0); }
  while (true) { if ((absl(encoderCountL) >= TICKS_90_DEG) || (absl(encoderCountR) >= TICKS_90_DEG)) break; delay(5); }
  moveForward(BASE_PWM_STRAIGHT, BASE_PWM_STRAIGHT); delay(50); stopMotors();
}
void pivot180(int pwmVal) {
  const int targetCounts = 550; encoderCountL = 0; encoderCountR = 0; pivotRight_Maze(pwmVal);
  while(absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    if (absl(encoderCountL) >= targetCounts) stopLeft(); if (absl(encoderCountR) >= targetCounts) stopRight(); delay(5);
  } stopMotors();
}
void smoothTurnLeft() { pivotTurn90_Maze(true, TURN_PWM_MAZE); }
void smoothTurnRight() { pivotTurn90_Maze(false, TURN_PWM_MAZE); }
void pivotLeft_Maze(int pwmVal) { analogWrite(R_RPWM, pwmVal); analogWrite(R_LPWM, 0); analogWrite(L_RPWM, pwmVal); analogWrite(L_LPWM, 0); }
void pivotRight_Maze(int pwmVal){ analogWrite(L_RPWM, 0); analogWrite(L_LPWM, pwmVal); analogWrite(R_RPWM, 0); analogWrite(R_LPWM, pwmVal); }
void moveForward(int pwmValR, int pwmValL) { analogWrite(R_RPWM, pwmValR); analogWrite(R_LPWM, 0); analogWrite(L_RPWM, 0); analogWrite(L_LPWM, pwmValL); }
void moveForwardDistance(int distance_cm, int pwmVal) {
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM; const long targetCounts = (long)(((float)distance_cm / wheelCircumference) * countsPerRev);
  noInterrupts(); encoderCountL = 0; encoderCountR = 0; interrupts(); moveForward(pwmVal, pwmVal);
  while (true) { noInterrupts(); long cL = encoderCountL; long cR = encoderCountR; interrupts(); if ((absl(cL) + absl(cR)) / 2 >= targetCounts) break; delay(10); } stopMotors();
}
void readLineSensors() {
  sensorBits = 0; for (int i = 0; i < 8; i++) {
    int raw = digitalRead(sensorPins[i]); bool isBlack = BLACK_IS_HIGH ? (raw == HIGH) : (raw == LOW);
    sensorVal[i] = isBlack ? 1 : 0; if (isBlack) sensorBits |= (1u << i);
  }
}
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2); digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); if (duration == 0) return -1; return duration * 0.034 / 2;
}
void stopMotors() { stopLeft(); stopRight(); }
void countEncoderL() { if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) encoderCountL++; else encoderCountL--; }
void countEncoderR() { if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) encoderCountR--; else encoderCountR++; }
void printDebugDashboard() {
  static unsigned long lastPrint = 0; if (millis() - lastPrint < 300) return; lastPrint = millis();
  long dLeft = readUltrasonic(TRIG_LEFT, ECHO_LEFT); long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  int wCount = 0; int bCount = 0; String irBits = "";
  for(int i=0; i<8; i++) { irBits += String(sensorVal[i]); if(sensorVal[i] == 0) wCount++; if(sensorVal[i] == 1) bCount++; }
  Serial3.print("["); 
  switch(currentState) {
    case STATE_MAPPING_SMALL: Serial3.print("MAP_SM"); break; case STATE_TRANS_TO_LINE: Serial3.print("TRANS"); break;
    case STATE_LINE_FOLLOW: Serial3.print("LINE"); break; case STATE_MAPPING_BIG: Serial3.print("MAP_BG"); break;
    case STATE_WAIT_FOR_RUN2: Serial3.print("WAIT"); break; case STATE_SOLVING_SMALL: Serial3.print("SOL_SM"); break;
    case STATE_SOLVING_LINE: Serial3.print("SOL_LN"); break; case STATE_SOLVING_BIG: Serial3.print("SOL_BG"); break;
    case STATE_DONE: Serial3.print("DONE"); break;
  }
  Serial3.print("] IR:"); Serial3.print(irBits); Serial3.print(" W:"); Serial3.print(wCount); Serial3.print(" B:"); Serial3.print(bCount);
  Serial3.print(" | Walls L:"); Serial3.print(dLeft); Serial3.print(" R:"); Serial3.println(dRight);
}

// ===================================================================================
// 11. MISSING HELPER FUNCTIONS (ADDED HERE)
// ===================================================================================
void linePivotLeft() {
  unsigned long t0 = millis(); uint8_t stable = 0;
  while (millis() - t0 < TURN_TIMEOUT_MS) {
    analogWrite(R_RPWM, TURN_PWM_LINE); analogWrite(R_LPWM, 0);
    analogWrite(L_RPWM, 0); analogWrite(L_LPWM, TURN_PWM_LINE);
    readLineSensors();
    if (sensorVal[3] || sensorVal[4]) stable++; else stable = 0;
    if (stable >= CENTER_STABLE_N) break; delay(4);
  } stopMotors();
}
void linePivotRight() {
  unsigned long t0 = millis(); uint8_t stable = 0;
  while (millis() - t0 < TURN_TIMEOUT_MS) {
    analogWrite(L_RPWM, TURN_PWM_LINE); analogWrite(L_LPWM, 0);
    analogWrite(R_RPWM, 0); analogWrite(R_LPWM, TURN_PWM_LINE);
    readLineSensors();
    if (sensorVal[3] || sensorVal[4]) stable++; else stable = 0;
    if (stable >= CENTER_STABLE_N) break; delay(4);
  } stopMotors();
}
void searchWhenLost() {
  if (lastErrorLine >= 0) { analogWrite(L_RPWM, SEARCH_PWM); analogWrite(L_LPWM, 0); analogWrite(R_RPWM, 0); analogWrite(R_LPWM, SEARCH_PWM); }
  else { analogWrite(R_RPWM, SEARCH_PWM); analogWrite(R_LPWM, 0); analogWrite(L_RPWM, 0); analogWrite(L_LPWM, SEARCH_PWM); }
  unsigned long t0 = millis();
  while (millis() - t0 < SEARCH_MS) {
    readLineSensors(); if (anyOnLine()) { stopMotors(); return; } delay(4);
  } stopMotors();
}
uint8_t countIdx(const uint8_t* idx, uint8_t n) { uint8_t c=0; for(uint8_t i=0;i<n;i++) c += sensorVal[idx[i]]; return c; }
uint8_t countAll() { uint8_t c=0; for(int i=0;i<8;i++) c += sensorVal[i]; return c; }
bool anyOnLine() { return sensorBits != 0; }
float getLineError() {
  static const float w[8] = {-3.5,-2.5,-1.5,-0.5,0.5,1.5,2.5,3.5};
  float sw=0, sn=0; for(int i=0;i<8;i++) { if(sensorVal[i]) { sw += w[i]; sn += 1.0; } }
  return sn ? (sw/sn) : lastErrorLine;
}
void printBits(uint8_t b) { for (int i = 7; i >= 0; i--) Serial.print((b >> i) & 1); }
void debugEvent(const char* tag, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc) {
  if (millis() - lastEventPrint < EVENT_GAP_MS) return; lastEventPrint = millis();
  Serial.print("EVT: "); Serial.print(tag); Serial.print(" B:"); Serial.println(Bc);
}
void debugPid(const char* mode, uint8_t Lc, uint8_t Cc, uint8_t Rc, uint8_t Bc, int mL, int mR) {
  if (millis() - lastPidPrint < PID_PRINT_MS) return; lastPidPrint = millis();
}