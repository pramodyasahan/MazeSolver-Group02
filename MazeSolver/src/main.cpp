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
const int   TURN_PWM       = 50;
const int   PIVOT_180_PWM  = 55;
const int   TICKS_90_DEG   = 560; 

// --- Navigation Thresholds ---
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

// --- Finish / Transition Settings ---
const int FINISH_LINE_SENSITIVITY = 2; // Number of White sensors to detect finish
const int BLACK_LINE_SENSITIVITY  = 6; // Number of Black sensors to detect transition to Big Maze

// --- Line Following PID ---
const int   TURN_PWM_LINE     = 140; 
float Kp_Line = 50.0;
float Ki_Line = 0.0;
float Kd_Line = 4.0;
int baseSpeedLine = 50;
int maxPWMLine    = 100;

// --- Line Logic Thresholds ---
const uint8_t CENTER_IDX[2] = {3, 4};
const uint8_t LEFT_IDX[3]   = {0, 1, 2};
const uint8_t RIGHT_IDX[3]  = {5, 6, 7};
const uint8_t LEFT_STRONG_MIN  = 2;
const uint8_t RIGHT_STRONG_MIN = 2;
const uint16_t PIVOT_TIMEOUT_MS = 600;
const uint16_t BRAKE_MS = 30;

// ===================================================================================
// 3. GLOBALS & STATE MACHINE
// ===================================================================================
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;
unsigned long lastDebugTime = 0; 

// --- Path Memory ---
char pathSmall[100];       
int pathLenSmall = 0;   
char pathBig[100];
int pathLenBig = 0;

// Pointers to the "Active" path being recorded or read
char* currentPathArr; 
int*  currentPathLenPtr;
int   readIndex = 0;    

uint8_t sensorVal[8];
uint8_t sensorBits = 0;

float lastErrorWall = 0;
float lastErrorLine = 0;
float integralLine = 0;

unsigned long modeStartTime = 0;

// --- Updated States ---
enum RobotState {
  STATE_MAPPING_SMALL,   // Phase 1: Map Small Maze
  STATE_TRANS_TO_LINE,   // Small -> Line Transition
  STATE_LINE_FOLLOW,     // Phase 1: Follow Line to Big Maze
  STATE_MAPPING_BIG,     // Phase 1: Map Big Maze
  STATE_WAIT_FOR_RUN2,   // Phase 1 Done: Wait for Pin 33
  STATE_SOLVING_SMALL,   // Phase 2: Solve Small Maze
  STATE_SOLVING_LINE,    // Phase 2: Follow Line
  STATE_SOLVING_BIG,     // Phase 2: Solve Big Maze
  STATE_DONE             // Completely Finished
};

RobotState currentState = STATE_MAPPING_SMALL;

// ===================================================================================
// 4. FUNCTION PROTOTYPES
// ===================================================================================
void stopMotors(); 
long readUltrasonic(int trigPin, int echoPin);
void readLineSensors();
void printIRValues(); 
void countEncoderL(); void countEncoderR();

void moveForward(int pwmValR, int pwmValL);
void moveForwardDistance(int distance_cm, int pwmVal);
void pivotTurn90(bool leftTurn, int pwmOuterMax);
void pivot180(int pwmVal);
void smoothTurnLeft(); void smoothTurnRight();
void pivotLeft(int pwmVal); void pivotRight(int pwmVal);

// Line Follow Helpers
void pivotLeftUntilCenter(); void pivotRightUntilCenter();
void pivotLeft90UntilLine(int pwmVal); 

// Logic
void logicMapping();
void logicSolving();
void logicLineFollow(bool isMappingPhase); // Combined logic
void PID_Logic_Wall(long dLeft, long dRight);
void recordPath(char direction);
void simplifyPath();
void printPaths();
void brake(uint16_t ms);

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

  // Initialize Pointers for Small Maze Mapping
  currentPathArr = pathSmall;
  currentPathLenPtr = &pathLenSmall;

  stopMotors();
  Serial3.println("BT: Robot Ready. Optimized Logic.");
  modeStartTime = millis();
}

// ===================================================================================
// 6. MAIN LOOP
// ===================================================================================
void loop() {
  switch (currentState) {
    
    // --- PHASE 1: MAPPING ---
    case STATE_MAPPING_SMALL:
      logicMapping(); 
      break;

    case STATE_TRANS_TO_LINE:
      // Just a small push to ensure we are clearly ON the line before logic takes over
      moveForwardDistance(4, BASE_PWM_STRAIGHT);
      currentState = STATE_LINE_FOLLOW;
      break;

    case STATE_LINE_FOLLOW:
      // True = Mapping Phase (Looks for walls+black to enter big maze)
      logicLineFollow(true); 
      break;

    case STATE_MAPPING_BIG:
      logicMapping();
      break;

    // --- INTERMISSION ---
    case STATE_WAIT_FOR_RUN2:
      stopMotors();
      if (millis() - lastDebugTime > 2000) {
        Serial3.println("BT: Waiting for Pin 33 (LOW) to Solve...");
        printPaths();
        lastDebugTime = millis();
      }
      if (digitalRead(SEARCH_MODE) == LOW) {
        delay(1000); // Debounce
        Serial3.println("BT: STARTING SOLVE RUN!");
        
        // Setup for Solving Small Maze
        currentPathArr = pathSmall;
        currentPathLenPtr = &pathLenSmall;
        readIndex = 0;
        
        modeStartTime = millis();
        currentState = STATE_SOLVING_SMALL;
      }
      break;

    // --- PHASE 2: SOLVING ---
    case STATE_SOLVING_SMALL:
      logicSolving();
      break;

    case STATE_SOLVING_LINE:
      // False = Solving Phase (Looks for walls+black to enter Big Maze Solve)
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
// 7. MAZE MAPPING LOGIC
// ===================================================================================
void logicMapping() {
  readLineSensors();
  
  // 1. CHECK FOR END OF CURRENT MAZE (White Line)
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) if (sensorVal[i] == 0) whiteCount++; 

  if (whiteCount >= FINISH_LINE_SENSITIVITY) {
    stopMotors(); 
    
    if (currentState == STATE_MAPPING_SMALL) {
       Serial3.println("BT: Small Map Done -> Line Follow");
       recordPath('S'); // Usually straight out of the maze
       currentState = STATE_TRANS_TO_LINE;
    } 
    else if (currentState == STATE_MAPPING_BIG) {
       Serial3.println("BT: Big Map Done -> Wait");
       recordPath('S'); 
       currentState = STATE_WAIT_FOR_RUN2;
    }
    return;
  }

  // 2. STANDARD WALL FOLLOWING & MAPPING
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool frontValid = (dFront > 0);
  bool leftValid  = (dLeft  > 0);
  bool rightValid = (dRight > 0);

  // A. DEAD END
  if (frontValid && dFront < OBSTACLE_THRESHOLD &&
      leftValid  && dLeft  < DEAD_END_THRESHOLD &&
      rightValid && dRight < DEAD_END_THRESHOLD)
  {
      stopMotors(); delay(100);
      pivot180(PIVOT_180_PWM);
      recordPath('B'); 
      stopMotors(); delay(100);
      return;
  }

  // B. OBSTACLE AHEAD
  else if (frontValid && dFront < OBSTACLE_THRESHOLD) {
    stopMotors(); delay(100);

    // Alignment before turn
    if (leftValid && rightValid && (abs(dLeft - dRight) > ALIGN_TOLERANCE_CM)) {
      if (dLeft < dRight) pivotLeft(ALIGN_PWM); else pivotRight(ALIGN_PWM);
      delay(ALIGN_DURATION_MS);
      stopMotors(); delay(100);
    }

    bool realLeftOpen = (leftValid && dLeft > NO_WALL_THRESHOLD);
    if (realLeftOpen) {
       smoothTurnLeft();
       recordPath('L');
    } else {
       smoothTurnRight();
       recordPath('R');
    }
    stopMotors(); delay(100);
    return;
  }

  // C. LEFT OPEN
  else if (leftValid && dLeft > NO_WALL_THRESHOLD) {
    moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
    stopMotors(); delay(10);
    smoothTurnLeft();
    recordPath('L'); 
    stopMotors(); delay(150); 
    return;
  }

  // D. STRAIGHT (PID)
  else {
    if (rightValid && dRight > NO_WALL_THRESHOLD) {
        // If right is open but front is clear, we must go straight (Left Hand Rule preference)
        if (dFront > OPEN_SPACE_THRESHOLD_CM) {
             recordPath('S');
             moveForwardDistance(15, BASE_PWM_STRAIGHT); 
             return;
        }
    }
    PID_Logic_Wall(dLeft, dRight);
  }
}

// ===================================================================================
// 8. MAZE SOLVING LOGIC
// ===================================================================================
void logicSolving() {
  readLineSensors();

  // 1. CHECK FOR END OF CURRENT MAZE
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) if (sensorVal[i] == 0) whiteCount++;

  if (whiteCount >= FINISH_LINE_SENSITIVITY) {
    stopMotors(); 
    
    if (currentState == STATE_SOLVING_SMALL) {
      Serial3.println("BT: Small Solved -> Line Follow");
      moveForwardDistance(5, BASE_PWM_STRAIGHT); 
      currentState = STATE_SOLVING_LINE;
    }
    else if (currentState == STATE_SOLVING_BIG) {
      Serial3.println("BT: Big Solved -> DONE");
      currentState = STATE_DONE;
    }
    return;
  }

  // 2. EXECUTE PATH
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool leftOpen = (dLeft > NO_WALL_THRESHOLD);
  bool rightOpen = (dRight > NO_WALL_THRESHOLD);
  bool frontBlocked = (dFront > 0 && dFront < OBSTACLE_THRESHOLD);

  // Check if we are at an intersection or corner
  if (leftOpen || rightOpen || frontBlocked) {
    stopMotors(); delay(100);

    // Align if hit wall
    if (frontBlocked && dLeft > 0 && dRight > 0 && abs(dLeft - dRight) > ALIGN_TOLERANCE_CM) {
       if (dLeft < dRight) pivotLeft(ALIGN_PWM); else pivotRight(ALIGN_PWM);
       delay(ALIGN_DURATION_MS);
       stopMotors();
    }

    if (readIndex < *currentPathLenPtr) {
      char move = currentPathArr[readIndex];
      readIndex++;
      Serial3.print("BT: Doing -> "); Serial3.println(move);

      if (move == 'L') {
        moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT); 
        stopMotors(); delay(10);
        smoothTurnLeft();
        stopMotors(); delay(100);
      } 
      else if (move == 'R') {
        moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT); 
        stopMotors(); delay(10);
        smoothTurnRight();
        stopMotors(); delay(100);
      } 
      else if (move == 'S') {
        moveForwardDistance(15, BASE_PWM_STRAIGHT);
      }
    } else {
       // Ran out of instructions, default to wall follow just in case
       PID_Logic_Wall(dLeft, dRight);
    }
  } 
  else {
     PID_Logic_Wall(dLeft, dRight);
  }
}

// ===================================================================================
// 9. LINE FOLLOWING LOGIC (Handles transition to Big Maze)
// ===================================================================================
void logicLineFollow(bool isMappingPhase) {
  readLineSensors();
  
  // 1. CHECK FOR ENTRY TO BIG MAZE (Black Line End + Walls)
  // Condition: Mostly Black line sensors OR End of line AND Walls detected
  int blackCount = 0;
  for(int i=0; i<8; i++) if(sensorVal[i] == 1) blackCount++;

  if (blackCount >= BLACK_LINE_SENSITIVITY) {
    long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
    long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
    
    // Check for walls to confirm we are entering the maze
    if ((dLeft > 0 && dLeft < WALL_DETECT_RANGE) || (dRight > 0 && dRight < WALL_DETECT_RANGE)) {
        stopMotors();
        Serial3.println("BT: Entering Big Maze...");
        
        moveForwardDistance(10, BASE_PWM_STRAIGHT); // Move fully into maze
        
        if (isMappingPhase) {
            // Setup for Mapping Big Maze
            currentPathArr = pathBig;
            currentPathLenPtr = &pathLenBig;
            currentState = STATE_MAPPING_BIG;
        } else {
            // Setup for Solving Big Maze
            currentPathArr = pathBig;
            currentPathLenPtr = &pathLenBig;
            readIndex = 0;
            currentState = STATE_SOLVING_BIG;
        }
        return;
    }
  }

  // 2. INTERSECTIONS (Standard Line Following)
  bool center = (sensorVal[CENTER_IDX[0]] || sensorVal[CENTER_IDX[1]]);
  uint8_t leftCount = 0;  for(int i=0; i<3; i++) if(sensorVal[LEFT_IDX[i]]) leftCount++;
  uint8_t rightCount = 0; for(int i=0; i<3; i++) if(sensorVal[RIGHT_IDX[i]]) rightCount++;

  if (leftCount >= LEFT_STRONG_MIN && !center) {
    brake(BRAKE_MS); pivotLeftUntilCenter(); brake(BRAKE_MS);
  }
  else if (rightCount >= RIGHT_STRONG_MIN && !center) {
    brake(BRAKE_MS); pivotRightUntilCenter(); brake(BRAKE_MS);
  }
  else {
    // 3. PID
    if (sensorBits == 0) {
       // If lost line, drift slightly or stop. 
       // In transition to big maze, it might be all white for a moment or all black.
       moveForward(baseSpeedLine, baseSpeedLine); 
       return;
    }

    static const float weight[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
    float sumW = 0.0f, sum = 0.0f;
    for (int i = 0; i < 8; i++) {
      if (sensorVal[i]) { sumW += weight[i]; sum += 1.0f; }
    }
    float error = (sum != 0.0f) ? (sumW / sum) : lastErrorLine;

    integralLine += error;
    float derivative = error - lastErrorLine;
    float correction = (Kp_Line * error) + (Ki_Line * integralLine) + (Kd_Line * derivative);
    lastErrorLine = error;

    int leftSpeed  = constrain((int)(baseSpeedLine + correction), 0, maxPWMLine);
    int rightSpeed = constrain((int)(baseSpeedLine - correction), 0, maxPWMLine);
    moveForward(rightSpeed, leftSpeed);
  }
  delay(5);
}

// ===================================================================================
// 10. HELPER FUNCTIONS
// ===================================================================================

// --- PID WALL ---
void PID_Logic_Wall(long dLeft, long dRight) {
  int leftSpeed, rightSpeed;
  bool leftValid = (dLeft > 0);
  bool rightValid = (dRight > 0);

  if (leftValid && rightValid && dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
    int error = (int)(dLeft - dRight);
    int correction = (int)(Kp_Wall * error);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
    rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
  }
  else if (leftValid && dLeft < 10) {
    int targetDist = 6;
    int error = (int)(targetDist - dLeft);
    int correction = (int)(Kp_Wall * error);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    leftSpeed  = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
    rightSpeed = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
  }
  else if (rightValid && dRight < 10) {
    int targetDist = 6;
    int error = (int)(targetDist - dRight);
    int correction = (int)(Kp_Wall * error);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
    rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
  }
  else {
    leftSpeed = BASE_PWM_STRAIGHT;
    rightSpeed = BASE_PWM_STRAIGHT;
  }
  moveForward(rightSpeed, leftSpeed);
  delay(10);
}

// --- RECORDING ---
void recordPath(char direction) {
  // Use the pointer to the active array (Small or Big)
  currentPathArr[*currentPathLenPtr] = direction; 
  (*currentPathLenPtr)++; 
  simplifyPath();
}

void simplifyPath() {
  int len = *currentPathLenPtr;
  if (len < 3 || currentPathArr[len-2] != 'B') return;
  
  char prev = currentPathArr[len-3];
  char curr = currentPathArr[len-1];
  char newMove = '?';

  // LBL -> S, LBS -> R, RBL -> B, SBL -> R, SBS -> B, LBR -> B
  if (prev == 'L' && curr == 'L') newMove = 'S';
  else if (prev == 'L' && curr == 'S') newMove = 'R';
  else if (prev == 'R' && curr == 'L') newMove = 'B';
  else if (prev == 'S' && curr == 'L') newMove = 'R';
  else if (prev == 'S' && curr == 'S') newMove = 'B';
  else if (prev == 'L' && curr == 'R') newMove = 'B';
  
  if (newMove != '?') { 
    currentPathArr[len-3] = newMove; 
    *currentPathLenPtr -= 2; 
  }
}

void printPaths() {
  Serial3.print("Path Small: ");
  for(int i=0; i<pathLenSmall; i++) Serial3.print(pathSmall[i]);
  Serial3.println();
  Serial3.print("Path Big: ");
  for(int i=0; i<pathLenBig; i++) Serial3.print(pathBig[i]);
  Serial3.println();
}

// --- MOVEMENT ---
void pivotTurn90(bool leftTurn, int pwmOuterMax) {
  int pwm = constrain(pwmOuterMax, 40, 255);
  encoderCountL = 0; encoderCountR = 0;
  
  if (leftTurn) {
    analogWrite(L_RPWM, 0);  analogWrite(L_LPWM, 0);
    analogWrite(R_RPWM, pwm);    analogWrite(R_LPWM, 0);
  } else {
    analogWrite(L_RPWM, 0);    analogWrite(L_LPWM, pwm);
    analogWrite(R_RPWM, 0);  analogWrite(R_LPWM, 0);
  }
  
  while (true) {
    long aL = absl(encoderCountL);
    long aR = absl(encoderCountR);
    if (aL >= TICKS_90_DEG || aR >= TICKS_90_DEG) break;
    delay(5);
  }
  moveForward(BASE_PWM_STRAIGHT, BASE_PWM_STRAIGHT);
  delay(50);
  stopMotors();
}

void pivot180(int pwmVal) {
  const int targetCounts = 550; 
  encoderCountL = 0; encoderCountR = 0;
  pivotRight(pwmVal);
  while(absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    if (absl(encoderCountL) >= targetCounts) stopLeft();
    if (absl(encoderCountR) >= targetCounts) stopRight();
    delay(5);
  }
  stopMotors();
}

void smoothTurnLeft()  { pivotTurn90(true, TURN_PWM); }
void smoothTurnRight() { pivotTurn90(false, TURN_PWM); }

void pivotLeft(int pwmVal) { analogWrite(R_RPWM, pwmVal); analogWrite(R_LPWM, 0); analogWrite(L_RPWM, pwmVal); analogWrite(L_LPWM, 0); }
void pivotRight(int pwmVal){ analogWrite(L_RPWM, 0);      analogWrite(L_LPWM, pwmVal); analogWrite(R_RPWM, 0);      analogWrite(R_LPWM, pwmVal); }

void moveForward(int pwmValR, int pwmValL) {
  analogWrite(R_RPWM, pwmValR); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);       analogWrite(L_LPWM, pwmValL);
}

void moveForwardDistance(int distance_cm, int pwmVal) {
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const long targetCounts = (long)(((float)distance_cm / wheelCircumference) * countsPerRev);
  noInterrupts(); encoderCountL = 0; encoderCountR = 0; interrupts();
  moveForward(pwmVal, pwmVal);
  while (true) {
    noInterrupts(); long cL = encoderCountL; long cR = encoderCountR; interrupts();
    if ((absl(cL) + absl(cR)) / 2 >= targetCounts) break;
    delay(10);
  }
  stopMotors();
}

// --- SENSORS & SYSTEM ---
void pivotLeftUntilCenter() {
  unsigned long t0 = millis();
  while (millis() - t0 < PIVOT_TIMEOUT_MS) {
    pivotLeft(TURN_PWM_LINE); readLineSensors();
    if (sensorVal[CENTER_IDX[0]] || sensorVal[CENTER_IDX[1]]) break;
    delay(4);
  }
  stopMotors();
}
void pivotRightUntilCenter() {
  unsigned long t0 = millis();
  while (millis() - t0 < PIVOT_TIMEOUT_MS) {
    pivotRight(TURN_PWM_LINE); readLineSensors();
    if (sensorVal[CENTER_IDX[0]] || sensorVal[CENTER_IDX[1]]) break;
    delay(4);
  }
  stopMotors();
}
void readLineSensors() {
  sensorBits = 0;
  for (int i = 0; i < 8; i++) {
    int raw = digitalRead(sensorPins[i]);
    bool isBlack = BLACK_IS_HIGH ? (raw == HIGH) : (raw == LOW);
    sensorVal[i] = isBlack ? 1 : 0;
    if (isBlack) sensorBits |= (1u << i);
  }
}
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); 
  if (duration == 0) return -1; 
  return duration * 0.034 / 2;
}
void stopMotors() { stopLeft(); stopRight(); }
void brake(uint16_t ms) { stopMotors(); delay(ms); }
void countEncoderL() { if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) encoderCountL++; else encoderCountL--; }
void countEncoderR() { if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) encoderCountR--; else encoderCountR++; }