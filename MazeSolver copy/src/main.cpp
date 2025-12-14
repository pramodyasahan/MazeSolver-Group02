#include <Arduino.h>

// ===================================================================================
// 1. PIN DEFINITIONS & HARDWARE SETTINGS
// ===================================================================================

// --- Ultrasonic Sensors ---
#define TRIG_FRONT 48
#define ECHO_FRONT 49
#define TRIG_RIGHT 50
#define ECHO_RIGHT 51
#define TRIG_LEFT  52
#define ECHO_LEFT  53

// --- Motor Driver ---
#define L_RPWM 4
#define L_LPWM 5
#define L_REN  6
#define L_LEN  7

#define R_RPWM 2
#define R_LPWM 3
#define R_REN  9
#define R_LEN  8

// --- Encoders ---
#define L_ENC_A 18
#define L_ENC_B 19
#define R_ENC_A 20
#define R_ENC_B 21

// --- Inputs ---
#define SEARCH_MODE 33 

// --- Line Sensor Settings ---
const uint8_t sensorPins[8] = {22, 23, 24, 25, 26, 27, 28, 29};
const bool BLACK_IS_HIGH = true; 

// ===================================================================================
// 2. CONSTANTS & TUNING
// ===================================================================================

const float WHEEL_DIAMETER_CM = 6.5f;
const int pulsesPerMotorRev   = 11;
const int gearRatio           = 20;
const int countsPerRev        = pulsesPerMotorRev * gearRatio * 2;

// --- Speed Settings ---
const int BASE_PWM_STRAIGHT = 65; 
const int TURN_PWM          = 55;   
const int PIVOT_180_PWM     = 60;   
const int ALIGN_PWM         = 45;
const int TURN_PWM_LINE     = 140; 

// --- Wall Following Constants ---
const float Kp_Wall = 3.5f;   
const float Kd_Wall = 10.0f; 
const int OBSTACLE_THRESHOLD      = 9;  
const int NO_WALL_THRESHOLD       = 25; 
const int WALL_DETECT_RANGE       = 20;  
const int TARGET_WALL_DIST        = 7;   
const int CORNER_CLEARANCE_CM     = 10;  
const int ALIGN_TOLERANCE_CM      = 1;   

// --- Line Following PID ---
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
// 3. GLOBALS
// ===================================================================================
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

char path[200];       
int pathLength = 0;   
int readIndex = 0;    

uint8_t sensorVal[8];
uint8_t sensorBits = 0;

float lastErrorWall = 0;
float lastErrorLine = 0;
float integralLine = 0;

// Timer for safety delays
unsigned long modeStartTime = 0;
unsigned long lastDebugTime = 0; // For printing intervals


enum RobotState {
  STATE_MAPPING_1,      
  STATE_WAIT_1,         
  STATE_SOLVING_1,      
  STATE_LINE_FOLLOW,    
  STATE_MAPPING_2,      
  STATE_WAIT_2,         
  STATE_SOLVING_2,      
  STATE_DONE            
};

RobotState currentState = STATE_MAPPING_1;

// ===================================================================================
// 4. FUNCTION PROTOTYPES
// ===================================================================================
void stopMotors(); 
long readUltrasonic(int trigPin, int echoPin);
void readLineSensors();
void countEncoderL(); void countEncoderR();

void moveForward(int pwmValR, int pwmValL);
void moveForwardDistance(int distance_cm, int pwmVal);
void pivotTurn90(bool leftTurn, int pwmOuterMax);
void pivot180(int pwmVal);
void pivotLeft(int pwmVal); void pivotRight(int pwmVal);
void pivotLeftUntilCenter(); void pivotRightUntilCenter();
void pivotLeft90UntilLine(int pwmVal); void pivotRight90UntilLine(int pwmVal);

void logicMapping();
void logicSolving();
void logicLineFollow();
void driveStraightPID_Wall(long dLeft, long dRight);
void recordPath(char direction);
void simplifyPath();
void printPathToBluetooth();
void brake(uint16_t ms);
void printIRValues(); // NEW DEBUG FUNCTION


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

  stopMotors();
  Serial.println("BT: Robot Ready. Mapping 1...");
  modeStartTime = millis();
}

// ===================================================================================
// 6. MAIN LOOP
// ===================================================================================
void loop() {
  switch (currentState) {
    case STATE_MAPPING_1:
      logicMapping(); 
      break;

    case STATE_WAIT_1:
      stopMotors();
      if (digitalRead(SEARCH_MODE) == LOW) {
        delay(1000); 
        Serial.println("BT: Solving 1...");
        modeStartTime = millis(); // Reset timer for safety
        currentState = STATE_SOLVING_1;
      }
      break;

    case STATE_SOLVING_1:
      logicSolving();
      break;

    case STATE_LINE_FOLLOW:
      logicLineFollow();
      break;

    case STATE_MAPPING_2:
      logicMapping();
      break;

    case STATE_WAIT_2:
      stopMotors();
      if (digitalRead(SEARCH_MODE) == LOW) {
        delay(1000);
        Serial.println("BT: Solving 2...");
        modeStartTime = millis(); // Reset timer for safety
        currentState = STATE_SOLVING_2;
      }
      break;

    case STATE_SOLVING_2:
      logicSolving();
      break;

    case STATE_DONE:
      stopMotors();
      // Halt forever
      break;
  }
}

// ===================================================================================
// 7. MAZE MAPPING LOGIC
// ===================================================================================
void logicMapping() {
  // 1. FINISH CHECK (Immediate Stop on White)
  readLineSensors();
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) if (sensorVal[i] == 0) whiteCount++; 

  if (whiteCount >= 4) {
    stopMotors(); 
    
    if (currentState == STATE_MAPPING_1) {
       Serial.println("BT: Map 1 Done. Waiting...");
       printPathToBluetooth();
       currentState = STATE_WAIT_1;
    } 
    else if (currentState == STATE_MAPPING_2) {
       Serial.println("BT: Map 2 Done. Waiting...");
       printPathToBluetooth();
       currentState = STATE_WAIT_2;
    }
    delay(1000); 
    return;
  }

  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool frontBlocked = (dFront > 0 && dFront < OBSTACLE_THRESHOLD);
  bool leftOpen     = (dLeft > NO_WALL_THRESHOLD);
  bool rightOpen    = (dRight > NO_WALL_THRESHOLD);

  if (frontBlocked && !leftOpen && !rightOpen) {
    stopMotors(); delay(100);
    pivot180(PIVOT_180_PWM);
    recordPath('B');
    return;
  }

  if (frontBlocked) {
    stopMotors(); delay(100);
    if (dLeft > 0 && dRight > 0 && abs(dLeft - dRight) > ALIGN_TOLERANCE_CM && dLeft < 20 && dRight < 20) {
      if (dLeft < dRight) pivotLeft(ALIGN_PWM); else pivotRight(ALIGN_PWM);
      delay(80); stopMotors();
    }
    if (leftOpen) { pivotTurn90(true, TURN_PWM); recordPath('L'); }
    else          { pivotTurn90(false, TURN_PWM); recordPath('R'); }
    moveForwardDistance(5, BASE_PWM_STRAIGHT);
    return;
  }

  if (leftOpen) {
    moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
    stopMotors(); delay(100);
    pivotTurn90(true, TURN_PWM);
    if (dFront > OBSTACLE_THRESHOLD || rightOpen) recordPath('L');
    else if (pathLength > 0 && path[pathLength-1] == 'B') recordPath('L');
    moveForwardDistance(5, BASE_PWM_STRAIGHT);
    return;
  }

  driveStraightPID_Wall(dLeft, dRight);
}

// ===================================================================================
// 8. MAZE SOLVING LOGIC
// ===================================================================================
void logicSolving() {
  // 1. FINISH CHECK
  // SAFETY: Don't check finish for first 1.5 seconds to get off start tile
  if (millis() - modeStartTime > 1500) {
    readLineSensors();
    int whiteCount = 0;
    for (int i = 0; i < 8; i++) if (sensorVal[i] == 0) whiteCount++;

    if (whiteCount >= 4) {
      stopMotors(); 

      if (currentState == STATE_SOLVING_1) {
        Serial.println("BT: Solve 1 Done -> Line Follow");
        printIRValues();
        // Move slightly to catch line if needed, but not too much
        moveForwardDistance(5, BASE_PWM_STRAIGHT); 
        modeStartTime = millis(); // Reset timer for Line Follow Safety
        currentState = STATE_LINE_FOLLOW;
      }
      else if (currentState == STATE_SOLVING_2) {
        Serial.println("BT: Solve 2 Done -> COMPLETE");
        currentState = STATE_DONE;
      }
      return;
    }
  }

  // 2. EXECUTE PATH
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool leftOpen = (dLeft > NO_WALL_THRESHOLD);
  bool rightOpen = (dRight > NO_WALL_THRESHOLD);
  bool frontBlocked = (dFront > 0 && dFront < OBSTACLE_THRESHOLD);

  if (leftOpen || rightOpen || frontBlocked) {
    stopMotors(); delay(100);
    
    if (frontBlocked && dLeft > 0 && dRight > 0 && abs(dLeft - dRight) > ALIGN_TOLERANCE_CM) {
       if (dLeft < dRight) pivotLeft(ALIGN_PWM); else pivotRight(ALIGN_PWM);
       delay(80); stopMotors();
    }

    if (readIndex < pathLength) {
      char move = path[readIndex];
      readIndex++;
      Serial.print("BT: Move "); Serial3.println(move);

      if (move == 'L') {
        moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT); 
        pivotTurn90(true, TURN_PWM);
        moveForwardDistance(5, BASE_PWM_STRAIGHT); 
      } 
      else if (move == 'R') {
        moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT); 
        pivotTurn90(false, TURN_PWM);
        moveForwardDistance(5, BASE_PWM_STRAIGHT); 
      } 
      else if (move == 'S') {
        moveForwardDistance(15, BASE_PWM_STRAIGHT);
      }
    } else {
      driveStraightPID_Wall(dLeft, dRight);
    }
  } else {
    driveStraightPID_Wall(dLeft, dRight);
  }
}

// ===================================================================================
// 9. LINE FOLLOWING LOGIC
// ===================================================================================
void logicLineFollow() {
  readLineSensors();

  // 1. BLACK STOP CONDITION
  // SAFETY: Ignore stop condition for first 3 seconds to ensure we are following line
  if (millis() - modeStartTime > 3000) {

    int blackCount = 0;
    for(int i=0; i<8; i++) if(sensorVal[i] == 1) blackCount++;

    if (blackCount >= 5) {
      stopMotors();
      Serial.println("BT: End of Line. Resetting Map 2...");
      printIRValues();
      delay(1000); 
      
      // Reset Path Memory for Maze 2
      pathLength = 0;
      readIndex = 0;
      
      currentState = STATE_MAPPING_2;
      return;
    }
  }

  // 2. INTERSECTIONS
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
       // Only search if we've actually started moving (after 1s)
       if(millis() - modeStartTime > 1000) pivotLeft90UntilLine(50);
       else moveForward(baseSpeedLine, baseSpeedLine); // Blind forward at start
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
void driveStraightPID_Wall(long dLeft, long dRight) {
  float error = 0;
  int base = BASE_PWM_STRAIGHT;

  if (dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) error = dLeft - dRight; 
  else if (dLeft < WALL_DETECT_RANGE) error = dLeft - TARGET_WALL_DIST;
  else if (dRight < WALL_DETECT_RANGE) error = TARGET_WALL_DIST - dRight;
  else {
    moveForward(base, base);
    lastErrorWall = 0;
    return;
  }

  float derivative = error - lastErrorWall;
  int correction = (int)((error * Kp_Wall) + (derivative * Kd_Wall));
  if (correction > 30) correction = 30;
  if (correction < -30) correction = -30;
  lastErrorWall = error;
  moveForward(base + correction, base - correction);
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

void pivotLeft90UntilLine(int pwmVal) {
  const long targetCounts = 560; 
  noInterrupts(); encoderCountL = 0; encoderCountR = 0; interrupts();
  pivotLeft(pwmVal);
  while (absl(encoderCountL) < targetCounts) {
    readLineSensors(); if (sensorBits != 0) break; delay(5);
  }
  stopMotors();
}

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
    delay(5);
  }
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

void pivotTurn90(bool leftTurn, int pwmOuterMax) {
  const int TICKS_90_DEG = 560;   
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
  stopMotors();
}

void pivotLeft(int pwmVal) { analogWrite(R_RPWM, pwmVal); analogWrite(R_LPWM, 0); analogWrite(L_RPWM, pwmVal); analogWrite(L_LPWM, 0); }
void pivotRight(int pwmVal){ analogWrite(L_RPWM, 0);      analogWrite(L_LPWM, pwmVal); analogWrite(R_RPWM, 0);      analogWrite(R_LPWM, pwmVal); }
void stopMotors() { stopLeft(); stopRight(); }
void brake(uint16_t ms) { stopMotors(); delay(ms); }

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 4000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}

void recordPath(char direction) {
  path[pathLength] = direction; pathLength++; simplifyPath();
}
void simplifyPath() {
  if (pathLength < 3 || path[pathLength-2] != 'B') return;
  char prev = path[pathLength-3], curr = path[pathLength-1], newMove = '?';
  if (prev == 'L' && curr == 'L') newMove = 'S';
  else if (prev == 'L' && curr == 'S') newMove = 'R';
  else if (prev == 'R' && curr == 'L') newMove = 'B';
  else if (prev == 'S' && curr == 'L') newMove = 'R';
  else if (prev == 'S' && curr == 'S') newMove = 'B';
  else if (prev == 'L' && curr == 'R') newMove = 'B';
  if (newMove != '?') { path[pathLength-3] = newMove; pathLength -= 2; }
}
void printPathToBluetooth() {
  Serial.println("MAP DONE. PATH:");
  for(int i=0; i<pathLength; i++) Serial3.print(path[i]); 
  Serial.println();
}

void printIRValues() {
  // Only print every 100ms to avoid slowing down the robot via Serial lag
  if (millis() - lastDebugTime > 100) {
    lastDebugTime = millis();
    
    int wCount = 0;
    String irStatus = "";
    
    for (int i = 0; i < 8; i++) {
      int val = digitalRead(sensorPins[i]);
      irStatus += String(val) + " ";
      if (val == LOW) wCount++; // Assuming LOW is White
    }
    
    // Print to Bluetooth for troubleshooting
    Serial.print("IR: [");
    Serial.print(irStatus);
    Serial.print("] Count: ");
    Serial.println(wCount);
  }
}


void countEncoderL() { if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) encoderCountL++; else encoderCountL--; }
void countEncoderR() { if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) encoderCountR--; else encoderCountR++;}
