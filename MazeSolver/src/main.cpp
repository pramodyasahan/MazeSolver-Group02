// src/main.cpp
#include <Arduino.h>
#include "RobotConfig.h"
#include "RobotState.h"

// =======================================================
// Transition tuning (TUNABLE)
// =======================================================

// Maze1 exit (white box)
static const uint8_t  WHITE_MIN        = 6;      // >=6/8 sensors white => white box
static const uint16_t WHITE_CONFIRM_MS = 120;    // debounce white box

// Maze1 exit STOP before line-follow (requested)
static const uint16_t MAZE1_EXIT_STOP_MS = 1500; // stop before switching to line follow

// Line reacquire during TRANS_TO_LINE
static const uint16_t LINE_CONFIRM_MS  = 60;     // debounce line reacquire
static const uint8_t  LINE_BLACK_MAX   = 6;      // avoid all-black as "line"

// Line -> BigMaze detection (NEW FIX)
static const uint8_t  FULL_BLACK_COUNT       = 8;     // 8/8 black
static const uint16_t FULL_BLACK_CONFIRM_MS  = 150;   // debounce full black
static const uint16_t LINE_TO_MAZE_STOP_MS   = 1500;  // stop before switching to wall-follow

// Optional: ultrasonic confirm while in TRANS_TO_MAZE2 (kept as extra safety)
static const int      WALL_RANGE_CM    = 20;     // wall seen when < 20 cm
static const uint16_t WALL_CONFIRM_MS  = 120;    // debounce wall detection

// Transition forward push
static const int      TRANS_FWD_PWM    = 55;     // gentle push
static const uint16_t TRANS_FWD_MS     = 350;    // push duration

// Safety timeout so we don't creep forever in TRANS_TO_MAZE2
static const uint16_t TRANS_TO_MAZE2_TIMEOUT_MS = 2500;

// Debug throttle
static const uint16_t DBG_TRANS_MS = 180;

// =======================================================
// State machine
// =======================================================
static RobotState currentState = STATE_MAPPING_SMALL;
static bool solveRunActive = false;  // false for mapping run, true for solve run

// Debounce timers
static unsigned long whiteStartMs = 0;
static unsigned long lineStartMs  = 0;
static unsigned long wallStartMs  = 0;
static unsigned long fullBlackStartMs = 0;

// Transition state timer
static unsigned long transStartMs = 0;

// Stop-hold timers
static unsigned long maze1StopStartMs = 0;
static unsigned long lineMazeStopStartMs = 0;

// Debug timing
static unsigned long lastDbgMs = 0;

// =======================================================
// Motor helpers (kept local to main for transitions)
// =======================================================
static inline void motorsStop() {
  analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0);
}

static inline void motorsForward(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(R_RPWM, pwm); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);   analogWrite(L_LPWM, pwm);
}

// =======================================================
// Ultrasonic helper
// =======================================================
static long readUltrasonicCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return -1;
  return (long)(duration * 0.034 / 2.0);
}

// =======================================================
// Line summary helper
// =======================================================
struct LineSummary {
  uint8_t whiteCount;
  uint8_t blackCount;
  bool centerBlack;
  uint8_t bits;
};

static LineSummary readLineSummary() {
  LineSummary ls{};
  ls.whiteCount = 0;
  ls.blackCount = 0;
  ls.centerBlack = false;
  ls.bits = 0;

  for (int i = 0; i < 8; i++) {
    int raw = digitalRead(LINE_SENSOR_PINS[i]);
    bool isBlack = BLACK_IS_HIGH ? (raw == HIGH) : (raw == LOW);

    if (isBlack) {
      ls.blackCount++;
      ls.bits |= (1u << i);
    } else {
      ls.whiteCount++;
    }
  }

  bool c3 = (ls.bits & (1u << 3)) != 0;
  bool c4 = (ls.bits & (1u << 4)) != 0;
  ls.centerBlack = (c3 || c4);

  return ls;
}

// =======================================================
// Debounce helpers
// =======================================================
static bool debounceHold(bool cond, unsigned long &startMs, uint16_t confirmMs) {
  if (cond) {
    if (startMs == 0) startMs = millis();
    if (millis() - startMs >= confirmMs) return true;
  } else {
    startMs = 0;
  }
  return false;
}

static void resetDebounces() {
  whiteStartMs = 0;
  lineStartMs  = 0;
  wallStartMs  = 0;
  fullBlackStartMs = 0;
}

// =======================================================
// Debug helpers
// =======================================================
static const char* stateName(RobotState s) {
  switch (s) {
    case STATE_MAPPING_SMALL: return "MAP_SMALL";
    case STATE_TRANS_TO_LINE: return "TRANS_TO_LINE";
    case STATE_LINE_FOLLOW: return "LINE_FOLLOW";
    case STATE_TRANS_TO_MAZE2: return "TRANS_TO_MAZE2";
    case STATE_MAPPING_BIG: return "MAP_BIG";
    case STATE_WAIT_FOR_RUN2: return "WAIT_RUN2";
    case STATE_SOLVING_SMALL: return "SOLVE_SMALL";
    case STATE_SOLVING_LINE: return "SOLVE_LINE";
    case STATE_SOLVING_BIG: return "SOLVE_BIG";
    case STATE_DONE: return "DONE";
    default: return "?";
  }
}

static void dbgTransitionSensors(const LineSummary& ls, bool whiteBox, bool lineFound, bool fullBlack, bool wallsDetected) {
  if (millis() - lastDbgMs < DBG_TRANS_MS) return;
  lastDbgMs = millis();

  DBG_PORT.print("DBG t="); DBG_PORT.print(millis());
  DBG_PORT.print(" st="); DBG_PORT.print(stateName(currentState));
  DBG_PORT.print(" bits=0x"); DBG_PORT.print(ls.bits, HEX);
  DBG_PORT.print(" W="); DBG_PORT.print(ls.whiteCount);
  DBG_PORT.print(" B="); DBG_PORT.print(ls.blackCount);
  DBG_PORT.print(" C="); DBG_PORT.print((int)ls.centerBlack);
  DBG_PORT.print(" whiteBox="); DBG_PORT.print((int)whiteBox);
  DBG_PORT.print(" lineFound="); DBG_PORT.print((int)lineFound);
  DBG_PORT.print(" fullBlack="); DBG_PORT.print((int)fullBlack);
  DBG_PORT.print(" walls="); DBG_PORT.println((int)wallsDetected);
}

// =======================================================
// setup
// =======================================================
void setup() {
  Serial.begin(9600);
  DBG_PORT.begin(DBG_BAUD);
  delay(150);

  DBG_PORT.println("========================================");
  DBG_PORT.println("BT: main.cpp (Maze1 -> Line -> Maze2) transitions + solve run");
  DBG_PORT.println("Fix: Line->Maze2 uses FULL BLACK only (debounced) + STOP-HOLD");
  DBG_PORT.println("========================================");

  // Ultrasonics
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // Motors
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);

  // Encoders (ISRs are in WallFollower.cpp)
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), countEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), countEncoderR, CHANGE);

  // Start pin
  pinMode(SEARCH_MODE, INPUT_PULLUP);

  // Line sensors
  for (int i = 0; i < 8; i++) pinMode(LINE_SENSOR_PINS[i], INPUT);

  // Modules
  mazeSolverBegin();
  wallFollowerBegin();
  lineFollowerBegin();

  // Start in mapping small
  mazeSolverSetActiveSmall();
  solveRunActive = false;
  currentState = STATE_MAPPING_SMALL;

  motorsStop();
  resetDebounces();

  DBG_PORT.println("BT: Ready (Mapping run starts in Small Maze)");
}

// =======================================================
// loop
// =======================================================
void loop() {
  LineSummary ls = readLineSummary();

  // Maze1 white-box (only relevant in maze states)
  const bool whiteBoxNow = (ls.whiteCount >= WHITE_MIN);
  const bool whiteBox    = debounceHold(whiteBoxNow, whiteStartMs, WHITE_CONFIRM_MS);

  // Line reacquire (only relevant in TRANS_TO_LINE)
  const bool lineFoundNow = (ls.centerBlack && ls.blackCount <= LINE_BLACK_MAX);
  const bool lineFound    = debounceHold(lineFoundNow, lineStartMs, LINE_CONFIRM_MS);

  // Full black detection (line -> big maze trigger)
  const bool fullBlackNow = (ls.blackCount == FULL_BLACK_COUNT);
  const bool fullBlack    = debounceHold(fullBlackNow, fullBlackStartMs, FULL_BLACK_CONFIRM_MS);

  // Optional ultrasonic confirm only while TRANS_TO_MAZE2
  bool wallsDetected = false;
  long dL = -1, dR = -1;
  if (currentState == STATE_TRANS_TO_MAZE2) {
    dL = readUltrasonicCm(TRIG_LEFT,  ECHO_LEFT);
    dR = readUltrasonicCm(TRIG_RIGHT, ECHO_RIGHT);
    bool wallsNow = ((dL > 0 && dL < WALL_RANGE_CM) || (dR > 0 && dR < WALL_RANGE_CM));
    wallsDetected = debounceHold(wallsNow, wallStartMs, WALL_CONFIRM_MS);
  } else {
    wallStartMs = 0;
  }

  // Live debug during transitions + line states (throttled)
  if (currentState == STATE_TRANS_TO_LINE ||
      currentState == STATE_TRANS_TO_MAZE2 ||
      currentState == STATE_LINE_FOLLOW ||
      currentState == STATE_SOLVING_LINE) {
    dbgTransitionSensors(ls, whiteBox, lineFound, fullBlack, wallsDetected);
  }

  switch (currentState) {

    // ---------------------------
    // Maze 1 mapping
    // ---------------------------
    case STATE_MAPPING_SMALL: {
      // Maze1 -> Line trigger: WHITE BOX, but STOP-HOLD first
      if (whiteBox) {
        if (maze1StopStartMs == 0) {
          maze1StopStartMs = millis();
          motorsStop();
          DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
          DBG_PORT.println(" Maze1 exit WHITE detected -> STOP-HOLD before TRANS_TO_LINE");
        }

        motorsStop();

        if (millis() - lastDbgMs > 300) {
          lastDbgMs = millis();
          DBG_PORT.print("HOLD maze1->line remaining(ms)=");
          DBG_PORT.println((long)MAZE1_EXIT_STOP_MS - (long)(millis() - maze1StopStartMs));
        }

        if (millis() - maze1StopStartMs >= MAZE1_EXIT_STOP_MS) {
          DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
          DBG_PORT.println(" STOP-HOLD done -> TRANS_TO_LINE");
          maze1StopStartMs = 0;

          motorsStop();
          transStartMs = millis();
          resetDebounces();
          currentState = STATE_TRANS_TO_LINE;
        }
        break;
      } else {
        maze1StopStartMs = 0;
      }

      wallFollowerMappingUpdate();
      break;
    }

    // ---------------------------
    // Transition: Maze -> Line
    // ---------------------------
    case STATE_TRANS_TO_LINE: {
      if (millis() - transStartMs < TRANS_FWD_MS) {
        motorsForward(TRANS_FWD_PWM);
        break;
      }

      if (lineFound) {
        DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
        DBG_PORT.println(" Line acquired -> LineFollow");

        motorsStop();
        resetDebounces();
        currentState = solveRunActive ? STATE_SOLVING_LINE : STATE_LINE_FOLLOW;
      } else {
        motorsForward(TRANS_FWD_PWM - 10);
      }
      break;
    }

    // ---------------------------
    // Line follow (mapping run)
    // FIX: do NOT use whiteBox here. Use FULL BLACK to exit line.
    // ---------------------------
    case STATE_LINE_FOLLOW: {
      // Detect entry to big maze floor (FULL BLACK) -> STOP-HOLD -> TRANS_TO_MAZE2
      if (fullBlack) {
        if (lineMazeStopStartMs == 0) {
          lineMazeStopStartMs = millis();
          motorsStop();
          DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
          DBG_PORT.println(" FULL BLACK detected (line->maze) -> STOP-HOLD");
        }

        motorsStop();

        if (millis() - lastDbgMs > 300) {
          lastDbgMs = millis();
          DBG_PORT.print("HOLD line->maze remaining(ms)=");
          DBG_PORT.println((long)LINE_TO_MAZE_STOP_MS - (long)(millis() - lineMazeStopStartMs));
        }

        if (millis() - lineMazeStopStartMs >= LINE_TO_MAZE_STOP_MS) {
          DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
          DBG_PORT.println(" STOP-HOLD done -> TRANS_TO_MAZE2");

          lineMazeStopStartMs = 0;
          motorsStop();
          transStartMs = millis();
          resetDebounces();
          currentState = STATE_TRANS_TO_MAZE2;
        }
        break;
      } else {
        lineMazeStopStartMs = 0; // reset if fullBlack disappears
      }

      lineFollowerUpdate(true);
      break;
    }

    // ---------------------------
    // Transition: Line -> Maze2
    // (kept: push forward, then (optional) ultrasonic confirm or timeout)
    // ---------------------------
    case STATE_TRANS_TO_MAZE2: {
      if (millis() - transStartMs < TRANS_FWD_MS) {
        motorsForward(TRANS_FWD_PWM);
        break;
      }

      if (wallsDetected) {
        DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
        DBG_PORT.print(" Walls confirmed (dL="); DBG_PORT.print(dL);
        DBG_PORT.print(" dR="); DBG_PORT.print(dR);
        DBG_PORT.println(") -> Maze2");

        motorsStop();
        resetDebounces();

        mazeSolverSetActiveBig();
        if (solveRunActive) mazeSolverResetReadIndex();

        currentState = solveRunActive ? STATE_SOLVING_BIG : STATE_MAPPING_BIG;
        break;
      }

      if (millis() - transStartMs > TRANS_TO_MAZE2_TIMEOUT_MS) {
        DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
        DBG_PORT.println(" Maze2 wall confirm TIMEOUT -> entering Maze2 anyway");

        motorsStop();
        resetDebounces();

        mazeSolverSetActiveBig();
        if (solveRunActive) mazeSolverResetReadIndex();

        currentState = solveRunActive ? STATE_SOLVING_BIG : STATE_MAPPING_BIG;
        break;
      }

      motorsForward(TRANS_FWD_PWM - 10);
      break;
    }

    // ---------------------------
    // Maze 2 mapping
    // ---------------------------
    case STATE_MAPPING_BIG: {
      // end-of-maze uses your existing white-box finish logic
      if (whiteBox) {
        DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
        DBG_PORT.println(" WHITE BOX (Maze2 end) -> WAIT_FOR_RUN2");

        motorsStop();
        resetDebounces();
        currentState = STATE_WAIT_FOR_RUN2;
        break;
      }

      wallFollowerMappingUpdate();
      break;
    }

    // ---------------------------
    // Wait for solve run start
    // ---------------------------
    case STATE_WAIT_FOR_RUN2: {
      motorsStop();

      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 2000) {
        DBG_PORT.println("BT: Waiting for Pin 33 (LOW) to start SOLVE RUN...");
        mazeSolverPrintPaths();
        lastPrint = millis();
      }

      if (digitalRead(SEARCH_MODE) == LOW) {
        delay(1000);
        DBG_PORT.println("BT: START SOLVE RUN!");
        solveRunActive = true;

        mazeSolverSetActiveSmall();
        mazeSolverResetReadIndex();

        resetDebounces();
        currentState = STATE_SOLVING_SMALL;
      }
      break;
    }

    // ---------------------------
    // Solve run: small maze
    // ---------------------------
    case STATE_SOLVING_SMALL: {
      if (whiteBox) {
        DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
        DBG_PORT.println(" WHITE BOX (Small solved) -> TRANS_TO_LINE");

        motorsStop();
        transStartMs = millis();
        resetDebounces();
        currentState = STATE_TRANS_TO_LINE;
        break;
      }

      wallFollowerSolvingUpdate();
      break;
    }

    // ---------------------------
    // Solve run: line
    // FIX: exit line on FULL BLACK (not white)
    // ---------------------------
    case STATE_SOLVING_LINE: {
      if (fullBlack) {
        DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
        DBG_PORT.println(" FULL BLACK (solve line->maze) -> STOP-HOLD");

        // optional hold here too
        if (lineMazeStopStartMs == 0) lineMazeStopStartMs = millis();
        motorsStop();

        if (millis() - lineMazeStopStartMs >= LINE_TO_MAZE_STOP_MS) {
          DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
          DBG_PORT.println(" STOP-HOLD done -> TRANS_TO_MAZE2");

          lineMazeStopStartMs = 0;
          transStartMs = millis();
          resetDebounces();
          currentState = STATE_TRANS_TO_MAZE2;
        }
        break;
      } else {
        lineMazeStopStartMs = 0;
      }

      lineFollowerUpdate(false);
      break;
    }

    // ---------------------------
    // Solve run: big maze
    // ---------------------------
    case STATE_SOLVING_BIG: {
      if (whiteBox) {
        DBG_PORT.print("EVT t="); DBG_PORT.print(millis());
        DBG_PORT.println(" WHITE BOX (Big solved) -> DONE");

        motorsStop();
        currentState = STATE_DONE;
        break;
      }

      wallFollowerSolvingUpdate();
      break;
    }

    case STATE_DONE:
    default:
      motorsStop();
      break;
  }
}
