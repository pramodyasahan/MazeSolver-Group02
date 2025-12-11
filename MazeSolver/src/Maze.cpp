// Maze.cpp
#include "Maze.h"

// internal flag
static bool mazeFinishedFlag = false;

// Forward declarations
static bool detectMazeWhiteBox();
static void mazeMappingStep();
static void mazeSolveStep();
static void keepCentered();

// ============================================================================

void initMaze() {
  mazeFinishedFlag = false;
}

// Detects large white area using line sensors (same idea as your code)
static bool detectMazeWhiteBox() {
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) {
    int val = digitalRead(LINE_SENSOR_PINS[i]);
    // assuming LOW = white (depends on your board)
    if (val == LOW) whiteCount++;
  }
  return (whiteCount >= WHITE_COUNT_THRESHOLD);
}

bool mazeReachedEnd() {
  if (mazeFinishedFlag) {
    mazeFinishedFlag = false;  // one-shot
    return true;
  }
  return false;
}

// ============================================================================
// Mapping: Left-hand rule with path recording + simplification
// ============================================================================
static void mazeMappingStep() {
  if (detectMazeWhiteBox()) {
    stopMotors();
    mazeFinishedFlag = true;
    return;
  }

  long dFront = readFrontDistance();
  long dLeft  = readLeftDistance();
  long dRight = readRightDistance();

  bool leftOpen   = (dLeft  > OPEN_SPACE_THRESHOLD_CM);
  bool frontOpen  = (dFront > OBSTACLE_THRESHOLD);
  bool rightOpen  = (dRight > OPEN_SPACE_THRESHOLD_CM);

  // Dead end: no openings
  if (!leftOpen && !frontOpen && !rightOpen) {
    stopMotors();
    delay(100);
    pivot180(PIVOT_180_PWM);
    recordMove('B');
    return;
  }

  // Priority: Left > Straight > Right (left-hand rule)
  if (leftOpen) {
    // small forward to be nicely in intersection, then left
    moveForwardDistance(5, BASE_PWM_STRAIGHT);
    stopMotors();
    delay(100);
    smoothTurnLeft();
    recordMove('L');
    moveForwardDistance(5, BASE_PWM_STRAIGHT);
  }
  else if (frontOpen) {
    // Straight: either record 'S' at intersections, or just keep centered
    recordMove('S');
    keepCentered();
  }
  else if (rightOpen) {
    stopMotors();
    delay(100);
    smoothTurnRight();
    recordMove('R');
    moveForwardDistance(5, BASE_PWM_STRAIGHT);
  }
}

// ============================================================================
// Solving: follow stored path using shortest version from Path.cpp
// ============================================================================
static void mazeSolveStep() {
  if (detectMazeWhiteBox()) {
    stopMotors();
    mazeFinishedFlag = true;
    return;
  }

  if (pathFinished()) {
    // path done, just keep centered until box detected
    keepCentered();
    return;
  }

  long dFront = readFrontDistance();
  bool frontBlocked = (dFront > 0 && dFront < FRONT_WALL_TRIGGER);

  // If at a close wall or intersection, execute next path move
  // (For simplicity, we don't attempt super precise intersection alignment here.)
  char move = getNextMove();
  Serial3.print("BT: Move -> ");
  Serial3.println(move);

  if (move == 'L') {
    smoothTurnLeft();
    moveForwardDistance(8, BASE_PWM_STRAIGHT);
  } else if (move == 'R') {
    smoothTurnRight();
    moveForwardDistance(8, BASE_PWM_STRAIGHT);
  } else if (move == 'S') {
    if (!frontBlocked) {
      moveForwardDistance(15, BASE_PWM_STRAIGHT);
    } else {
      Serial3.println("BT: Blocked! Skipping S move.");
    }
  } else if (move == 'B') {
    pivot180(PIVOT_180_PWM);
    moveForwardDistance(8, BASE_PWM_STRAIGHT);
  }
}

// ============================================================================
// Centering between walls (similar to your keepCentered())
// ============================================================================
static void keepCentered() {
  long dFront = readFrontDistance();
  long dLeft  = readLeftDistance();
  long dRight = readRightDistance();

  if (dFront > 0 && dFront < 5) {
    stopMotors();
    return;
  }

  int correction = 0;

  if (dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
    int error = (int)(dLeft - dRight);
    correction = (int)(error * Kp_Solve);
  } else if (dLeft < WALL_DETECT_RANGE) {
    int error = (int)(dLeft - TARGET_WALL_DIST);
    correction = (int)(error * Kp_Solve);
  } else if (dRight < WALL_DETECT_RANGE) {
    int error = (int)(TARGET_WALL_DIST - dRight);
    correction = (int)(error * Kp_Solve);
  }

  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  int pwmR = BASE_PWM_STRAIGHT + correction;
  int pwmL = BASE_PWM_STRAIGHT - correction;
  moveForward(pwmR, pwmL);
}

// ============================================================================
// Public API
// ============================================================================
void mazeUpdate(MazeMode mode) {
  switch (mode) {
    case MAZE_MODE_MAPPING:
      mazeMappingStep();
      break;
    case MAZE_MODE_SOLVING:
      mazeSolveStep();
      break;
  }
}
