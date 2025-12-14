#include <Arduino.h>
#include "RobotConfig.h"
#include "RobotState.h"

// Path memory (UNCHANGED sizes)
static char pathSmall[100];
static int  pathLenSmall = 0;

static char pathBig[100];
static int  pathLenBig = 0;

// Active pointers (like your original)
static char* currentPathArr = pathSmall;
static int*  currentPathLenPtr = &pathLenSmall;
static int   readIndex = 0;

static void simplifyPath() {
  int len = *currentPathLenPtr;
  if (len < 3 || currentPathArr[len-2] != 'B') return;

  char prev = currentPathArr[len-3];
  char curr = currentPathArr[len-1];
  char newMove = '?';

  if      (prev == 'L' && curr == 'L') newMove = 'S';
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

void mazeSolverBegin() {
  pathLenSmall = 0;
  pathLenBig = 0;
  currentPathArr = pathSmall;
  currentPathLenPtr = &pathLenSmall;
  readIndex = 0;
}

void mazeSolverSetActiveSmall() {
  currentPathArr = pathSmall;
  currentPathLenPtr = &pathLenSmall;
}

void mazeSolverSetActiveBig() {
  currentPathArr = pathBig;
  currentPathLenPtr = &pathLenBig;
}

void mazeSolverResetReadIndex() {
  readIndex = 0;
}

void mazeSolverRecord(char move) {
  if (*currentPathLenPtr >= 100) return; // safety
  currentPathArr[*currentPathLenPtr] = move;
  (*currentPathLenPtr)++;
  simplifyPath();
}

char mazeSolverGetNextMove(bool* hasMove) {
  if (readIndex < *currentPathLenPtr) {
    *hasMove = true;
    return currentPathArr[readIndex++];
  }
  *hasMove = false;
  return '?';
}

void mazeSolverPrintPaths() {
  DBG_PORT.print("Path Small: ");
  for (int i=0;i<pathLenSmall;i++) DBG_PORT.print(pathSmall[i]);
  DBG_PORT.println();

  DBG_PORT.print("Path Big: ");
  for (int i=0;i<pathLenBig;i++) DBG_PORT.print(pathBig[i]);
  DBG_PORT.println();
}
