// Path.cpp
#include "Path.h"

static char path[100];
static int pathLength = 0;
static int readIndex  = 0;

void resetPath() {
  pathLength = 0;
  readIndex  = 0;
}

void simplifyPath() {
  // Requires at least 3 moves: X, 'B', Y
  if (pathLength < 3 || path[pathLength - 2] != 'B') return;

  char prev = path[pathLength - 3];
  char curr = path[pathLength - 1];
  char newMove = '?';

  // Based on typical micromouse simplification rules
  if      (prev == 'L' && curr == 'L') newMove = 'S';
  else if (prev == 'L' && curr == 'S') newMove = 'R';
  else if (prev == 'R' && curr == 'L') newMove = 'B';
  else if (prev == 'S' && curr == 'L') newMove = 'R';
  else if (prev == 'S' && curr == 'S') newMove = 'B';
  else if (prev == 'L' && curr == 'R') newMove = 'B';

  if (newMove != '?') {
    path[pathLength - 3] = newMove;
    pathLength -= 2;
    Serial3.print("BT: Optimized to ");
    Serial3.println(newMove);
  }
}

void recordMove(char direction) {
  if (pathLength >= (int)(sizeof(path) / sizeof(path[0]))) return;
  path[pathLength++] = direction;
  simplifyPath();
}

char getNextMove() {
  if (readIndex >= pathLength) return '\0';
  return path[readIndex++];
}

bool pathFinished() {
  return (readIndex >= pathLength);
}

void printPathToBluetooth() {
  Serial3.println("\n--- PATH ---");
  for (int i = 0; i < pathLength; i++) {
    Serial3.print(path[i]);
    Serial3.print(' ');
  }
  Serial3.println("\n------------");
}
