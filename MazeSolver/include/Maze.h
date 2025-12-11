// Maze.h
#pragma once
#include "Config.h"
#include "Hardware.h"
#include "Path.h"

enum MazeMode {
  MAZE_MODE_MAPPING,
  MAZE_MODE_SOLVING
};

void initMaze();
void mazeUpdate(MazeMode mode);  // call repeatedly
bool mazeReachedEnd();           // one-shot flag when white box reached
