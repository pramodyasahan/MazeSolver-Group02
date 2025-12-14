#pragma once
#include <Arduino.h>

// =======================================================
// High-level robot states (includes 2 transition states)
// =======================================================
enum RobotState {
  STATE_MAPPING_SMALL,
  STATE_TRANS_TO_LINE,     // NEW: Maze -> Line transition
  STATE_LINE_FOLLOW,       // mapping run line follow
  STATE_TRANS_TO_MAZE2,    // NEW: Line -> Maze2 transition
  STATE_MAPPING_BIG,
  STATE_WAIT_FOR_RUN2,
  STATE_SOLVING_SMALL,
  STATE_SOLVING_LINE,
  STATE_SOLVING_BIG,
  STATE_DONE
};

// =======================================================
// Module APIs
// =======================================================
void lineFollowerBegin();
void lineFollowerUpdate(bool isMappingPhase);  // true: mapping line, false: solve line

void wallFollowerBegin();
void wallFollowerMappingUpdate();              // records path via MazeSolver
void wallFollowerSolvingUpdate();              // replays simplified path via MazeSolver

void mazeSolverBegin();
void mazeSolverSetActiveSmall();
void mazeSolverSetActiveBig();
void mazeSolverResetReadIndex();
char mazeSolverGetNextMove(bool* hasMove);
void mazeSolverRecord(char move);
void mazeSolverPrintPaths();

// =======================================================
// Encoder ISR symbols (implemented in WallFollower.cpp)
// =======================================================
void countEncoderL();
void countEncoderR();
