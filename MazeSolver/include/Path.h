// Path.h
#pragma once
#include <Arduino.h>

// Path of moves through the maze: 'L', 'R', 'S', 'B'
void resetPath();
void recordMove(char direction);      // appends + simplifies
char getNextMove();                   // returns '\0' if none
bool pathFinished();
void printPathToBluetooth();
