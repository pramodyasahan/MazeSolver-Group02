#ifndef FLOOD_H
#define FLOOD_H

#include <Arduino.h>

// External API
void runFloodController(); // call from loop(); reads SEARCH_MODE pin
void setMazeSize(int n);   // optional (default 9)
void setCellCm(int cm);    // optional (default 25)

#endif // FLOOD_H
