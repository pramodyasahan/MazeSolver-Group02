// LineFollow.h
#pragma once
#include "Config.h"
#include "Hardware.h"

void initLineFollow();
void lineFollowUpdate();   // call repeatedly in line-follow state
bool lineReachedEnd();     // white-box at end of line
