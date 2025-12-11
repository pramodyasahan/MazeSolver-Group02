// main.cpp
#include <Arduino.h>
#include "Config.h"
#include "Hardware.h"
#include "Maze.h"
#include "LineFollow.h"
#include "Path.h"

// High-level robot states
enum RobotState {
  STATE_MAZE1_MAPPING,
  STATE_MAZE1_SOLVING,
  STATE_LINEFOLLOW,
  // Future extension:
  // STATE_MAZE2_MAPPING,
  // STATE_MAZE2_SOLVING,
  STATE_FINISHED
};

RobotState currentState = STATE_MAZE1_MAPPING;

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);

  initMotors();
  initEncoders();
  initUltrasonic();
  initLineFollow();
  initMaze();

  resetPath();

  pinMode(SEARCH_MODE, INPUT_PULLUP);

  Serial.println("Robot Initialized...");
  Serial3.println("BT: Robot Ready.");
}

void loop() {
  switch (currentState) {
    case STATE_MAZE1_MAPPING:
      mazeUpdate(MAZE_MODE_MAPPING);
      if (mazeReachedEnd()) {
        stopMotors();
        printPathToBluetooth();
        Serial3.println("BT: Maze 1 mapping done. Flip to solving.");
        delay(1000);
        currentState = STATE_MAZE1_SOLVING;
      }
      break;

    case STATE_MAZE1_SOLVING:
      mazeUpdate(MAZE_MODE_SOLVING);
      if (mazeReachedEnd()) {
        stopMotors();
        Serial3.println("BT: Maze 1 solved. Moving to line-follow.");
        delay(1000);
        currentState = STATE_LINEFOLLOW;
      }
      break;

    case STATE_LINEFOLLOW:
      lineFollowUpdate();
      if (lineReachedEnd()) {
        stopMotors();
        Serial3.println("BT: Line-follow section done.");
        currentState = STATE_FINISHED;
      }
      break;

    // Future: Maze 2 mapping/solving states can go here

    case STATE_FINISHED:
      stopMotors();
      // Could blink LED or send BT spam here
      // For now, just halt
      while (1) {
        delay(1000);
      }
      break;
  }
}
