// Config.h
#pragma once
#include <Arduino.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Ultrasonic pins
#define TRIG_FRONT 48
#define ECHO_FRONT 49
#define TRIG_RIGHT 50
#define ECHO_RIGHT 51
#define TRIG_LEFT  52
#define ECHO_LEFT  53

// Motor driver pins (HW-039 + JGA25-370)
#define L_RPWM 4
#define L_LPWM 5
#define L_REN  6
#define L_LEN  7

#define R_RPWM 2
#define R_LPWM 3
#define R_REN  9
#define R_LEN  8

// Encoders
#define L_ENC_A 18
#define L_ENC_B 19
#define R_ENC_A 20
#define R_ENC_B 21

// Line sensor pins (QTR-8 style, digital mode)
// NOTE: static gives internal linkage, safe in a header.
static const uint8_t LINE_SENSOR_PINS[8] = {
  22, 23, 24, 25, 26, 27, 28, 29
};

// Mode switch (for starting solving, etc., optional)
#define SEARCH_MODE 33   // connect to GND to trigger, INPUT_PULLUP


// ============================================================================
// ROBOT CONSTANTS (TUNABLE)
// ============================================================================

// Mechanics
static const float WHEEL_DIAMETER_CM = 6.5f;
static const float WHEEL_BASE_CM     = 16.0f; // distance between wheels (approx, TODO: measure)

static const int pulsesPerMotorRev   = 11;
static const int gearRatio           = 20;
static const int countsPerRev        = pulsesPerMotorRev * gearRatio * 2; // quadrature

// Motor base PWMs
static const int BASE_PWM_STRAIGHT   = 65;   // TODO: tune
static const int TURN_PWM            = 55;   // for 90° turns, TODO: tune
static const int PIVOT_180_PWM       = 60;   // for 180°, TODO: tune

// Maze / wall following thresholds (cm)
static const int OBSTACLE_THRESHOLD      = 8;   // front too close
static const int FRONT_WALL_TRIGGER      = 13;  // solving safety
static const int OPEN_SPACE_THRESHOLD_CM = 20;  // left/right "open"
static const int WALL_DETECT_RANGE       = 20;  // used in centering
static const int DEAD_END_THRESHOLD      = 10;  // all three walls

// Maze centering gains
static const float Kp_Solve = 5.0f;
static const int   MAX_CORRECTION = 30;
static const int   TARGET_WALL_DIST = 7;   // ideal distance from wall, cm

// Line following PID (digital QTR-style)
static const float Kp_Line = 4.0f;   // TODO: tune
static const float Ki_Line = 0.0f;   // often 0 is fine
static const float Kd_Line = 12.0f;  // TODO: tune

static const int BASE_SPEED_LINE = 70;  // TODO: tune
static const int MAX_PWM_LINE    = 120; // limit for correction

// White-box detection
static const int WHITE_COUNT_THRESHOLD = 7;   // number of sensors seeing white
