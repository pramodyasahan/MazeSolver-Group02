#pragma once
#include <Arduino.h>

// =========================
// Ultrasonic Pins (UNCHANGED)
// =========================
#define TRIG_FRONT 48
#define ECHO_FRONT 49
#define TRIG_RIGHT 50
#define ECHO_RIGHT 51
#define TRIG_LEFT  52
#define ECHO_LEFT  53

// =========================
// Motor Pins (UNCHANGED)
// =========================
#define L_RPWM 4
#define L_LPWM 5
#define L_REN  6
#define L_LEN  7

#define R_RPWM 2
#define R_LPWM 3
#define R_REN  9
#define R_LEN  8

// =========================
// Encoder Pins (UNCHANGED)
// =========================
#define L_ENC_A 18
#define L_ENC_B 19
#define R_ENC_A 20
#define R_ENC_B 21

// =========================
// External Start/Mode Pin (UNCHANGED)
// =========================
#define SEARCH_MODE 33

// =========================
// Line Sensors (UNCHANGED)
// =========================
static const uint8_t LINE_SENSOR_PINS[8] = {22,23,24,25,26,27,28,29};
static const bool BLACK_IS_HIGH = true;

// =========================
// Debug
// =========================
#define DBG_PORT Serial3
#define DBG_BAUD 9600
