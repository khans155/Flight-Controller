#pragma once
#include <Arduino.h>
#include "pid.h"

// PID controllers (accessible by comms.cpp for live tuning)
extern PID rollAnglePID, pitchAnglePID;
extern PID rollRatePID, pitchRatePID;
extern PID yawRatePID;

// Motor offsets (adjustable via telnet)
extern float m1_offset, m2_offset, m3_offset, m4_offset;
extern float pitch_offset, roll_offset;

// Motor outputs (for telemetry)
extern uint16_t m1, m2, m3, m4;
extern uint16_t m1_corr, m2_corr, m3_corr, m4_corr;

// RC inputs (set by readPS5)
extern uint16_t throttle;
extern int rollOffset, pitchOffset, yawOffset;

void initMotors(const char* ps5Mac);
void mixMotor();
void readPS5();

// Utility
float floatMap(float x, float in_min, float in_max, float out_min, float out_max);
uint32_t usToDuty(uint16_t us);