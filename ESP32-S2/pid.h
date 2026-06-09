#pragma once
#include <Arduino.h>

struct PID {
  float kp, ki, kd;
  float integral, prevError;
  float dFiltered, dAlpha;
  float outputLimit;
};

float computePID(PID &pid, float error, float dt, bool resetIntegral);