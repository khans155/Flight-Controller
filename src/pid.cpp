#include "pid.h"

float computePID(PID &pid, float error, float dt, bool resetIntegral) {
  if (resetIntegral) {
    pid.integral = 0;
  } else {
    pid.integral = constrain(pid.integral + error * dt, -pid.outputLimit, pid.outputLimit);
  }

  float derivative = (error - pid.prevError) / dt;
  pid.dFiltered = pid.dFiltered + pid.dAlpha * (derivative - pid.dFiltered);
  float output = (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * pid.dFiltered);
  pid.prevError = error;

  return constrain(output, -pid.outputLimit, pid.outputLimit);
}