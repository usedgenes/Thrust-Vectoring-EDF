#include "ControlLoop.h"

void ControlLoop::SetGains(Constants _constants) {
  constants = _constants;
}

float ControlLoop::ComputeCorrection(float error, float loopTime) {
  integrator += error * loopTime;
  float derivative = (error - errorPrev) / loopTime;
  float output = (constants.Kp * error) + (constants.Ki * integrator) + (constants.Kd * derivative);

  errorPrev = error;

  if (output > MAX_PID_OUTPUT) {
    output = MAX_PID_OUTPUT;
  } else if (output < MIN_PID_OUTPUT) {
    output = MIN_PID_OUTPUT;
  }
  
  return output;
}
