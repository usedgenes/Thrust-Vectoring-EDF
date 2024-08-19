#include "ControlLoop.h"

void ControlLoop::SetGains(Constants _constants) {
  constants = _constants;
}

float ControlLoop::ComputeCorrection(float error, float loopTime) {
  integrator += error * loopTime;
  float derivative = (error - errorPrev) / loopTime;
  float output = (constants.Kp * error) + (constants.Ki * integrator) + (constants.Kd * derivative);

  errorPrev = error;
  
  return output;
}
