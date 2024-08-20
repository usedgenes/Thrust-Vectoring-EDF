#ifndef CONTROLLOOP_H_
#define CONTROLLOOP_H_

#include "Arduino.h"

class ControlLoop {
private:
  float Kp;
  float Ki;
  float Kd;
  float error = 0;
  float errorPrev = 0;
  float integrator = 0;

public:
  void SetGains(float _Kp, float _Ki, float _Kd);
  float ComputeCorrection(float error, float loopTime);
};

#endif
