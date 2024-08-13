#ifndef CONTROLLOOP_H_
#define CONTROLLOOP_H_

#include "ControlLoopConstants.h"
#include "CustomSerialPrint.h"

class ControlLoop {
  private:
    Constants constants;

    float error = 0;
    float errorPrev = 0;
    float integrator = 0;

  public:
    void SetGains(Constants _constants);
    float ComputeCorrection(float _cmd, float _pos, float _loopTime);
};

#endif // CONTROLLOOP_H_
