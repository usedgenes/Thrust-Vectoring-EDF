#ifndef CONTROLLOOP_H_
#define CONTROLLOOP_H_

typedef struct {
  float Kp;
  float Kd;
  float Ki;
} Constants;

class ControlLoop {
private:
  Constants constants;
  
  float error = 0;
  float errorPrev = 0;
  float integrator = 0;

public:
  void SetGains(Constants _constants);
  float ComputeCorrection(float error, float loopTime);
};

#endif  // CONTROLLOOP_H_
