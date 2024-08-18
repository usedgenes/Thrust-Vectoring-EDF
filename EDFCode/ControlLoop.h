#ifndef CONTROLLOOP_H_
#define CONTROLLOOP_H_

typedef struct {
  float Kp;
  float Kd;
  float Ki;
} Constants;

class ControlLoop {
#define MAX_PID_OUTPUT 1000
#define MIN_PID_OUTPUT -1000
private:
  Constants constants;

  float error = 0;
  float errorPrev = 0;
  float integrator = 0;

public:
  void SetGains(Constants _constants);
  float ComputeCorrection(float error, float loopTime);
};

#endif
