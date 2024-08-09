#ifndef CONTROLLOOPCONSTANTS_H_
#define CONTROLLOOPCONSTANTS_H_

#include "Singleton.h"

typedef struct {
    float G;
    float Kp;
    float Kd;
    float Ki;
} Constants;

class ControlLoopConstants : public SingletonSimple<ControlLoopConstants> {
  public:
    // Angle mode
    // These parameters are very important for flight success
    // They must be tuned for each frame type, Servos, and propeller used
    Constants anglePos = {.G = 0.010, .Kp = 50, .Kd = 0.5, .Ki = 0.0};
    Constants angleSpeed = {.G = 0.010, .Kp = 50, .Kd = 0.0, .Ki = 0.0};

    // Yaw PID (common to both modes, yaw is always speed controlled)
    Constants yawSpeed = {.G = 0.010, .Kp = 50, .Kd = 0.0, .Ki = 0.0};
};
#endif // CONTROLLOOPCONSTANTS_H_
