#include "ControlLoop.h"

void ControlLoop::SetGains(Constants _constants) {
    constants = _constants;
}

void ControlLoop::Reset() {
    errorPrev = 0;
    integrator = 0;
}

float ControlLoop::ComputeCorrection(float _cmd, float _pos, float _loopTime) {
    error = _cmd - _pos;
    integrator = integrator + error;
    float correction = (constants.G * (constants.Kp * error + constants.Kd * ((error - errorPrev) / (_loopTime)) + constants.Ki * integrator));

    errorPrev = error;

    // Correction in us
    return correction;
}

