#include "ControlLoop.h"

void ControlLoop::SetGains(float _Kp, float _Ki, float _Kd) {
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
}

float ControlLoop::ComputeCorrection(float error, float loopTime) {
  integrator += error * loopTime;
  float derivative = (error - errorPrev) / loopTime;
  float output = (Kp * error) + (Ki * integrator) + (Kd * derivative);
  Serial.println(String(Kp) + "\t" + String(Ki) + "\t" + String(Kd));

  errorPrev = error;
  
  return output;
}
