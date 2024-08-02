#ifndef UNIT_TEST

#ifndef STABILIZATION_H_
#define STABILIZATION_H_

#include "hardware/InertialMeasurementUnit.h"
#include "hardware/ServosSpeedControl.h"
#include "ControlLoop.h"
#include "hardware/RadioReception.h"
#include "ControlLoopConstants.h"
#include "../customLibs/CustomMath.h"

class Stabilization {
  private:
    const float mixing = 0.5;
    static const int nbAxis = 3;
    enum AXIS { XAXIS = 0, YAXIS = 1, ZAXIS = 2 };
    int rollServoPwr, pitchServoPwr, yawServoPwr = 0;
    float angularSpeedCurr[nbAxis] = {0.0, 0.0, 0.0}; // Teta speed (°/s) (only use gyro)
    float angularPosCurr[nbAxis] = {0.0, 0.0, 0.0};   // Teta position (°) (use gyro + accelero)

    /* /!\ HighPassFilterCoeff is an important coeff for complementary filter
          /!\
          Too high, position will drift, to low, there will be noise from
          accelerometer
          14-Jul-2017: loop time = 2.49ms. Gyro does not drift with coef =< 0.999
          timeCste = (coeff*dt)/(1-coeff)
           coeff = timeCste/(dt + timeCste) If we want 0.5sec, coeff = 0.5/(0.003 +
          0.5) = 0.994
        */
    static constexpr float HighPassFilterCoeff = 0.9995;

    uint16_t throttle = 0;
    ServosSpeedControl servosSpeedControl;
    ControlLoop rollPosPID_Angle, pitchPosPID_Angle;
    ControlLoop rollSpeedPID_Angle, pitchSpeedPID_Angle;
    ControlLoop rollSpeedPID_Accro, pitchSpeedPID_Accro;
    ControlLoop yawControlLoop;
    InertialMeasurementUnit inertialMeasurementUnit;
    RadioReception radioReception;

  public:
    void Init();
    void SetServosPwrXConfig();
    void Idle();
    void Accro(float _loopTimeSec);
    void Angle(float _loopTimeSec);
    void ResetPID();
    void ApplyServosSpeed(volatile uint16_t *TCNTn, volatile uint16_t *OCRnA) {
        servosSpeedControl.ApplySpeed(TCNTn, OCRnA);
    }
    int GetServosMaxPower() {
        return servosSpeedControl.GetServosMaxPower();
    }
    int GetServosMinPower() {
        return servosSpeedControl.GetServosMinPower();
    }
    int GetServosMaxThrottlePercent() {
        return servosSpeedControl.GetServosMaxThrottlePercent();
    }
    int GetServosMaxThrottle() {
        return servosSpeedControl.GetServosMaxThrottle();
    }
    int GetServosIdleThreshold() {
        return servosSpeedControl.GetServosIdleThreshold();
    }
    bool AreAttitudeOffsetsComputed() {
        return inertialMeasurementUnit.AreOffsetComputed();
    }
    void AttitudeComputeOffsets() {
        inertialMeasurementUnit.ComputeOffsets();
    }
    inline void ComputeRxImpulsionWidth() {
        radioReception.GetWidth();
    }
    inline int GetFlyingMode() {
        return radioReception.GetFlyingMode();
    }
    inline int GetThrottle() {
        return radioReception.GetThrottle(GetServosMinPower(), GetServosMaxThrottle());
    }

    bool IsThrottleIdle() {
        return GetThrottle() < GetServosIdleThreshold();
    }

  private:
    void PrintAccroModeParameters();
    void PrintAngleModeParameters();
    void SetAngleModeControlLoopConfig();
    void SetAccroModeControlLoopConfig();
    void SetYawControlLoopConfig();
    float GetFilterTimeConstant(float _loopTimeSec);
    void ComputeAttitude(float _angularPos[], float _angularSpeed[], float _loop_time);
    float ApplyComplementaryFilter(float _angularPos, float gyroRaw, float _angleDegrees, float _loopTime);
};
#endif // STABILIZATION_H_
#endif
