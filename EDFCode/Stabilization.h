#ifndef UNIT_TEST

#ifndef STABILIZATION_H_
#define STABILIZATION_H_

#include "InertialMeasurementUnit.h"
#include "ServosSpeedControl.h"
#include "ControlLoop.h"
#include "ControlLoopConstants.h"
#include "CustomMath.h"

class Stabilization {
  private:
    const float mixing = 0.5;
    static const int nbAxis = 3;
    enum AXIS { XAXIS = 0, YAXIS = 1, ZAXIS = 2 };
    float rollServoPwr, pitchServoPwr, yawServoPwr = 0;
    float angularSpeedCurr[nbAxis] = {0.0, 0.0, 0.0}; // Teta speed (°/s) (only use gyro)
    float angularPosCurr[nbAxis] = {0.0, 0.0, 0.0};   // Teta position (°) (use gyro + accelero)

    /* /!\ HighPassFilterCoeff is an important coeff for complementary filter
          /!\
          Too high, position will drift, too low, there will be noise from
          accelerometer
          14-Jul-2017: loop time = 2.49ms. Gyro does not drift with coef =< 0.999
          timeCste = (coeff*dt)/(1-coeff)
           coeff = timeCste/(dt + timeCste) If we want 0.5sec, coeff = 0.5/(0.003 +
          0.5) = 0.994
        */
    static constexpr float HighPassFilterCoeff = 0.9995;

    ServosSpeedControl servosSpeedControl;
    ControlLoop rollPosPID_Angle, pitchPosPID_Angle;
    ControlLoop rollSpeedPID_Angle, pitchSpeedPID_Angle;
    ControlLoop yawControlLoop;
    InertialMeasurementUnit inertialMeasurementUnit;

  public:
    void Init();
    void SetServosPosition();
    void Idle();
    void Angle(float _loopTimeSec);
    void ResetPID();

    int GetServosMaxPosition() {
        return servosSpeedControl.GetServosMaxPosition();
    }
    int GetServosMinPosition() {
        return servosSpeedControl.GetServosMinPosition();
    }

    bool AreAttitudeOffsetsComputed() {
        return inertialMeasurementUnit.AreOffsetComputed();
    }
    void AttitudeComputeOffsets() {
        inertialMeasurementUnit.ComputeOffsets();
    }

  private:
    void PrintAngleModeParameters();
    void SetAngleModeControlLoopConfig();
    void SetYawControlLoopConfig();
    float GetFilterTimeConstant(float _loopTimeSec);
    void ComputeAttitude(float _angularPos[], float _angularSpeed[], float _loop_time);
    float ApplyComplementaryFilter(float _angularPos, float gyroRaw, float _angleDegrees, float _loopTime);
};
#endif // STABILIZATION_H_
#endif
