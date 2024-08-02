#ifndef UNIT_TEST
#include <avr/wdt.h>
#include "Stabilization.h"

void Stabilization::Init() {
    ServosSpeedControl.Init();
    radioReception.Init();
    inertialMeasurementUnit.Init();

    if ((ServosSpeedControl.GetServosMaxPower() == 1860)
        && (ServosSpeedControl.GetServosMaxThrottle() >= (1860 * 0.8)))
        CustomSerialPrint::println(
                F("!!!!!!!!!!!!!!!!!!!!FLYING MODE "
                  "POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "));
    else if ((ServosSpeedControl.GetServosMaxPower() <= 1300))
        CustomSerialPrint::println(F("DEBUG MODE POWER!!! "));
    else
        CustomSerialPrint::println(F("UNEXPECTED POWER "));

    CustomSerialPrint::print(F("MAX_POWER: "));
    CustomSerialPrint::print(ServosSpeedControl.GetServosMaxPower());
    CustomSerialPrint::print(F(" MAX_THROTTLE_PERCENT: "));
    CustomSerialPrint::println(ServosSpeedControl.GetServosMaxThrottlePercent());

    SetAngleModeControlLoopConfig();
    SetAccroModeControlLoopConfig();
    SetYawControlLoopConfig();
}

void Stabilization::SetAngleModeControlLoopConfig() {
    rollPosPID_Angle.SetGains(ControlLoopConstants::GetInstance()->anglePos);
    pitchPosPID_Angle.SetGains(ControlLoopConstants::GetInstance()->anglePos);
    rollSpeedPID_Angle.SetGains(ControlLoopConstants::GetInstance()->angleSpeed);
    pitchSpeedPID_Angle.SetGains(ControlLoopConstants::GetInstance()->angleSpeed);
}

void Stabilization::SetAccroModeControlLoopConfig() {
    rollSpeedPID_Accro.SetGains(ControlLoopConstants::GetInstance()->accroSpeed);
    pitchSpeedPID_Accro.SetGains(ControlLoopConstants::GetInstance()->accroSpeed);
}

void Stabilization::SetYawControlLoopConfig() {
    // Adjust Kp from potentiometer on A0
    ControlLoopConstants::GetInstance()->yawSpeed.Kp = (float)map(analogRead(0), 0, 1023, 0, 500);
    CustomSerialPrint::print("Yaw kP: ");
    CustomSerialPrint::println(ControlLoopConstants::GetInstance()->yawSpeed.Kp);
    yawControlLoop.SetGains(ControlLoopConstants::GetInstance()->yawSpeed);
}

void Stabilization::Accro(float _loopTimeSec) {
    // Get current attitude (roll, pitch, yaw speeds)
    ComputeAttitude(angularPosCurr, angularSpeedCurr, _loopTimeSec);

    // Compute new speed command for each axis
    rollServoPwr = rollSpeedPID_Accro.ComputeCorrection(radioReception.GetRollSpeed(),
                                                        angularSpeedCurr[XAXIS], _loopTimeSec);
    pitchServoPwr = pitchSpeedPID_Accro.ComputeCorrection(radioReception.GetPitchSpeed(),
                                                          angularSpeedCurr[YAXIS], _loopTimeSec);
    yawServoPwr = yawControlLoop.ComputeCorrection(radioReception.GetYawSpeed(),
                                                   angularSpeedCurr[ZAXIS], _loopTimeSec);

    // Apply computed speed command to Servos
    SetServosPwrXConfig();
}

void Stabilization::Angle(float _loopTimeSec) {
    // Get current attitude (roll, pitch, yaw angles and speeds)
    ComputeAttitude(angularPosCurr, angularSpeedCurr, _loopTimeSec);

    // Compute roll position command
    int rollPosCmd = rollPosPID_Angle.ComputeCorrection(radioReception.GetRollAngle(),
                                                        angularPosCurr[XAXIS], _loopTimeSec);

    // Compute roll speed command
    rollServoPwr =
            rollSpeedPID_Angle.ComputeCorrection(rollPosCmd, angularSpeedCurr[XAXIS], _loopTimeSec);

    // Compute pitch position command
    int pitchPosCmd = pitchPosPID_Angle.ComputeCorrection(radioReception.GetPitchAngle(),
                                                          angularPosCurr[YAXIS], _loopTimeSec);

    // Compute pitch speed command
    pitchServoPwr = pitchSpeedPID_Angle.ComputeCorrection(pitchPosCmd, angularSpeedCurr[YAXIS],
                                                          _loopTimeSec);

    // Compute yaw speed command
    yawServoPwr = yawControlLoop.ComputeCorrection(radioReception.GetYawSpeed(),
                                                   angularSpeedCurr[ZAXIS], _loopTimeSec);

    // Apply computed command to Servos
    SetServosPwrXConfig();
}

float Stabilization::GetFilterTimeConstant(float _loopTimeSec) {
    return ((HighPassFilterCoeff * _loopTimeSec) / (1 - HighPassFilterCoeff));
}

// Compute attitude (pitch angle & speed and roll angle & speed) combining acc + gyro
void Stabilization::ComputeAttitude(float _angularPos[], float _angularSpeed[], float _loopTime) {
    float accRaw[nbAxis] = {0, 0, 0};
    float gyroRaw[nbAxis] = {0, 0, 0};

    // Get corrected data from gyro and accelero
    inertialMeasurementUnit.GetCorrectedAccelGyro(accRaw, gyroRaw);

    // Compute rotation speed using gyroscopes
    for (int axis = 0; axis < nbAxis; axis++)
        _angularSpeed[axis] = gyroRaw[axis];

    CustomMath::VectorNormalize(accRaw, nbAxis);

    float rollAngleDeg = RAD2DEG(atan(accRaw[YAXIS] / accRaw[ZAXIS]));
    _angularPos[XAXIS] =
            ApplyComplementaryFilter(_angularPos[XAXIS], gyroRaw[XAXIS], rollAngleDeg, _loopTime);

    float pitchAngleDeg = RAD2DEG(-atan(accRaw[XAXIS] / accRaw[ZAXIS]));
    _angularPos[YAXIS] =
            ApplyComplementaryFilter(_angularPos[YAXIS], gyroRaw[YAXIS], pitchAngleDeg, _loopTime);
}

// Use complementary filter to merge gyro and accelerometer data
// High pass filter on gyro, and low pass filter on accelerometer
float Stabilization::ApplyComplementaryFilter(float _angularPos, float _gyroRaw,
                                              float _angleDegrees, float _loopTime) {
    return HighPassFilterCoeff * (_angularPos + _gyroRaw * _loopTime)
           + (1 - HighPassFilterCoeff) * _angleDegrees;
}

void Stabilization::PrintAccroModeParameters() {
    CustomSerialPrint::println(F("/********* PID settings *********/"));
    rollSpeedPID_Accro.PrintGains();
    pitchSpeedPID_Accro.PrintGains();
    yawControlLoop.PrintGains();
    CustomSerialPrint::print(F("Mixing: "));
    CustomSerialPrint::println(mixing);
}

void Stabilization::PrintAngleModeParameters() {
    CustomSerialPrint::println(F("/********* PID settings *********/"));
    rollPosPID_Angle.PrintGains();
    pitchPosPID_Angle.PrintGains();

    rollSpeedPID_Angle.PrintGains();
    pitchSpeedPID_Angle.PrintGains();
    yawControlLoop.PrintGains();
    CustomSerialPrint::println(F("/********* Complementary filter *********/"));
    CustomSerialPrint::print("Coefficient: ");
    CustomSerialPrint::print(HighPassFilterCoeff);
    CustomSerialPrint::print(" Time constant: ");
    CustomSerialPrint::println(GetFilterTimeConstant(0.00249));
    CustomSerialPrint::print(F("Mixing: "));
    CustomSerialPrint::println(mixing);
}

void Stabilization::ResetPID() {
    pitchServoPwr = rollServoPwr = yawServoPwr = 0; // No correction if throttle put to min
    rollPosPID_Angle.Reset();
    pitchPosPID_Angle.Reset();
    rollSpeedPID_Angle.Reset();
    pitchSpeedPID_Angle.Reset();
    yawControlLoop.Reset();
    rollSpeedPID_Accro.Reset();
    pitchSpeedPID_Accro.Reset();

    SetServosPwrXConfig();
}

//    X configuration:
//  Servo0(CCW)  Servo1
//         \  /
//         /  \
//   Servo3   Servo2(CCW)
//
void Stabilization::SetServosPwrXConfig() {
    int throttle = GetThrottle();
    ServosSpeedControl.UpdateSpeed(Servo0, throttle - pitchServoPwr * mixing + rollServoPwr * mixing
                                                   - yawServoPwr * mixing);
    ServosSpeedControl.UpdateSpeed(Servo1, throttle - pitchServoPwr * mixing - rollServoPwr * mixing
                                                   + yawServoPwr * mixing);
    ServosSpeedControl.UpdateSpeed(Servo2, throttle + pitchServoPwr * mixing - rollServoPwr * mixing
                                                   - yawServoPwr * mixing);
    ServosSpeedControl.UpdateSpeed(Servo3, throttle + pitchServoPwr * mixing + rollServoPwr * mixing
                                                   + yawServoPwr * mixing);
}

void Stabilization::Idle() {
    ServosSpeedControl.Idle();
}
#endif