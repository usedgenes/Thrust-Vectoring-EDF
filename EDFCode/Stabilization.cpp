#ifndef UNIT_TEST
#include "Stabilization.h"

void Stabilization::Init() {
    servosSpeedControl.Init();
    inertialMeasurementUnit.Init();

    SetAngleModeControlLoopConfig();
    SetYawControlLoopConfig();
}

void Stabilization::SetAngleModeControlLoopConfig() {
    rollPosPID_Angle.SetGains(ControlLoopConstants::GetInstance()->anglePos);
    pitchPosPID_Angle.SetGains(ControlLoopConstants::GetInstance()->anglePos);
    rollSpeedPID_Angle.SetGains(ControlLoopConstants::GetInstance()->angleSpeed);
    pitchSpeedPID_Angle.SetGains(ControlLoopConstants::GetInstance()->angleSpeed);
}

void Stabilization::SetYawControlLoopConfig() {
    //change first number after map
    ControlLoopConstants::GetInstance()->yawSpeed.Kp = (float)map(0.5, 0, 1023, 0, 500);
    CustomSerialPrint::print("Yaw kP: ");
    CustomSerialPrint::println(ControlLoopConstants::GetInstance()->yawSpeed.Kp);
    yawControlLoop.SetGains(ControlLoopConstants::GetInstance()->yawSpeed);
}


void Stabilization::Angle(float _loopTimeSec) {
    // Get current attitude (roll, pitch, yaw angles and speeds)
    ComputeAttitude(angularPosCurr, angularSpeedCurr, _loopTimeSec);

    // Compute roll position command
    int rollPosCmd = rollPosPID_Angle.ComputeCorrection(0, angularPosCurr[XAXIS], _loopTimeSec);

    // Compute roll speed command
    rollServoPwr = rollSpeedPID_Angle.ComputeCorrection(rollPosCmd, angularSpeedCurr[XAXIS], _loopTimeSec);

    // Compute pitch position command
    int pitchPosCmd = pitchPosPID_Angle.ComputeCorrection(0, angularPosCurr[YAXIS], _loopTimeSec);

    // Compute pitch speed command
    pitchServoPwr = pitchSpeedPID_Angle.ComputeCorrection(pitchPosCmd, angularSpeedCurr[YAXIS],
                                                          _loopTimeSec);

    // Compute yaw speed command
    yawServoPwr = yawControlLoop.ComputeCorrection(0,angularSpeedCurr[ZAXIS], _loopTimeSec);

    // Apply computed command to Servos
    SetServosPosition();
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

    SetServosPosition();
}

void Stabilization::SetServosPosition() {
    servosSpeedControl.UpdatePosition(Servo0, pitchServoPwr * mixing + rollServoPwr * mixing - yawServoPwr * mixing);
    servosSpeedControl.UpdatePosition(Servo1, pitchServoPwr * mixing - rollServoPwr * mixing + yawServoPwr * mixing);
    servosSpeedControl.UpdatePosition(Servo2, pitchServoPwr * mixing - rollServoPwr * mixing - yawServoPwr * mixing);
    servosSpeedControl.UpdatePosition(Servo3, pitchServoPwr * mixing + rollServoPwr * mixing + yawServoPwr * mixing);
}


#endif