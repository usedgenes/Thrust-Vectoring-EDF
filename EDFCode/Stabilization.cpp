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
  // CustomSerialPrint::print("Yaw kP: ");
  // CustomSerialPrint::println(ControlLoopConstants::GetInstance()->yawSpeed.Kp);
  yawControlLoop.SetGains(ControlLoopConstants::GetInstance()->yawSpeed);
}


void Stabilization::Angle(float _loopTimeSec) {
  // Get current attitude (roll, pitch, yaw angles and speeds)
  ComputeAttitude(angularPosCurr, angularSpeedCurr, _loopTimeSec);

  // Compute roll position command
  float rollPosCmd = rollPosPID_Angle.ComputeCorrection(0, angularPosCurr[XAXIS], _loopTimeSec);

  // Compute roll speed command
  rollServoPwr = rollSpeedPID_Angle.ComputeCorrection(rollPosCmd, angularSpeedCurr[XAXIS], _loopTimeSec);

  // Compute pitch position command
  float pitchPosCmd = pitchPosPID_Angle.ComputeCorrection(0, angularPosCurr[YAXIS], _loopTimeSec);

  // Compute pitch speed command
  pitchServoPwr = pitchSpeedPID_Angle.ComputeCorrection(pitchPosCmd, angularSpeedCurr[YAXIS],
                                                        _loopTimeSec);

  // Compute yaw speed command
  yawServoPwr = yawControlLoop.ComputeCorrection(0, angularSpeedCurr[ZAXIS], _loopTimeSec);

  // Serial.print("roll position command: ");
  // Serial.println(rollPosCmd);
  // Serial.print("roll power: ");
  // Serial.println(rollServoPwr);
  // Serial.print("pitch position command: ");
  // Serial.println(pitchPosCmd);
  // Serial.print("pitch power: ");
  // Serial.println(pitchServoPwr);
  // Serial.print("yawServoPwr: ");
  // Serial.println(yawServoPwr);
  // Apply computed command to Servos
  SetServosPosition();
}

float Stabilization::GetFilterTimeConstant(float _loopTimeSec) {
  return ((HighPassFilterCoeff * _loopTimeSec) / (1 - HighPassFilterCoeff));
}

// Compute attitude (pitch angle & speed and roll angle & speed) combining acc + gyro
void Stabilization::ComputeAttitude(float _angularPos[], float _angularSpeed[], float _loopTime) {
  float accRaw[nbAxis] = { 0, 0, 0 };
  float gyroRaw[nbAxis] = { 0, 0, 0 };

  // Get corrected data from gyro and accelero
  inertialMeasurementUnit.GetCorrectedAccelGyro(accRaw, gyroRaw);

  // Compute rotation speed using gyroscopes
  for (int axis = 0; axis < nbAxis; axis++)
    _angularSpeed[axis] = gyroRaw[axis];
  
  // VectorNormalize(accRaw, nbAxis);

  // float rollAngleDeg = RAD2DEG(atan(accRaw[YAXIS] / accRaw[ZAXIS]));
  // _angularPos[XAXIS] =
  //   ApplyComplementaryFilter(_angularPos[XAXIS], gyroRaw[XAXIS], rollAngleDeg, _loopTime);

  // float pitchAngleDeg = RAD2DEG(-atan(accRaw[XAXIS] / accRaw[ZAXIS]));
  // _angularPos[YAXIS] =
  //   ApplyComplementaryFilter(_angularPos[YAXIS], gyroRaw[YAXIS], pitchAngleDeg, _loopTime);

  // CustomSerialPrint::print(F("Raw Gyro Data:"));
  // for (int axis = 0; axis < 3; axis++) {
  //   CustomSerialPrint::print(gyroRaw[axis]);
  //   CustomSerialPrint::print(" ");
  // }
  // CustomSerialPrint::println("");

  // CustomSerialPrint::print(F("Raw Accelerometer Data:"));
  // for (int axis = 0; axis < 3; axis++) {
  //   CustomSerialPrint::print(accRaw[axis]);
  //   CustomSerialPrint::print(" ");
  // }
  // CustomSerialPrint::println("");
}

// Use complementary filter to merge gyro and accelerometer data
// High pass filter on gyro, and low pass filter on accelerometer
float Stabilization::ApplyComplementaryFilter(float _angularPos, float _gyroRaw,
                                              float _angleDegrees, float _loopTime) {
  return HighPassFilterCoeff * (_angularPos + _gyroRaw * _loopTime)
         + (1 - HighPassFilterCoeff) * _angleDegrees;
}

void Stabilization::SetServosPosition() {
  servosSpeedControl.UpdatePosition(0, pitchServoPwr * mixing + rollServoPwr * mixing - yawServoPwr * mixing);
  Serial.print("1 ");
  Serial.println(pitchServoPwr * mixing + rollServoPwr * mixing - yawServoPwr * mixing);
  servosSpeedControl.UpdatePosition(1, pitchServoPwr * mixing - rollServoPwr * mixing + yawServoPwr * mixing);
  Serial.print("2 ");
  Serial.println(pitchServoPwr * mixing - rollServoPwr * mixing - yawServoPwr * mixing);
  servosSpeedControl.UpdatePosition(2, pitchServoPwr * mixing - rollServoPwr * mixing - yawServoPwr * mixing);
  Serial.print("3 ");
  Serial.println(pitchServoPwr * mixing + rollServoPwr * mixing + yawServoPwr * mixing);
  servosSpeedControl.UpdatePosition(3, pitchServoPwr * mixing + rollServoPwr * mixing + yawServoPwr * mixing);
  Serial.print("4 ");
  Serial.println(pitchServoPwr * mixing + rollServoPwr * mixing + yawServoPwr * mixing);
}

void Stabilization::VectorNormalize(float _vectorIn[], const int vectorSize) {
    float sumSquares = 0.0;
    for (int index = 0; index < vectorSize; index++)
        sumSquares += _vectorIn[index] * _vectorIn[index];

    float norm = sqrt(sumSquares);

    if (norm > 0.0)
        for (int index = 0; index < vectorSize; index++)
            _vectorIn[index] = _vectorIn[index] / norm;
    else
        for (int index = 0; index < vectorSize; index++)
            _vectorIn[index] = 0.0;
}


#endif