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

void Stabilization::GetCurrentAttitude() {
  inertial
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

#endif