#include "InertialMeasurementUnit.h"
#include "ControlLoop.h"
#include "ServoControl.h"

InertialMeasurementUnit imu;
ControlLoop yawPID, pitchPID, rollPID;
Constants rollConstants = { .Kp = 50, .Kd = 0.5, .Ki = 0.0 };
Constants pitchConstants = { .Kp = 50, .Kd = 0.0, .Ki = 0.0 };
Constants yawConstants = { .Kp = 50, .Kd = 0.0, .Ki = 0.0 };
ServoControl servos;

unsigned long currentTime;
unsigned long previousTime;

void setup() {
  Serial.begin(115200);
  imu.Init();
  delay(500);
  yawPID.SetGains(rollConstants);
  pitchPID.SetGains(pitchConstants);
  rollPID.SetGains(yawConstants);
  servos.Init();
  currentTime = 0;
}

// Main loop
void loop() {
  previousTime = currentTime;
  currentTime = millis();

  float output[] = { 0, 0, 0, 0 };
  imu.getRotation(output);
  // Serial.print("Rotation in quaternions: ");
  // Serial.print(output[0]);
  // Serial.print("\t");
  // Serial.print(output[1]);
  // Serial.print("\t");
  // Serial.print(output[2]);
  // Serial.print("\t");
  // Serial.println(output[3]);

  float yaw = 0;
  float pitch = 0;
  float roll = 0;
  imu.GetEulerAngle(yaw, pitch, roll, output);
  // Serial.print("Yaw: ");
  // Serial.print(yaw * 57.29);
  // Serial.print("\t");
  // Serial.print("Pitch: ");
  // Serial.print(pitch * 57.29);
  // Serial.print("\t");
  // Serial.print("Roll: ");
  // Serial.println(roll * 57.29);
  imu.GetAdjustedEulerAngle(yaw, pitch, roll);
  Serial.print("Adjusted Yaw: ");
  Serial.print(yaw * 57.29);
  Serial.print("\t");
  Serial.print("Adjusted Pitch: ");
  Serial.print(pitch * 57.29);
  Serial.print("\t");
  Serial.print("Adjusted Roll: ");
  Serial.println(roll * 57.29);

  float deltaTime = currentTime - previousTime;

  float yawCommand = yawPID.ComputeCorrection(yaw, deltaTime);
  float pitchCommand = pitchPID.ComputeCorrection(pitch, deltaTime);
  float rollCommand = rollPID.ComputeCorrection(roll, deltaTime);

  pitchServoPwr * mixing - rollServoPwr * mixing - yawServoPwr * mixing
  delay(100);
}
