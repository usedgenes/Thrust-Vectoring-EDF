#include "Stabilization.h"
#include "InertialMeasurementUnit.h"
#include "ControlLoop.h"
#include "ControlLoopConstants.h"

Stabilization stabilization;
InertialMeasurementUnit imu;

unsigned long loopTime;
unsigned long previousTime;

void setup() {
  Serial.begin(115200);  // Console print: initialize serial communication
  imu.Init();
  delay(500);
  // stabilization.Init();

  loopTime = 0;
}

// Main loop
void loop() {
  previousTime = loopTime;
  loopTime = millis();

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

  float deltaTime = (loopTime - previousTime) / 1000.0;
  // stabilization.Angle(deltaTime);

  // Serial.print("Loop time: ");
  // Serial.println(deltaTime);

  delay(100);
}
