#include "Stabilization.h"
#include "InertialMeasurementUnit.h"
#include "ControlLoop.h"
#include "ControlLoopConstants.h"

Stabilization stabilization;
InertialMeasurementUnit imu;

// #define PRINT_LOOP_TIME 0
#define PRINT_EULER_ANGLE 0
#define PRINT_QUATERNIONS 0

unsigned long loopTime;
unsigned long previousTime;

void setup() {
  Serial.begin(115200);  // Console print: initialize serial communication
  imu.Init();
  // stabilization.Init();

  loopTime = 0;
}

// Main loop
void loop() {
  previousTime = loopTime;
  loopTime = millis();

  float output[] = { 0, 0, 0, 0 };

#ifdef PRINT_QUATERNIONS
  Serial.print(output[0]);
  Serial.print("\t");
  Serial.print(output[1]);
  Serial.print("\t");
  Serial.print(output[2]);
  Serial.print("\t");
  Serial.println(output[3]);
#endif

  imu.getRotation(output);
  float yaw = 0;
  float pitch = 0;
  float roll = 0;
  imu.GetCurrentEulerAngle(yaw, pitch, yaw, output);

#ifdef PRINT_EULER_ANGLE
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print("Roll: ");
  Serial.println(roll);
#endif

  float deltaTime = (loopTime - previousTime) / 1000.0;
  // stabilization.Angle(deltaTime);

#ifdef PRINT_LOOP_TIME
  Serial.print("Loop time: ");
  Serial.println(deltaTime);
#endif

  delay(1000);
}
