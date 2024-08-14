#include "InertialMeasurementUnit.h"
#include "ControlLoop.h"
#include "ServoControl.h"

// #define PRINT_QUATERNIONS 0
// #define PRINT_RAW_ANGLE 0
// #define PRINT_ADJUSTED_ANGLE 0
#define PRINT_PID_COMMANDS 0
// #define PRINT_SERVO_POSITION 0

InertialMeasurementUnit imu;
ControlLoop yawPID, pitchPID, rollPID;
Constants rollConstants = { .Kp = 50, .Kd = 0.5, .Ki = 0.0 };
Constants pitchConstants = { .Kp = 50, .Kd = 0.0, .Ki = 0.0 };
Constants yawConstants = { .Kp = 50, .Kd = 0.0, .Ki = 0.0 };
ServoControl servos;

unsigned long currentTime;
unsigned long previousTime;
float mixing = 0.5;

void setup() {
  Serial.begin(115200);
  imu.Init();
  delay(500);
  yawPID.SetGains(rollConstants);
  pitchPID.SetGains(pitchConstants);
  rollPID.SetGains(yawConstants);
  // servos.Init();
  currentTime = 0;
}

// Main loop
void loop() {
  previousTime = currentTime;
  currentTime = millis();

  float output[] = { 0, 0, 0, 0 };
  imu.getRotation(output);
#ifdef PRINT_QUATERNIONS
  Serial.print("Rotation in quaternions: ");
  Serial.print(output[0]);
  Serial.print("\t");
  Serial.print(output[1]);
  Serial.print("\t");
  Serial.print(output[2]);
  Serial.print("\t");
  Serial.println(output[3]);
#endif
  float yaw = 0;
  float pitch = 0;
  float roll = 0;
  imu.GetEulerAngle(yaw, pitch, roll, output);

#ifdef PRINT_RAW_ANGLE
  Serial.print("Yaw: ");
  Serial.print(yaw * 57.29);
  Serial.print("\t");
  Serial.print("Pitch: ");
  Serial.print(pitch * 57.29);
  Serial.print("\t");
  Serial.print("Roll: ");
  Serial.println(roll * 57.29);
#endif

  imu.GetAdjustedEulerAngle(yaw, pitch, roll);

#ifdef PRINT_ADJUSTED_ANGLE
  Serial.print("Adjusted Yaw: ");
  Serial.print(yaw * 57.29);
  Serial.print("\t");
  Serial.print("Adjusted Pitch: ");
  Serial.print(pitch * 57.29);
  Serial.print("\t");
  Serial.print("Adjusted Roll: ");
  Serial.println(roll * 57.29);
#endif
  float deltaTime = currentTime - previousTime;

  float yawCommand = yawPID.ComputeCorrection(yaw, deltaTime);
  float pitchCommand = pitchPID.ComputeCorrection(pitch, deltaTime);
  float rollCommand = rollPID.ComputeCorrection(roll, deltaTime);

#ifdef PRINT_PID_COMMANDS
  Serial.print("Yaw: ");
  Serial.print(yawCommand);
  Serial.print("\t");
  Serial.print("Pitch: ");
  Serial.print(pitchCommand);
  Serial.print("\t");
  Serial.print("Roll: ");
  Serial.println(rollCommand);
#endif

  // float servo0pos = servos.WriteServoPosition(0, yawCommand * mixing - pitch * mixing - roll * mixing);
  // float servo1pos = servos.WriteServoPosition(1, yawCommand * mixing - pitch * mixing - roll * mixing);
  // float servo2pos = servos.WriteServoPosition(2, yawCommand * mixing - pitch * mixing - roll * mixing);
  // float servo3pos = servos.WriteServoPosition(3, yawCommand * mixing - pitch * mixing - roll * mixing);

#ifdef PRINT_SERVO_POSITION
  Serial.print("Servo 0: ");
  Serial.print(servo0pos);
  Serial.print("\t");
  Serial.print("Servo 1: ");
  Serial.print(servo1pos);
  Serial.print("\t");
  Serial.print("Servo 2: ");
  Serial.print(servo2pos);
  Serial.print("\t");
  Serial.print("Servo 3: ");
  Serial.println(servo3pos);
#endif

  delay(100);
}
