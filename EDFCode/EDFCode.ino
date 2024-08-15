#include "InertialMeasurementUnit.h"
#include "ControlLoop.h"
#include "ServoControl.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// #define PRINT_QUATERNIONS 0
// #define PRINT_RAW_ANGLE 0
// #define PRINT_ADJUSTED_ANGLE 0
#define PRINT_PID_COMMANDS 0
// #define PRINT_SERVO_POSITION 0

#define DEVICE_NAME "ESP_32"
#define SERVICE_UUID "9a8ca9ef-e43f-4157-9fee-c37a3d7dc12d"
#define SERVO_UUID "f74fb3de-61d1-4f49-bd77-419b61d188da"
#define BMI088_UUID "56e48048-19da-4136-a323-d2f3e9cb2a5d"

BLECharacteristic *pServo;
BLECharacteristic *pBMI088;

InertialMeasurementUnit imu;
ControlLoop yawPID, pitchPID, rollPID;
Constants rollConstants = { .Kp = 50, .Kd = 0.5, .Ki = 0.0 };
Constants pitchConstants = { .Kp = 50, .Kd = 0.0, .Ki = 0.0 };
Constants yawConstants = { .Kp = 50, .Kd = 0.0, .Ki = 0.0 };
ServoControl servos;
Servo EDF;

unsigned long currentTime;
unsigned long previousTime;
float mixing = 0.5;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Connected");
  };

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Disconnected");
  }
};

class ServoCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.substring(0, 1) == "0") {
      if (value.substring(1, 2) == "0") {
        servos.WriteServoPosition(0, value.substring(2, value.length()).toInt());
      } 
      else if (value.substring(1, 2) == "1") {
        servos.WriteServoPosition(1, value.substring(2, value.length()).toInt());
      } 
      else if (value.substring(1, 2) == "2") {
        servos.WriteServoPosition(2, value.substring(2, value.length()).toInt());
      } 
      else if (value.substring(1, 2) == "3") {
        servos.WriteServoPosition(3, value.substring(2, value.length()).toInt());
      } 
      else if (value.substring(1, 2) == "4") {
        EDF.write(value.substring(2, value.length()).toInt());
      }
    }
    Serial.println(value);
  }
};

class BMI088Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.substring(0, 1) == "0") {
      float output[] = { 0, 0, 0, 0 };
      imu.getRotation(output);
      float yaw = 0;
      float pitch = 0;
      float roll = 0;
      imu.GetEulerAngle(yaw, pitch, roll, output);
      imu.GetAdjustedEulerAngle(yaw, pitch, roll);

      pCharacteristic->setValue("1" + String(yaw, 2));
      pCharacteristic->notify();

      pCharacteristic->setValue("2" + String(pitch));
      pCharacteristic->notify();

      pCharacteristic->setValue("3" + String(roll));
      pCharacteristic->notify();

      Serial.println("notifying bmi088");
    }
  }
};

void setup() {
  Serial.begin(115200);

  String devName = DEVICE_NAME;
  String chipId = String((uint32_t)(ESP.getEfuseMac() >> 24), HEX);
  devName += '_';
  devName += chipId;

  BLEDevice::init(devName.c_str());
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pServo = pService->createCharacteristic(SERVO_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  pServo->setCallbacks(new ServoCallbacks());

  pBMI088 = pService->createCharacteristic(BMI088_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  pBMI088->setCallbacks(new BMI088Callbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();

  BLEAdvertisementData adv;
  adv.setName(devName.c_str());
  pAdvertising->setAdvertisementData(adv);

  BLEAdvertisementData adv2;
  adv2.setCompleteServices(BLEUUID(SERVICE_UUID));
  pAdvertising->setScanResponseData(adv2);

  pAdvertising->start();

  imu.Init();
  delay(500);
  yawPID.SetGains(rollConstants);
  pitchPID.SetGains(pitchConstants);
  rollPID.SetGains(yawConstants);
  // servos.Init();
  currentTime = 0;
  EDF.attach(15);
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

  float servo0pos = servos.WriteServoPosition(0, yawCommand * mixing - pitch * mixing - roll * mixing);
  float servo1pos = servos.WriteServoPosition(1, yawCommand * mixing - pitch * mixing - roll * mixing);
  float servo2pos = servos.WriteServoPosition(2, yawCommand * mixing - pitch * mixing - roll * mixing);
  float servo3pos = servos.WriteServoPosition(3, yawCommand * mixing - pitch * mixing - roll * mixing);

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
