#include "InertialMeasurementUnit.h"
#include "ControlLoop.h"
#include "ServoControl.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// #define PRINT_QUATERNIONS 0
// #define PRINT_RAW_ANGLE 0
#define PRINT_ADJUSTED_ANGLE 0
// #define PRINT_PID_COMMANDS 0
// #define PRINT_SERVO_POSITION 0
// #define PRINT_BLUETOOTH_BMI088
// #define PRINT_BLUETOOTH_SERVO 0
// #define PRINT_BLUETOOTH_PID 0

#define DEVICE_NAME "ESP_32"
#define SERVICE_UUID "9a8ca9ef-e43f-4157-9fee-c37a3d7dc12d"
#define SERVO_UUID "f74fb3de-61d1-4f49-bd77-419b61d188da"
#define BNO08X_UUID "c91b34b8-90f3-4fee-89f7-58c108ab198f"
#define PID_UUID "a979c0ba-a2be-45e5-9d7b-079b06e06096"
#define RESET_UUID "fb02a2fa-2a86-4e95-8110-9ded202af76b"


BLECharacteristic *pServo;
BLECharacteristic *pBNO08X;
BLECharacteristic *pPID;
BLECharacteristic *pReset;

InertialMeasurementUnit imu;
ControlLoop yawPID, pitchPID, rollPID;
Constants rollConstants = { .Kp = 75, .Kd = 0.5, .Ki = 0.0 };
Constants pitchConstants = { .Kp = 75, .Kd = 0.0, .Ki = 0.0 };
Constants yawConstants = { .Kp = 75, .Kd = 0.0, .Ki = 0.0 };
ServoControl servos;
Servo EDF;

bool pidOn = true;

unsigned long previousTime;
float mixing = 0.5;
float servo0pos = 0;
float servo1pos = 0;
float servo2pos = 0;
float servo3pos = 0;
float adjustedYaw = 0;
float adjustedPitch = 0;
float adjustedRoll = 0;
float yawCommand = 0;
float pitchCommand = 0;
float rollCommand = 0;
float previousAdjustedYaw = 0;

void (*resetFunc)(void) = 0;  // create a standard reset function

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Connected");
  };

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Disconnected");
    resetFunc();
  }
};


class ResetCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.substring(0, 1) == "0") {
      resetFunc();
    }
  }
};

class ServoCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.substring(0, 1) == "0") {
      servos.WriteServoPosition(0, value.substring(1, value.length()).toInt());
#ifdef PRINT_BLUETOOTH_SERVO
      Serial.print("Writing servo0: ");
      Serial.println(value.substring(1, value.length()).toInt());
#endif
    } else if (value.substring(0, 1) == "1") {
      servos.WriteServoPosition(1, value.substring(1, value.length()).toInt());
#ifdef PRINT_BLUETOOTH_SERVO
      Serial.print("Writing servo1: ");
      Serial.println(value.substring(1, value.length()).toInt());
#endif
    } else if (value.substring(0, 1) == "2") {
      servos.WriteServoPosition(2, value.substring(1, value.length()).toInt());
#ifdef PRINT_BLUETOOTH_SERVO
      Serial.print("Writing servo2: ");
      Serial.println(value.substring(1, value.length()).toInt());
#endif
    } else if (value.substring(0, 1) == "3") {
      servos.WriteServoPosition(3, value.substring(1, value.length()).toInt());
#ifdef PRINT_BLUETOOTH_SERVO
      Serial.print("Writing servo3: ");
      Serial.println(value.substring(1, value.length()).toInt());
#endif
    } else if (value.substring(0, 1) == "4") {
      EDF.write(value.substring(1, value.length()).toInt());
#ifdef PRINT_BLUETOOTH_SERVO
      Serial.print("Writing EDF: ");
      Serial.println(value.substring(1, value.length()).toInt());
#endif
    }
    if (value.substring(0, 1) == "2") {
      pCharacteristic->setValue("3" + String(servo0pos));
      pCharacteristic->notify();
      pCharacteristic->setValue("4" + String(servo1pos));
      pCharacteristic->notify();
      pCharacteristic->setValue("5" + String(servo2pos));
      pCharacteristic->notify();
      pCharacteristic->setValue("6" + String(servo3pos));
      pCharacteristic->notify();
    }
  }
};

class BNO08XCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.substring(0, 1) == "0") {
      pCharacteristic->setValue("7" + String(adjustedYaw, 2));
      pCharacteristic->notify();

      pCharacteristic->setValue("8" + String(adjustedPitch));
      pCharacteristic->notify();

      pCharacteristic->setValue("9" + String(adjustedRoll));
      pCharacteristic->notify();

#ifdef PRINT_BLUETOOTH_BMI088
      Serial.print("Writing bluetooth:");
      Serial.print("\t");
      Serial.print(adjustedYaw);
      Serial.print("\t");
      Serial.print(adjustedPitch);
      Serial.print("\t");
      Serial.println(adjustedRoll);
#endif
    }
    if (value.substring(0, 1) == "1") {
      imu.ComputeEulerOffsets();
    }
  }
};

class PIDCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.substring(0, 1) == "0") {
      rollConstants.Kp = value.substring(1, value.indexOf(',')).toFloat();
      rollConstants.Ki = value.substring(value.indexOf(',') + 1, value.indexOf('!')).toFloat();
      rollConstants.Kd = value.substring(value.indexOf('!') + 1, value.length()).toFloat();
#ifdef PRINT_BLUETOOTH_PID
      Serial.print("Roll: ");
      Serial.print("\t");
      Serial.print(rollConstants.Kp);
      Serial.print("\t");
      Serial.print(rollConstants.Ki);
      Serial.print("\t");
      Serial.println(rollConstants.Kd);
#endif
    }
    if (value.substring(0, 1) == "1") {
      pitchConstants.Kp = value.substring(1, value.indexOf(',')).toFloat();
      pitchConstants.Ki = value.substring(value.indexOf(',') + 1, value.indexOf('!')).toFloat();
      pitchConstants.Kd = value.substring(value.indexOf('!') + 1, value.length()).toFloat();
#ifdef PRINT_BLUETOOTH_PID
      Serial.print("Pitch: ");
      Serial.print("\t");
      Serial.print(pitchConstants.Kp);
      Serial.print("\t");
      Serial.print(pitchConstants.Ki);
      Serial.print("\t");
      Serial.println(pitchConstants.Kd);
#endif
    }
    if (value.substring(0, 1) == "2") {
      yawConstants.Kp = value.substring(1, value.indexOf(',')).toFloat();
      yawConstants.Ki = value.substring(value.indexOf(',') + 1, value.indexOf('!')).toFloat();
      yawConstants.Kd = value.substring(value.indexOf('!') + 1, value.length()).toFloat();
#ifdef PRINT_BLUETOOTH_PID
      Serial.print("Yaw: ");
      Serial.print("\t");
      Serial.print(yawConstants.Kp);
      Serial.print("\t");
      Serial.print(yawConstants.Ki);
      Serial.print("\t");
      Serial.println(yawConstants.Kd);
#endif
    }
    if (value.substring(0, 1) == "3") {
      pCharacteristic->setValue("4" + String(yawCommand, 3));
      pCharacteristic->notify();
      pCharacteristic->setValue("5" + String(pitchCommand, 3));
      pCharacteristic->notify();
      pCharacteristic->setValue("6" + String(rollCommand, 3));
      pCharacteristic->notify();
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

  pBNO08X = pService->createCharacteristic(BNO08X_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  pBNO08X->setCallbacks(new BNO08XCallbacks());

  pPID = pService->createCharacteristic(PID_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  pPID->setCallbacks(new PIDCallbacks());

  pReset = pService->createCharacteristic(RESET_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  pReset->setCallbacks(new ResetCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();

  BLEAdvertisementData adv;
  adv.setName(devName.c_str());
  pAdvertising->setAdvertisementData(adv);

  BLEAdvertisementData adv2;
  adv2.setCompleteServices(BLEUUID(SERVICE_UUID));
  pAdvertising->setScanResponseData(adv2);

  pAdvertising->start();

  yawPID.SetGains(rollConstants);
  pitchPID.SetGains(pitchConstants);
  rollPID.SetGains(yawConstants);
  servos.Init();
  imu.Init();
  EDF.attach(15);
  EDF.write(50);
}

// Main loop
void loop() {
  float deltaTime = millis() - previousTime;
  previousTime = millis();

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

  imu.GetAdjustedEulerAngle(yaw, pitch, roll, adjustedYaw, adjustedPitch, adjustedRoll);

#ifdef PRINT_ADJUSTED_ANGLE
  Serial.print("Adjusted Yaw: ");
  Serial.print(adjustedYaw * 57.29);
  Serial.print("\t");
  Serial.print("Adjusted Pitch: ");
  Serial.print(adjustedPitch * 57.29);
  Serial.print("\t");
  Serial.print("Adjusted Roll: ");
  Serial.println(adjustedRoll * 57.29);
#endif
  float yawVelocity = (adjustedYaw - previousAdjustedYaw) / deltaTime;
  previousAdjustedYaw = adjustedYaw;
  yawCommand = yawPID.ComputeCorrection(yawVelocity, deltaTime);
  pitchCommand = pitchPID.ComputeCorrection(adjustedPitch, deltaTime);
  rollCommand = rollPID.ComputeCorrection(adjustedRoll, deltaTime);

#ifdef PRINT_PID_COMMANDS
  Serial.print("Yaw Command: ");
  Serial.print(yawCommand);
  Serial.print("\t");
  Serial.print("Pitch Command: ");
  Serial.print(pitchCommand);
  Serial.print("\t");
  Serial.print("Roll Command: ");
  Serial.println(rollCommand);
#endif
  yawCommand = 0;
  servo0pos = servos.WriteServoPosition(0, -pitchCommand * mixing - yawCommand * mixing);
  servo1pos = servos.WriteServoPosition(1, -rollCommand * mixing - yawCommand * mixing);
  servo2pos = servos.WriteServoPosition(2, pitchCommand * mixing - yawCommand * mixing);
  servo3pos = servos.WriteServoPosition(3, rollCommand * mixing - yawCommand * mixing);

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
