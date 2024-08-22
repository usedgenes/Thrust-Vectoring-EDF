#include "InertialMeasurementUnit.h"
#include "ControlLoop.h"
#include "ServoControl.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define DEVICE_NAME "ESP_32"
#define SERVICE_UUID "9a8ca9ef-e43f-4157-9fee-c37a3d7dc12d"
#define SERVO_UUID "f74fb3de-61d1-4f49-bd77-419b61d188da"
#define BNO08X_UUID "c91b34b8-90f3-4fee-89f7-58c108ab198f"
#define PID_UUID "a979c0ba-a2be-45e5-9d7b-079b06e06096"
#define UTILITIES_UUID "fb02a2fa-2a86-4e95-8110-9ded202af76b"

#define BLUETOOTH_REFRESH_RATE 100

BLECharacteristic *pServo;
BLECharacteristic *pBNO08X;
BLECharacteristic *pPID;
BLECharacteristic *pUtilities;

InertialMeasurementUnit imu;
ControlLoop yawPID, pitchPID, rollPID;
ServoControl servos;
Servo EDF;

bool logBluetoothEulerAngle = false;
bool logBluetoothPID = false;
bool logBluetoothServo = false;

int bluetoothRefresh = 0;

unsigned long previousTime;
float mixing = 0.5;
float servo0pos = 0;
float servo1pos = 0;
float servo2pos = 0;
float servo3pos = 0;
float yawCommand = 0;
float pitchCommand = 0;
float rollCommand = 0;
float previousYaw = 0;

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


class UtilitiesCallbacks : public BLECharacteristicCallbacks {
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
      servos.servos[0].write(value.substring(1, value.length()).toInt());
    } else if (value.substring(0, 1) == "1") {
      servos.servos[1].write(value.substring(1, value.length()).toInt());
    } else if (value.substring(0, 1) == "2") {
      servos.servos[2].write(value.substring(1, value.length()).toInt());
    } else if (value.substring(0, 1) == "3") {
      servos.servos[3].write(value.substring(1, value.length()).toInt());
    } else if (value.substring(0, 1) == "4") {
      EDF.write(value.substring(1, value.length()).toInt());
    }
    if (value.substring(0, 1) == "1") {
      if (value.substring(1, 2) == "0") {
        logBluetoothServo = false;
      } else {
        logBluetoothServo = true;
      }
    }
  }
};

class BNO08XCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.substring(0, 1) == "0") {
      imu.ComputeEulerOffsets();
    }
    if (value.substring(0, 1) == "1") {
      if (value.substring(1, 2) == "0") {
        logBluetoothEulerAngle = false;
      } else {
        logBluetoothEulerAngle = true;
      }
    }
  }
};

class PIDCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.substring(0, 1) == "0") {
      rollPID.SetGains(value.substring(1, value.indexOf(',')).toFloat(), value.substring(value.indexOf(',') + 1, value.indexOf('!')).toFloat(), value.substring(value.indexOf('!') + 1, value.length()).toFloat());
    }
    if (value.substring(0, 1) == "1") {
      pitchPID.SetGains(value.substring(1, value.indexOf(',')).toFloat(), value.substring(value.indexOf(',') + 1, value.indexOf('!')).toFloat(), value.substring(value.indexOf('!') + 1, value.length()).toFloat());
    }
    if (value.substring(0, 1) == "2") {
      yawPID.SetGains(value.substring(1, value.indexOf(',')).toFloat(), value.substring(value.indexOf(',') + 1, value.indexOf('!')).toFloat(), value.substring(value.indexOf('!') + 1, value.length()).toFloat());
    }
    if (value.substring(0, 1) == "3") {
      if (value.substring(1, 2) == "0") {
        logBluetoothPID = false;
      } else {
        logBluetoothPID = true;
      }
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

  pUtilities = pService->createCharacteristic(UTILITIES_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  pUtilities->setCallbacks(new UtilitiesCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();

  BLEAdvertisementData adv;
  adv.setName(devName.c_str());
  pAdvertising->setAdvertisementData(adv);

  BLEAdvertisementData adv2;
  adv2.setCompleteServices(BLEUUID(SERVICE_UUID));
  pAdvertising->setScanResponseData(adv2);

  pAdvertising->start();

  yawPID.SetGains(1, 0.0, 0.0);
  pitchPID.SetGains(1, 0.0, 0.0);
  rollPID.SetGains(1, 0.0, 0.0);
  imu.Init();
  servos.Init();
  delay(10000);
  Serial.println("servos");
  EDF.attach(15);
  EDF.write(50);
}

// Main loop
void loop() {
  float deltaTime = millis() - previousTime;
  previousTime = millis();

  float output[] = { 0, 0, 0, 0 };
  imu.getRotation(output);

  float yaw = 0;
  float pitch = 0;
  float roll = 0;
  imu.GetEulerAngle(yaw, pitch, roll, output);
  yaw *= 57.29;
  pitch *= 57.29;
  roll *= 57.29;
  roll -= 90;

  float yawVelocity = (yaw - previousYaw) / (deltaTime/1000);
  previousYaw = yaw;
  yawCommand = yawPID.ComputeCorrection(yawVelocity, deltaTime);
  pitchCommand = pitchPID.ComputeCorrection(pitch, deltaTime);
  rollCommand = rollPID.ComputeCorrection(roll, deltaTime);

  servo0pos = servos.WriteServoPosition(0, -pitchCommand * mixing + yawCommand * mixing);
  servo1pos = servos.WriteServoPosition(1, -rollCommand * mixing + yawCommand * mixing);
  servo2pos = servos.WriteServoPosition(2, pitchCommand * mixing + yawCommand * mixing);
  servo3pos = servos.WriteServoPosition(3, rollCommand * mixing + yawCommand * mixing);

  if (bluetoothRefresh == BLUETOOTH_REFRESH_RATE) {
    if (logBluetoothEulerAngle) {
      pBNO08X->setValue("50" + String(yaw));
      pBNO08X->notify();
      pBNO08X->setValue("51" + String(pitch));
      pBNO08X->notify();
      pBNO08X->setValue("52" + String(roll));
      pBNO08X->notify();
      pBNO08X->setValue("53" + String(yawVelocity));
      pBNO08X->notify();
    }
    if (logBluetoothPID) {
      pPID->setValue("50" + String(yawCommand));
      pPID->notify();
      pPID->setValue("51" + String(pitchCommand));
      pPID->notify();
      pPID->setValue("52" + String(rollCommand));
      pPID->notify();
    }
    if (logBluetoothServo) {
      pServo->setValue("50" + String(servo0pos));
      pServo->notify();
      pServo->setValue("51" + String(servo1pos));
      pServo->notify();
      pServo->setValue("52" + String(servo2pos));
      pServo->notify();
      pServo->setValue("53" + String(servo3pos));
      pServo->notify();
    }
    pUtilities->setValue("5" + String(deltaTime));
    pUtilities->notify();
    bluetoothRefresh = 0;
    // Serial.println("Roll Constants: \t" + String(rollConstants.Kp) + "\t" + String(rollConstants.Ki) + "\t" + String(rollConstants.Kd));
    // Serial.println("Pitch Constants: \t" + String(pitchConstants.Kp) + "\t" + String(pitchConstants.Ki) + "\t" + String(pitchConstants.Kd));
    // Serial.println("Yaw Constants: \t" + String(yawConstants.Kp) + "\t" + String(yawConstants.Ki) + "\t" + String(yawConstants.Kd));
  }
  bluetoothRefresh += 1;
}
