#include "InertialMeasurementUnit.h"

void InertialMeasurementUnit::Init() {
  // BNO08X: join I2C bus
  vspi.begin(22, 23, 21, 33);  // Communication with MPU-6050 at 400KHz

  // Initialize MPU 6050
  accelgyro.beginSPI(33, 25, 26, 1000000, vspi);
  
  AcceleroSensitivity = 4096;
  GyroSensitivity = 32.8;

  setReports();
  ComputeAccelOffsets();
  ComputeGyroOffsets();
}

void InertialMeasurementUnit::GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]) {
  float accel[AXIS_NB] = { 0, 0, 0 };
  float speed[AXIS_NB] = { 0, 0, 0 };
  while(accelgyro.getSensorEventID() != SENSOR_REPORTID_ACCELEROMETER) {
    setReports();
    accelgyro.getSensorEvent();
  }
  accel[0] = accelgyro.getAccelX();
  accel[1] = accelgyro.getAccelY();
  accel[2] = accelgyro.getAccelZ();
  
  while(accelgyro.getSensorEventID() != SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
    setReports();
    accelgyro.getSensorEvent();
  }
  speed[0] = accelgyro.getGyroX();
  speed[1] = accelgyro.getGyroY();
  speed[2] = accelgyro.getGyroY();


  // Correct raw data with offset
  for (int axis = 0; axis < AXIS_NB; axis++) {
    _accMeasures[axis] = accel[axis] - accOffsets[axis];
    _gyroMeasures[axis] = speed[axis] - gyroOffsets[axis];
  }
}

bool InertialMeasurementUnit::ComputeGyroOffsets() {
  setReports();
  accelgyro.getSensorEvent();
  float gyroRaw[AXIS_NB][SAMPLES_NB];
  float mean = 0;

  for (int axis = 0; axis < AXIS_NB; axis++)
    for (int sample = 0; sample < 10; sample++)
      gyroRaw[axis][sample] = 0;

  // Get 10 samples during 2 sec
  for (int sample = 0; sample < 10; sample++) {
    setReports();
    accelgyro.getSensorEvent();
    while(accelgyro.getSensorEventID() != SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      setReports();
      accelgyro.getSensorEvent();
    }
    gyroRaw[0][sample] = accelgyro.getGyroX();
    gyroRaw[1][sample] = accelgyro.getGyroY();
    gyroRaw[2][sample] = accelgyro.getGyroZ();

    CustomSerialPrint::print(gyroRaw[0][sample]);
    CustomSerialPrint::print("\t");
    CustomSerialPrint::print(gyroRaw[1][sample]);
    CustomSerialPrint::print("\t");
    CustomSerialPrint::println(gyroRaw[2][sample]);
    delay(200);
  }

  // Compute mean
  for (int axis = 0; axis < AXIS_NB; axis++) {
    float total = 0;
    for(int sample = 0; sample < 10; sample++) {
      total += gyroRaw[axis][sample];
    }
    gyroOffsets[axis] = total/10;
  }

  CustomSerialPrint::print(F("Gyroscope offsets Computed :"));
  for (int axis = 0; axis < AXIS_NB; axis++) {
    CustomSerialPrint::print(gyroOffsets[axis]);
    CustomSerialPrint::print(" ");
  }
  CustomSerialPrint::println("(deg.s-1) ");
  return true;
}

bool InertialMeasurementUnit::ComputeAccelOffsets() {
  setReports();
  accelgyro.getSensorEvent();
  float accRaw[AXIS_NB][SAMPLES_NB];
  float mean = 0.0;

  for (int axis = 0; axis < AXIS_NB; axis++)
    for (int sample = 0; sample < 10; sample++)
      accRaw[axis][sample] = 0;

  // Get 10 samples during 2 sec
  for (int sample = 0; sample < 10; sample++) {
    setReports();
    accelgyro.getSensorEvent();
    while(accelgyro.getSensorEventID() != SENSOR_REPORTID_ACCELEROMETER) {
      setReports();
      accelgyro.getSensorEvent();
    }
    accRaw[0][sample] = accelgyro.getAccelX();
    accRaw[1][sample] = accelgyro.getAccelY();
    accRaw[2][sample] = accelgyro.getAccelZ();    
    CustomSerialPrint::print(accRaw[0][sample]);
    CustomSerialPrint::print("\t");
    CustomSerialPrint::print(accRaw[1][sample]);
    CustomSerialPrint::print("\t");
    CustomSerialPrint::println(accRaw[2][sample]);
    delay(200);
  }

  // Mean computation
  for (int axis = 0; axis < AXIS_NB; axis++) {
    float total = 0;
    for(int sample = 0; sample < 10; sample++) {
      total += accRaw[axis][sample];
    }
    accOffsets[axis] = total/10;
  }

  // offset for gravity
  accOffsets[2] = accOffsets[2] - 9.8;

  CustomSerialPrint::print(F("Acceleration offsets Computed :"));
  for (int axis = 0; axis < AXIS_NB; axis++) {
    CustomSerialPrint::print(accOffsets[axis]);
    CustomSerialPrint::print(" ");
  }
  CustomSerialPrint::println("(m.s-2) ");
  return true;
}

void InertialMeasurementUnit::setReports(void) {
  if (accelgyro.wasReset()) {
    if (accelgyro.enableGyro() == true) {
      Serial.println(F("Gyro enabled"));
    } else {
      Serial.println("Could not enable gyro");
    }
    delay(100);

    if (accelgyro.enableAccelerometer() == true) {
      Serial.println(F("Accelerometer enabled"));
    } else {
      Serial.println("Could not enable accelerometer");
    }
    delay(100);
  }
}