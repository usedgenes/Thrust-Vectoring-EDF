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

  CustomSerialPrint::println("imu initialized");
}

void InertialMeasurementUnit::GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]) {
  setReports();
  accelgyro.getSensorEvent();
  float accel[AXIS_NB] = { 0, 0, 0 };
  float speed[AXIS_NB] = { 0, 0, 0 };

  accel[0] = accelgyro.getAccelX();
  accel[1] = accelgyro.getAccelY();
  accel[2] = accelgyro.getAccelZ();
  
  speed[0] = accelgyro.getGyroX();
  speed[1] = accelgyro.getGyroY();
  speed[2] = accelgyro.getGyroY();


  // Correct raw data with offset
  for (int axis = 0; axis < AXIS_NB; axis++) {
    _accMeasures[axis] =
      static_cast<float>((accel[axis] - accOffsets[axis]) / AcceleroSensitivity);
    _gyroMeasures[axis] =
      static_cast<float>((speed[axis] - gyroOffsets[axis]) / GyroSensitivity);
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
    if (!CustomMath::ComputeMean(gyroRaw[axis], SAMPLES_NB, (10 * GyroSensitivity), &mean)) {
      CustomSerialPrint::println(F("ERROR DURING SPEED OFFSETS COMPUTATION !!"));
      return false;
    }
    gyroOffsets[axis] = static_cast<int16_t>(mean);
  }

  CustomSerialPrint::print(F("Gyroscope offsets Computed :"));
  for (int axis = 0; axis < AXIS_NB; axis++) {
    CustomSerialPrint::print(gyroOffsets[axis] / GyroSensitivity);
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
    if (!CustomMath::ComputeMean(accRaw[axis], SAMPLES_NB, (0.2 * AcceleroSensitivity),
                                 &mean)) {
      CustomSerialPrint::println(F("ERROR DURING ACCELERATION OFFSETS COMPUTATION !!"));
      return false;
    }
    accOffsets[axis] = static_cast<int16_t>(mean);
  }

  // Zacc is gravity, it should be 1G ie 4096 LSB/g at -+8g sensitivity
  accOffsets[2] = accOffsets[2] - AcceleroSensitivity;

  CustomSerialPrint::print(F("Acceleration offsets Computed :"));
  for (int axis = 0; axis < AXIS_NB; axis++) {
    CustomSerialPrint::print(accOffsets[axis] / AcceleroSensitivity);
    CustomSerialPrint::print(" ");
  }
  CustomSerialPrint::print("(m.s-2) ");
  return true;
}

void InertialMeasurementUnit::setReports(void) {
  if (accelgyro.wasReset()) {
    Serial.println("Setting desired reports");
    if (accelgyro.enableRotationVector() == true) {
      Serial.println(F("Rotation vector enabled"));
      Serial.println(F("Output in form i, j, k, real, accuracy"));
    } else {
      Serial.println("Could not enable rotation vector");
    }
    delay(100);

    if (accelgyro.enableGyro() == true) {
      Serial.println(F("Gyro enabled"));
      Serial.println(F("Output in form x, y, z, in radians per second"));
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