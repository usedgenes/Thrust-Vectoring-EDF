#include "InertialMeasurementUnit.h"

void InertialMeasurementUnit::Init() {
  vspi.begin(22, 23, 21, 33);  // Communication with MPU-6050 at 400KHz
  accelgyro.beginSPI(33, 25, 26, 1000000, vspi);
  setReports();
  ComputeRotationOffsets();
}

void ComputeRotationOffsets() {
  for (int i = 0; i < 25; i++) {
    float output[5] = { 0, 0, 0, 0, 0 };
    getRotation(output);
    rotationOffsets[0] += output[0];
    rotationOffsets[1] += output[1];
    rotationOffsets[2] += output[2];
    rotationOffsets[3] += output[3];
    rotationOffsets[4] += output[4];
  }
  rotationOffsets[0] = rotationOffsets[0] / 50;
  rotationOffsets[1] = rotationOffsets[1] / 50;
  rotationOffsets[2] = rotationOffsets[2] / 50;
  rotationOffsets[3] = rotationOffsets[3] / 50;
  rotationOffsets[4] = rotationOffsets[4] / 50;
}

void InertialMeasurementUnit::GetCorrectedRotation(float rotation[]) {
}

void getRotation(float output[]) {
  setReports();
  accelgyro.getSensorEvent();
  while (accellgyro.getSensorEventID != SENSOR_REPORTID_ROTATION_VECTOR) {}
  output[0] += accelgyro.getQuatI();
  output[1] += accelgyro.getQuatJ();
  output[2] += accelgyro.getQuatK();
  output[3] += accelgyro.getQuatReal();
  output[4] += accelgyro.getQuatRadianAccuracy();
}

void InertialMeasurementUnit::setReports(void) {
  if (accelgyro.wasReset()) {
    if (accelgyro.enableRotationVector() == true) {
      Serial.println(F("Rotation vector enabled"));
    } else {
      Serial.println("Could not enable rotation vector");
    }
  }
}