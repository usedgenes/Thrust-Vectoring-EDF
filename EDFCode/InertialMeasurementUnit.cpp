#include "InertialMeasurementUnit.h"

void InertialMeasurementUnit::Init() {
  vspi.begin(22, 23, 21, 33);  // Communication with MPU-6050 at 400KHz
  accelgyro.beginSPI(33, 25, 26, 1000000, vspi);
  setReports();
  ComputeRotationOffsets();
}

void InertialMeasurementUnit::GetCurrentEulerAngle(float& yaw, float& pitch, float& roll, float quaternions[]) {
  float x = quaternions[0];
  float y = quaternions[1];
  float z = quaternions[2];
  float w = quaternions[3];

  const double w2 = w * w;
  const double x2 = x * x;
  const double y2 = y * y;
  const double z2 = z * z;
  const double unitLength = w2 + x2 + y2 + z2;
  const double abcd = w * x + y * z;
  const double eps = 1e-7;
  const double pi = 3.14159265358979323846;
  if (abcd > (0.5 - eps) * unitLength) {
    yaw = 2 * atan2(y, w);
    pitch = pi;
    roll = 0;
  } else if (abcd < (-0.5 + eps) * unitLength) {
    yaw = -2 * ::atan2(y, w);
    pitch = -pi;
    roll = 0;
  } else {
    const double adbc = w * z - x * y;
    const double acbd = w * y - x * z;
    yaw = ::atan2(2 * adbc, 1 - 2 * (z2 + x2));
    pitch = ::asin(2 * abcd / unitLength);
    roll = ::atan2(2 * acbd, 1 - 2 * (y2 + x2));
  }
}

void InertialMeasurementUnit::ComputeRotationOffsets() {
  for (int i = 0; i < 25; i++) {
    float output[4] = { 0, 0, 0, 0 };
    getRotation(output);
    rotationOffsets[0] += output[0];
    rotationOffsets[1] += output[1];
    rotationOffsets[2] += output[2];
    rotationOffsets[3] += output[3];
  }
  rotationOffsets[0] = rotationOffsets[0] / 50;
  rotationOffsets[1] = rotationOffsets[1] / 50;
  rotationOffsets[2] = rotationOffsets[2] / 50;
  rotationOffsets[3] = rotationOffsets[3] / 50;
}

void InertialMeasurementUnit::getRotation(float output[]) {
  if (accelgyro.wasReset()) {
    setReports();
  }
  accelgyro.getSensorEvent();
  while (accelgyro.getSensorEventID() != SENSOR_REPORTID_ROTATION_VECTOR) {}
  output[0] = accelgyro.getQuatI();
  output[1] = accelgyro.getQuatJ();
  output[2] = accelgyro.getQuatK();
  output[3] = accelgyro.getQuatReal();
}

void InertialMeasurementUnit::setReports(void) {
  if (accelgyro.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
  delay(100);
}