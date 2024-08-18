#include "InertialMeasurementUnit.h"

void InertialMeasurementUnit::Init() {
  vspi.begin(22, 23, 21, 33);  // Communication with MPU-6050 at 400KHz
  accelgyro.beginSPI(33, 25, 26, 1000000, vspi);
  setReports();
  ComputeEulerOffsets();
}

void InertialMeasurementUnit::GetAdjustedEulerAngle(float input[], float output[]) {
  output[0] = input[0] - EulerOffsets[0];
  output[1] = input[1] - EulerOffsets[1];
  output[2] = input[2] - EulerOffsets[2];
}

void InertialMeasurementUnit::GetEulerAngle(float input[], float output[]) {
  float x = input[0];
  float y = input[1];
  float z = input[2];
  float w = input[3];

  const double w2 = w * w;
  const double x2 = x * x;
  const double y2 = y * y;
  const double z2 = z * z;
  const double unitLength = w2 + x2 + y2 + z2;
  const double abcd = w * x + y * z;
  const double eps = 1e-7;
  const double pi = 3.14159265358979323846;
  if (abcd > (0.5 - eps) * unitLength) {
    output[0] = 2 * atan2(y, w);
    output[1] = pi;
    output[2] = 0;
  } else if (abcd < (-0.5 + eps) * unitLength) {
    output[0] = -2 * ::atan2(y, w);
    output[1] = -pi;
    output[2] = 0;
  } else {
    const double adbc = w * z - x * y;
    const double acbd = w * y - x * z;
    output[0] = ::atan2(2 * adbc, 1 - 2 * (z2 + x2));
    output[1] = ::asin(2 * abcd / unitLength);
    output[2] = ::atan2(2 * acbd, 1 - 2 * (y2 + x2));
  }
}

void InertialMeasurementUnit::ComputeEulerOffsets() {
  for (int i = 0; i < 25; i++) {
    float quaternions[4] = { 0, 0, 0, 0 };
    getRotation(quaternions);
    float eulerAngle[3];
    GetEulerAngle(eulerAngle, quaternions);
    EulerOffsets[0] += eulerAngle[0];
    EulerOffsets[1] += eulerAngle[1];
    EulerOffsets[2] += eulerAngle[2];
    // Serial.print(EulerOffsets[0]);
    // Serial.print("\t");
    // Serial.print(EulerOffsets[1]);
    // Serial.print("\t");
    // Serial.println(EulerOffsets[2]);
  }
  EulerOffsets[0] = EulerOffsets[0] / 25;
  EulerOffsets[1] = EulerOffsets[1] / 25;
  EulerOffsets[2] = EulerOffsets[2] / 25;
  Serial.print("Offsets: ");
  Serial.print(EulerOffsets[0] * 57.29);
  Serial.print("\t");
  Serial.print(EulerOffsets[1] * 57.29);
  Serial.print("\t");
  Serial.println(EulerOffsets[2] * 57.29);
}

void InertialMeasurementUnit::getRotation(float output[]) {
  if (accelgyro.wasReset()) {
    setReports();
  }
  accelgyro.getSensorEvent();
  while (accelgyro.getSensorEventID() != SENSOR_REPORTID_ROTATION_VECTOR) {
    setReports();
    accelgyro.getSensorEvent();
  }
  output[0] = accelgyro.getQuatI();
  output[1] = accelgyro.getQuatJ();
  output[2] = accelgyro.getQuatK();
  output[3] = accelgyro.getQuatReal();
}

void InertialMeasurementUnit::setReports(void) {
  if (accelgyro.enableRotationVector() == true) {
  } else {
    Serial.println("Could not enable rotation vector");
  }
  delay(100);
}