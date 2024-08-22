#include "InertialMeasurementUnit.h"

void InertialMeasurementUnit::Init() {
  Serial.println("1");
  vspi.begin(22, 23, 21, 33);
  accelgyro.beginSPI(33, 25, 26, 1000000, vspi);
  setReports();
  Serial.println("2");
  ComputeEulerOffsets();
  Serial.println("3");
}

void InertialMeasurementUnit::GetAdjustedEulerAngle(float& yaw, float& pitch, float& roll, float& adjustedYaw, float& adjustedPitch, float& adjustedRoll) {
  adjustedYaw = yaw - EulerOffsets[0];
  adjustedPitch = pitch - EulerOffsets[1];
  adjustedRoll = roll - EulerOffsets[2];
}

void InertialMeasurementUnit::GetEulerAngle(float& yaw, float& pitch, float& roll, float quaternions[]) {
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

void InertialMeasurementUnit::ComputeEulerOffsets() {
  for (int i = 0; i < 25; i++) {
    float quaternions[4] = { 0, 0, 0, 0 };
    getRotation(quaternions);
    float yaw = 0;
    float pitch = 0;
    float roll = 0;
    GetEulerAngle(yaw, pitch, roll, quaternions);
    EulerOffsets[0] += yaw;
    EulerOffsets[1] += pitch;
    EulerOffsets[2] += roll;
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
  if (accelgyro.getSensorEvent() == true) {
    while (accelgyro.getSensorEventID() != SENSOR_REPORTID_ROTATION_VECTOR) {
      setReports();
      accelgyro.getSensorEvent();
      Serial.print("9");
    }
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