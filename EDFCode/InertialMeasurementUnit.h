#ifndef INERTIALMEASUREMENTUNIT_H_
#define INERTIALMEASUREMENTUNIT_H_

#include "SparkFun_BNO08x_Arduino_Library.h"

class InertialMeasurementUnit {
private:
  float EulerOffsets[3] = { 0, 0, 0 };
  SPIClass vspi = SPIClass(VSPI);
  BNO08x accelgyro;  // IMU

public:
  void setReports();
  void Init();
  void ComputeEulerOffsets();
  void getRotation(float output[]);
  void GetCorrectedAccelGyro(float rotation[]);
  void GetAdjustedEulerAngle(float input[], float output[]);
  void GetEulerAngle(float input[], float output[]);
};

#endif  // INERTIALMEASUREMENTUNIT_H_