#ifndef INERTIALMEASUREMENTUNIT_H_
#define INERTIALMEASUREMENTUNIT_H_

#include "SparkFun_BNO08x_Arduino_Library.h"
#include "CustomSerialPrint.h"

class InertialMeasurementUnit{
  private:
    float rotationOffsets[4] = {0, 0, 0, 0};
    SPIClass vspi = SPIClass(VSPI);
    BNO08x accelgyro; // IMU

  public:
    void setReports();
    void Init();
    void ComputeRotationOffsets();
    void getRotation(float output[]);
    void GetCorrectedAccelGyro(float rotation[]);
    void GetCurrentEulerAngle(float& yaw, float& pitch, float& roll, float quaternions[]);
};

#endif // INERTIALMEASUREMENTUNIT_H_