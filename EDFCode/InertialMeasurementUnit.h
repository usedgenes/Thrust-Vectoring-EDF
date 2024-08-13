#ifndef INERTIALMEASUREMENTUNIT_H_
#define INERTIALMEASUREMENTUNIT_H_

#include "SparkFun_BNO08x_Arduino_Library.h"
#include "CustomSerialPrint.h"

class InertialMeasurementUnit{
  private:
    float rotationOffsets[4] = {0, 0, 0, 0}
    SPIClass vspi = SPIClass(VSPI);
    BNO08x accelgyro; // IMU

  private:
    void setReports();
    void getReports();

  public:
    void Init();
    void ComputeRotationOffsets();
    void GetCorrectedAccelGyro(float rotation[]);
};

#endif // INERTIALMEASUREMENTUNIT_H_