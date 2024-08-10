#ifndef INERTIALMEASUREMENTUNIT_H_
#define INERTIALMEASUREMENTUNIT_H_

#include "Wire.h"
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "CustomMath.h"
#include "CustomSerialPrint.h"

class InertialMeasurementUnit{
  private:
    static const int AXIS_NB = 3;
    static const int SAMPLES_NB = 10;
    float AcceleroSensitivity = -1;
    float GyroSensitivity = -1;
    float gyroOffsets[AXIS_NB] = {0, 0, 0};
    float accOffsets[AXIS_NB] = {0, 0, 0};
    bool initialized = false;
    bool offsetComputed = false;
    SPIClass vspi = SPIClass(VSPI);
    BNO08x accelgyro; // IMU

  private:
    bool ComputeGyroOffsets();
    bool ComputeAccelOffsets();
    void SetAccRange(uint8_t _range);
    void SetGyroRange(uint8_t _range);
    void setReports();

  public:
    void Init();
    bool AreOffsetComputed(void) {
        return offsetComputed;
    }
    void ComputeOffsets();
    void GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]);
};

#endif // INERTIALMEASUREMENTUNIT_H_