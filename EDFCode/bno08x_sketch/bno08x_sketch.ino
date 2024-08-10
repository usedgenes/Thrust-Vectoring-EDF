#include "SparkFun_BNO08x_Arduino_Library.h"  

#define BNO08X_CS   33
#define BNO08X_INT  25
#define BNO08X_RST  26

BNO08x myIMU;

SPIClass vspi = SPIClass(VSPI);

void setup() {
  Serial.begin(115200);
  vspi.begin(22, 23, 21, 33);
  while(!Serial) delay(10);
  Serial.println();
  Serial.println("BNO08x Read Example");

  if (myIMU.beginSPI(BNO08X_CS, BNO08X_INT, BNO08X_RST, 1000000, vspi) == false) {
    Serial.println("BNO08x not detected. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");

  setReports();

  Serial.println("Reading events");
  delay(100);
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
  delay(100); 

  if (myIMU.enableGyro() == true) {
    Serial.println(F("Gyro enabled"));
    Serial.println(F("Output in form x, y, z, in radians per second"));
  } else {
    Serial.println("Could not enable gyro");
  }
  delay(100);

  if (myIMU.enableAccelerometer() == true) {
    Serial.println(F("Accelerometer enabled"));
  } else {
    Serial.println("Could not enable accelerometer");
  }
  delay(100);
}

void loop() {
  delay(10);

  if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (myIMU.getSensorEvent() == true) {
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
      float quatI = myIMU.getQuatI();
      float quatJ = myIMU.getQuatJ();
      float quatK = myIMU.getQuatK();
      float quatReal = myIMU.getQuatReal();
      float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
      // Serial.print("Rotation ");
      // Serial.print(quatI, 2);
      // Serial.print(F(","));
      // Serial.print(quatJ, 2);
      // Serial.print(F(","));
      // Serial.print(quatK, 2);
      // Serial.print(F(","));
      // Serial.print(quatReal, 2);
      // Serial.print(F(","));
      // Serial.print(quatRadianAccuracy, 2);

      // Serial.println();
    
    }
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      float x1 = myIMU.getGyroX();
      float y1 = myIMU.getGyroY();
      float z1 = myIMU.getGyroZ();

      Serial.print("Gyroscope ");
      Serial.print(x1, 2);
      Serial.print(F(","));
      Serial.print(y1, 2);
      Serial.print(F(","));
      Serial.print(z1, 2);

      Serial.println();
    }
    
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
      float x2= myIMU.getAccelX();
      float y2 = myIMU.getAccelY();
      float z2 = myIMU.getAccelZ();
      Serial.print("Accelerometer ");
      Serial.print(x2, 2);
      Serial.print(F(","));
      Serial.print(y2, 2);
      Serial.print(F(","));
      Serial.print(z2, 2);

      Serial.println();
    }

    delay(500);

  }
}
