#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SD.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

Adafruit_BMP3XX bmp;
BNO08x myIMU;
File myFile;
Servo servoLeft;
Servo servoRight;
Servo servoTop;
Servo servoBottom;

#define BNO08X_CS   5
#define BNO08X_INT  A4
#define BNO08X_RST  A5

void setup() {
  
  if (myIMU.beginSPI(BNO08X_CS, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
    setReports();
  float quatI = myIMU.getQuatI();
      float quatJ = myIMU.getQuatJ();
      float quatK = myIMU.getQuatK();
      float quatReal = myIMU.getQuatReal();
      float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
  delay(100); // This delay allows enough time for the BNO086 to accept the new 
              // configuration and clear its reset status
}


float pid(float currentAltitude, unsigned long currentTime) {
  unsigned long dt = currentTime - previousTime;
  if(dt == 0) {
    return 0;
  }
  previousTime = currentTime;
  float error = adjustedTargetAltitude - currentAltitude;
  float derivativeError = (error - previousError) / dt;
  integralError += error * dt;
  float output = Kp*error + Ki*integralError + Kd*derivativeError;
  previousError = error;

  if (output > MAX_OUTPUT) {
    output = MAX_OUTPUT;
  }
  else if (output < MIN_OUTPUT) {
    output = MIN_OUTPUT;
  }

  pidLogger(currentTime, dt, error, derivativeError, integralError, output);
}

void logger(unsigned long time, float altitude, float pressure, int servoPosition) {
  myFile = SD.open("ALTITUDE.TXT", FILE_WRITE);
 
  if (myFile) {
    myFile.print(time);
    myFile.print(" ");
    myFile.print(altitude);
    myFile.print(" ");
    myFile.print(pressure);
    myFile.print(" ");
    myFile.println(servoPosition);
    myFile.close();
  } 
}


void pidLogger(unsigned long time, unsigned long dt, float error, float derivativeError, float integralError, float output) {
  myFile = SD.open("PID.TXT", FILE_WRITE);
 
  if (myFile) {
    myFile.print(time);
    myFile.print(" ");
    myFile.print(dt);
    myFile.print(" ");
    myFile.print(error);
    myFile.print(" ");
    myFile.print(derivativeError);
    myFile.print(" ");
    myFile.println(integralError);
    myFile.print(" ");
    myFile.println(output);
    myFile.close();
  } 
}
