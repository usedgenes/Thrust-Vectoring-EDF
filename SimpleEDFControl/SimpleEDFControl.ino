#include <ESP32Servo.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)
Servo edf;
Servo servoTop;
Servo servoRight;
Servo servoBottom;
Servo servoLeft;

bool firstNumber = true;
int servoNumber;

int num;
void setup() {
  Serial.begin(115200);
  edf.attach(15);
  servoTop.attach(12);
  servoRight.attach(13);
  servoBottom.attach(14);
  servoLeft.attach(27);
  // Set up oversampling and filter initialization
}


void loop() {


while(Serial.available()>0)
{
num= Serial.parseInt();
if (firstNumber) {
  firstNumber = false;
  servoNumber = num;
  break;
}
else {
  if(servoNumber == 0) {
    edf.write(num);
  }
  if(servoNumber == 1) {
    servoTop.write(118 + num);
  }
  if(servoNumber == 2) {
    servoRight.write(82 + num);
  }
  if(servoNumber == 3) {
    servoBottom.write(100 + num);
  }
  if(servoNumber == 4) {
    servoLeft.write(56 + num);
  }
}
}

}
