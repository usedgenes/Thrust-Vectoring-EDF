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
String readString, servoT, servoR, servoB, servoL, edfString;

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
  while (Serial.available()) {
    delay(1);
    if (Serial.available() > 0) {
      char c = Serial.read();  //gets one byte from serial buffer
      readString += c;         //makes the string readString
    }
  }

  if (readString.length() > 0) {
    Serial.println(readString);  //see what was received

    // expect a string like 000000000000000
    edfString = readString.substring(0, 3); 
    servoT = readString.substring(3, 5);   
    servoR = readString.substring(5, 7);  
    servoB = readString.substring(7, 9);   
    servoL = readString.substring(9, 11);  

    int n0;
    int n1;  //declare as number
    int n2;
    int n3;
    int n4;

    char carray1[6];  //magic needed to convert string to a number
    edfString.toCharArray(carray1, sizeof(carray1));
    n0 = atoi(carray1);

    char carray2[6];
    servoT.toCharArray(carray2, sizeof(carray2));
    n1 = atoi(carray2);

    char carray3[6];
    servoT.toCharArray(carray3, sizeof(carray3));
    n2 = atoi(carray2);

    char carray4[6];
    servoT.toCharArray(carray4, sizeof(carray4));
    n3 = atoi(carray4);

    char carray5[6];
    servoT.toCharArray(carray5, sizeof(carray5));
    n4 = atoi(carray2);
    
    edf.w             rite(n0);
    servoTop.write(118+n1);
    servoRight.write(82+n2);
    servoBottom.write(100+n3);
    servoLeft.write(56+n4);
    readString = "";
  }
}
