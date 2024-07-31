#include <ESP32Servo.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)
Servo myservo;
int num;
void setup() {
  Serial.begin(115200);
  myservo.attach(13);
  // Set up oversampling and filter initialization
}


void loop() {


while(Serial.available()>0)
{
num= Serial.parseInt();
Serial.println(num);
}
myservo.write(num);


}
