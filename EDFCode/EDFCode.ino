#include "Stabilization.h"

Stabilization stabilization;

unsigned long loopTime;
unsigned long previousTime;

void setup() {
  Serial.begin(115200);  // Console print: initialize serial communication

  stabilization.Init();

  loopTime = 0;
}

// Main loop
void loop() {
  previousTime = loopTime;
  loopTime = millis(); 

  float deltaTime = (loopTime-previousTime)/1000.0;
  stabilization.Angle(deltaTime);

  Serial.print("Loop time: ");
  Serial.println(deltaTime);

  delay(1000);
}
