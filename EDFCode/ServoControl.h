#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#include "ESP32Servo.h"

//	ESP facing top
//    	12
//27		      13
//      14

//12: neutral
//13: neutral
//14: neutral
//27: 86 neutral

class ServoControl {
private:
  Servo servos[4];
  int servoPins[4] = { 12, 13, 14, 27};
  int servoStartingPosition[4] = {100, 100, 100, 100};
  int maxPosition = 30;
public:
  void Init();
  void WriteServoPosition(int servoNumber, int position);
};

#endif  // INERTIALMEASUREMENTUNIT_H_