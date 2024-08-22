#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#include "ESP32Servo.h"

//	ESP facing top
//    	12
//27		      13
//      14

//12: 105 neutral
//13: 110 neutral
//14: 110 neutral
//27: 135 neutral

class ServoControl {
private:
  int servoPins[4] = { 12, 13, 14, 27 };
  int servoStartingPosition[4] = { 105, 110, 110, 135 };
  int maxPosition = 30;
public:
  Servo servos[4];
  void Init();
  float WriteServoPosition(int servoNumber, float position);
};

#endif  