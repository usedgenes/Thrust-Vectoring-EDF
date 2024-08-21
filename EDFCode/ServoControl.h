#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#include "ESP32Servo.h"

//	ESP facing top
//    	12
//27		      13
//      14

//12: 80 neutral
//13: 105 neutral
//14: 80 neutral
//27: 135 neutral

class ServoControl {
private:
  int servoPins[4] = { 12, 13, 14, 27 };
  int servoStartingPosition[4] = { 80, 105, 80, 135 };
  int maxPosition = 20;
public:
  Servo servos[4];
  void Init();
  float WriteServoPosition(int servoNumber, float position);
};

#endif  