#include "ServosSpeedControl.h"

void ServosSpeedControl::Init() {
  servos = new Servo[nbServos];
  for(int i = 0; i < nbServos; i++) {
    servos[i].attach(servoPorts[i]);
  }
}


void ServosSpeedControl::UpdatePosition(int i, float position) {
  if(position < MIN_POSITION) {
    servos[i].write(MIN_POSITION + servoStartingPosition[i]);
  }
  else if(position > MAX_POSITION) {
    servos[i].write(MAX_POSITION + servoStartingPosition[i]);
  } else {
    servos[i].write(position + servoStartingPosition[i]);
  }
}