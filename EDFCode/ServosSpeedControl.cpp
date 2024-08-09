#include "ServosSpeedControl.h"

int ServosSpeedControl::currServo = -1;
uint16_t ServosSpeedControl::servos[nbServos] = {0, 0, 0, 0};

void ServosSpeedControl::Init() {
  test = 1;
}


void ServosSpeedControl::updatePosition(int _id, float position) {
  if(position < MIN_POSITION) {
    servos[_id] = MIN_POSITION;
  }
  else if(position > MAX_POSITION) {
    servos[_id] = MAX_POSITION;
  } else {
    servos[_id] = POSITION;
  }
}