#include "ServosSpeedControl.h"

int ServosSpeedControl::currServo = -1;
uint16_t ServosSpeedControl::servos[nbServos] = {0, 0, 0, 0};

void ServosSpeedControl::Init() {
  CustomSerialPrint::println("ServosSpeedControl initialized");
  test = 1;
}


void ServosSpeedControl::UpdatePosition(int _id, float position) {
  if(position < MIN_POSITION) {
    servos[_id] = MIN_POSITION;
  }
  else if(position > MAX_POSITION) {
    servos[_id] = MAX_POSITION;
  } else {
    servos[_id] = position;
  }
}