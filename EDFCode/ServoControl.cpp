#include "ServoControl.h"

void ServoControl::Init() {
  for (int i = 0; i < 4; i++) {
    servos[i].write(servoStartingPosition[i]);
    servos[i].attach(servoPins[i]);
  }
}

float ServoControl::WriteServoPosition(int servoNumber, float position) {
  if (position > maxPosition) {
    position = maxPosition;
  }
  if (position < -maxPosition) {
    position = -maxPosition;
  }
  servos[servoNumber].write(servoStartingPosition[servoNumber] + position);
  return position;
}
