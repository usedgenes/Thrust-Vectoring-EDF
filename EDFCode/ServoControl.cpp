#include "ServoControl.h"

void ServoControl::Init() {
  for (int i = 0; i < 4; i++) {
    servos[i].attach(servoPins[i]);
  }
}

void ServoControl::WriteServoPosition(int servoNumber, int position) {
  if (position > maxPosition) {
    position = maxPosition;
  }
  if (position < -maxPosition) {
    position = -maxPosition;
  }
  servos[servoNumber].write(servoStartingPosition[servoNumber] + position);
}
