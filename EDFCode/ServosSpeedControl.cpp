#include "ServosSpeedControl.h"

int ServosSpeedControl::currServo = -1;
uint16_t ServosSpeedControl::ServosTicks[nbServos] = {0, 0, 0, 0};

void ServosSpeedControl::Init() {
}

  