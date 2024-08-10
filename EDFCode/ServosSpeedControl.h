#ifndef SEROVSSPEEDCONTROL_H_
#define SERVOSSPEEDCONTROL_H_
#include <ESP32Servo.h>

#include "CustomSerialPrint.h"

enum ServoId { Servo0, Servo1, Servo2, Servo3 };

class ServosSpeedControl {
  private:
    static const int nbServos = 4;
    const int MIN_POSITION = 0;
    const int MAX_POSITION = 60;             
    Servo *servos;
    int servoPorts[4] = {12, 13, 14, 27};
    int servoStartingPosition[4] = {148, 112, 130, 86};
  public:
    void Init();
    void UpdatePosition(int _id, float position);
    const int GetServosMaxPosition() {
        return MAX_POSITION;
    }
    const int GetServosMinPosition() {
        return MIN_POSITION;
    }

};

#endif // ServoSPEEDCONTROL_H_