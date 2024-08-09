#ifndef SEROVSSPEEDCONTROL_H_
#define SERVOSSPEEDCONTROL_H_

#include "CustomSerialPrint.h"

enum ServoId { Servo0, Servo1, Servo2, Servo3 };

class ServosSpeedControl {
  private:
    static const int nbServos = 4;
    const int MIN_POSITION = 0;
    const int MAX_POSITION = 60;             
    static uint16_t servos[nbServos];
    static int currServo;
    int test = 0;

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