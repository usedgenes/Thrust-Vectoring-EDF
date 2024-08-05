#ifndef SEROVSSPEEDCONTROL_H_
#define SERVOSSPEEDCONTROL_H_

#include "CustomSerialPrint.h"

// converts microseconds to tick (assumes prescale of 8)
#define usToTicks(_us) ((clockCyclesPerMicrosecond() * _us))
// converts from ticks back to microseconds
#define ticksToUs(_ticks) (((unsigned)_ticks) / clockCyclesPerMicrosecond())

enum ServoId { Servo0, Servo1, Servo2, Servo3 };

class ServosSpeedControl {
  private:
    static const int nbServos = 4;
    const int MIN_POSITION = 0;
    const int MAX_POSITION = 60;             
    static uint16_t ServosTicks[nbServos];
    static int currServo;

  public:
    void Init();
    const int GetServosMaxPosition() {
        return MAX_POSITION;
    }
    const int GetServosMinPosition() {
        return MIN_POSITION;
    }
};

#endif // ServoSPEEDCONTROL_H_