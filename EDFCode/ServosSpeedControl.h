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
    const int MIN_POWER = 1060;
    const int MAX_POWER = 1860;             // Max pwr available. Set to 1860 to reach max
    const int MAX_THROTTLE_PERCENT = 100.0; // Percent to restrain max Servo power
    uint16_t MAX_THROTTLE = MAX_POWER * (MAX_THROTTLE_PERCENT / 100.0); // Restrained max power
    int IDLE_THRESHOLD = 1100;
    static uint16_t ServosTicks[nbServos];
    static int currServo;

  public:
    void Init();
    void UpdateSpeed(int _id, float _PWM);
    void Idle();
    static void ApplySpeed(volatile uint16_t *TCNTn, volatile uint16_t *OCRnA);
    const int GetServosMaxPower() {
        return MAX_POWER;
    }
    const int GetServosMinPower() {
        return MIN_POWER;
    }
    const int GetServosMaxThrottlePercent() {
        return MAX_THROTTLE_PERCENT;
    }
    const int GetServosMaxThrottle() {
        return MAX_THROTTLE;
    }
    int GetServosIdleThreshold() {
        return IDLE_THRESHOLD;
    }

};

#endif // ServoSPEEDCONTROL_H_