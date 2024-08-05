#include "ServosSpeedControl.h"

int ServosSpeedControl::currServo = -1;
uint16_t ServosSpeedControl::ServosTicks[nbServos] = {0, 0, 0, 0};

void ServosSpeedControl::Init() {
    // Set Servos pin PD4, PD5, PD6, PD7 as outputs
    InitTimer1();
    Idle();
}

// Timer interrupt to set PWM to Servos controllers
typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t;

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn,
                                     volatile uint16_t *OCRnA) {
    ServosSpeedControl::ApplySpeed(TCNTn, OCRnA);
}


// Set a falling edge for the previous Servo, and a rising edge for the current Servo
// Servo speed is managed by the pulse with: the larger the high level is, the faster the Servo run
void ServosSpeedControl::ApplySpeed(volatile uint16_t *TCNTn, volatile uint16_t *OCRnA) {
  
}