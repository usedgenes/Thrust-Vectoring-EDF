#include <math.h>
#include "CustomTime.h"
#include "Stabilization.h"

CustomTime myTime;
Stabilization stabilization;

// Initialiaze all sensors and communication pipes
void setup() {
  CustomSerialPrint::begin(230400);  // Console print: initialize serial communication

  stabilization.Init();
  myTime.Init();
}

// Main loop
void loop() {
  float loopTimeSec = 0.0;
  uint16_t loopNb = 0;
  float meanLoopTime = 0.0;

  loopTimeSec = myTime.GetloopTimeMilliseconds();

  // State Machine Initializing -> Ready -> AngleMode/AccroMode -> Safety -> Disarmed -> AngleMode/AccroMode
  stateMachine.Run(loopTimeSec);

  // Compute mean loop time and complementary filter time constant
  if (!stabilization.IsThrottleIdle()) {
    myTime.ComputeMeanLoopTime(loopTimeSec, meanLoopTime, loopNb);
  }
}
