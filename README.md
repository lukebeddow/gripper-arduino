# luke-gripper-arduino

Arduino libraries I have written to run the gripper.

Gripper_v2 contains the main gripper class. GripperCommunication is a class to handle sending serial messages back and forth. StepperObj is a class to control a stepper motor, including safety controls.

To use the gripper class:

```c++
#include <Arduino.h>
#include <Gripper_v2.h>

Gripper mygripper;

void setup() {
  // non-interruptible homing sequence, required to calibrate motors
  mygripper.homingSequence();
  
  // setup serial connection, Serial2 is hardcoded in GripperCommunication as a global
  Serial2.begin(115200);
}

void loop() {
  // run smoothed cycles (minimise noise) for 20ms each
  mygripper.smoothRun(20);
}
```
