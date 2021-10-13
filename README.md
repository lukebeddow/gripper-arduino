# luke-gripper-arduino

Arduino libraries I have written to run the gripper.

Gripper_v2 contains the main gripper class. GripperCommunication is a class to handle sending serial messages back and forth. StepperObj is a class to control a stepper motor, including safety controls.

The arudino pins for I/O are defined in Gripper_v2/pin_definitions.h. The Gripper_v2.h class also requires installing the HX711_Arudino_Library: https://www.arduino.cc/reference/en/libraries/hx711-arduino-library/

To use the gripper class, upload the following sketch to the arudino:

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
