# luke-gripper-arduino

Arduino libraries I have written to run the gripper.
* ```Gripper_v2``` contains the main gripper class and functions used to control it.
* ```GripperCommunication``` defines bluetooth message settings, byte definitions, and message contents.
* ```StepperObj``` defines a stepper motor class to control at low level.

The arudino pins for I/O are defined in ```Gripper_v2/pin_definitions.h```. The Gripper_v2.h class also requires installing the HX711_Arudino_Library: https://www.arduino.cc/reference/en/libraries/hx711-arduino-library/

To use the gripper class, upload the following sketch to the arudino:

```c++

#include <Gripper_v2.h>

/* the following arduino libraries are required for Gripper_v2.h:
    #include <GripperCommunication.h>
    #include <StepperObj.h>
    #include <HX711.h>
*/

Gripper mygripper;

void setup() {

  // non-interruptible homing sequence, required to calibrate motors
  mygripper.homingSequence();

  // connection with computer via usb, optional but useful for debugging
  Serial.begin(9600);
  
  // setup serial connection, Serial2 is hardcoded in GripperCommunication as a global
  Serial2.begin(115200);
  
}

void loop() {

  // run smoothed cycles (minimise noise) for 20ms each
  mygripper.smoothRun(20);
  
}
```
Rather than use smoothRun, the gripper can also be run using:

```c++

  // check for commands and gauge data
  mygripper.checkInputs();
  
  // publish a message if a new one is ready
  mygripper.publishOutput();
  
  // run the motors for a cycle of 20ms
  mygripper.runMotors(20);
```
Finally, the gripper can be put into power saving mode, or disabled - which means the motors cannot move but other functions (I/O) continues:

```c++

  // motors are switched off when not moving
  mygripper.powerSaving = true;
  
  // motors are prevented from moving
  mygripper.disabled = true
```

