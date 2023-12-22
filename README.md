# luke-gripper-arduino

Arduino libraries I have written to run the gripper.
* ```Gripper_v2``` contains the main gripper class and functions used to control it.
* ```GripperCommunication``` defines bluetooth message settings, byte definitions, and message contents.
* ```StepperObj``` defines a stepper motor class to control at low level.

The arudino pins for I/O are defined in ```Gripper_v2/pin_definitions.h```. The Gripper_v2.h class also requires installing the HX711_Arudino_Library: https://www.arduino.cc/reference/en/libraries/hx711-arduino-library/

To use the gripper class, upload the following sketch to the arudino:

```c++
#include <Gripper_v3.h>

/* the following arduino libraries are required for Gripper_v3.h:
    #include <GripperCommunication.h>
    #include <StepperObj.h>
    #include <HX711.h> // https://www.arduino.cc/reference/en/libraries/hx711-arduino-library/
*/

Gripper_v3 mygripper;

void setup() {
  // put your setup code here, to run once:

  // configure settings
  mygripper.powerSaving = true;     // motors turn off when stationary
  mygripper.disabled = false;       // false by default, true prevents motion
  mygripper.timedActionSecs = 1.0;  // time interval for timed actions
  mygripper.debug = true;           // print debug messages

  // set frequency of actions
  mygripper.gauge_hz = 100;         // readings arrive at 80Hz
  mygripper.publish_hz = 20;        // publish readings and current state
  mygripper.serial_hz = 20;         // check for new instructions
  mygripper.motor_hz = 100000;      // check motor pulses in excess of max rpm

  // non-interruptible homing, required to zero motor step counts
  mygripper.homingSequence();

  // main serial connection, either wired or bluetooth (see GripperCommunication.h)
  BTSERIAL.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  // run the motors and input/output
  mygripper.run();

}
```
