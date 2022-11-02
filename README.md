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
    #include <HX711.h> // https://www.arduino.cc/reference/en/libraries/hx711-arduino-library/
*/

Gripper mygripper;

void setup() {
  // put your setup code here, to run once:

  // non-interruptible homing, required to zero motor step counts
  mygripper.homingSequence();
  
  mygripper.powerSaving = true; // motors turn off when stationary
  mygripper.disabled = false;   // false by default, true prevents motion

  // optional usb connection, USBSERIAL is defined as Serial in GripperCommunication.h
  USBSERIAL.begin(9600);

  // required bluetooth connection, BTSERIAL is defined as Serial2 in GripperCommunication.h
  BTSERIAL.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:

  // run the motors and communications for cycles of 20 milliseconds
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
