#include <Gripper_v2.h>

Gripper mygripper;

void setup() {
  // put your setup code here, to run once:

  mygripper.homingSequence();
  mygripper.powerSaving = true;

  // connection with computer via usb, optional
  Serial.begin(9600);

  // bluetooth, 86us per byte at 115200, so 10 bytes = 0.9ms
  Serial2.begin(115200);

}

//unsigned long t0 = 0;
//unsigned long t1 = 0;
//unsigned long diff = 0;

void loop() {
  // put your main code here, to run repeatedly:

  // check for serial, joystick, or HX711 communication
  mygripper.checkInputs();

  // pulse the motors for 20 milliseconds
  mygripper.runMotors(20);
  
}
