#pragma once

// libraries to include
#include <StepperObj.h>				    // this is a library that I made
#include <GripperCommunication.h>	// this is a library that I made
#include <HX711.h>					      // external library for HX711 chip

// define pins
#include "pin_definitions_pcb_v3.h"

// define the gripper class
class Gripper_v3
{
private:

    /* ----- Key motor variables ----- */

    // define motor parameters
    static struct MotorParam {

        static constexpr int stepsPerRev = 200;    // step angle of 1.8deg -> 200 steps per rev

        // motor microstep modes, 1=fullstep, 2=halfstep, 4=quarterstep, ...etc... up to 16
        static struct Microstep {
            static constexpr int x = 4;
            static constexpr int y = 4;
            static constexpr int z = 4;
        } microstep;

        // maximum speeds for each motor in rpm
        static struct MaxSpeed {
            static constexpr float x = 200;
            static constexpr float y = 200;
            static constexpr float z = 400;
        } maxSpeed;

        // homing speeds for each motor in rpm
        static struct HomingSpeed {
            static constexpr float x = 350;
            static constexpr float y = 350;
            static constexpr float z = 400;
        } homingSpeed;

        // direction of rotation wrt to the limit switch
        static struct Clockwise {
            static constexpr bool x = true;
            static constexpr bool y = false;
            static constexpr bool z = false;
        } clockwise;

        // how many revolutions from the limit switch is home
        static struct HomeRevs {
            static constexpr float x = 0.5; // 1.0
            static constexpr float y = 0.5; // 1.0
            static constexpr float z = 1.0;
        } homeRevs;

        // how many revolutions from the limit switch is the limit
        static struct LimitRevs {
            static constexpr float x = 20.0; //22.0; // with microstep 4, 46mm is 17600 steps, hence 22 revs
            static constexpr float y = 21.0; //22.0;
            static constexpr float z = 34.0; // with microstep 4, 165mm is 28000 steps, hence 35 revs
        } limitRevs;

        // which motors are currently in use
        static struct InUse {
            static constexpr bool x = true;
            static constexpr bool y = true;
            static constexpr bool z = true;
        } inUse;

    } m;

    // gripper physical properties all in mm, (depends on MotorParam)
    static struct Params {

        // leadscrew distance apart, for calculating finger angle
        static constexpr float screwDistance_xy = 35;

        // home position in mm
        static struct Home {
            static constexpr float x = 134;         // set this
            static constexpr float y = x;
            static constexpr float z = 0;           // set this
        } home;

        // direction of increasing motor steps
        static struct Direction {
            static constexpr int x = -1;            // set this
            static constexpr int y = x;
            static constexpr int z = 1;             // set this
        } direction;

        // lead in mm
        static struct Lead {
            static constexpr float x = 4;           // set this
            static constexpr float y = x;
            static constexpr float z = 4.8768;      // set this
        } lead;

        // gear reduction from motor to screw
        static struct GearReduction {
            static constexpr float x = 1.0;         // set this
            static constexpr float y = x;
            static constexpr float z = 1;           // set this
        } gearReduction;

        // mm increment per revolution of the motor
        static struct MMPerRev{
            static constexpr float x = lead.x / gearReduction.x;
            static constexpr float y = lead.y / gearReduction.y;
            static constexpr float z = lead.z / gearReduction.z;
        } mmPerRev;

        // mm travel of the screws per step of the stepper motors, with direction
        static struct MMPerStep {
            static constexpr float x = 
                (mmPerRev.x / (m.stepsPerRev * m.microstep.x)) * direction.x;
            static constexpr float y = 
                (mmPerRev.y / (m.stepsPerRev * m.microstep.y)) * direction.y;
            static constexpr float z = 
                (mmPerRev.z / (m.stepsPerRev * m.microstep.z)) * direction.z;
        } mmPerStep;

    } params;

    // motor control commands
    struct Control {

        // speed the motors will currently turn, used with joystick
        struct Rpm {
            float x;
            float y;
            float z;
        } rpm;

        // step position input
        struct StepTarget {
            int x;
            int y;
            int z;
        } stepTarget;

        // motor state position
        float radius = 0.0;               // in mm (range 50 to 134mm)
        float angle = 0.0;                // in deg (range -40 to +40deg)
        float palm = 0.0;                 // in mm (range 0 to 160mm)

        // instruction command
        byte instructionByte;

    } control;

    // create motor objects
public:
    StepperObj motorX { xstep, xdir };
    StepperObj motorY { ystep, ydir };
    StepperObj motorZ { zstep, zdir };

    // create strain gauge objects
    HX711 gauge1;
    HX711 gauge2;
    HX711 gauge3;
    HX711 gauge4;

    // create input/output stream
    GripperCommunication iostream;

    // has the target been reached
    struct TargetReached {
        bool x = false;
        bool y = false;
        bool z = false;
        bool all = false;
    } targetReached;

    // what speed has been set via input message, default to max speed
    struct SetSpeed {
        float x;
        float y;
        float z;
    } setSpeed;

    // flag indicating operation mode, 0=serial, 1=joystick, 2=homing
    int operatingMode;

    // booleans to save whether we have unpublished gauge data
    bool newReadGauge1;
    bool newReadGauge2;
    bool newReadGauge3;
    bool newReadGauge4;

    // debug info
    float runHz = 0.0;
    float actualGaugeHz = 0.0;
    float actualPublishHz = 0.0;
    float actualSerialHz = 0.0;
    float actualMotorHz = 0.0;
    unsigned long lastGaugeTime = 0;
    unsigned long lastSerialTime = 0;
    unsigned long lastPublishTime = 0;
    unsigned long lastMotorTime = 0;

    bool timedActionInProgress = false;
    unsigned long timedActionStart_ms = 0;

    // action rates
    unsigned long motor_wait_us = 0;
    unsigned long gauge_wait_us = 0;
    unsigned long publish_wait_us = 0;
    unsigned long serial_wait_us = 0;

    /* ----- Public variables ----- */
public:

    bool powerSaving = true;        // motors are turned off when not moving
    bool disabled = false;          // motors are prevented from moving
    bool debug = false;             // print debug messages in Serial
    float timedActionSecs = 1.0;    // if doing timed commands, how long is the time interval
    float timedActionPubEarly = 0.1;    // publish target reached early by x seconds if doing a timed action

    float gauge_hz = 100;           // readings appear at 80Hz
    float publish_hz = 10;          // publish as new gauge data arrives
    float serial_hz = 20;           // check for new instructions
    float motor_hz = 20000;         // check the motors much faster than they need to move

    /* ----- Public Functions ----- */
public:
    Gripper_v3();
    void homingSequence();
    void readGauges();
    void checkInputs();
    void sendErrorMessage(byte error_code);
    void publishOutput();
    void runMotors(const int loopMillis);
    void run();
    void run(unsigned long runtime_ms);
    void setGripperTarget(float x, float y, float z);
    void print();
    void update_rates();

    /* ----- Private Functions ----- */
private:
    void setupMotors();
    void setPins();
    void setMotorPositions();
    bool readJoystick();
    void readGauge(const int gauge_num);
    bool checkSerial();
    void setHomeTarget();
    void setSpeedTarget();
    void setMessageTarget();
    void motorEnable(bool is_enabled);
};
