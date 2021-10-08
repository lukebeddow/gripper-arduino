#pragma once

// libraries to include
#include <StepperObj.h>				// this is a library that I made
#include <GripperCommunication.h>	// this is a library that I made
#include <HX711.h>					// external library for HX711 chip

// define pins
#include "pin_definitions.h"

// define the gripper class
class Gripper
{
public:

    /* ----- Public Variables ----- */

    // define motor parameters
    struct MotorParam {

        int stepsPerRev = 200;    // step angle of 1.8deg -> 200 steps per rev

        // motor microstep modes, 1=fullstep, 2=halfstep, 4=quarterstep, ...etc... up to 16
        struct Microstep {
            int x = 2;
            int y = 2;
            int z = 2;
        };
        Microstep microstep;

        // maximum speeds for each motor in rpm
        struct MaxSpeed {
            float x = 300;
            float y = 300;
            float z = 350;
        };
        MaxSpeed maxSpeed;

        // homing speeds for each motor in rpm
        struct HomingSpeed {
            float x = 400;
            float y = 400;
            float z = 450;
        };
        HomingSpeed homingSpeed;

        // direction of rotation wrt to the limit switch
        struct Clockwise {
            bool x = true;
            bool y = false;
            bool z = false;
        };
        Clockwise clockwise;

        // how many revolutions from the limit switch is home
        struct HomeRevs {
            float x = 1.0;
            float y = 1.0;
            float z = 1.0;
        };
        HomeRevs homeRevs;

        // how many revolutions from the limit switch is the limit
        struct LimitRevs {
            float x = 32.0; // hard limit at 32.5 (13000 steps)
            float y = 32.0;
            float z = 34.0;
        };
        LimitRevs limitRevs;

        // which motors are currently in use
        struct InUse {
            bool x = true;
            bool y = true;
            bool z = true;
        };
        InUse inUse;

        // has the target been reached
        struct TargetReached {
            bool x = false;
            bool y = false;
            bool z = false;
        };
        TargetReached targetReached;

    };
    MotorParam m;

    // motor control inputs
    struct Control {
        /* This structure contains control commands used by the gripper */

        // input speeds
        struct Rpm {
            float x;
            float y;
            float z;
        };
        Rpm rpm;

        // step position input
        struct StepTarget {
            int x;
            int y;
            int z;
        };
        StepTarget stepTarget;

        // motor state position
        float radius = 0.0;                        // in mm (range 50 to 134mm)
        float angle = 0.0;                         // in deg (range -40 to +40deg)
        float palm = 0.0;                          // in mm (range 0 to 160mm)

        // instruction command
        byte instructionByte;
    };
    Control control;

    // gripper physical properties
    struct Params {
        /* This structure contains physical constants */
        static constexpr float xy_lead = 4;        // lead in mm for x and y screws
        static constexpr float z_lead = 4.8768;    // lead in mm for z screw
        static constexpr float xy_gear_red = 1.5;  // gear reduction to xy screws
        static constexpr float xy_max = 134;       // home displacement value in mm
        static constexpr float xy_diff = 35;       // distance between screws in mm
        static constexpr float z_max = 160;        // home displacement value in mm
    };
    Params params;

    // operational parameters
    struct {
        float x;
        float y;
        float z;
    } mmPerStep;


    // create motor objects
    StepperObj motorX{ xstep, xdir };
    StepperObj motorY{ ystep, ydir };
    StepperObj motorZ{ zstep, zdir };

    // create strain gauge objects
    HX711 gauge1;
    HX711 gauge2;
    HX711 gauge3;

    // create input/output stream
    GripperCommunication iostream;

    bool targetReached;
    bool powerSaving;

    // booleans to save whether we have unpublished gauge data
    bool newReadGauge1;
    bool newReadGauge2;
    bool newReadGauge3;

    /* ----- Private Variables ----- */
private:
    int operatingMode;

    /* ----- Public Functions ----- */
public:
    Gripper();
    bool readJoystick();
    void homingSequence();
    void home();
    void prepareTarget(float radius, float angle, float palm);
    bool readGauges();
    bool checkSerial();
    void runMotors(const int loopMillis);
    void motorEnable(bool is_enabled);
    void setOutputMessagePosition();
    void checkInputs();

    /* ----- Private Functions ----- */
private:
    void setupMotors();
    void setPins();
};
