#include <Arduino.h>
#include <Gripper_v2.h>

Gripper::Gripper()
{
    /* Class constructor */

    setupMotors();
    setPins();

    // begin running HX711 chips to monitor gauges
    gauge1.begin(SG1_dt, SG1_sck);
    gauge2.begin(SG2_dt, SG2_sck);
    gauge3.begin(SG3_dt, SG3_sck);
    gauge4.begin(SG4_dt, SG4_sck);

    // do we apply a scaling and offset here? not currently
    // gauge1.set_scale(loadcell_divider);
    // gauge1.set_offset(loadcell_offset);

    // set flags to defaults (0=joystick, 1=serial)
    operatingMode = 1;
    targetReached.all = true;
    newReadGauge1 = false;
    newReadGauge2 = false;
    newReadGauge3 = false;
    newReadGauge4 = false;

    // default optional parameters
    powerSaving = true;
    disabled = false;
    debug = true;

    // speed defaults
    setSpeed.x = m.maxSpeed.x;
    setSpeed.y = m.maxSpeed.y;
    setSpeed.z = m.maxSpeed.z;

    // this code is redundant I think
    in_use.gauge1 = true;
    in_use.gauge2 = true;
    in_use.gauge3 = true;
    in_use.gauge4 = true;
    // waitGauge();
}

void Gripper::waitGauge()
{
    unsigned long timeout_ms = 20;
    unsigned long delay_ms = 2;
    unsigned long start_ms = millis();

    // USBSERIAL.begin(9600);

    int loops = timeout_ms / delay_ms;

    // while (millis() - start_ms < timeout_ms) {
    for (int i = 0; i < loops; i++) {

        // USBSERIAL.print("loop ");
        // USBSERIAL.println(i);

        if (not in_use.gauge1) {
            in_use.gauge1 = gauge1.is_ready();
        }

        if (not in_use.gauge2) {
            in_use.gauge2 = gauge2.is_ready();
        }

        if (not in_use.gauge3) {
            in_use.gauge3 = gauge3.is_ready();
        }

        if (not in_use.gauge4) {
            in_use.gauge4 = gauge4.is_ready();
        }

        // delay(delay_ms);

    }

    // print();

    return;
}

void Gripper::setupMotors()
{
    /* This initialisation function configures the motors */

    // set steps per revolution, accounting for microstepping mode
    motorX.setStepsPerRev(m.stepsPerRev * m.microstep.x);
    motorY.setStepsPerRev(m.stepsPerRev * m.microstep.y);
    motorZ.setStepsPerRev(m.stepsPerRev * m.microstep.z);

    // setup motor workspace and configure limit switches
    motorX.setupWorkspace(m.homeRevs.x, m.limitRevs.x, xlim, true);
    motorY.setupWorkspace(m.homeRevs.y, m.limitRevs.y, ylim, true);
    motorZ.setupWorkspace(m.homeRevs.z, m.limitRevs.z, zlim, true);

    // initialise to zero RPM
    motorX.setRPM(0);
    motorY.setRPM(0);
    motorZ.setRPM(0);

    // set homing speeds
    motorX.setHomingRPM(m.homingSpeed.x);
    motorY.setHomingRPM(m.homingSpeed.y);
    motorZ.setHomingRPM(m.homingSpeed.z);

    // orientate the motor wrt the limit switches
    motorX.setClockwise(m.clockwise.x);
    motorY.setClockwise(m.clockwise.y);
    motorZ.setClockwise(m.clockwise.z);

    // // calculate the mm per step parameters, this is never updated afterwards!
    // mmPerStep.x = params.mmPerRev.x / (m.stepsPerRev * m.microstep.x);
    // mmPerStep.y = params.mmPerRev.y / (m.stepsPerRev * m.microstep.y);
    // mmPerStep.z = params.mmPerRev.z / (m.stepsPerRev * m.microstep.z);

    // // adjust whether we gain +ve or -ve position from a step increase
    // mmPerStep.x *= params.direction.x;
    // mmPerStep.y *= params.direction.y;
    // mmPerStep.z *= params.direction.z;
}

void Gripper::setPins()
{
    /* This initialisation function configures the arduino pins */

    /* CNC shield pins */

    // analogue pins
    pinMode(reset, INPUT_PULLUP);
    pinMode(feed_hold, INPUT_PULLUP);
    pinMode(cycle_start, INPUT_PULLUP);
    pinMode(coolant_enable, OUTPUT); digitalWrite(coolant_enable, LOW);

    // digital pins
    pinMode(xstep, OUTPUT); digitalWrite(xstep, LOW);
    pinMode(ystep, OUTPUT); digitalWrite(ystep, LOW);
    pinMode(zstep, OUTPUT); digitalWrite(zstep, LOW);
    pinMode(xdir, OUTPUT); digitalWrite(xdir, LOW);
    pinMode(ydir, OUTPUT); digitalWrite(ydir, LOW);
    pinMode(zdir, OUTPUT); digitalWrite(zdir, LOW);

    pinMode(motor_enable, OUTPUT); digitalWrite(motor_enable, LOW); // LOW = motors enabled
    pinMode(xlim, INPUT_PULLUP);
    pinMode(ylim, INPUT_PULLUP);
    pinMode(zlim, INPUT_PULLUP);
    pinMode(spindle_enable, OUTPUT); digitalWrite(spindle_enable, LOW);
    pinMode(spindle_dir, OUTPUT); digitalWrite(spindle_dir, LOW);

    /* Joystick and button pins */

    // button pins
    pinMode(button_gnd, OUTPUT); digitalWrite(button_gnd, LOW);
    pinMode(blue_enable, INPUT_PULLUP);
    pinMode(white_forwards, INPUT_PULLUP);
    pinMode(green_backwards, INPUT_PULLUP);

    // joystick pins
    pinMode(joystick_gnd, OUTPUT); digitalWrite(joystick_gnd, LOW);
    pinMode(joystick_power, OUTPUT); digitalWrite(joystick_power, HIGH);
    pinMode(joystick_push, INPUT_PULLUP);

    /* Bluetooth pins */

    pinMode(bt_enable, OUTPUT); digitalWrite(bt_enable, LOW);
    pinMode(bt_state, OUTPUT); digitalWrite(bt_state, LOW);

    /* HX711 pins */
    // handled by the library
}

bool Gripper::readJoystick()
{
    /* Read to see if the joystick is enabled, and what it is set to */

    // is the joystick connected and enabled
    if (digitalRead(blue_enable) == HIGH) {
        // joystick is off
        control.rpm.x = 0.0;
        control.rpm.y = 0.0;
        control.rpm.z = 0.0;
        // we are not in joystick mode
        if (operatingMode == 0) {
            operatingMode = 1;
            targetReached.all = false;
            targetReached.x = false;
            targetReached.y = false;
            targetReached.z = false;
        }
        return false;
    }

    // are we within a homing sequence? Do not interrupt
    if (operatingMode == 2 and not targetReached.all) {
        return false;
    }

    // otherwise, the joystick is connected and enabled
    operatingMode = 0;

    // joystick overrides disabled signal
    if (disabled) {
        disabled = false;
        motorEnable(true);
    }

    // joystick button triggers homing sequence
    if (digitalRead(joystick_push) == LOW) {
        //homingSequence();
        setHomeTarget();
        return false;
    }

    // read the joystick position, gives unsigned 10bit value (0 to 1023)
    int vref_X = analogRead(joystick_x);
    int vref_Y = analogRead(joystick_y);

    // convert this into a signed speed for each motor
    control.rpm.x = (vref_X - 522) * (m.maxSpeed.x / 522);
    control.rpm.y = (vref_Y - 522) * (m.maxSpeed.y / 522);

    // if speed is too low, we don't move at all
    if (abs(control.rpm.x) < 50) {
        control.rpm.x = 0;
    }
    if (abs(control.rpm.y) < 50) {
        control.rpm.y = 0;
    }

    // finally, check the z motor button
    if (digitalRead(white_forwards) == LOW) {
        control.rpm.z = m.maxSpeed.z;
    }
    else if (digitalRead(green_backwards) == LOW) {
        control.rpm.z = -1 * m.maxSpeed.z;
    }
    else {
        control.rpm.z = 0;
    }

    // set the motor speeds
    motorX.setRPM(control.rpm.x);
    motorY.setRPM(control.rpm.y);
    motorZ.setRPM(control.rpm.z);

    return true;
}

void Gripper::homingSequence()
{
    /* Home all of the motors of the gripper, non-interruptible */

    // if gripper is in disabled mode, do not move any motors
    if (disabled) {
        return;
    }

    // which motors do we need to home
    bool x_homed = not m.inUse.x;
    bool y_homed = not m.inUse.y;
    bool z_homed = not m.inUse.z;

    // loop until all motors are homed
    while (not (x_homed and y_homed and z_homed)) {
        if (not x_homed) x_homed = motorX.homePulse();
        if (not y_homed) y_homed = motorY.homePulse();
        if (not z_homed) z_homed = motorZ.homePulse();
    }

    targetReached.all = true;
    targetReached.x = x_homed;
    targetReached.y = y_homed;
    targetReached.z = z_homed;
}

void Gripper::setHomeTarget()
{
    /* This function prepares for homing, afterwards runMotors should be used */

    if (debug) {
        USBSERIAL.print("Setting home target\n");
    }

    // in case a previous homing sequence was interrupted
    motorX.wipeHomingFlag();
    motorY.wipeHomingFlag();
    motorZ.wipeHomingFlag();
    
    // set gripper parameters for homing mode
    operatingMode = 2;
    targetReached.x = false;
    targetReached.y = false;
    targetReached.z = false;
    targetReached.all = false;
}

void Gripper::setSpeedTarget()
{
    /* save a speed target */

    if (debug) {
        USBSERIAL.print("Incoming speed request (x, y, z) of: (");
        USBSERIAL.print(iostream.inputMessage.x);
        USBSERIAL.print(", ");
        USBSERIAL.print(iostream.inputMessage.y);
        USBSERIAL.print(", ");
        USBSERIAL.print(iostream.inputMessage.z);
        USBSERIAL.print(")\n");
    }

    setSpeed.x = iostream.inputMessage.x;
    setSpeed.y = iostream.inputMessage.y;
    setSpeed.z = iostream.inputMessage.z;

    // safety checks
    if (setSpeed.x < 0) setSpeed.x = 0;
    if (setSpeed.x > m.maxSpeed.x) setSpeed.x = m.maxSpeed.x;

    if (setSpeed.y < 0) setSpeed.y = 0;
    if (setSpeed.y > m.maxSpeed.y) setSpeed.y = m.maxSpeed.y;

    if (setSpeed.z < 0) setSpeed.z = 0;
    if (setSpeed.z > m.maxSpeed.z) setSpeed.z = m.maxSpeed.z;

    if (debug) {
        USBSERIAL.print("Speed target (x, y, z) set: (");
        USBSERIAL.print(setSpeed.x);
        USBSERIAL.print(", ");
        USBSERIAL.print(setSpeed.y);
        USBSERIAL.print(", ");
        USBSERIAL.print(setSpeed.z);
        USBSERIAL.print(")\n");
    }
}

void Gripper::setMessageTarget()
{
    /* This function calculates the targets for a given input (mm and radians) */

    float x_mm = 0;
    float y_mm = 0;
    float z_mm = 0;

    // convert the input message to millimeter motor positions
    if (iostream.inputMessage.instructionByte == iostream.motorCommandByte_m) {
        if (debug) { USBSERIAL.print("Received motor command m\n"); }
        x_mm = iostream.inputMessage.x * 1e3;
        y_mm = iostream.inputMessage.y * 1e3;
        z_mm = iostream.inputMessage.z * 1e3;
    }
    else if (iostream.inputMessage.instructionByte == iostream.motorCommandByte_mm) {
        if (debug) { USBSERIAL.print("Received motor command mm\n"); }
        x_mm = iostream.inputMessage.x;
        y_mm = iostream.inputMessage.y;
        z_mm = iostream.inputMessage.z;
    }
    else if (iostream.inputMessage.instructionByte == iostream.jointCommandByte_m_rad) {
        if (debug) { USBSERIAL.print("Received motor command m rad\n"); }
        x_mm = iostream.inputMessage.x * 1e3;
        y_mm = x_mm - params.screwDistance_xy * sin(iostream.inputMessage.y);
        z_mm = iostream.inputMessage.z * 1e3;
    }
    else if (iostream.inputMessage.instructionByte == iostream.jointCommandByte_mm_deg) {
        if (debug) { USBSERIAL.print("Received motor command mm deg\n"); }
        constexpr float to_rad = 3.141592654 / 180.0;
        x_mm = iostream.inputMessage.x;
        y_mm = x_mm - params.screwDistance_xy * sin(iostream.inputMessage.y * to_rad);
        z_mm = iostream.inputMessage.z;
    }
    // else if (iostream.inputMessage.instructionByte == iostream.stepCommandByte) {
    //     if (debug) { USBSERIAL.print("Received motor command step\n"); }
    //     control.stepTarget.x = iostream.inputMessage.x;
    //     control.stepTarget.y = iostream.inputMessage.y;
    //     control.stepTarget.z = iostream.inputMessage.z;
    //     goto GOTO_after_step_set;
    // }
    else {
        sendErrorMessage(iostream.invalidCommandByte);
        return;
    }

    if (debug) {
        USBSERIAL.print("x_mm is "); USBSERIAL.println(x_mm);
        USBSERIAL.print("y_mm is "); USBSERIAL.println(y_mm);
        USBSERIAL.print("z_mm is "); USBSERIAL.println(z_mm);
    }

    // calculate target revolutions for each motor
    float revs_x = (params.home.x + x_mm * params.direction.x) / params.mmPerRev.x;
    float revs_y = (params.home.y + y_mm * params.direction.y) / params.mmPerRev.y;
    float revs_z = (params.home.z + z_mm * params.direction.z) / params.mmPerRev.z;

    // check the target revolutions are not exceeding our limits
    if (revs_x < 0 or revs_x > m.limitRevs.x or
        revs_y < 0 or revs_y > m.limitRevs.y or
        revs_z < 0 or revs_z > m.limitRevs.z) {
        if (debug) {
            USBSERIAL.println("Revs limit exeeded, abort");
        }
        return;
    }
    
    // convert target revolutions to target steps
    control.stepTarget.x = revs_x * m.stepsPerRev * m.microstep.x;
    control.stepTarget.y = revs_y * m.stepsPerRev * m.microstep.y;
    control.stepTarget.z = revs_z * m.stepsPerRev * m.microstep.z;

// GOTO_after_step_set:

    // set motor speeds
    motorX.setRPM(setSpeed.x);
    motorY.setRPM(setSpeed.y);
    motorZ.setRPM(setSpeed.z);

    // set the motor targets
    motorX.setTarget(control.stepTarget.x);
    motorY.setTarget(control.stepTarget.y);
    motorZ.setTarget(control.stepTarget.z);

    // reset target booleans
    targetReached.all = false;
    targetReached.x = false;
    targetReached.y = false;
    targetReached.z = false;

    operatingMode = 1;
}

void Gripper::readGauge(const int gauge_num)
{
    /* This function reads an individual gauge, reading takes 380us */

    switch (gauge_num) {
    case 1:
        if (not in_use.gauge1) {
            iostream.outputMessage.gaugeOneReading = 0;
            newReadGauge1 = true;
        }
        else if (gauge1.is_ready()) {
            iostream.outputMessage.gaugeOneReading = gauge1.read();
            newReadGauge1 = true;
        }
        break;
    case 2:
        if (not in_use.gauge2) {
            iostream.outputMessage.gaugeTwoReading = 0;
            newReadGauge2 = true;
        }
        else if (gauge2.is_ready()) {
            iostream.outputMessage.gaugeTwoReading = gauge2.read();
            newReadGauge2 = true;
        }
        break;
    case 3:
        if (not in_use.gauge3) {
            iostream.outputMessage.gaugeThreeReading = 0;
            newReadGauge3 = true;
        }
        else if (gauge3.is_ready()) {
            iostream.outputMessage.gaugeThreeReading = gauge3.read();
            newReadGauge3 = true;
        }
        break;
    case 4:
        if (not in_use.gauge4) {
            iostream.outputMessage.gaugeFourReading = 0;
            newReadGauge4 = true;
        }
        else if (gauge4.is_ready()) {
            iostream.outputMessage.gaugeFourReading = gauge4.read();
            newReadGauge4 = true;
        }
        break;
    }
}

void Gripper::readGauges()
{
    /* Read all gauges one after the other*/

    readGauge(1);
    readGauge(2);
    readGauge(3);
    readGauge(4);
}

bool Gripper::checkSerial()
{
    /* This function checks to see if there is an incoming serial message */

    if (debug) {
        if (USBSERIAL.available() > 0) {
            byte x = USBSERIAL.read();

            // print diagnostic information
            if (x == 'p') {
                print();
            }
        }
    }

    if (BTSERIAL.available() > 0) {
        // read any input bytes, see if we received a message
        if (iostream.readInput()) {

            if (debug) {
                USBSERIAL.print("Received message, instruction byte is ");
                USBSERIAL.println(iostream.inputMessage.instructionByte);
            }

            // have we received a position command message
            if (iostream.inputMessage.instructionByte >= iostream.commandByteMinimum and
                iostream.inputMessage.instructionByte <= iostream.commandByteMaximum) {
                setMessageTarget();
            }
            else {

                // check for special case message type
                switch (iostream.inputMessage.instructionByte) {
                case iostream.homeByte:
                    setHomeTarget();
                    break;
                case iostream.powerSavingOnByte:
                    if (debug) { USBSERIAL.print("Power saving set to true\n"); }
                    powerSaving = true;
                    break;
                case iostream.powerSavingOffByte:
                    if (debug) { USBSERIAL.print("Power saving set to false\n"); }
                    if (!disabled) motorEnable(true);
                    powerSaving = false;
                    break;
                case iostream.stopByte:
                    if (debug) { USBSERIAL.print("Disabled set to true\n"); }
                    motorEnable(false);
                    disabled = true;
                    break;
                case iostream.resumeByte:
                    if (debug) { USBSERIAL.print("Disabled set to false\n"); }
                    motorEnable(true);
                    disabled = false;
                    break;
                case iostream.setSpeedByte:
                    setSpeedTarget();
                    break;
                case iostream.debugOnByte:
                    USBSERIAL.print("Debug set to true\n");
                    debug = true;
                    break;
                case iostream.debugOffByte:
                    USBSERIAL.print("Debug set to false\n");
                    debug = false;
                    break;
                case iostream.printByte:
                    print();
                    break;
                }
            }

            return true;
        }
    }

    return false;
}

void Gripper::runMotors(const int loopMillis)
{
    /* This function pulses the motors based on the current operation mode */

    // if gripper is in disabled mode, do not move any motors
    if (disabled) {
        return;
    }

    unsigned long loopStartMillis = millis();

    // joystick mode
    if (operatingMode == 0) {

        if (powerSaving) {
            if (control.rpm.x == 0 and
                control.rpm.y == 0 and
                control.rpm.z == 0) {
                motorEnable(false);
            }
            else {
                motorEnable(true);
            }
        }

        while ((millis() - loopStartMillis) < loopMillis) {
            motorX.rampedPulse();
            motorY.rampedPulse();
            motorZ.rampedPulse();
        }

        // default for joystick mode
        targetReached.all = false;
    }
    // serial input target position mode
    else if (operatingMode == 1) {
        
        if (powerSaving) {
            motorEnable(not targetReached.all);
        }

        while ((millis() - loopStartMillis) < loopMillis) {
            targetReached.x = motorX.targetPulse();
            targetReached.y = motorY.targetPulse();
            targetReached.z = motorZ.targetPulse();
        }

        // check if the target has been reached
        if (targetReached.x and
            targetReached.y and
            targetReached.z) {
            targetReached.all = true;
        }
    }
    // homing mode
    else if (operatingMode == 2) {

        if (powerSaving) {
            motorEnable(not targetReached.all);
        }

        // which motors do we need to home
        bool x_homed = not m.inUse.x or targetReached.x;
        bool y_homed = not m.inUse.y or targetReached.y;
        bool z_homed = not m.inUse.z or targetReached.z;

        while ((millis() - loopStartMillis) < loopMillis) {
            // home pulse only if the motor is not yet homed
            if (not x_homed) x_homed = motorX.homePulse();
            if (not y_homed) y_homed = motorY.homePulse();
            if (not z_homed) z_homed = motorZ.homePulse();
        }

        // update if we have reached our target
        targetReached.x = x_homed;
        targetReached.y = y_homed;
        targetReached.z = z_homed;

        // when we reach our target, end the homing
        // might be able to remove this...TEST IT!
        if (targetReached.x and
            targetReached.y and
            targetReached.z) {
            targetReached.all = true;
            operatingMode = 1;
        }
    }
}

void Gripper::motorEnable(bool is_enabled)
{
    /* This function enables or disables the motors, in order to save power
    * and reduce heat generation when not moving */

    if (is_enabled) {
        digitalWrite(motor_enable, LOW);
    }
    else {
        digitalWrite(motor_enable, HIGH);
    }
}

void Gripper::setMotorPositions()
{
    /* This function sets the motor positions for the output message */

    iostream.outputMessage.motorX_mm = 
        params.home.x + (params.mmPerStep.x * motorX.getStep());
    iostream.outputMessage.motorY_mm = 
        params.home.y + (params.mmPerStep.y * motorY.getStep());
    iostream.outputMessage.motorZ_mm = 
        params.home.z + (params.mmPerStep.z * motorZ.getStep());
}

void Gripper::checkInputs()
{
    /* This function monitors the input streams to the gripper */

    // check for commands
    readJoystick();
    checkSerial();

    // check for incoming gauge data
    readGauges();
}

void Gripper::sendErrorMessage(byte error_code)
{
    /* send an empty message with a specific error code byte to indicate an error */

    iostream.outputMessage.informationByte = error_code;
    iostream.outputMessage.isTargetReached = 0;
	iostream.outputMessage.gaugeOneReading = 0;
	iostream.outputMessage.gaugeTwoReading = 0;
	iostream.outputMessage.gaugeThreeReading = 0;
    iostream.outputMessage.gaugeFourReading = 0;
	iostream.outputMessage.motorX_mm = -1;
	iostream.outputMessage.motorY_mm = -1;
	iostream.outputMessage.motorZ_mm = -1;

    iostream.publishOutput();
}

void Gripper::publishOutput()
{
    /* This function checks if there is a new output to publish, and if
    so it publishes it */

    // check if all the gauges have new data to publish
    if (newReadGauge1 and newReadGauge2 and newReadGauge3 and newReadGauge4) {

        // fill data into the output message
        iostream.outputMessage.informationByte = 0;
        iostream.outputMessage.isTargetReached = targetReached.all;
        setMotorPositions();

        // publish the message on Serial2 (hardcoded)
        iostream.publishOutput();
        
        // gauge readings are now out of date
        newReadGauge1 = false;
        newReadGauge2 = false;
        newReadGauge3 = false;
        newReadGauge4 = false;
    }
}

void Gripper::smoothRun(int cycleTime_ms)
{
    /* This function attempts to run the gripper smoothly, interspersing
    I/O operations with running the motors */

    /* Tasks:
            1. readJoystick();
            2. checkSerial();
            3. readGauge(1);
            4. readGauge(2);
            5. readGauge(3);
            6. readGauge(4);
            7. publishOutput();
    */

    unsigned long startTime_us = micros();

    constexpr int numTasks = 7;
    constexpr int minTaskTime_ms = 2;       // milliseconds
    constexpr int minRunPadding_ms = 2;     // milliseconds

    // enforce a minimum cycle time
    if (cycleTime_ms < minTaskTime_ms * numTasks + minRunPadding_ms) {
        cycleTime_ms = minTaskTime_ms * numTasks + minRunPadding_ms;

        if (debug) {
            USBSERIAL.print("Cycle time increased to ");
            USBSERIAL.println(cycleTime_ms);
        }
    }

    // calculate time for regular tasks
    int taskTime = (cycleTime_ms - minRunPadding_ms) / numTasks;

    /* execute tasks */
    readJoystick();
    runMotors(taskTime);

    checkSerial();
    runMotors(taskTime);

    readGauge(1);
    runMotors(taskTime);

    readGauge(2);
    runMotors(taskTime);

    readGauge(3);
    runMotors(taskTime);

    readGauge(4);
    runMotors(taskTime);

    publishOutput();
    runMotors(taskTime);

    // finally, we want to run the motors until the cycle time is up
    unsigned long endTime_us = micros();
    unsigned long elapsedTime_ms = (endTime_us - startTime_us) / 1000;
    int remainingTime_ms = cycleTime_ms - elapsedTime_ms;

    if (remainingTime_ms > 0) {
        runMotors(remainingTime_ms);
    }
    else if (debug) {
        USBSERIAL.print("Cycle time exceeded\n");
    }
    
}

void Gripper::print()
{
    /* print information to Serial terminal */

    USBSERIAL.print("\n--- start print ---\n");

    // print motor step position
    USBSERIAL.print("Current motor steps (x, y, z): (");
    USBSERIAL.print(motorX.getStep());
    USBSERIAL.print(", ");
    USBSERIAL.print(motorY.getStep());
    USBSERIAL.print(", ");
    USBSERIAL.print(motorZ.getStep());
    USBSERIAL.print(")\n");

    // print motor joint state
    USBSERIAL.print("Joint positions (x, th, z): (");
    USBSERIAL.print(params.home.x + (params.mmPerStep.x * motorX.getStep()));
    USBSERIAL.print(", ");
    USBSERIAL.print(params.home.y + (params.mmPerStep.y * motorY.getStep()));
    USBSERIAL.print(", ");
    USBSERIAL.print(params.home.z + (params.mmPerStep.z * motorZ.getStep()));
    USBSERIAL.print(")\n");

    // print motor set speeds
    USBSERIAL.print("Motor speed settings (x, y, z): (");
    USBSERIAL.print(setSpeed.x);
    USBSERIAL.print(", ");
    USBSERIAL.print(setSpeed.y);
    USBSERIAL.print(", ");
    USBSERIAL.print(setSpeed.z);
    USBSERIAL.print(")\n");

    // print gauge use
    USBSERIAL.print("Gauge use is: ");
    USBSERIAL.print(in_use.gauge1);
    USBSERIAL.print(in_use.gauge2);
    USBSERIAL.print(in_use.gauge3);
    USBSERIAL.print(in_use.gauge4);
    USBSERIAL.print(")\n");

    // print last gauge readings
    USBSERIAL.print("The last gauge readings were (g1, g2, g3, g4): (");
    USBSERIAL.print(iostream.outputMessage.gaugeOneReading);
    USBSERIAL.print(", ");
    USBSERIAL.print(iostream.outputMessage.gaugeTwoReading);
    USBSERIAL.print(", ");
    USBSERIAL.print(iostream.outputMessage.gaugeThreeReading);
    USBSERIAL.print(", ");
    USBSERIAL.print(iostream.outputMessage.gaugeFourReading);
    USBSERIAL.print(")\n");

    // print settings
    USBSERIAL.print("is target reached ");
    USBSERIAL.println(targetReached.all);
    USBSERIAL.print("power saving ");
    USBSERIAL.println(powerSaving);
    USBSERIAL.print("disabled ");
    USBSERIAL.println(disabled);
    USBSERIAL.print("debug ");
    USBSERIAL.println(debug);

    USBSERIAL.print("--- end ---\n\n");
}