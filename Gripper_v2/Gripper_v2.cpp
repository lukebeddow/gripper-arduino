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

    // do we apply a scaling and offset here? not currently
    // gauge1.set_scale(loadcell_divider);
    // gauge1.set_offset(loadcell_offset);

    // set to defaults (0=joystick, 1=serial)
    operatingMode = 1;
    targetReached = true;
    newReadGauge1 = false;
    newReadGauge2 = false;
    newReadGauge3 = false;
    powerSaving = false;
    disabled = false;
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

    // calculate the mm per step parameters, this is never updated afterwards!
    mmPerStep.x = params.mmPerRev.x / (m.stepsPerRev * m.microstep.x);
    mmPerStep.y = params.mmPerRev.y / (m.stepsPerRev * m.microstep.y);
    mmPerStep.z = params.mmPerRev.z / (m.stepsPerRev * m.microstep.z);

    // adjust whether we gain +ve or -ve position from a step increase
    mmPerStep.x *= params.direction.x;
    mmPerStep.y *= params.direction.y;
    mmPerStep.z *= params.direction.z;
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
            targetReached = false;
            m.targetReached.x = false;
            m.targetReached.y = false;
            m.targetReached.z = false;
        }
        return false;
    }

    // are we within a homing sequence? Do not interrupt
    if (operatingMode == 2 and not targetReached) {
        return false;
    }

    // otherwise, the joystick is connected and enabled
    operatingMode = 0;

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

    targetReached = true;
    m.targetReached.x = x_homed;
    m.targetReached.y = y_homed;
    m.targetReached.z = z_homed;
}

void Gripper::setHomeTarget()
{
    /* This function prepares for homing, afterwards runMotors should be used */

    // in case a previous homing sequence was interrupted
    motorX.wipeHomingFlag();
    motorY.wipeHomingFlag();
    motorZ.wipeHomingFlag();
    
    // set gripper parameters for homing mode
    operatingMode = 2;
    m.targetReached.x = false;
    m.targetReached.y = false;
    m.targetReached.z = false;
    targetReached = false;
}

void Gripper::setMessageTarget()
{
    /* This function calculates the targets for a given input (mm and radians) */

    float x_mm = 0;
    float y_mm = 0;
    float z_mm = 0;

    // convert the input message to millimeter motor positions
    if (iostream.inputMessage.instructionByte == iostream.motorCommandByte_m) {
        x_mm = iostream.inputMessage.x * 1e3;
        y_mm = iostream.inputMessage.y * 1e3;
        z_mm = iostream.inputMessage.z * 1e3;
    }
    else if (iostream.inputMessage.instructionByte == iostream.motorCommandByte_mm) {
        x_mm = iostream.inputMessage.x;
        y_mm = iostream.inputMessage.y;
        z_mm = iostream.inputMessage.z;
    }
    else if (iostream.inputMessage.instructionByte == iostream.jointCommandByte_m_rad) {
        x_mm = iostream.inputMessage.x * 1e3;
        y_mm = x_mm - params.screwDistance_xy * sin(iostream.inputMessage.y);
        z_mm = iostream.inputMessage.z * 1e3;
    }
    else if (iostream.inputMessage.instructionByte == iostream.jointCommandByte_mm_deg) {
        constexpr float to_rad = 3.141592654 / 180.0;
        x_mm = iostream.inputMessage.x;
        y_mm = x_mm - params.screwDistance_xy * sin(iostream.inputMessage.y * to_rad);
        z_mm = iostream.inputMessage.z;
    }
    else if (iostream.inputMessage.instructionByte == iostream.stepCommandByte) {
        control.stepTarget.x = iostream.inputMessage.x;
        control.stepTarget.y = iostream.inputMessage.y;
        control.stepTarget.z = iostream.inputMessage.z;
        goto GOTO_after_step_set;
    }
    else {
        sendErrorMessage(iostream.invalidCommandByte);
        return;
    }

    // // for testing
    // Serial.print("x_mm is "); Serial.println(x_mm);
    // Serial.print("y_mm is "); Serial.println(y_mm);
    // Serial.print("z_mm is "); Serial.println(z_mm);

    // calculate target revolutions for each motor
    float revs_x = (params.home.x + x_mm * params.direction.x) / params.mmPerRev.x;
    float revs_y = (params.home.y + y_mm * params.direction.y) / params.mmPerRev.y;
    float revs_z = (params.home.z + z_mm * params.direction.z) / params.mmPerRev.z;
    
    // convert target revolutions to target steps
    control.stepTarget.x = revs_x * m.stepsPerRev * m.microstep.x;
    control.stepTarget.y = revs_y * m.stepsPerRev * m.microstep.y;
    control.stepTarget.z = revs_z * m.stepsPerRev * m.microstep.z;

GOTO_after_step_set:

    // set motor speeds
    motorX.setRPM(m.maxSpeed.x);
    motorY.setRPM(m.maxSpeed.y);
    motorZ.setRPM(m.maxSpeed.z);

    // set the motor targets
    motorX.setTarget(control.stepTarget.x);
    motorY.setTarget(control.stepTarget.y);
    motorZ.setTarget(control.stepTarget.z);

    // reset target booleans
    targetReached = false;
    m.targetReached.x = false;
    m.targetReached.y = false;
    m.targetReached.z = false;

    operatingMode = 1;
}

void Gripper::readGauge(const int gauge_num)
{
    /* This function reads an individual gauge, reading takes 380us */

    switch (gauge_num) {
    case 1:
        if (gauge1.is_ready()) {
            iostream.outputMessage.gaugeOneReading = gauge1.read();
            newReadGauge1 = true;
        }
        break;
    case 2:
        if (gauge2.is_ready()) {
            iostream.outputMessage.gaugeTwoReading = gauge2.read();
            newReadGauge2 = true;
        }
        break;
    case 3:
        if (gauge3.is_ready()) {
            iostream.outputMessage.gaugeThreeReading = gauge3.read();
            newReadGauge3 = true;
        }
        break;
    }
}

void Gripper::readGauges()
{
    /* Read all three gauges */

    readGauge(1);
    readGauge(2);
    readGauge(3);
}

bool Gripper::checkSerial()
{
    /* This function checks to see if there is an incoming serial message */

    if (Serial2.available() > 0) {
        // read any input bytes, see if we received a message
        if (iostream.readInput()) {
            // have we received a position command message
            if (iostream.inputMessage.instructionByte >= iostream.commandByteMinimum or
                iostream.inputMessage.instructionByte >= iostream.commandByteMaximum) {
                setMessageTarget();
            }
            else {
                // check for special case message type
                switch (iostream.inputMessage.instructionByte) {
                // case iostream.motorCommandByte_m:
                // case iostream.motorCommandByte_mm:
                // case iostream.jointCommandByte_m_rad:
                // case iostream.jointCommandByte_mm_deg:
                // case iostream.stepCommandByte:
                //     setMessageTarget();
                //     break;
                case iostream.homeByte:
                    setHomeTarget();
                    break;
                case iostream.powerSavingOnByte:
                    powerSaving = true;
                    break;
                case iostream.powerSavingOffByte:
                    if (!disabled) motorEnable(true);
                    powerSaving = false;
                    break;
                case iostream.stopByte:
                    motorEnable(false);
                    disabled = true;
                    break;
                case iostream.resumeByte:
                    motorEnable(true);
                    disabled = false;
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
        targetReached = false;
    }
    // serial input target position mode
    else if (operatingMode == 1) {
        
        if (powerSaving) {
            motorEnable(not targetReached);
        }

        while ((millis() - loopStartMillis) < loopMillis) {
            m.targetReached.x = motorX.targetPulse();
            m.targetReached.y = motorY.targetPulse();
            m.targetReached.z = motorZ.targetPulse();
        }

        // check if the target has been reached
        if (m.targetReached.x and
            m.targetReached.y and
            m.targetReached.z) {
            targetReached = true;
        }
    }
    // homing mode
    else if (operatingMode == 2) {

        if (powerSaving) {
            motorEnable(not targetReached);
        }

        // which motors do we need to home
        bool x_homed = not m.inUse.x or m.targetReached.x;
        bool y_homed = not m.inUse.y or m.targetReached.y;
        bool z_homed = not m.inUse.z or m.targetReached.z;

        while ((millis() - loopStartMillis) < loopMillis) {
            // home pulse only if the motor is not yet homed
            if (not x_homed) x_homed = motorX.homePulse();
            if (not y_homed) y_homed = motorY.homePulse();
            if (not z_homed) z_homed = motorZ.homePulse();
        }

        // update if we have reached our target
        m.targetReached.x = x_homed;
        m.targetReached.y = y_homed;
        m.targetReached.z = z_homed;

        // when we reach our target, end the homing
        // might be able to remove this...TEST IT!
        if (m.targetReached.x and
            m.targetReached.y and
            m.targetReached.z) {
            targetReached = true;
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
        params.home.x + (mmPerStep.x * motorX.getStep());
    iostream.outputMessage.motorY_mm = 
        params.home.y + (mmPerStep.y * motorY.getStep());
    iostream.outputMessage.motorZ_mm = 
        params.home.z + (mmPerStep.z * motorZ.getStep());
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
    if (newReadGauge1 and newReadGauge2 and newReadGauge3) {

        // fill data into the output message
        iostream.outputMessage.isTargetReached = targetReached;
        setMotorPositions();

        // publish the message on Serial2 (hardcoded)
        iostream.publishOutput();
        
        // gauge readings are now out of date
        newReadGauge1 = false;
        newReadGauge2 = false;
        newReadGauge3 = false;
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
            6. publishOutput();
    */

    unsigned long startTime_us = micros();

    constexpr int numTasks = 6;
    constexpr int minTaskTime_ms = 2;       // milliseconds
    constexpr int minRunPadding_ms = 2;     // milliseconds

    // enforce a minimum cycle time
    if (cycleTime_ms < minTaskTime_ms * numTasks + minRunPadding_ms) {
        cycleTime_ms = minTaskTime_ms * numTasks + minRunPadding_ms;
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

    publishOutput();
    runMotors(taskTime);

    // finally, we want to run the motors until the cycle time is up
    unsigned long endTime_us = micros();
    unsigned long elapsedTime_ms = (endTime_us - startTime_us) / 1000;
    unsigned int remainingTime_ms = cycleTime_ms - elapsedTime_ms;

    runMotors(remainingTime_ms);
}