#include <Arduino.h>
#include <Gripper_v3.h>

Gripper_v3::Gripper_v3()
{
    /* Class constructor */

    setupMotors();
    setPins();

    // begin running HX711 chips to monitor gauges
    gauge1.begin(SG1_dt, SG1_sck);
    gauge2.begin(SG2_dt, SG2_sck);
    gauge3.begin(SG3_dt, SG3_sck);
    gauge4.begin(SG4_dt, SG4_sck);

    // do we apply a scaling and offset here? no plans to
    // gauge1.set_scale(loadcell_divider);
    // gauge1.set_offset(loadcell_offset);

    // set flags to defaults (0=joystick, 1=serial)
    operatingMode = 1;
    targetReached.all = true;
    newReadGauge1 = false;
    newReadGauge2 = false;
    newReadGauge3 = false;
    newReadGauge4 = false;

    // speed defaults
    setSpeed.x = m.maxSpeed.x;
    setSpeed.y = m.maxSpeed.y;
    setSpeed.z = m.maxSpeed.z;
}

void Gripper_v3::setupMotors()
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

void Gripper_v3::setPins()
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

bool Gripper_v3::readJoystick()
{
    /* Read to see if the joystick is enabled, and what it is set to */

    // disabled
    return false;

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

void Gripper_v3::homingSequence()
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

    if (powerSaving) motorEnable(false);
}

void Gripper_v3::setHomeTarget()
{
    /* This function prepares for homing, afterwards runMotors should be used */

    if (debug) {
        iostream.startDebugMessage();
        DEBUGSERIAL.print(F("Setting home target\n"));
        iostream.endDebugMessage();
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

void Gripper_v3::setSpeedTarget()
{
    /* save a speed target */

    if (debug) {
        iostream.startDebugMessage();
        DEBUGSERIAL.print(F("Incoming speed request (x, y, z) of: ("));
        DEBUGSERIAL.print(iostream.inputMessage.x);
        DEBUGSERIAL.print(F(", "));
        DEBUGSERIAL.print(iostream.inputMessage.y);
        DEBUGSERIAL.print(F(", "));
        DEBUGSERIAL.print(iostream.inputMessage.z);
        DEBUGSERIAL.print(F(")\n"));
    }

    setSpeed.x = iostream.inputMessage.x;
    setSpeed.y = iostream.inputMessage.y;
    setSpeed.z = iostream.inputMessage.z;

    // // check if the input message contains garbage (nans)
    // if (setSpeed.x != setSpeed.x or
    //     setSpeed.y != setSpeed.y or
    //     setSpeed.z != setSpeed.z) {
    //   setSpeed.x = 123.4;
    //   setSpeed.y = 123.4;
    //   setSpeed.z = 123.4;
    // }

    // safety checks
    if (setSpeed.x < 0) setSpeed.x = 0;
    if (setSpeed.x > m.maxSpeed.x) setSpeed.x = m.maxSpeed.x;

    if (setSpeed.y < 0) setSpeed.y = 0;
    if (setSpeed.y > m.maxSpeed.y) setSpeed.y = m.maxSpeed.y;

    if (setSpeed.z < 0) setSpeed.z = 0;
    if (setSpeed.z > m.maxSpeed.z) setSpeed.z = m.maxSpeed.z;

    if (debug) {
        DEBUGSERIAL.print(F("Speed target (x, y, z) set: ("));
        DEBUGSERIAL.print(setSpeed.x);
        DEBUGSERIAL.print(F(", "));
        DEBUGSERIAL.print(setSpeed.y);
        DEBUGSERIAL.print(F(", "));
        DEBUGSERIAL.print(setSpeed.z);
        DEBUGSERIAL.print(F(")\n"));
        iostream.endDebugMessage();
    }
}

void Gripper_v3::setMessageTarget()
{
    /* This function calculates the targets for a given input (mm and radians) */

    if (debug) { 
        iostream.startDebugMessage(); 
    }

    float x_mm = 0;
    float y_mm = 0;
    float z_mm = 0;

    timedActionInProgress = false;

    // convert the input message to millimeter motor positions
    if (iostream.inputMessage.instructionByte == iostream.motorCommandByte_m) {
        if (debug) { DEBUGSERIAL.print(F("Received motor command m\n")); }
        x_mm = iostream.inputMessage.x * 1e3;
        y_mm = iostream.inputMessage.y * 1e3;
        z_mm = iostream.inputMessage.z * 1e3;
    }
    else if (iostream.inputMessage.instructionByte == iostream.motorCommandByte_mm) {
        if (debug) { DEBUGSERIAL.print(F("Received motor command mm\n")); }
        x_mm = iostream.inputMessage.x;
        y_mm = iostream.inputMessage.y;
        z_mm = iostream.inputMessage.z;
    }
    else if (iostream.inputMessage.instructionByte == iostream.jointCommandByte_m_rad) {
        if (debug) { DEBUGSERIAL.print(F("Received motor command m rad\n")); }
        x_mm = iostream.inputMessage.x * 1e3;
        y_mm = x_mm - params.screwDistance_xy * sin(iostream.inputMessage.y);
        z_mm = iostream.inputMessage.z * 1e3;
    }
    else if (iostream.inputMessage.instructionByte == iostream.jointCommandByte_mm_deg) {
        if (debug) { DEBUGSERIAL.print(F("Received motor command mm deg\n")); }
        constexpr float to_rad = 3.141592654 / 180.0;
        x_mm = iostream.inputMessage.x;
        y_mm = x_mm - params.screwDistance_xy * sin(iostream.inputMessage.y * to_rad);
        z_mm = iostream.inputMessage.z;
    }
    // else if (iostream.inputMessage.instructionByte == iostream.stepCommandByte) {
    //     if (debug) { DEBUGSERIAL.print("Received motor command step\n"); }
    //     control.stepTarget.x = iostream.inputMessage.x;
    //     control.stepTarget.y = iostream.inputMessage.y;
    //     control.stepTarget.z = iostream.inputMessage.z;
    //     goto GOTO_after_step_set;
    // }
    else if (iostream.inputMessage.instructionByte == iostream.timedCommandByte_m) {
        if (debug) { DEBUGSERIAL.print(F("Received timed motor command m\n")); }
        x_mm = iostream.inputMessage.x * 1e3;
        y_mm = iostream.inputMessage.y * 1e3;
        z_mm = iostream.inputMessage.z * 1e3;
        timedActionInProgress = true;
        timedActionStart_ms = millis();
    }
    else {
        sendErrorMessage(iostream.invalidCommandByte);
        return;
    }

    if (debug) {
        DEBUGSERIAL.print(F("x_mm is ")); DEBUGSERIAL.println(x_mm);
        DEBUGSERIAL.print(F("y_mm is ")); DEBUGSERIAL.println(y_mm);
        DEBUGSERIAL.print(F("z_mm is ")); DEBUGSERIAL.println(z_mm);
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
            DEBUGSERIAL.println("Revs limit exeeded, abort");
            iostream.endDebugMessage();
        }
        return;
    }
    
    // convert target revolutions to target steps
    control.stepTarget.x = revs_x * m.stepsPerRev * m.microstep.x;
    control.stepTarget.y = revs_y * m.stepsPerRev * m.microstep.y;
    control.stepTarget.z = revs_z * m.stepsPerRev * m.microstep.z;

// GOTO_after_step_set:

    // if we are performing a timed action, set speeds to match
    if (iostream.inputMessage.instructionByte == iostream.timedCommandByte_m) {

        setSpeed.x = (revs_x) / (timedActionSecs / 60.0);
        setSpeed.y = (revs_y) / (timedActionSecs / 60.0);
        setSpeed.z = (revs_z) / (timedActionSecs / 60.0);

        // check we don't exceed our speed limits
        if (setSpeed.x > m.maxSpeed.x) setSpeed.x = m.maxSpeed.x;
        if (setSpeed.y > m.maxSpeed.y) setSpeed.y = m.maxSpeed.y;
        if (setSpeed.z > m.maxSpeed.z) setSpeed.z = m.maxSpeed.z;

        if (debug) {
            DEBUGSERIAL.print("x speed ");
            DEBUGSERIAL.println(setSpeed.x);
            DEBUGSERIAL.print("y speed ");
            DEBUGSERIAL.println(setSpeed.y);
            DEBUGSERIAL.print("z speed ");
            DEBUGSERIAL.println(setSpeed.z);
        }
    }

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

    if (debug) { 
        iostream.endDebugMessage(); 
    }
}

void Gripper_v3::setGripperTarget(float x, float y, float z)
{
    /* input a position target in metres */

    // set motor speeds
    iostream.inputMessage.instructionByte = iostream.motorCommandByte_m;
    iostream.inputMessage.x = x;
    iostream.inputMessage.y = y;
    iostream.inputMessage.z = z;

    setMessageTarget();
}

void Gripper_v3::readGauge(const int gauge_num)
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
    case 4:
        if (gauge4.is_ready()) {
            iostream.outputMessage.gaugeFourReading = gauge4.read();
            newReadGauge4 = true;
        }
        break;
    }
}

void Gripper_v3::readGauges()
{
    /* Read all gauges one after the other*/

    readGauge(1);
    readGauge(2);
    readGauge(3);
    readGauge(4);
}

bool Gripper_v3::checkSerial()
{
    /* This function checks to see if there is an incoming serial message */

    if (BTSERIAL.available() > 0) {
        // read any input bytes, see if we received a message
        if (iostream.readInput()) {

            if (debug) {
                iostream.startDebugMessage();
                DEBUGSERIAL.print(F("Received message, instruction byte is "));
                DEBUGSERIAL.println(iostream.inputMessage.instructionByte);
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
                    if (debug) { 
                        DEBUGSERIAL.print(F("Power saving set to true\n")); 
                    }
                    powerSaving = true;
                    break;
                case iostream.powerSavingOffByte:
                    if (debug) { 
                        DEBUGSERIAL.print(F("Power saving set to false\n")); 
                    }
                    if (!disabled) motorEnable(true);
                    powerSaving = false;
                    break;
                case iostream.stopByte:
                    if (debug) { 
                        DEBUGSERIAL.print(F("Disabled set to true\n")); 
                    }
                    motorEnable(false);
                    disabled = true;
                    break;
                case iostream.resumeByte:
                    if (debug) { 
                        DEBUGSERIAL.print(F("Disabled set to false\n")); 
                    }
                    motorEnable(true);
                    disabled = false;
                    break;
                case iostream.setSpeedByte:
                    setSpeedTarget();
                    break;
                case iostream.debugOnByte:
                    iostream.startDebugMessage();
                    DEBUGSERIAL.print(F("Debug set to true\n"));
                    iostream.endDebugMessage();
                    debug = true;
                    break;
                case iostream.debugOffByte:
                    iostream.startDebugMessage();
                    DEBUGSERIAL.print(F("Debug set to false\n"));
                    iostream.endDebugMessage();
                    debug = false;
                    break;
                case iostream.printByte:
                    print();
                    break;
                case iostream.changeTimedActionByte:
                    timedActionSecs = iostream.inputMessage.x;
                    if (timedActionSecs < 0.01) timedActionSecs = 0.01;
                    else if (timedActionSecs > 100) timedActionSecs = 100;
                    if (debug) { 
                        DEBUGSERIAL.print(F("Changed timedActionSecs to "));
                        DEBUGSERIAL.println(timedActionSecs); 
                    }
                    break;
                case iostream.changeTimedActionPubEarlyByte:
                    timedActionPubEarly = iostream.inputMessage.x;
                    if (timedActionPubEarly < 0.01) timedActionPubEarly = -1;
                    else if (timedActionPubEarly > 100) timedActionPubEarly = 100;
                    if (debug) { 
                        DEBUGSERIAL.print(F("Changed timedActionPubEarly to "));
                        DEBUGSERIAL.println(timedActionPubEarly); 
                    }
                    break;
                case iostream.setGaugeHzByte:
                    gauge_hz = iostream.inputMessage.x;
                    if (gauge_hz < 0.01) gauge_hz = 0.01;
                    else if (gauge_hz > 100) gauge_hz = 100;
                    if (debug) { 
                        DEBUGSERIAL.print(F("Gauge hz set to "));
                        DEBUGSERIAL.println(gauge_hz);
                    }
                    break;
                case iostream.setPublishHzByte:
                    publish_hz = iostream.inputMessage.x;
                    if (publish_hz < 0.01) publish_hz = 0.01;
                    else if (publish_hz > 100) publish_hz = 100;
                    if (debug) { 
                        DEBUGSERIAL.print(F("Publish hz set to "));
                        DEBUGSERIAL.println(publish_hz);
                    }
                    break;
                case iostream.setSerialHzByte:
                    serial_hz = iostream.inputMessage.x;
                    if (serial_hz < 0.01) serial_hz = 0.01;
                    else if (serial_hz > 100) serial_hz = 100;
                    if (debug) { 
                        DEBUGSERIAL.print(F("Serial hz set to "));
                        DEBUGSERIAL.println(serial_hz);
                    }
                    break;
                case iostream.setMotorHzByte:
                    motor_hz = iostream.inputMessage.x;
                    if (motor_hz < 0.01) motor_hz = 0.01;
                    else if (motor_hz > 100000) motor_hz = 100000;
                    if (debug) { 
                        DEBUGSERIAL.print(F("Motor hz set to "));
                        DEBUGSERIAL.println(motor_hz);
                    }
                    break;
                }
            }

            if (debug) {
                iostream.endDebugMessage();
            }

            return true;
        }
    }

    return false;
}

void Gripper_v3::runMotors(const int loopMillis)
{
    /* This function pulses the motors based on the current operation mode */

    // if gripper is in disabled mode, do not move any motors
    if (disabled) {
        return;
    }

    unsigned long loopStartMillis = millis();

    // prevent joystick motion
    if (operatingMode = 0) operatingMode = 1;

    // serial input target position mode
    if (operatingMode == 1) {
        
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

void Gripper_v3::motorEnable(bool is_enabled)
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

void Gripper_v3::setMotorPositions()
{
    /* This function sets the motor positions for the output message */

    iostream.outputMessage.motorX_mm = 
        params.home.x + (params.mmPerStep.x * motorX.getStep());
    iostream.outputMessage.motorY_mm = 
        params.home.y + (params.mmPerStep.y * motorY.getStep());
    iostream.outputMessage.motorZ_mm = 
        params.home.z + (params.mmPerStep.z * motorZ.getStep());
}

void Gripper_v3::checkInputs()
{
    /* This function monitors the input streams to the gripper */

    // check for commands
    readJoystick();
    checkSerial();

    // check for incoming gauge data
    readGauges();
}

void Gripper_v3::sendErrorMessage(byte error_code)
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

void Gripper_v3::publishOutput()
{
    /* This function checks if there is a new output to publish, and if
    so it publishes it */

    // check if all the gauges have new data to publish
    if (newReadGauge1 and newReadGauge2 and newReadGauge3 and newReadGauge4) {

        // fill data into the output message
        iostream.outputMessage.informationByte = 0;
        iostream.outputMessage.isTargetReached = targetReached.all;
        setMotorPositions();

        // for timed actions, we can publish early that target is reached for sync
        if (targetReached.all) {
            timedActionInProgress = false;
        }
        if (timedActionInProgress) {
            if (millis() - (timedActionSecs - timedActionPubEarly) > timedActionStart_ms) {
                iostream.outputMessage.isTargetReached = true;
            }
            if (debug) {
                iostream.startDebugMessage();
                DEBUGSERIAL.println("Timed action early publish target reached");
                iostream.endDebugMessage();
            }
        }

        // publish the message on Serial2 (hardcoded)
        iostream.publishOutput();
        
        // gauge readings are now out of date
        newReadGauge1 = false;
        newReadGauge2 = false;
        newReadGauge3 = false;
        newReadGauge4 = false;
    }
}

void Gripper_v3::run()
{
    /* call the run function without argument specifying how long for. Runs the function
    only once. It is fine to loop and call the function like this */

    run(0); // zero indicates run only once
}

void Gripper_v3::run(unsigned long runtime_ms)
{
    /* Run the gripper for a given number of milliseconds. It is fine to call this
    function repeatedly without giving a number of milliseconds. If runtime_ms=0
    this function does not loop and just executes the main body once. */

    // if we are only running one loop
    bool run_once = false;
    if (runtime_ms == 0) {
        runtime_ms = 10; // value to ensure we run one loop
        run_once = true;
    }

    // persist information between function calls
    static unsigned long total_calls = 0;
    static unsigned long first_call_time = millis();

    static unsigned long last_publish_us = 0;
    static unsigned long last_serial_us = 0;
    static unsigned long last_motor_us = 0;
    static unsigned long last_gauge_us = 0;

    static unsigned long motor_wait_us = (1e6 / motor_hz);
    static unsigned long gauge_wait_us = (1e6 / gauge_hz);
    static unsigned long publish_wait_us = (1e6 / publish_hz);
    static unsigned long serial_wait_us = (1e6 / serial_hz);

    // begin main body of function
    unsigned long function_start_ms = millis();

    while ((millis() - function_start_ms) < runtime_ms) {

        // first pulse the motors
        if ((micros() - last_motor_us) > motor_wait_us and not disabled) {

            unsigned long t1 = micros();

            if (powerSaving) {
                motorEnable(not targetReached.all);
            }

            if (operatingMode == 1) {

                // move towards a given target
                targetReached.x = motorX.targetPulse();
                targetReached.y = motorY.targetPulse();
                targetReached.z = motorZ.targetPulse();

            }
            else if (operatingMode == 2) {

                // which motors do we need to home
                bool x_homed = not m.inUse.x or targetReached.x;
                bool y_homed = not m.inUse.y or targetReached.y;
                bool z_homed = not m.inUse.z or targetReached.z;

                // home pulse only if the motor is not yet homed
                if (not x_homed) x_homed = motorX.homePulse();
                if (not y_homed) y_homed = motorY.homePulse();
                if (not z_homed) z_homed = motorZ.homePulse();

                // update if we have reached our target
                targetReached.x = x_homed;
                targetReached.y = y_homed;
                targetReached.z = z_homed;

            }

            // check if the target has been reached
            if (targetReached.x and
                targetReached.y and
                targetReached.z) {
                targetReached.all = true;
                operatingMode = 1;
            }

            if (not targetReached.all) {
                lastMotorTime = micros() - t1;
            }

            last_motor_us = micros();
        }

        // now read the gauges
        if ((micros() - last_gauge_us) > gauge_wait_us) {
            unsigned long t1 = micros();
            readGauges();
            last_gauge_us = micros();
            lastGaugeTime = micros() - t1;
        }

        // check to publish output
        if ((micros() - last_publish_us) > publish_wait_us) {
            unsigned long t1 = micros();
            publishOutput();
            last_publish_us = micros();
            lastPublishTime = micros() - t1;
        }

        // check incoming serial commands
        if ((micros() - last_serial_us) > serial_wait_us) {
            unsigned long t1 = micros();
            checkSerial();
            last_serial_us = micros();
            lastSerialTime = micros() - t1;
        }

        total_calls += 1;
        runHz = ((float) total_calls * 1000) / (millis() - first_call_time);

        if (run_once) break;
    
    }
}

void Gripper_v3::print()
{
    /* print information to Serial terminal */

    iostream.startDebugMessage();

    // print motor step position
    DEBUGSERIAL.print(F("Current motor steps (x, y, z): ("));
    DEBUGSERIAL.print(motorX.getStep());
    DEBUGSERIAL.print(F(", "));
    DEBUGSERIAL.print(motorY.getStep());
    DEBUGSERIAL.print(F(", "));
    DEBUGSERIAL.print(motorZ.getStep());
    DEBUGSERIAL.print(F(")\n"));

    // print motor joint state
    DEBUGSERIAL.print(F("Joint positions (x, th, z): ("));
    DEBUGSERIAL.print(params.home.x + (params.mmPerStep.x * motorX.getStep()));
    DEBUGSERIAL.print(F(", "));
    DEBUGSERIAL.print(params.home.y + (params.mmPerStep.y * motorY.getStep()));
    DEBUGSERIAL.print(F(", "));
    DEBUGSERIAL.print(params.home.z + (params.mmPerStep.z * motorZ.getStep()));
    DEBUGSERIAL.print(F(")\n"));

    // print motor set speeds
    DEBUGSERIAL.print(F("Motor speed settings (x, y, z): ("));
    DEBUGSERIAL.print(setSpeed.x);
    DEBUGSERIAL.print(F(", "));
    DEBUGSERIAL.print(setSpeed.y);
    DEBUGSERIAL.print(F(", "));
    DEBUGSERIAL.print(setSpeed.z);
    DEBUGSERIAL.print(F(")\n"));

    // print last gauge readings
    DEBUGSERIAL.print(F("The last gauge readings were (g1, g2, g3, g4): ("));
    DEBUGSERIAL.print(iostream.outputMessage.gaugeOneReading);
    DEBUGSERIAL.print(F(", "));
    DEBUGSERIAL.print(iostream.outputMessage.gaugeTwoReading);
    DEBUGSERIAL.print(F(", "));
    DEBUGSERIAL.print(iostream.outputMessage.gaugeThreeReading);
    DEBUGSERIAL.print(F(", "));
    DEBUGSERIAL.print(iostream.outputMessage.gaugeFourReading);
    DEBUGSERIAL.print(F(")\n"));

    // print settings
    DEBUGSERIAL.print(F("is target reached "));
    DEBUGSERIAL.println(targetReached.all);
    DEBUGSERIAL.print(F("power saving "));
    DEBUGSERIAL.println(powerSaving);
    DEBUGSERIAL.print(F("disabled "));
    DEBUGSERIAL.println(disabled);
    DEBUGSERIAL.print(F("debug "));
    DEBUGSERIAL.println(debug);
    DEBUGSERIAL.print(F("timed action seconds "));
    DEBUGSERIAL.println(timedActionSecs);
    DEBUGSERIAL.print(F("runHz "));
    DEBUGSERIAL.println(runHz);
    DEBUGSERIAL.print(F("motor_hz "));
    DEBUGSERIAL.println(motor_hz);
    DEBUGSERIAL.print(F("gauge_hz "));
    DEBUGSERIAL.println(gauge_hz);
    DEBUGSERIAL.print(F("serial_hz "));
    DEBUGSERIAL.println(serial_hz);
    DEBUGSERIAL.print(F("publish_hz "));
    DEBUGSERIAL.println(publish_hz);
    DEBUGSERIAL.print(F("lastMotorTime "));
    DEBUGSERIAL.println(lastMotorTime);
    DEBUGSERIAL.print(F("lastGaugeTime "));
    DEBUGSERIAL.println(lastGaugeTime);
    DEBUGSERIAL.print(F("lastPublishTime "));
    DEBUGSERIAL.println(lastPublishTime);
    DEBUGSERIAL.print(F("lastSerialTime "));
    DEBUGSERIAL.println(lastSerialTime);

    iostream.endDebugMessage();
}
