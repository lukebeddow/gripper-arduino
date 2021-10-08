#include <Arduino.h>
#include <Gripper_v2.h>

Gripper::Gripper()
{
    /* Class constructor */

    setupMotors();
    setPins();

    /* ----- Setup the gauges ----- */
    gauge1.begin(SG1_dt, SG1_sck);
    //gauge1.set_scale(loadcell_divider);
    //gauge1.set_offset(loadcell_offset);

    gauge2.begin(SG2_dt, SG2_sck);
    gauge3.begin(SG3_dt, SG3_sck);

    // set to defaults (0=joystick, 1=serial)
    operatingMode = 1;
    targetReached = true;
    newReadGauge1 = false;
    newReadGauge2 = false;
    newReadGauge3 = false;
    powerSaving = false;
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

    // calculate mm per step, assume motor parameters unchanged from here!
    mmPerStep.x = (params.xy_lead / params.xy_gear_red)
        / (m.stepsPerRev * m.microstep.x);
    mmPerStep.y = (params.xy_lead / params.xy_gear_red)
        / (m.stepsPerRev * m.microstep.y);
    mmPerStep.z = params.z_lead / (m.stepsPerRev * m.microstep.z);
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
        home();
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

void Gripper::home()
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

void Gripper::prepareTarget(float radius, float angle, float palm)
{
    /* This function calculates the targets for a given input */

    // calculate what step the target displacement corresponds to (x motor)
    float revs_x = (params.xy_max - radius) / (params.xy_lead / params.xy_gear_red);
    control.stepTarget.x = revs_x * m.stepsPerRev * m.microstep.x;

    // calculate what step for target angle (y motor)
    float y_adjust = params.xy_diff * sin(angle * (3.141593 / 180.0));
    float revs_y = revs_x + (y_adjust / (params.xy_lead / params.xy_gear_red));
    control.stepTarget.y = revs_y * m.stepsPerRev * m.microstep.y;

    // calculate what step for target palm position (z motor)
    float revs_z = (params.z_max - palm) / (params.z_lead);
    control.stepTarget.z = revs_z * m.stepsPerRev * m.microstep.z;

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

bool Gripper::readGauges()
{
    /* Check if data can be extracted from strain gauges, read if so */

    // check to see if gauges have a new reading
    if (gauge1.is_ready() and not newReadGauge1) {
        // read the gauge (takes 380us)
        iostream.outputMessage.gaugeOneReading = gauge1.read();
        newReadGauge1 = true;
    }
    if (gauge2.is_ready() and not newReadGauge2) {
        // read the gauge (takes 380us)
        iostream.outputMessage.gaugeTwoReading = gauge2.read();
        newReadGauge2 = true;
    }
    if (gauge3.is_ready() and not newReadGauge3) {
        // read the gauge (takes 380us)
        iostream.outputMessage.gaugeThreeReading = gauge3.read();
        newReadGauge3 = true;
    }

    // if all the gauges are ready to publish, return true
    if (newReadGauge1 and newReadGauge2 and newReadGauge3) {
        newReadGauge1 = false;
        newReadGauge2 = false;
        newReadGauge3 = false;
        return true;
    }

    return false;
}

bool Gripper::checkSerial()
{
    /* This function checks to see if there is an incoming serial message */

    if (Serial2.available() >= iostream.inputMessageSize) {
        // read any input bytes, see if we received a message
        if (iostream.readInput()) {
            // check to see what type of message we received
            switch (iostream.inputMessage.instructionByte) {
            case iostream.homeByte:
                homingSequence();
                break;
            case iostream.sendCommandByte:
                // FOR TESTING
                if (iostream.inputMessage.radius == 0 and
                    iostream.inputMessage.angle == 0 and
                    iostream.inputMessage.palm == 0) {
                    home();
                    break;
                }
                prepareTarget(iostream.inputMessage.radius,
                    iostream.inputMessage.angle,
                    iostream.inputMessage.palm);
                break;
            case iostream.getTargetStatusByte:
                // REMOVE THIS?
                break;
            case iostream.requestGaugeByte:
                // REMOVE THIS?
                break;
            }

            return true;
        }
    }

    return false;
}

void Gripper::runMotors(const int loopMillis)
{
    /* This function pulses the motors based on the current operation mode */

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

void Gripper::setOutputMessagePosition()
{
    /* This function sets the output message with the current motor position */

    iostream.outputMessage.xMotorPosition = params.xy_max
        - motorX.getStep() * mmPerStep.x;

    iostream.outputMessage.yMotorPosition = params.xy_max
        - motorY.getStep() * mmPerStep.y;

    iostream.outputMessage.zMotorPosition = motorZ.getStep() * mmPerStep.z;
}

void Gripper::checkInputs()
{
    /* This function checks to see if the gripper is receiving inputs */

    // check the two input streams for commands
    readJoystick();
    checkSerial();

    // check the ADC inputs for the strain gauges
    if (readGauges()) {
        // prepare and publish the output message
        iostream.outputMessage.isTargetReached = targetReached;
        setOutputMessagePosition();
        iostream.publishOutput();
    }

    // check if we have reached our target (n/a for joystick mode)
    if (not targetReached and operatingMode != 0) {
        if (m.targetReached.x and
            m.targetReached.y and
            m.targetReached.z) {
            targetReached = true;
        }
    }
}