/*
 * Class to control a simple stepper motor via a motor board, hence
 * all we need to control is the step and direction pins
 */

#include <Arduino.h>
#include <StepperObj.h>

StepperObj::StepperObj(int stepPin, int dirPin)
{ /* Initialisation method */

	/* ----- Initialise variables to default values ----- */

	// initialise core motor parameters to defaults
	_clockwise = true;			// logic level at dir pin for clockwise rotation
	_stepsPerRev = 200;			// steps for one revolution
	_pulseWidthMicro = 5;		// pulse width in microseconds
													// -> the A4988 is rising edge triggered
	_lastPulseMicros = 0;		// time of the last step pulse in microseconds
	_rpm = 100.0;						// default motor speed in revs per minute

	// define safety parameters
	_limitPPS = 4000;				// maximum allowable pulses per second
													// -> typically lose torque above 1400pps
													// -> don't exceed 20,000pps
													// -> 2000pps = 300rpm @ 400 steps per rev

	// initialise limit pin and workspace parameters to defaults
	_stepNumber = 0;				// which step are we on
	_stepTarget = 0;				// which step number is our target
	_increment = 1;					// value of one step, +1 or -1
	_homeNumber = 0;				// which step is home position
	_endNumber = 1;					// which step is the end of the workspace
	_homingSeq = 0;					// homing sequence flag (0 = not homing)
	_homingSpeed = 100.0;		// speed at which homing should occur
	_limitSwitch = false;		// are we using a limit switch
	_limitActive = true;		// logic level when limit switch is active

	/* ----- Configure the motor pins ----- */

	// save which pins to use
	_stepPin = stepPin;
	_dirPin = dirPin;

	// set these pins to output
	pinMode(_stepPin, OUTPUT);
	pinMode(_dirPin, OUTPUT);

	// default is both low
	digitalWrite(_stepPin, LOW);
	digitalWrite(_dirPin, LOW);

	/* ----- Calculate timings ----- */

	// determine how long to wait between pulses in microseconds
	double timePerRev = 60.0 / _rpm;
	double timePerStep = timePerRev / _stepsPerRev;
	double timePerStepMicro = 1000.0 * 1000.0 * timePerStep;
	_waitMicro = timePerStepMicro - _pulseWidthMicro;

	_waitMicro = (1000.0 * 1000.0) / ((_rpm * _stepsPerRev) / 60.0);
	
	// determine the maximum speed from the max pulses per second
	_maxRPM = 60.0 * _limitPPS * (1.0 / _stepsPerRev);

	/* ----- Ramping acceleration parameters ----- */

	// startup acceleration parameters
	_noRampPPS = 500;							// speed that the motor can reach without ramping
	_changedDirection = false;		// initialise to false
	_ramping = true;							// are we currently ramping up to speed
	_rampReductionMicro = 40;			// rate at which we reduce time between pulses
	_rampWaitMicro = _waitMicro;	// time between steps for ramping

	// determine the no ramp rpm, below which we will not ramp
	_noRampRPM = 60.0 * _noRampPPS * (1.0 / _stepsPerRev);

	// determine the wait time between pulses at the noRampPPS
	_startRampMicro = (1000.0 * 1000.0) / _noRampPPS;

}

void StepperObj::setClockwise(bool clockwise)
{ /* This method sets which logic level at the direction pin corresponds to
  a clockwise rotation. Default is high */
	
	_clockwise = clockwise;
}

float StepperObj::setDirection(float num)
{ /* Sets the direction of the motor, recieves a positive number
  for forwards, or a negative number for backwards. Does nothing
  with a value of zero. Converts num to abs(num) */

	// set the direction
	if (num > 0) {
		digitalWrite(_dirPin, _clockwise);
		// is this a change of direction
		if (_increment == -1) {
			_changedDirection = true;
		}
		else {
			_changedDirection = false;
		}
		// set the new direction
		_increment = 1;
	}
	else if (num < 0) {
		digitalWrite(_dirPin, not _clockwise);
		// is this a change of direction
		if (_increment == 1) {
			_changedDirection = true;
		}
		else {
			_changedDirection = false;
		}
		// set the new direction
		_increment = -1;
		// we want to return a positive number
		num *= -1;
	}

	return num;
}

void StepperObj::setStepsPerRev(int steps)
{ /* This method defines the step angle */
	_stepsPerRev = steps;

	// determine the new wait time between pulses to achieve the rpm
	_waitMicro = (1000.0 * 1000.0) / ((_rpm * _stepsPerRev) / 60.0);

	// determine the maximum speed from the max pulses per second
	_maxRPM = 60.0 * _limitPPS * (1.0 / _stepsPerRev);

	// determine the new maximum rpm without ramping
	_noRampRPM = 60.0 * _noRampPPS * (1.0 / _stepsPerRev);
}

void StepperObj::setRPM(double rpm)
{ /* This method sets the speed of the motor and writes to the direction
  pin. Use a negative number for going backwards */

	rpm = setDirection(rpm);

	// if rpm is close to zero, this motor should not move
	if (rpm < 1) {
		_rpm = 0;
		// set the expression (_waitMicros + _lastPulseMicros) as large as possible
		// unsigned long max value is (2^32 - 1), or 4,294,967,295
		// this way the pulse() function will never trigger another step
		_waitMicro = 4294967295 - _lastPulseMicros;
		return;
	}

	// check if the requested speed exceeds our maximum possible
	if (rpm > _maxRPM) {
		// limit the speed to the maximum
		rpm = _maxRPM;
	}

	// set the speed
	_rpm = rpm;

	// how long will we wait between steps in microseconds (float->int)
	_waitMicro = (1000.0 * 1000.0) / ((_rpm * _stepsPerRev) / 60.0);

}

void StepperObj::setHomingRPM(double rpm)
{ /* This function sets the speed for homing */

	// check for bad inputs
	if (rpm <= 0) {
		// set to a default instead
		_homingSpeed = 100.0;
	}
	else {
		_homingSpeed = rpm;
	}
}

void StepperObj::rotate(double revs)
{ /* This method turns the motor */

	revs = setDirection(revs);

	// how many steps do we need for this revolution (round down float->int)
	int turnSteps = revs * _stepsPerRev;

	// loop through all the demanded steps
	for (int i = 0; i < turnSteps; i++)
	{
		// pulse the step pin to rotate the motor
		digitalWrite(_stepPin, HIGH);
		delayMicroseconds(_pulseWidthMicro);

		// pause to ensure we run at the right speed
		digitalWrite(_stepPin, LOW);
		delayMicroseconds(_waitMicro);
	}
}

void StepperObj::pulse()
{  /* This method is designed for driving multiple stepper motors at the same
   time. This method pulses the stepper no faster than the set RPM and should
   be called at a faster rate than pulses are needed */

	// we assume that the rpm is already set, and the direction pin is already
	// written to the correct logic level

	// get the current time
	unsigned long currentTimeMicros = micros();

	// pulse only if enough time has elapsed
	if (_lastPulseMicros + _waitMicro < currentTimeMicros)
	{
		// if we are using limit switches
		if (_limitSwitch) {

			// check if the limit switch is high
			if (digitalRead(_limitPin) == _limitActive) {
				// reset the step count
				_stepNumber = 0;

				// we can only go forwards, not backwards
				if (_increment == -1) {
					return;
				}
			}

			// check if we have reached the limit of our workspace
			if (_stepNumber == _endNumber) {
				// we can only go backwards, not forwards
				if (_increment == 1) {
					return;
				}
			}
		}

		// deliver one step pulse
		digitalWrite(_stepPin, HIGH);
		//delayMicroseconds(_pulseWidthMicro);
		digitalWrite(_stepPin, LOW);

		// increment the step counter
		_stepNumber += _increment;

		// increment the move state (for ramped starts only)
		_moveState += 1;

		// update the record of the last pulse
		_lastPulseMicros = micros();
	}

	return;
}

void StepperObj::rampedPulse()
{
	/* This function ramps up the motor speed when the motor is slow */

	// calculate the time since the last pulse
	unsigned long elapsedTime = micros() - _lastPulseMicros;

	// if not enough time has elapsed since the last pulse (full speed)
	if (elapsedTime < _waitMicro) {
		return;
	}

	// are we at high enough speed to consider ramping
	if (_rpm > _noRampRPM) {
		// has the motor stopped turning or changed direction
		if ((elapsedTime > _startRampMicro + 1000) or _changedDirection) {
			// we need to ramp, reset ramping variables
			_ramping = true;
			_changedDirection = false;
			_rampWaitMicro = _startRampMicro;
		}
	}
	else {
		_ramping = false;
	}

	// are we currently ramping up speed
	if (_ramping) {
		// check that enough time has elapsed for a ramping pulse
		if (elapsedTime < _rampWaitMicro) {
			return;
		}
		// reduce the time between pulses
		_rampWaitMicro -= _rampReductionMicro;
		// check if we have finished the ramp
		if (_rampWaitMicro <= _waitMicro) {
			_ramping = false;
			_rampWaitMicro = _waitMicro;
		}
	}
	else {
		_rampWaitMicro = _waitMicro;
	}

	// if we are using limit switches, certain motions are forbidden
	if (_limitSwitch) {

		// check if the limit switch is high
		if (digitalRead(_limitPin) == _limitActive) {
			// reset the step count
			_stepNumber = 0;

			// we can only go forwards, not backwards
			if (_increment == -1) {
				return;
			}
		}

		// check if we have reached the limit of our workspace
		if (_stepNumber == _endNumber) {
			// we can only go backwards, not forwards
			if (_increment == 1) {
				return;
			}
		}
	}

	// deliver one step pulse
	digitalWrite(_stepPin, HIGH);
	//delayMicroseconds(_pulseWidthMicro);
	digitalWrite(_stepPin, LOW);

	// increment the step counter
	_stepNumber += _increment;

	// update the record of the last pulse
	_lastPulseMicros = micros();

	return;
}

void StepperObj::setTarget(int stepTarget) 
{
	/* This function sets a step target for the motor */

	// safety checks
	if (stepTarget < _homeNumber) {
		stepTarget = _homeNumber + 1;
	}
	else if (stepTarget > _endNumber) {
		stepTarget = _endNumber - 1;
	}

	_stepTarget = stepTarget;
}

bool StepperObj::targetPulse() 
{
	/* This function pulses the motor until the step target is reached */

	int diff = _stepTarget - _stepNumber;

	// if we haven't yet reached our target
	if (diff != 0) {

		rampedPulse();

		// if we are going the wrong way, the difference goes up
		if (abs(diff) < abs(_stepTarget - _stepNumber)) {
			// reverse the direction
			setDirection(_increment * -1);
		}

		return false;
	}

	// if we have reached our target
	return true;
}

bool StepperObj::homePulse()
{  /* This method is designed for homing a motor, and every call of this
   function moves the motor towards home. When home is reached, the
   function returns true, otherwise it returns false */

	// has the homing sequence started yet
	if (_homingSeq == 0) {
		
		// set flag to phase 1
		_homingSeq = 1;

		// first, we need to hit our limit switch
		setRPM(-1 * _homingSpeed);
	}
	
	// complete the homing sequence
	if (_homingSeq == 1) {

		// whilst we have not reached the limit switch
		if (not (digitalRead(_limitPin) == _limitActive and
			_stepNumber == 0)) {
			rampedPulse();
		}
		else {
			// change direction
			setRPM(_homingSpeed);

			// set flag to phase 2
			_homingSeq = 2;

			_moveState = 0;
		}
	}
	else if (_homingSeq == 2) {
		
		// whilst we have not reached the home position
		if (_stepNumber != _homeNumber)
		{
			rampedPulse();
		}
		else {
			// reset flag to not homing
			_homingSeq = 0;

			_stepTarget = _homeNumber;

			return true;
		}
	}

	return false;
}

bool StepperObj::wipeHomingFlag()
{
	/* This function can be called to reset the homing sequence */
	_homingSeq = 0;
}

void StepperObj::publishSteps()
{ /* This method publishes the step count */

	Serial.print("Step number: ");
	Serial.print(_stepNumber);
	Serial.print('\n');
}

void StepperObj::addLimitSwitch(int limitPin, bool true_state)
{ /* This method adds a limit switch to the motor logic at the given pin,
  it also needs to know whether LOW or HIGH should trigger the logic */

	// setup the pin
	pinMode(limitPin, INPUT_PULLUP);
	_limitPin = limitPin;
	
	// save which state (low/high) means switch is active
	_limitActive = true_state;

	// save a flag that specifies we have a limit switch
	_limitSwitch = true;

}

void StepperObj::setupWorkspace(float home_revs, float end_revs, int limitPin,
	bool true_state)
{ /* This method adds a limit switch and sets up a home and end position for the
  motor workspace */

	// check we have good inputs
	if (home_revs >= end_revs) {
		return;
	}
	if (home_revs < 0 or end_revs < 0) {
		return;
	}

	// add a limit switch
	addLimitSwitch(limitPin, true_state);

	// convert revolutions to number of steps (float -> int)
	_homeNumber = _stepsPerRev * home_revs;
	_endNumber = _stepsPerRev * end_revs;

}

int StepperObj::getStep()
{
	/* This function returns the step the motor is on */
	return _stepNumber;
}

int StepperObj::setStep(int step)
{
	/* for testing: can set the step number */

	_stepNumber = step;
}