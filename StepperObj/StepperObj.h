#ifndef StepperObj_h
#define StepperObj_h

class StepperObj
{
public:
	StepperObj(int stepPin, int dirPin);
	void setClockwise(bool clockwise);
	float setDirection(float num);
	void setStepsPerRev(int steps);
	void setRPM(double rpm);
	void setTarget(int stepTarget);
	void setHomingRPM(double rpm);
	void rotate(double revs);
	void pulse();
	void rampedPulse();
	bool targetPulse();
	bool homePulse();
	bool wipeHomingFlag();
	void publishSteps();
	void addLimitSwitch(int limitPin, bool true_state);
	void setupWorkspace(float home_revs, float end_revs, int limit_pin,
		bool true_state);
	int getStep();

private:
	// core motor parameters
	bool _clockwise;								// logic level at dir pin for clockwise rotation
	int _stepPin;										// step pin no.
	int _dirPin;										// direction pin no.
	int _stepsPerRev;								// steps for one revolution
	unsigned long _pulseWidthMicro;	// pulse width in microseconds
	unsigned long _waitMicro;				// wait time between pulses in microseconds
	unsigned long _lastPulseMicros;	// time of the last step pulse in microseconds
	double _rpm;										// motor speed in revs per minute

	// safety paramters
	double _maxRPM;									// maximum achievable speed
	double _limitPPS;								// maximum allowable pulses per second

	// limit pin, workspace, and target position parameters
	int _limitPin;									// pin that limit switch is connected to
	int _stepNumber;								// which step are we on
	int _stepTarget;								// which step number is our target
	int _increment;									// value of one step, +1 or -1
	int _homeNumber;								// which step is home position
	int _endNumber;									// which step is the end of the workspace
	int _homingSeq;									// homing sequence flag (0 = not homing)
	float _homingSpeed;							// speed at which homing should occur
	bool _limitSwitch;							// are we using a limit switch
	bool _limitActive;							// logic level when limit switch is active

	// ramped startup acceleration parameters
	long _moveState;								// is the motor moving? 0=still
	long _fullSpeedState;						// moveState at full speed
	unsigned long _startSpinMicro;	// time at which motor began to spin
	unsigned long _maxWaitMicro;		// time after which motor is considered to be still
	float _noRampPPS;								// pulses per second the motor can reach without ramping
	float _noRampRPM;								// speed in rpm that motor can reach without ramping
	float _rampRatePPS;							// pulses per second gained per millisecond during ramp
	bool _changedDirection;					// have we just set to a different direction

	// ramp 2
	bool _ramping;									// are we in a ramping sequence
	unsigned long _startRampMicro;	// time between pulses at starting of ramp
	unsigned long _endRampMicro;		// target time between pulses at full speed
	unsigned long _rampReductionMicro;	// reduction in time between pulses during ramping
	unsigned long _rampWaitMicro;		// time between steps for ramping

};

#endif
