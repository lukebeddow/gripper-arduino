// This class requires a global Serial2 connection in order to function

#include <Arduino.h>
#include <GripperCommunication.h>

GripperCommunication::GripperCommunication()
{
	/* Initialisation method */

	// initialise variables to zero
	inputMessage.instructionByte = 0;
	inputMessage.radius = 0.0;
	inputMessage.angle = 0.0;

	outputMessage.isTargetReached = false;
	outputMessage.gaugeOneReading = 0;
	outputMessage.gaugeTwoReading = 0;
	outputMessage.gaugeThreeReading = 0;
}

bool GripperCommunication::readInput()
{
	/* This member function reads any serial bytes into the buffer and detects
	if a full message has been received, returning true in this case and false
	if a full message has not been received. */

	_inputSuccess = false;

	while (Serial2.available() > 0) {

		// extract the next byte from the serial buffer
		byte x = Serial2.read();

		// are we in the middle of reading a message
		if (_inputInProgress) {
			_inputBuffer[_dataReceivedCount] = x;
			_dataReceivedCount++;
		}

		/* The below logic checks for start and end signatures */
		// if the message hasn't started
		if (not _inputInProgress) {
			// if we reach a start marker, the message might be starting
			if (x == startMarkerByte) {
				_signatureCount++;
				if (_signatureCount == startEndSize) {
					_inputInProgress = true;
					_inputSuccess = false;
					_signatureCount = 0;
				}
			}
			// else we do not have multiple start markers in a row
			else {
				_signatureCount = 0;
			}
		}
		// else the message has started
		else {
			// check for end markers in a row
			if (x == endMarkerByte) {
				_signatureCount++;
				if (_signatureCount == startEndSize) {
					// remove end markers from the message
					_dataReceivedCount -= startEndSize;
					_inputSuccess = true;
					_inputInProgress = false;
					break;
				}
			}
			// else we do not have multiple end markers in a row
			else {
				_signatureCount = 0;
			}
		}
	}

	// have we received the full message yet
	if (_inputSuccess == false) {
		return false;
	}

	// we have the full message, now decode it
	for (int i = 0; i < _dataReceivedCount; i++) {
		inputUnion.byteArray[i] = _inputBuffer[i];
	}

	// extract the raw data
	inputMessage.instructionByte = inputUnion.structure.instructionByte;

	// if a command has been sent, there will be more data
	if (inputUnion.structure.instructionByte == sendCommandByte) {
		inputMessage.radius = inputUnion.structure.radius;
		inputMessage.angle = inputUnion.structure.angle;
		inputMessage.palm = inputUnion.structure.palm;
	}

	// now reset all variables before a new message comes in
	_inputSuccess == false;
	_dataReceivedCount = 0;
	_signatureCount = 0;

	return true;
}

void GripperCommunication::publishOutput()
{
	/* This member function pubishes a message on the serial connection */

	// set the data
	outputUnion.structure.isTargetReached = outputMessage.isTargetReached;
	outputUnion.structure.gaugeOneReading = outputMessage.gaugeOneReading;
	outputUnion.structure.gaugeTwoReading = outputMessage.gaugeTwoReading;
	outputUnion.structure.gaugeThreeReading = outputMessage.gaugeThreeReading;
	outputUnion.structure.motorXPosition = outputMessage.motorXPosition;
	outputUnion.structure.motorYPosition = outputMessage.motorYPosition;
	outputUnion.structure.motorZPosition = outputMessage.motorZPosition;

	// begin the message with the start marker
	for (int i = 0; i < startEndSize; i++) {
		Serial2.write(startMarkerByte);
	}

	// loop through sending the entire byte array
	for (byte x : outputUnion.byteArray) {
		Serial2.write(x);
	}

	// finish the message with the end marker
	for (int i = 0; i < startEndSize; i++) {
		Serial2.write(endMarkerByte);
	}
}