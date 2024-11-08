// This class requires a global Serial2 connection in order to function

#include <Arduino.h>
#include <GripperCommunication.h>

GripperCommunication::GripperCommunication()
{
	/* Initialisation method */

	// initialise variables to zero
	inputMessage.instructionByte = 0;
	inputMessage.x = 0.0;
	inputMessage.y = 0.0;
	inputMessage.z = 0.0;

	outputMessage.informationByte = 0;
	outputMessage.isTargetReached = false;
	outputMessage.gaugeOneReading = 0;
	outputMessage.gaugeTwoReading = 0;
	outputMessage.gaugeThreeReading = 0;
	outputMessage.gaugeFourReading = 0;
	outputMessage.motorX_mm = -1;
	outputMessage.motorY_mm = -1;
	outputMessage.motorZ_mm = -1;
}

bool GripperCommunication::readInput()
{
	/* This member function reads any serial bytes into the buffer and detects
	if a full message has been received, returning true in this case and false
	if a full message has not been received. */

	_inputSuccess = false;

	while (BTSERIAL.available() > 0) {

		// extract the next byte from the serial buffer
		byte x = BTSERIAL.read();

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

	// constexpr bool debug_input_messages = false;
	// if (debug_input_messages) {
	// 	startDebugMessage();
	// 	DEBUGSERIAL.write("Msg length ");
	// 	DEBUGSERIAL.write(std::to_string(_dataReceivedCount).c_str());
	// 	DEBUGSERIAL.write("\nRaw bytes: {");
	// 	for (int i = 0; i < _dataReceivedCount - 1; i++) {
	// 		DEBUGSERIAL.write(_inputBuffer[i]);
	// 		DEBUGSERIAL.write(',');
	// 	}
	// 	DEBUGSERIAL.write(_inputBuffer[_dataReceivedCount - 1]);
	// 	DEBUGSERIAL.write('}');
	// 	endDebugMessage();
	// }

	/* for single byte messages, _dataReceivedCount is 1,
		 for x,y,z messages, _dataReceivedCount is 13
		 It currently works without checking for this */

	// extract the raw data, it is fine if we get garbage for x,y,z
	inputMessage.instructionByte = inputUnion.structure.instructionByte;
	inputMessage.pad1 = inputUnion.structure.pad1;
	inputMessage.pad2 = inputUnion.structure.pad2;
	inputMessage.pad3 = inputUnion.structure.pad3;
	inputMessage.x = inputUnion.structure.x;
	inputMessage.y = inputUnion.structure.y;
	inputMessage.z = inputUnion.structure.z;

	// now reset all variables before a new message comes in
	_inputSuccess = false;
	_dataReceivedCount = 0;
	_signatureCount = 0;

	return true;
}

void GripperCommunication::publishStartTokens()
{
  /* publish the message start tokens */

	for (int i = 0; i < startEndSize; i++) {
		BTSERIAL.write(startMarkerByte);
	}
}

void GripperCommunication::publishEndTokens()
{
  /* publish the message end tokens */

	// finish the message with the end marker
	for (int i = 0; i < startEndSize; i++) {
		BTSERIAL.write(endMarkerByte);
	}
}

void GripperCommunication::publishOutput()
{
	/* This member function pubishes a message on the serial connection */

	// set the data
	outputUnion.structure.informationByte = outputMessage.informationByte;
	outputUnion.structure.isTargetReached = outputMessage.isTargetReached;
	outputUnion.structure.gaugeOneReading = outputMessage.gaugeOneReading;
	outputUnion.structure.gaugeTwoReading = outputMessage.gaugeTwoReading;
	outputUnion.structure.gaugeThreeReading = outputMessage.gaugeThreeReading;
	outputUnion.structure.gaugeFourReading = outputMessage.gaugeFourReading;
	outputUnion.structure.motorX_mm = outputMessage.motorX_mm;
	outputUnion.structure.motorY_mm = outputMessage.motorY_mm;
	outputUnion.structure.motorZ_mm = outputMessage.motorZ_mm;

	publishStartTokens();
	
	for (int i = 0; i < outputDataSize; i++) {
		BTSERIAL.write(outputUnion.byteArray[i]);
	}

  publishEndTokens();
}

void GripperCommunication::startDebugMessage()
{
	/* begin a debug message */

	if (debug_message_started) return;

	publishStartTokens();
	DEBUGSERIAL.write(debugMessageByte);

	debug_message_started = true;
}

void GripperCommunication::endDebugMessage()
{
	/* finish a debug message */

	if (not debug_message_started) return;

	publishEndTokens();

	debug_message_started = false;
}