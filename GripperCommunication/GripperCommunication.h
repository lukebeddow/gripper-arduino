#pragma once

class GripperCommunication
{
public:
	// define bytes for communication instructions
	static constexpr byte sendCommandByte = 100;
	static constexpr byte homeByte = 101;
	static constexpr byte getTargetStatusByte = 102;
	static constexpr byte requestGaugeByte = 103;

	static constexpr byte messageReceivedByte = 200;
	static constexpr byte messageFailedByte = 201;
	static constexpr byte targetNotReachedByte = 202;
	static constexpr byte targetReachedByte = 203;

	static constexpr byte specialByte = 253;
	static constexpr byte startMarkerByte = 254;
	static constexpr byte endMarkerByte = 255;

	static constexpr byte startEndSize = 2;

	// functions
	GripperCommunication();
	bool readInput();
	void publishOutput();

	/* Message structures for input and output */
	struct InputMessage {
		byte instructionByte;
		float radius;
		float angle;
		float palm;

	};
	InputMessage inputMessage;

	static const int inputDataSize = sizeof(inputMessage);
	static const int inputMessageSize = inputDataSize + (startEndSize * 2);

	union InputUnion {
		InputMessage structure;
		byte byteArray[inputDataSize];
	};
	InputUnion inputUnion;

	struct OutputMessage {
		byte isTargetReached;  // message needs to be whole number of bytes
		long gaugeOneReading;
		long gaugeTwoReading;
		long gaugeThreeReading;
	};
	OutputMessage outputMessage;

	static const int outputDataSize = sizeof(outputMessage);
	static const int outputMessageSize = outputDataSize + (startEndSize * 2);

	union OutputUnion {
		OutputMessage structure;
		byte byteArray[outputDataSize];
	};
	OutputUnion outputUnion;

private:

	// variables
	byte _dataReceivedCount = 0;
	byte _signatureCount = 0;
	byte _inputBuffer[inputMessageSize];	
	bool _inputInProgress = false;
	bool _inputSuccess = false;

};