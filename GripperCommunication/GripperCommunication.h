#pragma once

class GripperCommunication
{
public:
	// define bytes for communication instructions
	static constexpr byte motorCommandByte_m = 100;
	static constexpr byte motorCommandByte_mm = 101;
	static constexpr byte jointCommandByte_m_rad = 102;
	static constexpr byte jointCommandByte_mm_deg = 103;
	static constexpr byte stepCommandByte = 104;

	static constexpr byte commandByteMinimum = 100; // set these based on above
	static constexpr byte commandByteMaximum = 104;

	// bytes for special behaviour and modifying settings
	static constexpr byte homeByte = 110;
	static constexpr byte powerSavingOnByte = 111;
	static constexpr byte powerSavingOffByte = 112;
	static constexpr byte stopByte = 113;
	static constexpr byte resumeByte = 114;

	// information bytes and error codes
	static constexpr byte messageReceivedByte = 200;
	static constexpr byte messageFailedByte = 201;
	static constexpr byte targetNotReachedByte = 202;
	static constexpr byte targetReachedByte = 203;
	static constexpr byte invalidCommandByte = 204;

	// message signature bytes
	static constexpr byte specialByte = 253;
	static constexpr byte startMarkerByte = 254;
	static constexpr byte endMarkerByte = 255;

	// how long is the message signature at each of the start and end
	static constexpr byte startEndSize = 2;

	// functions
	GripperCommunication();
	bool readInput();
	void publishOutput();

	/* Message structures for input and output */
	struct InputMessage {
		byte instructionByte;
		float x;
		float y;
		float z;
		// byte speedX;
		// byte speedY;
		// byte speedZ;

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
		byte informationByte;	 // used for reporting error codes etc
		byte isTargetReached;  // message needs to be whole number of bytes
		long gaugeOneReading;
		long gaugeTwoReading;
		long gaugeThreeReading;
		float motorX_mm;
		float motorY_mm;
		float motorZ_mm;
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