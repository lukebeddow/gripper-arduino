#pragma once

// hardcode: are we wired or wireless (bluetooth)
#define LUKE_WIRELESS 0

#if LUKE_WIRELESS
  // bluetooth is connected to Serial2 for wireless coms
  #define BTSERIAL Serial2
  #define USBSERIAL Serial // for debugging
	#define DEBUGSERIAL Serial
#else
  // if wired then our 'bluetooth' is actually wired Serial
  #define BTSERIAL Serial
  #define USBSERIAL Serial2 // not useable
	#define DEBUGSERIAL Serial
#endif

class GripperCommunication
{
public:

	/* note that only commandBytes and setSpeedByte are set up to save x,y,z data
	from the input message, ALL others do not save this data */

	// define bytes for communication instructions
	static constexpr byte motorCommandByte_m = 100;
	static constexpr byte motorCommandByte_mm = 101;
	static constexpr byte jointCommandByte_m_rad = 102;
	static constexpr byte jointCommandByte_mm_deg = 103;
	static constexpr byte stepCommandByte = 104;
	static constexpr byte timedCommandByte_m = 105;

	static constexpr byte commandByteMinimum = 100; // set these based on above
	static constexpr byte commandByteMaximum = 105;

	// bytes for special behaviour and modifying settings
	static constexpr byte homeByte = 110;
	static constexpr byte powerSavingOnByte = 111;
	static constexpr byte powerSavingOffByte = 112;
	static constexpr byte stopByte = 113;
	static constexpr byte resumeByte = 114;
	static constexpr byte setSpeedByte = 115;
	static constexpr byte debugOnByte = 116;
	static constexpr byte debugOffByte = 117;
	static constexpr byte printByte = 118;
	static constexpr byte changeTimedActionByte = 119;
	static constexpr byte changeTimedActionPubEarlyByte = 120;
	static constexpr byte setGaugeHzByte = 121;
	static constexpr byte setPublishHzByte = 122;
	static constexpr byte setSerialHzByte = 123;
	static constexpr byte setMotorHzByte = 124;

	// information bytes and error codes
	static constexpr byte messageReceivedByte = 200;
	static constexpr byte messageFailedByte = 201;
	static constexpr byte targetNotReachedByte = 202;
	static constexpr byte targetReachedByte = 203;
	static constexpr byte invalidCommandByte = 204;
	static constexpr byte debugMessageByte = 205;

	// message signature bytes
	static constexpr byte specialByte = 253;
	static constexpr byte startMarkerByte = 254;
	static constexpr byte endMarkerByte = 250; // do not use 255 as max value more likely than others

	// how long is the message signature at each of the start and end
	static constexpr byte startEndSize = 3;

	// functions
	GripperCommunication();
	bool readInput();
	void publishOutput();
	void startDebugMessage();
	void endDebugMessage();

  // private, not recommended to use
  void publishEndTokens();
  void publishStartTokens();

	/* Message structures for input and output */
	struct InputMessage {
		byte instructionByte;
		byte pad1;
		byte pad2;
		byte pad3;
		float x;
		float y;
		float z;
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
		byte pad1;						 // giga aligns bytes in 4's, these are essential
		byte pad2;						 // giga aligns bytes in 4's, these are essential
		long gaugeOneReading;
		long gaugeTwoReading;
		long gaugeThreeReading;
		long gaugeFourReading;
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
	int _dataReceivedCount = 0;
	byte _signatureCount = 0;
	byte _inputBuffer[inputMessageSize];	
	bool _inputInProgress = false;
	bool _inputSuccess = false;
	
	bool debug_message_started = false;

};