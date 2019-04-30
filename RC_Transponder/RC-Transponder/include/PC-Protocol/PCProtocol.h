/*
	PCProtocol.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 


#ifndef PCPROTOCOL_H_
#define PCPROTOCOL_H_

#include "Arduino.h"
#include "E28-2G4M20S.h" // RadioData_t struct. 
#include "RFService.h" // For RF protocol.

//class RFProtocol; // forward declaration.

class PCProtocol
{
	// Public functions to be used on all Messages
	public:
	PCProtocol(RFProtocol *inout, E28_2G4M20S *radio);

	void Service();
	
	// General helper functions and variables only used by inherited MSG classes.
	protected:
	



	// Parameters only used on Telegram mother class.
	private:
	void ApplicationCMDHandler();
	void RadioDataHandler();
	void WriteToSerial(RadioData_t *data);
	void clearInputData();

	int NumberOfBytesToRead=0;
	
	enum ApplicationCMD_t{
		HW_REPLY=0x00   // Application sends this to test if HW is on this USB port, HW must reply with same message.
	}ApplicationCMD;

	enum PayloadID_t{
		RADAR_APPLICATION_ID=0x00,
		RADIO_DATA_TO_RF,
		RADIO_DATA_TO_PC
	}PayloadID;
		
	enum StateMachine_t{
		LOOKING_FOR_START,
		READ_DATA_LENGTH,
		READ_PAYLOAD_ID,
		READ_PAYLOAD,
		READ_CRC1,
		READ_CRC2
	}SerialState = LOOKING_FOR_START;

	char _newChar;
	uint8_t data[MAX_PAYLOAD_LENGTH];
	uint8_t dataLength;
	uint8_t dataIndex;
	RFProtocol *_inout = NULL;
	E28_2G4M20S *RadioForCRC = NULL;
	RadioData_t input;
	uint8_t output[MAX_PAYLOAD_LENGTH];
	uint16_t CRC = 0;
};


#endif /* PCPROTOCOL_H_ */