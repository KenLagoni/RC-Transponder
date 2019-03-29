/*
	SerialProtocol.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 


#ifndef SERIALPROTOCOL_H_
#define SERIALPROTOCOL_H_

#include "Arduino.h"

//#include "RFProtocol.h"
 
#define DATA_BUFFER_SIZE 64


struct SerialData_t
{
	uint8_t payload[DATA_BUFFER_SIZE];
	uint8_t payloadLength;
};


class SerialProtocol
{
	// Public functions to be used on all Messages
	public:
	//SerialProtocol(RFProtocol *inout);
	SerialProtocol(){};
	
	void Service();
	
	// General helper functions and varibels only used by inherited MSG classes.
	protected:
	
	// Parameters only used on Telegram mother class.
	private:
	int NumberOfBytesToRead=0;
	
	enum StateMachine{
		LOOKING_FOR_START,
		READ_MSG_LENGTH,
		READ_DATA
	};

	StateMachine SerialState = LOOKING_FOR_START;
	char _newChar;
	uint8_t data[DATA_BUFFER_SIZE];
	uint8_t dataLength;
	uint8_t dataIndex;
	//RFProtocol *_inout = NULL;
};


#endif /* SERIALPROTOCOL_H_ */