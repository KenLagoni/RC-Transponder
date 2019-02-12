/*
	Telegram_MSG_1.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 



#ifndef TELEGRAM_MSG_1_H_
#define TELEGRAM_MSG_1_H_

#include "Arduino.h" // Needed for uint8 types mf.
#include "Telegram.h"

class Telegram_MSG_1 : public Telegram
{
	public:
	virtual void ReadPayload( void );		 // Each messages should be able to parse/decode the payload to specific cases.
	virtual void GeneratePayload( void );	 // Each messages should be able to generate a payload based on values.
	virtual void SerialPrintMessage( void ); // Function for each massage to print out the data to Serial.print	
	ProtocolMSG_t GetMsgRequest( void );
	
	Telegram_MSG_1(uint16_t _to, uint16_t _from, ProtocolMSG_t _msgRequest);	// Constructor to create message from variables.
	Telegram_MSG_1(uint8_t *data, uint8_t size);								// Constructor to create messages from payload array.

	private:
	ProtocolMSG_t MSG_req;  // Message requested.

};


#endif /* TELEGRAM_MSG_1_H_ */