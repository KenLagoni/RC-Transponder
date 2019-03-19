/*
	Telegram_MSG_3.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 



#ifndef Telegram_MSG_3_H_
#define Telegram_MSG_3_H_

#include "Arduino.h" // Needed for uint8 types mf.
#include "Telegram.h"

typedef enum
{
	CMD_Request_Transponder_Beacon        = 0x01,
	CMD_Request_NEXT_Beacon_Relay,
	CMD_Do_Power_Off,
}ProtocolCMD_t;

class Telegram_MSG_3 : public Telegram
{
	public:
	virtual void SerialPrintMessage( void );	// Function for each massage to print out the data to Serial.print.
	ProtocolCMD_t GetCommand(void);				// Returns the Command.
		
	// Constructor to create message from variables.
	Telegram_MSG_3(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4, ProtocolCMD_t _command);
	
	// Constructor to create messages from payload array.
	Telegram_MSG_3(uint8_t *data, uint8_t size);

	protected:
	virtual void ReadPayload( void );		 // Each messages should be able to parse/decode the payload to specific cases.
	virtual void GeneratePayload( void );	 // Each messages should be able to generate a payload based on values.

	private:
	ProtocolCMD_t cmd;
};


#endif /* Telegram_MSG_3_H_ */