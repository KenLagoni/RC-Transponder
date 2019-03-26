/*
	Telegram_MSG_3.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 



#ifndef Telegram_MSG_3_H_
#define Telegram_MSG_3_H_

#include "Arduino.h" // Needed for uint8 types mf.
#include "Telegram.h"

#define MSG_3_PAYLOAD_SIZE 1

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
	virtual uint8_t * GetRadioMSG(void);		 // Function which returns the payload length for Radio transmittion.
	virtual uint8_t * GetSerialMSG(void);        // Function which returns the payload length for Serial transmittion.
//	virtual bool RadioCRCValid(void);
//	virtual bool SerialCRCValid(void);
		
	ProtocolCMD_t GetCommand(void);				// Returns the Command.
		
	// Constructor to create message from variables.
	Telegram_MSG_3(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4, ProtocolCMD_t _command);
	
	// Constructor to create messages from payload array.
	Telegram_MSG_3(uint8_t *data, uint8_t size);

	protected:
	virtual void ReadPayload( void );		 // Each messages should be able to parse/decode the payload to specific cases.
	virtual void GeneratePayload( uint8_t *data);	 // Each messages should be able to generate a payload based on values.
	
	private:
	void SetMSGLength();
	ProtocolCMD_t cmd;
};


#endif /* Telegram_MSG_3_H_ */