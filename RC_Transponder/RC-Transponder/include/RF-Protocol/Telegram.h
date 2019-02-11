/*
 * Telegram.h
 *
 * Created: 08-02-2019 11:31:34
 *  Author: klo
 */ 


#ifndef TELEGRAM_H_
#define TELEGRAM_H_

#include "Arduino.h" // Needed for uint8 types mf.

#define MAX_PAYLOAD_LENGTH 25
#define HEADER_SIZE 5 // First 5 bytes are Header

typedef enum
{
	MSG_Request               = 0x01,
	MSG_Transponder_Data,
	MSG_Transponder_Status,
	MSG_Beacon_Broadcast,
	MSG_Relay_Request,
	MSG_Relay_Reply,
}ProtocolMSG_t;

/*
struct TelegramHeader
{
	uint16_t To;		   // Message To
	uint16_t From;		   // Message From
	ProtocolMSG_t MSG_ID;  // Message ID
	char more_data[MAX_PAYLOAD_LENGTH - 4 - sizeof(ProtocolMSG_t)];
};*/

class Telegram
{
	public:
	uint8_t Payload[MAX_PAYLOAD_LENGTH]; // Message Payload.
	uint8_t PayloadLength; // Length of the payload.

	
	Telegram();
	Telegram(uint8_t *data, uint8_t size);

	/*
	Telegram(uint8_t *payload, uint8_t payloadSize);
	Telegram(TelegramHeader *header, uint8_t size);*/

	

	// Functions for every messages
	virtual void ReadPayload( void ) = 0;		// Each messages should be able to parse/decode the payload to specific cases.
	virtual void GeneratePayload( void ) = 0;	// Each messages should be able to generate a payload based on values.
	virtual void SerialPrintMessage( void ) = 0;// Function for each meassage to print out the data to Serial.print
	uint8_t GetPayloadLength(void);			    // Function which returns the number of bytes of the payload.
	
	
	uint16_t To;		   // Message To
	uint16_t From;		   // Message From
	ProtocolMSG_t MSG_ID;  // Message ID

	void GenerateHeader(void);		
	
	private:
	void ReadPayloadHeader(void);
};




#endif /* TELEGRAM_H_ */