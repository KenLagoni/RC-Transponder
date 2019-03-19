/*
	Telegram.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 



#ifndef TELEGRAM_H_
#define TELEGRAM_H_

#include "Arduino.h" // Needed for uint8 types mf.

#define MAX_PAYLOAD_LENGTH 45
#define HEADER_SIZE 17 

typedef enum
{
	MSG_Beacon_Broadcast        = 0x01,
	MSG_Beacon_Relay,
	MSG_Command,
}ProtocolMSG_t;


class Telegram
{
	// Public functions to be used on all Messages
	public:
	uint8_t GetPayloadLength(void);			     // Function which returns the number of bytes of the payload.
	uint8_t * GetPayload(void);				     // Function which returns a pointer to the payload.
	uint8_t GetFrom(void);						 // Function which returns the transmitter of the mesage.
	virtual void SerialPrintMessage( void ) = 0; // Function for each message to print out the data to Serial.print	
	bool TelegramValid(void);					 // Function for each message, must only return true if CRC is valid.  
	bool TelegramMatchUniqueID(uint32_t _destinationID_1, uint32_t _destinationID_2, uint32_t _destinationID_3, uint32_t _destinationID_4); // Function will compare input destination ID with message, and return true if message it is a match.

	// Message To using 128 bit unique serial number.
	uint32_t Unique_ID_1, Unique_ID_2, Unique_ID_3, Unique_ID_4;
	
				
	// General helper functions and varibels only used by inherited MSG classes.
	protected:
	Telegram();
	Telegram(uint8_t *data, uint8_t size);

	// Functions for every messages
	virtual void ReadPayload( void ) = 0;		  // Each messages should be able to parse/decode the payload to specific cases.
	virtual void GeneratePayload( void ) = 0;	  // Each messages should be able to generate a payload based on values.
	void GenerateHeader(void);					  // Helper function to generate the Payload header based on variables.
	uint16_t CalculateCRC(uint8_t Payloadlength); // Helper function to calculate 16bit crc on Payload array

	uint8_t Payload[MAX_PAYLOAD_LENGTH];		// Message Payload.
	uint8_t PayloadLength;						// Length of the payload.

	ProtocolMSG_t MSG_ID;						// Message ID
	int8_t rssi;
	int8_t snr;
	uint16_t CRC;
	bool crcValid;								// True if CRC is ok.
	
	// Parameters only used on Telegram mother class.
	private:
	void ReadPayloadHeader(void);				// Helper function to read the payload header.
};




#endif /* TELEGRAM_H_ */