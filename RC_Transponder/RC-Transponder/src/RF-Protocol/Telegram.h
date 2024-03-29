/*
	Telegram.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 



#ifndef TELEGRAM_H_
#define TELEGRAM_H_

#include "E28-2G4M20S.h" // Needed for RadioData_t
#include <string> // Std:string not Arduino STring

#define HEADER_SIZE 17 

typedef enum
{
	MSG_Beacon_Broadcast=0x01,
	MSG_Beacon_Relay,
	MSG_Command,
	UNKOWN,
}ProtocolMSG_t;

// Command for MSG 3
typedef enum
{
	CMD_Request_Transponder_Beacon        = 0x01,
	CMD_Request_NEXT_Beacon_Relay,
	CMD_Do_Power_Off,
	CMD_Simulate_run_on_battery,
}ProtocolCMD_t;


class Telegram
{
	// Public functions to be used on all Messages
	public:
	virtual void SerialPrintMessage( void )=0; // Function for each message to print out the data to Serial.print	
	RadioData_t * GetRadioData();
	Telegram(){};
	virtual ~Telegram(){};					 	 // Destructor is virtual to ensure correct destructor is used when deleting telegrams.
	void setup(ProtocolMSG_t _MSG_ID,  uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4);
	bool TelegramMatchUniqueID(uint32_t _destinationID_1, uint32_t _destinationID_2, uint32_t _destinationID_3, uint32_t _destinationID_4); // Function will compare input destination ID with message, and return true if message it is a match.

	ProtocolMSG_t GetRadioMSG_ID();
	uint32_t GetUniqueID1();
	uint32_t GetUniqueID2();
	uint32_t GetUniqueID3();
	uint32_t GetUniqueID4();
	std::string GetUniqueID();
	int8_t GetRSSI();
	int8_t GetSNR();
	static std::string base64_encode(uint8_t bytes_to_encode[], int in_len);
	static std::string base64_decode(std::string const& encoded_string);
				
	// General helper functions and varibels only used by inherited MSG classes.
	protected:
	Telegram(RadioData_t *data); // constructor for reading the header data from RadioData payload.
	Telegram(ProtocolMSG_t _MSG_ID,  uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4);
	Telegram(ProtocolMSG_t _MSG_ID,  std::string Unique_ID);

	// Message To using 128 bit unique serial number.
	ProtocolMSG_t MSG_ID;						// Message ID
	uint32_t Unique_ID_1, Unique_ID_2, Unique_ID_3, Unique_ID_4;
	RadioData_t TelegramData;
	
	// Parameters only used on Telegram mother class.
	private:
	const static std::string base64_chars;
};




#endif /* TELEGRAM_H_ */