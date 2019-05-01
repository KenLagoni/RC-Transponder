/*
	Telegram_MSG_3.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 

#include "Telegram_MSG_3.h"

Telegram_MSG_3::Telegram_MSG_3(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4, 
							   ProtocolCMD_t _command): Telegram(MSG_Command, _Unique_ID_1, _Unique_ID_2, _Unique_ID_3, _Unique_ID_4)
{
	//Payload
	this->cmd = _command;

	//Set TelegramData (RadioData) Payload
	TelegramData.payload[HEADER_SIZE+0] = (uint8_t)(cmd & 0xFF); 
	
	TelegramData.payloadLength = HEADER_SIZE + 0 + 1;
}


Telegram_MSG_3::Telegram_MSG_3(RadioData_t *radioData) : Telegram(radioData)
{
	// Data is copied in Telegram constructor to TelegramData.

	if( (TelegramData.payloadLength >= HEADER_SIZE+0+1)){
		cmd  = (ProtocolCMD_t)TelegramData.payload[HEADER_SIZE+0];
	}
}

Telegram_MSG_3::Telegram_MSG_3(std::string Unique_ID, ProtocolCMD_t command): Telegram(MSG_Command, Unique_ID)
{
	//Payload
	this->cmd = command;

	//Set TelegramData (RadioData) Payload
	TelegramData.payload[HEADER_SIZE+0] = (uint8_t)(cmd & 0xFF);
	TelegramData.payloadLength = HEADER_SIZE + 0 + 1;
}

ProtocolCMD_t Telegram_MSG_3::GetCommand( void ){
	return cmd;
}

void Telegram_MSG_3::SerialPrintMessage( void )
{
/*
	Serial.println("");
	Serial.println("Message Header:");
	Serial.println("MSG ID                 :" + String(this->MSG_ID));
	Serial.println("From Unique ID         :" + String(this->Unique_ID_1) + String(this->Unique_ID_2) + String(this->Unique_ID_3) + String(this->Unique_ID_4));

	Serial.println("Message Data:");
	Serial.println("Command ID             :" + String(this->cmd));
	Serial.println("----------------------------------------------");
	Serial.println("");*/
}