/*
	Telegram_MSG_3.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 

#include "Telegram_MSG_3.h"

Telegram_MSG_3::Telegram_MSG_3(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4, ProtocolCMD_t _command)
{
	// header:
	this->MSG_ID = MSG_Command;
	this->Unique_ID_1 = _Unique_ID_1;
	this->Unique_ID_2 = _Unique_ID_2;
	this->Unique_ID_3 = _Unique_ID_3;
	this->Unique_ID_4 = _Unique_ID_4;
	
	//Payload
	this->cmd = _command;
	this->crcValid = true; // always true when we generate the msg ourselves.
			
	this->GeneratePayload();
}

Telegram_MSG_3::Telegram_MSG_3(uint8_t *data, uint8_t size) : Telegram(data,size)
{
	// Data is copied in Telegram constructor to Payload.
	this->ReadPayload();
}


void Telegram_MSG_3::ReadPayload( void )
{
	if(PayloadLength >= HEADER_SIZE){
		cmd = (ProtocolCMD_t)(Payload[HEADER_SIZE+0]);

		CRC = (uint16_t)((Payload[HEADER_SIZE+1] << 8) + Payload[HEADER_SIZE+2]);
	
		//Check CRC:
		if(CalculateCRC(HEADER_SIZE+0) == CRC){
			crcValid=true;
			rssi=(uint8_t)Payload[HEADER_SIZE+3];
			snr=(uint8_t)Payload[HEADER_SIZE+4];
		}else{
			crcValid=false;
		}
	}
}


void Telegram_MSG_3::GeneratePayload( void )
{
	//Generate the first 17 bytes.
	this->GenerateHeader();

	// Payload:
	Payload[HEADER_SIZE+0] = (uint8_t)(cmd & 0xFF);
		
	CRC = CalculateCRC(HEADER_SIZE+0);
	
	Payload[HEADER_SIZE+1] = (uint8_t)((CRC >>  8) & 0xFF);
	Payload[HEADER_SIZE+2] = (uint8_t)(CRC & 0xFF);
	
	PayloadLength = HEADER_SIZE+3; //6
}


ProtocolCMD_t Telegram_MSG_3::GetCommand( void ){
	return cmd;
}

void Telegram_MSG_3::SerialPrintMessage( void )
{
	Serial.println("");
	Serial.println("Message Header:");
	Serial.println("MSG ID                 :" + String(this->MSG_ID));
	Serial.println("From Unique ID         :" + String(this->Unique_ID_1) + String(this->Unique_ID_2) + String(this->Unique_ID_3) + String(this->Unique_ID_4));

	Serial.println("Message Data:");
	Serial.println("Command ID             :" + String(this->cmd));
	Serial.println("----------------------------------------------");
	Serial.println("");
}
