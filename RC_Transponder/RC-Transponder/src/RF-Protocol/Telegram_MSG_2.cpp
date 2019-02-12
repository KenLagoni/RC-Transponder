/*
	Telegram_MSG_2.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 

#include "Telegram_MSG_2.h"

Telegram_MSG_2::Telegram_MSG_2(uint16_t _to, uint16_t _from, uint32_t _utc, int32_t _lat, int32_t _long, uint8_t _numberOfSats, uint8_t _fix, float _altitude)
{
	this->To = _to; 
	this->From = _from;
	this->MSG_ID = MSG_Transponder_Data;

	this->UTCTime = _utc;
	this->Latitude = _lat;
	this->Longitude = _long;
	this->NumberOfSats = _numberOfSats;
	this->Fix = _fix;
	this->Altitude = _altitude;
	
	this->GeneratePayload();
}

Telegram_MSG_2::Telegram_MSG_2(uint8_t *data, uint8_t size) : Telegram(data,size)
{
	// Data is copied in Telegram constructor to Payload.
	this->ReadPayload();
}


void Telegram_MSG_2::ReadPayload( void )
{
	if(PayloadLength >= HEADER_SIZE+15){
		Serial.println("Reading payload");
		UTCTime       = (uint32_t)((Payload[HEADER_SIZE+0] << 16) + (Payload[HEADER_SIZE+1] << 8) + Payload[HEADER_SIZE+2]);
		Latitude      = (uint32_t)((Payload[HEADER_SIZE+3] << 24) + (Payload[HEADER_SIZE+4] << 16) + (Payload[HEADER_SIZE+5] << 8) + Payload[HEADER_SIZE+6]);
		Longitude     = (uint32_t)((Payload[HEADER_SIZE+7] << 24) + (Payload[HEADER_SIZE+8] << 16) + (Payload[HEADER_SIZE+9] << 8) + Payload[HEADER_SIZE+10]);
		NumberOfSats  = (uint8_t)(Payload[HEADER_SIZE+11] & 0xFF);
		Fix			  = (uint8_t)(Payload[HEADER_SIZE+12] & 0xFF);
		int16_t dummy = (int16_t)((Payload[HEADER_SIZE+13] << 8) + Payload[HEADER_SIZE+14]);
		Altitude      = ((float)dummy)/10;
	}
}


void Telegram_MSG_2::GeneratePayload( void )
{
	//Generate the first 5 bytes.
	this->GenerateHeader();
	
	// Payload:
	// UTC Time
	Payload[HEADER_SIZE+0] = (uint8_t)((UTCTime >> 16) & 0xFF);
	Payload[HEADER_SIZE+1] = (uint8_t)((UTCTime >> 8) & 0xFF);
	Payload[HEADER_SIZE+2] = (uint8_t)(UTCTime & 0xFF);
	
	// Latitude transformed to 32bits.
	Payload[HEADER_SIZE+3] = (uint8_t)((Latitude >> 24) & 0xFF);
	Payload[HEADER_SIZE+4] = (uint8_t)((Latitude >> 16) & 0xFF);
	Payload[HEADER_SIZE+5] = (uint8_t)((Latitude >>  8) & 0xFF);
	Payload[HEADER_SIZE+6] = (uint8_t)(Latitude & 0xFF);

	// Longitude transformed to 32bits.
	Payload[HEADER_SIZE+7]  = (uint8_t)((Longitude >> 24) & 0xFF);
	Payload[HEADER_SIZE+8]  = (uint8_t)((Longitude >> 16) & 0xFF);
	Payload[HEADER_SIZE+9]  = (uint8_t)((Longitude >>  8) & 0xFF);
	Payload[HEADER_SIZE+10] = (uint8_t)(Longitude & 0xFF);
	
	// Number of satellites
	Payload[HEADER_SIZE+11] = (uint8_t)(NumberOfSats & 0xFF);
	
	// Fix
	Payload[HEADER_SIZE+12] = (uint8_t)(Fix & 0xFF);
	
	// Altitude
	int16_t dummy = (int16_t)(Altitude*10);
	Payload[HEADER_SIZE+13] = (uint8_t)((dummy >> 8) & 0xFF);
	Payload[HEADER_SIZE+14] = (uint8_t)(dummy & 0xFF);

	PayloadLength = HEADER_SIZE+15;
}

void Telegram_MSG_2::SerialPrintMessage( void )
{
	Serial.println("");
	Serial.println("Message Header:");
	Serial.println("To           :" + String(this->To));
	Serial.println("From         :" + String(this->From));
	Serial.println("MSG ID       :" + String(this->MSG_ID));
	Serial.println("Message Data:");
	Serial.println("UTC          :" + String(this->UTCTime));
	Serial.println("Lat          :" + String(this->Latitude));
	Serial.println("Long         :" + String(this->Longitude));
	Serial.println("Number of Sat:" + String(this->NumberOfSats));
	Serial.println("Fix          :" + String(this->Fix));
	Serial.println("Altitude     :" + String(this->Altitude));
	Serial.println("--------------");
}