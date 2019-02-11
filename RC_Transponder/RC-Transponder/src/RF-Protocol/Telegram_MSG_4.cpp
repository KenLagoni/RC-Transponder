/*
 * Telegram_MSG_2.cpp
 *
 * Created: 08-02-2019 12:24:17
 *  Author: klo
 */ 
#include "Telegram_MSG_4.h"

Telegram_MSG_4::Telegram_MSG_4(uint16_t _from, uint32_t _utc, int32_t _lat, int32_t _long, uint8_t _status)
{
	this->To = 0xFFFF; // To All
	this->From = _from;
	this->MSG_ID = MSG_Beacon_Broadcast;

	this->UTCTime = _utc;
	this->Latitude = _lat;
	this->Longitude = _long;
	this->Status = _status;
	this->GeneratePayload();
}

Telegram_MSG_4::Telegram_MSG_4(uint8_t *data, uint8_t size) : Telegram(data,size)
{
	// Data is copied in Telegram constructor to Payload.
	this->ReadPayload();
}


void Telegram_MSG_4::ReadPayload( void )
{
//	Serial.println("Payloadlength must be larger or equal to="+String(HEADER_SIZE+11));
//	Serial.println("Payloadlength="+String(PayloadLength));
	if(PayloadLength >= HEADER_SIZE+11){
//		Serial.println("Reading payload");
		UTCTime =   (uint32_t)((Payload[HEADER_SIZE+0] << 16) + (Payload[HEADER_SIZE+1] << 8) + Payload[HEADER_SIZE+2]);
		Latitude =  (uint32_t)((Payload[HEADER_SIZE+3] << 24) + (Payload[HEADER_SIZE+4] << 16) + (Payload[HEADER_SIZE+5] << 8) + Payload[HEADER_SIZE+6]);
		Longitude = (uint32_t)((Payload[HEADER_SIZE+7] << 24) + (Payload[HEADER_SIZE+8] << 16) + (Payload[HEADER_SIZE+9] << 8) + Payload[HEADER_SIZE+10]);
		Status = (uint8_t)(Payload[HEADER_SIZE+11] & 0xFF);
	}
}


void Telegram_MSG_4::GeneratePayload( void )
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
	
	// Status bits
	Payload[HEADER_SIZE+11] = (uint8_t)(Status & 0xFF);

	PayloadLength = HEADER_SIZE+12;
}


void Telegram_MSG_4::SerialPrintMessage( void )
{
	Serial.println("");
	Serial.println("Message Header:");
	Serial.println("To    :" + String(this->To));
	Serial.println("From  :" + String(this->From));
	Serial.println("MSG ID:" + String(this->MSG_ID));
	Serial.println("Message Data:");					
	Serial.println("UTC   :" + String(this->UTCTime));
	Serial.println("Lat   :" + String(this->Latitude));
	Serial.println("Long  :" + String(this->Longitude));
	Serial.println("Status:" + String(this->Status));
	Serial.println("--------------");					
}