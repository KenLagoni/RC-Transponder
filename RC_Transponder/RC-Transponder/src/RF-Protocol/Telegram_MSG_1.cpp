/*
	Telegram_MSG_1.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
  #include "Telegram_MSG_1.h"

Telegram_MSG_1::Telegram_MSG_1(uint16_t _to, uint16_t _from, ProtocolMSG_t _msgRequest)
{
	this->To = _to; 
	this->From = _from;
	this->MSG_ID = MSG_Request;

	this->MSG_req = _msgRequest;
	this->GeneratePayload();
}


Telegram_MSG_1::Telegram_MSG_1(uint8_t *data, uint8_t size) : Telegram(data,size)
{
	// Data is copied in Telegram constructor to Payload.
	this->ReadPayload();
}


void Telegram_MSG_1::ReadPayload( void )
{
	if(PayloadLength >= HEADER_SIZE){
		this->MSG_req = (ProtocolMSG_t)Payload[HEADER_SIZE+0]; 
	}
}


void Telegram_MSG_1::GeneratePayload( void )
{
	//Generate the first 5 bytes.
	this->GenerateHeader();
	
	// Payload:
	Payload[HEADER_SIZE+0] = (uint8_t)(this->MSG_req & 0xFF);
	PayloadLength = HEADER_SIZE+1;
}

ProtocolMSG_t Telegram_MSG_1::GetMsgRequest(void){
	return this->MSG_req;
}


void Telegram_MSG_1::SerialPrintMessage( void )
{
	Serial.println("");
	Serial.println("Message Header:");
	Serial.println("To            :" + String(this->To));
	Serial.println("From          :" + String(this->From));
	Serial.println("MSG ID        :" + String(this->MSG_ID));
	Serial.println("Message Data:");
	Serial.println("Requesting MSG:" + String(this->MSG_req));
	Serial.println("--------------");
}