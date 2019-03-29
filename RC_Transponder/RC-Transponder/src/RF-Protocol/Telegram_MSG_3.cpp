/*
	Telegram_MSG_3.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 

#include "Telegram_MSG_3.h"

Telegram_MSG_3::Telegram_MSG_3(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4, ProtocolCMD_t _command)
{
	SetMSGLength();
	
	// header:
	this->MSG_ID = MSG_Command;
	this->Unique_ID_1 = _Unique_ID_1;
	this->Unique_ID_2 = _Unique_ID_2;
	this->Unique_ID_3 = _Unique_ID_3;
	this->Unique_ID_4 = _Unique_ID_4;
	
	//Payload
	this->cmd = _command;
}

/*Telegram_MSG_3::Telegram_MSG_3(uint8_t *data, uint8_t size) : Telegram(data,size)
{
	SetMSGLength();
	
	// Data is copied in Telegram constructor to Payload.
	if(size >= HEADER_SIZE){
		this->ReadPayload();
		
		if(size > 20){
			// only for data from rf, if serial ,then size = 20
			rssi=(uint8_t)Payload[HEADER_SIZE+3];
			snr=(uint8_t)Payload[HEADER_SIZE+4];
		}
		RadioMSGLength=HEADER_SIZE+3;
		//SerialMSGLength=HEADER_SIZE;
	}
}
*/

Telegram_MSG_3::Telegram_MSG_3(RadioData_t *radioData)
{
	SetMSGLength();
	_CRCValid=false;

	memset(&Payload, 0,  MAX_PAYLOAD_LENGTH);
	memcpy(&Payload,radioData->payload,radioData->payloadLength);
	
	// Data is copied in Telegram constructor to Payload.
	if(radioData->payloadLength >= HEADER_SIZE){
		this->ReadPayloadHeader();
		this->ReadPayload(); // Assumes data is from radio.
		this->rssi = radioData->rssi;
		this->snr = radioData->snr;
	}
}
Telegram_MSG_3::Telegram_MSG_3(SerialData_t *serialData)
{
	SetMSGLength();
	_CRCValid=false;

	memset(&Payload, 0,  MAX_PAYLOAD_LENGTH);
	memcpy(&Payload,serialData->payload,serialData->payloadLength);
	delete serialData;
	
	// Data is copied in Telegram constructor to Payload.
	if(serialData->payloadLength >= HEADER_SIZE){
		this->ReadPayloadHeader();
		this->ReadPayload();
	}
}


void Telegram_MSG_3::SetMSGLength(void){
	this->RadioMSGLength =HEADER_SIZE+MSG_3_PAYLOAD_SIZE+CRC_SIZE; // 17+1+2 = 20
	this->SerialMSGLength=SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_3_PAYLOAD_SIZE+RSSI_SIZE+CRC_SIZE; // 2+17+1+2+2 = 24
}

void Telegram_MSG_3::ReadPayload( void )
{
	cmd = (ProtocolCMD_t)(Payload[HEADER_SIZE+0]);

	uint16_t crc = (uint16_t)((Payload[HEADER_SIZE+1] << 8) + Payload[HEADER_SIZE+2]);
	uint16_t crc_temp = CalculateCRC(&Payload[0], RadioMSGLength-2);
	if(crc == crc_temp){
		_CRCValid=true;
	}	
}


void Telegram_MSG_3::GeneratePayload( uint8_t *data )
{
	// Payload:
	*data = (uint8_t)(cmd & 0xFF); // 1 byte
}


uint8_t * Telegram_MSG_3::GetRadioMSG(void){

	//Generate the first 17 bytes.
	this->GenerateHeader(&Payload[0]); // no offset

	// Payload without CRC:
	this->GeneratePayload(&Payload[HEADER_SIZE]);
	
	uint16_t temp_crc = CalculateCRC(&Payload[0], HEADER_SIZE+MSG_3_PAYLOAD_SIZE); // no offset
	
	Payload[HEADER_SIZE+MSG_3_PAYLOAD_SIZE+0] = (uint8_t)((temp_crc >>  8) & 0xFF);
	Payload[HEADER_SIZE+MSG_3_PAYLOAD_SIZE+1] = (uint8_t)(temp_crc & 0xFF);
	
	return &Payload[0];
}


uint8_t * Telegram_MSG_3::GetSerialMSG(void){
	
	// Serial header:
	Payload[0]=0x1E; // Start byte
	Payload[1]=HEADER_SIZE+MSG_3_PAYLOAD_SIZE+RSSI_SIZE+CRC_SIZE; // 17+1+2+2 = 22

	//Generate the first 17 bytes.
	this->GenerateHeader(&Payload[SERIAL_HEADER_SIZE]); // 2 offset

	// Payload without CRC:
	this->GeneratePayload(&Payload[SERIAL_HEADER_SIZE+HEADER_SIZE]); // 1Byte
	
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_3_PAYLOAD_SIZE+0] = rssi;
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_3_PAYLOAD_SIZE+1] = snr;
	
	uint16_t temp_crc = CalculateCRC(&Payload[SERIAL_HEADER_SIZE], HEADER_SIZE+MSG_3_PAYLOAD_SIZE+RSSI_SIZE);
	
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_3_PAYLOAD_SIZE+RSSI_SIZE+0] = (uint8_t)((temp_crc >>  8) & 0xFF);
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_3_PAYLOAD_SIZE+RSSI_SIZE+1] = (uint8_t)(temp_crc & 0xFF);
	
	return &Payload[0];
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