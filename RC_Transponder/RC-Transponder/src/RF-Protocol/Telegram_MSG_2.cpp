/*
	Telegram_MSG_2.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
#include "Telegram_MSG_2.h"
#include "Telegram_MSG_1.h"

Telegram_MSG_2::Telegram_MSG_2(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4,
							   uint32_t _UTCTime,  uint32_t _Lattitude, uint32_t _Longitude, uint8_t _NumberOfSat, uint8_t _Fix,
							   bool _RunningOnBattery, float _Hdop, float _GroundSpeed, uint8_t _SecondsSinceLastGSContact,
							   float _BatteryVoltage, float _FirmwareVersion, uint8_t _PCBVersion, int8_t _RSSI, int8_t _SNR) 
							   : Telegram(MSG_Beacon_Relay, _Unique_ID_1, _Unique_ID_2, _Unique_ID_3, _Unique_ID_4)
{
	//Payload
	this->UTCTime = _UTCTime;
	this->Latitude = _Lattitude;
	this->Longitude = _Longitude;
	this->NumberOfSat = _NumberOfSat;
	this->Fix = _Fix;
	this->RunningOnBattery = _RunningOnBattery;
	this->Hdop = _Hdop;
	this->GroundSpeed = _GroundSpeed;
	this->SecondsSinceLastGSContact = _SecondsSinceLastGSContact;
	this->BatteryVoltage = _BatteryVoltage;
	this->FirmwareVersion = _FirmwareVersion;
	this->PCBVersion = _PCBVersion;
	this->RSSI_relay=_RSSI;
	this->SNR_relay=_SNR;

	GeneratePayload();
}

Telegram_MSG_2::Telegram_MSG_2(Telegram_MSG_1 *msg) : Telegram(MSG_Beacon_Relay,msg->GetUniqueID1(),msg->GetUniqueID2(),msg->GetUniqueID3(),msg->GetUniqueID4()) 
{
	//Payload
	this->UTCTime = msg->GetUTCTime();
	this->Latitude = msg->GetLatitude();
	this->Longitude = msg->GetLongitude();
	this->NumberOfSat = msg->GetNumberOfSat();
	this->Fix = msg->GetFix();
	this->RunningOnBattery = msg->GetRunningOnBattery();
	this->Hdop = msg->GetHDOP();
	this->GroundSpeed = msg->GetGroundSpeed();
	this->SecondsSinceLastGSContact = msg->GetSecondsSinceLastGSContact();
	this->BatteryVoltage = msg->GetBatteryVoltage();
	this->FirmwareVersion = msg->GetFirmwareVersion();
	this->PCBVersion = msg->GetPCBVersion();
	this->RSSI_relay = msg->GetRSSI();
	this->SNR_relay = msg->GetSNR();	

	GeneratePayload();
}


Telegram_MSG_2::Telegram_MSG_2(RadioData_t *radioData) : Telegram(radioData)
{
	// Data is copied in Telegram constructor to TelegramData.

	if( (TelegramData.payloadLength >= HEADER_SIZE+22+1)){
		UTCTime =   (uint32_t)((TelegramData.payload[HEADER_SIZE+0] << 16) + (TelegramData.payload[HEADER_SIZE+1] << 8) + TelegramData.payload[HEADER_SIZE+2]);
		Latitude =  (uint32_t)((TelegramData.payload[HEADER_SIZE+3] << 24) + (TelegramData.payload[HEADER_SIZE+4] << 16) + (TelegramData.payload[HEADER_SIZE+5] << 8) + TelegramData.payload[HEADER_SIZE+6]);
		Longitude = (uint32_t)((TelegramData.payload[HEADER_SIZE+7] << 24) + (TelegramData.payload[HEADER_SIZE+8] << 16) + (TelegramData.payload[HEADER_SIZE+9] << 8) + TelegramData.payload[HEADER_SIZE+10]);
		
		NumberOfSat = (uint8_t)(TelegramData.payload[HEADER_SIZE+11] & 0b00011111);
		Fix			= (uint8_t)((TelegramData.payload[HEADER_SIZE+11]& 0b01100000) >> 5);
		if((TelegramData.payload[HEADER_SIZE+11] & 0b10000000)){
			RunningOnBattery = true;
		}
		
		Hdop = 	(float)((TelegramData.payload[HEADER_SIZE+12] << 8) + TelegramData.payload[HEADER_SIZE+13]);
		Hdop /= 100;
		GroundSpeed = (float)((TelegramData.payload[HEADER_SIZE+14] << 8) + TelegramData.payload[HEADER_SIZE+15]);
		GroundSpeed = GroundSpeed / 10;
		
		SecondsSinceLastGSContact = (uint8_t)TelegramData.payload[HEADER_SIZE+16];
		BatteryVoltage  = (float)TelegramData.payload[HEADER_SIZE+17];
		BatteryVoltage = (BatteryVoltage/100)+2.0;
		
		FirmwareVersion = (float)((TelegramData.payload[HEADER_SIZE+18] << 8) + TelegramData.payload[HEADER_SIZE+19]);
		FirmwareVersion = FirmwareVersion / 100;
		
		PCBVersion      = (uint8_t)TelegramData.payload[HEADER_SIZE+20];
		RSSI_relay		= (uint8_t)TelegramData.payload[HEADER_SIZE+21]; 
		SNR_relay		= (uint8_t)TelegramData.payload[HEADER_SIZE+22]; 
	}
}

void Telegram_MSG_2::GeneratePayload()
{

	//Set TelegramData (RadioData) Payload
	uint16_t temp_16= 0;
	uint8_t temp_8= 0;

	// UTC Time
	TelegramData.payload[HEADER_SIZE+0] = (uint8_t)((UTCTime >> 16) & 0xFF);
	TelegramData.payload[HEADER_SIZE+1] = (uint8_t)((UTCTime >> 8) & 0xFF);
	TelegramData.payload[HEADER_SIZE+2] = (uint8_t)(UTCTime & 0xFF);

	// Latitude transformed to 32bits.
	TelegramData.payload[HEADER_SIZE+3] = (uint8_t)((Latitude >> 24) & 0xFF);
	TelegramData.payload[HEADER_SIZE+4] = (uint8_t)((Latitude >> 16) & 0xFF);
	TelegramData.payload[HEADER_SIZE+5] = (uint8_t)((Latitude >>  8) & 0xFF);
	TelegramData.payload[HEADER_SIZE+6] = (uint8_t)(Latitude & 0xFF);

	// Longitude transformed to 32bits.
	TelegramData.payload[HEADER_SIZE+7]  = (uint8_t)((Longitude >> 24) & 0xFF);
	TelegramData.payload[HEADER_SIZE+8]  = (uint8_t)((Longitude >> 16) & 0xFF);
	TelegramData.payload[HEADER_SIZE+9]  = (uint8_t)((Longitude >>  8) & 0xFF);
	TelegramData.payload[HEADER_SIZE+10] = (uint8_t)(Longitude & 0xFF);

	temp_8 = NumberOfSat & 0b00011111;
	temp_8 |= ((Fix & 0b00000011) << 5);
	if(RunningOnBattery)
	temp_8 |= 0b10000000;

	TelegramData.payload[HEADER_SIZE+11] = temp_8;

	// Pressure
	temp_16=(uint16_t)(Hdop*100);
	TelegramData.payload[HEADER_SIZE+12]  = (uint8_t)((temp_16 >>  8) & 0xFF);
	TelegramData.payload[HEADER_SIZE+13] = (uint8_t)(temp_16 & 0xFF);

	// Ground speed
	temp_16=(uint16_t)(GroundSpeed*10);
	TelegramData.payload[HEADER_SIZE+14]  = (uint8_t)(((temp_16) >>  8) & 0xFF);
	TelegramData.payload[HEADER_SIZE+15] = (uint8_t)((temp_16) & 0xFF);

	// Seconds since last ground contact
	TelegramData.payload[HEADER_SIZE+16] = SecondsSinceLastGSContact;

	// Battery Voltage
	temp_8=(uint8_t)((BatteryVoltage-2.0)*100);
	TelegramData.payload[HEADER_SIZE+17] = temp_8;

	// Firmware Version
	temp_8 = (uint8_t)(FirmwareVersion);
	TelegramData.payload[HEADER_SIZE+18] = temp_8;
	TelegramData.payload[HEADER_SIZE+19] = (uint8_t)((FirmwareVersion*100)-temp_8*100);

	// PCB Version
	TelegramData.payload[HEADER_SIZE+20] = PCBVersion;

	// RSSI relay
	TelegramData.payload[HEADER_SIZE+21] = RSSI_relay;

	// SNR relay
	TelegramData.payload[HEADER_SIZE+22] = SNR_relay;

	TelegramData.payloadLength = HEADER_SIZE + 22 + 1;
}

//uint8_t testData[] = {1,0,1,226,64,0,1,226,64,0,1,226,64,0,1,226,64,1,138,248,3,79,207,16,0,187,102,156,76,0,0,0,123,250,219,1,90,11,0,10,100};


void Telegram_MSG_2::SerialPrintMessage( void )
{
	/*
		Serial.println("I Have received this:");
		Serial.println("");
		Serial.println("");
		Serial.println("Like this:");
		
		for(int a=0;a<PayloadLength;a++){
			Serial.println("Payload["+String(a)+"]=" + String(Payload[a]));
		}
		*/
		/*
	Serial.println("");
	Serial.println("Message Header:");
	Serial.println("MSG ID                 :" + String(this->MSG_ID));
	Serial.println("From Unique ID         :" + String(this->Unique_ID_1) + String(this->Unique_ID_2) + String(this->Unique_ID_3) + String(this->Unique_ID_4));
	Serial.println("Message Data:");
	Serial.println("UTC  Time              :" + String(this->UTCTime));
	Serial.println("Latitude               :" + String(this->Latitude));
	Serial.println("Longitude              :" + String(this->Longitude));
	Serial.println("# of Satelites         :" + String(this->NumberOfSat));
	Serial.println("Fix                    :" + String(this->Fix));
	if(this->RunningOnBattery)
		Serial.println("Running on Battery!");
	else
		Serial.println("Powered from external source");
	Serial.println("Pressure              :" + String(this->Pressure));		
	Serial.println("Ground Speed          :" + String(this->GroundSpeed));
	Serial.println("SSLGC                 :" + String(this->SecondsSinceLastGSContact));
	Serial.println("Battery Voltage       :" + String(this->BatteryVoltage));
	Serial.println("Firmware Version      :" + String(this->FirmwareVersion));
	Serial.println("PCB Version           :" + String(this->PCBVersion));
	Serial.println("RSSI Relay            :" + String(this->RSSI_relay));
	Serial.println("SNR Relay             :" + String(this->SNR_relay));
	Serial.println("----------------------------------------------------------------");
	Serial.println("");*/
}


uint32_t Telegram_MSG_2::GetUTCTime( void )
{
	return UTCTime;
}
