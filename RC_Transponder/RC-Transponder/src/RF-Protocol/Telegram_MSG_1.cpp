/*
	Telegram_MSG_1.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
  #include "Telegram_MSG_1.h"

// constructor for main init.
Telegram_MSG_1::Telegram_MSG_1()
{
	// header:
	this->MSG_ID = MSG_Beacon_Broadcast;
	this->Unique_ID_1 = 0;
	this->Unique_ID_2 = 0;
	this->Unique_ID_3 = 0;
	this->Unique_ID_4 = 0;
	
	//Payload
	this->UTCTime = 0;
	this->Latitude = 0;
	this->Longitude = 0;
	this->NumberOfSat = 0;
	this->Fix = 0;
	this->RunningOnBattery = 0;
	this->Pressure = 0;
	this->GroundSpeed = 0;
	this->SecondsSinceLastGSContact = 0;
	this->BatteryVoltage = 0;
	this->FirmwareVersion = 0;
	this->PCBVersion = 0;
	this->NumberOfBeaconsToRelay = 0;
	this->crcValid = false; // always flase when init
		
	this->GeneratePayload();
}


Telegram_MSG_1::Telegram_MSG_1(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4, uint32_t _UTCTime,  uint32_t _Lattitude, uint32_t _Longitude, uint8_t _NumberOfSat, uint8_t _Fix, bool _RunningOnBattery, float _Pressure, float _GroundSpeed, uint8_t _SecondsSinceLastGSContact, float _BatteryVoltage, float _FirmwareVersion, uint8_t _PCBVersion, uint8_t _NumberOfBeaconsToRelay)
{
	// header:
	this->MSG_ID = MSG_Beacon_Broadcast;
	this->Unique_ID_1 = _Unique_ID_1;
	this->Unique_ID_2 = _Unique_ID_2;
	this->Unique_ID_3 = _Unique_ID_3;
	this->Unique_ID_4 = _Unique_ID_4;
		
	//Payload
	this->UTCTime = _UTCTime;
	this->Latitude = _Lattitude;
	this->Longitude = _Longitude;			
	this->NumberOfSat = _NumberOfSat;
	this->Fix = _Fix;
	this->RunningOnBattery = _RunningOnBattery;
	this->Pressure = _Pressure;
	this->GroundSpeed = _GroundSpeed;
	this->SecondsSinceLastGSContact = _SecondsSinceLastGSContact;
	this->BatteryVoltage = _BatteryVoltage;
	this->FirmwareVersion = _FirmwareVersion;
	this->PCBVersion = _PCBVersion;
	this->NumberOfBeaconsToRelay = _NumberOfBeaconsToRelay;	
	this->crcValid = true; // always true when we generate the msg ourselves.
	
	
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
		UTCTime =   (uint32_t)((Payload[HEADER_SIZE+0] << 16) + (Payload[HEADER_SIZE+1] << 8) + Payload[HEADER_SIZE+2]);
		Latitude =  (uint32_t)((Payload[HEADER_SIZE+3] << 24) + (Payload[HEADER_SIZE+4] << 16) + (Payload[HEADER_SIZE+5] << 8) + Payload[HEADER_SIZE+6]);
		Longitude = (uint32_t)((Payload[HEADER_SIZE+7] << 24) + (Payload[HEADER_SIZE+8] << 16) + (Payload[HEADER_SIZE+9] << 8) + Payload[HEADER_SIZE+10]);
		
		NumberOfSat = (uint8_t)(Payload[HEADER_SIZE+11] & 0b00011111);
		Fix			= (uint8_t)((Payload[HEADER_SIZE+11]& 0b01100000) >> 5);
		if((Payload[HEADER_SIZE+11] & 0b10000000)){
			RunningOnBattery = true;
		}
		
		Pressure = 	(float)((Payload[HEADER_SIZE+12] << 8) + Payload[HEADER_SIZE+13]);
		GroundSpeed = (float)((Payload[HEADER_SIZE+14] << 8) + Payload[HEADER_SIZE+15]);
		GroundSpeed = GroundSpeed / 10;
				
		SecondsSinceLastGSContact = (uint8_t)Payload[HEADER_SIZE+16];		
		BatteryVoltage  = (float)Payload[HEADER_SIZE+17];	
		BatteryVoltage = (BatteryVoltage/100)+2.0;
		
		FirmwareVersion = (float)((Payload[HEADER_SIZE+18] << 8) + Payload[HEADER_SIZE+19]);
		FirmwareVersion = FirmwareVersion / 100;
		
		PCBVersion      = (uint8_t)Payload[HEADER_SIZE+20];	
		NumberOfBeaconsToRelay = (uint8_t)Payload[HEADER_SIZE+21];	
		
		CRC = (uint16_t)((Payload[HEADER_SIZE+22] << 8) + Payload[HEADER_SIZE+23]);
		
		//Check CRC:
		if(CalculateCRC(HEADER_SIZE+21) == CRC){
			crcValid=true;
			rssi=(uint8_t)Payload[HEADER_SIZE+24];
			snr=(uint8_t)Payload[HEADER_SIZE+25];
		}else{
			crcValid=false;
		}		
	}
}


void Telegram_MSG_1::GeneratePayload( void )
{
	uint16_t temp_16= 0;
	uint8_t temp_8= 0;
	//Generate the first 17 bytes.
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
	
	temp_8 = NumberOfSat & 0b00011111;
	temp_8 |= ((Fix & 0b00000011) << 5);
	if(RunningOnBattery)
		temp_8 |= 0b10000000;
	
	Payload[HEADER_SIZE+11] = temp_8;
	
	// Pressure
	temp_16=(uint16_t)(Pressure);
	Payload[HEADER_SIZE+12]  = (uint8_t)((temp_16 >>  8) & 0xFF);
	Payload[HEADER_SIZE+13] = (uint8_t)(temp_16 & 0xFF);
	
	// Ground speed
	temp_16=(uint16_t)(GroundSpeed*10);
	Payload[HEADER_SIZE+14]  = (uint8_t)(((temp_16) >>  8) & 0xFF);
	Payload[HEADER_SIZE+15] = (uint8_t)((temp_16) & 0xFF);
	
	// Seconds since last ground contact
	Payload[HEADER_SIZE+16] = SecondsSinceLastGSContact;
	
	// Battery Voltage 	
	temp_8=(uint8_t)((BatteryVoltage-2.0)*100);
	Payload[HEADER_SIZE+17] = temp_8;
	
	// Firmware Version
	temp_8 = (uint8_t)(FirmwareVersion);
	Payload[HEADER_SIZE+18] = temp_8;  
	Payload[HEADER_SIZE+19] = (uint8_t)((FirmwareVersion*100)-temp_8*100); 
	
	// PCB Version
	Payload[HEADER_SIZE+20] = PCBVersion;  
	
	//Number of Beacons to Relay
	Payload[HEADER_SIZE+21] = (uint8_t)NumberOfBeaconsToRelay;
		
	CRC = CalculateCRC(HEADER_SIZE+21);
	
	Payload[HEADER_SIZE+22] = (uint8_t)((CRC >>  8) & 0xFF);
	Payload[HEADER_SIZE+23] = (uint8_t)(CRC & 0xFF);
			
	PayloadLength = HEADER_SIZE+24; //17+24=41

}


void Telegram_MSG_1::SerialPrintMessage( void )
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
	Serial.println("");
	Serial.println("Message Header:");
	Serial.println("MSG ID                 :" + String(this->MSG_ID));
	Serial.println("From Unique ID         :" + String(this->Unique_ID_1) + String(this->Unique_ID_2) + String(this->Unique_ID_3) + String(this->Unique_ID_4));
	Serial.println("Message Data:");
	Serial.println("UTC  Time              :" + String(this->UTCTime));
	Serial.println("Latitude               :" + String(this->Latitude));
	Serial.println("Longitude              :" + String(this->Longitude));
	Serial.println("# of satellites        :" + String(this->NumberOfSat));
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
	Serial.println("# of Beacons to relay :" + String(this->NumberOfBeaconsToRelay));
	Serial.println("----------------------------------------------------------------");
	Serial.println("");
}


uint8_t Telegram_MSG_1::GetNumberOfSecondsSinceLastGroundStationCom( void )
{	
	return SecondsSinceLastGSContact;
}

uint32_t Telegram_MSG_1::GetUTCTime( void )
{
	return UTCTime;
}

int32_t Telegram_MSG_1::GetLatitude( void )
{
	return Latitude;
}

int32_t Telegram_MSG_1::GetLongitude( void )
{
	return Longitude;
}

uint8_t Telegram_MSG_1::GetNumberOfSat( void )
{
	return NumberOfSat;
}

uint8_t Telegram_MSG_1::GetFix( void )
{
	return Fix;
}

bool Telegram_MSG_1::GetRunningOnBattery( void )
{
	return RunningOnBattery;
}

float Telegram_MSG_1::GetPressure( void )
{
	return Pressure;
}

float Telegram_MSG_1::GetGroundSpeed( void )
{
	return GroundSpeed;
}

uint8_t Telegram_MSG_1::GetSecondsSinceLastGSContact( void )
{
	return SecondsSinceLastGSContact;
}

float Telegram_MSG_1::GetBatteryVoltage( void )
{
	return BatteryVoltage;
}

float Telegram_MSG_1::GetFirmwareVersion( void )
{
	return FirmwareVersion;
}


uint8_t Telegram_MSG_1::GetPCBVersion( void )
{
	return PCBVersion;
}


uint8_t Telegram_MSG_1::GetNumberOfBeaconsToRelay( void )
{
	return NumberOfBeaconsToRelay;
}


uint8_t Telegram_MSG_1::GetRSSI( void )
{
	return rssi;
}


uint8_t Telegram_MSG_1::GetSNR( void )
{
	return snr;
}
