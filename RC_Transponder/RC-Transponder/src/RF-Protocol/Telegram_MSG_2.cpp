/*
	Telegram_MSG_2.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
  #include "Telegram_MSG_2.h"
  #include "Telegram_MSG_1.h"

Telegram_MSG_2::Telegram_MSG_2(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4, uint32_t _UTCTime,  uint32_t _Lattitude, uint32_t _Longitude, uint8_t _NumberOfSat, uint8_t _Fix, bool _RunningOnBattery, float _Pressure, float _GroundSpeed, uint8_t _SecondsSinceLastGSContact, float _BatteryVoltage, float _FirmwareVersion, uint8_t _PCBVersion, int8_t _RSSI, int8_t _SNR)
{
	SetMSGLength();
	// header:
	this->MSG_ID = MSG_Beacon_Relay;
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
	this->RSSI_relay = _RSSI;
	this->SNR_relay = _SNR;
}

/*
Telegram_MSG_2::Telegram_MSG_2(uint8_t *data, uint8_t size) : Telegram(data,size)
{
	SetMSGLength();
	// Data is copied in Telegram constructor to Payload.
	if(size >= HEADER_SIZE){ 
		this->ReadPayload();	
	}
}
*/
Telegram_MSG_2::Telegram_MSG_2(Telegram_MSG_1 *msg) 
{
	SetMSGLength();
	// header:
	this->MSG_ID = MSG_Beacon_Relay;
	this->Unique_ID_1 = msg->GetUniqueID1();
	this->Unique_ID_2 = msg->GetUniqueID2();
	this->Unique_ID_3 = msg->GetUniqueID3();
	this->Unique_ID_4 = msg->GetUniqueID4();
	
	//Payload
	this->UTCTime = msg->GetUTCTime();
	this->Latitude = msg->GetLatitude();
	this->Longitude = msg->GetLongitude();
	this->NumberOfSat = msg->GetNumberOfSat();
	this->Fix = msg->GetFix();
	this->RunningOnBattery = msg->GetRunningOnBattery();
	this->Pressure = msg->GetPressure();
	this->GroundSpeed = msg->GetGroundSpeed();
	this->SecondsSinceLastGSContact = msg->GetSecondsSinceLastGSContact();
	this->BatteryVoltage = msg->GetBatteryVoltage();
	this->FirmwareVersion = msg->GetFirmwareVersion();
	this->PCBVersion = msg->GetPCBVersion();
	this->RSSI_relay = msg->GetRSSI();
	this->SNR_relay = msg->GetSNR();	
}


Telegram_MSG_2::Telegram_MSG_2(RadioData_t *radioData)
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

Telegram_MSG_2::Telegram_MSG_2(SerialData_t *serialData)
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



void Telegram_MSG_2::SetMSGLength(void){
	this->RadioMSGLength =HEADER_SIZE+MSG_2_PAYLOAD_SIZE+CRC_SIZE; // 17+23+2 = 42
	this->SerialMSGLength=SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_2_PAYLOAD_SIZE+RSSI_SIZE+CRC_SIZE; // 2+17+23+2+2 = 46
}

void Telegram_MSG_2::ReadPayload( void )
{	
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
	RSSI_relay		= (uint8_t)Payload[HEADER_SIZE+21];
	SNR_relay		= (uint8_t)Payload[HEADER_SIZE+22];
	
	uint16_t temp_crc = (uint16_t)((Payload[HEADER_SIZE+23] << 8) + Payload[HEADER_SIZE+24]);
	
	if(CalculateCRC(&Payload[0],RadioMSGLength-CRC_SIZE) == temp_crc){
		_CRCValid=true;
	}
	
}



void Telegram_MSG_2::GeneratePayload( uint8_t *data )
{
	uint16_t temp_16= 0;
	uint8_t temp_8= 0;
	// Payload:
	// UTC Time
	*data++ = (uint8_t)((UTCTime >> 16) & 0xFF);
	*data++ = (uint8_t)((UTCTime >> 8) & 0xFF);
	*data++ = (uint8_t)(UTCTime & 0xFF);
	
	// Latitude transformed to 32bits.
	*data++ = (uint8_t)((Latitude >> 24) & 0xFF);
	*data++ = (uint8_t)((Latitude >> 16) & 0xFF);
	*data++ = (uint8_t)((Latitude >>  8) & 0xFF);
	*data++ = (uint8_t)(Latitude & 0xFF);

	// Longitude transformed to 32bits.
	*data++  = (uint8_t)((Longitude >> 24) & 0xFF);
	*data++  = (uint8_t)((Longitude >> 16) & 0xFF);
	*data++  = (uint8_t)((Longitude >>  8) & 0xFF);
	*data++ = (uint8_t)(Longitude & 0xFF);
	
	temp_8 = NumberOfSat & 0b00011111;
	temp_8 |= ((Fix & 0b00000011) << 5);
	if(RunningOnBattery)
	temp_8 |= 0b10000000;
	
	*data++ = temp_8;
	
	// Pressure
	temp_16=(uint16_t)(Pressure);
	*data++  = (uint8_t)((temp_16 >>  8) & 0xFF);
	*data++ = (uint8_t)(temp_16 & 0xFF);
	
	// Ground speed
	temp_16=(uint16_t)(GroundSpeed*10);
	*data++  = (uint8_t)(((temp_16) >>  8) & 0xFF);
	*data++ = (uint8_t)((temp_16) & 0xFF);
	
	// Seconds since last ground contact
	*data++ = SecondsSinceLastGSContact;
	
	// Battery Voltage
	temp_8=(uint8_t)((BatteryVoltage-2.0)*100);
	*data++ = temp_8;
	
	// Firmware Version
	temp_8 = (uint8_t)(FirmwareVersion);
	*data++ = temp_8;
	*data++ = (uint8_t)((FirmwareVersion*100)-temp_8*100);
	
	// PCB Version
	*data++ = PCBVersion;
	
	// RSSI Relay
	*data++ = (uint8_t)RSSI_relay;
	
	// SNR Relay
	*data++ = (uint8_t)SNR_relay;

}

uint8_t * Telegram_MSG_2::GetRadioMSG(void){

	//Generate the first 17 bytes.
	this->GenerateHeader(&Payload[0]); // no offset

	// Payload without CRC:
	this->GeneratePayload(&Payload[HEADER_SIZE]);
	
	uint16_t temp_crc = CalculateCRC(&Payload[0], HEADER_SIZE+MSG_2_PAYLOAD_SIZE); // no offset
	
	Payload[HEADER_SIZE+MSG_2_PAYLOAD_SIZE+0] = (uint8_t)((temp_crc >>  8) & 0xFF);
	Payload[HEADER_SIZE+MSG_2_PAYLOAD_SIZE+1] = (uint8_t)(temp_crc & 0xFF);
	
	return &Payload[0];
}


//uint8_t testData[] = {1,0,1,226,64,0,1,226,64,0,1,226,64,0,1,226,64,1,138,248,3,79,207,16,0,187,102,156,76,0,0,0,123,250,219,1,90,11,0,10,100};

uint8_t * Telegram_MSG_2::GetSerialMSG(void){
	
	// Serial header:
	Payload[0]=0x1E; // Start byte
	Payload[1]=HEADER_SIZE+MSG_2_PAYLOAD_SIZE+RSSI_SIZE+CRC_SIZE; // 44

	//Generate the first 17 bytes.
	this->GenerateHeader(&Payload[SERIAL_HEADER_SIZE]); // 2 offset

	// Payload without CRC:
	this->GeneratePayload(&Payload[SERIAL_HEADER_SIZE+HEADER_SIZE]); // 23 bytes
	
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_2_PAYLOAD_SIZE+0] = rssi;
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_2_PAYLOAD_SIZE+1] = snr;
	
	uint16_t temp_crc = CalculateCRC(&Payload[SERIAL_HEADER_SIZE], HEADER_SIZE+MSG_2_PAYLOAD_SIZE+RSSI_SIZE);
	
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_2_PAYLOAD_SIZE+RSSI_SIZE+0] = (uint8_t)((temp_crc >>  8) & 0xFF);
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_2_PAYLOAD_SIZE+RSSI_SIZE+1] = (uint8_t)(temp_crc & 0xFF);
	
	return &Payload[0];
}


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
	Serial.println("");
}