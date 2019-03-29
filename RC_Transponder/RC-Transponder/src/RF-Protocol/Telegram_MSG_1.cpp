/*
	Telegram_MSG_1.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
  #include "Telegram_MSG_1.h"

// constructor for main init.
Telegram_MSG_1::Telegram_MSG_1() 
{
	SetMSGLength();

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
	_CRCValid=true;
}


Telegram_MSG_1::Telegram_MSG_1(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4, uint32_t _UTCTime,  uint32_t _Lattitude, uint32_t _Longitude, uint8_t _NumberOfSat, uint8_t _Fix, bool _RunningOnBattery, float _Pressure, float _GroundSpeed, uint8_t _SecondsSinceLastGSContact, float _BatteryVoltage, float _FirmwareVersion, uint8_t _PCBVersion, uint8_t _NumberOfBeaconsToRelay)
{
	SetMSGLength();
		
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
	
	_CRCValid=true;

	// test:
	Latitude =  55562000;
	Longitude = 12281500;
}

/*
Telegram_MSG_1::Telegram_MSG_1(uint8_t *data, uint8_t size) : Telegram(data,size)
{
	SetMSGLength();

	_CRCValid=false;
	// Data is copied in Telegram constructor to Payload.
	if(size >= HEADER_SIZE){
		this->ReadPayload(); // Assumes data is from radio.
		// for test!
	 //   this->FixedPayload(); // test
	}
	// if input is from serial:
}
*/

Telegram_MSG_1::Telegram_MSG_1(RadioData_t *radioData)
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


Telegram_MSG_1::Telegram_MSG_1(SerialData_t *serialData)
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


void Telegram_MSG_1::SetMSGLength(void){
	this->RadioMSGLength =HEADER_SIZE+MSG_1_PAYLOAD_SIZE+CRC_SIZE; // 17+22+2 = 41
	this->SerialMSGLength=SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_1_PAYLOAD_SIZE+RSSI_SIZE+CRC_SIZE; // 2+17+22+2+2 = 45
}

void Telegram_MSG_1::FixedPayload( void ) // debug!
{
	MSG_ID = MSG_Beacon_Broadcast;
	/*
	Unique_ID_1 = 123456;
	Unique_ID_2 = 123456;
	Unique_ID_3 = 123456;
	Unique_ID_4 = 123456;
	*/
	UTCTime =   (uint32_t)((Payload[17] << 16) + (Payload[18] << 8) + Payload[19]);
	Latitude =  55562000;
	Longitude = 12281500;

	NumberOfSat = 12;
	Fix			= 2;
	RunningOnBattery = false;
	
	Pressure = 	0;
	GroundSpeed = 123;
	GroundSpeed = GroundSpeed / 10;
	
	SecondsSinceLastGSContact = 250;
	BatteryVoltage = 4.2;
	
	FirmwareVersion = 1.90;
	
	PCBVersion      = 11;
	NumberOfBeaconsToRelay = 0;
	
	// read the last two bytes but don't count up the counter.
	rssi=10;
	snr=100; // will not count up the RadioMSGLength counter
	_CRCValid=true;
}


void Telegram_MSG_1::ReadPayload( void )
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
	NumberOfBeaconsToRelay = (uint8_t)Payload[HEADER_SIZE+21];	
	
	uint16_t temp_crc = (uint16_t)((Payload[HEADER_SIZE+22] << 8) + Payload[HEADER_SIZE+23]);
	
	if(CalculateCRC(&Payload[0],RadioMSGLength-CRC_SIZE) == temp_crc){
		_CRCValid=true;
	}
}

void Telegram_MSG_1::GeneratePayload(uint8_t *data){
	uint16_t temp_16= 0;
	uint8_t temp_8= 0;
		
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
	
	//Number of Beacons to Relay
	*data = (uint8_t)NumberOfBeaconsToRelay;
}

uint8_t * Telegram_MSG_1::GetRadioMSG(void){

	//Generate the first 17 bytes.
	this->GenerateHeader(&Payload[0]); // no offset

	// Payload without CRC:
	this->GeneratePayload(&Payload[HEADER_SIZE]);
	
	uint16_t temp_crc = CalculateCRC(&Payload[0], HEADER_SIZE+MSG_1_PAYLOAD_SIZE); // no offset
	
	Payload[HEADER_SIZE+MSG_1_PAYLOAD_SIZE+0] = (uint8_t)((temp_crc >>  8) & 0xFF);
	Payload[HEADER_SIZE+MSG_1_PAYLOAD_SIZE+1] = (uint8_t)(temp_crc & 0xFF);
			
	return &Payload[0];
}

//uint8_t testData[] = {1,0,1,226,64,0,1,226,64,0,1,226,64,0,1,226,64,1,138,248,3,79,207,16,0,187,102,156,76,0,0,0,123,250,219,1,90,11,0,10,100};
	
uint8_t * Telegram_MSG_1::GetSerialMSG(void){
	
	// Serial header:
	Payload[0]=0x1E; // Start byte
	Payload[1]=HEADER_SIZE+MSG_1_PAYLOAD_SIZE+RSSI_SIZE+CRC_SIZE; // 43

	//Generate the first 17 bytes.
	this->GenerateHeader(&Payload[SERIAL_HEADER_SIZE]); // 2 offset

	// Payload without CRC:
	this->GeneratePayload(&Payload[SERIAL_HEADER_SIZE+HEADER_SIZE]); // 22 bytes
		
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_1_PAYLOAD_SIZE+0] = rssi;
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_1_PAYLOAD_SIZE+1] = snr;
		
	uint16_t temp_crc = CalculateCRC(&Payload[SERIAL_HEADER_SIZE], HEADER_SIZE+MSG_1_PAYLOAD_SIZE+RSSI_SIZE);
	
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_1_PAYLOAD_SIZE+RSSI_SIZE+0] = (uint8_t)((temp_crc >>  8) & 0xFF);
	Payload[SERIAL_HEADER_SIZE+HEADER_SIZE+MSG_1_PAYLOAD_SIZE+RSSI_SIZE+1] = (uint8_t)(temp_crc & 0xFF);
	
	return &Payload[0];
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
