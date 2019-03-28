/*
	Telegram_MSG_2.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 



#ifndef TELEGRAM_MSG_2_H_
#define TELEGRAM_MSG_2_H_

#include "Arduino.h" // Needed for uint8 types mf.
#include "Telegram.h"
#include "Telegram_MSG_1.h"

#define MSG_2_PAYLOAD_SIZE 23

class Telegram_MSG_2 : public Telegram
{
	public:
	virtual void SerialPrintMessage( void ); // Function for each massage to print out the data to Serial.print.	
	virtual uint8_t * GetRadioMSG(void);		 // Function which returns the payload length for Radio transmittion.
	virtual uint8_t * GetSerialMSG(void);        // Function which returns the payload length for Serial transmittion.
		
	// Constructor to create message from variables. (Here Unique ID is From)
	Telegram_MSG_2(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4, uint32_t _UTCTime,  uint32_t _Lattitude, uint32_t _Longitude, uint8_t _NumberOfSat, uint8_t _Fix, bool _RunningOnBattery,float _Pressure, float _GroundSpeed ,uint8_t _SecondsSinceLastGSContact, float _BatteryVoltage, float _FirmwareVersion, uint8_t _PCBVersion, int8_t _RSSI_relay, int8_t _SNR_relay);
	
	// Constructor to create messages from payload array.
	Telegram_MSG_2(uint8_t *data, uint8_t size);	
	
	// Constructor to create messages from MSG 1.
	Telegram_MSG_2(Telegram_MSG_1 *msg);

	virtual	~Telegram_MSG_2(){};	// destructor.	

	protected:
	virtual void ReadPayload( void );		 // Each messages should be able to parse/decode the payload to specific cases.
	virtual void GeneratePayload( uint8_t *data);	 // Each messages should be able to generate a payload based on values.
	
	private:
	void SetMSGLength();
	uint32_t UTCTime;				    	// UTC time Zulu in seconds 130032.000 -> 130032
	int32_t Latitude;						//  5550.0898N -> 55500898 ||  5550.0898S -> -55500898
	int32_t Longitude;						// 01224.0718E -> 12240718 || 01224.0718W -> -12240718
	uint8_t NumberOfSat;
	uint8_t Fix;
	bool RunningOnBattery;
	float Pressure;
	float GroundSpeed;
	uint8_t SecondsSinceLastGSContact;
	float BatteryVoltage;
	float FirmwareVersion;
	uint8_t PCBVersion;
	int8_t RSSI_relay;
	int8_t SNR_relay;
};


#endif /* TELEGRAM_MSG_2_H_ */