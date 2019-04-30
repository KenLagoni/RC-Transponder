/*
	Telegram_MSG_1.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 



#ifndef TELEGRAM_MSG_1_H_
#define TELEGRAM_MSG_1_H_

#include "Telegram.h"
#include "E28-2G4M20S.h" // for RadioData_t struct.

#define MSG_1_PAYLOAD_SIZE 22

class Telegram_MSG_1 : public Telegram
{
	public:
	virtual void SerialPrintMessage();    // Function for each massage to print out the data to Serial.print.	

	// Constructor to create empty messages. (only used for compile to allocate memory)
	Telegram_MSG_1();
	
	// Constructor to create message from variables. (Here Unique ID is From)
	Telegram_MSG_1(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4, uint32_t _UTCTime,  uint32_t _Lattitude, uint32_t _Longitude, uint8_t _NumberOfSat, uint8_t _Fix, bool _RunningOnBattery,float _Pressure, float _GroundSpeed ,uint8_t _SecondsSinceLastGSContact, float _BatteryVoltage, float _FirmwareVersion, uint8_t _PCBVersion, uint8_t _NumberOfBeaconsToRelay);

	// Constructor to create messages from Radio data struct.
	Telegram_MSG_1(RadioData_t *radioData);

	
	virtual	~Telegram_MSG_1(){};	// destructor.

	uint8_t GetNumberOfSecondsSinceLastGroundStationCom(); // function which returns Number Of Seconds Since Last Ground Station Communication.
	uint32_t GetUTCTime();
	int32_t GetLatitude();
	int32_t GetLongitude();
	uint8_t GetNumberOfSat();
	uint8_t GetFix();
	bool GetRunningOnBattery();
	float GetPressure();
	float GetGroundSpeed();
	uint8_t GetSecondsSinceLastGSContact();
	float GetBatteryVoltage();
	float GetFirmwareVersion();
	uint8_t GetPCBVersion();
	uint8_t GetNumberOfBeaconsToRelay();	

	protected:
	void FixedPayload(); // debug
											
	private:
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
	uint8_t NumberOfBeaconsToRelay;
};


#endif /* TELEGRAM_MSG_1_H_ */