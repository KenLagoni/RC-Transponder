/*
	main.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 


#ifndef MAIN_H_
#define MAIN_H_

#include "Transponder_hal.h"

enum TRANSPONDER_STATES {
	STARTING_UP,
	NORMAL,
	GET_READY_TO_RUN_ON_BATTERY,
	RUNNING_ON_BATTERY_GPS_ON,
	RUNNING_ON_BATTERY_GPS_OFF,
	POWER_OFF
};

typedef struct {
	boolean valid;
	boolean isGroundstation;
	uint32_t POWER_DOWN_DELAY;
	uint32_t GPS_ON_TIME;
	uint32_t GPS_OFF_TIME;
	uint8_t BEACON_INTERVAL;
	char callsign[9];
}Configuration_t;

extern struct SystemInformation_t
{	
	// Hardware and Software version defines
	const float FIRMWARE_VERSION = 2.09;
	//const uint8_t pcbVersion = PCB_VERSION;
	const uint8_t pcbVersion = PCB_VERSION;
	
	// Counters
	uint8_t SecondCounter = 0;
	uint8_t BeaconSecondCounter = 0;
	uint32_t SecondsSinceStart = 0;

	// system overall statemachine
	enum TRANSPONDER_STATES state=STARTING_UP;

	float BatteryVoltage = 0;
	float InputVoltage = 0;
	float USBVoltage = 0;

	//Serial number:
	volatile uint32_t *AddrSerialNumber1 = (volatile uint32_t *)0x0080A00C;
	uint32_t SerialNumber1= *AddrSerialNumber1;
	volatile uint32_t *AddrSerialNumber2 = (volatile uint32_t *)0x0080A040;
	uint32_t SerialNumber2= *AddrSerialNumber2;
	volatile uint32_t *AddrSerialNumber3 = (volatile uint32_t *)0x0080A044;
	uint32_t SerialNumber3= *AddrSerialNumber3;
	volatile uint32_t *AddrSerialNumber4 = (volatile uint32_t *)0x0080A048;
	uint32_t SerialNumber4= *AddrSerialNumber4;	

	uint8_t SecondsBatteryLowCounter = 0;
	bool SimulateRunningOnBattery = false;
	uint16_t GPSActiveCounter = 0;
	uint8_t SafteSwitchPushedTimer=0;
	bool SaftySwitchFirstTimePushed =0;
	bool SaftySwitchPushed =0;
	
	// Local GPS Data:
	uint32_t UTCTime=0;
	int32_t Latitude=0;   // 1E7
	int32_t Longitude=0;
	uint8_t  NumberOfSat=0;
	uint8_t  Fix=0;
	float hdop=0;
	float groundspeed=0; // not used yet (not extracted from GPS)
	
	bool gpsValidSentOnlyOnce=false;
	
	Configuration_t savedSettings;
}SystemInformation;



#endif /* MAIN_H_ */