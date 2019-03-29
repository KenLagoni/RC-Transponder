/*
	main.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 


#ifndef MAIN_H_
#define MAIN_H_
#include "hw.h"


enum TRANSPONDER_STATES {
	STARTING_UP,
	NORMAL,
	WAIT_FOR_RX,
	SEND_TX_PACKAGE,
	GET_READY_TO_RUN_ON_BATTERY,
	RUNNING_ON_BATTERY,
	POWER_OFF
};

extern struct SystemInformation_t
{	
	// Hardware and Software version defines
	const float FIRMWARE_VERSION = 0.95;
	const uint8_t pcbVersion = PCB_VERSION;
	
	// Counters
	uint8_t SecondCounter = 0;
	uint8_t BeaconSecondCounter = 0;
	uint8_t SecondCounterSinceLasteGroundStationContact = 0;

	// system overall statemachine
	enum TRANSPONDER_STATES state=STARTING_UP;
	
	uint8_t NumberOfBeaconsToRelay = 0;

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

}SystemInformation;


#define MAX_NUMBER_OF_BEACONS_TO_SAVE 10


#endif /* MAIN_H_ */