/*
	main.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 


#ifndef MAIN_H_
#define MAIN_H_

enum TRANSPONDER_STATES {
	STARTING_UP,
	NORMAL,
	WAIT_FOR_RX,
	SEND_TX_PACKAGE,
	RUNNING_ON_BATTERY,
	POWER_OFF
};
enum TRANSPONDER_STATES state;

// Hardware and Software version defines
const float FIRMWARE_VERSION = 0.90;
uint16_t UNIT_ID=1;
const uint8_t PCB_VERSION=10;
const bool isGroundStation=false; // set to false if Mounted in plane

#endif /* MAIN_H_ */