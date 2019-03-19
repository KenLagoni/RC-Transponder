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
	GET_READY_TO_RUN_ON_BATTERY,
	RUNNING_ON_BATTERY,
	POWER_OFF
};
enum TRANSPONDER_STATES state;

// Hardware and Software version defines
const float FIRMWARE_VERSION = 0.95;

#define MAX_NUMBER_OF_BEACONS_TO_SAVE 10


#endif /* MAIN_H_ */