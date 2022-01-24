/*
	PCProtocol.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 


#ifndef PCPROTOCOL_H_
#define PCPROTOCOL_H_

#include <Arduino.h>
#include <string>  // std::string not Arduino String
#include "main.h"
#include "FlashStorage.h"

class PCProtocol
{
	// Public functions to be used on all Messages
	public:
	PCProtocol();
	void begin(SystemInformation_t *ptr1);
	bool service(); // return true if data needs to be saved to NVM
	
	// General helper functions and variables only used by inherited MSG classes.
	protected:
	 


	// Parameters only used on Telegram mother class.
	private:
	
	bool handlecommand(void);

/*
	enum ApplicationCMD_t{
		PRINT=0x00,
		SET_CALLSIGN,
		SET_BEACON_INTERVAL,
		UNKNOWN
	}ApplicationCMD;
	*/
	SystemInformation_t *systemData=NULL;
	int dataReady=0;
	char _newChar=0;
	uint32_t startTime = 0;
	uint32_t now = 0;
	bool timeout = false;


	std::size_t firstCMDSize;
	std::size_t secondCMDSize;
	std::string inputCMD;
	std::string command;
	std::string parameter;
};


#endif /* PCPROTOCOL_H_ */