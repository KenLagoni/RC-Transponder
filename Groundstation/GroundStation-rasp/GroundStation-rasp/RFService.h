/*
 * RFService.h
 *
 * Created: 12-Apr-19 22:56:24
 *  Author: Kenneth
 */ 
#ifndef RFSERVICE_H_
#define RFSERVICE_H_

#include "RFProtocol.h"
#include "E28-2G4M20S.h"
#include "Telegram.h"
#include "Telegram_MSG_1.h"
#include "Telegram_MSG_2.h"
#include "Telegram_MSG_3.h"
#include "plane.h"
#include <list>
#include <stdio.h>
#include <string>

class RFService : public RFProtocol
{
	// Public functions to be used on all Messages
	public:
	RFService(E28_2G4M20S *RadioModule, std::string logpath); //constructor.
	void ServiceHandel();
	void PingActivePlanes();
				
	// General helper functions and varibels only used by inherited MSG classes.
	protected:
	
	// Parameters only used on Telegram mother class.
	RFProtocol::RFProtocolStates_t RXHandler();
	RFProtocol::RFProtocolStates_t TXHandler();
	uint32_t milliSeconds();
	
	private:		
	std::list<Plane>::iterator it;
	std::list<Plane> PlaneList;
	std::string base64_encode(uint8_t bytes_to_encode[], int in_len);
	std::string planeListPath = "";
	std::string planeListFile = "Transponders.csv";
	std::string planeListLogFile = ""; 
	uint8_t unknownPlanes = 1;
};



#endif /* RFSERVICE_H_ */