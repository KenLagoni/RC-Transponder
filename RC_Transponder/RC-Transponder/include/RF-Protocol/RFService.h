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

// GPS
#include "GPSL80Lite.h"

// SystemInformation
#include "main.h"

#define SAVED_BEACONS_FIFO_SIZE 30

class RFService : public RFProtocol
{
	// Public functions to be used on all Messages
	public:
	RFService(E28_2G4M20S *RadioModule, GpsDataLite *GPS, SystemInformation_t *status); //constructor.
	
	void SendBeacon(); // Add a beacon message to TX FIFO.
	void PowerDown();
	void WakeUp();
	void SeccondCounter();
	
	// General helper functions and varibels only used by inherited MSG classes.
	protected:
	
	// Parameters only used on Telegram mother class.
	RFProtocol::RFProtocolStates_t RXHandler();
	RFProtocol::RFProtocolStates_t TXHandler();
	uint32_t milliSeconds();
	
	private:
	
	struct RFProtocolStatus_t
	{
		uint8_t NumberOfBeaconsToRelay = 0;
		uint8_t SecondCounterSinceLasteGroundStationContact = 0;
		bool Sleep = false;
	};

	RFProtocolStatus_t RFProtocolStatus;
		
	E28_2G4M20S *Radio = nullptr;
	GpsDataLite *GPSData = nullptr;
	SystemInformation_t *SystemInformation = nullptr;	
	Telegram_MSG_2 SavedBeacons[SAVED_BEACONS_FIFO_SIZE];
	
	bool SaveTransponderBeacon(Telegram_MSG_1 *msg); // Return true if message is saved
};



#endif /* RFSERVICE_H_ */