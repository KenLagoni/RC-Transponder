/*
	RFProtocol.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 



#ifndef RFPROTOCOL_H_
#define RFPROTOCOL_H_

 // Radio
 #include "E28-2G4M20S.h"

 // Radio protocol
 #include "Telegram.h"
 #include "Telegram_MSG_1.h"
 #include "Telegram_MSG_2.h"
 #include "Telegram_MSG_3.h"

 // GPS
 #include "GPSL80Lite.h"

 //FIFO
 #include "RingBuf.h"

 // SystemInformation
 #include "main.h"

#define FIFO_SIZE 30
#define TIMEOUT_MS 100

class RFProtocol
{
	// Public functions to be used on all Messages
	public:
	RFProtocol(E28_2G4M20S *RadioModule, GpsDataLite *GPS, SystemInformation_t *status); //constructor.
	
	void AddData(RadioData_t *msg); // Add to TX FIFO.
	RadioData_t * GetData();

	int Available(); // returns number of Telegrams in RX FIFO.
	void SendBeacon(); // Add a beacon message to TX FIFO.
	void PowerDown();
	void WakeUp();
//	void IRQHandler(); // Function to be called when Radio pin makes IRQ.
	void RFService();
	void SeccondCounter();

	// General helper functions and varibels only used by inherited MSG classes.
	protected:
	
	// Parameters only used on Telegram mother class.
	private:

	struct RFProtocolStatus_t
	{
		uint8_t NumberOfBeaconsToRelay = 0;
		uint8_t SecondCounterSinceLasteGroundStationContact = 0;
		bool Sleep = false;
	};

	RFProtocolStatus_t RFProtocolStatus;

	enum RFProtocolStates_t {
		RX_IDLE = 0,
		WAITING_FOR_REPLY,
		TX_WITHOUT_REPLY,
		TX_WITH_REPLY,
	};
	
	RFProtocolStates_t RFstate = RX_IDLE;
	unsigned long timeoutStart = 0;

	E28_2G4M20S *Radio = NULL;
	GpsDataLite *GPSData = NULL;
	SystemInformation_t *SystemInformation = NULL;
	
	RingBuf<Telegram*, FIFO_SIZE> rxFIFO;
	RingBuf<Telegram*, FIFO_SIZE> txFIFO;
	Telegram_MSG_1* SavedBeacons[FIFO_SIZE];
//	Telegram_MSG_1 SavedBeacons[FIFO_SIZE];
	Telegram_MSG_2 * GetSavedTransponderBeaconForRelay();

	RFProtocolStates_t RXHandler();
	RFProtocolStates_t TXHandler();

	Telegram * ConvertToTelegram(RadioData_t *data); // return pointer to incomming package if CRC is ok, else NULL.
	bool SaveTransponderBeacon(Telegram_MSG_1 *msg); // Return true if message is saved

	RadioData_t rxbuffer; // When using "GetData()" the Telegram is removed from FIFO and only RadioData is saved. Telegram is den deleted.
	
//	void ServiceStateMachine();
};


#endif /* RFPROTOCOL_H_ */