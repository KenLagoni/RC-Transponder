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

//#include "Arduino.h" // Needed for uint8 types mf.

#define FIFO_SIZE 30

class RFProtocol
{
	// Public functions to be used on all Messages
	public:
	RFProtocol(E28_2G4M20S *RadioModule, GpsDataLite *GPS, SystemInformation_t *status); //constructor.
	void AddToTXFIFO(SerialData_t *msg); // Add to TX FIFO.
	Telegram * GetTelegram(); // return next telegram in RX FIFO.
	int TelegramAvailable(); // returns number of Telegrams in RX FIFO.

	void SendBeacon(); // Add a beacon message to TX FIFO.
	
	void PowerDown();
	void WakeUp();

	void IRQHandler(); // Function to be called when Radio pin makes IRQ.
	

	// General helper functions and varibels only used by inherited MSG classes.
	protected:
	
	// Parameters only used on Telegram mother class.
	private:

	struct RFProtocolStatus_t
	{
		uint8_t NumberOfBeaconsToRelay = 0;
		uint8_t SecondCounterSinceLasteGroundStationContact = 0;
		bool Sleep = false;
	}RFProtocolStatus;

	enum RFProtocolStates_t {
		RX_IDLE = 0,
		WAITING_FOR_REPLY,
		TX_WITHOUT_REPLY,
		TX_WITH_REPLY,
	}state = RX_IDLE;

	E28_2G4M20S *Radio = NULL;
	GpsDataLite *GPSData = NULL;
	SystemInformation_t *SystemInformation = NULL;
	
	RingBuf<Telegram*, FIFO_SIZE> rxFIFO;
	RingBuf<Telegram*, FIFO_SIZE> txFIFO;
	Telegram_MSG_1* SavedBeacons[FIFO_SIZE];
	Telegram_MSG_2 * GetSavedTransponderBeaconForRelay();

	RFProtocolStates_t RXHandler();
	RFProtocolStates_t TXHandler();
	Telegram * ConvertIncommingDataToTelegram(); // return pointer to incomming package if CRC is ok, else NULL.
	Telegram * ConvertSerialDataToTelegram(SerialData_t *newdata); // return pointer to incomming package if CRC is ok, else NULL.
	void SendTelegram(Telegram *msg);
	bool SaveTransponderBeacon(Telegram_MSG_1 *msg); // Return true if message is saved
};


#endif /* RFPROTOCOL_H_ */