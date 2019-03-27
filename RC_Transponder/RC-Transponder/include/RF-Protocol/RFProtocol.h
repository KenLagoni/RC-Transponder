/*
	RFProtocol.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 



#ifndef RFPROTOCOL_H_
#define RFPROTOCOL_H_

 #include "RFProtocol.h"

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
	RFProtocol(E28_2G4M20S *RadioModule, GpsDataLite *GPS, SystemInfomation *status); //constructor.
	void TxTelegram(Telegram *msg); // Add Telegram to TX FIFO.
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
	E28_2G4M20S *Radio = NULL;
	GpsDataLite *GPSData = NULL;
	SystemInfomation *SystemStatus = NULL;
	

	RingBuf<Telegram*, FIFO_SIZE> rxFIFO;
	RingBuf<Telegram*, FIFO_SIZE> txFIFO;
	RingBuf<Telegram*, FIFO_SIZE> RelayTelegrams;



};


#endif /* RFPROTOCOL_H_ */