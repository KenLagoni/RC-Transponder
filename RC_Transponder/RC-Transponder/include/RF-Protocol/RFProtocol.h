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

//FIFO
#include "RingBuf.h"

#define FIFO_SIZE 30
#define TIMEOUT_MS 100

class RFProtocol
{
	// Public functions to be used on all Messages
	public:
	RFProtocol(E28_2G4M20S *RadioModule); //constructor.
	
	void AddData(RadioData_t *msg); // Add to TX FIFO.
	RadioData_t * GetData();
	int Available(); // returns number of Telegrams in RX FIFO.
	void Service();

	// General helper functions and varibels only used by inherited MSG classes.
	protected:
	
	enum RFProtocolStates_t {
		RX_IDLE = 0,
		WAITING_FOR_REPLY,
		TX_WITHOUT_REPLY,
		TX_WITH_REPLY,
	};
	
	virtual uint32_t milliSeconds() = 0;
	virtual RFProtocolStates_t RXHandler() = 0;
	virtual RFProtocolStates_t TXHandler() = 0;
	void _WakeUp();
	void _PowerDown();
	
	E28_2G4M20S *Radio = NULL;
	RingBuf<Telegram*, FIFO_SIZE> rxFIFO;
	RingBuf<Telegram*, FIFO_SIZE> txFIFO;
	

	
	RFProtocolStates_t RFstate = RX_IDLE;
	Telegram * ConvertToTelegram(RadioData_t *data); // return pointer to incomming package if CRC is ok, else NULL.
			
	// Parameters only used on Telegram mother class.
	private:
	unsigned long timeoutStart = 0;
	RadioData_t rxbuffer; // When using "GetData()" the Telegram is removed from FIFO and only RadioData is saved. Telegram is den deleted.
	
	//String base64_encode(byte bytes_to_encode[], int in_len);
};


#endif /* RFPROTOCOL_H_ */