/*
	E28-2G4M20S.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
#ifndef __E28_2G4M20S_H__
#define __E28_2G4M20S_H__

#include "Arduino.h" // Needed for Strings and Serial
#include "radio.h"
#include "sx1280-hal.h"

//#define MODE_BLE
#define MODE_LORA
//#define MODE_GENERIC
//#define MODE_FLRC

// Defines the buffer size, i.e. the payload size
#define MAX_PAYLOAD_LENGTH                          45
//#define FIFO_SIZE									10

#define TX_TIMEOUT_VALUE                            50 // ms


// Number of tick size steps for rx timeout
#define RX_TIMEOUT_VALUE                            50 // ms


// Size of ticks (used for Tx and Rx timeout)
#define RX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_1000_US


struct RadioStatus
{	
	bool txDone;
	bool rxDone;
	bool txTimeout;
	bool rxTimeout;	
};

struct RadioData
{
	uint8_t payload[MAX_PAYLOAD_LENGTH];
	uint8_t payloadLength;
	uint8_t rssi;
	uint8_t snr;	
};

class E28_2G4M20S
{
  public:
    E28_2G4M20S(int chipSelectPin, int resetPin, int busyPin, int dio1Pin, int dio2Pin, int dio3Pin, int txEnablePin, int rxEnablePin);
	void Init(); 
	void IRQHandler();
	void SetRXMode(bool useTimeout); 
	void Debug();
	void Sleep();
	bool IsIdle(void);
	void WakeUp(void);
	void test(void);
	
	// New functions:
	bool NewPackageReady(void);
	void SendPackage(uint8_t *payload, uint8_t payloadLength);
//	uint8_t GetPackage(uint8_t *payload, uint8_t maxSize); // returns the payload length
	uint8_t * GetPayload(uint8_t &len);
	RadioData * GetRadioData();
	void SetBufferReady(bool _set);
	RadioStatus GetRadioStatus();
		
  private:
    // Pins:
	int _chipSelectPin;
	int _resetPin;
	int _busyPin;
	int _dio1Pin;
	int _dio2Pin;
	int _dio3Pin;
	int _txEnablePin;
	int _rxEnablePin;
	
	// Callback functions
	void OnTxDone( void ); // Function to be executed on Radio Tx Done event
	void OnRxDone( void ); // Function to be executed on Radio Rx Done event
	void OnTxTimeout( void ); //Function executed on Radio Tx Timeout event
	void OnRxTimeout( void ); //Function executed on Radio Rx Timeout event
	void OnRxError( IrqErrorCode_t ); // Function executed on Radio Rx Error event
		    
    void SetTxModeActive( void );
	void SetRxModeActive( void );
    
    uint32_t rf_frequency = 2400000000;
    uint8_t tx_power = 13;
  
    PacketParams_t PacketParams;
    PacketStatus_t PacketStatus;
    ModulationParams_t modulationParams;
  	
    SX1280Hal *Radio = NULL;
	
	RadioStatus RadioStatusData;
	
	////////////////////
	
	// Define buffers:
//	uint8_t InputFIFO[FIFO_SIZE][MAX_PAYLOAD_LENGTH];
//	uint8_t OutputFIFO[FIFO_SIZE][MAX_PAYLOAD_LENGTH];
//	uint8_t InputFifoIndex = 0;
//	uint8_t OutputFifoIndex = 0;
		
	// Define the possible message type for this application
	uint8_t BufferSize = MAX_PAYLOAD_LENGTH; //The size of the buffer
	uint8_t Buffer[MAX_PAYLOAD_LENGTH]; //The buffer
	bool BufferReady = false;

	int8_t RssiValue = 0;
	int8_t SnrValue = 0;
	uint16_t RxIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT; // Mask of IRQs to listen to in rx mode
	uint16_t TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT; // Mask of IRQs to listen to in tx mode	
	
	bool radioIdle = true;
};

#endif // __E28_2G4M20S_H__