/*
	GPSL80Lite.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 

#ifndef GPSL80Lite_h
#define GPSL80Lite_h

#include "Arduino.h" // Needed for Strings and Serial

#if defined (__AVR__) || (__avr__)
#include <SoftwareSerial.h>
#elif defined(ARDUINO_SAMD_MKRZERO)  // Arduino MKR Zero
		
#endif


#define MAX_LAT_SIZE 9	
#define MAX_LONG_SIZE 10	

class GPSL80Lite
{
  public:
    GPSL80Lite();

	#if defined (__AVR__) || (__avr__)	
//		void init(SoftwareSerial *ptr1, GpsDataLite *ptr3);
		void init(SoftwareSerial *ptr1, unsigned long baudrate);
	#elif defined(ARDUINO_SAMD_MKRZERO)  // Arduino MKR Zero
//		void init(Uart *ptr1, GpsDataLite *ptr3);
		void init(Uart *ptr1, unsigned long baudrate);
	#endif
	
	void update();
	void loopback();
//	void printGPSData();

  private:
  struct GpsDataLite {	
	float UTCTime;  					    	// UTC time Zulu in seconds 130032.000 -> 130032
	uint8_t UTC_sec;
	uint8_t UTC_min;
	uint8_t UTC_hour;
	char LatitudeDEG[MAX_LAT_SIZE+1];			// 5550.0898 
	char LatitudeDIR; 							// N or S
	char LongitudeDEG[MAX_LONG_SIZE+1];			// 01224.0718 
	char LongitudeDIR; 							// E or W
	int32_t Latitude;							//  5550.0898N -> 55500898 ||  5550.0898S -> -55500898
	int32_t Longitude;							// 01224.0718E -> 12240718 || 01224.0718W -> -12240718
	float LatitudeDecimal;						//
	float LongitudeDecimal;						//
	char Fix;									// '0' = Invalid, '1'=GPS fix, '2'=DGPS
	uint8_t FixDecimal;							// 0 = Invalid, 1=GPS fix, 2=DGPS
	uint8_t NumberOfSatellites;					// Satellites in view.
	float HDOP;				   					// Horizontal Dilution of Precision (HDOP) 
	float Altitude;								// Altitude above sea level.
	char AltitudeUnit;		    				// Altitude unit M=meters

	bool DataIsValid = false;					// Indicates if data is valid or not.	
	/*
	uint8_t CRCErrors;			// Numbers of CRC errors detected. (will stop at 255).
	uint8_t ReadErrors;			// Number of read errors (will stop at 255).
	*/
  };


	enum StateMachine{
		LOOKING_FOR_START,
		READ_HEADER,
		READ_UTC_DATA,
		WAIT_FOR_NEXT_PARAMETER,
		READ_LAT_DATA,
		READ_LAT_DIR_DATA,
		READ_LONG_DATA,
		READ_LONG_DIR_DATA,
		READ_FIX_DATA,
		READ_SAT_DATA,
		READ_HDOP_DATA,
		READ_ALITUDE_DATA,
		READ_ALITUDE_UNIT_DATA,
		WAIT_FOR_CRC,
		READ_CRC_1,
		READ_CRC_2
	};
//	uint8_t getChecksum(String *str); 
	uint8_t charToInt(char input);
	bool inputValid(char input);
	void updateData();
	void resetTempData();


	// Private functions needed for data anlysis:
	bool dataReady = false;                 // Flag used by update();
	char _newChar = 0; 		                // Variable used by update();
	#if defined (__AVR__) || (__avr__)
		SoftwareSerial *SerialGPS = NULL;       // Pointer to the serial port used
	#elif defined(ARDUINO_SAMD_MKRZERO)  // Arduino MKR Zero
		Uart *SerialGPS = NULL;       // Pointer to the serial port used
	#endif

	StateMachine state = LOOKING_FOR_START; // ENUM used by state maschine in update();
	uint8_t dataLength = 0; 				// counter for number of bytes. (checked against MAX_header/data_SIZE)
	uint8_t i = 0;							// counter for loop used in update();


	GpsDataLite dataOut;               // GpsDataLite struct (where validated data will be saved).	
	GpsDataLite tempData;			   // struct where data is saved before it is verified.
	
	uint8_t byteNumber = 0;
	
	uint8_t CRCErrors;			// Numbers of CRC errors detected. (will stop at 255).
	uint8_t ReadErrors;			// Number of read errors (will stop at 255).
	uint8_t CRC = 0;			// Varible for calculated CRC.
	uint8_t CRC_RESULT = 0;		// Varible for calculated CRC result.
	uint8_t decimal = 0;
};

#endif
