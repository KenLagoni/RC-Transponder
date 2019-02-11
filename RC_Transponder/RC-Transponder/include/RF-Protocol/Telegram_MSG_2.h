/*
 * Telegram_MSG_2.h
 *
 * Created: 08-02-2019 12:23:57
 *  Author: klo
 */ 


#ifndef TELEGRAM_MSG_2_H_
#define TELEGRAM_MSG_2_H_

#include "Arduino.h" // Needed for uint8 types mf.
#include "Telegram.h"

class Telegram_MSG_2 : public Telegram
{
	public:
	virtual void ReadPayload( void );		 // Each messages should be able to parse/decode the payload to specific cases.
	virtual void GeneratePayload( void );    // Each messages should be able to generate a payload based on values.
	virtual void SerialPrintMessage( void ); // Function for each massage to print out the data to Serial.print

	Telegram_MSG_2(uint16_t _to, uint16_t _from, uint32_t _utc, int32_t _lat, int32_t _long, uint8_t _numberOfSats, uint8_t _fix, float _altitude);	// Constructor to create message from variables.
	Telegram_MSG_2(uint8_t *data, uint8_t size);																// Constructor to create messages from payload array.

	private:
	uint32_t UTCTime;				    	// UTC time Zulu in seconds 130032.000 -> 130032
	int32_t Latitude;						//  5550.0898N -> 55500898 ||  5550.0898S -> -55500898
	int32_t Longitude;						// 01224.0718E -> 12240718 || 01224.0718W -> -12240718
	uint8_t NumberOfSats;
	uint8_t Fix;
	float Altitude;
};


#endif /* TELEGRAM_MSG_2_H_ */