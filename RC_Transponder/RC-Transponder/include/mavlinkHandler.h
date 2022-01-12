/*
 * mavlinkHandler.h
 *
 * Created: 12/01/2022 11.11.03
 *  Author: klo
 */ 


#ifndef MAVLINKHANDLER_H_
#define MAVLINKHANDLER_H_

#include "common/mavlink.h"
#include "ardupilotmega/mavlink.h"
#include "Arduino.h" // Needed for Strings and Serial
#include "FrSkySportSensorPassthrough.h"
#include "FrSkySportSensorGps.h"


class MavlinkHandler
{
	// Public functions to be used on all Messages
	public:
	MavlinkHandler();
	void begin(Uart *ptr1, FrSkySportSensorGps *ptr2, FrSkySportSensorPassthrough *ptr3);
	void service(void);
		
	protected:
		
	private:
	Uart *serial = NULL;       // Pointer to the serial port used
	FrSkySportSensorPassthrough *FrskyPASS = NULL;
	FrSkySportSensorGps *FrskyGPS = NULL;
		
	mavlink_status_t msgStatus;
	mavlink_message_t msgInput;

	struct Battery {
		float    mAh;
		float    tot_mAh;
		float    avg_cA;
		float    avg_mV;
		uint32_t prv_millis;
		uint32_t tot_volts;      // sum of all samples
		uint32_t tot_mW;
		uint32_t samples;
		bool ft;
	};

	struct Battery bat1 = {
	0, 0, 0, 0, 0, 0, 0, true};

	int32_t home_latitude, home_longitude;
	bool frameTypeSent=false;
	float cog;
	uint16_t currentWP;
		
	uint32_t Get_Current_Average1(uint16_t cA);
	uint32_t Get_Volt_Average1(uint16_t mV); 
	void Accum_Volts1(uint32_t mVlt); 
	void Accum_mAh1(uint32_t cAs); 
};




#endif /* MAVLINKHANDLER_H_ */