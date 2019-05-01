/*
 * plane.h
 *
 * Created: 14-Apr-19 19:33:32
 *  Author: Kenneth
 */ 


#ifndef PLANE_H_
#define PLANE_H_

#include <iostream> // cout debug
#include <string>
#include <ctime> // for time / date
#include "Telegram_MSG_1.h"
#include "Telegram_MSG_2.h"
#include "Telegram_MSG_3.h"
#include "log.h"

#define ACTIVE_TIMEOUT 30 // retries before stopping the active pinging.

class Plane
{
	// Public functions to be used on all Messages
	public:
	Plane(std::string flightNumber, std::string Planeowner, std::string UniqueID, Log logfile);
	bool IsActive();
	void CountDownActiveCounter();
	void ResetActiveCounter();
	std::string GetFlightNumber();
	std::string GetPlaneOwner();
	std::string GetUniqueID();
    Telegram_MSG_3 * GetTelegramCMD(ProtocolCMD_t cmd);	
	void logDataTelegram(Telegram_MSG_1 *msg);
		
	// General helper functions and varibels only used by inherited MSG classes.
	protected:
	
	// Parameters only used on Telegram mother class.
	
	private:
	std::string _flightnumber = "";
	std::string _owner = "";
	std::string _uniqueIDbase64 = "";
	uint8_t _active = ACTIVE_TIMEOUT;			
	Log _logfile;
};



#endif /* PLANE_H_ */