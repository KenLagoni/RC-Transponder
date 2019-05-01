/*
 * plane.cpp
 *
 * Created: 14-Apr-19 19:33:21
 *  Author: Kenneth
 */ 
#include "plane.h"


Plane::Plane(std::string flightNumber, std::string Planeowner, std::string UniqueID, Log logfile)
{
	_flightnumber=flightNumber;
	_owner= Planeowner;
	_uniqueIDbase64 = UniqueID;
	_logfile = logfile;
}

void Plane::CountDownActiveCounter()
{
	if(_active > 0)
	{
		_active--;
	}
}

void Plane::ResetActiveCounter()
{
	_active = ACTIVE_TIMEOUT;
}

bool Plane::IsActive()
{
	if(_active == 0)
		return false;
	else
		return true;
}

std::string Plane::GetFlightNumber()
{
	return _flightnumber;	
}

std::string Plane::GetPlaneOwner()
{
	return _owner;
}
std::string Plane::GetUniqueID()
{
	return _uniqueIDbase64;
}

Telegram_MSG_3 * Plane::GetTelegramCMD(ProtocolCMD_t cmd)
{
	Telegram_MSG_3 *msg = new Telegram_MSG_3(_uniqueIDbase64, cmd);
	return msg;
}

void Plane::logDataTelegram(Telegram_MSG_1 *msg)
{
	
	double LatitudeHH_DMS = (double)(msg->GetLatitude() / 1000000);
	double LatitudeMM_DMS = (double)(msg->GetLatitude() - (LatitudeHH_DMS * 1000000)) / 10000;
	double LatitudeSEC_DMS = 0;
	double latitudeDD = (double)(LatitudeHH_DMS + LatitudeMM_DMS / 60 + LatitudeSEC_DMS / 3600);
	
	double LongitudeHH_DMS = (double)(msg->GetLongitude() / 1000000);
	double LongitudeMM_DMS = (double)(msg->GetLongitude() - (LongitudeHH_DMS * 1000000)) / 10000;
	double LongitudeSEC_DMS = 0;
	double longitudeDD =(double)(LongitudeHH_DMS + LongitudeMM_DMS / 60 + LongitudeSEC_DMS / 3600);
	
	std::time_t t = std::time(0);   // get time now
	std::tm* now = std::localtime(&t);
	
	std::string sec = "";
	std::string min = "";	
	std::string hour = "";
	std::string day = "";
	std::string month = "";
	std::string year = std::to_string(now->tm_year+1900);
	
	if(now->tm_sec < 10)
	{
		sec = "0" + std::to_string(now->tm_sec);
	}
	else
	{
		sec = std::to_string(now->tm_sec);
	}
	
	if(now->tm_min < 10)
	{
		min = "0" + std::to_string(now->tm_min);
	}
	else
	{
		min = std::to_string(now->tm_min);
	}
	
	if(now->tm_hour < 10)
	{
		hour = "0" + std::to_string(now->tm_hour);
	}
	else
	{
		hour = std::to_string(now->tm_hour);
	}
	
	if(now->tm_mday < 10)
	{
		day = "0" + std::to_string(now->tm_mday);
	}
	else
	{
		day = std::to_string(now->tm_mday);
	}
	
	if(now->tm_mon < 10)
	{
		month = "0" + std::to_string(now->tm_mon);
	}
	else
	{
		month = std::to_string(now->tm_mon);
	}
	
	
	
	
	
	std::string data = hour + ":" + min + ":" + sec + "-" + day + "-" + month + "-" + year + ","
						    + std::to_string(msg->GetUTCTime()) + ","
						    + std::to_string(latitudeDD) + ","	
							+ std::to_string(longitudeDD) + ","
							+ std::to_string(msg->GetFix()) + ","
							+ std::to_string(msg->GetNumberOfSat()) + ","
							+ std::to_string(msg->GetPressure()) + ","
							+ std::to_string(msg->GetGroundSpeed()) + ","
							+ std::to_string(msg->GetSecondsSinceLastGSContact()) + ","
							+ std::to_string(msg->GetBatteryVoltage()) + ","
							+ std::to_string(msg->GetFirmwareVersion()) + ","
							+ std::to_string(msg->GetPCBVersion()) + ","
							+ std::to_string(msg->GetNumberOfBeaconsToRelay()) + ","
							+ std::to_string(msg->GetRSSI()) + ","
							+ std::to_string(msg->GetSNR()) + '\n';
	// log data:
	// LOG TIME, UTC TIME HHmmss, LAT , LONG , FIX , #sat , pressure , groundspeed , seconds sins last ground contact, vbat, Firmware version, PCB verison, Number of sat to relay, rssi , snr
	std::cout << "Data to log:" << data << ":" << std::endl;
	_logfile.WriteLog(data);		
	//this->writeLog(data);
}