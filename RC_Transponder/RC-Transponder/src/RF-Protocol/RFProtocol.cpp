/*
	RFProtocol.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
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


 RFProtocol::RFProtocol(E28_2G4M20S *Radio, GpsDataLite *GPS, SystemInfomation *status){
	this->Radio = Radio;
	this->GPSData = GPS;
	this->SystemStatus = status;

	Radio->Init();
	Radio->SetRXMode(false);				
 }
 
 void RFProtocol::TxTelegram(Telegram *msg)
 {
	if(!(this->txFIFO.isFull()))
		this->txFIFO.push(msg);
 }

 Telegram * RFProtocol::GetTelegram()
 {
	if(this->rxFIFO.isEmpty())
		return NULL;
	else{
		Telegram * msg = NULL; 
		rxFIFO.pop(msg);
		return msg;
	}
 }


 int RFProtocol::TelegramAvailable()
 {
	 return this->rxFIFO.size();
 }

 void RFProtocol::SendBeacon()
 {
 /*
	Telegram_MSG_1 msg = new Telegram_MSG_1(SerialNumber1, SerialNumber2, SerialNumber3, SerialNumber4,
											(uint32_t)GPSData->UTCTime, GPSData->Latitude, GPSData->Longitude,
											GPSData->NumberOfSatellites, GPSData->FixDecimal, (state==RUNNING_ON_BATTERY),
											//pressure, groundspeed,
											0, 0,
											SecondCounterSinceLasteGroundStationContact, BatteryVoltage, FIRMWARE_VERSION, PCB_VERSION, NumberOfBeaconsToRelay);
											*/
 }


 void RFProtocol::IRQHandler()
 {
	  this->Radio->HandleIRQ();

	  //Do some more protocol stuff here.
 }