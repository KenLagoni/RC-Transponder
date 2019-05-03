/*
 * RFService.cpp
 *
 * Created: 12-Apr-19 22:55:58
 *  Author: Kenneth
 */ 

#include "RFService.h"

RFService::RFService(E28_2G4M20S *Radio, GpsDataLite *GPS, SystemInformation_t *status) : RFProtocol(Radio)
{
	this->GPSData = GPS;
	this->SystemInformation = status;
}

void RFService::SendBeacon()
{
	// Make beacon msg
	float pressure=0;
	float groundspeed=0;
	
	Telegram_MSG_1 *msg = new Telegram_MSG_1(SystemInformation->SerialNumber1, SystemInformation->SerialNumber2, SystemInformation->SerialNumber3, SystemInformation->SerialNumber4,
											 (uint32_t)GPSData->UTCTime, GPSData->Latitude, GPSData->Longitude,
											 GPSData->NumberOfSatellites, GPSData->FixDecimal, ((SystemInformation->state==RUNNING_ON_BATTERY_GPS_ON) || (SystemInformation->state==GET_READY_TO_RUN_ON_BATTERY) || (SystemInformation->state==RUNNING_ON_BATTERY_GPS_OFF)),
											 pressure, groundspeed,
										  	 RFProtocolStatus.SecondCounterSinceLasteGroundStationContact, SystemInformation->BatteryVoltage,
											 SystemInformation->FIRMWARE_VERSION, SystemInformation->pcbVersion, RFProtocolStatus.NumberOfBeaconsToRelay);

	if(this->txFIFO.isFull()){
		SerialAUX->println("Error! - Tx FIFO Full, Unable to send Beacon message");
		//		this->ServiceStateMachine();
	}else{
		if(msg!=NULL){
			this->txFIFO.push(msg);	//Add to TX FIFO
			//			this->ServiceStateMachine();
		}
	}
}


bool RFService::SaveTransponderBeacon(Telegram_MSG_1 *msg){
//	 SerialAUX->println("msg seconds since last:" + String(msg->GetNumberOfSecondsSinceLastGroundStationCom()));
//	 SerialAUX->println("Number of beacons to relay:" + String(RFProtocolStatus.NumberOfBeaconsToRelay));
	 if(msg->GetNumberOfSecondsSinceLastGroundStationCom() > 20){
//		 SerialAUX->print("We should save this message...");
		 // Have we saved a beacon from this unit before? if so updated is:
		 for(int a=0;a<RFProtocolStatus.NumberOfBeaconsToRelay; a++){
			 //						SerialAUX->Print("Is memory location "+ String(a) + " from the same beacon?... ");
			 if( SavedBeacons[a].TelegramMatchUniqueID(msg->GetUniqueID1(), msg->GetUniqueID2(), msg->GetUniqueID3(), msg->GetUniqueID4()) == true ){
				 //							SerialAUX->Println("Yes! - Updating!");
				 SavedBeacons[a] = Telegram_MSG_2(msg);
//				 SerialAUX->println("It was all-ready in the list! at:" + String(a) + " We have a total of:" + String(RFProtocolStatus.NumberOfBeaconsToRelay) + "To relay");
				 return true; // Done, msg was updated in list.

				 }else{
				 //					SerialAUX->Println("No!");
			 }
		 }
	 
		 // We should save this if room
		 if(RFProtocolStatus.NumberOfBeaconsToRelay < FIFO_SIZE){
			 //			SerialAUX->Println("Message is saved in memory slot: " + String(NumberOfBeaconsToRelay));
			 SavedBeacons[RFProtocolStatus.NumberOfBeaconsToRelay] = Telegram_MSG_2(msg);
//			 SerialAUX->println("It was added to the list at:" + String( RFProtocolStatus.NumberOfBeaconsToRelay));
			 Serial.println("It was added to the list at:" + String( RFProtocolStatus.NumberOfBeaconsToRelay));
			 RFProtocolStatus.NumberOfBeaconsToRelay++;
			 return false; // Done, msg was added to list
		 }else{
			 //	SerialAUX->Println("Memory is full! - can't save message" + String(NumberOfBeaconsToRelay));
			 //		SerialAUX->Println("");
			 //			SerialAUX->Println("-------------- Printing Beacon messages saved in Memory -------------");
			 
			 // debug
//			 SerialAUX->println("FIFO Full! unable to save any more messages :-(");
//			 Serial.println("FIFO Full! unable to save any more messages :-(");
			 /*
			 for(int a=0;a<FIFO_SIZE; a++){
				 //			SerialAUX->Println("Messages number: " + String(a));
				 //SavedBeacons[a]->SerialPrintMessage();
				 SerialAUX->println("FIFO Full! unable to save any more messages :-(");
			 }*/
		 }
	}	
	return false;
}

void RFService::SeccondCounter(){
	if(RFProtocolStatus.SecondCounterSinceLasteGroundStationContact < 254){
		RFProtocolStatus.SecondCounterSinceLasteGroundStationContact++;
	}
}

RFProtocol::RFProtocolStates_t RFService::RXHandler()
{
	Telegram *msg = ConvertToTelegram(Radio->GetRadioData());
	RFProtocolStates_t returnState =RX_IDLE; 

	if(msg!=NULL)
	{
		bool SaveTelegram = false;
					
		switch(msg->GetRadioMSG_ID())
		{
			case MSG_Beacon_Broadcast:

//				SerialAUX->print("\n\r************ MSG - 1(MSG_Beacon_Broadcast) Received!....");
				SaveTelegram = SaveTransponderBeacon((Telegram_MSG_1 *)msg);
				
				//Telegram_MSG_1 *msg1 = ((Telegram_MSG_1 *)msg).GetNumberOfBeaconsToRelay();				
//				SerialAUX->println("msg has number of beacons to relay:" + String(((Telegram_MSG_1 *)msg)->GetNumberOfBeaconsToRelay()));
/*							
				if(SaveTelegram){
					SerialAUX->println("Saved");					
				}else{
					SerialAUX->println("Deleted");					
				}
*/
			break;

			case MSG_Beacon_Relay:
//				SerialAUX->println("\n\r************ MSG - 2(MSG_Beacon_Relay) Received!.... Delete");
					// Do nothing.
			break;

			case MSG_Command:
//				SerialAUX->print("\n\r************ MSG - 3 Received!...");
				if(msg->TelegramMatchUniqueID(SystemInformation->SerialNumber1, SystemInformation->SerialNumber2, SystemInformation->SerialNumber3, SystemInformation->SerialNumber4))
				{
//					SerialAUX->println("For us!");
					RFProtocolStatus.SecondCounterSinceLasteGroundStationContact=0;
					// Command received for us!
					switch(((Telegram_MSG_3 *)msg)->GetCommand())
					{
						case CMD_Request_Transponder_Beacon:
						{
//							SerialAUX->print("Reply with Transponder Beacon?...");
							// Reply with transponder beacon:
														
							 if(RFProtocolStatus.Sleep == true){ // don't send reply if sleep mode is requested.
//								 SerialAUX->println("No - We are asleep zzzz");
								 returnState=RX_IDLE;							
							 }else{
//								 SerialAUX->println("Yes!");
								Telegram_MSG_1 msgReply = Telegram_MSG_1(SystemInformation->SerialNumber1, SystemInformation->SerialNumber2,
 																		SystemInformation->SerialNumber3, SystemInformation->SerialNumber4,
 																		(uint32_t)GPSData->UTCTime, GPSData->Latitude, GPSData->Longitude,
 																		GPSData->NumberOfSatellites, GPSData->FixDecimal, ((SystemInformation->state==RUNNING_ON_BATTERY_GPS_ON) || (SystemInformation->state==GET_READY_TO_RUN_ON_BATTERY) || (SystemInformation->state==RUNNING_ON_BATTERY_GPS_OFF)),
 																		0, 0,
																		RFProtocolStatus.SecondCounterSinceLasteGroundStationContact
																		// 100 // Debug
																		,SystemInformation->BatteryVoltage,
																		SystemInformation->FIRMWARE_VERSION, SystemInformation->pcbVersion, RFProtocolStatus.NumberOfBeaconsToRelay);
								Radio->SendRadioData(msgReply.GetRadioData());
								returnState=TX_WITHOUT_REPLY;
							 }
						}
						break;

						case CMD_Request_NEXT_Beacon_Relay:
						{
//							SerialAUX->println("Reply with next saved beacon if any beacons left to sent: " + String(RFProtocolStatus.NumberOfBeaconsToRelay));
							// Reply with transponder beacon:

							if(RFProtocolStatus.Sleep == true){ // don't send reply if sleep mode is requested.
								returnState=RX_IDLE;
							}else
							{
								if(RFProtocolStatus.NumberOfBeaconsToRelay > 0){
									Telegram_MSG_2 msgReply = SavedBeacons[RFProtocolStatus.NumberOfBeaconsToRelay-1];
									RFProtocolStatus.NumberOfBeaconsToRelay--;
									if(msgReply.GetRadioMSG_ID() == MSG_Beacon_Relay)
									{
										Radio->SendRadioData(msgReply.GetRadioData());
										returnState=TX_WITHOUT_REPLY;
									}
								}
							}
							
						}
						break;

						case CMD_Do_Power_Off:
						{
//							SerialAUX->println("Power off");
							digitalWrite(led2Pin, HIGH);	
							delay(2000);
							digitalWrite(led2Pin, LOW);	
							SystemInformation->SaftySwitchPushed=true; // ensure next state is POWER_OFF
						}
						break;		
						
						case CMD_Simulate_run_on_battery:
						{
	//						SerialAUX->println("RF Message says - Simulate run on battery");
							SystemInformation->SimulateRunningOnBattery = !SystemInformation->SimulateRunningOnBattery; // toggle.
						}
						break;
													
						default:
						break;					
					}
				}
				else{
//					SerialAUX->println("Not For us!");
				}
			break;
			
			default:
//				SerialAUX->println("\n\r************MSG - ? Default!....");
			break;
		}		
					
		// Save in RX FIFO:
		if(!(rxFIFO.isFull()))
		{
//			SerialAUX->println("Adding MSG to RX FIFO.");	
			rxFIFO.push(msg);
			SaveTelegram=true;
		}else
		{
//			SerialAUX->println("RX FIFO full!.");	
		}
		
		if(!SaveTelegram)
		{ // delete message if FIFO was full and telegram not saved in SavedBeacons List.
			delete msg;
		}
		
	}else{
//		SerialAUX->println("MSG is NULL!");
	}
	 // if msg!=null		
	return returnState;
}

RFProtocol::RFProtocolStates_t RFService::TXHandler(){
	Telegram *msg = NULL;
	RFProtocolStates_t _nextState = RX_IDLE;
	
	//End fast! - don't do any more TX, main system wants to sleep.
	if(RFProtocolStatus.Sleep == true){
		return _nextState;
	}

	if(txFIFO.pop(msg)){
		if(msg!=NULL){
			switch(msg->GetRadioMSG_ID())
			{
				case MSG_Beacon_Broadcast:
				case MSG_Beacon_Relay:
				Radio->SendRadioData(msg->GetRadioData());
				_nextState=TX_WITHOUT_REPLY;
				break;

				case MSG_Command:
				switch(((Telegram_MSG_3 *)msg)->GetCommand())
				{
					case CMD_Request_Transponder_Beacon:
					case CMD_Request_NEXT_Beacon_Relay:
					{
						Radio->SendRadioData(msg->GetRadioData());
						_nextState=TX_WITH_REPLY;
					}
					break;

					case CMD_Do_Power_Off:
					{
						Radio->SendRadioData(msg->GetRadioData());
						_nextState=TX_WITHOUT_REPLY;
					}
					break;
					
					case CMD_Simulate_run_on_battery:
					{
						Radio->SendRadioData(msg->GetRadioData());
						_nextState=TX_WITHOUT_REPLY;
					}
					break;
					
					default:
					//							SerialAUX->println("Unknown MSG CMD to send !.");
					break;
				}
				break;
			}
			delete msg;
		}
	}
	return _nextState;
}


void RFService::PowerDown(){
	// Wait for Radio to finish current transmission.
	//	SerialAUX->println("Sleep=true");
	RFProtocolStatus.Sleep = true;
	this->_PowerDown();
}

void RFService::WakeUp(){
	//	SerialAUX->println("Sleep=false");
	RFProtocolStatus.Sleep = false;
	this->_WakeUp();
}

uint32_t RFService::milliSeconds(){
	return millis();
}
