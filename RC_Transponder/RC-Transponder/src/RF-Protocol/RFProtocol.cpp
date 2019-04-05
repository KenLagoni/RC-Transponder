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


 RFProtocol::RFProtocol(E28_2G4M20S *Radio, GpsDataLite *GPS, SystemInformation_t *status){
	this->Radio = Radio;
	this->GPSData = GPS;
	this->SystemInformation = status;

	Radio->Init();
	Radio->SetRXMode(false); // no timeout
	this->RFstate = RX_IDLE;
	
	for(int a=a;a<FIFO_SIZE;a++)	
		SavedBeacons[a]=NULL;
 }
 
 void RFProtocol::AddData(RadioData_t *newdata)
 {
	 // fail fast:
	if(this->txFIFO.isFull()){
		SerialAUX->println("Error! - Tx FIFO Full, deleting new data");
		return;
	}

	Telegram *msg = this->ConvertToTelegram(newdata);
	
	//Convert Telegram to specefic telegram.

	if(msg!=NULL)
	{		
		this->txFIFO.push(msg);	//Add to TX FIFO
		//this->ServiceStateMachine();
	}
}

RadioData_t * RFProtocol::GetData()
{
	if(this->rxFIFO.isEmpty())
		return NULL;
	else{
		Telegram * msg = NULL;
		rxFIFO.pop(msg);
		memset(&rxbuffer.payload, 0 , MAX_PAYLOAD_LENGTH);
		memcpy(&rxbuffer.payload, msg->GetRadioData()->payload, msg->GetRadioData()->payloadLength);	// copy the data.
		rxbuffer.payloadLength = msg->GetRadioData()->payloadLength;
		rxbuffer.rssi = msg->GetRadioData()->rssi;
		rxbuffer.snr = msg->GetRadioData()->snr;
		delete msg;
		return &rxbuffer;
	}
}

 int RFProtocol::Available()
 {
	 return this->rxFIFO.size();
 }

 void RFProtocol::SendBeacon()
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

Telegram_MSG_2 * RFProtocol::GetSavedTransponderBeaconForRelay(){
	
	if(RFProtocolStatus.NumberOfBeaconsToRelay == 0){
		return NULL;
	}
	
	Telegram_MSG_1 *savedmsg = SavedBeacons[RFProtocolStatus.NumberOfBeaconsToRelay-1];
	RFProtocolStatus.NumberOfBeaconsToRelay--;	
	
	// Convert to relay msg
	//Telegram_MSG_2 test = Telegram_MSG_2(savedmsg);
	
	Telegram_MSG_2 *msg = new Telegram_MSG_2(savedmsg);
	delete savedmsg;

	return msg;
}

bool RFProtocol::SaveTransponderBeacon(Telegram_MSG_1 *msg){
	 SerialAUX->println("msg seconds since last:" + String(msg->GetNumberOfSecondsSinceLastGroundStationCom()));
	 SerialAUX->println("Number of beacons to relay:" + String(RFProtocolStatus.NumberOfBeaconsToRelay));
	 
	 if(msg->GetNumberOfSecondsSinceLastGroundStationCom() > 20){
		 SerialAUX->print("We should save this message...");
		 // Have we saved a beacon from this unit before? if so updated is:
		 for(int a=0;a<RFProtocolStatus.NumberOfBeaconsToRelay; a++){
			 //						SerialAUX->Print("Is memory location "+ String(a) + " from the same beacon?... ");
			 if( SavedBeacons[a]->TelegramMatchUniqueID(msg->GetUniqueID1(), msg->GetUniqueID2(), msg->GetUniqueID3(), msg->GetUniqueID4()) == true ){
				 //							SerialAUX->Println("Yes! - Updating!");
				 SavedBeacons[a] = msg;
				 SerialAUX->println("It was all-ready in the list! at:" + String(a) + " We have a total of:" + String(RFProtocolStatus.NumberOfBeaconsToRelay) + "To relay");
				 return true; // Done, msg was updated in list.

				 }else{
				 //					SerialAUX->Println("No!");
			 }
		 }
	 
		 // We should save this if room
		 if(RFProtocolStatus.NumberOfBeaconsToRelay < FIFO_SIZE){
			 //			SerialAUX->Println("Message is saved in memory slot: " + String(NumberOfBeaconsToRelay));
			 SavedBeacons[RFProtocolStatus.NumberOfBeaconsToRelay] = msg;
			 SerialAUX->println("It was added to the list at" + String( RFProtocolStatus.NumberOfBeaconsToRelay));
			 RFProtocolStatus.NumberOfBeaconsToRelay++;
			 return true; // Done, msg was added to list
		 }else{
			 //	SerialAUX->Println("Memory is full! - can't save message" + String(NumberOfBeaconsToRelay));
			 //		SerialAUX->Println("");
			 //			SerialAUX->Println("-------------- Printing Beacon messages saved in Memory -------------");
			 
			 // debug
			 SerialAUX->println("FIFO Full! unable to save any more messages :-(");
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

RFProtocol::RFProtocolStates_t RFProtocol::RXHandler()
{
	Telegram *msg = ConvertToTelegram(Radio->GetRadioData());
	RFProtocolStates_t returnState =RX_IDLE; 

	if(msg!=NULL)
	{
		bool SaveTelegram = false;
					
		switch(msg->GetRadioMSG_ID())
		{
			case MSG_Beacon_Broadcast:

				SerialAUX->print("\n\r************ MSG - 1(MSG_Beacon_Broadcast) Received!....");
				SaveTelegram = SaveTransponderBeacon((Telegram_MSG_1 *)msg);
				
				//Telegram_MSG_1 *msg1 = ((Telegram_MSG_1 *)msg).GetNumberOfBeaconsToRelay();				
				SerialAUX->println("msg has number of beacons to relay:" + String(((Telegram_MSG_1 *)msg)->GetNumberOfBeaconsToRelay()));
							
				if(SaveTelegram){
					SerialAUX->println("Saved");					
				}else{
					SerialAUX->println("Deleted");					
				}
			break;

			case MSG_Beacon_Relay:
				SerialAUX->println("\n\r************ MSG - 2(MSG_Beacon_Relay) Received!.... Delete");
					// Do nothing.
			break;

			case MSG_Command:
				SerialAUX->print("\n\r************ MSG - 3 Received!...");
				if(msg->TelegramMatchUniqueID(SystemInformation->SerialNumber1, SystemInformation->SerialNumber2, SystemInformation->SerialNumber3, SystemInformation->SerialNumber4))
				{
					SerialAUX->println("For us!");
					RFProtocolStatus.SecondCounterSinceLasteGroundStationContact=0;
					// Command received for us!
					switch(((Telegram_MSG_3 *)msg)->GetCommand())
					{
						case CMD_Request_Transponder_Beacon:
						{
							SerialAUX->print("Reply with Transponder Beacon?...");
							// Reply with transponder beacon:
														
							 if(RFProtocolStatus.Sleep == true){ // don't send reply if sleep mode is requested.
								 SerialAUX->println("No - We are asleep zzzz");
								 returnState=RX_IDLE;							
							 }else{
								 SerialAUX->println("Yes!");
								Telegram_MSG_1 msgReply = Telegram_MSG_1(SystemInformation->SerialNumber1, SystemInformation->SerialNumber2,
 																		SystemInformation->SerialNumber3, SystemInformation->SerialNumber4,
 																		(uint32_t)GPSData->UTCTime, GPSData->Latitude, GPSData->Longitude,
 																		GPSData->NumberOfSatellites, GPSData->FixDecimal, ((SystemInformation->state==RUNNING_ON_BATTERY_GPS_ON) || (SystemInformation->state==GET_READY_TO_RUN_ON_BATTERY) || (SystemInformation->state==RUNNING_ON_BATTERY_GPS_OFF)),
 																		0, 0,
																		RFProtocolStatus.SecondCounterSinceLasteGroundStationContact, SystemInformation->BatteryVoltage,
																		SystemInformation->FIRMWARE_VERSION, SystemInformation->pcbVersion, RFProtocolStatus.NumberOfBeaconsToRelay);
								Radio->SendRadioData(msgReply.GetRadioData());
								returnState=TX_WITHOUT_REPLY;
							 }
						}
						break;

						case CMD_Request_NEXT_Beacon_Relay:
						{
							SerialAUX->println("Reply with next saved beacon if any beacons left to sent: " + String(RFProtocolStatus.NumberOfBeaconsToRelay));
							// Reply with transponder beacon:

							if(RFProtocolStatus.Sleep == true){ // don't send reply if sleep mode is requested.
								returnState=RX_IDLE;
							}else
							{
								Telegram_MSG_2 * msgReply = GetSavedTransponderBeaconForRelay();
								if(msgReply != NULL){
									Radio->SendRadioData(msgReply->GetRadioData());
									delete msgReply;
									returnState=TX_WITHOUT_REPLY;
								}	
							}
							
						}
						break;

						case CMD_Do_Power_Off:
						{
							SerialAUX->println("Power off");
							digitalWrite(led2Pin, HIGH);	
							delay(2000);
							digitalWrite(led2Pin, LOW);	
							SystemInformation->state = POWER_OFF;
						}
						break;		
						
						case CMD_Simulate_run_on_battery:
						{
							SerialAUX->println("RF Message says - Simulate run on battery");
							SystemInformation->SimulateRunningOnBattery = !SystemInformation->SimulateRunningOnBattery; // toggle.
						}
						break;
													
						default:
						break;					
					}
				}
				else{
					SerialAUX->println("Not For us!");
				}
			break;
			
			default:
				SerialAUX->println("\n\r************MSG - ? Default!....");
			break;
		}		
					
		// Save in RX FIFO:
		if(!(rxFIFO.isFull()))
		{
			SerialAUX->println("Adding MSG to RX FIFO.");	
			rxFIFO.push(msg);
			SaveTelegram=true;
		}else
		{
			SerialAUX->println("RX FIFO full!.");	
		}
		
		if(!SaveTelegram)
		{ // delete message if FIFO was full and telegram not saved in SavedBeacons List.
			delete msg;
		}
		
	}else{
		SerialAUX->println("MSG is NULL!");
	}
	 // if msg!=null		
	return returnState;
}

RFProtocol::RFProtocolStates_t RFProtocol::TXHandler(){
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
							SerialAUX->println("Unknown MSG CMD to send !.");
						break;					
					}
				break;
			}
			delete msg;
		}		
	}
	return _nextState;
}
/*
void RFProtocol::IRQHandler(){
	int test = digitalRead(dio1Pin);
	SerialAUX->println("IRQ ******* START!:" + String(test));
	this->Radio->IRQHandler();
	this->ServiceStateMachine();
	SerialAUX->println("IRQ ------- STOP!");
}*/
/*
void RFProtocol::ServiceStateMachine()
 {
	RadioIRQStatus_t status = Radio->GetRadioStatus();
	Radio->ClearRadioStatus();

	SerialAUX->println("Service RF Statemachine state:" + String(RX_IDLE) + " and nextState:" + String(nextState));

	switch(state)
	{
		case RX_IDLE:
		{
			if(status.rxDone == true){
			// New package received  
				nextState=RXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
				SerialAUX->println("rxDone: next state:" + String(nextState));
			}
				 
			if((status.txDone == true) || (status.txTimeout == true) || (status.rxTimeout == true)) {
				// This must be a mistake?
				SerialAUX->println("RF Protocol Error: txDone||txTimeout||rxTimeout in RX_IDLE State - Nextstate set to RX_IDLE");				
				nextState=RX_IDLE;
			}				 
			
			if(nextState == RX_IDLE){ // If we didn't need to reply to the incoming data, then test if we need to transmit anything from the buffer:
				nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
				SerialAUX->println("RX_IDLE - RX_IDLE: next state:" + String(nextState));
				if(nextState == RX_IDLE){
					Radio->SetRXMode(false); // Set RX without timout.
				}
			}
		}
		break;
		  	 
		case WAITING_FOR_REPLY:
		{
			if(status.rxDone == true){
				// New package received
				nextState=RXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDL
				SerialAUX->println("rxDone: next state:" + String(nextState));
				if(nextState == RX_IDLE){ // If we didn't need to reply to the incoming data, then test if we need to transmit anything from the buffer:
					nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
				}
			}
			if(status.rxTimeout == true){
				// Send next telegram in FIFO, else to nextstate will be RX_IDLE.
				nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
				SerialAUX->println("rxTimeout: next state:" + String(nextState));
			}
			if((status.txDone == true) || (status.txTimeout == true)) {
				// This must be a mistake?
				SerialAUX->println("RF Protocol Error: txDone||txTimeout in WAITING_FOR_REPLY State");
				nextState=RX_IDLE;
			} 
			if(nextState == RX_IDLE)
				SerialAUX->println("RX_IDLE in waiting for reply: next state:" + String(nextState));
				Radio->SetRXMode(false); // Set RX without timout.	
		}
		break;
		  	 
			   
		case TX_WITHOUT_REPLY:
		{
			if(status.txDone == true){
				// Done sending - Send next telegram in FIFO, else to nextstate will be RX_IDLE.
				nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE'
				SerialAUX->println("txDone: next state:" + String(nextState));
				if(nextState == RX_IDLE)
					Radio->SetRXMode(false); // Set RX without timout.				
			}
			if(status.txTimeout == true){
				// Error in transmission
				SerialAUX->println("RF Protocol Error: Transmission failed!");
				nextState=RX_IDLE;
			}
			if((status.rxDone == true) || (status.rxTimeout == true)) {
				// This must be a mistake?
				SerialAUX->println("RF Protocol Error: rxDone||rxTimeout in TX_WITHOUT_REPLY State");
				nextState=RX_IDLE;
			}			  	 
			if(nextState == RX_IDLE)
				SerialAUX->println("RX_IDLE: next state:" + String(nextState));
				Radio->SetRXMode(false); // Set RX without timout.
		}
		break;

		case TX_WITH_REPLY:
		{
			if(status.txDone == true){
				// Done sending, set Radio to RX with timeout.
				Radio->SetRXMode(true); // Set RX with timeout.
				SerialAUX->println("txDone: next state:" + String(nextState));
				nextState=WAITING_FOR_REPLY;
			}
			if(status.txTimeout == true){
				// Error in transmission
				nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
				SerialAUX->println("RF Protocol Error: Transmission failed! - nextState:" + String(nextState));
			}
			if((status.rxDone == true) || (status.rxTimeout == true)) {
				// This must be a mistake?
				SerialAUX->println("RF Protocol Error: rxDone||rxTimeout in TX_WITH_REPLY State");
				Radio->SetRXMode(false); // Set RX without.
				nextState=RX_IDLE;
			}
			if(nextState == RX_IDLE)
				Radio->SetRXMode(false); // Set RX without timout.
		}
		break;
			   
		default:
			SerialAUX->println("Error! - Default!");
			Radio->SetRXMode(false); // Set RX without.
			nextState=RX_IDLE;
		break;
	}
 	state=nextState;	 
 }
  */
Telegram * RFProtocol::ConvertToTelegram(RadioData_t *newdata) // must delete newdata to avoid memory leaks.
{ 
	 SerialAUX->print("New message is...");
	 if(newdata == NULL){
		SerialAUX->println("NULL!");
		return NULL;
	 }
	 
	 ProtocolMSG_t newMessageID = (ProtocolMSG_t)newdata->payload[0];
	 
	 Telegram *msg =NULL;

	 switch(newMessageID)
	 {
		 case MSG_Beacon_Broadcast: // Create Beacon Telegram.
		 {
			 msg = new Telegram_MSG_1(newdata);
			 SerialAUX->println("MSG 1");
		 }
		 break;
		 
		 case MSG_Beacon_Relay: //Create Beacon Relay Telegram.
		 {
			 msg = new Telegram_MSG_2(newdata);
			 SerialAUX->println("MSG 2");
		 }
		 break;
		 
		 case MSG_Command: // Create command Telegram.
		 {
			 msg = new Telegram_MSG_3(newdata);
			 SerialAUX->println("MSG 3");
		 }
		 break;
		 
		 default:
			SerialAUX->println("Unknown/Null!");
			return NULL;
		 break;
	 }
	return msg;	
 }

 
void RFProtocol::PowerDown(){
	// Wait for Radio to finish current transmission.
//	SerialAUX->println("Sleep=true");
	RFProtocolStatus.Sleep = true;
	
	do{
		this->RFService();
		}while(this->RFstate!=RX_IDLE);

	Radio->Sleep();
}		

void RFProtocol::WakeUp(){
//	SerialAUX->println("Sleep=false");
	RFProtocolStatus.Sleep = false;
	Radio->WakeUp();
	Radio->SetRXMode(false); // Set Radio to RX mode with timeout. This will trigger rx.timeout, and thus the continues flow of the statemachine, should there be data left in TX fifo.
}

void RFProtocol::RFService(){

	if(digitalRead(dio1Pin) == HIGH){
		this->Radio->IRQHandler();
	}

	RadioIRQStatus_t status = Radio->GetRadioStatus();
	Radio->ClearRadioStatus();

	RFProtocolStates_t nextState = RX_IDLE;

//	SerialAUX->println("Service RF Statemachine state:" + String(RX_IDLE) + " and nextState:" + String(nextState));


	switch(this->RFstate)
	{
		case RX_IDLE:
		{
			nextState=RX_IDLE;

			// fail fast:
			if((status.txDone == true) || (status.txTimeout == true) || (status.rxTimeout == true))
			{
				// This must be a mistake?
				SerialAUX->println("RF Protocol Error: txDone||txTimeout||rxTimeout in RX_IDLE State - Nextstate set to RX_IDLE");
				Radio->SetRXMode(false); // Set RX without timout.
				nextState=RX_IDLE;
			}else
			{
				if(status.rxDone == true)
				{
					// New package received
					nextState=RXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
				}else
				{
					// No news, lets see if we need to send somthing from the buffer. 
					nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
				}
			}
//			SerialAUX->println("RX_IDLE: next state:" + String(nextState));
		}
		break;
		
		case WAITING_FOR_REPLY:
		{
			nextState=WAITING_FOR_REPLY;

			// fail fast:
			if((status.txDone == true) || (status.txTimeout == true))
			{
				// This must be a mistake?
				SerialAUX->println("RF Protocol Error: txDone||txTimeout in WAITTING_FOR_REPLY State - Nextstate set to RX_IDLE");
				Radio->SetRXMode(false); // Set RX without timout.
				nextState=RX_IDLE;
			}else
			{
				if(status.rxDone == true)
				{
					// New package received
					nextState=RXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
				}else
				{
					if(status.rxTimeout == true){
						// No reply recieved, lets go to RX_IDLE
						nextState=RX_IDLE;
						Radio->SetRXMode(false); // Set RX without timout.
					}else{
						// Nothing........
						// Software timeout here?
						if(millis() > this->timeoutStart+TIMEOUT_MS){
							SerialAUX->println("Software timeout on WAITING_FOR_REPLY");
							nextState=RX_IDLE;
							Radio->SetRXMode(false); // Set RX without timout.
						}
					}
				}
			}
	//		SerialAUX->println("WAITING_FOR_REPLY: nextState:" + String(nextState));
		}
		break;
		
		
		case TX_WITHOUT_REPLY:
		{
			nextState=TX_WITHOUT_REPLY;
			if((status.rxDone == true) || (status.rxTimeout == true) || (status.txTimeout == true)) 
			{
				// This must be a mistake?
				SerialAUX->println("RF Protocol Error: rxDone||rxTimeout||txTimeout in TX_WITHOUT_REPLY State");
				nextState=RX_IDLE;
				Radio->SetRXMode(false); // Set RX without timout.
			}else
			{
				if(status.txDone == true){
					// Done sending 
					nextState=RX_IDLE; 
					Radio->SetRXMode(false); // Set RX without timout.			
				}else{
					// nothing... perhaps software timeout here?
					if(millis() > this->timeoutStart+TIMEOUT_MS){
						SerialAUX->println("Software timeout on TX_WITHOUT_REPLY");
						nextState=RX_IDLE;
						Radio->SetRXMode(false); // Set RX without timout.
					}
				}
			}		
	//		SerialAUX->println("TX_WITHOUT_REPLY: nextState:" + String(nextState));
		}
		break;

		case TX_WITH_REPLY:
		{
			nextState=TX_WITH_REPLY;
			if((status.rxDone == true) || (status.rxTimeout == true) || (status.txTimeout == true))
			{
				// This must be a mistake?
				SerialAUX->println("RF Protocol Error: rxDone||rxTimeout||txTimeout in TX_WITHOUT_REPLY State");
				nextState=RX_IDLE;
				Radio->SetRXMode(false); // Set RX without timout.
			}else
			{
				if(status.txDone == true){
					// Done sending, set Radio to RX with timeout.
					Radio->SetRXMode(true); // Set RX with timeout.
					nextState=WAITING_FOR_REPLY;
				}else{
					// nothing... perhaps software timeout here?
					if(millis() > this->timeoutStart+TIMEOUT_MS){
						SerialAUX->println("Software timeout on TX_WITHOUT_REPLY");
						nextState=RX_IDLE;
						Radio->SetRXMode(false); // Set RX without timout.
					}
				}
			}
			//SerialAUX->println("TX_WITHOUT_REPLY: nextState:" + String(nextState));
		}
		break;
		
		default:
			nextState=RX_IDLE;
			SerialAUX->println("Error! - Default!");
			Radio->SetRXMode(false); // Set RX without.
		break;
	}
	
	if(this->RFstate != nextState){
		this->timeoutStart = millis(); // for software timeout.
		SerialAUX->println("New state! - RFSate:" +String(RFstate) + " NextState:" + String(nextState));
		this->RFstate=nextState;

	}
}

void RFProtocol::SeccondCounter(){
	if(RFProtocolStatus.SecondCounterSinceLasteGroundStationContact < 254){
		RFProtocolStatus.SecondCounterSinceLasteGroundStationContact++;
	}
}
