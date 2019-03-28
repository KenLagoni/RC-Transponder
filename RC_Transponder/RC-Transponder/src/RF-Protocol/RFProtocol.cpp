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
	this->state = RX_IDLE;
 }
 
 void RFProtocol::TxTelegram(Telegram *msg)
 {
	 // fail fast:
	if(this->txFIFO.isFull()){
		delete msg;
		Serial.println("Error! - Tx FIFO Full, deleting new data");
		return;
	}

	this->txFIFO.push(msg);	//Add to TX FIFO

	//Downcasting to correct telegram.
	//Telegram_MSG_1 *test = (Telegram_MSG_1 *)&msg;	
	//Radio->SendPackage(msg->GetRadioMSG(),msg->GetRadioMSGLength());
	//Radio->SendPackage(test->GetRadioMSG(),test->GetRadioMSGLength());

	if(state == RX_IDLE){ // force transmit	to get IRQ going.			
		Telegram *outgoingmsg = NULL;
		this->txFIFO.pop(outgoingmsg);	//get from FIFIO.
		Radio->SendPackage(outgoingmsg->GetRadioMSG(),outgoingmsg->GetRadioMSGLength());
		
		switch(outgoingmsg->GetRadioMSG_ID())
		{
			case MSG_Beacon_Broadcast:
			case MSG_Beacon_Relay:
			{
				// Start transmission and don't wait for reply.
				this->state=TX_WITHOUT_REPLY;						
			}
			break;
			
			case MSG_Command:
			{
				// Start transmission and wait for reply.
				this->state=TX_WITH_REPLY;
			}
			break;
		   
			default:
			break;
		}
		
		delete outgoingmsg;
	}	
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



Telegram_MSG_2 * RFProtocol::GetSavedTransponderBeaconForRelay(){
	
	if(RFProtocolStatus.NumberOfBeaconsToRelay == 0){
		return NULL;
	}
	
	Telegram_MSG_1 *savedmsg = SavedBeacons[RFProtocolStatus.NumberOfBeaconsToRelay-1];
	RFProtocolStatus.NumberOfBeaconsToRelay--;	
	
	// Convert to relay msg:
	Telegram_MSG_2 *msg = new Telegram_MSG_2(SavedBeacons[RFProtocolStatus.NumberOfBeaconsToRelay-1]);
	delete savedmsg;

	return msg;
}

bool RFProtocol::SaveTransponderBeacon(Telegram_MSG_1 *msg){
	 if(msg->GetNumberOfSecondsSinceLastGroundStationCom() > 20){
		 // Have we saved a beacon from this unit before? if so updated is:
		 for(int a=0;a<RFProtocolStatus.NumberOfBeaconsToRelay; a++){
			 //						Serial.print("Is memory location "+ String(a) + " from the same beacon?... ");
			 if( SavedBeacons[a]->TelegramMatchUniqueID(msg->GetUniqueID1(), msg->GetUniqueID2(), msg->GetUniqueID3(), msg->GetUniqueID4()) == true ){
				 //							Serial.println("Yes! - Updating!");
				 SavedBeacons[a] = msg;
				 return true; // Done, msg was updated in list.

				 }else{
				 //					Serial.println("No!");
			 }
		 }
	 
		 // We should save this if room
		 if(RFProtocolStatus.NumberOfBeaconsToRelay < FIFO_SIZE){
			 //			Serial.println("Message is saved in memory slot: " + String(NumberOfBeaconsToRelay));
			 SavedBeacons[RFProtocolStatus.NumberOfBeaconsToRelay] = msg;
			 RFProtocolStatus.NumberOfBeaconsToRelay++;
			 return true; // Done, msg was added to list
		 }else{
			 //	Serial.println("Memory is full! - can't save message" + String(NumberOfBeaconsToRelay));
			 //		Serial.println("");
			 //			Serial.println("-------------- Printing Beacon messages saved in Memory -------------");
			 
			 // debug
			 for(int a=0;a<FIFO_SIZE; a++){
				 //			Serial.println("Messages number: " + String(a));
				 SavedBeacons[a]->SerialPrintMessage();
			 }
		 }
	}	
	return false;
}

RFProtocol::RFProtocolStates_t RFProtocol::RXHandler(){
	Telegram *msg = ConvertIncommingDataToTelegram();
	if(msg!=NULL){
		bool SaveTelegram = false;
					
		switch(msg->GetRadioMSG_ID())
		{
			case MSG_Beacon_Broadcast:
				SaveTelegram = SaveTransponderBeacon((Telegram_MSG_1 *)msg);				
			break;

			case MSG_Beacon_Relay:
					// Do nothing.
			break;

			case MSG_Command:
				if(msg->TelegramMatchUniqueID(SystemInformation->SerialNumber1, SystemInformation->SerialNumber2, SystemInformation->SerialNumber3, SystemInformation->SerialNumber4))
				{
					// Command received for us!
					switch(((Telegram_MSG_3 *)msg)->GetCommand())
					{
						case CMD_Request_Transponder_Beacon:
						{
							Serial.println("Reply with Transponder Beacon!");
							 
							// Reply with transponder beacon:
							/*
							Telegram_MSG_1 msgReply = Telegram_MSG_1(SerialNumber1, SerialNumber2, SerialNumber3, SerialNumber4,
							(uint32_t)GPSData->UTCTime, GPSData->Latitude, GPSData->Longitude,
							GPSData->NumberOfSatellites, GPSData->FixDecimal, (SystemStatus.state==RUNNING_ON_BATTERY),
							0, 0,
							SystemStatus.SecondCounterSinceLasteGroundStationContact, SystemStatus.BatteryVoltage, SystemStatus.FIRMWARE_VERSION, PCB_VERSION, SystemStatus.NumberOfBeaconsToRelay);
							*/
							//Radio->SendPackage(msgReply->GetRadioMSG(), msgReply->GetRadioMSGLength())
							return TX_WITHOUT_REPLY;
						}
						break;

						case CMD_Request_NEXT_Beacon_Relay:
						{
							Serial.println("Reply with next saved beacon if any beacons left to sent: " + String(RFProtocolStatus.NumberOfBeaconsToRelay));
							// Reply with transponder beacon:
							Telegram_MSG_2 * msgReply = GetSavedTransponderBeaconForRelay();
							if(msgReply != NULL){
								Radio->SendPackage(msgReply->GetRadioMSG(), msgReply->GetRadioMSGLength());
								delete msgReply;
								return TX_WITHOUT_REPLY;
							}
						}
						break;

						case CMD_Do_Power_Off:
						{
							Serial.println("Power off");
						}
						break;		
								
						default:
						break;					
					}
				}
			break;
		}					
		// Save in RX FIFO:
		if(!(rxFIFO.isFull())){
			rxFIFO.push(msg);
			SaveTelegram=true;
		}
		if(!SaveTelegram) // delete message if FIFO was full and telegram not saved in SavedBeacons List.
			delete msg;
	}		

	return RX_IDLE;
}

RFProtocol::RFProtocolStates_t RFProtocol::TXHandler(){
	Telegram *msg = NULL;
	if(txFIFO.pop(msg)){
		if(msg!=NULL){
			bool SaveTelegram = false;
					
			switch(msg->GetRadioMSG_ID())
			{
				case MSG_Beacon_Broadcast:
				case MSG_Beacon_Relay:
					Radio->SendPackage(msg->GetRadioMSG(), msg->GetRadioMSGLength());
					delete msg;
					return TX_WITHOUT_REPLY;
				break;

				case MSG_Command:
					switch(((Telegram_MSG_3 *)msg)->GetCommand())
					{
						case CMD_Request_Transponder_Beacon:
						case CMD_Request_NEXT_Beacon_Relay:
						{
							Radio->SendPackage(msg->GetRadioMSG(),msg->GetRadioMSGLength());
							delete msg;
							return TX_WITH_REPLY;
						}
						break;

						case CMD_Do_Power_Off:
						{
							Radio->SendPackage(msg->GetRadioMSG(),msg->GetRadioMSGLength());
							delete msg;
							return TX_WITHOUT_REPLY;
							Serial.println("Power off");
						}
						break;		
								
						default:
						break;					
					}
				break;
			}
		}		
	}
	return RX_IDLE;
}

void RFProtocol::IRQHandler()
 {
	 if(state != SLEEP){
		this->Radio->IRQHandler();
		RadioStatus status = Radio->GetRadioStatus();
		RFProtocolStates_t nextState = RX_IDLE;
	
		switch(state)
		{
			case RX_IDLE:
			{
				if(status.rxDone == true){
				// New package received  
					nextState=RXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
					if(nextState == RX_IDLE){ // If we didn't need to reply to the incoming data, then test if we need to transmit anything from the buffer:
						nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
					}
				}
				 
				if((status.txDone == true) || (status.txTimeout == true) || (status.rxTimeout == true)) {
					// This must be a mistake?
					Serial.println("RF Protocol Error: txDone||txTimeout||rxTimeout in RX_IDLE State");				
					nextState=RX_IDLE;
				}				 
			}
			break;
		  	 
			case WAITING_FOR_REPLY:
			{
				if(status.rxDone == true){
					// New package received
					nextState=RXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
					if(nextState == RX_IDLE){ // If we didn't need to reply to the incoming data, then test if we need to transmit anything from the buffer:
						nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
					}
				}
				if(status.rxTimeout == true){
					// Send next telegram in FIFO, else to nextstate will be RX_IDLE.
					nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
				}
				if((status.txDone == true) || (status.txTimeout == true)) {
					// This must be a mistake?
					Serial.println("RF Protocol Error: txDone||txTimeout in WAITING_FOR_REPLY State");
					nextState=RX_IDLE;
				} 
			}
			break;
		  	 
			   
			case TX_WITHOUT_REPLY:
			{
				if(status.txDone == true){
					// Done sending - Send next telegram in FIFO, else to nextstate will be RX_IDLE.
					nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
				}
				if(status.txTimeout == true){
					// Error in transmission
					Serial.println("RF Protocol Error: Transmission failed!");
					nextState=RX_IDLE;
				}
				if((status.rxDone == true) || (status.rxTimeout == true)) {
					// This must be a mistake?
					Serial.println("RF Protocol Error: rxDone||rxTimeout in TX_WITHOUT_REPLY State");
					nextState=RX_IDLE;
				}			  	 
			}
			break;

			case TX_WITH_REPLY:
			{
				if(status.txDone == true){
					// Done sending, set Radio to RX with timeout.
					nextState=WAITING_FOR_REPLY;
				}
				if(status.txTimeout == true){
					// Error in transmission
					nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
					Serial.println("RF Protocol Error: Transmission failed!");
				}
				if((status.rxDone == true) || (status.rxTimeout == true)) {
					// This must be a mistake?
					Serial.println("RF Protocol Error: rxDone||rxTimeout in TX_WITH_REPLY State");
					nextState=RX_IDLE;
				}
			}
			break;
		/*
			case SLEEP:
			{
			  	 
			}
			break;
		*/
		   
			default:
			break;
		}

		if(nextState == WAITING_FOR_REPLY){
			Radio->SetRXMode(true); // Set RX with timeout.
		}else if(nextState == RX_IDLE){
			Radio->SetRXMode(false); // Set RX without timeout.
		}

 		state=nextState;	 
	 }  
 }
 
 
Telegram * RFProtocol::ConvertIncommingDataToTelegram(){

	 RadioData *newdata = Radio->GetRadioData();
 
	 if(newdata == NULL)
		return NULL;
	 
	 ProtocolMSG_t newMessageID = (ProtocolMSG_t)newdata->payload[0];
	 
	 Telegram *msg =NULL;

	 switch(newMessageID)
	 {
		 case MSG_Beacon_Broadcast: // Create Beacon Telegram.
		 {
			 msg =new Telegram_MSG_1(newdata->payload, newdata->payloadLength);
		 }
		 break;
		 
		 case MSG_Beacon_Relay: //Create Beacon Realy Telegram.
		 {
			 msg =new Telegram_MSG_2(newdata->payload, newdata->payloadLength);
		 }
		 break;
		 
		 case MSG_Command: // Create command Telegram.
		 {
			 msg =new Telegram_MSG_2(newdata->payload, newdata->payloadLength);
		 }
		 break;
		 
		 default:
			Serial.println("Unknown Incoming message! from Radio.");
			delete newdata; 
			return NULL;
		 break;
	 }

	 delete newdata; 
	 if(msg->CRCValid()){
		 return msg;
	 }else{
		 delete msg;
		 return NULL;
	 }
 }

 
					
