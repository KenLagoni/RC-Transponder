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


	
 void RFProtocol::IRQHandler()
 {
	 if(state != SLEEP){
		this->Radio->IRQHandler();
		RadioStatus status = Radio->GetRadioStatus();
		state nextState = RX_IDLE;
	
		switch(state)
		{
			case RX_IDLE:
			{
				if(status.rxDone == true){
				// New package received  
					if(IncommingPackageHandler()){
						nextState=TX_WITHOUT_REPLY;
					}
				}
				 
				if((status.txDone == true) || (status.txTimeout == true) || (status.rxTimeout == true)) {
					// This must be a mistake?
					Serial.println("RF Protocol Error: txDone||txTimeout||rxTimeout in RX_IDLE State");
				}				 
			}
			break;
		  	 
			case WAITING_FOR_REPLY:
			{
				if(status.rxDone == true){
					// New package received
					if(IncommingPackageHandler()){
						nextState=TX_WITHOUT_REPLY; // Radio is busy transmitting something
					}else{
						if(txFIFO.isEmpty()){
							nextState=RX_IDLE;  // no more to TX, then lets just listen
							Radio->SetRXMode(false); // No timeout
						}else{
							// send next data in fifio.
							// delete next data 
						}
							
					}
				}
				if(status.rxTimeout == true){
					// We didn't received any data
					 
					// More data to send? -> TX else ->RX					 					 
				}
				if((status.txDone == true) || (status.txTimeout == true)) {
					// This must be a mistake?
					Serial.println("RF Protocol Error: txDone||txTimeout in WAITING_FOR_REPLY State");
				} 
			}
			break;
		  	 
			   
			case TX_WITHOUT_REPLY:
			{
				if(status.txDone == true){
					// Done sending
					// FIFO TX fifo !empty -> TX next, else go to RX
				}
				if(status.txTimeout == true){
					// Error in transmission
					Serial.println("RF Protocol Error: Transmission failed!");
				}
				if((status.rxDone == true) || (status.rxTimeout == true)) {
					// This must be a mistake?
					Serial.println("RF Protocol Error: rxDone||rxTimeout in TX_WITHOUT_REPLY State");
				}			  	 
			}
			break;

			case TX_WITH_REPLY:
			{
				if(status.txDone == true){
					// Done sending
					// go to WAITING_FOR_REPLY.
				}
				if(status.txTimeout == true){
					// Error in transmission
					Serial.println("RF Protocol Error: Transmission failed!");
				}
				if((status.rxDone == true) || (status.rxTimeout == true)) {
					// This must be a mistake?
					Serial.println("RF Protocol Error: rxDone||rxTimeout in TX_WITH_REPLY State");
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
 		 
	 }  
 }
 
 
bool RFProtocol::IncommingPackageHandler(){

	 RadioData *newdata = Radio->GetRadioData();
	 
	 bool replySent = false;
	 
	 if(newdata == NULL)
	 return replySent;
	 
	 ProtocolMSG_t newMessageID = (ProtocolMSG_t)newdata->payload[0];
	 
	 switch(newMessageID)
	 {
		 case MSG_Beacon_Broadcast: // Save to rxFIFO, save if not GS contact indication, 
		 {
			 Telegram_MSG_1 *msg = new Telegram_MSG_1(newdata->payload, newdata->payloadLength);
			 bool msgUnused = true;
			 if(msg->CRCValid()){
				 
				 // Save in RX FIFO:
				 if(!(rxFIFO.isFull())){
					 rxFIFO.push(msg);
					 msgUnused=false;
				 }
				 
				 if(msg->GetNumberOfSecondsSinceLastGroundStationCom() > 20){
					 bool updateComplete = false;
					 // Have we saved a beacon from this unit before? if so updated is:
					 for(int a=0;a<RFProtocolStatus.NumberOfBeaconsToRelay; a++){
						 //						Serial.print("Is memory location "+ String(a) + " from the same beacon?... ");
						 if( SavedBeacons[a]->TelegramMatchUniqueID(msg->GetUniqueID1(), msg->GetUniqueID2(), msg->GetUniqueID3(), msg->GetUniqueID4()) == true ){
							 //							Serial.println("Yes! - Updating!");
							 SavedBeacons[a] = msg;
							 updateComplete = true;
							 msgUnused=false;
							 break;
							 }else{
							 //					Serial.println("No!");
						 }
					 }
					 
					 if(updateComplete)
					 break;
					 
					 // We should save this if room
					 if(RFProtocolStatus.NumberOfBeaconsToRelay < FIFO_SIZE){
						 //			Serial.println("Message is saved in memory slot: " + String(NumberOfBeaconsToRelay));
						 SavedBeacons[RFProtocolStatus.NumberOfBeaconsToRelay] = msg;
						 RFProtocolStatus.NumberOfBeaconsToRelay++;
						 msgUnused=false;
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
			 }
			 
			 if(msgUnused == true){ // Delete msg if: not saved to rxFIFO, not saved/updated to SavedBeacons or CRC is not ok.
				 delete msg;
			 }
		 }
		 break;
		 
		 case MSG_Beacon_Relay: // Save Beacon relay to rxFiFo.
		 {
			 // Relay messages are saved in rxFIFO at the moment
			 Telegram_MSG_2 *msg = new Telegram_MSG_2(newdata->payload, newdata->payloadLength);
			 bool msgUnused = true;
			 if(msg->CRCValid()){
				 // Save in RX FIFO:
				 if(!(rxFIFO.isFull())){
					 rxFIFO.push(msg);
					 msgUnused=false;
				 }
			 }
			 if(msgUnused == true){ // Delete msg if: not saved to rxFIFO, not saved/updated to SavedBeacons or CRC is not ok.
				 delete msg;
			 }
		 }
		 break;
		 
		 case MSG_Command: // Handle Commands.
		 {
			 Telegram_MSG_3 *msg = new Telegram_MSG_3(newdata->payload, newdata->payloadLength);
			 bool msgUnused = true;
			  
			 if(msg->CRCValid()){
				 //Serial.println("Message 3 Received!...");
				 if(msg->TelegramMatchUniqueID(SystemStatus->SerialNumber1, SystemStatus->SerialNumber2, SystemStatus->SerialNumber3, SystemStatus->SerialNumber4)){
					 Serial.println("For me! - Reading command ID: " + String(msg->GetCommand()));
					 ProtocolCMD_t messageCMD = msg->GetCommand();
					 
					 // A command was received for us (reset counter for last ground station contact..  also what to do now:
					 RFProtocolStatus.SecondCounterSinceLasteGroundStationContact = 0;
					 
					 switch(messageCMD)
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
							  replySent = true;					 
						 }
						 break;

						 case CMD_Request_NEXT_Beacon_Relay:
						 {
							 Serial.println("Reply with next saved beacon if any beacons left to sent: " + String(RFProtocolStatus.NumberOfBeaconsToRelay));
							 // Reply with transponder beacon:
							 
							 if(RFProtocolStatus.NumberOfBeaconsToRelay != 0){
								 Telegram_MSG_2 msgReply = Telegram_MSG_2(SavedBeacons[RFProtocolStatus.NumberOfBeaconsToRelay-1]);
								 RFProtocolStatus.NumberOfBeaconsToRelay--;
								 Radio->SendPackage(msgReply.GetRadioMSG(), msgReply.GetRadioMSGLength());
								 replySent = true;
							 }
						 }
						 break;

						 case CMD_Do_Power_Off:
						 {
							 Serial.println("Power off");
						 }
						 break;

						 default:
						 Serial.println("Unknown command.");
						 break;
					 }
					 
					 
					 //							msg.SerialPrintMessage();
					 }else{
					 Serial.println("Not For me! :-(");
				 }
			 }
			 if(msgUnused == true){ // Delete msg if: not saved.
				delete msg;
			 }
		 }
		 break;
		 
		 default:
		 Serial.println("Unknown Incoming message! from Radio.");
		 break;
	 }
	 
	 delete newdata; // memcpy in Telegram constructor.
	 
	 return replySent; // true if reply has been started.
 }
