/*
	RFProtocol.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
 #include "RFProtocol.h"
 #include "hw.h" // only for SerialAUX debug

 RFProtocol::RFProtocol(E28_2G4M20S *Radio){
	this->Radio = Radio;

	Radio->Init();
	Radio->SetRXMode(false); // no timeout
	this->RFstate = RX_IDLE;
 }
 
 void RFProtocol::AddData(RadioData_t *newdata)
 {
	 // fail fast:
	if(this->txFIFO.isFull()){
		SerialAUX->println("Error! - Tx FIFO Full!");
		return;
	}

	Telegram *msg = this->ConvertToTelegram(newdata);

	if(msg!=NULL)
	{		
		this->txFIFO.push(msg);	//Add to TX FIFO
	}
}


void RFProtocol::AddData(Telegram *msg)
{
	// fail fast:
	if(this->txFIFO.isFull()){
		SerialAUX->println("Error! - Tx FIFO Full, deleting new data");
		delete msg;
		return;
	}

	if(msg!=NULL)
	{
		this->txFIFO.push(msg);	//Add to TX FIFO
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


RadioData_t * RFProtocol::GetDataIncludingRSSI()
{
	if(this->rxFIFO.isEmpty())
	{
		SerialAUX->println("Error - No data to get (rx FIFO empty)!");
		return NULL;
	}
	else
	{
		Telegram * msg = NULL;
		rxFIFO.pop(msg);
		memset(&rxbuffer.payload, 0 , MAX_PAYLOAD_LENGTH);
		memcpy(&rxbuffer.payload, msg->GetRadioData()->payload, msg->GetRadioData()->payloadLength);	// copy the data.
		rxbuffer.payload[msg->GetRadioData()->payloadLength] =(uint8_t)(msg->GetRadioData()->rssi);
		rxbuffer.payload[msg->GetRadioData()->payloadLength+1] = (uint8_t)(msg->GetRadioData()->snr);
		rxbuffer.payloadLength = msg->GetRadioData()->payloadLength+2;
		delete msg;
		return &rxbuffer;
	}
}


 int RFProtocol::Available()
 {
	 return this->rxFIFO.size();
 }


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

 
void RFProtocol::_PowerDown(){
	// Wait for Radio to finish current transmission.

	do{
		this->Service();
		}while(this->RFstate!=RX_IDLE);

	Radio->Sleep();
}		

void RFProtocol::_WakeUp(){
	Radio->WakeUp();
	Radio->SetRXMode(false); // Set Radio to RX mode with timeout. This will trigger rx.timeout, and thus the continues flow of the statemachine, should there be data left in TX fifo.
}

void RFProtocol::Service(){

	if(Radio->GetDioPinStatus() == HIGH){
		SerialAUX->println("IRQ from Radio");	
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
					digitalWrite(led2Pin, HIGH);
 					nextState=TXHandler(); // Returns TX_WITHOUT_REPLY || TX_WITH_REPLY || RX_IDLE
					digitalWrite(led2Pin, LOW);

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
//				SerialAUX->println("RF Protocol Error: txDone||txTimeout in WAITTING_FOR_REPLY State - Nextstate set to RX_IDLE");
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
						if(milliSeconds() > this->timeoutStart+TIMEOUT_MS){
//							SerialAUX->println("Software timeout on WAITING_FOR_REPLY");
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
//				SerialAUX->println("RF Protocol Error: rxDone||rxTimeout||txTimeout in TX_WITHOUT_REPLY State");
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
					if(milliSeconds() > this->timeoutStart+TIMEOUT_MS){
//						SerialAUX->println("Software timeout on TX_WITHOUT_REPLY");
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
//				SerialAUX->println("RF Protocol Error: rxDone||rxTimeout||txTimeout in TX_WITHOUT_REPLY State");
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
					if(milliSeconds() > this->timeoutStart+TIMEOUT_MS){
//						SerialAUX->println("Software timeout on TX_WITHOUT_REPLY");
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
//			SerialAUX->println("Error! - Default!");
			Radio->SetRXMode(false); // Set RX without.
		break;
	}
	
	if(this->RFstate != nextState){
		this->timeoutStart = milliSeconds(); // for software timeout.
//		SerialAUX->println("New state! - RFSate:" +String(RFstate) + " NextState:" + String(nextState));
		this->RFstate=nextState;
	}
}
