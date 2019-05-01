/*
 * RFService.cpp
 *
 * Created: 12-Apr-19 22:55:58
 *  Author: Kenneth
 */ 

#include "RFService.h"
#include <iostream> // cout debug

#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

// File IO
#include <fstream>

//#include <boost/algorithm/string.hpp>  // boost split string

RFService::RFService(E28_2G4M20S *Radio, std::string logpath) : RFProtocol(Radio)
{
	// read all known plane from list:
	if(logpath.back() == '/')
	{
		planeListPath = logpath;
	}else
	{
		planeListPath = logpath + "/";
	}
	planeListLogFile = planeListPath + planeListFile;
	
	// Read Plane list:
	std::ifstream input(planeListLogFile.c_str());
	std::string line;

	while( std::getline( input, line ) ) {
		std::cout<< "Line:" << line<<'\n';
		
		 std::stringstream ss(line);
		 std::string token;
 		 std::string inputData[3];
		 uint8_t count = 0; 
		 while (std::getline(ss, token, ',')) {
			std::cout<< "result:" << token << '\n';
			if(count<3){
				inputData[count]=token;
				count++;
			}
		 }
		 std::cout<< "Unique ID:" << inputData[0] << ":" << '\n';
		 std::cout<< "FlightNumber:" << inputData[1] << ":" << '\n';
		 std::cout<< "Owner:" << inputData[2] << ":" << '\n';
		 
		 std::string unknownPlaneCount = std::to_string(unknownPlanes);
		 std::string  unknownPlane = "FlightNumber#" + unknownPlaneCount;
		 if(inputData[1].compare(unknownPlane) == 0)
		 {
		  	  std::cout<< "this is an unknown, counting up the counter." << '\n';
			  unknownPlanes++;
		 }
		 
		 // base64 decode using dummy telegram (telegram has base64 encode/decode):
//		 Telegram_MSG_3 msg = Telegram_MSG_3(inputData[0], CMD_Request_NEXT_Beacon_Relay);
//		 std::cout<< "UniqueIDoutput:" <<  msg.GetUniqueID() << ":" << '\n';

		 std::string planelogfile = inputData[1] + ".log";
		 Log newlog = Log(planeListPath, planelogfile, LOG_MODE_NEW_FILE_DAYLY);
		 Plane newPLane = Plane(inputData[1], inputData[2], inputData[0], newlog);	 
		 
		 // create plane from input data:
		 PlaneList.push_back(newPLane);
	}
}

void RFService::ServiceHandel()
{
	// Handel....
	this->Service();	
}

RFProtocol::RFProtocolStates_t RFService::RXHandler()
{

//	printf("Converting input data");
	Telegram *msg = this->ConvertToTelegram(Radio->GetRadioData());
//	printf("done ");

	RFProtocolStates_t returnState =RX_IDLE; 

	if(msg != nullptr)
	{
		bool SaveTelegram = false;
		bool found = false;

		switch(msg->GetRadioMSG_ID())
		{
			case MSG_Beacon_Broadcast:
				std::cout << std::endl << "UniqeID:" << msg->GetUniqueID() << ":" << std::endl;
				
				for (it = PlaneList.begin(); it != PlaneList.end(); it++)
				{
					// Access the object through iterator
					if(msg->GetUniqueID().compare(it->GetUniqueID()) == 0)
					{
						//Match	
						it->ResetActiveCounter();
						printf("Found in list - Reseting ActiveCounter!\n");
						
						if(((Telegram_MSG_1 *)msg)->GetNumberOfBeaconsToRelay() > 0)
						{
							// If the plane has transponder to relay, add cmd telegram to TX fifo.
							this->AddData(it->GetTelegramCMD(CMD_Request_NEXT_Beacon_Relay));
						}
						
						it->logDataTelegram(((Telegram_MSG_1 *)msg));
						found = true;
						break;
					}
				}		
				
				if(found == false)
				{
					//PlaneList add!
//					printf("Adding plane to list!\n");
					std::string unknownPlaneCount = std::to_string(unknownPlanes);			
					
		   		    std::string planelogfile = "FlightNumber#" + unknownPlaneCount + ".log";
					Log newlog = Log(planeListPath, planelogfile, LOG_MODE_NEW_FILE_DAYLY);
					Plane newPlane = Plane( "FlightNumber#" + unknownPlaneCount,  "Unknown Owner", msg->GetUniqueID(), newlog);
										
					// update plane list with new unknown plane:
					//newPlane.
					
					
					FILE * PlaneListFile = fopen(planeListLogFile.c_str(),"a");
					std::string outputdata = newPlane.GetUniqueID() + "," + newPlane.GetFlightNumber() + "," + newPlane.GetPlaneOwner() + "\n";
					fprintf(PlaneListFile, outputdata.c_str());  // String to file
					fclose (PlaneListFile); // must close after opening
										
					PlaneList.push_back(newPlane);
					unknownPlanes++;
					
					// write plane to file.
				}
			break;

			case MSG_Beacon_Relay:
			break;

			case MSG_Command:
			break;
			
			default:
			break;
		}		
					
		// Save in RX FIFO:
		if(!(rxFIFO.isFull()))
		{
//			SerialAUX->println("Adding MSG to RX FIFO.");	
			rxFIFO.push(msg);
		}else
		{
			delete msg;
//			SerialAUX->println("RX FIFO full!.");	
		}
		
	}else{
//		SerialAUX->println("MSG is NULL!");
	}
		
	return returnState;
}

RFProtocol::RFProtocolStates_t RFService::TXHandler(){
	Telegram *msg = nullptr;
	RFProtocolStates_t _nextState = RX_IDLE;
	
	//End fast! - don't do any more TX, main system wants to sleep.

	if(txFIFO.pop(msg)){
		if(msg!=nullptr){
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

uint32_t RFService::milliSeconds(){
	return 0;
	//return millis();
}

void RFService::PingActivePlanes()
{
	if(PlaneList.size()==0)
	{
		printf("No Planes in list");	
	}else
	{
		// go thorugh all planes in list.
		for (it = PlaneList.begin(); it != PlaneList.end(); it++)
		{
			// Access the object through iterator
//			std::cout << std::endl << "Plane in list:" << it->GetUniqueID() <<  ":" <<std::endl;
			if(it->IsActive())
			{
				it->CountDownActiveCounter(); // To ensure stop pinging if we loose contact with plane.
				this->AddData(it->GetTelegramCMD(CMD_Request_Transponder_Beacon)); // Make Ping telegram and put it in TX FIFO.
//				std::cout << std::endl << "Making Ping telegram for:"  << it->GetUniqueID() << ":" << std::endl;
			}
		}
//		std::cout << std::endl << "Done!"  << std::endl;
	}
}


