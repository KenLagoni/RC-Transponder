/*
	PCProtocol.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
#include "PCProtocol.h"  // For add and get function to RF protocol RX/TX FIFO

PCProtocol::PCProtocol(){
}

void PCProtocol::begin(SystemInformation_t *ptr1){
	systemData=ptr1;
}

bool PCProtocol::service(void){
	bool returnStatus=false;
	
	this->startTime = millis();
	
	do
	{
		dataReady=Serial.available();
		//Serial.println("input: " + String(dataReady) + ":");
		if(dataReady > 0)
		{
			//Serial.println(String(dataReady));
			_newChar=Serial.read(); // read char from input buffer
			Serial.write(_newChar); // echo input back.
			
			if(_newChar == 0x0D){
				// Handle command 
				firstCMDSize = inputCMD.find(" ");
				if (firstCMDSize != std::string::npos){ // string contained space, thus a command with parameter
					command = inputCMD.substr(0,firstCMDSize);					
					parameter = inputCMD.substr(firstCMDSize+1, inputCMD.length()); // the reset must be parameter
					Serial.println("");
					Serial.print("CMD:"); Serial.write(command.c_str()); Serial.println(":");
					Serial.print("Parameter:"); Serial.write(parameter.c_str()); Serial.println(":");
				}else{ // no space, just command:
					command = inputCMD;
					Serial.println("");
					Serial.print("CMD:"); Serial.write(command.c_str()); Serial.println(":");
				}
				
				inputCMD.clear(); // reset input string.
				
				returnStatus=handlecommand();	
				command.clear();
				parameter.clear();			
				
			}else{
				// Sample command
				if(_newChar != 0x0A){ // don't save newline
					inputCMD.append(&_newChar);	
					if(inputCMD.length() >= 100){
						inputCMD.clear();
						Serial.println("Input too long!");
					}
				}
			}
		}
		
		
		now = millis();
		if( (now - startTime) > 50){
			timeout=true;
		}
		
//	}while(dataReady && !timeout);
	}while(dataReady > 0);
	return returnStatus;
}


bool PCProtocol::handlecommand(void){
	// Switch on the command:
	
	//ApplicationCMD state = UNKNOWN;
	
	if(systemData==NULL){
		return false;
	}
	
	/*
	for(int a=0;a<command.length(); a++){
		int dummy=0;
		dummy=command[a];
		Serial.print("0x");
		Serial.print(String(dummy,HEX));
		Serial.print(" ");
	}*/
	
	if (command.compare("print") == 0){
		Serial.println("");
		Serial.print("Callsign:"); Serial.write(systemData->savedSettings.callsign); Serial.println(":");
		Serial.println("Beacon Interval="+String(systemData->savedSettings.BEACON_INTERVAL));
		Serial.println("GPS ON TIME="+String(systemData->savedSettings.GPS_ON_TIME));
		Serial.println("GPS OFF TIME="+String(systemData->savedSettings.GPS_OFF_TIME));
		Serial.println("Power Down Delay="+String(systemData->savedSettings.POWER_DOWN_DELAY));
	}else if(command.compare("setcallsign") == 0){
		if(parameter.length() == 0){
			Serial.println("callsign length can't be 0");
		}else if(parameter.length() > 8){
			Serial.println("callsign length max length is 8");
		}else{
			Serial.println("");
		
			for(int a=0;a<9;a++) {systemData->savedSettings.callsign[a]=0;} // clear callsign string
			
			for(int a=0;a<parameter.length();a++)
			systemData->savedSettings.callsign[a]=parameter[a];
			Serial.print("Callsign set to:"); Serial.write(systemData->savedSettings.callsign); Serial.println(":");
			return true;
		}
	}else if(command.compare("setbeaconinterval") == 0){
		Serial.println("beacon interval");
	}else if(command.compare("setgpsontime") == 0){
	}else if(command.compare("setgpsofftime") == 0){
	}else if(command.compare("help") == 0){
		Serial.println("");
		Serial.println("Use:");
		Serial.println("help - for this menu.");
		Serial.println("print - to show settings.");
		Serial.println("setcallsign [CALLSIGN] - to set the callsign. MAX 8 charactors.");
	}else{
		Serial.println("command not found");
		command.clear();
		parameter.clear();
	}	
	return false;
}								