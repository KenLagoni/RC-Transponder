/*
	SerialProtocol.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
#include "SerialProtocol.h"
#include "RFProtocol.h"

/*
 SerialProtocol::SerialProtocol(RFProtocol *inout){
	this->_inout = inout;
	memset(&data, 0,  DATA_BUFFER_SIZE);
 }
 */
 
 void SerialProtocol::Service(){
	do
	{
		NumberOfBytesToRead = Serial.available();
			 
		// fail fast
		if(NumberOfBytesToRead == 0)
		return;
			 
		// input data: 0x1E [LENGTH] [PAYLOAD]
			 
		if(NumberOfBytesToRead)
		{
			_newChar=Serial.read(); // read char from input buffer
				 
			switch (SerialState)
			{
				case LOOKING_FOR_START: // look for 0x1E in incoming data
				if(_newChar == 0x1E)
				{
					SerialAUX->println("Start found!");
					dataLength = 0;    // counter number of bytes read.
					dataIndex = 0;
					SerialState=READ_MSG_LENGTH; // Start of string found!
				}
				break;
					 
				case READ_MSG_LENGTH:
					 
				if(_newChar >= DATA_BUFFER_SIZE){
					SerialAUX->println("Length too long.");
					SerialState=LOOKING_FOR_START; // Error, restart:
				}
					 
				dataLength = _newChar;
				SerialAUX->println("Length is:" + String(dataLength));
				SerialState=READ_DATA; // Start of string found!
				break;
					 
				case READ_DATA:
				data[dataIndex] = _newChar;
				//	SerialAUX->println("dataIndex[" + String(dataIndex) + "]=" + String(_newChar));
					 
				if(dataIndex < DATA_BUFFER_SIZE)
				dataIndex++;
					 
				if(dataIndex >= dataLength){
					// Analyse input data.
						 
					SerialAUX->println("Reading Complete - Analysinging data!");
						 
					for(int a=0;a<dataLength;a++){
						SerialAUX->println("Inputdata " + String(a) + ":" + String(data[a]));
					}
			 
					//ProtocolMSG_t newMessageID = (ProtocolMSG_t)data[0];
					uint8_t newMessageID = data[0];
					
					switch(newMessageID) // MSG ID
					{
						case 0: // UART Identify
						{
							SerialAUX->println("Send UART ID reply message");
							// Groundstation needs a reply to indicate this devices is a ground station.
							uint8_t SerialReply[3];
							SerialReply[0] = 0x1E;
							SerialReply[1] = 1;
							SerialReply[2] = 0;
							Serial.write(SerialReply, 3);
						}
						break;
						 
						default:
							
							SerialData_t *newdata = new SerialData_t;
							memcpy(newdata->payload,data,dataIndex);
							//_inout->AddToTXFIFO(newdata);

						break;
					}
	
					SerialState=LOOKING_FOR_START; // Done, restart.
				}
				break;
					 
				default:
				SerialState = LOOKING_FOR_START;
				break;
			}
		}
	}while(NumberOfBytesToRead); // loop until buffer is empty
		 
}
