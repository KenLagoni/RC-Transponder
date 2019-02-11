#include "GPSL80Lite.h"
#include "Arduino.h" // Needed for Serial.print
#if defined (__AVR__) || (__avr__)
	#include <SoftwareSerial.h>
#elif defined(ARDUINO_SAMD_MKRZERO)  // Arduino MKR Zero
	#include "wiring_private.h" // need for pinPeripheral when running on SAMD ARM (Arduino MKRZero)
#endif
#include <stdlib.h>

/*
Uart::Uart(SERCOM *_s, uint8_t _pinRX, uint8_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX) :
Uart(_s, _pinRX, _pinTX, _padRX, _padTX, NO_RTS_PIN, NO_CTS_PIN)
{
}

Uart::Uart(SERCOM *_s, uint8_t _pinRX, uint8_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX, uint8_t _pinRTS, uint8_t _pinCTS)
{
*/

GPSL80Lite::GPSL80Lite()
{
}
	#if defined (__AVR__) || (__avr__)
		void GPSL80Lite::init(SoftwareSerial *ptr1, GpsDataLite *ptr2, unsigned long baudrate, int rxPin , int txPin)
	#elif defined(ARDUINO_SAMD_MKRZERO)  // Arduino MKR Zero
		void GPSL80Lite::init(Uart *ptr1, GpsDataLite *ptr2, unsigned long baudrate, int rxPin , int txPin)
	#endif
{
	SerialGPS = ptr1;
	dataOut =  ptr2;
	
	// Reset data:
	uint8_t i=0;

	dataOut->DataIsValid = false;
	
	// Setup and configure GPS
	SerialGPS->begin(9600); // 9600 is default baud after power cycle.. which we don't do here but assumes had been done.
	
	#if defined(ARDUINO_SAMD_MKRZERO)  // Arduino MKR Zero
		pinPeripheral(txPin, PIO_SERCOM_ALT); //Assign TX function to GPS TX pin.
		pinPeripheral(rxPin, PIO_SERCOM_ALT); //Assign RX function to GPS RX pin.
	#endif
		
	delay(1000);
	SerialGPS->println("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"); //Setup the GPS for only GPGGA output at 1Hz.
	delay(1000);
	
	 switch(baudrate){
		 case 9600:
			// We are allready running at 9600.
		 	//SerialGPS->println("$PMTK251,0*28");        //Setup GPS serial speed to default 9600.			 
			 delay(1000);
			 break;

		 case 19200:
		    SerialGPS->println("$PMTK251,19200*22");    //Setup GPS serial speed for 19200. (This gives us less idle time thus saving power).			
			delay(1000);
			SerialGPS->end();
			SerialGPS->begin(19200);  // GPS module uses 19200 as standard speed.*/
		 break;

		 case 38400:
			SerialGPS->println("$PMTK251,38400*27");    //Setup GPS serial speed for 38400. (This gives us less idle time thus saving power).This looks like i is too fast for the software serial on 8MMz.			
			delay(1000);
			SerialGPS->end();
			SerialGPS->begin(38400);  // GPS module uses 38400 as standard speed.*/

		 break;		
		  
		 case 57600:
			SerialGPS->println("$PMTK251,57600*2C");     //Setup GPS serial speed for 57600. (This gives us less idle time thus saving power). This looks like i is too fast for the software serial on 8MMz.			
			delay(1000);
			SerialGPS->end();
			SerialGPS->begin(57600);  // GPS module uses 57600 as standard speed.*/
		 break;
 
		 default:
		 break;
	 }

	#if defined(ARDUINO_SAMD_MKRZERO)  // Arduino MKR Zero
		pinPeripheral(txPin, PIO_SERCOM_ALT); //Assign TX function to GPS TX pin.
		pinPeripheral(rxPin, PIO_SERCOM_ALT); //Assign RX function to GPS RX pin.
	#endif	
	
	resetTempData();
	updateData();	
}

void GPSL80Lite::loopback()
{
	if(SerialGPS->available()){
	#if defined (__AVR__) || (__avr__)
		Serial.write(SerialGPS->read());  
	#elif defined(ARDUINO_SAMD_MKRZERO)  // Arduino MKR Zero
		SerialUSB.write(SerialGPS->read());  
	#endif
    
    }
}

/*
uint8_t GPSL80Lite::getChecksum(String *str)
{
	uint8_t i, crc = 0;
		
	for (i = 0; i < (str->length()) ; i++)
	{
		crc = crc ^ ((uint8_t)str->charAt(i));  
	}
  return crc;
}
*/
uint8_t GPSL80Lite::charToInt(char input)
{
	uint8_t dummy=(char)input;
	
	if(dummy > 57) // Must be A-F
	{
		dummy = dummy - 55;		
	}
	else // if dummy < 57 then it must be 0-9 
	{
		dummy = dummy - 48;
	}	
		
	return dummy;
}

void GPSL80Lite::resetTempData()
{
	uint8_t a = 0;

	tempData.UTCTime=0;	
	for(a=0;a<MAX_LAT_SIZE;a++){
		tempData.LatitudeDEG[a] = 0;
	}	
	for(a=0;a<MAX_LONG_SIZE;a++){
		tempData.LongitudeDEG[a] = 0;
	}
	tempData.Latitude = 0;
	tempData.Longitude = 0;
	tempData.LatitudeDIR='*';
	tempData.LongitudeDIR='*';
	tempData.Fix = '*';
	tempData.FixDecimal = 0;
	tempData.NumberOfSatellites = 0;
	tempData.HDOP = 0;
	tempData.Altitude = 0;
	tempData.AltitudeUnit = '*';
	tempData.LatitudeDecimal = 0;
	tempData.LongitudeDecimal = 0;
}

void GPSL80Lite::updateData()
{
	//	130532
	uint8_t a = 0;
	dataOut->UTCTime=tempData.UTCTime;
	uint32_t secconds;
	uint32_t minutes;
	uint32_t hours;
		
	hours = tempData.UTCTime / 10000;
	minutes = tempData.UTCTime / 100 - hours*100;
	secconds = tempData.UTCTime - hours*10000 - minutes*100;
	dataOut->UTC_sec = (uint8_t)secconds;
	dataOut->UTC_min = (uint8_t)minutes;
	dataOut->UTC_hour = (uint8_t)hours;
		
	for(a=0;a<MAX_LAT_SIZE;a++){
		dataOut->LatitudeDEG[a]=tempData.LatitudeDEG[a];
	}
	for(a=0;a<MAX_LONG_SIZE;a++){
		dataOut->LongitudeDEG[a]=tempData.LongitudeDEG[a];
	}

	/*
	tempData.LatitudeDEG[0] = '6';
	tempData.LatitudeDEG[1] = '5';
	tempData.LatitudeDEG[2] = '2';
	tempData.LatitudeDEG[3] = '3';
	tempData.LatitudeDEG[4] = '.';
	tempData.LatitudeDEG[5] = '1';
	tempData.LatitudeDEG[6] = '2';
	tempData.LatitudeDEG[7] = '3';
	tempData.LatitudeDEG[8] = '4';
	tempData.LatitudeDIR = 'S';
	*/
/*
	Serial.println();
	Serial.println(":" + String(tempData.LatitudeDEG) + ":");
	*/
	int32_t factor = 10000000;
	if(tempData.LatitudeDEG[0] != 0)
	{
		dataOut->Latitude = 0;
		// Convert the Latitude*1000 to int32 (no decimal).
		for(a=0;a<MAX_LAT_SIZE;a++){
			if(tempData.LatitudeDEG[a] != '.'){ // Don't convert the dot!
				//			Serial.println("Input for a=" + String(a) + " :" + String(tempData.LatitudeDEG[a])+":");
				//		    Serial.println("Factor=" + String(factor) );
				dataOut->Latitude += charToInt(tempData.LatitudeDEG[a]) * factor;
				//			Serial.println("Output for a=" + String(a) + " :" + String(dataOut->Latitude)+":");
				factor /= 10;
			}
		}
		if(tempData.LatitudeDIR == 'S'){
			dataOut->Latitude *= -1;
		}

	}
	float Degrees = (float)(dataOut->Latitude / 1000000);
	float Minutes = ((float)(dataOut->Latitude) / 10000) - (Degrees * 100);
	dataOut->LatitudeDecimal = Degrees + Minutes/60;
	/*
	tempData.LongitudeDEG[0] = '1';
	tempData.LongitudeDEG[1] = '8';
	tempData.LongitudeDEG[2] = '0';
	tempData.LongitudeDEG[3] = '2';
	tempData.LongitudeDEG[4] = '2';
	tempData.LongitudeDEG[5] = '.';
	tempData.LongitudeDEG[6] = '1';
	tempData.LongitudeDEG[7] = '2';
	tempData.LongitudeDEG[8] = '3';
	tempData.LongitudeDEG[9] = '4';
	tempData.LongitudeDIR = 'W';
	*/
	if(tempData.LongitudeDEG[0] != 0)
	{
		factor = 100000000;
		dataOut->Longitude = 0;
		// Convert the Latitude*1000 to int32 (no decimal).
		for(a=0;a<MAX_LONG_SIZE;a++){
			if(tempData.LongitudeDEG[a] != '.'){ // Don't convert the dot!
//							   Serial.println("Input for a=" + String(a) + " :" + String(tempData.LongitudeDEG[a])+":");
//							   Serial.println("Factor=" + String(factor) );
				dataOut->Longitude += charToInt(tempData.LongitudeDEG[a]) * factor;
				 			//Serial.println("Output for a=" + String(a) + " :" + String(dataOut->Longitude)+":");
				factor /= 10;
			}
		}
		if(tempData.LongitudeDIR == 'W'){
			dataOut->Longitude *= -1;
		}
	}
	
	
	
	 //5550.0898N -> 55500898 ||  5550.0898S -> -55500898
	/*
	
	GPS:
	http://www.latlong.net/c/?lat=55.808648&long=12.511684

	To convert Seconds to decimals, divide by 3600.
	Decimal value = Seconds/3600
	As an example, a latitude of 122 degrees 45 minutes 45 seconds North is equal to 122.7625 degrees North. (If it is S (south) then multiply by -1, same goes for )

	So the complete formula looks similar the following:
	Decimal value = (Degrees + (Minutes/60) + (Seconds/3600) )(*-1 if S or W)
	*/
	// 01224.0718E -> 12240718 || 01224.0718W -> -12240718
//	Serial.println("Longitude uint: " + String(dataOut->Longitude));	
	Degrees = (float)(dataOut->Longitude / 1000000);
//	Serial.println("");
//	Serial.println("Degrees: " + String(Degrees,4));	
	Minutes = ((float)(dataOut->Longitude) / 10000) - (Degrees * 100);
//	Serial.println("Minutes: " + String(Minutes,4));
//	
//				   -12
//					   -1224.0718 + 1200 = 1200 - 1224.0718 = -24,0718 	
	
	dataOut->LongitudeDecimal = Degrees + Minutes/60;
//	Serial.println("Longitude Decimal: " + String(dataOut->LongitudeDecimal,6));	
//	do{}while(1);
	
	/*
	Serial.println(":" + String(tempData.LatitudeDEG) + ":");
	Serial.println(":" + String(tempData.LongitudeDEG) + ":");
	Serial.println(":" + String(tempData.LatitudeDIR) + ":");
	Serial.println(":" + String(tempData.LongitudeDIR) + ":");
	Serial.println(":" + String(dataOut->Latitude) + ":");
	Serial.println(":" + String(dataOut->Longitude) + ":");
	Serial.println("");*/

	dataOut->LatitudeDIR=tempData.LatitudeDIR;
	dataOut->LongitudeDIR=tempData.LongitudeDIR;
	dataOut->Fix=tempData.Fix;
	dataOut->FixDecimal=tempData.FixDecimal;
	dataOut->NumberOfSatellites=tempData.NumberOfSatellites;
	dataOut->HDOP=tempData.HDOP;
	dataOut->Altitude=tempData.Altitude;
	dataOut->AltitudeUnit=tempData.AltitudeUnit;
	dataOut->DataIsValid=true;
}

void GPSL80Lite::printGPSData(void)
{
	Serial.print("Time         :");
	Serial.print(String(dataOut->UTCTime, 0));
	Serial.println(":");

	Serial.print("Lat DEG      :");
	Serial.print(String(dataOut->LatitudeDEG));
	Serial.println(":");

	Serial.print("Lat Dir      :");
	Serial.print(dataOut->LatitudeDIR);
	Serial.println(":");

	Serial.print("Long DEG     :");
	Serial.print(String(dataOut->LongitudeDEG));
	Serial.println(":");

	Serial.print("Long Dir     :");
	Serial.print(dataOut->LongitudeDIR);
	Serial.println(":");

	Serial.print("Fix          :");
	Serial.print(dataOut->Fix);
	Serial.println(":");
	
	Serial.print("Number Of Sat:");
	Serial.print(String(dataOut->NumberOfSatellites));
	Serial.println(":");
	
	Serial.print("HDOP         :");
	Serial.print(String(String(dataOut->HDOP)));
	Serial.println(":");
	
	Serial.print("Altitude     :");
	Serial.print(String(String(dataOut->Altitude)+dataOut->AltitudeUnit));
	Serial.println(":");
}

bool GPSL80Lite::inputValid(char input)
{
	if((input == '\r') || (input == '$')) 
			// If a '\r' is detected at this point we must restart.
			// If a '$' is detected at this point we must restart.
	{
		if( ReadErrors < 255)
			ReadErrors++;
		state=LOOKING_FOR_START; // Go back to start							
		return 0;
	}
	return 1;		
}

void GPSL80Lite::update()
{
	do
	{
		dataReady=SerialGPS->available();
		
		if(dataReady)
		{
			_newChar=SerialGPS->read(); // read char from input buffer
//			Serial.print(_newChar);
//			data[byteNumber++] = _newChar; // Save the incomming data to buffer.
								
			switch (state)
			{
				case LOOKING_FOR_START: // look for $ in incomming data
					if(_newChar == '$')
					{
						dataLength = 0;    // counter number of bytes read.
						byteNumber = 0;    // Reset the pointer in data
						CRC=0;			   // Reeat CRC calculation for new data.
						resetTempData();
								
						state=READ_HEADER; // Start of string found!						
//						Serial.println("MAX UTC SIZE:" + String(MAX_UTC_DATA_SIZE));
					}
				break;

				case READ_HEADER: // saving header while input is not ','				
					dataLength++;
					CRC = CRC ^ uint8_t(_newChar); // Calculate CRC

					// only search for GPGGA Header		
					//                $GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C 			
					switch (dataLength)
					{
						case 1: 
							if(_newChar == 'G')
								state = READ_HEADER;
							else
								state = LOOKING_FOR_START;
						break;
						
						case 2: 
							if(_newChar == 'P')
								state = READ_HEADER;
							else
								state = LOOKING_FOR_START;
						break;
						
						case 3: 
							if(_newChar == 'G')
								state = READ_HEADER;
							else
								state = LOOKING_FOR_START;
						break;
						
						case 4: 
							if(_newChar == 'G')
								state = READ_HEADER;
							else
								state = LOOKING_FOR_START;
						break;

						case 5: 
							if(_newChar == 'A')
								state = READ_HEADER;
							else
								state = LOOKING_FOR_START;					
						break;

						case 6: 
							if(_newChar == ','){
								dataLength=0;
								state = READ_UTC_DATA; 
							}
							else
								state = LOOKING_FOR_START;					
						break;
						
						default: 
							state = LOOKING_FOR_START;
						break;
					}				
				break;
		
				case READ_UTC_DATA: 
					dataLength++;

					if(inputValid(_newChar))
					{
						CRC = CRC ^ uint8_t(_newChar); // Calculate CRC
						if(_newChar == '.')
						{
							state=WAIT_FOR_NEXT_PARAMETER;
						}else
						{ // if(dataLength <= MAX_UTC_DATA_SIZE)
//							Serial.println("dataLength:" + String(dataLength) + " Newchar:" + _newChar);
//							data2->UTCTime[dataLength-1]=_newChar;
//							tempData.UTCTime[dataLength-1]=_newChar;
							tempData.UTCTime=tempData.UTCTime * 10 + charToInt(_newChar);								
						}
//							Serial.println("UTCTime2:" + String(tempData.UTCTime2));
					}
				break;		
				
				case WAIT_FOR_NEXT_PARAMETER: 
					CRC = CRC ^ uint8_t(_newChar); // Calculate CRC
					if(_newChar == ',') 
					{
						dataLength=0;
						state=READ_LAT_DATA;
					}						
				break;	
				
				
				case READ_LAT_DATA: 
					dataLength++;					
					if(inputValid(_newChar))
					{
						CRC = CRC ^ uint8_t(_newChar); // Calculate CRC
						if(_newChar == ',')
						{
							dataLength=0;
							state=READ_LAT_DIR_DATA;
						}else if(dataLength <= MAX_LAT_SIZE)
						{ 	
							tempData.LatitudeDEG[dataLength-1]=_newChar;
						}	
					}					
				break;
		
				case READ_LAT_DIR_DATA: 
					dataLength++;				
					if(inputValid(_newChar))
					{
						CRC = CRC ^ uint8_t(_newChar); // Calculate CRC			
						if(_newChar == ',')
						{
							dataLength=0;
							state=READ_LONG_DATA;
						}
						else if(dataLength == 1){
							//data2->LatitudeDIR=_newChar;			
							tempData.LatitudeDIR=_newChar;
						}
					}					
				break;
				
				case READ_LONG_DATA: 
					dataLength++;					
					if(inputValid(_newChar))
					{
						CRC = CRC ^ uint8_t(_newChar); // Calculate CRC
						if(_newChar == ',')
						{
							dataLength=0;
							state=READ_LONG_DIR_DATA;
						}else if(dataLength <= MAX_LONG_SIZE){ 	
							tempData.LongitudeDEG[dataLength-1]=_newChar;
						}	
					}						
				break;
				
				case READ_LONG_DIR_DATA: 
					dataLength++;
					if(inputValid(_newChar))
					{
						CRC = CRC ^ uint8_t(_newChar); // Calculate CRC
						if(_newChar == ',')
						{
							dataLength=0;
							state=READ_FIX_DATA;
						}
						else if(dataLength == 1)
//							data2->LongitudeDIR=_newChar;			
							tempData.LongitudeDIR=_newChar;
					}
				break;
				
				case READ_FIX_DATA: 
					dataLength++;
					if(inputValid(_newChar))
					{
						CRC = CRC ^ uint8_t(_newChar); // Calculate CRC				
						if(_newChar == ',')
						{
							dataLength=0;
							state=READ_SAT_DATA;
						}
						else if(dataLength == 1){
							//data2->Fix=_newChar;
							tempData.Fix=_newChar;	
							tempData.FixDecimal = charToInt(_newChar);			
						}
					}					
				break;
				
				case READ_SAT_DATA: 
					dataLength++;
					if(inputValid(_newChar))
					{
						CRC = CRC ^ uint8_t(_newChar); // Calculate CRC						
						if(_newChar == ',')
						{
							dataLength=0;
							decimal=0;
							state=READ_HDOP_DATA;
						}
						else if(dataLength == 1){
							tempData.NumberOfSatellites=charToInt(_newChar);
						}else{
							tempData.NumberOfSatellites = tempData.NumberOfSatellites * 10 + charToInt(_newChar);
						}
					}					
				break;
				
				case READ_HDOP_DATA: 
					dataLength++;
					if(inputValid(_newChar))
					{
						CRC = CRC ^ uint8_t(_newChar); // Calculate CRC
						if(_newChar == ',')
						{
							dataLength=0;
							decimal=0;
							state=READ_ALITUDE_DATA;
						}else if(_newChar == '.'){				
							decimal=10;
						}else if(_newChar == '-'){				
							tempData.HDOP=-1;
						}else{
							if(decimal == 0){
								tempData.HDOP = tempData.HDOP * 10 + charToInt(_newChar);
							}else if(decimal <= 100){
								tempData.HDOP = tempData.HDOP + ((float)charToInt(_newChar) / (float)decimal);
								if(decimal != 100)
									decimal = decimal * 10;
							}
						}
					}					
				break;
				
				case READ_ALITUDE_DATA: 
					dataLength++;
					if(inputValid(_newChar))
					{
						CRC = CRC ^ uint8_t(_newChar); // Calculate CRC
						if(_newChar == ',')
						{
							dataLength=0;
							state=READ_ALITUDE_UNIT_DATA;
						}else if(_newChar == '.'){				
							decimal=10;
						}else if(_newChar == '-'){				
							tempData.Altitude=-1;
						}else{
							if(decimal == 0){
								tempData.Altitude = tempData.Altitude * 10 + charToInt(_newChar);
							}else if(decimal <= 100){
								tempData.Altitude = tempData.Altitude + ((float)charToInt(_newChar) / (float)decimal);
								if(decimal != 100)
									decimal = decimal * 10;
							}
						}
					}					
				break;
				
				case READ_ALITUDE_UNIT_DATA: 
					dataLength++;
					if(inputValid(_newChar))
					{
						CRC = CRC ^ uint8_t(_newChar); // Calculate CRC						
						if(_newChar == ',')
						{
							dataLength=0;
							state=WAIT_FOR_CRC;
						}else if(dataLength == 1){
							tempData.AltitudeUnit=_newChar;				
						}
					}					
				break;
				
				case WAIT_FOR_CRC: 
					dataLength++;
					if(inputValid(_newChar))
					{
						if(_newChar == '*')
						{
							dataLength=0;
							CRC_RESULT = 0;
							state=READ_CRC_1;
						}else{
							CRC = CRC ^ uint8_t(_newChar); // Calculate CRC
						}
					}					
				break;
				
				case READ_CRC_1: // read first CRC char					
 					CRC_RESULT += charToInt(_newChar)*16;
					state=READ_CRC_2;
				break;

				
				case READ_CRC_2: // read last CRC char and perform CRC check
					CRC_RESULT += charToInt(_newChar);
		
					if(CRC_RESULT == CRC) // check if CRC is ok.
					{
//						Serial.print("CRC is ok!");
						updateData();							
					}
					else
					{
						if(CRCErrors < 255)
							CRCErrors++;
						
					}
					
					state = LOOKING_FOR_START;
				break;
				
				default: 
					state = LOOKING_FOR_START;
				break;
			}
		}			
	}while(dataReady); // loop until buffer is empty	
	
}



/*

					if((dataLength > MAX_HEADER_SIZE) || (_newChar == '\r') || (_newChar == '$')) 
									// If header is to long, we assume that we missed a ',' thus restart. 
									// If a '\r' is detected at this point we must restart.
									// If a '$' is detected at this point we must restart.
					{
						if(ReadErrors < 255)
							ReadErrors++;
						//Serial.println("Error in state READ_HEADER!");
						state=LOOKING_FOR_START; // Go back to start					
					}
					else if(_newChar == ',') // end of header read
					{			
						dataLength=0;    // reset coutner for next state.				
						
						state=READ_DATA; // go to next state.			
					}
					else
					{
//						results[0]+=_newChar; // Add input char to result
//						_incommingData +=_newChar; // _incommingData is later used for CRC check.
					}
*/