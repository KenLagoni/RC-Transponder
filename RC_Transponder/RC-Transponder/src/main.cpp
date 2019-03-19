/*
RC Transponder

Copyright (c) 2019 Lagoni
Not for commercial use

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
Clocks used in project:
After Power on reset:
Generic Clock Generator 0	- Enabled running from internal 8MHz
Generic Clock Generator 1	- Disabled
Generic Clock Generator 2	- Enabled running from internal 32Khz.
Generic Clock Generator 3-8 - Disabled

Arduino Setup will configure the clocks to:
Generic Clock Generator 0	- Enabled running 48MHz with GCG1 as reference to closed loop. (Used for USB, SERCOM, timers, GCM_EIC*, ADC, DAC)
Generic Clock Generator 1	- Enabled running from External 32KHz. (used for closed loop multiplier for GCG0) 
Generic Clock Generator 2	- Enabled running from internal 32KHz.
Generic Clock Generator 3	- Enabled running from internal 8MHz.
Generic Clock Generator 4-8 - Disabled

*If set to sleep, and GCM_EIC clock is stopped, wakeup on external interrupt is not possible. GCM_EIC should be set to running clock (32khz) or so.

*/

#include <Arduino.h>
#include <SPI.h>

// GPS NMEA decoder
#include <GPSL80Lite.h>
 
#include "wiring_private.h" // Needed for pinPeripheral function.
#include "timer.h"
#include "main.h"
#include "hw.h"

// Radio
#include "E28-2G4M20S.h"

// Radio protocol
#include "Telegram.h"
#include "Telegram_MSG_1.h"
#include "Telegram_MSG_2.h"
#include "Telegram_MSG_3.h"

// for SPORT
#include "FrSkySportSensor.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"

void HandelSerial(void);
void HandelRadio(void);
void LowPowerTest(void);
void Do_ground_station_loop(void);
float getBatteryVoltage(void);
float getInputVoltage(void);
bool BeaconService(void);
void GoToSleep(void);

//Global variables
Telegram_MSG_1 SavedBeacons[MAX_NUMBER_OF_BEACONS_TO_SAVE];
uint8_t NumberOfBeaconsToRelay = 0;
float BatteryVoltage = 0;

// FRSKY SPORT GPS
FrSkySportSensorGps FrskyGPS;                 // Create GPS sensor with default ID
FrSkySportTelemetry FrskySport;           // Create telemetry object without polling

// a SBUS object, which is on hardware  serial port 4 (SERCOM4):
//SBUS x8r(SerialSBUS);

// channel, fail safe, and lost frames data for SBUS
//uint16_t channels[16];
//bool failSafe;
//bool lostFrame;

// Object and varibels for SX1280/E28_2G4 radio chip
E28_2G4M20S *Radio = NULL;
//Telegram *msg = NULL;

// $GPGGA,193648.000,5550.0838,N,01224.0718,E,2,8,0.96,32.7,M,41.5,M,0000,0000*62
GPSL80Lite *GPS = NULL; // GPS class, defined in GPSL80.h
GpsDataLite *GPSData = NULL;  // GPGGA GPS data:

//ISR for radio
void Radio_isr(void){
//	Serial.println("Time:" + String(millis()));
  Radio->HandleIRQ(); // The radio module has something for us.
}

void setup() {
	
	pinMode(led2Pin, OUTPUT);
	digitalWrite(led2Pin, HIGH);  // Low=off, High=On
	
	do{}while(1);
			
	hwInit(); // Setup all pins according to hardware.
	
	delay(5000); // Time to get USB bootloader ready.
	
	// Init USB serial debug /setup:
	Serial.begin(115200);
	delay(500);
	Serial.println("Starting RC Transponder ver. " + String((int)FIRMWARE_VERSION) + "." + String((int)((FIRMWARE_VERSION-((int)FIRMWARE_VERSION))*100)));
	
	// Read and store the 128 bit serial number.
	ReadSerialNumberFromChipFlash();
	Serial.println("Chip unique serial number part 1:" + String(SerialNumber1));
	Serial.println("Chip unique serial number part 2:" + String(SerialNumber2));
	Serial.println("Chip unique serial number part 3:" + String(SerialNumber3));
	Serial.println("Chip unique serial number part 4:" + String(SerialNumber4));
	Serial.println("Chip unique serial number: \"" + String(SerialNumber1) + String(SerialNumber2) + String(SerialNumber3) + String(SerialNumber4)+"\"");
	/*
	for(int a = 0;a<35;a++){
		if(!((a == 22) || (a == 23))){
			//			Serial.println("PIN " + String(a) + " PULLEN = " + String(PORT->Group[g_APinDescription[a].ulPort].WRCONFIG.bit.PULLEN, HEX));
			Serial.println("PIN " + String(a) + " INEN = " + String(PORT->Group[g_APinDescription[a].ulPort].PINCFG[g_APinDescription[a].ulPin].bit.INEN, HEX));
			
		}
	}
	*/
	// Disable input buffer on pins used for output.
	PORT->Group[g_APinDescription[0].ulPort].PINCFG[g_APinDescription[0].ulPin].bit.INEN = 0;
	PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.INEN = 0;
	PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.INEN = 0;
	PORT->Group[g_APinDescription[17].ulPort].PINCFG[g_APinDescription[17].ulPin].bit.INEN = 0;
	PORT->Group[g_APinDescription[18].ulPort].PINCFG[g_APinDescription[18].ulPin].bit.INEN = 0;
	PORT->Group[g_APinDescription[20].ulPort].PINCFG[g_APinDescription[20].ulPin].bit.INEN = 0;	
	PORT->Group[g_APinDescription[21].ulPort].PINCFG[g_APinDescription[21].ulPin].bit.INEN = 0;	
	PORT->Group[g_APinDescription[32].ulPort].PINCFG[g_APinDescription[32].ulPin].bit.INEN = 0;	

				
	// GPS INIT;
	PowerONGPS();// Turn on GPS. price 31uA
	GPS = new GPSL80Lite();
	GPSData = new GpsDataLite();
	GPS->init(SerialGPS, GPSData, 57600, GPSRxPin, GPSTxPin);
		
	// FrskySPORT
	//x8r.begin(); // using SBUSSerial
	//pinPeripheral(sBUSRXPin, PIO_SERCOM_ALT); //Assign RX function to pin SBUS pin.
	
	// Init E28 Radio module (SX1280 chip): When in sleep mode (all data rtained, it costs ~70uA.
	Radio = new E28_2G4M20S(chipSelectPin,resetPin,busyPin,dio1Pin,0,0,txEnPin,rxEnPin);
	Radio->Init();
	attachInterrupt(dio1Pin, Radio_isr, RISING); // Hack in mkr1000 Variant.h to add EXTERNAL_INTERRUPT 15 on pin 30 or EXTERNAL_INT_3 on pin 25 (PCB_VERSION 11)
	
	// Init Frsky Smart port:
	SerialfrskySPort = new Uart(&sercom3, fryskySmartPortRXPin, fryskySmartPortTXPin, SERCOM_RX_PAD_3, UART_TX_PAD_2);   // Create the new UART instance for the Frsky SPORT module
	SerialfrskySPort->begin(57600);  //Baudrate for frskys SPORT protocol.
	pinPeripheral(fryskySmartPortRXPin, PIO_SERCOM_ALT); //Assign RX function to pin.
	pinPeripheral(fryskySmartPortTXPin, PIO_SERCOM_ALT); //Assign TX function to pin.
	FrskySport.begin(SerialfrskySPort, &FrskyGPS);
			
	// Init Auxiliary serial port:
	SerialAUX->begin(115200);

	// Set Timer 3 as 1 sec interrupt.
	//startTimer(1); // 1Hz
	startTimer3(1); // 1Hz
	
	state=STARTING_UP;
	/*
	if(isGroundStation){
		Serial.println("I am a groundstation");
		
	}else{
		Serial.println("I am a RC transponder");
	}
	*/
	PowerON(); // Ensure transponder keeps running from battery if external power is lost.
	PowerONGPSBackup(); // Enable backup power for GPS.
	BatteryVoltage = getBatteryVoltage();
}

void Recharge(void){
		PowerOFFGPS(); // Turn off  GPS to save battery.
		Radio->Sleep(); // Put radio to sleep to save power.
		PowerOFF(); 
		do
		{
		//	LowPower.sleep(5000);
			digitalWrite(led2Pin, HIGH);
		//	LowPower.sleep(1000);
			digitalWrite(led2Pin, LOW);
		}while(1);	
}

void loop() {
  
    // LowPowerTest();
  
	// Recharge();

	Serial.println("Battery voltage is " + String(getBatteryVoltage()) + "V  Based on:" + String(analogRead(analogVbatPin)));
	Serial.println("Input voltage is " + String(getInputVoltage()) + "V Based on:" + String(analogRead(analogVinPin)));
	Serial.println("Input 5V voltage is " + String(getInput5VVoltage()) + "V Based on:" + String(analogRead(analogVin5VPin)));

	
//	do{}while(1);
	
  /*
	if(isGroundStation){
		Do_ground_station_loop();
	}
	*/
  
	
	  
	Serial.println("Receiver ID,Transmitter ID,UTC Time,GPS Latitude,GPS Longitude,GPS Fix,Number Of Satellites,Altitude,RSSI,SNR");	  
	
	do{
		if(state == NORMAL)
		{	
			//// only every second (check status)
			while(SecondCounter){
//				Serial.println("Seccond passed!");
//				digitalWrite(led2Pin, HIGH);
				SecondCounter--;
	
				BatteryVoltage = getBatteryVoltage();
	
				// Measure all stuff here:
				if(getInputVoltage() <= 4.3){
					state=GET_READY_TO_RUN_ON_BATTERY;				
				}
		
			
				// Update the FrSky GPS emulator with the latest values from the GPS. (GPS Lite needs to be updated to read $GPRMC in order to get speed, cog and date information:
				FrskyGPS.setData(GPSData->LatitudeDecimal, GPSData->LongitudeDecimal,GPSData->Altitude,0,0,0,0,0,GPSData->UTC_hour,GPSData->UTC_min,GPSData->UTC_sec);		
				
				if(!BeaconService()){ // Transmit beacon every N second.	
					 // Enter here when no beacon is sent...
					 			
					 // Request transponder data from ID 2: (debug)
					 // Telegram_MSG_1 msg = Telegram_MSG_1(2,UNIT_ID,MSG_Transponder_Data);
					 // Radio->SendPackage(msg.Payload, msg.GetPayloadLength());
				}			

			}
	
			////// Below this line, code is executed fast! 
//			HandelSerial();	 // Communication via USB (only if used as groundstation
			HandelRadio();  // Read new messages and reply as needed.  
			GPS->update();  // Function empty serial buffer and analyzes string.
			// RCin->read();  // SBUS, PPM or PWM. 
			FrskySport.send(); // Service the Serial for SPORT. 	
			
		}else if(state == GET_READY_TO_RUN_ON_BATTERY){ // Power off GPS
			PowerONGPSBackup(); // Ensure backup power is enabled for GPS.
			PowerOFFGPS();// Turn OFF GPS.
			state=RUNNING_ON_BATTERY;
		}else if(state == RUNNING_ON_BATTERY){
			BeaconService();
			
			if(BatteryVoltage > 4.3){
				state=STARTING_UP;
			}else if(BatteryVoltage <= 3.0){
					PowerOFF(); // No more battery left, power off.
					do{}while(1);
			}else{
				do{}while(!Radio->IsIdle()); // Wait for Beacon to be transmitted before sleep.
				GoToSleep(); // Sleep until 1 sec interrupt will wake us up.		
				BatteryVoltage = getBatteryVoltage();
			}
			
		}else if(state == STARTING_UP){
			PowerONGPS();// Turn on GPS. 			
			
			// Set the radio to RX mode without timeout.
			Radio->SetRXMode(false); // No timeout
			SecondCounter=0; // reset second counter.
			state=NORMAL;
		}

	}while(1); 
  
}

void GoToSleep(void){
	Radio->Sleep(); // Put radio to sleep to save power.
	USBDevice.detach();
	digitalWrite(led2Pin, LOW);
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	USBDevice.attach();
	digitalWrite(led2Pin, HIGH);
	Radio->WakeUp(); // Wake up radio.
}

void LowPowerTest(void){

	PowerOFFGPS(); // Turn off  GPS to save battery.
	Serial.println("Battery analog reading is " + String(analogRead(analogVbatPin)));
	Serial.println("Battery voltage is " + String(getBatteryVoltage()) + "V");
	digitalWrite(led2Pin, LOW);
	do
	{
	//	Serial.println("Sleep!");	
		GoToSleep();
	//	Serial.println("Wake!");
	}while(1);

}

void Do_ground_station_loop(void){
	
	Radio->SetRXMode(false); // No timeout
	digitalWrite(led2Pin, LOW);
	
	do{
//		if(Radio->telegramValid == true){
			//Serial.println("We got a telegram!");
			digitalWrite(led2Pin, HIGH);
			//				Serial.println("We got a telegram!");
//			ReceiveTelegram = Radio->GetTelegram();
			//				PrintRadioTelegram(ReceiveTelegram); // Debug to USB serial port.
//			PrintRadioTelegramCSV(ReceiveTelegram); // Debug to USB serial port.
			delay(100);
			digitalWrite(led2Pin, LOW);
//		}else{
			//Serial.println("False!");
//		}
		delay(100);
	}while(1);
}

void HandelRadio(void){
	if(Radio->NewPackageReady()){
//		Serial.println("New Messages received!");
		// New telegram ready
		
		uint8_t size;
		uint8_t *data = Radio->GetPayload(size);
		if(data == NULL){
			Serial.println("NULL");
			return;
		}
		
		ProtocolMSG_t newMessageID = (ProtocolMSG_t)data[0];
		
//		for(int a=0;a<size;a++)
//			Serial.println("data["+String(a)+"]=" + String(data[a]));
		
//		Serial.println("Message ID="+String(newMessageID));
		switch(newMessageID)
		{
			case MSG_Beacon_Broadcast:
			{				
				Telegram_MSG_1 msg = Telegram_MSG_1(data, size);
				
				if(msg.TelegramValid()){
				
					uint8_t Serialmsg[msg.GetPayloadLength()+2];
					Serialmsg[0] = 0x1E;
					Serialmsg[1] = msg.GetPayloadLength();
					memcpy(&Serialmsg[2], msg.GetPayload(), msg.GetPayloadLength());									
					Serial.write(Serialmsg, msg.GetPayloadLength()+2);
				uint8_t *data = Serialmsg;
				for(int a =0;a<msg.GetPayloadLength()+2;a++){
					SerialAUX->println("Data["+ String(a) + "]=" + String(*data));					
					data++;
				}
				SerialAUX->println("-------------------");

		

		//			Serial.println("Message 1 Received! (Beacon)");
		//			msg.SerialPrintMessage();
										
					// We have received a beacon... Check if it has contact with ground station, if not then save it and indicate it is ready to relay.
					if(msg.GetNumberOfSecondsSinceLastGroundStationCom() > 20){
						
						bool updateComplete = false;
						
						// Have we saved a beacon from this unit before? if so updated is:
						for(int a=0;a<NumberOfBeaconsToRelay; a++){
	//						Serial.print("Is memory location "+ String(a) + " from the same beacon?... ");
						
							if( SavedBeacons[a].TelegramMatchUniqueID(msg.Unique_ID_1, msg.Unique_ID_2, msg.Unique_ID_3, msg.Unique_ID_4) == true ){
	//							Serial.println("Yes! - Updating!");	
								SavedBeacons[a] = msg;
								updateComplete = true;
								break;
							}else{
			//					Serial.println("No!");
							}													
						}
						
						if(updateComplete)
							break;
																		
						// We should save this if room 
						if(NumberOfBeaconsToRelay < MAX_NUMBER_OF_BEACONS_TO_SAVE){					
				//			Serial.println("Message is saved in memory slot: " + String(NumberOfBeaconsToRelay));						
						
							SavedBeacons[NumberOfBeaconsToRelay] = msg;
							NumberOfBeaconsToRelay++;	
						}else{
						//	Serial.println("Memory is full! - can't save message" + String(NumberOfBeaconsToRelay));						
					//		Serial.println("");
					//			Serial.println("-------------- Printing Beacon messages saved in Memory -------------");
						
							for(int a=0;a<MAX_NUMBER_OF_BEACONS_TO_SAVE; a++){
					//			Serial.println("Messages number: " + String(a));
					//			SavedBeacons[a].SerialPrintMessage();								
							}							
						}									
					}
				}
			}
			break;
			
			case MSG_Beacon_Relay:
			{
				// Don't do anything with relay messages.				
			}
			break;
		
			case MSG_Command:
			{
				Telegram_MSG_3 msg = Telegram_MSG_3(data, size);
				if(msg.TelegramValid()){
					Serial.println("Message 3 Received!...");
					if(msg.TelegramMatchUniqueID(SerialNumber1, SerialNumber2, SerialNumber3, SerialNumber4)){
						Serial.println("For me! - Reading command ID: " + String(msg.GetCommand()));
							ProtocolCMD_t cmd = msg.GetCommand();	
							
							// A command was received for us (reset counter for last ground station contact..  also what to do now:
							SecondCounterSinceLasteGroundStationContact = 0;
							
							switch(newMessageID)
							{								
								case CMD_Request_Transponder_Beacon:
								{
									Serial.println("Reply with Transponder Beacon!");
									// Reply with transponder beacon:
									Telegram_MSG_1 msgReply = Telegram_MSG_1(SerialNumber1, SerialNumber2, SerialNumber3, SerialNumber4,
																			(uint32_t)GPSData->UTCTime, GPSData->Latitude, GPSData->Longitude,
																			GPSData->NumberOfSatellites, GPSData->FixDecimal, (state==RUNNING_ON_BATTERY),
																			0, 0,
																			SecondCounterSinceLasteGroundStationContact, BatteryVoltage, FIRMWARE_VERSION, PCB_VERSION, NumberOfBeaconsToRelay);
									do{	} while(!Radio->IsIdle());	// Ensure we wait for other TX job to finish first.
									Radio->SendPackage(msgReply.GetPayload(), msgReply.GetPayloadLength());						
								}
								break;

								case CMD_Request_NEXT_Beacon_Relay:
								{
										Serial.println("Reply with next saved beacon if any beacons left to sent: " + String(NumberOfBeaconsToRelay));
										// Reply with transponder beacon:
										if(NumberOfBeaconsToRelay != 0){
											Telegram_MSG_2 msgReply = Telegram_MSG_2(&SavedBeacons[NumberOfBeaconsToRelay-1]);
											NumberOfBeaconsToRelay--;
											do{	} while(!Radio->IsIdle());	// Ensure we wait for other TX job to finish first.
											Radio->SendPackage(msgReply.GetPayload(), msgReply.GetPayloadLength());
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
				
			}
			break;		
			
			default:
				break;
		}
		Radio->SetBufferReady(false);
	}
}

// Ensure a beacon is transmitted every N second.
bool BeaconService(void){
	if(BeaconSecondCounter == 5){
		// Send a Standard beacon:
		BeaconSecondCounter =  0; // Reset Beacon counter.

		//Serial.println("Send Beacon!");

		// Make beacon msg
		float pressure=0;
		float groundspeed=0;
		
		Telegram_MSG_1 msg = Telegram_MSG_1(SerialNumber1, SerialNumber2, SerialNumber3, SerialNumber4,
											(uint32_t)GPSData->UTCTime, GPSData->Latitude, GPSData->Longitude,
											GPSData->NumberOfSatellites, GPSData->FixDecimal, (state==RUNNING_ON_BATTERY),
											pressure, groundspeed,								
											SecondCounterSinceLasteGroundStationContact, BatteryVoltage, FIRMWARE_VERSION, PCB_VERSION, NumberOfBeaconsToRelay);																					
		
		do{	} while(!Radio->IsIdle());	// Ensure we wait for other TX job to finish first.													
		
		Radio->SendPackage(msg.GetPayload(), msg.GetPayloadLength());
		return true;	
	}	
	return false;
}




enum StateMachine{
	LOOKING_FOR_START,
	READ_MSG_LENGTH,
	READ_DATA
};

#define INPUT_BUFFER_SIZE 64
StateMachine SerialState = LOOKING_FOR_START;
char _newChar;
uint8_t incommingData[INPUT_BUFFER_SIZE];
uint8_t dataLength;
uint8_t dataIndex;
int NumberOfBytesToRead;
/*
void HandelSerial(void){
	
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
				
					if(_newChar >= INPUT_BUFFER_SIZE){		
						SerialAUX->println("Length too long.");
						SerialState=LOOKING_FOR_START; // Error, restart:				
					}
					
					dataLength = _newChar;
					SerialAUX->println("Length is:" + String(dataLength));
					SerialState=READ_DATA; // Start of string found!
				break;
				
				case READ_DATA: 
					incommingData[dataIndex] = _newChar;
				//	SerialAUX->println("dataIndex[" + String(dataIndex) + "]=" + String(_newChar));
									
					if(dataIndex < INPUT_BUFFER_SIZE)
						dataIndex++;
					
					if(dataIndex >= dataLength){
					
						// Analyse input data.						
						
						SerialAUX->println("Reading Complete - Analysinging data!");
						
						for(int a=0;a<dataLength;a++){
							SerialAUX->println("Inputdata " + String(a) + ":" + String(incommingData[a]));
						}
						
						switch(incommingData[0]) // MSG ID
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
							
							case 1:
							{
								// Don't do anything with relay messages.
							}
							break;
							
							default:
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
	
}*/