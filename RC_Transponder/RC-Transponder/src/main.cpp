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
#include "Telegram_MSG_4.h"

// for SPORT
#include "FrSkySportSensor.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"

void HandelRadio(void);
void LowPowerTest(void);
void Do_ground_station_loop(void);
float getBatteryVoltage(void);
float getInputVoltage(void);

//Global variables

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
  Radio->HandleIRQ(); // The radio module has something for us.
}

void setup() {
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
	attachInterrupt(dio1Pin, Radio_isr, RISING); // Hack in mkr1000 Variant.h to add EXTERNAL_INTERRUPT 15 on pin
	
	// Init Frsky Smart port:
	SerialfrskySPort = new Uart(&sercom3, fryskySmartPortRXPin, fryskySmartPortTXPin, SERCOM_RX_PAD_3, UART_TX_PAD_2);   // Create the new UART instance for the Frsky SPORT module
	SerialfrskySPort->begin(57600);  //Baudrate for frskys SPORT protocol.
	pinPeripheral(fryskySmartPortRXPin, PIO_SERCOM_ALT); //Assign RX function to pin.
	pinPeripheral(fryskySmartPortTXPin, PIO_SERCOM_ALT); //Assign TX function to pin.
	FrskySport.begin(SerialfrskySPort, &FrskyGPS);
			
	// Init Auxiliary serial port:
//	SerialAUX = new Uart(&sercom5, auxRXPin, auxTXPin, SERCOM_RX_PAD_3, UART_TX_PAD_2);   // Create the new UART instance for the AUX serial port.
//	SerialAUX->begin(19200);


	// Set Timer 3 as 1 sec interrupt.
	//startTimer(1); // 1Hz
	startTimer3(1); // 1Hz
	
	state=STARTING_UP;
	
	if(isGroundStation){
		Serial.println("I am a groundstation");
		
	}else{
		Serial.println("I am a RC transponder");
	}
	
	PowerON(); // Ensure transponder keeps running from battery if external power is lost.

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
  
	if(isGroundStation){
		Do_ground_station_loop();
	}
	
	// Set the radio to RX mode without timeout.
	Radio->SetRXMode(false); // No timeout
	  
	do{
		//// only every second (check status)
		while(SecondCounter){
//			Serial.println("Seccond passed!");
//			digitalWrite(led2Pin, HIGH);
			SecondCounter--;
	
			// Measure all stuff here:
//			InputVoltage=getInputVoltage();
//			BatteryVoltage=getBatteryVoltage();		
			
			// Update the FrSky GPS emulator with the latest values from the GPS. (GPS Lite needs to be updated to read $GPRMC in order to get speed, cog and date information:
			FrskyGPS.setData(GPSData->LatitudeDecimal, GPSData->LongitudeDecimal,GPSData->Altitude,0,0,0,0,0,GPSData->UTC_hour,GPSData->UTC_min,GPSData->UTC_sec);		
					
			if(BeaconSecondCounter == 20){
				// Send a Standard beacon:
				BeaconSecondCounter =  0; // Reset Beacon counter.

				// Make beacon msg
				Telegram_MSG_4 msg = Telegram_MSG_4(UNIT_ID, (uint32_t)GPSData->UTCTime, GPSData->Latitude, GPSData->Longitude,0);
				Radio->SendPackage(msg.Payload, msg.GetPayloadLength());
			}
			else{
				// Request transponder data from ID 2: (debug)
				Telegram_MSG_1 msg = Telegram_MSG_1(2,UNIT_ID,MSG_Transponder_Data);
				Radio->SendPackage(msg.Payload, msg.GetPayloadLength());
			}			

		}
	
	////// Below this line code is executed fast 
		HandelRadio();  // Read new messages and reply as needed.  
		GPS->update();  // Function empty serial buffer and analyzes string.
		// RCin->read();  // SBUS, PPM or PWM. 
		FrskySport.send(); // Service the Serial for SPORT. 
	
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
		
		ProtocolMSG_t newMessageID = (ProtocolMSG_t)data[4];
		
		//for(int a=0;a<size;a++)
		//	Serial.println("data["+String(a)+"]=" + String(data[a]));
		
//		Serial.println("Message ID="+String(newMessageID));
		switch(newMessageID)
		{
			case MSG_Request:
				{
					Telegram_MSG_1 msg = Telegram_MSG_1(data, size);
					if(msg.GetMsgRequest() == MSG_Transponder_Data){
						// A Transponder Data telegram has been requested... reply!
						Telegram_MSG_2 msgReply = Telegram_MSG_2(msg.From,UNIT_ID,GPSData->UTCTime, GPSData->Latitude, GPSData->Longitude,GPSData->NumberOfSatellites,GPSData->FixDecimal,GPSData->Altitude);
						Radio->SendPackage(msgReply.Payload, msgReply.GetPayloadLength());	
					}	
				}
				break;		

			case MSG_Transponder_Data:
				{
					Serial.println("Message 2 Received! (Transponder Data)");
					Telegram_MSG_2 msg = Telegram_MSG_2(data, size);			
					msg.SerialPrintMessage();	
				}
				break;

			case MSG_Transponder_Status:
				break;

			case MSG_Beacon_Broadcast:
				{
					Serial.println("Message 4 Received! (Beacon)");
					Telegram_MSG_4 msg = Telegram_MSG_4(data, size);
					//msg.SerialPrintMessage();	
				}
				break;

			case MSG_Relay_Request:
				break;

			case MSG_Relay_Reply:
				break;
			
			default:
				break;
		}
		Radio->SetBufferReady(false);
	}
}

/*

RadioTelegram_t LastGoodRadioTelegram;
void BuildPayload(void){

	RadioTelegram.receiver_ID = 0;
	RadioTelegram.transmitter_ID = 100;
	RadioTelegram.UTCTime = (uint32_t)GPSData->UTCTime;
	RadioTelegram.Latitude = GPSData->Latitude;
	RadioTelegram.Longitude = GPSData->Longitude;
	RadioTelegram.PCBVersion = PCB_VERSION;
	RadioTelegram.FirmwwareVersion = FIRMWARE_VERSION;
	RadioTelegram.NumberOfSatellites = GPSData->NumberOfSatellites;
	RadioTelegram.Altitude = GPSData->Altitude;
	
	LastGoodRadioTelegram=RadioTelegram;
}
*/
/*
void BuildRadioTelegram(void){

		// check if new GPS data has fix, else send last known GPS coordinate
		 switch(GPSData->Fix){
			 case '0':
			 RadioTelegram.Fix = 0;
			 /*
			  // Use last good telegram
			  if(LastGoodRadioTelegram.Fix == 0){
			  	RadioTelegram.Fix = 0;
			  	BuildPayload(); // if both the old and the new has fix 0, then use new.
			  }else{
				RadioTelegram=LastGoodRadioTelegram; // If old GPS has better fix use that.
			  }
			  *//*
			  break;
			 			
			 case '1':		
			  // if new GPS coordinate is only fix 1 and old is fix 2, use old.
			  /*
			  if(LastGoodRadioTelegram.Fix == 2){
				RadioTelegram=LastGoodRadioTelegram;			  	 
			  }else{ // same fix type or old is lower, use new.
			  	RadioTelegram.Fix = 1;
				BuildPayload();
			  }
			  *//*
			  RadioTelegram.Fix = 1;
		      break;
			  
			 case '2': //New GPS has fix type 2, always use this.
 			  	RadioTelegram.Fix = 2;
//				BuildPayload();
			  break;

			 default:
			  break;
		 }
		 
		BuildPayload();
		 //Always update the battery voltage:
		 RadioTelegram.BatteryVoltage = BatteryVoltage;
}



void PrintRadioTelegram(RadioTelegram_t *telegram){

	String dummy;
	Serial.println("Telegram printout:");	 
	Serial.println("Receiver ID  | Transmitter ID  |   UTC Time  | GPS Latitude  | GPS Longitude | GPS Fix | PCB Version | Firmware Version  |  Number Of Satellites |  Battery Voltage |  Altitude |  RSSI |");
	dummy = String(telegram->receiver_ID);
	for(int a=0; a<(14/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(14/2); a++){Serial.print(" ");}
	
	dummy = String(telegram->transmitter_ID);
	for(int a=0; a<(18/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(18/2); a++){Serial.print(" ");}

	dummy = String(telegram->UTCTime);
	for(int a=0; a<(14/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(14/2); a++){Serial.print(" ");}

	dummy = String(telegram->Latitude);
	for(int a=0; a<(16/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(16/2); a++){Serial.print(" ");}

	dummy = String(telegram->Longitude);
	for(int a=0; a<(16/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(16/2); a++){Serial.print(" ");}
	
	dummy = String(telegram->Fix);
	for(int a=0; a<(10/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(10/2); a++){Serial.print(" ");}						
						
	dummy = String(telegram->PCBVersion);
	for(int a=0; a<(14/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(14/2); a++){Serial.print(" ");}						

	dummy = String(telegram->FirmwwareVersion);
	for(int a=0; a<(20/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(20/2); a++){Serial.print(" ");}

	dummy = String(telegram->NumberOfSatellites);
	for(int a=0; a<(24/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(24/2); a++){Serial.print(" ");}
	
	dummy = String(telegram->BatteryVoltage);
	for(int a=0; a<(18/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(18/2); a++){Serial.print(" ");}		

	dummy = String(telegram->Altitude);
	for(int a=0; a<(12/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(12/2); a++){Serial.print(" ");}		

	dummy = String(telegram->rssi);
	for(int a=0; a<(8/2-dummy.length()); a++){Serial.print(" ");}
	Serial.print(dummy);
	for(int a=0; a<(8/2); a++){Serial.print(" ");}

	Serial.println("");	
}

bool firstTimePrint = true;
void PrintRadioTelegramCSV(RadioTelegram_t *telegram){

	String dummy;

	if(firstTimePrint){
		Serial.println("Receiver ID,Transmitter ID,UTC Time,GPS Latitude,GPS Longitude,GPS Fix,PCB Version,Firmware Version,Number Of Satellites,Battery Voltage,Altitude,RSSI");
		firstTimePrint=false;
	}
	
	dummy = String(telegram->receiver_ID);
	Serial.print(dummy+",");
	
	dummy = String(telegram->transmitter_ID);
	Serial.print(dummy+",");
		
	dummy = String(telegram->UTCTime);
	Serial.print(dummy+",");
	
	dummy = String(telegram->Latitude);
	Serial.print(dummy+",");
	
	dummy = String(telegram->Longitude);
	Serial.print(dummy+",");
		
	dummy = String(telegram->Fix);
	Serial.print(dummy+",");
		
	dummy = String(telegram->PCBVersion);
	Serial.print(dummy+",");
	
	dummy = String(telegram->FirmwwareVersion);
	Serial.print(dummy+",");
	
	dummy = String(telegram->NumberOfSatellites);
	Serial.print(dummy+",");
		
	dummy = String(telegram->BatteryVoltage);
	Serial.print(dummy+",");
	
	dummy = String(telegram->Altitude);
	Serial.print(dummy+",");
	
	dummy = String(telegram->rssi);
	Serial.println(dummy);
}

/*
  
  
  //// old main
  do{
	 //Sample the voltages:
	 InputVoltage=getInputVoltage();
	 BatteryVoltage=getBatteryVoltage();
	 	
     switch(state){
        case STARTING_UP:
            // Find out if RCin is PWM, PPM or SBUS
			if(isGroundStation){
				Radio->SetRXMode(false); // No timeout
				state=WAIT_FOR_RX;			
			}
			else{
				state=SEND_TX_PACKAGE;
			}
          break;
		
		case WAIT_FOR_RX:
			if(InputVoltage < 4.8){
				Serial.println("Input voltage is " + String(InputVoltage) + "V, Now Running on Battery");
				PowerOFFGPS(); // Turn off  GPS to save battery.
				Radio->Sleep(); // Put radio to sleep to save power.
				digitalWrite(led2Pin, LOW);
			
				/*
				SPI.end();
				digitalWrite(txEnPin, LOW); // RX enabled by default.
				digitalWrite(rxEnPin, LOW); // RX enabled by default.
				digitalWrite(chipSelectPin, LOW); // RX enabled by default.
				digitalWrite(PIN_SPI_SCK, LOW); // RX enabled by default.
				digitalWrite(PIN_SPI_MOSI, LOW); // RX enabled by default.
				digitalWrite(PIN_SPI_MISO, LOW); // RX enabled by default.								
				*/	
				/*		
				state=RUNNING_ON_BATTERY;
			}
			//if(InputVoltage < 4.8){
			//	Serial.println("Input voltage is " + String(InputVoltage) + "V, Now Running on Battery");
			//	state=RUNNING_ON_BATTERY;
			//}
//			if(Radio->telegramValid == true){
//				Serial.println("We got a telegram!");
	//			ReceiveTelegram = Radio->GetTelegram();
//				PrintRadioTelegram(ReceiveTelegram); // Debug to USB serial port.	
				PrintRadioTelegramCSV(ReceiveTelegram); // Debug to USB serial port.	
	//		}
		  break;
		
		case SEND_TX_PACKAGE:
			if(InputVoltage < 4.8){
				Serial.println("Input voltage is " + String(InputVoltage) + "V, Now Running on Battery");
				
					state=RUNNING_ON_BATTERY;					
				
			}
			BuildRadioTelegram();
	//		Radio->SendTelegram(&RadioTelegram);
			
			PrintRadioTelegram(&RadioTelegram); // Debug to USB serial port.
			GPS->printGPSData(); // debug print GPS data
			
			digitalWrite(led2Pin, isLEDOn);
			isLEDOn = !isLEDOn;
			
			// debug
			if(BatteryVoltage <= 3.0){
				state=POWER_OFF;
			}
			
		  break;
	
        case NORMAL:
		  break;

        case RUNNING_ON_BATTERY:	   
			if(InputVoltage >= 4.8){
				Serial.println("Input voltage is " + String(InputVoltage) + "V, Now running from input power");
				// No longer running on battery.
				PowerONGPS();// Turn on GPS.
				Radio->SetRXMode(false); // Wake up the radio module and put it to constant RX mode - No timeout.
				USBDevice.standby(); // Power up USB again.
				if(isGroundStation){				
					state=WAIT_FOR_RX;
				}else{
					state=SEND_TX_PACKAGE;
				}
			}else{
				// Running on battery.
			
				// Don't discharge the battery to less than 3.0V.
				if(BatteryVoltage <= 3.0){
					state=POWER_OFF;
			    }
				
				//LowPower.sleep(2000); // Sleep for  1 second
			}
          break;

        case POWER_OFF:
		  digitalWrite(led2Pin, isLEDOn);
		  Serial.println("Input voltage is " + String(InputVoltage) + "V, Power OFF!");
		  Serial.println("Battery voltage is " + String(BatteryVoltage) + "V, Power OFF!");
		  delay(500);// time to serial print.
		  PowerOFF();
		  do{}while(1); // Stay here and power off.
          break;
        
        default:
          break;  
     }
	 
	  // Below this line code is executed fast!
	  if(RadioIRQFlag){
		  RadioIRQFlag=false;
		  Radio->HandleIRQ();
		  digitalWrite(led2Pin, isLEDOn);
		  isLEDOn = !isLEDOn;
	  }
	  
	  GPS->update();  // Function empty serial buffer and analyses string.
	  // RCin->read();  // SBUS, PPM or PWM.

  	  digitalWrite(led2Pin, isLEDOn);
  	  isLEDOn = !isLEDOn;
  }while(1);
*/