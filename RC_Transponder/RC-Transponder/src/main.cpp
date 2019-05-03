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
// GPS NMEA decoder
#include <GPSL80Lite.h>
 
#include "wiring_private.h" // Needed for pinPeripheral function.
#include "timer.h"
#include "main.h"
#include "hw.h"
#include "RFService.h"
#include "PCProtocol.h"

// Radio
#include "E28-2G4M20S.h"

// for SPORT
#include "FrSkySportSensor.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"

//void HandelSerial(void);
void LowPowerTest(void);
void GoToSleep(void);
String base64_encode(byte[], int);
void BeaconService(void);
void One_second_Update(void);

// System information
SystemInformation_t SystemInformation;

// FRSKY SPORT GPS
FrSkySportSensorGps FrskyGPS;             // Create GPS sensor with default ID
FrSkySportTelemetry FrskySport;           // Create telemetry object without polling

// a SBUS object, which is on hardware  serial port 4 (SERCOM4):
//SBUS x8r(SerialSBUS);

// channel, fail safe, and lost frames data for SBUS
//uint16_t channels[16];
//bool failSafe;
//bool lostFrame;

// Object and varibels for SX1280/E28_2G4 radio chip
E28_2G4M20S *Radio = NULL;
RFService *RadioService = NULL;
PCProtocol *SerialProtocol = NULL;

// $GPGGA,193648.000,5550.0838,N,01224.0718,E,2,8,0.96,32.7,M,41.5,M,0000,0000*62
GPSL80Lite *GPS = NULL; // GPS class, defined in GPSL80.h
GpsDataLite *GPSData = NULL;  // GPGGA GPS data:

//ISR for radio
/*
void Radio_isr(void){
//	SerialAUX->Println("Time:" + String(millis()));
  //Radio->HandleIRQ(); // The radio module has something for us.
//  RadioProtocol->IRQHandler();
}*/

#define POWER_DOWN_DELAY 60 // Wait 60 seconds after power is lost before go to low power mode.
#define GPS_ON_TIME   60 //  60 seconds
#define GPS_OFF_TIME 600 // 600 seconds (10min)


void setup() {	
	hwInit(); // Setup all pins according to hardware.
	
	#ifndef DEBUG
		delay(5000); // Time to get USB bootloader ready, but only if debugger is not connected.
	#endif
	
	// Init Auxiliary serial port:
	SerialAUX->begin(115200);
	
	// Init USB serial debug /setup:
	Serial.begin(115200);
	delay(500);
	
	Serial.println("Starting RC Transponder ver. " + String((int)SystemInformation.FIRMWARE_VERSION) + "." + String((int)((SystemInformation.FIRMWARE_VERSION-((int)SystemInformation.FIRMWARE_VERSION))*100)));
	
	// Read and store the 128 bit serial number.
//	ReadSerialNumberFromChipFlash();
	Serial.println("Chip unique serial number part 1:" + String(SystemInformation.SerialNumber1));
	Serial.println("Chip unique serial number part 2:" + String(SystemInformation.SerialNumber2));
	Serial.println("Chip unique serial number part 3:" + String(SystemInformation.SerialNumber3));
	Serial.println("Chip unique serial number part 4:" + String(SystemInformation.SerialNumber4));
	Serial.println("Chip unique serial number: \"" + String(SystemInformation.SerialNumber1) + String(SystemInformation.SerialNumber2) + String(SystemInformation.SerialNumber3) + String(SystemInformation.SerialNumber4)+"\"");
	#define SERIALNUMBER_SIZE 16
	uint8_t data[SERIALNUMBER_SIZE];
	data[0] = (byte)((SystemInformation.SerialNumber1 >> 24) & 0xFF);
	data[1] = (byte)((SystemInformation.SerialNumber1 >> 16) & 0xFF);
	data[2] = (byte)((SystemInformation.SerialNumber1 >> 8) & 0xFF);
	data[3] = (byte)(SystemInformation.SerialNumber1 & 0xFF);
	data[4] = (byte)((SystemInformation.SerialNumber2 >> 24) & 0xFF);
	data[5] = (byte)((SystemInformation.SerialNumber2 >> 16) & 0xFF);
	data[6] = (byte)((SystemInformation.SerialNumber2 >> 8) & 0xFF);
	data[7] = (byte)(SystemInformation.SerialNumber2 & 0xFF);
	data[8] = (byte)((SystemInformation.SerialNumber3 >> 24) & 0xFF);
	data[9] = (byte)((SystemInformation.SerialNumber3 >> 16) & 0xFF);
	data[10] = (byte)((SystemInformation.SerialNumber3 >> 8) & 0xFF);
	data[11] = (byte)(SystemInformation.SerialNumber3 & 0xFF);
	data[12] = (byte)((SystemInformation.SerialNumber4 >> 24) & 0xFF);
	data[13] = (byte)((SystemInformation.SerialNumber4 >> 16) & 0xFF);
	data[14] = (byte)((SystemInformation.SerialNumber4 >> 8) & 0xFF);
	data[15] = (byte)(SystemInformation.SerialNumber4 & 0xFF);
	Serial.print("Chip unique serial number in Base64 encode:\"");
	Serial.print(Telegram::base64_encode(data,SERIALNUMBER_SIZE).c_str());
	Serial.println("\"");

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
	Radio = new E28_2G4M20S(chipSelectPin,resetPin,busyPin,dio1Pin,0,0,txEnPin,rxEnPin, led2Pin);
	RadioService = new RFService(Radio, GPSData, &SystemInformation);
//	attachInterrupt(dio1Pin, Radio_isr, RISING); // Hack in mkr1000 Variant.h to add EXTERNAL_INTERRUPT 15 on pin 30 or EXTERNAL_INT_3 on pin 25 (PCB_VERSION 11)
	
	SerialProtocol = new PCProtocol(RadioService, Radio);
	
	// Init Frsky Smart port:
	SerialfrskySPort = new Uart(&sercom3, fryskySmartPortRXPin, fryskySmartPortTXPin, SERCOM_RX_PAD_3, UART_TX_PAD_2);   // Create the new UART instance for the Frsky SPORT module
	SerialfrskySPort->begin(57600);  //Baudrate for frskys SPORT protocol.
	pinPeripheral(fryskySmartPortRXPin, PIO_SERCOM_ALT); //Assign RX function to pin.
	pinPeripheral(fryskySmartPortTXPin, PIO_SERCOM_ALT); //Assign TX function to pin.
	FrskySport.begin(SerialfrskySPort, &FrskyGPS);
			

	// Set Timer 3 as 1 sec interrupt.
	startTimer3(1); // 1Hz
	
	SystemInformation.state=STARTING_UP;
	PowerON(); // Ensure transponder keeps running from battery if external power is lost.
	PowerONGPSBackup(); // Enable backup power for GPS.
	SystemInformation.BatteryVoltage = getBatteryVoltage();
	SystemInformation.InputVoltage = getInputVoltage();
	SystemInformation.USBVoltage = getInput5VVoltage();
	LEDSaftySwitchOFF();
}

void loop() {
  
     //LowPowerTest();
	  
//	SerialAUX->println("Receiver ID,Transmitter ID,UTC Time,GPS Latitude,GPS Longitude,GPS Fix,Number Of Satellites,Altitude,RSSI,SNR");	  

	do{
		One_second_Update();
		GPS->update();  // Function empty serial buffer and analyzes string.
		BeaconService();
		RadioService->Service();
	
		if(SystemInformation.SaftySwitchPushed == true){
			SerialAUX->println("System - Button pushed! - go to POWER_OFF");
			SystemInformation.state=POWER_OFF;
		}

		switch(SystemInformation.state)
		{
			case NORMAL: 
			{
				////// Below this line, code is executed fast!

				if(SystemInformation.SecondsBatteryLowCounter > 2){ // filter.
					SystemInformation.state=GET_READY_TO_RUN_ON_BATTERY;
//					SerialAUX->println("Main State: NORMAL -> GET_READY_TO_RUN_ON_BATTERY");
				}else{
				    // normal fast loop.
					SerialProtocol->Service(); // Comunincation to PC.
					FrskySport.send(); // Service the Serial for SPORT.
				}			
			}
			break;
			 
			case GET_READY_TO_RUN_ON_BATTERY:
			{
				if(SystemInformation.SecondsBatteryLowCounter == 0){
					// Power is back!
					SystemInformation.state=NORMAL;
//					SerialAUX->println("Main State: NORMAL");
				}
				else{
					// check if it is time to go to battery 
					if(SystemInformation.SecondsBatteryLowCounter > POWER_DOWN_DELAY){
						PowerONGPSBackup(); // Ensure backup power is enabled for GPS.
						SystemInformation.GPSActiveCounter=0; // ensure GPS active counter is reset.
						SystemInformation.state=RUNNING_ON_BATTERY_GPS_ON;
//						SerialAUX->println("Main State: GET_READY_TO_RUN_ON_BATTERY -> RUNNING_ON_BATTERY_GPS_ON");
					}else{
						SerialProtocol->Service(); // Comunincation to PC;
						FrskySport.send(); // Service the Serial for SPORT.
					}
				}
			} 
			break;
			
			case RUNNING_ON_BATTERY_GPS_ON:
			{
//				SerialAUX->println("Main State: RUNNING_ON_BATTERY_GPS_ON. GPSActiveCounter: " + String(SystemInformation.GPSActiveCounter));		
				if(SystemInformation.InputVoltage > 4.3 && (!SystemInformation.SimulateRunningOnBattery)){ // in debug mode force running on battery mode.
					SystemInformation.state=STARTING_UP;
					SerialAUX->println("Main State: RUNNING_ON_BATTERY_GPS_ON -> STARTING_UP");
				}else if(SystemInformation.BatteryVoltage <= 3.0){
					SystemInformation.state=POWER_OFF;
					SerialAUX->println("Main State: RUNNING_ON_BATTERY_GPS_ON -> POWER_OFF");
				}else{
					GoToSleep(); // Sleep until 1 sec interrupt will wake us up.
					if(++SystemInformation.GPSActiveCounter > GPS_ON_TIME){ // power off GPS after 1 min.
						SystemInformation.GPSActiveCounter=0;
						delay(2000); // busy wait while GPS serial gets time to receive data from GPS (Unable in sleep mode).
//						SerialAUX->println("Serial data for GPS available: " + String(SerialGPS->available())); 
						GPS->update();  // Service the GPS.
						PowerOFFGPS();// Turn OFF GPS main power.
						SystemInformation.state=RUNNING_ON_BATTERY_GPS_OFF;
//						SerialAUX->println("Main State: RUNNING_ON_BATTERY_GPS_ON -> RUNNING_ON_BATTERY_GPS_OFF");
					}
				}
			}
			break;

			case RUNNING_ON_BATTERY_GPS_OFF:
			{
//			    SerialAUX->println("Main State: RUNNING_ON_BATTERY_GPS_OFF. GPSActiveCounter: " + String(SystemInformation.GPSActiveCounter));		
				if(SystemInformation.InputVoltage > 4.3 && (!SystemInformation.SimulateRunningOnBattery)){ // in debug mode force running on battery mode.
					SystemInformation.state=STARTING_UP;
//					SerialAUX->println("Main State: RUNNING_ON_BATTERY_GPS_OFF -> STARTING_UP");		
					}else if(SystemInformation.BatteryVoltage <= 3.0){
						SystemInformation.state=POWER_OFF;
//						SerialAUX->println("Main State: RUNNING_ON_BATTERY_GPS_OFF -> POWER_OFF");					
					}else{
						GoToSleep(); // Sleep until 1 sec interrupt will wake us up.
						if(++SystemInformation.GPSActiveCounter > GPS_OFF_TIME){ // Turn on GPS every 10 mins.
							SystemInformation.GPSActiveCounter=0;
							PowerONGPS();// Turn OFF GPS main power.
							SystemInformation.state=RUNNING_ON_BATTERY_GPS_ON;
//							SerialAUX->println("Main State: RUNNING_ON_BATTERY_GPS_OFF -> RUNNING_ON_BATTERY_GPS_ON");
						}
				}
			}
			break;


			case POWER_OFF:
			{
				PowerOFF(); 
				do{
					// CPU should have died here, but if we are still powered the flash fast led.
					digitalWrite(led2Pin, HIGH);
					LEDSaftySwitchON();
					delay(50);
					digitalWrite(led2Pin, LOW);
					LEDSaftySwitchOFF();
					delay(50);
				}while(1);
			}
			break;			


			case STARTING_UP:
			{
				PowerONGPS();// Turn on GPS.
				// Set the radio to RX mode without timeout.
				SystemInformation.SecondCounter=0; // reset second counter.
				SystemInformation.SecondsBatteryLowCounter = 0;
				SystemInformation.state=NORMAL;		
//				SerialAUX->println("Main State: STARTING_UP -> NORMAL");								
			}
			break;
			 
			default:
//				SerialAUX->println("Main State: ERROR(default) -> STARTING_UP");		
				SystemInformation.state = STARTING_UP;
			break;
		}
	}while(1); 
}

void GoToSleep(void){
	SerialAUX->println("Going to sleep!");
	delay(100);

	RadioService->PowerDown();
	
	//delay(10); // time to TX Serial.

	//delay(1000);	
	USBDevice.detach();
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	USBDevice.attach();
	SerialAUX->print("Wake-up...");

	RadioService->WakeUp();

	SerialAUX->println("Done!");
}

void LowPowerTest(void){

	PowerOFFGPS(); // Turn off  GPS to save battery.
	PowerOFFGPSBackup(); // Ensure backup power is enabled for GPS.
	
	//
	digitalWrite(SaftyLEDPin, HIGH); // Off!
	
	SerialAUX->println("Battery analog reading is " + String(analogRead(analogVbatPin)));
	SerialAUX->println("Battery voltage is " + String(getBatteryVoltage()) + "V");
	digitalWrite(led2Pin, LOW);
	do
	{
	//	SerialAUX->Println("Sleep!");	
		GoToSleep();
	//	SerialAUX->Println("Wake!");
	}while(1);

}

// Ensure a beacon is transmitted every N second.
void BeaconService(void){
	if(SystemInformation.BeaconSecondCounter >= 5){
		SerialAUX->print("Time to make beacon message...");
		// Send a Standard beacon:
		SystemInformation.BeaconSecondCounter =  0; // Reset Beacon counter.
		if(SystemInformation.IsGroundStation==false){
			RadioService->SendBeacon();
		}else{
			SerialAUX->println("Im groundstation, NoPing!");
		}
		SerialAUX->println("Done!");	
	}	
}


#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
	char top;
	#ifdef __arm__
	return &top - reinterpret_cast<char*>(sbrk(0));
	#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
	return &top - __brkval;
	#else  // __arm__
	return __brkval ? &top - __brkval : &top - __malloc_heap_start;
	#endif  // __arm__
}


void One_second_Update(void){
	//// only every second (check status)
	while(SystemInformation.SecondCounter){
		SerialAUX->print("One_Second_Updated...");
		//				digitalWrite(led2Pin, HIGH);
		SystemInformation.SecondCounter--;
		
		SystemInformation.BatteryVoltage = getBatteryVoltage();
		SerialAUX->print("Battery Voltage...");
		SystemInformation.InputVoltage = getInputVoltage();
		SerialAUX->print("Input Voltage...");
		SystemInformation.USBVoltage = getInput5VVoltage();				
		SerialAUX->print("USB Voltage...");
		
		// Update the FrSky GPS emulator with the latest values from the GPS. (GPS Lite needs to be updated to read $GPRMC in order to get speed, cog and date information:
		FrskyGPS.setData(GPSData->LatitudeDecimal, GPSData->LongitudeDecimal,GPSData->Altitude,0,0,0,0,0,GPSData->UTC_hour,GPSData->UTC_min,GPSData->UTC_sec);	
		SerialAUX->print("FRsky Data update...");					
		SerialAUX->print("Free RAM = "); //F function does the same and is now a built in library, in IDE > 1.0.0
		SerialAUX->println(freeMemory(), DEC);  // print how much RAM is available.
//		SerialAUX->Println("Battery voltage is " + String(SystemInformation.BatteryVoltage) + "V.");
//		SerialAUX->Println("Input voltage is " + String(SystemInformation.InputVoltage) + "V.");
//		SerialAUX->Println("Input 5V voltage is " + String(SystemInformation.USBVoltage) + "V.");
//		SerialAUX->println("");
//		SerialAUX->println("Beacon Counter: " + String(SystemInformation.BeaconSecondCounter));

		if(SystemInformation.InputVoltage <= 4.3 || (SystemInformation.SimulateRunningOnBattery)){
			if(SystemInformation.SecondsBatteryLowCounter < 255){
				SystemInformation.SecondsBatteryLowCounter++;
			}
		}else{
			SystemInformation.SecondsBatteryLowCounter=0;
		}			

//		Serial.println("Switch:" + String(digitalRead(SaftySwitchPin)));
		
		if(digitalRead(SaftySwitchPin) == HIGH)
		{
			int count=0;
			for(int b=0;b<10;b++){
				count += digitalRead(SaftySwitchPin);
				delay(2);
			}
			if(count== 10){
				SystemInformation.SafteSwitchPushedTimer++;
				if(SystemInformation.SafteSwitchPushedTimer == 2){
					LEDSaftySwitchON();
					SystemInformation.SaftySwitchFirstTimePushed=true;
				}else if(SystemInformation.SafteSwitchPushedTimer >= 4){
					LEDSaftySwitchOFF();
					SystemInformation.SaftySwitchFirstTimePushed=false;
				}
			}
		}else{
			if(SystemInformation.SaftySwitchFirstTimePushed==true)
			{
				for(int a=0;a<10;a++)
				{
					LEDSaftySwitchOFF();
					delay(50);
					LEDSaftySwitchON();
					delay(50);
				}
				SystemInformation.SaftySwitchFirstTimePushed=false;
				SystemInformation.SaftySwitchPushed=true;
			}else
			{
				SystemInformation.SafteSwitchPushedTimer=0;
			}
		}
		SerialAUX->println("Done!");
	}
}
	