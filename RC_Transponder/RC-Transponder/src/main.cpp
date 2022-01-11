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
#include "FrSkySportSensorPassthrough.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"

// for Mavlink
#include "common/mavlink.h"
#include "ardupilotmega/mavlink.h"

//void HandelSerial(void);
void LowPowerTest(void);
void GoToSleep(void);
String base64_encode(byte[], int);
void BeaconService(void);
void One_second_Update(void);

void ADSB_test(void);
void MavlinkService(void);
uint32_t Get_Volt_Average1(uint16_t); 	
uint32_t Get_Current_Average1(uint16_t); 
void Accum_Volts1(uint32_t);
void Accum_mAh1(uint32_t);

// System information
SystemInformation_t SystemInformation;

// FRSKY SPORT GPS
FrSkySportSensorGps FrskyGPS;             // Create GPS sensor with default ID
FrSkySportSensorPassthrough FrskyPASS;    // Create DIY sensor with default ID for Passthrough 
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

/* Debug
#define POWER_DOWN_DELAY 5 // Wait 60 seconds after power is lost before go to low power mode.
#define GPS_ON_TIME  60 //  60 seconds
#define GPS_OFF_TIME 5 // 600 seconds (10min)
*/
void setup() {	
	hwInit(); // Setup all pins according to hardware.
	
	#ifndef DEBUG
		delay(5000); // Time to get USB bootloader ready, but only if debugger is not connected.
	#endif
	
	// Init Auxiliary serial port:
	SerialAUX->begin(115200);
//	SerialAUX->begin(57600);
	
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

	// GPS INIT;
	PowerONGPS();// Turn on GPS. price 31uA
	ResetGPS();
	
	GPS = new GPSL80Lite();
	GPSData = new GpsDataLite();
	GPS->init(SerialGPS, GPSData, 57600, GPSRxPin, GPSTxPin);
	
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
	FrskySport.begin(SerialfrskySPort, &FrskyGPS, &FrskyPASS);
			

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
		MavlinkService();
	
		if(SystemInformation.SaftySwitchPushed == true){
//			SerialAUX->println("System - Button pushed! - go to POWER_OFF");
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
//					SerialProtocol->Service(); // Comunincation to PC.
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
//						SerialProtocol->Service(); // Comunincation to PC;
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
//					SerialAUX->println("Main State: RUNNING_ON_BATTERY_GPS_ON -> STARTING_UP");
				}else if(SystemInformation.BatteryVoltage <= 3.0){
					SystemInformation.state=POWER_OFF;
//					SerialAUX->println("Main State: RUNNING_ON_BATTERY_GPS_ON -> POWER_OFF");
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
				if( ((SystemInformation.InputVoltage > 4.3) || (SystemInformation.USBVoltage > 2.0 )) && (!SystemInformation.SimulateRunningOnBattery)){ // in debug mode force running on battery mode.
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
					LEDON();
					LEDSaftySwitchON();
					delay(50);
					LEDOFF();
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
//	SerialAUX->println("Going to sleep!");
//	delay(100);

	RadioService->PowerDown();
	auxSerialPowerDown();
	//delay(10); // time to TX Serial.

	//delay(1000);	
	USBDevice.detach();
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	USBDevice.attach();
//	SerialAUX->print("Wake-up...");
	auxSerialPowerUp();
	RadioService->WakeUp();
	
//	SerialAUX->println("Done!");
}

void LowPowerTest(void){

	PowerOFFGPS(); // Turn off  GPS to save battery.
	PowerOFFGPSBackup(); // Ensure backup power is enabled for GPS.	
	LEDSaftySwitchOFF();

	
	digitalWrite(fryskyInvertPin, LOW);
	pinMode(fryskySmartPortRXPin, INPUT_PULLDOWN);
	
	
//	SerialAUX->println("Battery analog reading is " + String(analogRead(analogVbatPin)));
//	SerialAUX->println("Battery voltage is " + String(getBatteryVoltage()) + "V");
	LEDOFF();
	do
	{
	//	SerialAUX->Println("Sleep!");	
		GoToSleep();
	//	SerialAUX->Println("Wake!");
	}while(1);

}

// Ensure a beacon is transmitted every N second.
void BeaconService(void){
	if(SystemInformation.BeaconSecondCounter >= 5){ // SystemInformation.BeaconSecondCounter is counted up en Timer3 (1Hz) ISR.
//		SerialAUX->print("Time to make beacon message...");
		// Send a Standard beacon:
		SystemInformation.BeaconSecondCounter =  0; // Reset Beacon counter.
		if(SystemInformation.IsGroundStation==false){
			RadioService->SendBeacon();
		}else{
//			SerialAUX->println("Im groundstation, NoPing!");
		}
//		SerialAUX->println("Done!");	
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

int debugCounter=0;
uint8_t serCount = 0;
//int attitudeMsgHZ;

void One_second_Update(void){
	//// only every second (check status)
	while(SystemInformation.SecondCounter){
		
		//ADSB test:
		//ADSB_test();
		
//		SerialAUX->print("One_Second_Updated...");
		//				digitalWrite(led2Pin, HIGH);
		SystemInformation.SecondCounter--;

		Serial.print(String(GPSData->UTC_hour+1) + ":" + String(GPSData->UTC_min) + ":" +  String(GPSData->UTC_sec) + " ");
		SystemInformation.BatteryVoltage = getBatteryVoltage();
		Serial.print("Battery Voltage (" + String(SystemInformation.BatteryVoltage) + "V) ChargeState(" + String(GetChargeState()) + ") ");
		SystemInformation.InputVoltage = getInputVoltage();
		Serial.print("Input Voltage (" + String(SystemInformation.InputVoltage) + "V) ");
		SystemInformation.USBVoltage = getInput5VVoltage();				
		Serial.print("USB Voltage (" + String(SystemInformation.USBVoltage) + "V) ");
		Serial.print("\n\r");

		// Update the FrSky GPS emulator with the latest values from the GPS. (GPS Lite needs to be updated to read $GPRMC in order to get speed, cog and date information:
		//FrskyGPS.setData(GPSData->LatitudeDecimal, GPSData->LongitudeDecimal,GPSData->Altitude,0,0,0,0,0,GPSData->UTC_hour,GPSData->UTC_min,GPSData->UTC_sec);	
		
	//	Serial.println("Attitude " + String(attitudeMsgHZ) + "Hz");
		//attitudeMsgHZ=0;
		
		if(debugCounter >= 10)
		{
			debugCounter=0;
			if(serCount++ >= 8){
				serCount=0;
			}
			char *text   = "0123456789";
//			FrskyPASS.setDataTextMSG(text, 4);
		}else{
			debugCounter++;
		}
				
//		SerialAUX->print("FRsky Data update...");					
//		SerialAUX->print("Free RAM = "); //F function does the same and is now a built in library, in IDE > 1.0.0
//		SerialAUX->println(freeMemory(), DEC);  // print how much RAM is available.
//		SerialAUX->println("Battery voltage is " + String(SystemInformation.BatteryVoltage) + "V.");
//		SerialAUX->println("Input voltage is " + String(SystemInformation.InputVoltage) + "V.");
//		SerialAUX->println("Input 5V voltage is " + String(SystemInformation.USBVoltage) + "V.");
//		SerialAUX->println("");
//		SerialAUX->println("Beacon Counter: " + String(SystemInformation.BeaconSecondCounter));

		if( ((SystemInformation.InputVoltage <= 4.3) && (SystemInformation.USBVoltage <=2 )) || (SystemInformation.SimulateRunningOnBattery)){
			if(SystemInformation.SecondsBatteryLowCounter < 255){
				SystemInformation.SecondsBatteryLowCounter++;
			}
		}else{
			SystemInformation.SecondsBatteryLowCounter=0;
		}			

//		Serial.println("Switch:" + String(digitalRead(SaftySwitchPin)));
		
		if(SaftySwitchPushed())
		{			
			SystemInformation.SafteSwitchPushedTimer++;
			if(SystemInformation.SafteSwitchPushedTimer == 2){
				LEDSaftySwitchON();
				SystemInformation.SaftySwitchFirstTimePushed=true;
			}else if(SystemInformation.SafteSwitchPushedTimer >= 4){
				LEDSaftySwitchOFF();
				SystemInformation.SaftySwitchFirstTimePushed=false;
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
//		SerialAUX->println("Done!");
	}
}
	
	
void ADSB_test(void){
	
	uint8_t txBuffer[512]; 
	
	//lets make an ADSB msg and sent it to pixhawk on Serial2:
	mavlink_adsb_vehicle_t msgADSB;
	mavlink_message_t msg;
  
	msgADSB.ICAO_address = 0xCAFF;
	msgADSB.lat = 558450604;
	msgADSB.lon = 124708557;
	msgADSB.altitude = 35000;
	msgADSB.heading = 90;
	msgADSB.hor_velocity = 10;
	msgADSB.ver_velocity = 0;
	msgADSB.flags = 319;
	msgADSB.squawk = 777;
	msgADSB.altitude_type = 0; // BARO
	msgADSB.callsign[0] = 'L';
	msgADSB.callsign[1] = 'a';
	msgADSB.callsign[2] = 'g';
	msgADSB.callsign[3] = 'o';
	msgADSB.callsign[4] = 'n';
	msgADSB.callsign[5] = 'i';
	msgADSB.callsign[6] = 0;
	msgADSB.callsign[7] = 0;
	msgADSB.callsign[8] = 0;
	msgADSB.emitter_type = 14; // UAV
	msgADSB.tslc = 0;
  
	mavlink_msg_adsb_vehicle_encode(0, 0, &msg, &msgADSB);
	int chan = MAVLINK_COMM_0;
	static const uint8_t mavlink_message_crcs[256] = {  50, 124, 137,   0, 237, 217, 104, 119,   0,   0, //  0-9
		0,  89,   0,   0,   0,   0,   0,   0,   0,   0, // 10-19
		214, 159, 220, 168,  24,  23, 170, 144,  67, 115, // 20-29
		39, 246, 185, 104, 237, 244, 222, 212,   9, 254, // 30-39
		230,  28,  28, 132, 221, 232,  11, 153,  41,  39, // 40-49
		78, 196,   0,   0,  15,   3,   0,   0,   0,   0, // 50-59
		0, 153, 183,  51,  59, 118, 148,  21,   0, 243, // 60-69
		124,   0,  20,  38,  20, 158, 152, 143,   0,   0, // 70-79
		0, 106,  49,  22, 143, 140,   5, 150,   0, 231, // 80-89
		183,  63,  54,   0,   0,   0,   0,   0,   0,   0, // 90-99
		175, 102, 158, 208,  56,  93, 138, 108,  32, 185, //100-109
		84,  34, 174, 124, 237,   4,  76, 128,  56, 116, //110-119
		134, 237, 203, 250,  87, 203, 220,  25, 226,  46, //120-129
		29, 223,  85,   6, 229, 203,   1, 195, 109, 168, //130-139
		181,  47,  72, 131, 127,   0, 103, 154, 178, 200, //140-149
		134,   0, 208,   0,   0,   0,   0,   0,   0,   0, //150-159
		0,   0,   0, 127,   0,  21,   0,   0,   1,   0, //160-169
		0,   0,   0,   0,   0,   0,   0,   0,  47,   0, //170-179
		0,   0, 229,   0,   0,   0,   0,   0,   0,   0, //180-189
		0,   0,   0,  71,   0,   0,   0,   0,   0,   0, //190-199
		0,   0,   0,   0,   0,   0,   0,   0,   0,   0, //200-209
		0,   0,   0,   0,   0,   0,   0,   0,   0,   0, //210-219
		34,  71,  15,   0,   0,   0,   0,   0,   0,   0, //220-229
		163, 105,   0,  35,   0,   0,   0,   0,   0,   0, //230-239
		0,  90, 104,  85,  95, 130, 184,   0,   8, 204, //240-249
	49, 170,  44,  83,  46,   0};          //250-256
	mavlink_finalize_message_chan(&msg, 0, 0, chan, msg.len, msg.len, mavlink_message_crcs[msg.msgid]);
	uint16_t totalLength=0;
	bzero(txBuffer, 512);
	totalLength = mavlink_msg_to_send_buffer(txBuffer, &msg);

	for(int a=0;a<totalLength;a++){
		SerialAUX->write(txBuffer[a]);
	}		
	
	
	
	
	mavlink_statustext_t hello;
	hello.severity=MAV_SEVERITY_INFO;
	std::string s = "Hello to Lagonis Mavlink Server Program";
	strcpy(hello.text, s.c_str());
	
}



mavlink_status_t msgStatus;
mavlink_message_t msgInput;
//uint32_t lastTime;

struct Battery {
	float    mAh;
	float    tot_mAh;
	float    avg_cA;
	float    avg_mV;
	uint32_t prv_millis;
	uint32_t tot_volts;      // sum of all samples
	uint32_t tot_mW;
	uint32_t samples;
	bool ft;
};

struct Battery bat1 = {
0, 0, 0, 0, 0, 0, 0, true};

int32_t home_latitude, home_longitude;
bool frameTypeSent=false;
float cog;
uint16_t currentWP;

void MavlinkService(void){
	
	 if(SerialAUX->available()){ // service serial data from FC.
		 uint8_t rbyte= SerialAUX->read();
		 if (mavlink_parse_char(MAVLINK_COMM_1, rbyte, &msgInput, &msgStatus))
		 {
//			Serial.println("New Mavlink messages #" + String(msgInput.msgid));
			// New Mavlink message received:		
			 switch(msgInput.msgid)
			 {
				 
				 case MAVLINK_MSG_ID_ATTITUDE: // #30
				 {
					 FrskyPASS.setDataAttitude( mavlink_msg_attitude_get_roll(&msgInput), mavlink_msg_attitude_get_pitch(&msgInput) );
					 // attitudeMsgHZ++;
					 // Serial.println("Attitude time: " + String(millis()-lastTime));
					 // lastTime=millis();
				 }
				 break;
				 
				 
				 case MAVLINK_MSG_ID_HEARTBEAT: // #0
				 {
					 uint8_t ap_type = mavlink_msg_heartbeat_get_type(&msgInput);
					 if (ap_type == 5 || ap_type == 6 || ap_type == 18 || ap_type == 26 || ap_type == 27) break;
					 // Ignore heartbeats from GCS (6) or Ant Trackers(5) or Onboard_Controllers(18) or Gremsy Gimbal(26) or ADSB (27))

					 uint32_t ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msgInput);
					 uint8_t ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&msgInput);
					 uint8_t armed = ap_base_mode >> 7;
					 FrskyPASS.setDataAPStatus((uint8_t)(ap_custom_mode + 1), 0, armed, armed);

					 if(!frameTypeSent){
						frameTypeSent=true;
						FrskyPASS.setDataParameterFrameType(ap_type);						 
					 }

				 }
				 break;


				 case MAVLINK_MSG_ID_SYS_STATUS: // #1
				 {
					 float voltage_battery1 = (float)Get_Volt_Average1(mavlink_msg_sys_status_get_voltage_battery(&msgInput));        // V  from Get_Volt_Average1()
					 voltage_battery1=voltage_battery1/1000; // now in [V]
					 float current_battery1 = (float)Get_Current_Average1(mavlink_msg_sys_status_get_current_battery(&msgInput));     // dA,  100 = 1A
					 current_battery1 = current_battery1 / 100; // now in [A]
					 FrskyPASS.setDataBattery1Status(voltage_battery1,current_battery1,bat1.tot_mAh);
				 }
				 break;
				 
				 case MAVLINK_MSG_ID_PARAM_VALUE: // #22 (I think we need to request this!
				 {
					 uint16_t param_index = mavlink_msg_param_value_get_param_index(&msgInput);
					
					 if(param_index == 356){ // Battery 1 capacity parameter
						FrskyPASS.setDataParameterBatteryPack1Capacity(mavlink_msg_param_value_get_param_value(&msgInput));
					 }
					 
					 if(param_index == 364){ // Battery 2 capacity parameter
						FrskyPASS.setDataParameterBatteryPack2Capacity(mavlink_msg_param_value_get_param_value(&msgInput));
					 }
				 }
				 break;			 
				 
				 case MAVLINK_MSG_ID_STATUSTEXT: //#253
				 {
					 mavlink_statustext_t msg;
					 mavlink_msg_statustext_decode(&msgInput, &msg);
					 FrskyPASS.setDataTextMSG(msg.text, msg.severity);
				 }
				 break;


				 case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // #33
				 {
				    int32_t current_lat = mavlink_msg_global_position_int_get_lat(&msgInput);             // Latitude, expressed as degrees * 1E7
				    int32_t current_lon = mavlink_msg_global_position_int_get_lon(&msgInput);             // Longitude, expressed as degrees * 1E7
					int32_t alt_ag = mavlink_msg_global_position_int_get_relative_alt(&msgInput);         // Altitude above ground (millimeters)
										
					FrskyPASS.setDataHomeStatus_FromMavlink(home_latitude,home_longitude,current_lat,current_lon, alt_ag); // Home lat/long from MSG #242.
				 }
				 break;
				 				 
				    
			 	 case MAVLINK_MSG_ID_HOME_POSITION: // #242
			 	 {
				 	 home_latitude = mavlink_msg_home_position_get_latitude(&msgInput);
				 	 home_longitude = mavlink_msg_home_position_get_longitude(&msgInput);
			 	 }
			 	 break;
				  
				 case MAVLINK_MSG_ID_VFR_HUD:      //  #74
			 	 {
					 FrskyPASS.setDataVYAWStatus((float)mavlink_msg_vfr_hud_get_climb(&msgInput), (float)mavlink_msg_vfr_hud_get_groundspeed(&msgInput), (float)mavlink_msg_vfr_hud_get_heading(&msgInput));
					 FrskyPASS.setDataVFRHUD(mavlink_msg_vfr_hud_get_airspeed(&msgInput), mavlink_msg_vfr_hud_get_throttle(&msgInput), mavlink_msg_vfr_hud_get_alt(&msgInput));
			 	 }
			 	 break;				 
				  
		
		
				 case MAVLINK_MSG_ID_BATTERY_STATUS: // #147 
				 {

					  uint8_t battery_id = mavlink_msg_battery_status_get_id(&msgInput);
					  if (battery_id == 0) {  // Battery 1
						  bat1.tot_mAh = (float)mavlink_msg_battery_status_get_current_consumed(&msgInput);    // mAh
					  }/* 
					  else if (battery_id == 1) {  // Battery 2
						  pt_bat2_mAh = ap_current_consumed;
					  }*/					 
				 }
				 break;
				 
				 
			 	 case MAVLINK_MSG_ID_GPS_RAW_INT: // #24
				 {
					uint8_t fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msgInput);  // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
					uint8_t sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msgInput);    // number of visible satellites
					int32_t lat = mavlink_msg_gps_raw_int_get_lat(&msgInput);
					int32_t lon = mavlink_msg_gps_raw_int_get_lon(&msgInput);
					int32_t amsl = mavlink_msg_gps_raw_int_get_alt(&msgInput);     // 1m =1000
					uint16_t hdop = mavlink_msg_gps_raw_int_get_eph(&msgInput);     // GPS HDOP 
					
					// for Mission Waypoint
					cog=(float)mavlink_msg_gps_raw_int_get_cog(&msgInput); // in [cdeg]
					cog=cog/100; // now in [deg]
					
					FrskyPASS.setDataGPSStatus(sat_visible, (fixtype & 0x03),lat,lon, hdop, ((fixtype>>2) & 0x03), amsl);
					FrskyGPS.setData((float)(lat/1E7),(float)(lon/1E7),0,0,0,0,0,0,0,0,0); // only lat/lon data.	
				 }
				 break;

			 	 case MAVLINK_MSG_ID_MISSION_COUNT: // #44   received back after #43 Mission_Request_List sent
				 {
					FrskyPASS.setDataParameterNumberOfWaypointInMission(mavlink_msg_mission_count_get_count(&msgInput));
				 }
				 break;	

			 	 case MAVLINK_MSG_ID_MISSION_CURRENT: // #42 
			 	 {
					currentWP =  mavlink_msg_mission_current_get_seq(&msgInput);  
				  }
			 	 break;
				  
			 	 case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: // #62
			 	 {
					int16_t target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msgInput);   // Bearing to current waypoint/target [degrees]
					uint16_t wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msgInput);                // Distance to active waypoint [meters]
					float xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&msgInput);         // Current crosstrack error on x-y plane [meters]
	
					FrskyPASS.setDataMissionWaypoints(currentWP,wp_dist,xtrack_error,cog,target_bearing);								
			 	 }
			 	 break;

				 default:
				 break;
			 }
		 }
	 }
	
}

//=================================================================================================

uint32_t Get_Current_Average1(uint16_t cA)  {   // in 100*milliamperes (1 = 100 milliampere)
	
	Accum_mAh1(cA);
	
	if (bat1.avg_cA < 1){
		bat1.avg_cA = cA;  // Initialise first time
	}

	bat1.avg_cA = (bat1.avg_cA * 0.6666F) + (cA * 0.333F);  // moving average

	return bat1.avg_cA;
}

uint32_t Get_Volt_Average1(uint16_t mV)  {

	if (bat1.avg_mV < 1) bat1.avg_mV = mV;  // Initialise first time

	// bat1.avg_mV = (bat1.avg_mV * 0.9) + (mV * 0.1);  // moving average
	bat1.avg_mV = (bat1.avg_mV * 0.6666) + (mV * 0.3333);  // moving average
	Accum_Volts1(mV);
	return bat1.avg_mV;
}

void Accum_Volts1(uint32_t mVlt) {    //  mV   milli-Volts
	bat1.tot_volts += (mVlt / 1000);    // Volts
	bat1.samples++;
}


void Accum_mAh1(uint32_t cAs) {        //  cA    100 = 1A
	if (bat1.ft) {
		bat1.prv_millis = millis() -1;   // prevent divide zero
		bat1.ft = false;
	}
	uint32_t period = millis() - bat1.prv_millis;
	bat1.prv_millis = millis();
	
	double hrs = (float)(period / 3600000.0f);  // ms to hours

	bat1.mAh = cAs * hrs;     //  Tiny cAh consumed this tiny period di/dt
	// bat1.mAh *= 100;        //  dA to mA
	bat1.mAh *= 10;           //  cA to mA
	bat1.mAh *= 1.0625;       // Emirical adjustment Markus Greinwald 2019/05/21
	bat1.tot_mAh += bat1.mAh;   //   Add them all in
}

//=================================================================================================