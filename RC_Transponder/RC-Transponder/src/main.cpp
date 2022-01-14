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
#include "main.h"
#include "Transponder_hal.h"
#include "timer.h"
#include "RFService.h"
//#include "PCProtocol.h"

// for SPORT
#include "FrSkySportSensor.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorPassthrough.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"

// Mavlink Handling 
#include "MavlinkHandler.h"

void LowPowerTest(void);
void GoToSleep(void);
String base64_encode(byte[], int);
void BeaconService(void);
void One_second_Update(void);
void mavlinkSendADSB(float latitude, float longitude, uint8_t serverity, char *callsign);

// Transponder Hardware apstraction layer
Transponder_hal systemHardware;

// System information
SystemInformation_t SystemInformation;

// FRSKY SPORT 
FrSkySportSensorGps FrskyGPS;             // Create GPS sensor with default ID
FrSkySportSensorPassthrough FrskyPASS;    // Create DIY sensor with default ID for Passthrough 
FrSkySportTelemetry FrskySport;           // Create telemetry object without polling
MavlinkHandler MavlinkData;				  // Create Mavlink class to handle Mavlink from FC and pass it to Frsky Passthrough.

// Object for Radio communication
RFService *RadioService = NULL;

// Object for PC/User communication
//PCProtocol *SerialProtocol = NULL;

#define POWER_DOWN_DELAY 60 // Wait 60 seconds after power is lost before go to low power mode.
#define GPS_ON_TIME   60 //  60 seconds
#define GPS_OFF_TIME 600 // 600 seconds (10min)

void setup() {	
	
	// Init USB serial debug /setup:
	Serial.begin(115200);
	delay(100);
	
	systemHardware.begin(); // Setup all pins according to hardware.
	
	#ifndef DEBUG
		delay(5000); // Time to get USB bootloader ready, but only if debugger is not connected.
	#endif
	
	// Init Auxiliary serial port:
	systemHardware.getSerialAUX()->begin(115200);
	
	Serial.println("Starting RC Transponder ver. " + String((int)SystemInformation.FIRMWARE_VERSION) + "." + String((int)((SystemInformation.FIRMWARE_VERSION-((int)SystemInformation.FIRMWARE_VERSION))*100)));
	
	// Read the 128 bit serial number.
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

	// Init the Radio protocol
	RadioService = new RFService(systemHardware.getRadio(), &SystemInformation);
//	SerialProtocol = new PCProtocol(RadioService, Radio);
	
	// Start the FrSky SPort libary with Serial port and the two sensors (GPS and Passthrought):	
	FrskySport.begin(systemHardware.getSerialFrSkySPort(), &FrskyGPS, &FrskyPASS);
	
	// Start the Mavlink decoder and link it to FrSky GSP and Passthrough sensors for data update:
	MavlinkData.begin(systemHardware.getSerialAUX(), &FrskyGPS, &FrskyPASS);		

	// Set Timer 3 as 1 sec interrupt.
	startTimer3(1); // 1Hz
	
	SystemInformation.state=STARTING_UP;
	systemHardware.PowerON(); // Ensure transponder keeps running from battery if external power is lost.
	systemHardware.PowerONGPSBackup(); // Enable backup power for GPS.
	SystemInformation.BatteryVoltage = systemHardware.getBatteryVoltage();
	SystemInformation.InputVoltage = systemHardware.getInputVoltage();
	SystemInformation.USBVoltage = systemHardware.getInput5VVoltage();
	systemHardware.LEDSaftySwitchOFF();
}

void loop() {
  
     //LowPowerTest();
	  
//	SerialAUX->println("Receiver ID,Transmitter ID,UTC Time,GPS Latitude,GPS Longitude,GPS Fix,Number Of Satellites,Altitude,RSSI,SNR");	  

	do{
		One_second_Update();
		systemHardware.getGPS()->update();  // Function empty serial buffer and analyzes string.
		RadioService->Service();			// Service the Radio module
		MavlinkData.service();				// Service the Mavlink data.
		BeaconService();				    // Time to make beacon for radio tx?
	
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
						systemHardware.PowerONGPSBackup(); // Ensure backup power is enabled for GPS.
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
						systemHardware.getGPS()->update();  // Service the GPS.
						systemHardware.PowerOFFGPS();// Turn OFF GPS main power.
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
							systemHardware.PowerONGPS();// Turn OFF GPS main power.
							SystemInformation.state=RUNNING_ON_BATTERY_GPS_ON;
//							SerialAUX->println("Main State: RUNNING_ON_BATTERY_GPS_OFF -> RUNNING_ON_BATTERY_GPS_ON");
						}
				}
			}
			break;


			case POWER_OFF:
			{
				systemHardware.PowerOFF(); 
				do{
					// CPU should have died here, but if we are still powered the flash fast led.
					systemHardware.LEDON();
					systemHardware.LEDSaftySwitchON();
					delay(50);
					systemHardware.LEDOFF();
					systemHardware.LEDSaftySwitchOFF();
					delay(50);
				}while(1);
			}
			break;			


			case STARTING_UP:
			{
				systemHardware.PowerONGPS();// Turn on GPS.
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
	RadioService->PowerDown();
	systemHardware.auxSerialPowerDown();
	systemHardware.PowerOFFFrSkySmartPort();

	USBDevice.detach();
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	
	// Wake up:
	USBDevice.attach();

	systemHardware.PowerONFrSkySmartPort();
	systemHardware.auxSerialPowerUp();
	RadioService->WakeUp();	
}

void LowPowerTest(void){
	systemHardware.PowerOFFGPS(); // Turn off  GPS to save battery.
	systemHardware.PowerOFFGPSBackup(); // Ensure backup power is enabled for GPS.	
	systemHardware.LEDSaftySwitchOFF();
	systemHardware.PowerOFFFrSkySmartPort();
	
//	SerialAUX->println("Battery analog reading is " + String(analogRead(analogVbatPin)));
//	SerialAUX->println("Battery voltage is " + String(getBatteryVoltage()) + "V");
	systemHardware.LEDOFF();
	do
	{
	//	SerialAUX->Println("Sleep!");	
		GoToSleep();
	//	SerialAUX->Println("Wake!");
	}while(1);

}

// Ensure a beacon is transmitted every N second.
void BeaconService(void){
	if(SystemInformation.BeaconSecondCounter >= 10){ // SystemInformation.BeaconSecondCounter is counted up en Timer3 (1Hz) ISR.
//		SerialAUX->print("Time to make beacon message...");
		// Send a Standard beacon:
		SystemInformation.BeaconSecondCounter =  0; // Reset Beacon counter.
		if(SystemInformation.IsGroundStation==false){
			// update system with latest GPS values;
//			Serial.println("Sending Beacon: " + String(systemHardware.getGPS()->getLatitude()) + "," + String(systemHardware.getGPS()->getLongitude()));
			SystemInformation.UTCTime = systemHardware.getGPS()->getUTCTime();
			SystemInformation.Latitude = systemHardware.getGPS()->getLatitude();
			SystemInformation.Longitude = systemHardware.getGPS()->getLongitude();
			SystemInformation.NumberOfSat = systemHardware.getGPS()->getNumberOfSat();
			SystemInformation.Fix = systemHardware.getGPS()->getFix();
			SystemInformation.hdop = systemHardware.getGPS()->getHDOP();
			RadioService->SendBeacon();
		}else{
//			SerialAUX->println("Im groundstation, NoPing!");
		}
//		SerialAUX->println("Done!");	
	}else{ // see if there are any messages from radio to Mavlink / FrSky.
		
		transponderData_t *temp = RadioService->getRadioText();		
		if(temp->dataReady){
			FrskyPASS.setDataTextMSG(temp->data, temp->servirity);

			// Make Mavlink ADSB msg:
			mavlinkSendADSB(temp->latitude,temp->longitude,temp->servirity, "Lagoni-1");
			
			temp->dataReady=false;
		}
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


//		Serial.print(String(GPSData->UTC_hour+1) + ":" + String(GPSData->UTC_min) + ":" +  String(GPSData->UTC_sec) + " ");
		SystemInformation.BatteryVoltage = systemHardware.getBatteryVoltage();
		//Serial.print("Battery Voltage (" + String(SystemInformation.BatteryVoltage) + "V) ChargeState(" + String(systemHardware.GetChargeState()) + ") ");
		SystemInformation.InputVoltage = systemHardware.getInputVoltage();
		//Serial.print("Input Voltage (" + String(SystemInformation.InputVoltage) + "V) ");
		SystemInformation.USBVoltage = systemHardware.getInput5VVoltage();				
		//Serial.print("USB Voltage (" + String(SystemInformation.USBVoltage) + "V) ");
		//Serial.print("\n\r");

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
		
		if(systemHardware.SaftySwitchPushed())
		{			
			SystemInformation.SafteSwitchPushedTimer++;
			if(SystemInformation.SafteSwitchPushedTimer == 2){
				systemHardware.LEDSaftySwitchON();
				SystemInformation.SaftySwitchFirstTimePushed=true;
			}else if(SystemInformation.SafteSwitchPushedTimer >= 4){
				systemHardware.LEDSaftySwitchOFF();
				SystemInformation.SaftySwitchFirstTimePushed=false;
			}
		}else{
			if(SystemInformation.SaftySwitchFirstTimePushed==true)
			{
				for(int a=0;a<10;a++)
				{
					systemHardware.LEDSaftySwitchOFF();
					delay(50);
					systemHardware.LEDSaftySwitchON();
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


void mavlinkSendADSB(float latitude, float longitude, uint8_t serverity, char *callsign){

	uint8_t txBuffer[512]; 
	
	//lets make an ADSB msg and sent it to pixhawk on Serial2:
	mavlink_adsb_vehicle_t msgADSB;
	mavlink_message_t msg;
  
	msgADSB.ICAO_address = 0xCAFF;
	msgADSB.lat = (int32_t)(latitude*1E7);
	msgADSB.lon = (int32_t)(longitude*1E7);
//	msgADSB.lat = 558450604;
//	msgADSB.lon = 124708557;
	msgADSB.altitude = 1000;
	msgADSB.heading = 4500;
	msgADSB.hor_velocity = 10;
	msgADSB.ver_velocity = 0;
	msgADSB.flags = 319;
	
	if(serverity <= 4){
		msgADSB.squawk = 7700;		
	}else{
		msgADSB.squawk = 1800;		
	}
		
	msgADSB.altitude_type = 0; // BARO
	/*
	msgADSB.callsign[0] = 'L';
	msgADSB.callsign[1] = 'A';
	msgADSB.callsign[2] = 'G';
	msgADSB.callsign[3] = 'O';
	msgADSB.callsign[4] = 'N';
	msgADSB.callsign[5] = 0;
	msgADSB.callsign[6] = 0;
	msgADSB.callsign[7] = 0;
	msgADSB.callsign[8] = 0;
	*/
	for(int a=0;a<9;a++){
		msgADSB.callsign[a] = *callsign;
		callsign++;	
	}
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
		systemHardware.getSerialAUX()->write(txBuffer[a]);
	}		
}

	
	/*
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
*/
	
	
void SerialPrintHEX(int data){
	Serial.print("0x");
	if(data < 16){
		Serial.print("0");
	}
	Serial.print(data,HEX);
}


// Not part of Class but needed to overwride Ardunio functions:

// Attach the interrupt handler to the SERCOM
void SERCOM2_Handler(){
	systemHardware.getSerialGPS()->IrqHandler();
}

void SERCOM3_Handler(){
	systemHardware.getSerialFrSkySPort()->IrqHandler();
}

void SERCOM4_Handler(){
	systemHardware.getSerialSBUS()->IrqHandler();
}

void SERCOM5_Handler(){
	systemHardware.getSerialAUX()->IrqHandler();
}
