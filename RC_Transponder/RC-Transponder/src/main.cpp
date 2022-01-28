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
Generic Clock Generator 2	- Enabled running from internal 32KHz. (Used for Watchdog)
Generic Clock Generator 3	- Enabled running from internal 8MHz.
Generic Clock Generator 4   - Enabled running from internal 32KHz.  Used for Timer 3 (1 Hz interrupt).
Generic Clock Generator 5-8 - Disabled

*If set to sleep, and GCM_EIC clock is stopped, wakeup on external interrupt is not possible. GCM_EIC should be set to running clock (32khz) or so.

*/
#include <Arduino.h>
#include "main.h"
#include "Transponder_hal.h"
#include "timer.h"
#include "RFService.h"
#include "PCProtocol.h"
#include "FlashStorage.h"
#include "Adafruit_SleepyDog.h"

// for SPORT
#include "FrSkySportPollingDynamic.h"
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

FlashStorage(storedConfiguration, Configuration_t);

// Transponder Hardware abstraction layer
Transponder_hal systemHardware;

// System information
SystemInformation_t SystemInformation;

// FRSKY SPORT 
FrSkySportSensorGps FrskyGPS;             // Create GPS sensor with default ID
FrSkySportSensorGps FrskyGPS2(FrSkySportSensor::ID5);     // Create second GPS sensor (to be used in modulebay)
FrSkySportSensorPassthrough FrskyPASS;    // Create DIY sensor with default ID for Passthrough 
FrSkySportTelemetry FrskySport;           // Create telemetry object without polling
FrSkySportTelemetry FrskySportInModuleBay(new FrSkySportPollingDynamic(false));  // Create telemetry object with polling but disable the rxstatus (RSSI and Vbat tx part to radio)
MavlinkHandler MavlinkData;				  // Create Mavlink class to handle Mavlink from FC and pass it to Frsky Passthrough.

// Object for Radio communication
//RFService *RadioService = NULL;
RFService RadioService;

// Object for PC/User communication
PCProtocol serviceInterface;

/*
#define POWER_DOWN_DELAY  60 // Wait 60 seconds after power is lost before go to low power mode.
#define GPS_ON_TIME       60 //  60 seconds
#define GPS_OFF_TIME     600 // 600 seconds (10min)
*/
// DEBUG
//#define POWER_DOWN_DELAY  10 // Wait 60 seconds after power is lost before go to low power mode.
//#define GPS_ON_TIME       60 //  60 seconds
//#define GPS_OFF_TIME      10 // 600 seconds (10min)


void setup() {	
	// Init USB serial debug /setup:
	systemHardware.begin(); // Setup all pins according to hardware.
	Serial.begin(115200);

	uint32_t tempWDTCause = Watchdog.resetCause();
	/// Debug:
/*
	systemHardware.getSerialAUX()->println("");
	systemHardware.getSerialAUX()->print("Reset cause:" + String(tempWDTCause) + " ->");
	if(tempWDTCause == 1){
		systemHardware.getSerialAUX()->println("Power On Reset");
		}else if(tempWDTCause == 2){
		systemHardware.getSerialAUX()->println("1.2V Brownout!");
		}else if(tempWDTCause == 4){
		systemHardware.getSerialAUX()->println("3.3V Brownout!");
		}else if(tempWDTCause == 4){
		systemHardware.getSerialAUX()->println("3.3V Brownout!");
		}else if(tempWDTCause == 16){
		systemHardware.getSerialAUX()->println("External reset!");
		}else if(tempWDTCause == 32){
		systemHardware.getSerialAUX()->println("Watchdog!");
		/*
		do{
			systemHardware.LEDON();
			delay(50);
			systemHardware.LEDOFF();
			delay(50);
		}while(1);
		*//*
		}else if(tempWDTCause == 64){
		systemHardware.getSerialAUX()->println("System reset");
		}else{
		systemHardware.getSerialAUX()->println("Unknown");
	}
*/
	systemHardware.setBrownOut();
	
	SystemInformation.savedSettings = storedConfiguration.read();
	if(SystemInformation.savedSettings.valid == false){ // use default settings
		SystemInformation.savedSettings.isGroundstation = false;
		SystemInformation.savedSettings.POWER_DOWN_DELAY=60;   // Wait 60 seconds after power is lost before go to low power mode.
		SystemInformation.savedSettings.GPS_ON_TIME=60;		   //  60 seconds
		SystemInformation.savedSettings.GPS_OFF_TIME=600;	   // 600 seconds (10min)
		SystemInformation.savedSettings.BEACON_INTERVAL=10;   // every 10 seconds
		SystemInformation.savedSettings.callsign[0] = 'A'; 
		SystemInformation.savedSettings.callsign[1] = 'B'; 
		SystemInformation.savedSettings.callsign[2] = 'C'; 
		SystemInformation.savedSettings.callsign[3] = 'D'; 
		SystemInformation.savedSettings.callsign[4] = 'E'; 
		SystemInformation.savedSettings.callsign[5] = 'F'; 
		SystemInformation.savedSettings.callsign[6] = 'G'; 
		SystemInformation.savedSettings.callsign[7] = 'H';
		SystemInformation.savedSettings.callsign[8] = 0;
	}	
	String callsign = String("ADSB using callsign:" + String(SystemInformation.savedSettings.callsign));
	systemHardware.LEDSaftySwitchON();
	
	
	String resetCause = String("Watchdog reset cause:" + String(tempWDTCause));
	
	int firmwareMinor = round((SystemInformation.FIRMWARE_VERSION-(int)SystemInformation.FIRMWARE_VERSION)*100);	
	String firmwareVersion;
	if(firmwareMinor<10){
		firmwareVersion = String(String((int)SystemInformation.FIRMWARE_VERSION) + ".0" + String(firmwareMinor));
	}else{
		firmwareVersion = String(String((int)SystemInformation.FIRMWARE_VERSION) + "." + String(firmwareMinor));
	}	
	String startMSG = String("ADSB Initializing (ver. " + firmwareVersion + ") ");	

	// Init Auxiliary serial port:
	systemHardware.getSerialAUX()->begin(AUX_SERIAL_BAUDRATE);
	String auxSettings = String("ADSB Mavlink Baud@" + String(AUX_SERIAL_BAUDRATE));

	// Read the 128 bit serial number.
	//Serial.println("Chip unique serial number part 1:" + String(SystemInformation.SerialNumber1));
	//Serial.println("Chip unique serial number part 2:" + String(SystemInformation.SerialNumber2));
	//Serial.println("Chip unique serial number part 3:" + String(SystemInformation.SerialNumber3));
	//Serial.println("Chip unique serial number part 4:" + String(SystemInformation.SerialNumber4));
	//Serial.println("Chip unique serial number: \"" + String(SystemInformation.SerialNumber1) + String(SystemInformation.SerialNumber2) + String(SystemInformation.SerialNumber3) + String(SystemInformation.SerialNumber4)+"\"");
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
	
	// ADSB ID: "[BASE64CODE]"
	String base64MSG = String("ADSB ID:\"" + String(Telegram::base64_encode(data,SERIALNUMBER_SIZE).c_str()) + "\"");
	
	// Init the Radio protocol
//	RadioService = new RFService(systemHardware.getRadio(), &SystemInformation);
	RadioService.begin(systemHardware.getRadio(), &SystemInformation);
//	SerialProtocol = new PCProtocol(RadioService, Radio);

	String opperationMode;
	if(SystemInformation.savedSettings.isGroundstation){
		// Start the FrSky SPort library with polling using Serial port and only GPS2 second GPS:
		FrskySportInModuleBay.begin(systemHardware.getSerialFrSkySPort(), &FrskyGPS2);
		opperationMode = String("ADSB operating as groundstation");
	}else{
		// Start the FrSky SPort library with Serial port and the two sensors (GPS and Passthrought):
		FrskySport.begin(systemHardware.getSerialFrSkySPort(), &FrskyGPS, &FrskyPASS);		
		opperationMode = String("ADSB operating as transponder beacon");
	}
		
	// Start the Mavlink decoder and link it to FrSky GSP and Passthrough sensors for data update:
	MavlinkData.begin(systemHardware.getSerialAUX(), &FrskyGPS, &FrskyPASS);		

	// Set Timer 3 as 1 sec interrupt.
	startTimer3(1); // 1Hz
	//startTimer3(50); // 50Hz
	
	SystemInformation.state=STARTING_UP;
	systemHardware.PowerON(); // Ensure transponder keeps running from battery if external power is lost.
	systemHardware.PowerONGPSBackup(); // Enable backup power for GPS.
	SystemInformation.BatteryVoltage = systemHardware.getBatteryVoltage();
	SystemInformation.InputVoltage = systemHardware.getInputVoltage();
	SystemInformation.USBVoltage = systemHardware.getInputUSBVoltage();


	Serial.println(startMSG);
	Serial.println(resetCause);
	Serial.println(base64MSG);
	Serial.println(opperationMode);
	Serial.println(callsign);
	Serial.println(auxSettings);
	FrskyPASS.setDataTextMSG((char*)startMSG.c_str(), 7);  // ADSB Initializing (ver. 2.00)
	FrskyPASS.setDataTextMSG((char*)opperationMode.c_str(), 4);  // ADSB Mode (Groundstation or Beacon).	
	FrskyPASS.setDataTextMSG((char*)base64MSG.c_str(), 7); // ADSB ID: "[BASE64CODE]"
	FrskyPASS.setDataTextMSG((char*)callsign.c_str(), 7);  // ADSB using callsign [callsign]
	FrskyPASS.setDataTextMSG((char*)auxSettings.c_str(), 7); // ADSB Mavlink Baud@115200
	
	serviceInterface.begin(&SystemInformation);
	
//	systemHardware.getSerialAUX()->println("Debug start!");
//	delay(5000);
	Watchdog.enable(5000);
}

void loop() {

     //LowPowerTest();
/*
	 systemHardware.getSerialAUX()->println("Going to sleep!");
	 delay(100);
	 SystemInformation.SecondsSinceStart=20;
	 bool led = false;
	 systemHardware.PowerOFF();
	 do{
		Watchdog.reset();
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
		if(led){ systemHardware.LEDON(); led=false;}else{ systemHardware.LEDOFF();led=true;}
		__DSB();
		__WFI();
		SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;		 
	 }while(1);

	 
	 systemHardware.getSerialAUX()->println("Awake! -should not happen!");
	*/ 
	  
	do{
		One_second_Update();
		systemHardware.getGPS()->update();  // Function empty serial buffer and analyzes string.
		RadioService.Service();			    // Service the Radio module
		MavlinkData.service();				// Service the Mavlink data.
		BeaconService();				    // Time to make beacon for radio tx?
		if(serviceInterface.service()){
			Serial.println("Updating NVM parameters");
			SystemInformation.savedSettings.valid = true;
			storedConfiguration.write(SystemInformation.savedSettings);
		}
			
		
//		if(Serial.available()>0){
//			Serial.write(Serial.read());
//		}
			
		
		if(SystemInformation.SaftySwitchPushed == true){
			SystemInformation.state=POWER_OFF;
		}

		switch(SystemInformation.state)
		{
			case NORMAL: 
			{
				////// Below this line, code is executed fast!
				if(SystemInformation.SecondsBatteryLowCounter > 2){ // filter.
					SystemInformation.state=GET_READY_TO_RUN_ON_BATTERY;
				}else{
				    // normal fast loop.
//					SerialProtocol->Service(); // Comunincation to PC.
					if(SystemInformation.savedSettings.isGroundstation){
						FrskySportInModuleBay.send(); // FrskySport with pollling.					
					}else{
						FrskySport.send(); // Service the Serial for SPORT.
					}
				}			
			}
			break;
			 
			case GET_READY_TO_RUN_ON_BATTERY:
			{
				if(SystemInformation.SecondsBatteryLowCounter == 0){
					// Power is back!
					SystemInformation.state=NORMAL;
				}
				else{
					// check if it is time to go to battery 
					if(SystemInformation.SecondsBatteryLowCounter > SystemInformation.savedSettings.POWER_DOWN_DELAY){
						systemHardware.PowerONGPSBackup(); // Ensure backup power is enabled for GPS.
						SystemInformation.GPSActiveCounter=0; // ensure GPS active counter is reset.
						SystemInformation.state=RUNNING_ON_BATTERY_GPS_ON;
					}else{
//						SerialProtocol->Service(); // Comunincation to PC;
						FrskySport.send(); // Service the Serial for SPORT.
					}
				}
			} 
			break;
			
			case RUNNING_ON_BATTERY_GPS_ON:
			{
				if(SystemInformation.InputVoltage > 4.3 && (!SystemInformation.SimulateRunningOnBattery)){ // in debug mode force running on battery mode.
					SystemInformation.state=STARTING_UP;
				}else if(SystemInformation.BatteryVoltage <= 3.0){
					SystemInformation.state=POWER_OFF;
				}else{
					GoToSleep(); // Sleep until 1 sec interrupt will wake us up.
					if(++SystemInformation.GPSActiveCounter > SystemInformation.savedSettings.GPS_ON_TIME){ // power off GPS after 1 min.
						SystemInformation.GPSActiveCounter=0;
						delay(2000); // busy wait while GPS serial gets time to receive data from GPS (Unable in sleep mode).
						systemHardware.getGPS()->update();  // Service the GPS.
						systemHardware.PowerOFFGPS();// Turn OFF GPS main power.
						SystemInformation.state=RUNNING_ON_BATTERY_GPS_OFF;
					}
				}
			}
			break;

			case RUNNING_ON_BATTERY_GPS_OFF:
			{
				if( ((SystemInformation.InputVoltage > 4.3) || (SystemInformation.USBVoltage > 2.0 )) && (!SystemInformation.SimulateRunningOnBattery)){ // in debug mode force running on battery mode.
					SystemInformation.state=STARTING_UP;
					}else if(SystemInformation.BatteryVoltage <= 3.0){
						SystemInformation.state=POWER_OFF;
					}else{
						GoToSleep(); // Sleep until 1 sec interrupt will wake us up.
						if(++SystemInformation.GPSActiveCounter > SystemInformation.savedSettings.GPS_OFF_TIME){ // Turn on GPS if off time is meet.
							SystemInformation.GPSActiveCounter=0;
							systemHardware.PowerONGPS();// Turn OFF GPS main power.
							SystemInformation.state=RUNNING_ON_BATTERY_GPS_ON;
						}
				}
			}
			break;


			case POWER_OFF:
			{
				systemHardware.PowerOFF(); 
				Serial.println("ADSB Status: Power OFF - Please remove power.");		
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
			}
			break;
			 
			default:
				SystemInformation.state = STARTING_UP;
			break;
		}
	}while(1); 
}

void GoToSleep(void){
	RadioService.PowerDown();
	systemHardware.auxSerialPowerDown();
	systemHardware.PowerOFFFrSkySmartPort();

	if(SystemInformation.SecondsSinceStart > 10){ // Don't sleep the first 10 seconds after power on.
		USBDevice.detach();
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		
		SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
		__DSB();
		__WFI();
		SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
		// Wake up:
		USBDevice.attach();
	}

	systemHardware.PowerONFrSkySmartPort();
	systemHardware.auxSerialPowerUp();
	RadioService.WakeUp();	
}

void LowPowerTest(void){
	systemHardware.PowerOFFGPS(); // Turn off  GPS to save battery.
	systemHardware.PowerOFFGPSBackup(); // Ensure backup power is enabled for GPS.	
	systemHardware.LEDSaftySwitchOFF();
	systemHardware.PowerOFFFrSkySmartPort();
	systemHardware.LEDOFF();
	do
	{
		GoToSleep();
	}while(1);
}

// Ensure a beacon is transmitted every N second.
void BeaconService(void){
	if(SystemInformation.BeaconSecondCounter >= SystemInformation.savedSettings.BEACON_INTERVAL){ // SystemInformation.BeaconSecondCounter is counted up en Timer3 (1Hz) ISR.
//		SerialAUX->print("Time to make beacon message...");
		// Send a Standard beacon:
		SystemInformation.BeaconSecondCounter =  0; // Reset Beacon counter.
		if(SystemInformation.savedSettings.isGroundstation==false){
			// Serial print status:
			Serial.print("Transponder Status: Battery=" + String(SystemInformation.BatteryVoltage,2) + "V ");
			if(systemHardware.GetChargeState() == HIGH){
				Serial.print("(Complete)");
				}else{
				Serial.print("(Charging)");
			}
			Serial.println(" SaftySwitch:" + String(systemHardware.SaftySwitchPushed()) + "|" + String(SystemInformation.SafteSwitchPushedTimer) + "|" + String(SystemInformation.SaftySwitchFirstTimePushed));
			
			// update system with latest GPS values;
//			Serial.println("Sending Beacon: " + String(systemHardware.getGPS()->getLatitude()) + "," + String(systemHardware.getGPS()->getLongitude()));
			SystemInformation.UTCTime = systemHardware.getGPS()->getUTCTime();
			SystemInformation.Latitude = systemHardware.getGPS()->getLatitude();
			SystemInformation.Longitude = systemHardware.getGPS()->getLongitude();
			SystemInformation.NumberOfSat = systemHardware.getGPS()->getNumberOfSat();
			SystemInformation.Fix = systemHardware.getGPS()->getFix();
			SystemInformation.hdop = systemHardware.getGPS()->getHDOP();
			RadioService.SendBeacon();
		}else{
//			SerialAUX->println("Im groundstation, NoPing!");
		}
//		SerialAUX->println("Done!");	
	}else{ // see if there are any messages from radio to Mavlink / FrSky.
		
		transponderData_t *temp = RadioService.getRadioText();		
		if(temp->dataReady){
			FrskyPASS.setDataTextMSG(temp->data, temp->servirity);

			// Make Mavlink ADSB msg:
	//		systemHardware.getSerialAUX()->println("Data ready for Mavlink:");
			mavlinkSendADSB(temp->latitude,temp->longitude,temp->servirity, SystemInformation.savedSettings.callsign);
			FrskyGPS2.setData(temp->latitude,temp->longitude,0,0,0,0,0,0,0,0,0); // Set second GPS data on SPORT (for module bay) when groundstation.
			temp->dataReady=false;
		}
	}	
}

void One_second_Update(void){
	//// only every second (check status)
	while(SystemInformation.SecondCounter){
		Watchdog.reset();
		SystemInformation.SecondCounter--;
		SystemInformation.BatteryVoltage = systemHardware.getBatteryVoltage();
		SystemInformation.InputVoltage = systemHardware.getInputVoltage();
		SystemInformation.USBVoltage = systemHardware.getInputUSBVoltage();				
		
		if( ((SystemInformation.InputVoltage <= 4.3) && (SystemInformation.USBVoltage <=2 )) || (SystemInformation.SimulateRunningOnBattery)){
			if(SystemInformation.SecondsBatteryLowCounter < 255){
				SystemInformation.SecondsBatteryLowCounter++;
			}
		}else{
			SystemInformation.SecondsBatteryLowCounter=0;
		}			
	
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
		
		if(SystemInformation.SecondsSinceStart > 5 && !SystemInformation.SaftySwitchFirstTimePushed){
			systemHardware.LEDSaftySwitchOFF();
		}
		
		if(systemHardware.getGPS()->dataIsValid()){
			if(SystemInformation.gpsValidSentOnlyOnce == false){
				String text = String("ADSB local GPS sensor detected");
				FrskyPASS.setDataTextMSG((char*)text.c_str(), 7);		
				Serial.println(text);		
				SystemInformation.gpsValidSentOnlyOnce=true;
			}
		}
		/*
		Serial.print("Transponder Status: Battery=" + String(SystemInformation.BatteryVoltage,2) + "V ");
		if(systemHardware.GetChargeState() == HIGH){
			Serial.print("(Complete)");		
		}else{
			Serial.print("(Charging)");					
		}
		Serial.println(" SaftySwitch:" + String(systemHardware.SaftySwitchPushed()) + "|" + String(SystemInformation.SafteSwitchPushedTimer) + "|" + String(SystemInformation.SaftySwitchFirstTimePushed));
		*/
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
	msgADSB.altitude = 0;
	msgADSB.heading = 0;
	msgADSB.hor_velocity = 0;
	msgADSB.ver_velocity = 0;
	msgADSB.flags = 49; // Valid Coords, Callsign and Squawk.  319; 
	
	if(serverity <= 4){
		msgADSB.squawk = 7700;		
	}else{
		msgADSB.squawk = 1800;		
	}	
	msgADSB.altitude_type = 0; // BARO
	
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
	
	
void SerialPrintHEX(int data){
	Serial.print("0x");
	if(data < 16){
		Serial.print("0");
	}
	Serial.print(data,HEX);
}



// Not part of Class but needed to override Ardunio functions:

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
