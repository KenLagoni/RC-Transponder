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
#include "RFProtocol.h"
#include "PCProtocol.h"

// Radio
#include "E28-2G4M20S.h"

// for SPORT
#include "FrSkySportSensor.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"

void HandelSerial(void);
void LowPowerTest(void);
void Do_ground_station_loop(void);
void GoToSleep(void);
String base64_encode(byte[], int);

// System information
SystemInformation_t SystemInformation;

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
RFProtocol *RadioProtocol = NULL;
PCProtocol *SerialProtocol = NULL;

// $GPGGA,193648.000,5550.0838,N,01224.0718,E,2,8,0.96,32.7,M,41.5,M,0000,0000*62
GPSL80Lite *GPS = NULL; // GPS class, defined in GPSL80.h
GpsDataLite *GPSData = NULL;  // GPGGA GPS data:

//ISR for radio
void Radio_isr(void){
//	Serial.println("Time:" + String(millis()));
  //Radio->HandleIRQ(); // The radio module has something for us.
  RadioProtocol->IRQHandler();
}

void setup() {	
	hwInit(); // Setup all pins according to hardware.
	
	delay(5000); // Time to get USB bootloader ready.
	
	// Init USB serial debug /setup:
	Serial.begin(115200);
	delay(500);
	Serial.println("Starting RC Transponder ver. " + String((int)SystemInformation.FIRMWARE_VERSION) + "." + String((int)((SystemInformation.FIRMWARE_VERSION-((int)SystemInformation.FIRMWARE_VERSION))*100)));
	
	// Read and store the 128 bit serial number.
	ReadSerialNumberFromChipFlash();
	Serial.println("Chip unique serial number part 1:" + String(SerialNumber1));
	Serial.println("Chip unique serial number part 2:" + String(SerialNumber2));
	Serial.println("Chip unique serial number part 3:" + String(SerialNumber3));
	Serial.println("Chip unique serial number part 4:" + String(SerialNumber4));
	Serial.println("Chip unique serial number: \"" + String(SerialNumber1) + String(SerialNumber2) + String(SerialNumber3) + String(SerialNumber4)+"\"");
	#define SERIALNUMBER_SIZE 16
	uint8_t data[SERIALNUMBER_SIZE];
	data[0] = (byte)((SerialNumber1 >> 24) & 0xFF);
	data[1] = (byte)((SerialNumber1 >> 16) & 0xFF);
	data[2] = (byte)((SerialNumber1 >> 8) & 0xFF);
	data[3] = (byte)(SerialNumber1 & 0xFF);
	data[4] = (byte)((SerialNumber2 >> 24) & 0xFF);
	data[5] = (byte)((SerialNumber2 >> 16) & 0xFF);
	data[6] = (byte)((SerialNumber2 >> 8) & 0xFF);
	data[7] = (byte)(SerialNumber2 & 0xFF);
	data[8] = (byte)((SerialNumber3 >> 24) & 0xFF);
	data[9] = (byte)((SerialNumber3 >> 16) & 0xFF);
	data[10] = (byte)((SerialNumber3 >> 8) & 0xFF);
	data[11] = (byte)(SerialNumber3 & 0xFF);
	data[12] = (byte)((SerialNumber4 >> 24) & 0xFF);
	data[13] = (byte)((SerialNumber4 >> 16) & 0xFF);
	data[14] = (byte)((SerialNumber4 >> 8) & 0xFF);
	data[15] = (byte)(SerialNumber4 & 0xFF);
	Serial.println("Chip unique serial number in Base64 encode:\"" + base64_encode(data,SERIALNUMBER_SIZE) +"\"");
	
	
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

	RadioProtocol = new RFProtocol(Radio, GPSData, &SystemInformation);
	attachInterrupt(dio1Pin, Radio_isr, RISING); // Hack in mkr1000 Variant.h to add EXTERNAL_INTERRUPT 15 on pin 30 or EXTERNAL_INT_3 on pin 25 (PCB_VERSION 11)
	
	//SerialProtocolINST = new SerialProtocol(RadioProtocol);
	
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
	
	SystemInformation.state=STARTING_UP;
	/*
	if(isGroundStation){
		Serial.println("I am a groundstation");
		
	}else{
		Serial.println("I am a RC transponder");
	}
	*/
	PowerON(); // Ensure transponder keeps running from battery if external power is lost.
	PowerONGPSBackup(); // Enable backup power for GPS.
	SystemInformation.BatteryVoltage = getBatteryVoltage();
}

void Recharge(void){
		PowerOFFGPS(); // Turn off  GPS to save battery.
		//Radio->Sleep(); // Put radio to sleep to save power.
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
		if(SystemInformation.state == NORMAL)
		{	
			//// only every second (check status)
			while(SystemInformation.SecondCounter){
//				Serial.println("Seccond passed!");
//				digitalWrite(led2Pin, HIGH);
				SystemInformation.SecondCounter--;
	
				SystemInformation.BatteryVoltage = getBatteryVoltage();
	
				// Measure all stuff here:
				if(getInputVoltage() <= 4.3){
					SystemInformation.state=GET_READY_TO_RUN_ON_BATTERY;				
				}
		
			
				// Update the FrSky GPS emulator with the latest values from the GPS. (GPS Lite needs to be updated to read $GPRMC in order to get speed, cog and date information:
				FrskyGPS.setData(GPSData->LatitudeDecimal, GPSData->LongitudeDecimal,GPSData->Altitude,0,0,0,0,0,GPSData->UTC_hour,GPSData->UTC_min,GPSData->UTC_sec);		
				/*
				if(!BeaconService()){ // Transmit beacon every N second.	
					 // Enter here when no beacon is sent...
					 			
					 // Request transponder data from ID 2: (debug)
					 // Telegram_MSG_1 msg = Telegram_MSG_1(2,UNIT_ID,MSG_Transponder_Data);
					 // Radio->SendPackage(msg.Payload, msg.GetPayloadLength());
				}		*/	

			}
	
			////// Below this line, code is executed fast! 
//			HandelSerial();	 // Communication via USB (only if used as groundstation
			SerialProtocol->Service();
//			HandelRadio();  // Read new messages and reply as needed.  
			GPS->update();  // Function empty serial buffer and analyzes string.
			// RCin->read();  // SBUS, PPM or PWM. 
			FrskySport.send(); // Service the Serial for SPORT. 	
			
		}else if(SystemInformation.state == GET_READY_TO_RUN_ON_BATTERY){ // Power off GPS
			PowerONGPSBackup(); // Ensure backup power is enabled for GPS.
			PowerOFFGPS();// Turn OFF GPS.
			SystemInformation.state=RUNNING_ON_BATTERY;
		}else if(SystemInformation.state == RUNNING_ON_BATTERY){
			//BeaconService();
			
			if(SystemInformation.BatteryVoltage > 4.3){
				SystemInformation.state=STARTING_UP;
			}else if(SystemInformation.BatteryVoltage <= 3.0){
					PowerOFF(); // No more battery left, power off.
					do{}while(1);
			}else{
				GoToSleep(); // Sleep until 1 sec interrupt will wake us up.		
				SystemInformation.BatteryVoltage = getBatteryVoltage();
			}
			
		}else if(SystemInformation.state == STARTING_UP){
			PowerONGPS();// Turn on GPS. 			
			
			// Set the radio to RX mode without timeout.
			SystemInformation.SecondCounter=0; // reset second counter.
			SystemInformation.state=NORMAL;
		}

	}while(1); 
  
}

void GoToSleep(void){
	RadioProtocol->PowerDown();
	USBDevice.detach();
	digitalWrite(led2Pin, LOW);
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	USBDevice.attach();
	digitalWrite(led2Pin, HIGH);
	RadioProtocol->WakeUp();
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
/*

// Ensure a beacon is transmitted every N second.
bool BeaconService(void){
	if(SystemStatus.BeaconSecondCounter == 5){
		// Send a Standard beacon:
		SystemStatus.BeaconSecondCounter =  0; // Reset Beacon counter.

		RadioProtocol->SendBeacon();

		//Serial.println("Send Beacon!");
/*
		// Make beacon msg
		float pressure=0;
		float groundspeed=0;
		
		Telegram_MSG_1 msg = Telegram_MSG_1(SerialNumber1, SerialNumber2, SerialNumber3, SerialNumber4,
											(uint32_t)GPSData->UTCTime, GPSData->Latitude, GPSData->Longitude,
											GPSData->NumberOfSatellites, GPSData->FixDecimal, (state==RUNNING_ON_BATTERY),
											pressure, groundspeed,								
											SecondCounterSinceLasteGroundStationContact, BatteryVoltage, FIRMWARE_VERSION, PCB_VERSION, NumberOfBeaconsToRelay);																					
		
		do{	} while(!Radio->IsIdle());	// Ensure we wait for other TX job to finish first.													
		
		Radio->SendPackage(msg.GetRadioMSG(), msg.GetRadioMSGLength());
		*/
		//return true;	
//	}	
//	return false;
//}



  static String base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  String base64_encode(byte bytes_to_encode[], int in_len)
  {
	  String ret = "";
	  int i = 0;
	  int j = 0;
	  byte char_array_3[3];
	  byte char_array_4[4];
	  int place = 0;

	  while (in_len-- > 0) {
		  char_array_3[i++] = bytes_to_encode[place++];
		  if (i == 3) {
			  char_array_4[0] = (byte)((char_array_3[0] & 0xfc) >> 2);
			  char_array_4[1] = (byte)(((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4));
			  char_array_4[2] = (byte)(((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6));
			  char_array_4[3] = (byte)(char_array_3[2] & 0x3f);

			  for(i = 0; (i<4) ; i++)
				 ret += base64_chars[char_array_4[i]];
				 
			  i = 0;
		  }
	  }

	  if (i > 0) {
		  for(j = i; j< 3; j++)
			char_array_3[j] = 0;

		  char_array_4[0] = (byte)(( char_array_3[0] & 0xfc) >> 2);
		  char_array_4[1] = (byte)(((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4));
		  char_array_4[2] = (byte)(((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6));

		  for (j = 0; (j<i + 1); j++)
			ret += base64_chars[char_array_4[j]];

		  while((i++ < 3))
			ret += '=';

	  }

	  return ret;

  }