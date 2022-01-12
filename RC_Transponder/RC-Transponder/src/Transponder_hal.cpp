/*
 * Transponder_hal.cpp
 *
 * Created: 12/01/2022 11.43.36
 *  Author: klo
 */ 

#include "Transponder_hal.h"

Transponder_hal::Transponder_hal(){}

void Transponder_hal::begin(){

	// Init Pins for Power:
	setPin(powerOnPin,OUTPUT,LOW);

	// Init pins for analog voltage read:  PRICE ~70uA (or not)
	setPin(analogVinPin,INPUT,LOW);
	setPin(analogVbatPin,INPUT,LOW);
	analogReference(AR_INTERNAL1V0); // Use internal 1V as voltage reference.

	// Init pins for GPS: Price: ~0uA
	setPin(GPSPowerOnPin,OUTPUT,LOW);
	setPin(GPSBackupPowerPin,OUTPUT,HIGH);

	// Init for FRSKY SPORT:
	setPin(fryskyInvertPin,OUTPUT,HIGH);

	// Init LED 2 (nr. never used)
	setPin(led2Pin,OUTPUT,HIGH);

	// Init SPI and pins used for Radio price 0uA
	setPin(chipSelectPin,OUTPUT,LOW);
	setPin(resetPin,OUTPUT,LOW);
	setPin(rxEnPin,OUTPUT,HIGH);		 // RX enabled by default.
	setPin(txEnPin,OUTPUT,LOW);
	setPin(busyPin,INPUT,LOW);
	setPin(dio1Pin,INPUT,LOW);

	// Init Safety switch and LED:
	setPin(SaftySwitchPin,INPUT,LOW);
	setPin(SaftyLEDPin,OUTPUT,LOW);

	pinMode(analogVin5VPin, INPUT);

	// Not used pins
	setPin(pa11Pin,OUTPUT,LOW);

	#if PCB_VERSION == 11
	_serialSBUS = new Uart(&sercom4, sBUSRXPin, sBUSRXPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);   // Create the new UART instance for the GPS module
	setPin(sBUSInvertPin,OUTPUT,LOW);
	setPin(Baro_chipSelectPin,OUTPUT,HIGH); // Never implemented.

	// unused pins set to input low:
	setPin(pa13Pin,OUTPUT,LOW);

	#elif PCB_VERSION == 12
	// PCB-12 no longer has RCin or SBUS. (never implemented).

	//this->_serialPC = SerialUSB;

	setPin(powerOnClk,OUTPUT,LOW);	// new power pins (for external flip-flop.
	setPin(chargeState,INPUT,LOW);  // Charge state input.
	digitalWrite(chargeState, OUTPUT);  //  when OUT=1 in INPUT mode, pull resistor is pull-up, else pull-down. ( This will set pull-up when in input mode).
	//PORT->Group[g_APinDescription[chargeState].ulPort].PINCFG[g_APinDescription[chargeState].ulPin].bit.PULLEN = 1; //Enable pull up.

	setPin(GPSResetPin,OUTPUT,HIGH);

	// unused pins set to input low:
	setPin(pa04Pin,OUTPUT,LOW);
	setPin(pa18Pin,OUTPUT,LOW);
	#endif

	delay(2000);

	// Now that the GPS has been power cycled, we know it is talking 9600.
	this->_serialGPS = new Uart(&sercom2, GPSRxPin, GPSTxPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);   // Create the new UART instance for the GPS module
	pinPeripheral(GPSTxPin, PIO_SERCOM_ALT); //Assign TX function to GPS TX pin.
	pinPeripheral(GPSRxPin, PIO_SERCOM_ALT); //Assign RX function to GPS RX pin.

	// GPS INIT;
	this->_GPS = new GPSL80Lite();
	PowerONGPS(); // Turn on GPS. price 31uA
	ResetGPS();
	this->_GPS->init(this->_serialGPS, 57600);

	// Make SerialAUX Uart:
	this->_serialAUX = new Uart(&sercom5, auxRXPin, auxTXPin, SERCOM_RX_PAD_3, UART_TX_PAD_2);   // Create the new UART instance for the AUX serial port
	auxSerialPowerUp();	
	
	// Init Frsky Smart port:
	this->_serialfrskySPort = new Uart(&sercom3, fryskySmartPortRXPin, fryskySmartPortTXPin, SERCOM_RX_PAD_3, UART_TX_PAD_2);   // Create the new UART instance for the Frsky SPORT module
	this->_serialfrskySPort->begin(57600);  //Baudrate for frskys SPORT protocol.
	pinPeripheral(fryskySmartPortRXPin, PIO_SERCOM_ALT); //Assign RX function to pin.
	pinPeripheral(fryskySmartPortTXPin, PIO_SERCOM_ALT); //Assign TX function to pin.
	
	// Init E28 Radio module (SX1280 chip): When in sleep mode (all data retained, it costs ~70uA.
	this->_radio = new E28_2G4M20S(chipSelectPin,resetPin,busyPin,dio1Pin,0,0,txEnPin,rxEnPin, led2Pin);
	//	attachInterrupt(dio1Pin, Radio_isr, RISING); // Hack in mkr1000 Variant.h to add EXTERNAL_INTERRUPT 15 on pin 30 or EXTERNAL_INT_3 on pin 25 (PCB_VERSION 11)
}

void Transponder_hal::setPin(int pin, int mode, int initOutput){
	if(mode == OUTPUT){
		digitalWrite(pin, initOutput);
		pinMode(pin, OUTPUT);
		PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].bit.INEN = 0; // Disable input buffer for Pins used at output to save power.
	}else{ // INPUT
		pinMode(pin, INPUT);
	}
}


E28_2G4M20S * Transponder_hal::getRadio(void){
	return this->_radio;
}

Uart * Transponder_hal::getSerialPC(void){
	return this->_serialPC;
}	

Uart * Transponder_hal::getSerialAUX(void){
	return this->_serialAUX;
}

Uart * Transponder_hal::getSerialFrSkySPort(void){
	return this->_serialfrskySPort;
}

Uart * Transponder_hal::getSerialGPS(void){
	return this->_serialGPS;
}

Uart * Transponder_hal::getSerialSBUS(void){
	return this->_serialSBUS;
}

GPSL80Lite * Transponder_hal::getGPS(void){
	return this->_GPS;
}


float Transponder_hal::getBatteryVoltage(void){
	#if PCB_VERSION == 10
	return ((((float)analogRead(analogVbatPin))*4.9)/1024.0);
	#elif PCB_VERSION == 11
	return ((((float)analogRead(analogVbatPin))*5.7)/1024.0); // R1=470k R2=100k; Vinput=R2/(R1+R2)*Vbat = 100k/570k*Vbat----- Vbat=AnalogRead/1024*1V*(570k/100k) = AR/1024*5.7
	#elif PCB_VERSION == 12
	return ((((float)analogRead(analogVbatPin))*5.99)/1024.0); // R1=499k R2=100k; Vinput=R2/(R1+R2)*Vbat = 100k/599k*Vbat----- Vbat=AnalogRead/1024*1V*(599k/100k) = AR/1024*5.99
	#else
	#error What the fuck
	#endif
}

float Transponder_hal::getInputVoltage(void){
	return ((((float)analogRead(analogVinPin))*11)/1024.0);
}


float Transponder_hal::getInput5VVoltage(void){
	#if PCB_VERSION == 10
	return 0;
	#elif PCB_VERSION == 11
	return ((((float)analogRead(analogVin5VPin))*12)/1024.0); // R1=110k R2=10k; Vinput=R2/(R1+R2)*Vin_5v = 10k/120k*Vbat----- Vin_5v=AnalogRead*1V/1024*(120k/10k) = AR/1024*12
	#elif PCB_VERSION == 12
	return ((((float)analogRead(analogVin5VPin))*11)/1024.0); // R1=100k R2=10k; Vinput=R2/(R1+R2)*Vin_5v = 10k/110k*Vbat----- Vin_5v=AnalogRead*1V/1024*(110k/10k) = AR/1024*11
	#endif
	
	return 0;
}


void Transponder_hal::PowerON(void){
	digitalWrite(powerOnPin, HIGH); // LOW = OFF
	
	#if PCB_VERSION == 12 // use flip-flop
	delay(5);
	digitalWrite(powerOnClk, LOW);
	delay(5);
	digitalWrite(powerOnClk, HIGH);
	delay(5);
	digitalWrite(powerOnClk, LOW);
	
	delay(5);
	digitalWrite(powerOnPin, LOW);
	#endif
}

void Transponder_hal::PowerOFF(void){
	digitalWrite(powerOnPin, LOW); // LOW = OFF
	
	#if PCB_VERSION == 12 // use flip-flop
	delay(5);
	digitalWrite(powerOnClk, LOW);
	delay(5);
	digitalWrite(powerOnClk, HIGH);
	delay(5);
	digitalWrite(powerOnClk, LOW);
	#endif
}

void Transponder_hal::PowerONGPS(void){
	digitalWrite(GPSPowerOnPin, LOW);  // High=Off, Low= On
}

void Transponder_hal::PowerOFFGPS(void){
	digitalWrite(GPSPowerOnPin, HIGH);  // High=Off, Low= On
}

void Transponder_hal::PowerONGPSBackup(void){
	digitalWrite(GPSBackupPowerPin, HIGH);  // High=On, Low= Off
}

void Transponder_hal::PowerOFFGPSBackup(void){
	digitalWrite(GPSBackupPowerPin, LOW);  // High=On, Low= Off
}

void Transponder_hal::ResetGPS(void){
	// pull reset for 15ms.
	#if PCB_VERSION == 12 // drive GPS Reset pin
	digitalWrite(GPSResetPin, LOW); // active low
	delay(15);
	digitalWrite(GPSResetPin, HIGH); // active low
	#endif
}


void Transponder_hal::LEDSaftySwitchON(void){
	digitalWrite(SaftyLEDPin, LOW);
}

void Transponder_hal::LEDSaftySwitchOFF(void){
	digitalWrite(SaftyLEDPin, HIGH);
}

bool Transponder_hal::SaftySwitchPushed(void){
	// filter
	int count=0;
	
	for(int b=0;b<10;b++){
		if(digitalRead(SaftySwitchPin) == HIGH){
			count++;
			}else{
			return false; // one sample was 0 then return false. (not pushed).
		}
		delay(2);
	}
	
	return true; // only get here if all 10 times sampled are high. (20ms filter time)
}


void Transponder_hal::LEDON(void){
	digitalWrite(led2Pin, HIGH);
}

void Transponder_hal::LEDOFF(void){
	digitalWrite(led2Pin, LOW);
}

int Transponder_hal::GetChargeState(void){
	#if PCB_VERSION == 11
	return 1;
	#elif PCB_VERSION == 12
	return digitalRead(chargeState);
	#endif
}

void Transponder_hal::auxSerialPowerDown(void){
	pinMode(auxRXPin, INPUT_PULLDOWN);
}

void Transponder_hal::auxSerialPowerUp(void){
	pinMode(auxRXPin, INPUT_PULLUP);
}

void Transponder_hal::PowerOFFFrSkySmartPort(void){
	digitalWrite(fryskyInvertPin, LOW);
	pinMode(fryskySmartPortRXPin, INPUT_PULLDOWN);
}

void Transponder_hal::PowerONFrSkySmartPort(void){
	digitalWrite(fryskyInvertPin, HIGH);
	pinMode(fryskySmartPortRXPin, INPUT_PULLUP);
}




