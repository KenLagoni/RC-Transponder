/*
	hw.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
#include <Arduino.h>
#include "wiring_private.h"
#include "hw.h"

Uart *SerialGPS = NULL;
//Uart gpsSerial(&sercom2, GPSRxPin, GPSTxPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);                                // Create the new UART instance for the GPS module

Uart *SerialfrskySPort = NULL;
//Uart frskySPortSerial(&sercom3, fryskySmartPortRXPin, fryskySmartPortTXPin, SERCOM_RX_PAD_3, UART_TX_PAD_2); // Create the new UART instance for the frskySmartPort

Uart *SerialSBUS = NULL;
//Uart SBUSSerial(&sercom4, sBUSRXPin, sBUSRXPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);                             // Create the new UART instance for the SBUS (RC inputs)

// Must be freed up in variant.cpp or alternative use Serial1 (default on mkrzero and uses same pins)
Uart *SerialAUX = NULL;
//Uart auxSerial(&sercom5, auxRXPin, auxTXPin, SERCOM_RX_PAD_3, UART_TX_PAD_2);                                // Create the new UART instance for auxillary serial.

void hwInit() {

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
		SerialSBUS = new Uart(&sercom4, sBUSRXPin, sBUSRXPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);   // Create the new UART instance for the GPS module
		setPin(sBUSInvertPin,OUTPUT,LOW);
		setPin(Baro_chipSelectPin,OUTPUT,HIGH); // Never implemented.
		
		// unused pins set to input low:
		setPin(pa13Pin,OUTPUT,LOW);

	#elif PCB_VERSION == 12
		// PCB-12 no longer has RCin or SBUS. (never implemented).

		setPin(powerOnClk,OUTPUT,LOW);	// new power pins (for external flip-flop.		
		setPin(chargeState,INPUT,LOW);  // Charge state input.
		setPin(GPSResetPin,OUTPUT,HIGH);
				
		// unused pins set to input low:
		setPin(pa04Pin,OUTPUT,LOW);
		setPin(pa18Pin,OUTPUT,LOW);
	#endif

	delay(2000);

	// Now that the GPS has been power cycled, we know it is talking 9600.
	SerialGPS = new Uart(&sercom2, GPSRxPin, GPSTxPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);   // Create the new UART instance for the GPS module
	
	// Make SerialAUX Uart:
	SerialAUX = new Uart(&sercom5, auxRXPin, auxTXPin, SERCOM_RX_PAD_3, UART_TX_PAD_2);   // Create the new UART instance for the AUX serial port
}

void setPin(int pin, int mode, int initOutput){
	if(mode == OUTPUT){
		digitalWrite(pin, initOutput);  
		pinMode(pin, OUTPUT);
		PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].bit.INEN = 0; // Create the new UART instance for the GPS module

	}else{ // INPUT
		pinMode(pin, INPUT);		
	}
}

		
// Attach the interrupt handler to the SERCOM
void SERCOM2_Handler(){
	SerialGPS->IrqHandler();
}

void SERCOM3_Handler(){
	SerialfrskySPort->IrqHandler();
}
	
void SERCOM4_Handler(){
	SerialSBUS->IrqHandler();
}
	
void SERCOM5_Handler(){
	SerialAUX->IrqHandler();
}

float getBatteryVoltage(void){
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

float getInputVoltage(void){
	return ((((float)analogRead(analogVinPin))*11)/1024.0); 
}


float getInput5VVoltage(void){
	#if PCB_VERSION == 10
	return 0;
	#elif PCB_VERSION == 11
		return ((((float)analogRead(analogVin5VPin))*12)/1024.0); // R1=110k R2=10k; Vinput=R2/(R1+R2)*Vin_5v = 10k/120k*Vbat----- Vin_5v=AnalogRead*1V/1024*(120k/10k) = AR/1024*12
	#elif PCB_VERSION == 12
	 	return ((((float)analogRead(analogVin5VPin))*11)/1024.0); // R1=100k R2=10k; Vinput=R2/(R1+R2)*Vin_5v = 10k/110k*Vbat----- Vin_5v=AnalogRead*1V/1024*(110k/10k) = AR/1024*11
	#endif
	
	return 0;
}


void PowerON(void){
	digitalWrite(powerOnPin, HIGH); // LOW = OFF
	
	#if PCB_VERSION == 12 // use flip-flop
		delay(5);
		digitalWrite(powerOnClk, LOW); 
		delay(5);
		digitalWrite(powerOnClk, HIGH);
		delay(5);
		digitalWrite(powerOnClk, LOW); 
	#endif
}

void PowerOFF(void){
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

void PowerONGPS(void){
	digitalWrite(GPSPowerOnPin, LOW);  // High=Off, Low= On
	
	#if PCB_VERSION == 12 // drive GPS Reset pin
		delay(5);
		digitalWrite(GPSResetPin, HIGH); // active low
	#endif
}
	
void PowerOFFGPS(void){
	digitalWrite(GPSPowerOnPin, HIGH);  // High=Off, Low= On

	#if PCB_VERSION == 12 // drive GPS Reset pin
		digitalWrite(GPSResetPin, LOW); // active low
	#endif
}

void PowerONGPSBackup(void){
	digitalWrite(GPSBackupPowerPin, HIGH);  // High=On, Low= Off
}

void PowerOFFGPSBackup(void){
	digitalWrite(GPSBackupPowerPin, LOW);  // High=On, Low= Off
}

void LEDSaftySwitchON(void){
	digitalWrite(SaftyLEDPin, LOW);
}

void LEDSaftySwitchOFF(void){
	digitalWrite(SaftyLEDPin, HIGH);
}

bool SaftySwitchPushed(void){
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


void LEDON(void){
	digitalWrite(led2Pin, HIGH);		
}

void LEDOFF(void){
	digitalWrite(led2Pin, LOW);
}
