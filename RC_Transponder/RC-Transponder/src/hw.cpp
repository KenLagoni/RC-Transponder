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

// Must be freed up in variant.cpp or alternative use Serial1 (defualt on mkrzero and uses same pins)
Uart *SerialAUX = NULL;
//Uart auxSerial(&sercom5, auxRXPin, auxTXPin, SERCOM_RX_PAD_3, UART_TX_PAD_2);                                // Create the new UART instance for auxillary serial.

void hwInit() {


	// Init Pins for Power:
	pinMode(powerOnPin, OUTPUT);
	digitalWrite(powerOnPin, LOW); // LOW = OFF

	// Init pins for analog voltage read:  PRICE ~70uA (or not)
	pinMode(analogVinPin, INPUT);
	pinMode(analogVbatPin, INPUT);
	analogReference(AR_INTERNAL1V0); // Use internal 1V as voltage reference.

	#if PCB_VERSION == 11
		pinMode(analogVin5VPin, INPUT);
	#endif

	
	// Init pins for GPS: Price: ~0uA
	pinMode(gPSPowerOnPin, OUTPUT);
	digitalWrite(gPSPowerOnPin, LOW);  // High=Off, Low= On
	#if PCB_VERSION == 11
		pinMode(GPSBackupPowerPin, OUTPUT);
		digitalWrite(GPSBackupPowerPin, HIGH);  // High= backup power on, LOW=Backup power off.
	#endif

	delay(2000);	

	// Now that the GPS has been power cycled, we know it is talking 9600.
	SerialGPS = new Uart(&sercom2, GPSRxPin, GPSTxPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);   // Create the new UART instance for the GPS module

	// Init for FRSKY SPORT:
	pinMode(fryskyInvertPin, OUTPUT);    // THIS LINE MAKES COM20 UNIT STALL HERE!!!!
	digitalWrite(fryskyInvertPin, HIGH);

	// Init LED 2
	pinMode(led2Pin, OUTPUT);
	digitalWrite(led2Pin, HIGH);  // Low=off, High=On

	
	// Init the RCin SBUS pins: price 0uA
	SerialSBUS = new Uart(&sercom4, sBUSRXPin, sBUSRXPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);   // Create the new UART instance for the GPS module
	pinMode(sBUSInvertPin, OUTPUT);
	digitalWrite(sBUSInvertPin, LOW);
	
	
	// Init SPI and pins used for Radio price 0uA
	pinMode(chipSelectPin, OUTPUT);
	digitalWrite(chipSelectPin, LOW);
	pinMode(resetPin, OUTPUT);
	digitalWrite(resetPin, LOW);
	pinMode(rxEnPin, OUTPUT);
	digitalWrite(rxEnPin, HIGH); // RX enabled by default.
	pinMode(txEnPin, OUTPUT);
	digitalWrite(txEnPin, LOW);
	pinMode(busyPin, INPUT);
	pinMode(dio1Pin, INPUT);

	// Init Baro
	#if PCB_VERSION == 11
		pinMode(Baro_chipSelectPin, OUTPUT);
		digitalWrite(Baro_chipSelectPin, HIGH); // Not active!
	#endif
	
	// Init Safety switch and LED:
	#if PCB_VERSION == 11
		pinMode(SaftySwitchPin, INPUT);
		pinMode(SaftyLEDPin, OUTPUT);
		digitalWrite(SaftyLEDPin, LOW);
	#endif
	
	// Make SerialAUX Uart:
	//SerialAUX = new Uart(&sercom5, auxRXPin, auxTXPin, SERCOM_RX_PAD_3, UART_TX_PAD_2);   // Create the new UART instance for the AUX serial port
	
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
	#else
		#error What the fuck
	#endif	
}

float getInputVoltage(void){
	#if PCB_VERSION == 10
		return 0;
	#elif PCB_VERSION == 11 
		return ((((float)analogRead(analogVinPin))*11)/1024.0); 
	#endif	
}

float getInput5VVoltage(void){
	#if PCB_VERSION == 11
		return ((((float)analogRead(analogVin5VPin))*11)/1024.0);
	#endif
}

void PowerON(void){
	digitalWrite(powerOnPin, HIGH); // LOW = OFF
}

void PowerOFF(void){
	digitalWrite(powerOnPin, LOW); // LOW = OFF
}

void PowerONGPS(void){
	digitalWrite(gPSPowerOnPin, LOW);  // High=Off, Low= On
}
	
void PowerOFFGPS(void){
	digitalWrite(gPSPowerOnPin, HIGH);  // High=Off, Low= On
}

#if PCB_VERSION == 11
	void PowerONGPSBackup(void){
		digitalWrite(GPSBackupPowerPin, HIGH);  // High=On, Low= Off
	}

	void PowerOFFGPSBackup(void){
		digitalWrite(GPSBackupPowerPin, LOW);  // High=On, Low= Off
	}

#endif