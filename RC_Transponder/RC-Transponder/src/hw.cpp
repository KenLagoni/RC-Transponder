/*
 * hw.cpp
 *
 * Created: 16-Dec-18 10:28:46
 *  Author: Kenneth
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

// Chip 128 bit unique serial number from flash
uint32_t SerialNumber1, SerialNumber2, SerialNumber3, SerialNumber4;

void hwInit() {
	

	// Init Pins for Power:
	pinMode(powerOnPin, OUTPUT);
	digitalWrite(powerOnPin, LOW); // LOW = OFF

	// Init pins for analog voltage read:  PRICE ~70uA (or not)
	pinMode(analogVrefPin, INPUT);
	pinMode(analogVinPin, INPUT);
	pinMode(analogVbatPin, INPUT);
	analogReference(AR_INTERNAL1V0); // Use internal 1V as voltage reference.
	//analogReference(AR_INTERNAL); // Use internal 1V as voltage reference.
	
	// Init pins for GPS: Price: ~0uA
	pinMode(gPSPowerOnPin, OUTPUT);
	digitalWrite(gPSPowerOnPin, LOW);  // High=Off, Low= On
	delay(2000);	

	// Now that the GPS has been power cycled, we know it is talking 9600.
	SerialGPS = new Uart(&sercom2, GPSRxPin, GPSTxPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);   // Create the new UART instance for the GPS module

	// Init for FRSKY SPORT:
	pinMode(fryskyInvertPin, OUTPUT);    // THIS LINE MAKES COM20 UNIT STALL HERE!!!!
	digitalWrite(fryskyInvertPin, HIGH);


	
	// Init pins for LED1 and 2
	// LED1 is used as input for DI0 interrupt from E28!
	//  pinMode(led1Pin, OUTPUT);
	//  digitalWrite(led1Pin, LOW);  // Low=off, High=On
	
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
	
	}
		
	// Attach the interrupt handler to the SERCOM
	void SERCOM2_Handler()
	{
		SerialGPS->IrqHandler();
	}

	void SERCOM3_Handler()
	{
		SerialfrskySPort->IrqHandler();
	}
	
	void SERCOM4_Handler()
	{
		SerialSBUS->IrqHandler();
	}
	
	void SERCOM5_Handler()
	{
		SerialAUX->IrqHandler();
	}
	

	float getBatteryVoltage(void)
	{
//		return ((((float)analogRead(analogVbatPin))*3.0)/512.0);
		return ((((float)analogRead(analogVbatPin))*4.9)/1024.0);
//		return 4;
	}

	float getInputVoltage(void)
	{
//		return ((((float)analogRead(analogVinPin))*3.0)/512.0);
		return 0;
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

void ReadSerialNumberFromChipFlash(void){
	volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
	SerialNumber1 = *ptr1;
	volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
	SerialNumber2 = *ptr;
	ptr++;
	SerialNumber3 = *ptr;
	ptr++;
	SerialNumber4 = *ptr;
}