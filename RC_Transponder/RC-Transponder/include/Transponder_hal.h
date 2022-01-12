/*
 * Transponder_hal.h
 *
 * Created: 12/01/2022 11.43.55
 *  Author: klo
 */ 


#ifndef TRANSPONDER_HAL_H_
#define TRANSPONDER_HAL_H_

#include <Arduino.h>
#include "wiring_private.h" // Needed for pinPeripheral function.
#include "E28-2G4M20S.h"    // Radio
#include "GPSL80Lite.h"     // Module for GPS Class


#define PCB_VERSION 12
#if PCB_VERSION == 10
#error PCB version 10 is no longer supported.
#elif  PCB_VERSION == 11
#elif  PCB_VERSION == 12
#else
	#error No valid PCB hardware selected.
#endif

class Transponder_hal
{
	// Public functions to be used on all Messages
	public:
	Transponder_hal();
	void begin(void);	
	Uart * getSerialPC(void);
	Uart * getSerialAUX(void);
	Uart * getSerialFrSkySPort(void);
	Uart * getSerialGPS(void);
	Uart * getSerialSBUS(void);
	E28_2G4M20S * getRadio(void);
	GPSL80Lite * getGPS(void); // GPS class, defined in GPSL80.h	// $GPGGA,193648.000,5550.0838,N,01224.0718,E,2,8,0.96,32.7,M,41.5,M,0000,0000*62
	
	
	float getBatteryVoltage(void);
	float getInputVoltage(void);
	float getInput5VVoltage(void);
	
	void PowerON(void);
	void PowerOFF(void);
	void PowerONGPS(void);
	void PowerOFFGPS(void);
	void PowerONGPSBackup(void);
	void PowerOFFGPSBackup(void);
	void ResetGPS(void);
	
	void LEDSaftySwitchON(void);
	void LEDSaftySwitchOFF(void);
	bool SaftySwitchPushed(void);
	
	void LEDON(void);
	void LEDOFF(void);
	int GetChargeState(void);
	void auxSerialPowerDown(void);
	void auxSerialPowerUp(void);	
	void PowerOFFFrSkySmartPort(void);
	void PowerONFrSkySmartPort(void);
	
		
	
	private:
	void setPin(int pin, int mode, int initOutput);
	Uart *_serialPC = NULL;		    // UART To the PC (USB)
	Uart *_serialGPS = NULL;		// UART for the GPS module
	Uart *_serialfrskySPort = NULL; // UART for frskySmartPort
	Uart *_serialAUX = NULL;		// UART for auxiliary serial (Used for Mavlink to FC).
	Uart *_serialSBUS = NULL;	    // UART for the SBUS (RC inputs) (PCB -11 only)
	E28_2G4M20S *_radio = NULL;     // Radio module. 
	GPSL80Lite *_GPS = NULL;        // GPS module.
	
	// Define Pins for Radio
	const int chipSelectPin = 1;  // Pin D1  on MKRZero Board - Chip pin is 32 or PA23
	const int misoPin       = 10; // Pin D10 on MKRZero Board - Chip pin is 28 or PA19
	const int mosiPin       = 8;  // Pin D8  on MKRZero Board - Chip pin is 25 or PA16
	const int sckPin        = 9;  // Pin D9  on MKRZero Board - Chip pin is 26 or PA17
	const int rxEnPin       = 17; // Pin A2  on MKRZero Board - Chip pin is 48 or PB03
	const int txEnPin       = 32; // Pin LED on MKRZero Board - Chip pin is  7 or PB08
	const int resetPin      = 4;  // Chip pin is 19 or PB10
	const int busyPin       = 5;  // Chip pin is 20 or PB11
	const int dio1Pin		= 25; // PA03

	// POWER ON pin for Battery
	const int powerOnPin    = 0;  //Pin D0 on MKRZero Board - Chip pin is 31 or PA22

	// Analog inputs
	const int analogVinPin  = 19;  // Chip pin is 10 or PA05
	const int analogVbatPin = 16;  // Chip pin is 47 or PB02

	// Frsky Smart Port (S. Port) Pins:
	const int fryskyInvertPin = 2;          // Chip pin is 15 or PA10
	const int fryskySmartPortTXPin = 6;     // Chip pin is 29 or PA20 (SERCOM3 PAD2 TX)
	const int fryskySmartPortRXPin = 7;     // Chip pin is 30 or PA21 (SERCOM3 PAD3 RX)

	// Auxillary serial port pins:
	const int auxTXPin = 14;     // Chip pin is 29 or PB22 (SERCOM5 PAD2 TX)
	const int auxRXPin = 13;     // Chip pin is 30 or PB23 (SERCOM5 PAD3 RX)

	// GPS pins:
	const int GPSPowerOnPin = 20;      // Chip pin is 11 or PA06
	const int GPSBackupPowerPin = 29;    // Chip pin is 24 or PA15
	const int GPSTxPin = 11;           // Chip pin is 13 or PA08 (SERCOM2 PAD0 TX)
	const int GPSRxPin = 12;           // Chip pin is 14 or PA09 (SERCOM2 PAD1 RX)

	// LED pins:
	const int led2Pin = 21;      // Chip pin is 12 or PA07

	// Safty Switch
	const int SaftySwitchPin = 26; // PA12

	// LED in external safty switch
	const int SaftyLEDPin = 30;      // PA27

	// Analog input for 5V input.
	const int analogVin5VPin = 15; // PA02

	// Unused pins:
	// PA30-31 (SWDCLK/SWDIO) Not defined in variants.cpp
	const int pa11Pin = 3;  // PA11
	const int pa28Pin = 31; // PA28
	
	#if PCB_VERSION == 11
		// SBUS pins:
		const int sBUSInvertPin = 18;     // Chip pin is 9 or PA04
		const int sBUSRXPin     = 33;     // Chip pin is 8 or PB09 (SERCOM4 PAD1 RX)

		// Barometer Chip select:
		const int Baro_chipSelectPin = 24; // PA18

		// Not used pins
		const int pa13Pin = 27; // PA13
		
		const int pa14Pin = 28; // PA14
	#elif PCB_VERSION == 12
	
		// Charger state input
		const int chargeState = 33;     // Chip pin is 8 or PB09 (SERCOM4 PAD1 RX)

		// Power
		const int powerOnClk = 28;		// PA14 - Clock for gate for power on.
		const int GPSResetPin = 27;		// PA13 - Reset for GPS module.

		// unused pins:
		const int pa04Pin = 18; // Chip pin is 9 or PA04
		const int pa18Pin = 24; // PA18
	#endif
	
};


// Unique 128bit serial number from chip flash
// Chip unique serial number part 1: 800256040
// Chip unique serial number part 2:1347311437
// Chip unique serial number part 3: 808333642
// Chip unique serial number part 4:4278389281
// Chip unique serial number: " 80025604013473114378083336424278389281" (Prototype 1 - flight test)

// Chip unique serial number part 1:2946562988
// Chip unique serial number part 2:1347311437
// Chip unique serial number part 3: 808333642
// Chip unique serial number part 4:4278394144
// Chip unique serial number: "294656298813473114378083336424278394144" (Prototype 2 - ground test and low power mod)

#endif /* TRANSPONDER_HAL_H_ */