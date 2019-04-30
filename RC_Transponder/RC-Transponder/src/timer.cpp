/*
	timer.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
#include "timer.h"
#include "hw.h"
#include "main.h"
#include "Arduino.h"
#include "RFService.h"


void setTimerFrequency(int frequencyHz) {
	
	//int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
	int compareValue = (KHZ_OSC / (16 * frequencyHz)) - 1;
	TcCount16* TC = (TcCount16*) TC3;
	
	// Make sure the count is in a proportional position to where it was
	// to prevent any jitter or disconnect when changing the compare value.
	TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
	TC->CC[0].reg = compareValue;
	//  Serial.println(TC->COUNT.reg);
	//  Serial.println(TC->CC[0].reg);
	while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer3(int frequencyHz){
	// Setup Timer 3 to use internal 32khz, and ensure it runs in standby.

	// Put Generic Clock Generator 1 (32Khz) as source for Timer 3 (and 2)
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_TCC2_TC3)     | // Timer 2 and Timer 3
						GCLK_CLKCTRL_GEN_GCLK2			  | // Generic Clock Generator 2 is source (Internal 32khz).
						GCLK_CLKCTRL_CLKEN;
	
	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2)				   | // Generic Clock Generator 2
						GCLK_GENCTRL_SRC_OSCULP32K		   | // Use internal 32 Khz oscilator
						GCLK_GENCTRL_GENEN				   | // Clock Generator Enabled
						GCLK_GENCTRL_RUNSTDBY;				 // Run in standtby.

						
	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync
	TcCount16* TC = (TcCount16*) TC3;

	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

	TC->CTRLA.bit.RUNSTDBY = 1;

	// Use the 16-bit timer
	TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

	// Use match mode so that the timer counter resets when the count matches the compare register
	TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  
	// Set prescaler to 1024
	//	TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
	TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	setTimerFrequency(frequencyHz);

	// Enable the compare interrupt
	TC->INTENSET.reg = 0;
	TC->INTENSET.bit.MC0 = 1;
	NVIC_EnableIRQ(TC3_IRQn);
	TC->CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

// ISR function doe timer 3
void TC3_Handler() {
	TcCount16* TC = (TcCount16*) TC3;
	// If this interrupt is due to the compare register matching the timer count
	// we toggle the LED.
	if (TC->INTFLAG.bit.MC0 == 1) {
		TC->INTFLAG.bit.MC0 = 1;
		// Write callback here!!!
		
		RadioService->SeccondCounter(); // Count up the Seconds since laste ground contact.
		
		if(SystemInformation.SecondCounter < 254){
			SystemInformation.SecondCounter++;
		}
		
		if(SystemInformation.BeaconSecondCounter < 20){
			SystemInformation.BeaconSecondCounter++;
		}
	}
}
