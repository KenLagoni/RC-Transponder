/*
	timer.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include <Arduino.h>

//// For TIMER ISR -----------------
#define KHZ_OSC 32768
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

void startTimer3(int frequencyHz);
void setTimerFrequency(int frequencyHz);

//// For TIMER ISR -----------------




#endif /* TIMER_H_ */