/*
 * timer.h
 *
 * Created: 15-Dec-18 22:32:48
 *  Author: Kenneth
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include <Arduino.h>

//// For TIMER ISR -----------------
#define KHZ_OSC 32768
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
//void startTimer(int frequencyHz);
void startTimer3(int frequencyHz);
void setTimerFrequency(int frequencyHz);
//void TC3_Handler();
extern uint8_t SecondCounter;
extern uint8_t BeaconSecondCounter;
//// For TIMER ISR -----------------




#endif /* TIMER_H_ */