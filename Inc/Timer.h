/*
 * Timer.h
 *
 *  Created on: Nov 21, 2017
 *      Author: user2
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>
#include "RCC.h"
#include "Gpio.h"

#define TIMER8_BASE_ADDR		0x40010400
//#define TIMER_BASE_ADDR		0x40023800
//#define TIMER_BASE_ADDR		0x40023800
//#define TIMER_BASE_ADDR		0x40023800
//#define TIMER_BASE_ADDR		0x40023800
//#define TIMER_BASE_ADDR		0x40023800
//#define TIMER_BASE_ADDR		0x40023800
//#define TIMER_BASE_ADDR		0x40023800



typedef struct TimerReg TimerReg;

struct TimerReg{
	volatile uint32_t CR1;			//control reg 1
	volatile uint32_t CR2;
	volatile uint32_t SMCR;			//slave mode control reg
	volatile uint32_t DIER;			//DMA and Interrupt En
	volatile uint32_t SR;			//Status reg
	volatile uint32_t EGR;			//event generation
	volatile uint32_t CCMR1;		//capture/compare mode
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;			//capture/compare En
	volatile uint32_t CNT;			//counter

	volatile uint32_t PSC;			//prescale
	volatile uint32_t ARR;			//auto reload
	volatile uint32_t RCR;			//repetition counter reg
	volatile uint32_t CCR1;			//capture/compare reg
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	volatile uint32_t BDTR;			//break and dead time
	volatile uint32_t DCR;			//DMA control
	volatile uint32_t DMAR;			//DMA address

};


#define Timer8			((TimerReg *)(TIMER8_BASE_ADDR))
void initTimer8();

#endif /* TIMER_H_ */
