/*
 * Timer.c
 *
 *  Created on: Nov 21, 2017
 *      Author: user2
 */
#include "Timer.h"

void initTimer8(){
	rcc->apb2Rstr &= ~(1 << 1);	//Unreset Timer8
	rcc->apb2Enr |= (1 << 1);   //Enable the Timer8

	Timer8->ARR = 500;		//configure the maximum counter value by setting ARR
	Timer8->PSC = 7999;
	Timer8->SMCR &= ~(3);      //Select the clock source (internal clk)
	Timer8->CR1 |= (1);		  //Enable timer 8




	//configure the prescaler by setting PSC
	//Select the clock source (internal clk)

}

void initTimer8Channel1(){
	// Enable Timer
	rcc->apb2Rstr &= ~(1 << 1);	//Unreset Timer8
	rcc->apb2Enr |= (1 << 1);   //Enable the Timer8

	// Enable PORT C, set pin as altFunction out . PP
  	enableGpio(2);		// port C
	gpioConfig(GpioC,6,GPIO_MODE_AF,GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_LOW_SPEED);
	gpioConfigAltFunction(GpioC,6,ALT_FUNCT3);

	Timer8->CCMR1 &= (~3);
	Timer8->CCMR1 &= (~7);
	Timer8->CCMR1 |= 3;

	Timer8->CCER |= 1;
	Timer8->BDTR |= (1<<14);

	Timer8->ARR = 500;		//configure the maximum counter value by setting ARR
	Timer8->PSC = 7999;
	Timer8->SMCR &= ~(3);      //Select the clock source (internal clk)
	Timer8->CR1 |= (1);		  //Enable timer 8



}


// 	configure timer 8 channel 1 pin (PC6) - Enable PORT C, set pin as altFunction out . PP
//	configure channel 1 as output compare -- ccmr1 :CC1S[1:0] = 0

// configure output compare toggle mode -- CCMR1:OC1M[6:4] = 1
// configure OC pin polarity and enable it.

