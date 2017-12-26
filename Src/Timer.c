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
	rcc->apb2Rstr &= ~(1 << 1);	//Unreset Timer8
	rcc->apb2Enr |= (1 << 1);   //Enable the Timer8

	// Enable PORT C, set pin as altFunction out . PP
  	enableGpio(2);
	gpioConfig(GpioC,6,GPIO_MODE_AF,GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_VHI_SPEED);
	gpioConfigAltFunction(GpioC,6,ALT_FUNCT3);

	gpioConfig(GpioC,8,GPIO_MODE_AF,GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_VHI_SPEED);
	gpioConfigAltFunction(GpioC,8,ALT_FUNCT3);


	Timer8->CCER |=  CC1E_EN;	//Enable OC1 pin to generate an output
	Timer8->CR2 |= NO_PRELOAD;



	gpioConfig(GpioC,8,GPIO_MODE_AF,GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_LOW_SPEED);
	gpioConfigAltFunction(GpioC,8,ALT_FUNCT3);
	Timer8->CCMR2 &= ~(3);		// reset
	Timer8->CCMR2 |= (1);		//CC3 channel is configured as input, IC3 is mapped on TI3
	//CC3 channel is configured as input, IC3 is mapped on TI3
	Timer8->CCMR2 &=  ~(3<<8);
	Timer8->CCMR2 |= (2<<8);	// CC4 channel is configured as input, IC4 is mapped on TI3

	Timer8->CCER  &= ~(3<<9);	//set
	Timer8->CCER |= CC3E_EN;		//Enable input3 capture

	Timer8->CCER  &= ~(3<<13);
	Timer8->CCER |= (1<<13);
	Timer8->CCER |= CC4E_EN;		//Enable input4 capture


	Timer8->BDTR |= (1<<15);

}

void configureTimer8(int arrVal, int pscVal){

	Timer8->ARR = arrVal;		//configure the maximum counter value by setting ARR
	Timer8->CCR1 =arrVal;
	Timer8->CNT = arrVal - 1;
	Timer8->PSC = pscVal;
	Timer8->SMCR &= ~(3);      //Select the clock source (internal clk)
	Timer8->CR1 |= (1);		   //start timer counter
}

void forceOutCompareChannel1High(void){
	Timer8->CCMR1 &= ~(7<<4);
	Timer8->CCMR1 |= FORCE_HI;
}

void forceOutCompareChannel1Low(void){
	Timer8->CCMR1 &= ~(7<<4);
	Timer8->CCMR1 |= FORCE_LO;
}

void toggleOutCompareChannel1WithForce(){
	forceOutCompareChannel1High();
	forceOutCompareChannel1Low();
}



// 	configure timer 8 channel 1 pin (PC6) - Enable PORT C, set pin as altFunction out . PP
//	configure channel 1 as output compare -- ccmr1 :CC1S[1:0] = 0

// configure output compare toggle mode -- CCMR1:OC1M[6:4] = 1
// configure OC pin polarity and enable it.

