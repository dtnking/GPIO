/*
 * RCC.c
 *
 *  Created on: Oct 27, 2017
 *      Author: user2
 */

#include "RCC.h"





void enableGpio(int port){
	rcc->ahb1Rstr &= ~(1 << port); // Unreset GPIOG
	rcc->ahb1Enr |= (1 << port);  	// Start Clock GPIOG
}

void enableRng(void){
	rcc->ahb2Rstr &= ~(1<<6);
	rcc->ahb2Enr |= (1<<6);
}

void enableMCO1(void){
	rcc->cfgr |= (3<<24);
	rcc->cfgr &= ~(3<<21);
	rcc->cfgr |= (2<<21);
}

/*
 * Enable and clock DMA
 * @param dmaPinBit is one of the following
 * 					DMA1_DEV
 * 					DMA2_DEV
 */
void enableDMA(int dmaPinBit){
	rcc->ahb1Rstr &= ~(1 << dmaPinBit); // Unreset DMAx
	rcc->ahb1Enr |= (1 << dmaPinBit);  	// Start Clock DMAx
}
