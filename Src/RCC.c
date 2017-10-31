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
