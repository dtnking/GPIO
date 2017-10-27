/*
 * RCC.c
 *
 *  Created on: Oct 27, 2017
 *      Author: user2
 */

#include "RCC.h"

uint32_t *rccAhb1Rst = (uint32_t *)(RCC_BASE_ADDR + RCC_AHB1RST_OFF);
uint32_t *rccAhb1En = (uint32_t *)(RCC_BASE_ADDR + RCC_AHB1EN_OFF);

void enableGpioA(void){
	// Unreset GPIOG
	*rccAhb1Rst &= ~1;

	// Start Clock GPIOG
	*rccAhb1En |= 1 ;
}


void enableGpioG(void){
	// Unreset GPIOG
	*rccAhb1Rst &= ~(1 << 6);

	// Start Clock GPIOG
	*rccAhb1En |= 1 << 6;
}
