/*
 * SysCfg.h
 *
 *  Created on: Nov 7, 2017
 *      Author: user2
 */

#ifndef SYSCFG_H_
#define SYSCFG_H_
#include <stdint.h>

#define SYSCFG_BASE_ADDR			0x40013800
#define syscfg						((sysCfgReg *)(SYSCFG_BASE_ADDR))

typedef struct sysCfgReg sysCfgReg;
struct sysCfgReg{
	volatile uint32_t MEMRMP;				// memory remap register
	volatile uint32_t PMC;					// peripheral mode configuration register
	volatile uint32_t EXTICR[4];			// external interrupt configuration register
	volatile uint32_t CMPCR;				// Compensation cell control register
};
#define PORTA 		0
#define PORTB 		1
#define PORTC 		2
#define PORTD 		3
#define PORTE 		4
#define PORTF 		5



#define sysCfgConfigureGPIO(extIntrGrp,gpio)			\
	do{													\
					(syscfg->EXTICR[((extIntrGrp>>2) & 0xf)] &= ~(0xf<<(extIntrGrp& 0x3)<<2)) ;	\
					(syscfg->EXTICR[(extIntrGrp>>2) & 0xf]  |= ~gpio<<((extIntrGrp& 0x3)<<2)) ;	\
				}while(0)
#endif /* SYSCFG_H_ */
