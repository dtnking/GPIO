/*
 * EXTI.h
 *
 *  Created on: Nov 7, 2017
 *      Author: user2
 */

#ifndef EXTI_H_
#define EXTI_H_
#include <stdint.h>

#define EXTINTR0_BASE_ADDR			0x40013C00
#define ExtI0						((ExtIntrReg *)(EXTINTR0_BASE_ADDR))


#define RISING_EDGE					1
#define FALLING_EDGE				1
#define EDGE_ARRIVE					1

typedef struct ExtIntrReg ExtIntrReg;
struct ExtIntrReg{
	volatile uint32_t IMR;				// Interrupt mask register
	volatile uint32_t EMR;				// Event mask register
	volatile uint32_t RTSR;				// Rising trigger selection register
	volatile uint32_t FTSR;				// Falling trigger selection register
	volatile uint32_t SWIER;			// Software interrupt event register
	volatile uint32_t PR;				// Pending register
};




#define extIntEnable(x)				(ExtI0->IMR |= (1<<(x)))
#define extIntDisable(x)			(ExtI0->IMR &=  ~(1<<(x)))
#define extMaskEnable(x)			(ExtI0->EMR |= (1<<(x)))
#define extMaskDisable(x)			(ExtI0->EMR &=  ~(1<<(x)))
#define setRisingEdge()				(ExtI0->RTSR |= RISING_EDGE)
#define setFallingEdge()			(ExtI0->FTSR |= FALLING_EDGE)
#define softwareTrigger(x)			(ExtI0->SWIER |= (x))
#define extIntReadPR(x)	      	    (ExtI0->PR)
#define extIntClearPR()  			(ExtI0->PR |= EDGE_ARRIVE)

#endif /* EXTI_H_ */
