/*
 * SysTick.h
 *
 *  Created on: Nov 7, 2017
 *      Author: user2
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#include <stdint.h>

#define SYSTICK_BASE_ADDR			0xE000E010
#define sysTick						((SysTickReg *)(SYSTICK_BASE_ADDR))

#define SYSTICK_EN					1
#define SYSTICK_INTR_EN				(1<<1)
#define SYSTICK_PROC_CLOCK_SRC		(1<<2)
#define SYSTICK_CNTR_OVRFLOW		(1<<16)

typedef struct SysTickReg SysTickReg;
struct SysTickReg{
	volatile uint32_t CTRL;				// Control and status register
	volatile uint32_t LOAD;				// reload value register
	volatile uint32_t VAL;				//	current value register
	volatile uint32_t CALIB;			// calibration value register

};

#define sysTickEnable()						(sysTick->CTRL |= SYSTICK_EN)
#define sysTickDisable()					(sysTick->CTRL &= ~SYSTICK_EN)
#define sysTickInterEnable()				(sysTick->CTRL |= SYSTICK_INTR_EN)
#define sysTickInterDisable()				(sysTick->CTRL &= ~SYSTICK_INTR_EN)
#define sysTickSetReload(x)					(sysTick->LOAD = (x))
#define sysTickReadCounter(x)				(sysTick->VAL)
#define sysTickClrCounter()					(sysTick->VAL = 0xbadface)
//Set SysTick to run @ processor speed
#define sysTickFullSpeed()					(sysTick->CTRL |= SYSTICK_PROC_CLOCK_SRC)
//Set SysTick to run @1/8 of AHB frequency
#define sysTickPrescaledSpeed()				(sysTick->CTRL &= ~SYSTICK_PROC_CLOCK_SRC)
//Determine if SysTick timer has expired. Return non-zero if has expired
#define sysTickHasExpired()					(sysTick->CTRL & SYSTICK_CNTR_OVRFLOW)

#endif /* SYSTICK_H_ */
