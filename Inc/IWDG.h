/*
 * IWDG.h
 *
 *  Created on: Jan 9, 2018
 *      Author: user2
 */

#ifndef IWDG_H_
#define IWDG_H_
#include <stdint.h>

#define IWDG_BASE_ADDRS			(0x40003000)
#define WWDG_BASE_ADDRS			(0x40002C00)

#define iwdg	((iwdgReg *)(IWDG_BASE_ADDRS))
#define wwdg	((wwdgReg *)(WWDG_BASE_ADDRS))

typedef struct iwdgReg iwdgReg;
struct iwdgReg{
	volatile uint32_t KR;		//0h
	volatile uint32_t PR;		//4h
	volatile uint32_t RLR;
	volatile uint32_t SR;
};

typedef struct wwdgReg wwdgReg;
struct wwdgReg{
	volatile uint32_t CR;		//0h
	volatile uint32_t CFR;		//4h
	volatile uint32_t SR;
};


// Status Reg
#define EWIF		1

//Prescaler Reg
#define PRESCALER_DIV_4   	0
#define PRESCALER_DIV_8   	1
#define PRESCALER_DIV_16   	2
#define PRESCALER_DIV_32   	3
#define PRESCALER_DIV_64   	4
#define PRESCALER_DIV_128   5
#define PRESCALER_DIV_254   6

//configuration Reg
#define WWDG_COUNTER_CLK_DIV_1		0
#define WWDG_COUNTER_CLK_DIV_2		1
#define WWDG_COUNTER_CLK_DIV_4		2
#define WWDG_COUNTER_CLK_DIV_5		3
#define WWDG_EWI					(1<<9)
#define WWDG_WIN_MASK				(~(0x7f))
#define WWDG_COUNTER_CLK_MASK		(~(3<<7))

//WWDG Control Reg
#define WWDG_WDGA			(1<<7)
#define WWDG_T6				(1<<6)

#define wwdgSetTimeOutAndActivate(to)			(wwdg->CR = WWDG_WDGA |WWDG_T6 | (to & ~WWDG_WIN_MASK))

#define wwdgIsEarlyWakeupInterrupt()		(wwdg->SR & EWIF)
#define wwdgEnableWakeupInterrupt()			(wwdg->CFR |= WWDG_EWI)
#define wwdgDisableWakeupInterrupt()		(wwdg->CFR &= WWDG_EWI)
#define clearEarlyWakeupInterrupt()			(wwdg->SR = 0)

//Status Reg
#define IWDG_RVU		(1<<1)		//Reload value update
#define IWDG_PVU		(1)			//prescaler value update

//Key Reg
#define IWDG_RESET_WATCHDOG		(0xAAAA)
#define IWDG_START_WATCHDOG		(0xCCCC)
#define IWDG_CONF_WATCHDOG		(0x5555)

#define iwdgStart()						(iwdg->KR = IWDG_START_WATCHDOG)
#define iwdgEnableConfiguration()		(iwdg->KR = IWDG_CONF_WATCHDOG)
#define iwdgReset()						(iwdg->KR = IWDG_RESET_WATCHDOG)

#define iwdgSetReloadValue(val)			(iwdg->RLR = (val))
#define iwdgSetPrescalerValue(val)		(iwdg->PR = (val))

void iwdgWaitTillPrescalerDivided(int val);
void iwdgWaitTillLoaded(int val);


#endif /* IWDG_H_ */
