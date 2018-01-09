/*
 * RCC.h
 *
 *  Created on: Oct 27, 2017
 *      Author: user2
 */

#ifndef RCC_H_
#define RCC_H_
#include "Common.h"

#include <stdint.h>

#define RCC_BASE_ADDR		0x40023800
#define rcc					((RccReg *)(RCC_BASE_ADDR))

#define rccClearAllResetFlags()			(rcc->csr |= RCC_RMVF)
#define HI_SPEED_CLK		0
#define LO_SPEED_EXT		1
#define HI_SPEED_EXT		2

#define MCO_DIV_BY_5		7
#define MCO_DIV_BY_4		6
#define MCO_DIV_BY_3		5
#define MCO_DIV_BY_2		4

typedef struct RccReg RccReg;
struct RccReg {
	volatile uint32_t cr;
	volatile uint32_t pllcfgr;
	volatile uint32_t cfgr;
	volatile uint32_t cir;
	volatile uint32_t ahb1Rstr;
	volatile uint32_t ahb2Rstr;
	volatile uint32_t ahb3Rstr;
	volatile uint32_t reserverd1;
	volatile uint32_t apb1Rstr;
	volatile uint32_t apb2Rstr;
	volatile uint32_t reserved2[2];
	volatile uint32_t ahb1Enr;
	volatile uint32_t ahb2Enr;
	volatile uint32_t ahb3Enr;
	volatile uint32_t reserved3;
	volatile uint32_t apb1Enr;
	volatile uint32_t apb2Enr;
	volatile uint32_t reserved4[2];
	volatile uint32_t ahb1lpEnr;
	volatile uint32_t ahb2lpEnr;
	volatile uint32_t ahb3lpEnr;
	volatile uint32_t reserved5;
	volatile uint32_t apb1lpEn;
	volatile uint32_t apb2lpEn;
	volatile uint32_t reserved6[2];
	volatile uint32_t bdcr;
	volatile uint32_t csr;
	volatile uint32_t reserved7[2];
	volatile uint32_t sscgr;
	volatile uint32_t pll2scFgr;
};

#define rccSelectMco1Src(x)							\
		do{											\
			rcc->cfgr &= (~(3 << 21));				\
			rcc->cfgr |= (x << 21);					\
		}while(0)

#define rccSelectMco1Prescale(x)					\
		do{											\
			rcc->cfgr &= ~(7<<24);					\
			rcc->cfgr |= (x<<24);					\
		}while(0)


// Control Status Register (CSR)
#define RCC_LPWRRSTF		(1<<31)
#define RCC_WWDGRSTF		(1<<30)
#define RCC_IWDGRSTF		(1<<29)
#define RCC_SFTRSTF			(1<<28)
#define RCC_PORRSTF			(1<<27)
#define RCC_PINRSTF			(1<<26)
#define RCC_BORRSRSTF		(1<<25)
#define RCC_RMVF			(1<<24)


extern uint32_t *rccAhb1Rst;
extern uint32_t *rccAhb1En;

void enableGpio(int port);
void enableRng(void);
void enableDMA(int dmaPinBit);
void enableAdc();
void enableWWDG();

#endif /* RCC_H_ */
