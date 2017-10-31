/*
 * RCC.h
 *
 *  Created on: Oct 27, 2017
 *      Author: user2
 */

#ifndef RCC_H_
#define RCC_H_

#include <stdint.h>

#define RCC_BASE_ADDR		0x40023800
#define rcc					((RccReg *)(RCC_BASE_ADDR))

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


extern uint32_t *rccAhb1Rst;
extern uint32_t *rccAhb1En;

void enableGpio(int port);
void enableRng(void);


#endif /* RCC_H_ */
