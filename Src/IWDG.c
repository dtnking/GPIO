/*
 * IWDG.c
 *
 *  Created on: Jan 9, 2018
 *      Author: user2
 */
#include "IWDG.h"

void iwdgWaitTillLoaded(int val){
	while((iwdg->SR & IWDG_RVU)){
	}
	iwdg->RLR = (val);
}

void iwdgWaitTillPrescalerDivided(int val){
	while((iwdg->SR & IWDG_PVU)){
	}
	iwdg->PR = val;
}

void wwdgSetWindowValue(int windowValue){
	wwdg->CFR &= WWDG_WIN_MASK;
	wwdg->CFR |= (WWDG_T6|windowValue) & ~WWDG_WIN_MASK;
}

void wwdgSetPrescaler(int timerBase){
	wwdg->CFR &= WWDG_COUNTER_CLK_MASK;
	wwdg->CFR |= (timerBase<<7) & ~WWDG_COUNTER_CLK_MASK;
}


