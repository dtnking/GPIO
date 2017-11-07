/*
 * RNG.h
 *
 *  Created on: Oct 31, 2017
 *      Author: user2
 */

#ifndef RNG_H_
#define RNG_H_

#include <stdint.h>
#define RNG_BASE_ADDR		0x50060800

#define Rng				((RngReg *)(RNG_BASE_ADDR))

typedef struct RngReg RngReg;
struct RngReg{
	volatile uint32_t CR;		// 0h
	volatile uint32_t SR;		// 4h
	volatile uint32_t DR;		// 8h
};

//SR
#define RNG_FAULTY_SEQUENCE		(1 << 2)
#define RNG_FAULTY_CLK			(1 << 1)
#define RNG_DRDY				1

//CR
#define RNG_RNGEN				(1 << 2)

#define RNG_IE					(1 << 3)


void generateRandomNumber(void);
int isRandomNumberAvailable(void);
int isAnyError(void);
int isFaultyRandomSequence(void);
int getRandomNumber(void);
void getRandomNumberByInterrupt(void);

#endif /* RNG_H_ */
