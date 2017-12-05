/*
 * I2C.h
 *
 *  Created on: Nov 28, 2017
 *      Author: user2
 */

#ifndef I2C_H_
#define I2C_H_

#include "RCC.h"
#include "GPIO.h"


#define I2C1_BASE_ADDR		(0x40005400)
#define I2C2_BASE_ADDR		(0x40005800)
#define I2C3_BASE_ADDR		(0x40005C00)

// Start Generation
#define MASTER_START		(1<<8)
#define MASTER_NO_START		(0<<8)
#define SLAVE_START			(1<<8)
#define SLAVE_NO_START		(0<<8)

// PE
#define PERIPHERAL_EN		1
#define PERIPHERAL_DISABLE	0

#define SM_MODE				0
#define FM_MODE				1

typedef struct i2cReg i2cReg;

struct i2cReg{
	volatile uint32_t CR1;			//control reg 1
	volatile uint32_t CR2;			//control reg 2
	volatile uint32_t OAR1;			//own address 1
	volatile uint32_t OAR2;			//own address 2
	volatile uint32_t DR;			//data register
	volatile uint32_t SR1;			//status register 1
	volatile uint32_t SR2;			//status register 2
	volatile uint32_t CCR;			//clock control register
	volatile uint32_t TRISE;		//time rise register
	volatile uint32_t FLTR;			//filter

};

#define i2c1			((i2cReg *)(I2C1_BASE_ADDR))
#define i2c2			((i2cReg *)(I2C2_BASE_ADDR))
#define i2c3			((i2cReg *)(I2C3_BASE_ADDR))

void initI2C(void);

#endif /* I2C_H_ */
