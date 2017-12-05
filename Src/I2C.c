/*
 * I2C.c
 *
 *  Created on: Nov 28, 2017
 *      Author: user2
 */
#include "I2C.h"

void initI2C(void){
	rcc->apb1Rstr &= ~(1<<21);		// Enable I2c1 as master
	rcc->apb1Enr |= (1<<21);

	rcc->apb1Rstr &= ~(1<<22);		// Enable I2c2 as slave
	rcc->apb1Enr |= (1<<22);

	enableGpio(1);		    // Enable GPIOB
	gpioConfig(GpioB,6,GPIO_MODE_AF,GPIO_OPEN_DRAIN,GPIO_PULL_UP,GPIO_LOW_SPEED);
	gpioConfig(GpioB,7,GPIO_MODE_AF,GPIO_OPEN_DRAIN,GPIO_PULL_UP,GPIO_LOW_SPEED);
	gpioConfigAltFunction(GpioB, 6 ,ALT_FUNCT4);
	gpioConfigAltFunction(GpioB, 7 ,ALT_FUNCT4);

	i2c1->CR1 |= (1<<15);		//software reset
	i2c1->CR1 &= ~(1<<15);		//software unreset

	i2c1->CCR  &= ~(1<<15); 	//set to standard mode
	i2c1->CR2 &= ~(20);			//set I2C frequency to 20MHz
	i2c1->CR2 |= (20);

	i2c1->CCR &= ~(100);	// Clear CCR
	i2c1->CCR |= (100);		// Set CCR to 100

	i2c1->TRISE &= ~(21);
	i2c1->TRISE |= (21);

/*	enableGpio(6);			// Enable GPIOF
	gpioConfig(GpioF,0,GPIO_MODE_AF,GPIO_OPEN_DRAIN,GPIO_PULL_UP,GPIO_LOW_SPEED);
	gpioConfig(GpioF,1,GPIO_MODE_AF,GPIO_OPEN_DRAIN,GPIO_PULL_UP,GPIO_LOW_SPEED);
	gpioConfigAltFunction(GpioF, 0 ,ALT_FUNCT4);
	gpioConfigAltFunction(GpioF, 1 ,ALT_FUNCT4);


	i2c2->CR2 &= ~(20);
	i2c2->CR2 |= (20);

	i2c2->CCR &= ~(100);	// Clear CCR
	i2c2->CCR |= (100);		// Set CCR to 465

	i2c2->TRISE &= ~(21);
	i2c2->TRISE |= (21);
*/

	i2c1->CR1 |= PERIPHERAL_EN;
	i2c1->CR1 |= MASTER_START;
}
