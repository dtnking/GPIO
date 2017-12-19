/*
 * GPIO.c
 *
 *  Created on: Oct 27, 2017
 *      Author: user2
 */

#include "GPIO.h"

void gpioConfig(GpioReg *Gpio, int pin, int mode, int outDriveType, int pullType, int speed){
	Gpio->mode &= ~(3 << (pin * 2));			// Clear pin mode to 0 first
	Gpio->mode |= mode << (pin * 2);			// Set pin mode

	Gpio->outSpeed &= ~(3 << (pin * 2));			// Clear pin speed to 0 first
	Gpio->outSpeed |= speed << (pin * 2);			// Set pin speed

	Gpio->pullType &= ~(3 << (pin * 2));			// Clear pin pull-type to 0 first
	Gpio->pullType |= pullType << (pin * 2);		// Set pin pull-type

	Gpio->outType &= ~(1 << pin);					// Clear pin drive-type to 0 first
	Gpio->outType |= outDriveType << pin;			// Set pin drive-type
}

void gpioConfigAltFunction(GpioReg *Gpio, int pin,int altFunction){
	if(pin<8){
		Gpio->altFuncLo &= ~(0xf<<pin*4);
		Gpio->altFuncLo |= (altFunction<< pin*4);
	}
	else{
		Gpio->altFuncHi &= ~(0xf<<((pin-8)*4));
		Gpio->altFuncHi |= (altFunction<<((pin-8)*4));

	}
}

void _gpioWrite(GpioReg *Gpio, int pin, int state){
	if(state == 1){
		Gpio->outData |= 1 << pin;
	}else {
		Gpio->outData &= ~(1 << pin);
	}
}

void gpioWrite(GpioReg *Gpio, int pin, int state){
	if(state == 1){
		SET_PIN(Gpio,pin);
	}else {
		RESET_PIN(Gpio,pin);
	}
}

int gpioRead(GpioReg *Gpio, int pin){
	return Gpio->inData & (1 << pin);
}
