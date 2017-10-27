/*
 * GPIO.c
 *
 *  Created on: Oct 27, 2017
 *      Author: user2
 */

#include "GPIO.h"

void gpioGCOnfig(int pin, int mode, int outDriveType, int pullType, int speed){
	GpioG->mode &= ~(3 << (pin * 2));			// Clear pin mode to 0 first
	GpioG->mode |= mode << (pin * 2);			// Set pin mode

	GpioG->outSpeed &= ~(3 << (pin * 2));			// Clear pin speed to 0 first
	GpioG->outSpeed |= speed << (pin * 2);			// Set pin speed

	GpioG->pullType &= ~(3 << (pin * 2));			// Clear pin pull-type to 0 first
	GpioG->pullType |= pullType << (pin * 2);		// Set pin pull-type

	GpioG->outType &= ~(1 << pin);					// Clear pin drive-type to 0 first
	GpioG->outType |= outDriveType << pin;			// Set pin drive-type
}

void gpioGWrite(int pin, int state){
	if(state == 1){
		GpioG->outData |= 1 << pin;
	}else {
		GpioG->outData &= ~(1 << pin);
	}
}
