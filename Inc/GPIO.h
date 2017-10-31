/*
 * GPIO.h
 *
 *  Created on: Oct 27, 2017
 *      Author: user2
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>

#define GPIOA_BASE_ADDR		0x40020000
#define GPIOB_BASE_ADDR		0x40020400
#define GPIOC_BASE_ADDR		0x40020800
#define GPIOD_BASE_ADDR		0x40020C00
#define GPIOE_BASE_ADDR		0x40021000
#define GPIOF_BASE_ADDR		0x40021400
#define GPIOG_BASE_ADDR		0x40021800
#define GPIOH_BASE_ADDR		0x40021C00
#define GPIOI_BASE_ADDR		0x40022000

#define GpioA				((GpioReg *)(GPIOA_BASE_ADDR))
#define GpioB				((GpioReg *)(GPIOB_BASE_ADDR))
#define GpioC				((GpioReg *)(GPIOC_BASE_ADDR))
#define GpioD				((GpioReg *)(GPIOD_BASE_ADDR))
#define GpioE				((GpioReg *)(GPIOE_BASE_ADDR))
#define GpioF				((GpioReg *)(GPIOF_BASE_ADDR))
#define GpioG				((GpioReg *)(GPIOG_BASE_ADDR))
#define GpioH				((GpioReg *)(GPIOH_BASE_ADDR))
#define GpioI				((GpioReg *)(GPIOI_BASE_ADDR))

#define SET_PIN(gpio,pinNum)		gpio->BSRR = ( 1 << pinNum )
#define RESET_PIN(gpio,pinNum)		gpio->BSRR = ( 1 << ( 16 + pinNum ))

typedef struct GpioReg GpioReg;
struct GpioReg{
	volatile uint32_t mode;			// 0h
	volatile uint32_t outType;		// 4h
	volatile uint32_t outSpeed;		// 8h
	volatile uint32_t pullType;		// ch
	volatile uint32_t inData;		// 10h
	volatile uint32_t outData;		// 14h
	volatile uint32_t BSRR;		// 18h
	volatile uint32_t lock;			// 1ch
	volatile uint32_t altFuncLo;	// 20h
	volatile uint32_t altFuncHi;	// 24h
};

// Pin Modes
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_AF		2
#define GPIO_MODE_ANA		3

// Pin output drive type
#define GPIO_PUSH_PULL		0
#define GPIO_OPEN_DRAIN		1

// Pin Pull-Type
#define GPIO_NO_PULL		0
#define GPIO_PULL_UP 		1
#define GPIO_PULL_DOWN		2

// Pin output speed
#define GPIO_LOW_SPEED		0
#define GPIO_MED_SPEEd		1
#define GPIO_HI_SPEED		2
#define GPIO_VHI_SPEED		3


// Export variables to other modules
void gpioConfig(GpioReg *Gpio, int pin, int mode, int outDriveType, int pullType, int speed);
void gpioWrite(GpioReg *Gpio, int pin, int state);
int gpioRead(GpioReg *Gpio, int pin);

#endif /* GPIO_H_ */
