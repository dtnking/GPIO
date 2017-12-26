/*
 * USART.c
 *
 *	The connection between UART1 pins on STM32f429ZIT6 and the USB-to-serial (CH340) are as follow:
 *
 *	STM32F429ZI	 | CH340
 *	 Name   Pin	 | Name
 *	---------------------
 *    Tx	PA9	 | Rx
 *	  Rx	PA10 | Tx
 *	  GND	GND  | GND
 *	---------------------
 *
 *  Created on: Dec 12, 2017
 *      Author: user2
 */

#include "USART.h"
#include <malloc.h>
#include <stdarg.h>
#include <stdio.h>

void initUsart1(void){
	//Enable USART 1
	rcc->apb2Rstr &= ~(1<<4);
	rcc->apb2Enr |= (1<<4);

	enableGpio(0);		    // Enable GPIOA
	gpioConfig(GpioA,9,GPIO_MODE_AF,GPIO_PUSH_PULL,GPIO_PULL_UP,GPIO_LOW_SPEED);
	gpioConfig(GpioA,10,GPIO_MODE_AF,GPIO_PUSH_PULL,GPIO_PULL_UP,GPIO_LOW_SPEED);
	gpioConfigAltFunction(GpioA, 9 ,ALT_FUNCT7);
	gpioConfigAltFunction(GpioA, 10 ,ALT_FUNCT7);
	usartEnableDmaTx();
	usartEnableDmaRx();
	usart1->BRR =0x30d;

	usart1->CR1 = OVER8|USART_EN|WORD_LEN_9|PARITY_CNTRL_EN|PARITY_SEL_ODD;
	usart1->CR2 = STOP_2BIT;
	usart1->CR1 |= TRANSMIT_EN;
	usart1->CR1 |= RECEIVE_EN;


}


void usartTransmit(char *val){
	usart1->CR1 |= TRANSMIT_EN;
	usart1->DR = *val;

	while(!(usart1->SR & TXE));
}


void usartReceiveUntilEnter(char *rxBuffer){
	usart1->CR1 |= RECEIVE_EN;

	*(rxBuffer)=usartReceiveOneByte();
		while(*(rxBuffer) != 0xa){
			rxBuffer ++;
			*(rxBuffer) = usartReceiveOneByte();
		}
		*(rxBuffer) = 0;
}

uint8_t usartReceiveOneByte(){
	while(!(usart1->SR & RXNE));
	return (uint8_t)usart1->DR;
}

/*
 * Print to the serial comm
 *
 * Requirement:
 * 	 Serial terminal or any terminal console that can display the character
 * 	 strings send through the USART
 *
 * Usage Example:
 *   serialPrint("Data: %d, %f, %s, %c", 123, doubleVal, "Hello", charVal);
 *
 * WARNING: Cannot be called from interrupt!
 */
void serialPrint(char *message,...) {
  va_list args;
  char *buffer;
  int i,length;

  va_start(args, message);

  length = vsnprintf(buffer, 0, message, args);
  buffer = malloc(length + 1);
  vsnprintf(buffer, length + 1, message, args);
  for(i=0;i<length+1;i++){
	  usartTransmit(&buffer[i]);
  }
  free(buffer);

}
