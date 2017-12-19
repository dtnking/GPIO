/*
 * USART.h
 *
 *  Created on: Dec 12, 2017
 *      Author: user2
 */

#ifndef USART_H_
#define USART_H_
#include "RCC.h"
#include "GPIO.h"

#define USART_BASE_ADDRS		(0x40011000)

typedef struct usartReg usartReg;

struct usartReg{
	volatile uint32_t SR;			//status reg 1
	volatile uint32_t DR;			//data reg 2
	volatile uint32_t BRR;			//baud rate register
	volatile uint32_t CR1;			//control reg 1
	volatile uint32_t CR2;			//control reg 2
	volatile uint32_t CR3;			//control reg 3
	volatile uint32_t GTPR;			//guard time and prescaler register
};

// Status register bit
#define TXE 				(1<<7)
#define TC					(1<<6)
#define RXNE				(1<<5)
#define ORE					(1<<3)
#define FE					(1<<1)
#define PE					(1)

//control register bit
#define OVER8				(0)
#define OVER16				(1<<15)
#define USART_EN			(1<<13)
#define USART_DIS			(0)
#define WORD_LEN_9			(1<<12)
#define WORD_LEN_8			(0)
#define PARITY_CNTRL_EN		(1<<10)
#define PARITY_CNTRL_DIS		(0)
#define PARITY_SEL_ODD		(1<<9)
#define PARITY_SEL_EVEN		(0)
#define TRANSMIT_EN			(1<<3)
#define TRANSMIT_DIS		(0)
#define RECEIVE_EN			(1<<2)
#define RECEIVE_DIS			(0)

//control register 2 bit
#define STOP_1BIT			(0)
#define STOP_05BIT			(1<<12)
#define STOP_2BIT			(2<<12)
#define STOP_15BIT			(3<<12)

//control register 3
#define UART_DMA_Tx_EN		(1<<7)
#define UART_DMA_Rx_EN		(1<<6)

#define usartEnableDmaTx()  (usart1->CR3 |= UART_DMA_Tx_EN)
#define usartEnableDmaRx()	(usart1->CR3 |= UART_DMA_Rx_EN)

#define usart1				((usartReg *)(USART_BASE_ADDRS))

void initUsart1(void);
void usartTransmit(char *val);
uint8_t usartReceiveOneByte();
void usartReceiveUntilEnter(char *rxBuffer);

#endif /* USART_H_ */

