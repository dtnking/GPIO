/*
 * Flash.h
 *
 *  Created on: Dec 5, 2017
 *      Author: user2
 */

#ifndef FLASH_H_
#define FLASH_H_

#include <stdint.h>
#define FLASH_BASE_ADDR		(0x40023C00)

#define FLASH_BYTE_SIZE				0
#define FLASH_HALFWORD_SIZE			1
#define FLASH_WORD_SIZE				2
#define	FLASH_DOUBLEWORD_SIZE		3

#define FLASH_SECTOR_ERASE			(1<<1)
#define FLASH_START					(1<<16)
#define FLASH_EOP_INTR				(1<<24)
#define FLASH_SECTOR(sectorNum)		(sectorNum<<3)
#define FLASH_PROGRAMMING			1
#define FLASH_SIZE(programSize)		(programSize<<8)

#define FLASH_BUSY					(1<<16)
#define FLASH_OPERR					(1<<1)
#define FLASH_EOP					1

int flashEraseSector(int sectorNum);
int flashEnableProgramming(int programSize);
int flashDisable();
int writeMessage(char *message, char *memoryToWrite);

typedef struct flashReg flashReg;

struct flashReg{
	volatile uint32_t ACR;			//control reg
	volatile uint32_t KEYR;			//control reg 2
	volatile uint32_t OPTKEYR;			//own address 1
	volatile uint32_t SR;			//own address 2
	volatile uint32_t CR;			//data register
	volatile uint32_t OPTCR;			//status register 1
	volatile uint32_t OPTCR1;			//status register 1
};

#define flash			((flashReg *)(FLASH_BASE_ADDR))

#endif /* FLASH_H_ */
