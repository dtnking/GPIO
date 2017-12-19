/*
 * Flash.c
 *
 *  Created on: Dec 5, 2017
 *      Author: user2
 */

#include "Flash.h"

/**
 * 	Section erase the flash memory (STM32F429Zi MCU only)
 * 	Input  :
 * 	  SectionNum is the section
 * 	Return :
 * 	  0		Fail to erase Check SR
 * 	  1		Successful
 */
int flashEraseSector(int sectorNum){
	if(sectorNum>11)
		sectorNum += 4;
	//Unlock through KEYR
	flash->KEYR = 0x45670123;
	flash->KEYR = 0xCDEF89AB;
	//Set the sector number to erase
	flash->CR = FLASH_SECTOR(sectorNum)|FLASH_SECTOR_ERASE|FLASH_START;
	//Set the sector erase

	//Start erase

	//Check for failure or success
	return 1;
	//Return status
}

/**
 * 	Enable flash programming with the stated program size
 * 	input:
 * 		programSize is the size when writing to the flash
 * 				FLASH_BYTE_SIZE
 * 				FLASH_HALFWORD_SIZE
 * 				FLASH_WORD_SIZE
 * 				FLASH_DOUBLEWORD_SIZE
 */
int flashEnableProgramming(int programSize){
	flash->CR |= FLASH_PROGRAMMING;
	flash->CR |= FLASH_SIZE(programSize);
}

int flashDisable(){
	flash->CR &= ~(FLASH_PROGRAMMING);
}

// Sector 13 start address : 0x08084000
int writeMessage(char *message, char *memoryToWrite){
	strcpy(memoryToWrite,message);
}
