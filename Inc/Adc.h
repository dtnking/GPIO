/*
 * Adc.h
 *
 *  Created on: Jan 2, 2018
 *      Author: user2
 */

#ifndef ADC_H_
#define ADC_H_

#include "RCC.h"
#include "GPIO.h"

#define ADC1_BASE_ADDRS			(0x40012000)
#define ADC2_BASE_ADDRS			(0x40012100)
#define ADC3_BASE_ADDRS			(0x40012200)
#define COMMON_REG_BASE_ADDRS	(0x40012300)

typedef struct adcReg adcReg;

struct adcReg{
	volatile uint32_t SR;		//0h
	volatile uint32_t CR1;		//4h
	volatile uint32_t CR2;
	volatile uint32_t SMPR1;
	volatile uint32_t SMPR2;
	volatile uint32_t JOFR1;
	volatile uint32_t JOFR2;
	volatile uint32_t JOFR3;
	volatile uint32_t JOFR4;
	volatile uint32_t HTR;
	volatile uint32_t LTR;
	volatile uint32_t SQR1;
	volatile uint32_t SQR2;
	volatile uint32_t SQR3;
	volatile uint32_t JSQR;
	volatile uint32_t JDR1;
	volatile uint32_t JDR2;
	volatile uint32_t JDR3;
	volatile uint32_t JDR4;
	volatile uint32_t DR;
};


#define ADC_EN		(1)
#define ADC_DIS 	(0)
#define SIN_CONV	(0)
#define CONT_CONV	(1<<1)
#define START_CONV	(1<<30)
#define EOCS_EN		(1<<10)

#define EOC_IS_SET	(1)

#define RESET_OVR	(~(1<<5))

#define CHN_MASK	(~(3))
#define CHN_3_CYLCE	(0)
#define CHN_480_CYLCE	(7)

#define ADC_3_CYCLES 	0
#define ADC_15_CYCLES 	1
#define ADC_28_CYCLES 	2
#define ADC_56_CYCLES 	3
#define ADC_84_CYCLES  	4
#define ADC_112_CYCLES  5
#define ADC_144_CYCLES  6
#define ADC_480_CYCLES  7

#define RES_MASK		(~(3<<24))
#define RES_12_BIT		0
#define RES_10_BIT		1
#define RES_8_BIT		2
#define RES_6_BIT		3


#define SEQ_LEN_MASK	(~(3<<20))
#define SEQ_LEN_1		(0)
#define SEQ_LEN_2		(1<<20)

#define RES_12 			(0)




typedef struct adcCommonReg adcCommonReg;
struct adcCommonReg{
	volatile uint32_t CSR;			//status reg 1
	volatile uint32_t CCR;			//data reg 2
	volatile uint32_t CDR;			//baud rate register
};
#define adc1		((adcReg *)(ADC1_BASE_ADDRS))
#define adc2		((adcReg *)(ADC2_BASE_ADDRS))
#define adc3		((adcReg *)(ADC3_BASE_ADDRS))
#define adcCommon	((adcCommonReg *)(COMMON_REG_BASE_ADDRS))
void initAdc(void);
void setCycle(int chn,int cycle);


#endif /* ADC_H_ */
