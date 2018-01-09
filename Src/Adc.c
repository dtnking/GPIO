/*
 * Adc.c
 *
 *  Created on: Jan 2, 2018
 *      Author: user2
 */
#include "Adc.h"


void initAdc(void){
	gpioConfig(GpioA,5,GPIO_MODE_ANA,\
	  		  	  GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_VHI_SPEED);
	gpioConfig(GpioC,3,GPIO_MODE_ANA,\
		  		  	  GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_VHI_SPEED);
	enableAdc();
	adc1->CR2  |= ADC_EN;
	adc1->CR2  |= CONT_CONV;
	adc1->CR2  |= EOCS_EN;
	adc1->SQR1 &= SEQ_LEN_MASK;
	adc1->SQR1 |= SEQ_LEN_2;
	setCycle(5,CHN_3_CYLCE);
	setCycle(13,CHN_480_CYLCE);
	adc1->SQR3 = 5|(13<<5);
	adc1->CR2  |= START_CONV;
}


void setCycle(int chn,int cycle){
	adc1->SMPR1 &= CHN_MASK;
	adc1->SMPR2 &= CHN_MASK;
	if(chn>9){
		chn-=10;
		adc1->SMPR1 |= (cycle<<(chn*3));
	}
	else{
		adc1->SMPR2 |= (cycle<<(chn*3));
	}
}


//void initAdc(void){
//	gpioConfig(GpioA,5,GPIO_MODE_ANA,\
//	  		  	  GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_VHI_SPEED);
//	gpioConfig(GpioC,3,GPIO_MODE_ANA,\
//		  		  	  GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_VHI_SPEED);
//	enableAdc();
//
//
//	adc1->CR2  |= ADC_EN;
//	adc1->CR2  |= CONT_CONV;
//	adc1->CR2  |= EOCS_EN;
//	adc1->SQR1 &= SEQ_LEN_MASK;
//	adc1->SQR1 |= SEQ_LEN_2;
//	setCycle(5,CHN_3_CYLCE);
//	setCycle(13,CHN_480_CYLCE);
//	adc1->SQR3 = 5|(13<<5);
//	adc1->CR2  |= START_CONV;
//}

//void adcSetChannelSamplingSequence(AdcReg *adc, int channels[], int numOfChan){
//
//	for(int i=0;i<numOfChan;i++){
//		if(i<6)
//			adc->SQR3 |= (channels[i]<< (i*5));
//		else if(i<12){
//			adc->SQR2 |= (channels[i]<< ((i-7)*5));
//		}
//		else if(i<16)
//			adc->SQR1 |= (channels[i]<< ((i-13)*5));
//	}
//
//
//}
