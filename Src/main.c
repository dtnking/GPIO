/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include "GPIO.h"
#include "RCC.h"
#include "RNG.h"
#include "NVIC.h"
#include "SysCfg.h"
#include "SysTick.h"
#include "EXTI.h"
#include "Timer.h"
#include "Flash.h"
#include "DbgMcu.h"
#include "I2C.h"
#include <stdio.h>
#include <malloc.h>
#include "USART.h"
#include <string.h>
#include "Dma.h"
#include "Adc.h"
#include "IWDG.h"

#define redLedPin  		14
#define greenLedPin  	13
#define blueButtonPin 	0
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void initialise_monitor_handles(void);
void printCausedReset(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	initialise_monitor_handles();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
 // MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
 printf("Hello,world!\n");

  //Enable RNG & HASH Interrupt
  //nvicEnableIrq(80);
  //nvicSetPriority(80,4);

  //***********Enable EXTI0 Interrupt***************
 /*
   nvicEnableIrq(6);
   nvicSetPriority(6,4);
   sysCfgConfigureGPIO(0,PORTA);
   extIntClearPR();
   extIntEnable(0);
   extMaskDisable(0);
   setRisingEdge();
*/
 enableGpio(0);
 enableGpio(2);
  //***********Enable SysTick***********************
  /*sysTickSetReload(11250000);
  sysTickPrescaledSpeed();
  sysTickClrCounter();
  sysTickEnable();
  sysTickInterEnable();
  sysTickDisable();
  */

  //***********Enable RNG*************************
  //enableRng();

  //***********Enable GPIO*************************
  enableGpio(6);		// port G
//  enableGpio(0);		// port A

  //**********Configure GPIO************************
//  gpioConfig(GpioA,blueButtonPin,GPIO_MODE_IN,\
//  		  	  0,GPIO_NO_PULL,0);
  gpioConfig(GpioG,greenLedPin,GPIO_MODE_OUT,\
  		  	  GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_VHI_SPEED);
  gpioConfig(GpioG,redLedPin,GPIO_MODE_OUT,\
   		  	  GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_VHI_SPEED);
//
//  gpioConfig(GpioA,8,GPIO_MODE_AF,\
//  		  	  GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_VHI_SPEED);
//  gpioConfigAltFunction(GpioA,8,ALT_FUNCT0);

  //rccSelectMco1Src(HI_SPEED_EXT);
  //rccSelectMco1Prescale(MCO_DIV_BY_5);

  //************Halt Timer8***************************
  //haltTimer8WhenDebugging();


  //************Enable Timer8***************************
  //initTimer8();
  //initTimer8Channel1();
  //configureTimer8(125,1);

  //************Enable ADC***************************
  // initAdc();



  //***********Enable DMA******************************
  // enableDMA(DMA2_DEV);
  // dmaInitForUsart1(dma2,2,CH4,INCR4,INCR4,DBM_DIS,PL_HI,PINCOS_DIS,MSIZE_BYTE,PSIZE_BYTE,MINC_EN,PINC_DIS,CIRC_DIS,DIR_P_TO_M,PFCTRL_DMA);
  //dmaInitForUsart1(dma2,7,CH4,INCR4,INCR4,DBM_DIS,PL_HI,PINCOS_DIS,MSIZE_BYTE,PSIZE_BYTE,MINC_EN,PINC_DIS,CIRC_DIS,DIR_M_TO_P,PFCTRL_DMA);
  char str[256] = "hello, world!\n";


 // dmaInitForUsart1(dma2,2,CH7,INCR4,SIN_TRANFER,DBM_DIS,PL_VHI,PINCOS_DIS,MSIZE_HALFWORD,PSIZE_HALFWORD,MINC_EN,PINC_DIS,CIRC_DIS,DIR_M_TO_P,PFCTRL_DMA);
 // dmaSetAddressAndSize(dma2,7,(uint32_t)str,0x40011004,strlen(str));

  //getRandomNumberByInterrupt();

//  char data[250];
//	usart1->CR1 |= TRANSMIT_EN;
//
//  initUsart1();

//  char *Data = (char*)malloc(sizeof(char) * 100);

 // uint16_t timerWaveform[]= {11,125,11,125,125+2};
 // serialPrint("Data: %d, %s", 123, "Hello");
  //writeMessage("Hello World",(char *)0x08084000);
 // toggleOutCompareChannel1WithForce();
//  forceOutCompareChannel1High();
//  Timer8->CCMR1 &= ~(7<<4);
//  Timer8->CCMR1 |= (3<<4);
//  Timer8->DIER	|= (1<<9);
//  dmaSetAddressAndSize(dma2,2,(uint32_t)timerWaveform,&Timer8->CCR1,strlen(timerWaveform));




  //******************Enable FLASH******************
 /* flashEraseSector(13);
  if(flashEraseSector(13)==1){
	 flashEnableProgramming(FLASH_BYTE_SIZE);
	 writeMessage("Hello World",(char *)0x08084000);
	 flashDisable(0);
	  while(1);
  }else{
	  while(1);
  }*/

  //Start I2c
  //initI2C();
  //haltI2c1WhenDebugging();
  int i=0;

  // Experiment for WWDG as a time keeper starts
  printCausedReset();
  rccClearAllResetFlags();
  enableWWDG();

  nvicEnableIrq(0);
  nvicSetPriority(0,10);

  wwdgSetWindowValue(55);			// Set Window to 5ms
  wwdgSetPrescaler(WWDG_COUNTER_CLK_DIV_2);
  wwdgSetTimeOutAndActivate(55);		// Set timeout
  clearEarlyWakeupInterrupt();
  wwdgEnableWakeupInterrupt();

  while(1){
	  gpioWrite(GpioG,greenLedPin,1);
	  waitFor500ms();
	  gpioWrite(GpioG,greenLedPin,0);
	  waitFor500ms();
  }
  // Experiment for WWDG as a time keeper ends


// Independent Watchdog Experiment starts
//  printCausedReset();
//  rccClearAllResetFlags();
//  HAL_Delay(1000);
//  iwdgStart();
//  iwdgEnableConfiguration();
//  iwdgWaitTillPrescalerDivided(PRESCALER_DIV_64);
//  iwdgWaitTillLoaded(2000);
//  iwdgReset();
// Independent Watchdog Experiment ends

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  //Window Watchdog Experiment Starts
	  HAL_Delay(2);						//Change the digit to get different situation (within the window will not reset)
	  wwdgSetTimeOutAndActivate(63);	// Refresh the value
	  //Window Watchdog Experiment Ends

//	  gpioWrite(GpioG,greenLedPin,1);
//	  HAL_Delay(50);
//	  gpioWrite(GpioG,greenLedPin,0);
//	  HAL_Delay(50);


// ADC Experiment starts
//	  while(adc1->CR2 & EOC_IS_SET){
//		  float x= (3.3* (adc1->DR))/4096;
//		  printf("%f\n",x);
//		  adc1->SR &= RESET_OVR;
//		  adc1->CR2  |= START_CONV;
//	  }
// ADC Experiment ends

//
//	  while(Timer8->SR &= CC1F_MATCH){
//	  	  		Timer8->CCR1 = temp[y];
//	  	  		y++;
//	  	  		if(y>1)
//	  	  			y=0;
//	  }

// USART Experiment starts
//	  usartReceiveUntilEnter(Data);
//
//		  if(strcmp("turn on", Data) == 0){
//			  gpioWrite(GpioG,redLedPin,1);
//		  }
//		  else if(strcmp("turn off",Data) == 0){
//			  gpioWrite(GpioG,redLedPin,0);
//		  }
// 	  usartTransmit("h");
//	  usartTransmit("e");
//	  usartTransmit("l");
//	  usartTransmit("l");
//	  usartTransmit("o");
//	  usartTransmit(" ");
//	  usartTransmit("w");
//	  usartTransmit("o");
//	  usartTransmit("r");
//	  usartTransmit("l");
//	  usartTransmit("d");
//	  usartTransmit("!");
//	  usartTransmit("\n");
// USART Experiment ends



	 // gpioWrite(GpioG,greenLedPin,1);
	 // wait500ms();
	 // gpioWrite(GpioG,greenLedPin,0);
	 // wait500ms();

	  //__WFI();
	  //gpioWrite(GpioG,redLedPin,0);
	  /* gpioWrite(GpioG,redLedPin,1);
	  while(!sysTickHasExpired())
		  gpioWrite(GpioG,redLedPin,0);
	  while(!sysTickHasExpired())
		  gpioWrite(GpioG,redLedPin,1);
	 // int num = getRandomNumber();
	 // printf("%d 0x%x\n", i++ , num);
	 // volatile int blueButtonState;
	 //volatile int lockPattern;
	  gpioWrite(GpioG,redLedPin, 0);
	  HAL_Delay(200);
	  gpioWrite(GpioG,redLedPin, 1);
	  HAL_Delay(200);
	  GpioG->lock = (1<<16)|(1<<redLedPin);
	  GpioG->lock = 1 << redLedPin;
	  GpioG->lock = (1<<16)|(1<<redLedPin);
	  lockPattern = GpioG->lock;
	  gpioConfig(GpioG,redLedPin,GPIO_MODE_IN,\
	  		  	  GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_HI_SPEED);


	  blueButtonState = gpioRead(GpioA,blueButtonPin);
	  if(blueButtonState == 1){
		  gpioWrite(GpioG,greenLedPin,1);
	  } else{
		  gpioWrite(GpioG,greenLedPin,0);
	  }
	  */
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

void waitFor500ms(){
	int y=0;
//	wwdgSetWindowValue(55);			// Set Window to 5ms
//	wwdgEnableWakeupInterrupt();
//	wwdgSetPrescaler(WWDG_COUNTER_CLK_DIV_2);
//	wwdgSetTimeOutAndActivate(55);		// Set timeout
	while(y++<50){
		while(!wwdgIsEarlyWakeupInterrupt());
		wwdgSetTimeOutAndActivate(55);			//10 ms
		clearEarlyWakeupInterrupt();
	}

}

void printCausedReset(void){
	printf("Cause of reset : \n");
	if(rcc->csr & RCC_LPWRRSTF)
		printf("\tLow Power Reset\n");
	if(rcc->csr & RCC_WWDGRSTF)
		printf("\tWindow Watchdog Reset\n");
	if(rcc->csr & RCC_IWDGRSTF)
		printf("\tIndependent Watchdog Reset\n");
	if(rcc->csr & RCC_SFTRSTF)
		printf("\tSoftware Reset\n");
	if(rcc->csr & RCC_PORRSTF)
		printf("\tPower On Reset\n");
	if(rcc->csr & RCC_PINRSTF)
		printf("\tNSRT Reset\n");
	if(rcc->csr & RCC_BORRSRSTF)
		printf("\tBrownout Reset\n");
	if(rcc->csr & RCC_RMVF)
		printf("\tRemove Reset\n");
	else
		printf("\tNo Reset");
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
 // __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void){
	extIntClearPR();
	static volatile int count = 0;
	gpioWrite(GpioG,redLedPin,1);
	count ++;
	gpioWrite(GpioG,redLedPin,0);

}

void My_SysTick_Handler(void){
	static int ledState = 0;
// Just do nothing , but reading the CTRL register to clear the counter flag.
	volatile int flags = sysTick->CTRL;
//	gpioWrite(GpioG,redLedPin,(ledState = !ledState));

}

void WWDG_IRQHandler(void){
	static int i = 0;
	wwdgSetTimeOutAndActivate(55);			//10 ms
	clearEarlyWakeupInterrupt();
	if(i++ >= 50){
		gpioToggle(GpioG,redLedPin);
		i=0;
	}

}
void HASH_RNG_IRQHandler(void){
	volatile int rand = Rng->DR;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
//}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
