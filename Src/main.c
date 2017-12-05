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
#include "DbgMcu.h"
#include "I2C.h"
#include <stdio.h>

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
  MX_GPIO_Init();

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
  enableGpio(0);		// port A

  //**********Configure GPIO************************
  gpioConfig(GpioA,blueButtonPin,GPIO_MODE_IN,\
  		  	  0,GPIO_NO_PULL,0);
  gpioConfig(GpioG,greenLedPin,GPIO_MODE_OUT,\
  		  	  GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_LOW_SPEED);

  gpioConfig(GpioA,8,GPIO_MODE_AF,\
  		  	  GPIO_PUSH_PULL,GPIO_NO_PULL,GPIO_VHI_SPEED);
  gpioConfigAltFunction(GpioA,8,ALT_FUNCT0);

  //rccSelectMco1Src(HI_SPEED_EXT);
  //rccSelectMco1Prescale(MCO_DIV_BY_5);

  //************Halt Timer8***************************
  //haltTimer8WhenDebugging();


  //************Enable Timer8***************************
  //initTimer8();
  //initTimer8Channel1();

  //getRandomNumberByInterrupt();
  int i=0;

  //Start I2c

  initI2C();
  haltI2c1WhenDebugging();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int i;
	  while (1){
		  i++;
	  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
	gpioWrite(GpioG,redLedPin,(ledState = !ledState));

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

void wait500ms(){
	while(!(Timer8->SR & 1));					// Check update interrupt flag(UIF) of TIM8_SR
	Timer8->SR &= ~1;							// Clear the flag
}
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

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
