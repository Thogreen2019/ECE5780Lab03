/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
 
  HAL_Init();
  SystemClock_Config();
	
	RCC -> APB1ENR |= 1; //Enables TIM2, the first bit of RCC -> APB1ENR
	RCC -> AHBENR |= 1<<17; //Enables the clock on IO Port A, which is AHBENR bit 17.
	
	//Set up Green and Orange LEDs
	GPIOC -> MODER |= (1<<18); //Set Green LED to Output
	GPIOC -> OTYPER &= (0<<9); //Set Green LED to no push/pull
	GPIOC -> OSPEEDR &= (0<<18); //Set Green LED to Low Speed
	GPIOC -> PUPDR &= (0<<18); //Set Green LED to no Pull up/down
	
	GPIOC -> MODER |= (1<<16); //Set Orange LED to Output
	GPIOC -> OTYPER &= (0<<8); //Set Orange LED to no push/pull
	GPIOC -> OSPEEDR &= (0<<16); //Set Orange LED to Low Speed
	GPIOC -> PUPDR &= (0<<16); //Set Orange LED to no Pull up/down
	
	GPIOC -> ODR |= (1<<9); //Set Green LED to on
	
	//Next, configure the timer (UEV) to update at 4Hz
	//4Hz = fclk/((PSC+1)*ARR), fclk = 8MHz, PSC + 1 = 8000 -> PSC = 7999
	//-> 4Hz = 1kHz/ARR -> ARR = 250
	TIM2 -> PSC |= 7999;
	TIM2 -> ARR |= 250;
	
	//Next, Configure the Timer's DMA to update the interrupt (bit 0 of TIMx->DIER):
	TIM2 -> DIER |= 1;
	
	//Enable the Interrupts from the timer
	NVIC_EnableIRQ(15);
	
	//Set Interrupt's Priority
	NVIC_SetPriority(15, 1);
	
	//Enable Timer (Using bit 0 of the CR1 Register). Note that this is done last.
	TIM2 -> CR1 |= 1;
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
}


/*
*
* Timer Interrupt Handler
*
*/
void TIM2_IRQHandler(void){
	if(GPIOC -> ODR & (1<<8)){ //Orange LED is on and Green is off
			GPIOC -> ODR -= (1<<8); //Turn Orange LED off and Green on
		  GPIOC -> ODR += (1<<9);
		}else{ //Orange LED is off and Green is on
			GPIOC -> ODR += (1<<8); //Turn Orange LED on and Green off
			GPIOC -> ODR -= (1<<9);
		}
		TIM2 -> SR &= 0; //Clear Pending Flag
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
