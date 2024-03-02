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
	
	//Enable RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	//Set Up Red LED
	GPIOC->MODER &= ~(1<<12); //Set to alternte function mode
	GPIOC->MODER |= (1<<13);
	GPIOC->OTYPER &= ~(1<<6); //Set to push-pull mode
	GPIOC->OSPEEDR &= ~(1<<12); //Set to low speed
	GPIOC->OSPEEDR &= ~(1<<13);
	GPIOC->PUPDR &= ~(1<<12); //Set no pull-up/down
	GPIOC->PUPDR &= ~(1<<13);

	//Set Up Blue LED
	GPIOC->MODER &= ~(1<<14); //Set to alternate function mode
	GPIOC->MODER |= (1<<15);
	GPIOC->OTYPER &= ~(1<<7); //Set to push-pull mode
	GPIOC->OSPEEDR &= ~(1<<14); //Set to low speed
	GPIOC->OSPEEDR &= ~(1<<15);
	GPIOC->PUPDR &= ~(1<<14); //Set no pull-up/down
	GPIOC->PUPDR &= ~(1<<15);
	
	//Set Up GPIOC AFR for Red and Blue LEDS to TIM3
	GPIOC->AFR[0] &= ~(1<<24);
	GPIOC->AFR[0] &= ~(1<<25);
	GPIOC->AFR[0] &= ~(1<<26);
	GPIOC->AFR[0] &= ~(1<<27);
	GPIOC->AFR[0] &= ~(1<<28);
	GPIOC->AFR[0] &= ~(1<<29);
	GPIOC->AFR[0] &= ~(1<<30);
	GPIOC->AFR[0] &= ~(1<<31);
	
	//Set Up Orange LED
	GPIOC->MODER |= (1<<16); //Set to output mode
	GPIOC->MODER &= ~(1<<17);
	GPIOC->OTYPER &= ~(1<<8); //Set to push-pull mode
	GPIOC->OSPEEDR &= ~(1<<16); //Set to low speed
	GPIOC->OSPEEDR &= ~(1<<17);
	GPIOC->PUPDR &= ~(1<<16); //Set no pull-up/down
	GPIOC->PUPDR &= ~(1<<17);
	
	//Set Up Green LED
	GPIOC->MODER |= (1<<18); //Set to output mode
	GPIOC->MODER &= ~(1<<19);
	GPIOC->OTYPER &= ~(1<<9); //Set to push-pull mode
	GPIOC->OSPEEDR &= ~(1<<18); //Set to low speed
	GPIOC->OSPEEDR &= ~(1<<19);
	GPIOC->PUPDR &= ~(1<<18); //Set no pull-up/down
	GPIOC->PUPDR &= ~(1<<19);
	
	GPIOC->ODR |= (1<<6); //Set Red LED to on
	GPIOC->ODR |= (1<<7); //Set Blue LED to on
	GPIOC->ODR &= ~(1<<8); //Set Orange LED to off
	GPIOC->ODR |= (1<<9); //Set Green LED to on
	
	//TIM2 needs to be set to 4Hz starting at 8MHz
	//Set PSC to 7999 to get the clock freqency to 1kHz
	//Multiply new clock frequency by 250 to get 4Hz
	TIM2->PSC = 7999;
	TIM2->ARR = 250;
	
	TIM2->DIER |= (1<<0); // Enable the update interrupt
	
	//TIM3 needs to be set to 800Hz starting from 8MHz
	//Set PSC to 7 to get the clock frequency to 100MHz
	//Set ARR to 1250 to get 800Hz
	TIM3->PSC = 7;
	TIM3->ARR = 1250;
	
	//Set output channel 1 to PWM Mode 2
	TIM3->CCMR1 |= (1<<4);
	TIM3->CCMR1 |= (1<<5);
	TIM3->CCMR1 |= (1<<6);
	
	//Enable output compare preload for channel 1
	TIM3->CCMR1 |= (1<<3);
	
	//Set output channel 2 to PWM Mode 1
	TIM3->CCMR1 &= ~(1<<12);
	TIM3->CCMR1 |= (1<<13);
	TIM3->CCMR1 |= (1<<14);
	
	//Enable output compare preload for channel 2
	TIM3->CCMR1 |= (1<<11);
	
	//Set the output enable bits for channels 1 and 2
	TIM3->CCER |= (1<<0); // Channel 1
	TIM3->CCER |= (1<<4); // Channel 2
	
	//Set the CCRx to 20% of ARR (1250*0.2 = 250)
	TIM3->CCR1 = 250;
	TIM3->CCR2 = 250;
	
	// Enable EXTI interrupt
	NVIC_EnableIRQ(TIM2_IRQn);
	
	// Enable CR1 register of Timer (done last)
	TIM2->CR1 |= (1<<0);
	TIM3->CR1 |= (1<<0);
	
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
