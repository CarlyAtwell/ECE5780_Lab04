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

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


void transmitChar(char c)
{

	while( !((USART3->ISR) &(1 << 7)) ){
		
	}
	
	GPIOC->ODR |= (1 << 6); // debug light to show through the while loop

	USART3->TDR = c;
}

void transmitString(char carray[])
{
	int length = sizeof(*carray)/sizeof(char);
	for( int i =0; i < length ; i++){
		if(carray[i] == 0){
			break;
		}
		else{
		transmitChar(carray[i]);
		}
	}
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // enable gpiob clock in rcc
	RCC-> AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the GPIOC clock in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // enable the usart3 clock in rcc
	
	// configure system clock to usart3 clock
	RCC->CFGR3 |= (1 << 18);
	RCC->CFGR3 &= ~(1 << 19);
	
	// Set pins PC8 and PC9 to output mode
	GPIOC->MODER |= (1 << 12) |  (1 << 14);
	GPIOC->MODER &= ~((1 << 13) | (1 << 15));
	// Set pins PC8 and PC9 to output push-pull
	GPIOC->OTYPER &= ~((1 << 6) | (1 << 7));
	// Set pins PC8 and PC9 to low speed
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14));
	// Set pins PC8 and PC9 to no pullup, pulldown
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15));
	
	// set pins pb10 and pb11 to alternate function mode
	GPIOB->MODER |= (1 << 21) | (1 << 23);
	GPIOB->MODER &= ~((1 << 20) | (1 << 22));
	
	// pins connect to usart3
	GPIOB->AFR[1] |= GPIO_AF4_USART3;
	
	// set baud rate to 115200
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
	
	// enable transmitter and receiver
	USART3->CR1 |= (1 << 3) | (1 << 2) | (1 << 0);
	
	// Set PC6 low
	GPIOC->ODR &= ~(1 <<6);
	
  while (1)
  {
		//HAL_Delay(200);
		
		//GPIOC->ODR |= GPIO_ODR_6;

		transmitChar('A');
		transmitChar('A');

	}
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
