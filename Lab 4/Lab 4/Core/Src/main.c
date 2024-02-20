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

void USART_TransmitChar(char c) {
    // Implementation of character transmit function can be added here
    // For demonstration purposes, we'll simply print the character
	while(1){
		if((USART3->ISR & (1<<7)) == (1<<7)){
			break;
		}
	}
    USART3->TDR = c;
}

void USART_TransmitString(const char* str){
			int i = 0; // Counter for indexing the array
	    while (str[i] != '\0') { // Loop until null character is encountered
        USART_TransmitChar(str[i]); // Transmit the current character
        i++; // Move to the next character in the array
    }
}

void setLED(void){
	    switch(USART3->RDR) {
        case 'g':
					//green LED
            GPIOC->ODR ^=(1<<9);
            break;
        case 'b':
            GPIOC->ODR ^=(1<<7);
            break;
        case 'r':
            GPIOC->ODR ^=(1<<6);
            break;
				 case 'o':
            GPIOC->ODR ^=(1<<8);
            break;
        default:
					USART_TransmitString("Invalid choice\n");
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();


  SystemClock_Config();
	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
		//configure leds
	
GPIOC->MODER &=~(1<<19); 
GPIOC->MODER |=(1<<18);
GPIOC->MODER &=~(1<<17); 
GPIOC->MODER |=(1<<16);
GPIOC->MODER &=~(1<<15); 
GPIOC->MODER |=(1<<14); 
GPIOC->MODER &=~(1<<13); 
GPIOC->MODER |=(1<<12); 
	
GPIOC->OTYPER &=~(1<<6);
GPIOC->OTYPER &=~(1<<7);
GPIOC->OTYPER &=~(1<<8);
GPIOC->OTYPER &=~(1<<9);
	
GPIOC->OSPEEDR &=~(1<<18); 
GPIOC->OSPEEDR &=~(1<<16);
GPIOC->OSPEEDR &=~(1<<14);
GPIOC->OSPEEDR &=~(1<<12); 
 
	
GPIOC->PUPDR &=~(1<<19); 
GPIOC->PUPDR &=~(1<<18);
GPIOC->PUPDR &=~(1<<17); 
GPIOC->PUPDR &=~(1<<16);
GPIOC->PUPDR &=~(1<<15); 
GPIOC->PUPDR &=~(1<<14);
GPIOC->PUPDR &=~(1<<13); 
GPIOC->PUPDR &=~(1<<12); 

GPIOC->ODR |=(1<<9);
GPIOC->ODR |=(1<<8);
GPIOC->ODR |=(1<<7);
GPIOC->ODR |=(1<<6);
	
	//set alternate function mode
	GPIOB->MODER  |=(1<<21);
	GPIOB->MODER  |=(1<<23);
	GPIOB->MODER &=~(1<<20);
	GPIOB->MODER &=~(1<<22);
	
	//set AFR
	GPIOB->AFR[1] |=(1<<10);
	GPIOB->AFR[1] |=(1<<14);
	
	//set Baud Rate
	USART3->BRR =  HAL_RCC_GetHCLKFreq()/9600;
	
	//enable transmit and enable and enable usart
	
	USART3->CR1 |=(1<<2);
	USART3->CR1 |=(1<<3);
	USART3->CR1 |=(1<<0);
	


	

  while (1)
  {

	while(1){
		if((USART3->ISR & (1<<5)) == (1<<5)){
			break;
		}
	}

	setLED();
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
