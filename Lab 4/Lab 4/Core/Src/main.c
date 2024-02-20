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


// Declaration of global variables
char global_char;
volatile char *read_ready;

/* Private includes ----------------------------------------------------------*/
void SystemClock_Config(void);



/**
* @brief send char to device
* @param c char to send
**/
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

/**
* @brief send string to device
* @param str string to send
**/
void USART_TransmitString(const char* str){
			int i = 0; // Counter for indexing the array
	    while (str[i] != '\0') { // Loop until null character is encountered
        USART_TransmitChar(str[i]); // Transmit the current character
        i++; // Move to the next character in the array
    }
}

/**
	*handler for interrupt
**/
void USART3_4_IRQHandler(void){
	global_char = USART3->RDR;
	*read_ready = 't';
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	char test = 'f';
	read_ready = &test;
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
	
	//enable transmit and enable and enable usart and interrupt
	USART3->CR1 |=(1<<5);
	USART3->CR1 |=(1<<2);
	USART3->CR1 |=(1<<3);
	USART3->CR1 |=(1<<0);
	
	//enable global interrupt
	NVIC_EnableIRQ(USART3_4_IRQn);
NVIC_SetPriority(USART3_4_IRQn,2);


  while (1)
  {
		USART_TransmitString("Choose LED\n\r");
		
//check for button input
	while(*read_ready != 't'){
	}
	
			//select LED
	int led;
		    switch(global_char) {
        case 'g':
					//green LED
            led = 9;
				USART_TransmitString("Green Selected:\n\r");
				*read_ready = 'f';
            break;
				
        case 'b':
					//blue LED
            led = 7;
				USART_TransmitString("Blue Selected:\n\r");
				*read_ready = 'f';
            break;

        case 'r':
					//red LED
            led = 6;
				USART_TransmitString("Red Selected:\n\r");
				*read_ready = 'f';
            break;
				
				 case 'o':
					 //orange LED
            led = 8;
				 USART_TransmitString("Orange Selected:\n\r");
				 *read_ready = 'f';
            break;
        default:
				USART_TransmitString("Invalid choice\n\r");
				*read_ready = 'f';
					continue;
    }
				//second loop
	while(*read_ready != 't'){	
	}
			    switch(global_char) {
						
        case '0':
					//led OFF
				GPIOC->ODR &=~(1<<led);
				USART_TransmitString("LED OFF:\n\r");
				*read_ready = 'f';
            break;
				
        case '1':
        	//led ON
				GPIOC->ODR |=(1<<led);
				USART_TransmitString("LED ON:\n\r");
				*read_ready = 'f';
            break;
				
        case '2':
          //led toggle
				GPIOC->ODR ^=(1<<led);
				USART_TransmitString("LED TOGGLE:\n\r");
				*read_ready = 'f';
            break;
				
        default:
				USART_TransmitString("Invalid choice\n\r");
				*read_ready = 'f';
					continue;
    }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n\r", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
