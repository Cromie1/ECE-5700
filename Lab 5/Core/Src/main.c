/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Gyroscopic sensing code to control LED's
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



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  Main function to control application.
  * @retval int
  */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	//setup RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

	//configure LEDS
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

	GPIOC->ODR &=~(1<<9);
	GPIOC->ODR &=~(1<<8);
	GPIOC->ODR &=~(1<<7);
	GPIOC->ODR &=~(1<<6);
	
	//begin setup of I2C2
	// set pb11 to alternate function
	GPIOB->MODER &=~(1<<22); 
	GPIOB->MODER |=(1<<23);

	GPIOB->OTYPER |=(1<<11);
	
	//set afr
	GPIOB->AFR[1] |=(1<<12);

	// set pb13 to alternate function
	GPIOB->MODER &=~(1<<26); 
	GPIOB->MODER |=(1<<27);

	GPIOB->OTYPER |=(1<<13);
	
	//set afr
	GPIOB->AFR[1] |=(1<<22);
	GPIOB->AFR[1] |=(1<<20);

	//set pb14
	GPIOB->MODER |=(1<<28);
	GPIOB->OTYPER &=~(1<<14);
	//set pb14 high
	GPIOB->ODR |=(1<<14);

	//set pc0
	GPIOC->MODER |=(1<<0);
	GPIOC->OTYPER &=~(1<<0);
	//set pb14 high
	GPIOC->ODR |=(1<<0);

	//set to 100kHz
	I2C2->TIMINGR |= 0x13;
	I2C2->TIMINGR |= (0xF<<8);
	I2C2->TIMINGR |= (0x2<<16);
	I2C2->TIMINGR |= (0x4<<20);
	I2C2->TIMINGR |= (0x1<<28);

	//i2c enable
	I2C2->CR1 |=(1<<0);

	//begin Setup of Gyrosopic sensor
	
	//Set slave Address
	I2C2->CR2 |= (0x69<<1); 
	//set 1 byte
	I2C2->CR2 |= (1<<16);
	//RD WRN
	I2C2->CR2 &=~(1<<10); 
	//START
	I2C2->CR2 |=(1<<13); 

	//wait for TXIS
  while (1)
  {
		if((I2C2->ISR & (1<<1))){
	
			break;
		}
  }
	//write CR!
	I2C2->TXDR = 0x20;

  
		//wait until transfer complete
	  while (1)
  {
		if((I2C2->ISR & (1<<6))){
			break;
		}
  }
		I2C2->CR2 |=(1<<14);
	
	I2C2->CR2 |= (0x69<<1); 
//set n bytes
I2C2->CR2 |= (1<<16);
//RD WRN
I2C2->CR2 &=~(1<<10); 
//START
I2C2->CR2 |=(1<<13); 

  while (1)
  {
		if((I2C2->ISR & (1<<1))){
	
			break;
		}
  }
	//write CR!
	I2C2->TXDR = 0xE;

		//wait until transfer complete
	  while (1)
  {
		if((I2C2->ISR & (1<<6))){
			break;
		}
  }
	
	//stop 
	I2C2->CR2 |=(1<<14);
	char xL;
	char xH;
	char yL;
	char yH;
	int16_t x;
	int16_t y;
	while(1){
	
	HAL_Delay(100);
	

	
	//get xL
	//set slave address
I2C2->CR2 |= (0x69<<1); 
//set n bytes
I2C2->CR2 |= (1<<16);
I2C2->CR2 &=~(1<<17);	
//RD WRN
I2C2->CR2 &=~(1<<10); 
//START
I2C2->CR2 |=(1<<13); 

  while (1)
  {
		if((I2C2->ISR & I2C_ISR_TXIS)){
	
			break;
		}
  }
	//write Out_X_L
	I2C2->TXDR = 0xA8;
	
	//wait until transfer complete
	  while (1)
  {
		if((I2C2->ISR & (1<<6))){
			break;
		}
  }
	
	//set slave address
I2C2->CR2 |= (0x69<<1); 
//set n bytes
I2C2->CR2 |= (1<<17); 
I2C2->CR2 &=~(1<<16);
//RD WRN
I2C2->CR2 |=(1<<10); 
//START
I2C2->CR2 |=(1<<13);
	//wait until RXNE
	  while (1)
  {
		if((I2C2->ISR & I2C_ISR_RXNE)){
			break;
		}
  }
		xL = I2C2->RXDR;

	
		//wait until RXNE
	  while (1)
  {
		if((I2C2->ISR & I2C_ISR_RXNE)){
			break;
		}
  }
		xH = I2C2->RXDR;
		//wait until transfer complete
	  while (1)
  {
		if((I2C2->ISR & (1<<6))){
			break;
		}
  }

			I2C2->CR2 |=(1<<14);
	
	
	
	
	
	//y values 
	I2C2->CR2 |= (0x69<<1); 
//set n bytes
I2C2->CR2 |= (1<<16);
I2C2->CR2 &=~(1<<17);	
//RD WRN
I2C2->CR2 &=~(1<<10); 
//START
I2C2->CR2 |=(1<<13); 

  while (1)
  {
		if((I2C2->ISR & I2C_ISR_TXIS)){
	
			break;
		}
  }
	//write Out_X_L
	I2C2->TXDR = 0xAA;
	
	//wait until transfer complete
	  while (1)
  {
		if((I2C2->ISR & (1<<6))){
			break;
		}
  }
	
	//set slave address
I2C2->CR2 |= (0x69<<1); 
//set n bytes
I2C2->CR2 |= (1<<17); 
I2C2->CR2 &=~(1<<16);
//RD WRN
I2C2->CR2 |=(1<<10); 
//START
I2C2->CR2 |=(1<<13);
	//wait until RXNE
	  while (1)
  {
		if((I2C2->ISR & I2C_ISR_RXNE)){
			break;
		}
  }
		yL = I2C2->RXDR;

	
		//wait until RXNE
	  while (1)
  {
		if((I2C2->ISR & I2C_ISR_RXNE)){
			break;
		}
  }
		yH = I2C2->RXDR;
		//wait until transfer complete
	  while (1)
  {
		if((I2C2->ISR & (1<<6))){
			break;
		}
  }

			I2C2->CR2 |=(1<<14);
	
	x = (xH<<8) | xL;
	y = (yH<<8) | yL;
if(x > 500){
	GPIOC->ODR |=(1<<9);
	}
else{
GPIOC->ODR &=~(1<<9);
}
if(x < -500){
	GPIOC->ODR |=(1<<8);
	}
else{
GPIOC->ODR &=~(1<<8);
}

if(y > 500){
	GPIOC->ODR |=(1<<6);
	}
else{
GPIOC->ODR &=~(1<<6);
}
if(y < -500){
	GPIOC->ODR |=(1<<7);
	}
else{
GPIOC->ODR &=~(1<<7);
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
