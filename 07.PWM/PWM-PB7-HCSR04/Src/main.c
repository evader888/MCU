/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
void InitXXX(void);
uint8_t flag = 0;
uint16_t capturebuf[3] = {0};
uint8_t capturecnt = 0;
uint16_t high_time = 0;
float distance = 999.9f;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t len;
	uint8_t cishu;
//	uint8_t dir=0;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	InitXXX();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(DMA_usart1_Rx_Flag==1)
		{
			DMA_usart1_Rx_Flag=0;	
			RevData_Process();
			len=DMA_usart1_Rx_Size;
			for(uint8_t m=0;m<len;m++)
			{
				USART1_DMA_TX_BUFFER[m] = USART_RX_BUF[m];
			}
      USER_DMA_send(USART1_DMA_TX_BUFFER,len);
		}
		
    if(flag)
		{
			switch(capturecnt)
			{
				case 0:
					capturecnt++;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				  HAL_Delay(1);
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
				  HAL_Delay(1);
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_RISING);
					HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
				break;
				case 3:
					high_time = capturebuf[1] - capturebuf[0];
				  capturecnt = 0;
				  HAL_Delay(100);
				  
				  distance = high_time*0.017;
				  printf("%0.2f cm ",distance);
				
				  if(distance<60.0f)
					{
						cishu++;
						if(cishu>10)
						{
							cishu = 10;
							//TIM4->CCR2 = 1500;
							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
							HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
							
						}
					}
					else
					{
						cishu=0;
						//TIM4->CCR2 = 2500;
						HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
					}
					printf("  %d \r\n",cishu);
				break;
				
			}
			
			
			
//			lastcount = tim2count;
//			
//			while((tim2count-lastcount)<=1){};
//			
//      lastcount = tim2count;
//			while((tim2count-lastcount)<=1){};
//			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);

//			while(0==HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)){};
//      lastcount = tim2count;
//			while(1==HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)){};
//	    
//			timetotalus = tim2count-lastcount;
//				
//      dista= timetotalus *0.34f;
//			distance=(uint16_t)dista;

		}
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void InitXXX(void)
{
	//HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量

	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//使能IDLE中断
	HAL_UART_Receive_DMA(&huart1,USART1_DMA_RX_BUFFER,USART1_DMA_RX_SIZE);//打开DMA接收，数据存入USART1_DMA_RX_BUFFER数组中

	//HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	//HAL_TIM_Base_Start_IT(&htim2);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case GPIO_PIN_13:
			flag ^=1;
			break;
		default:
			break;
		
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		switch(capturecnt)
		{
			case 1:
				capturebuf[0] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);
			  __HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_FALLING);
			  capturecnt++;
			
				break;
			case 2:
				capturebuf[1] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);
			  HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_2);
			  capturecnt++;
				break;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
