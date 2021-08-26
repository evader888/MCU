/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	 
//Target--Code Generation--Use MicroLIB(XXX)
#if 1
#pragma import(__use_no_semihosting)             
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;   
void _sys_exit(int x) 
{ 
	x = x; 
}
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0){}; 
    USART1->DR = (uint8_t) ch;      
	return ch;
}
#endif 

//串口1中断服务程序	
uint8_t USART_RX_BUF[USART_MAX_LEN];     //接收缓冲,最大USART_MAX_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART_RX_STA = 0;       //接收状态标记

uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲

uint8_t USART1_DMA_TX_BUFFER[USART1_DMA_TX_SIZE];
uint8_t USART1_DMA_RX_BUFFER[USART1_DMA_RX_SIZE];

volatile uint8_t DMA_usart1_Rx_Size;
volatile uint8_t DMA_usart1_Rx_Flag = 0;
volatile uint8_t DMA_usart1_Tx_Flag = 1;

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }


	
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */
    
  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(huart->Instance==USART1)//如果是串口1
	{
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else 
				{
					USART_RX_STA|=0x8000;	//接收完成了 
				}
			}
			else //还没收到0X0D
			{	
				if(aRxBuffer[0]==0x0d)
				{
					USART_RX_STA|=0x4000;
				}
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_MAX_LEN-1))
						USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}
		
		HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
	}
}

void USER_DMA_send(uint8_t *buf,uint8_t len)
{
	if( 1 == DMA_usart1_Tx_Flag)
	{
		HAL_UART_Transmit_DMA(&huart1,buf,len);
		DMA_usart1_Tx_Flag=0;
	}
}

void RevData_Process(void)
{
	if(USART_RX_BUF[0]=='#')
	{
		if(USART_RX_BUF[1]=='l'&&USART_RX_BUF[2]=='e'&&USART_RX_BUF[3]=='d')
		{
			if(USART_RX_BUF[5]=='o'&&USART_RX_BUF[6]=='n')
			{
				switch(USART_RX_BUF[4])
				{
					case '1':
						HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_RESET);
						break;
					case '2':
						HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
						break;
					case '3':
						HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_RESET);
						break;
					case '4':
						HAL_GPIO_WritePin(GPIOE, LED4_Pin, GPIO_PIN_RESET);
						break;
					case '5':
						HAL_GPIO_WritePin(GPIOE, LED5_Pin, GPIO_PIN_RESET);
						break;
					case '6':
						HAL_GPIO_WritePin(GPIOE, LED6_Pin, GPIO_PIN_RESET);
						break;
					case '7':
						HAL_GPIO_WritePin(GPIOE, LED7_Pin, GPIO_PIN_RESET);
						break;
					case '8':
						HAL_GPIO_WritePin(GPIOE, LED8_Pin, GPIO_PIN_RESET);
						break;
					default:
            break;						
					}
			}
			else if(USART_RX_BUF[5]=='o'&&USART_RX_BUF[6]=='f'&&USART_RX_BUF[7]=='f')
			{
				switch(USART_RX_BUF[4])
				{
					case '1':
						HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
						break;
					case '2':
						HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
						break;
					case '3':
						HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_SET);
						break;
					case '4':
						HAL_GPIO_WritePin(GPIOE, LED4_Pin, GPIO_PIN_SET);
						break;
					case '5':
						HAL_GPIO_WritePin(GPIOE, LED5_Pin, GPIO_PIN_SET);
						break;
					case '6':
						HAL_GPIO_WritePin(GPIOE, LED6_Pin, GPIO_PIN_SET);
						break;
					case '7':
						HAL_GPIO_WritePin(GPIOE, LED7_Pin, GPIO_PIN_SET);
						break;
					case '8':
						HAL_GPIO_WritePin(GPIOE, LED8_Pin, GPIO_PIN_SET);
						break;
					default:
            break;						
					}
				
			}
		}
		else if(USART_RX_BUF[1]=='c'&&USART_RX_BUF[2]=='m'&&USART_RX_BUF[3]=='d')
		{
			switch(USART_RX_BUF[4])
			{
				case '1':
					HAL_GPIO_WritePin(GPIOE, LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin 
                          |LED2_Pin|LED1_Pin|LED8_Pin|LED7_Pin, GPIO_PIN_RESET);
					break;
				case '2':
					HAL_GPIO_WritePin(GPIOE, LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin 
                          |LED2_Pin|LED1_Pin|LED8_Pin|LED7_Pin, GPIO_PIN_SET);
					break;
				case '3':
					break;
				case '4':
					break;
				case '5':
					break;
				case '6':
					break;
				case '7':
					break;
				case '8':
					break;
				case '9':
					break;
				default:
					break;
			}
		}
	}
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
