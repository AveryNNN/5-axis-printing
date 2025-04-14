/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   使用串口3，重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  * @attention
  *
  * 实验平台:SKR PRO V1.2控制板
  *
  ******************************************************************************
  */ 
  
#include "usart.h"

UART_HandleTypeDef UartHandle;

/* 命令接收完成 */
uint8_t receive_cmd = 0;

 /**
  * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1
  * @param  无
  * @retval 无
  */  
void SERIAL_USART_Config(unsigned int baud)
{ 
  
  UartHandle.Instance          = SERIAL_USART;
  
  UartHandle.Init.BaudRate     = baud;
  UartHandle.Init.WordLength   = USART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = USART_STOPBITS_1;
  UartHandle.Init.Parity       = USART_PARITY_NONE;
  UartHandle.Init.Mode         = USART_MODE_TX_RX;
  
  HAL_UART_Init(&UartHandle);
    
  /*使能串口接收中断 */
  __HAL_UART_ENABLE_IT(&UartHandle, USART_IT_RXNE);
}

/**
  * @brief UART MSP 初始化 
  * @param huart: UART handle
  * @retval 无
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  if(huart->Instance == SERIAL_USART)
  {
    SERIAL_USART_CLK_ENABLE();
    
    SERIAL_USART_RX_GPIO_CLK_ENABLE();
    SERIAL_USART_TX_GPIO_CLK_ENABLE();
    
    /**USART3 GPIO Configuration    
      PD8     ------> USART3_TX
      PD9     ------> USART3_RX 
    */
    /* 配置Tx引脚为复用功能 */
    GPIO_InitStruct.Pin = SERIAL_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = SERIAL_USART_TX_AF;
    HAL_GPIO_Init(SERIAL_USART_TX_GPIO_PORT, &GPIO_InitStruct);
    
    /* 配置Rx引脚为复用功能 */
    GPIO_InitStruct.Pin = SERIAL_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = SERIAL_USART_RX_AF;
    HAL_GPIO_Init(SERIAL_USART_RX_GPIO_PORT, &GPIO_InitStruct); 
   
    /* 配置UART3中断优先级 */
    HAL_NVIC_SetPriority(SERIAL_USART_IRQ, 0, 0);	//抢占优先级0，子优先级0
    HAL_NVIC_EnableIRQ(SERIAL_USART_IRQ);		    //使能USART3中断通道  
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  ;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_ENABLE_IT(&UartHandle, USART_IT_ERR);
  huart->Instance->CR1 |= (1U)<<5;
}

/*****************  发送字符 **********************/
void Usart_SendByte(uint8_t str)
{
  HAL_UART_Transmit(&UartHandle, &str, 1, 1000);
}

/*****************  发送字符串 **********************/
void Usart_SendString(uint8_t *str)
{
  unsigned int k=0;
  do 
  {
    HAL_UART_Transmit(&UartHandle,(uint8_t *)(str + k) ,1,1000);
    k++;
  } while(*(str + k)!='\0');
}

///重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
  /* 发送一个字节数据到串口DEBUG_USART */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	
  return (ch);
}

///重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
  int ch;
  HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 1000);	
  return (ch);
}

/*********************************************END OF FILE**********************/