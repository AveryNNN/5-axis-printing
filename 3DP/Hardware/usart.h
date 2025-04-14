#ifndef _USART_H
#define	_USART_H

#include "stm32f4xx.h"
#include <stdio.h>

//引脚定义
/*******************************************************/
// 使用SKR PRO的UART3作为主串口
#define SERIAL_USART                             USART3
#define SERIAL_USART_CLK_ENABLE()                __HAL_RCC_USART3_CLK_ENABLE();

#define RCC_PERIPHCLK_UARTx                      RCC_PERIPHCLK_USART3
#define RCC_UARTxCLKSOURCE_SYSCLK                RCC_USART3CLKSOURCE_SYSCLK

// 根据原理图，SKR PRO的UART3 RX连接到PD9
#define SERIAL_USART_RX_GPIO_PORT                GPIOD
#define SERIAL_USART_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define SERIAL_USART_RX_PIN                      GPIO_PIN_9
#define SERIAL_USART_RX_AF                       GPIO_AF7_USART3

// 根据原理图，SKR PRO的UART3 TX连接到PD8
#define SERIAL_USART_TX_GPIO_PORT                GPIOD
#define SERIAL_USART_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define SERIAL_USART_TX_PIN                      GPIO_PIN_8
#define SERIAL_USART_TX_AF                       GPIO_AF7_USART3

#define SERIAL_USART_IRQHandler                USART3_IRQHandler
#define SERIAL_USART_IRQ                 		 USART3_IRQn
/************************************************************/


void uart_FlushRxBuffer(void);
void Usart_SendByte(uint8_t str);
void Usart_SendString(uint8_t *str);
void SERIAL_USART_Config(unsigned int baud);
extern UART_HandleTypeDef UartHandle;

#endif /* __USART1_H */

