#ifndef _USART_H
#define	_USART_H

#include "stm32f4xx.h"
#include <stdio.h>

//串口接收缓冲数组大小

//引脚定义
/*******************************************************/
#define SERIAL_USART                             USART1
#define SERIAL_USART_CLK_ENABLE()                __USART1_CLK_ENABLE();

#define RCC_PERIPHCLK_UARTx                      RCC_PERIPHCLK_USART1
#define RCC_UARTxCLKSOURCE_SYSCLK                RCC_USART1CLKSOURCE_SYSCLK

#define SERIAL_USART_RX_GPIO_PORT                GPIOB
#define SERIAL_USART_RX_GPIO_CLK_ENABLE()        __GPIOB_CLK_ENABLE()
#define SERIAL_USART_RX_PIN                      GPIO_PIN_7
#define SERIAL_USART_RX_AF                       GPIO_AF7_USART1

#define SERIAL_USART_TX_GPIO_PORT                GPIOB
#define SERIAL_USART_TX_GPIO_CLK_ENABLE()        __GPIOB_CLK_ENABLE()
#define SERIAL_USART_TX_PIN                      GPIO_PIN_6
#define SERIAL_USART_TX_AF                       GPIO_AF7_USART1

#define SERIAL_USART_IRQHandler                  USART1_IRQHandler
#define SERIAL_USART_IRQ                 		     USART1_IRQn
/************************************************************/


void uart_FlushRxBuffer(void);
void Usart_SendByte(uint8_t str);
void Usart_SendString(uint8_t *str);
void SERIAL_USART_Config(unsigned int baud);
extern UART_HandleTypeDef UartHandle;
#endif /* __USART1_H */
