/**
  ******************************************************************************
  * @file    simple_uart.c
  * @author  Debug Assistant
  * @version V1.0
  * @date    2025-03-20
  * @brief   简易串口通信程序 - 用于基本的串口收发测试
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "usart.h"
#include "gpio_config.h"
#include "led.h"
#include "delay.h"

/* 定义接收缓冲区 */
#define MAX_BUFFER_LENGTH 128
static uint8_t rx_buffer[MAX_BUFFER_LENGTH];
static volatile uint16_t rx_index = 0;
static volatile uint8_t rx_complete = 0;

/**
  * @brief  系统初始化
  * @param  无
  * @retval 无
  */
void system_init(void)
{
    /* 配置系统时钟 */
    SystemClock_Config();
    
    /* 初始化LED */
    LED_GPIO_Config();
    
    /* 初始化延时功能 */
    HAL_InitTick(0);
    
    /* 初始化串口 */
    SERIAL_USART_Config(115200);
    
    /* 使能串口接收中断 */
    __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
    
    /* 初始化LED状态 */
    LED1 = 0;
    LED2 = 0;
    LED3 = 0;
    LED4 = 0;
}

/**
  * @brief  串口接收字符处理
  * @param  ch: 接收到的字符
  * @retval 无
  */
void process_received_char(uint8_t ch)
{
    /* 存储接收到的字符 */
    if(rx_index < MAX_BUFFER_LENGTH - 1)
    {
        rx_buffer[rx_index++] = ch;
        
        /* 检查是否接收到命令结束符 */
        if(ch == '\n' || ch == '\r')
        {
            rx_buffer[rx_index] = '\0';  /* 字符串结束符 */
            rx_complete = 1;
        }
    }
    else
    {
        /* 缓冲区已满，重置 */
        rx_index = 0;
    }
}

/**
  * @brief  发送字符串
  * @param  str: 要发送的字符串
  * @retval 无
  */
void uart_send_string(const char* str)
{
    Usart_SendString((uint8_t*)str);
}

/**
  * @brief  处理接收到的命令
  * @param  无
  * @retval 无
  */
void process_command(void)
{
    /* 检查是否接收到完整命令 */
    if(rx_complete)
    {
        /* 命令处理示例 - 闪烁LED */
        if(strstr((char*)rx_buffer, "LED1") != NULL)
        {
            LED1 = !LED1;
            uart_send_string("LED1切换状态\r\n");
        }
        else if(strstr((char*)rx_buffer, "LED2") != NULL)
        {
            LED2 = !LED2;
            uart_send_string("LED2切换状态\r\n");
        }
        else if(strstr((char*)rx_buffer, "LED3") != NULL)
        {
            LED3 = !LED3;
            uart_send_string("LED3切换状态\r\n");
        }
        else if(strstr((char*)rx_buffer, "LED4") != NULL)
        {
            LED4 = !LED4;
            uart_send_string("LED4切换状态\r\n");
        }
        /* 命令回显 */
        else
        {
            uart_send_string("收到命令: ");
            uart_send_string((char*)rx_buffer);
            uart_send_string("\r\n");
        }
        
        /* 重置接收状态 */
        rx_index = 0;
        rx_complete = 0;
    }
}

/**
  * @brief  发送"Hello World"测试
  * @param  无
  * @retval 无
  */
void send_hello_world(void)
{
    uart_send_string("\r\n\r\n");
    uart_send_string("*****************************************\r\n");
    uart_send_string("*      STM32F4 串口通信测试程序        *\r\n");
    uart_send_string("*****************************************\r\n");
    uart_send_string("Hello World! 串口通信测试开始\r\n");
    uart_send_string("请发送命令测试通信\r\n");
    uart_send_string("支持的命令: LED1, LED2, LED3, LED4\r\n\r\n");
}

/**
  * @brief  测试发送数字
  * @param  无
  * @retval 无
  */
void test_number_send(void)
{
    char buf[32];
    
    uart_send_string("数字发送测试:\r\n");
    
    /* 测试整数发送 */
    for(int i = 0; i < 10; i++)
    {
        sprintf(buf, "整数 %d: %d\r\n", i, i*100);
        uart_send_string(buf);
        HAL_Delay(200);
    }
    
    /* 测试浮点数发送 */
    for(float f = 0.0f; f < 1.0f; f += 0.1f)
    {
        sprintf(buf, "浮点数 %.1f: %.3f\r\n", f, f*10.0f);
        uart_send_string(buf);
        HAL_Delay(200);
    }
    
    uart_send_string("数字发送测试完成\r\n\r\n");
}

/**
  * @brief  串口中断处理函数 
  * @param  无
  * @retval 无
  */
void USART_IRQHandler(void)
{
    uint8_t ch;
    
    if(__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET)
    {
        ch = (uint8_t)(UartHandle.Instance->DR & 0xFF);
        
        /* 处理接收到的字符 */
        process_received_char(ch);
        
        /* 回显字符 */
        Usart_SendByte(ch);
        
        /* 清除标志位 */
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_RXNE);
    }
    
    /* 处理过载错误 */
    if(__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_ORE);
    }
}

/**
  * @brief  主函数
  * @param  无
  * @retval int
  */
int main(void)
{
    uint32_t led_time = 0;
    
    /* 系统初始化 */
    system_init();
    
    /* 发送欢迎信息 */
    send_hello_world();
    
    /* 测试数字发送 */
    test_number_send();
    
    /* 主循环 */
    while(1)
    {
        /* 处理命令 */
        process_command();
        
        /* LED闪烁，表示程序运行中 */
        if(HAL_GetTick() - led_time >= 500)
        {
            LED1 = !LED1;
            led_time = HAL_GetTick();
        }
    }
}

/**
  * @brief  中断处理函数重定向
  * @note   这段代码确保中断能够正确路由到我们的处理函数
  * @param  无
  * @retval 无
  */
void SERIAL_USART_IRQHandler(void)
{
    USART_IRQHandler();
}