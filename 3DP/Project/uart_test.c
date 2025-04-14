/**
  ******************************************************************************
  * @file    uart_test.c
  * @author  Debug Assistant
  * @version V1.0
  * @date    2025-03-20
  * @brief   串口通信测试程序 - 用于调试3D打印机固件的串口通信
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "usart.h"
#include "gpio_config.h"
#include "led.h"
#include "connect_bsp.h"
#include "delay.h"

/* 定义接收缓冲区大小 */
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256

/* 接收和发送缓冲区 */
uint8_t rx_buffer[RX_BUFFER_SIZE]; 
uint8_t tx_buffer[TX_BUFFER_SIZE];

/* 接收缓冲区索引 */
volatile uint16_t rx_index = 0;

/* 命令完成标志 */
volatile uint8_t command_received = 0;

/**
  * @brief  初始化串口通信测试
  * @param  baud: 波特率
  * @retval 无
  */
void uart_test_init(uint32_t baud)
{
    /* 初始化LED指示灯 */
    LED_GPIO_Config();
    
    /* 初始化串口 */
    SERIAL_USART_Config(baud);
    
    /* 清空接收缓冲区 */
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    rx_index = 0;
    
    /* 初始化延时函数 */
    HAL_InitTick(0);
    
    printf("\r\n\r\n");
    printf("=================================================\r\n");
    printf("        STM32F4 3D打印机固件串口通信测试        \r\n");
    printf("=================================================\r\n");
    printf("波特率: %d\r\n", baud);
    printf("通信已就绪，可以开始发送G代码命令\r\n\r\n");
}

/**
  * @brief  发送一条G代码命令
  * @param  cmd: 要发送的G代码命令
  * @retval 无
  */
void send_gcode(const char* cmd)
{
    Usart_SendString((uint8_t*)cmd);
    Usart_SendByte('\n');
    
    printf("已发送: %s\r\n", cmd);
}

/**
  * @brief  处理收到的字符
  * @param  ch: 收到的字符
  * @retval 无
  */
void process_received_char(uint8_t ch)
{
    /* 将收到的字符加入到接收缓冲区 */
    if (rx_index < RX_BUFFER_SIZE - 1) {
        rx_buffer[rx_index++] = ch;
        
        /* 简单的回显 */
        Usart_SendByte(ch);
        
        /* 检测命令结束符(换行或回车) */
        if (ch == '\n' || ch == '\r') {
            rx_buffer[rx_index] = '\0'; // 字符串结束符
            command_received = 1;
        }
    } else {
        /* 缓冲区溢出，重置 */
        rx_index = 0;
        memset(rx_buffer, 0, RX_BUFFER_SIZE);
    }
}

/**
  * @brief  处理收到的完整命令
  * @param  无
  * @retval 无
  */
void process_command(void)
{
    if (command_received) {
        /* 指示灯切换状态，表示命令接收 */
        LED1 = !LED1;
        
        /* 转发命令给固件的命令处理函数 */
        for (uint16_t i = 0; i < rx_index; i++) {
            bsp_serial_isr(rx_buffer[i]);
        }
        
        /* 打印接收到的命令 */
        printf("\r\n接收到命令: %s\r\n", rx_buffer);
        
        /* 手动调用命令处理流程 */
        bsp_command_process();
        
        /* 重置接收缓冲区 */
        rx_index = 0;
        memset(rx_buffer, 0, RX_BUFFER_SIZE);
        command_received = 0;
    }
}

/**
  * @brief  执行预定义的自动测试序列
  * @param  无
  * @retval 无
  */
void run_auto_test_sequence(void)
{
    printf("\r\n开始执行自动测试序列...\r\n");
    
    /* 测试命令序列 */
    const char* test_commands[] = {
        "M114",                   // 查询当前位置
        "G28",                    // 全部回零
        "G1 X10 Y10 Z10 F1000",   // 移动到指定位置
        "G1 X20 Y20 Z20 F1000",   // 继续移动
        "M114",                   // 再次查询位置
        "G1 X0 Y0 Z0 F1000"       // 回到起始位置
    };
    
    int num_commands = sizeof(test_commands) / sizeof(test_commands[0]);
    
    /* 依次发送测试命令并等待处理完成 */
    for (int i = 0; i < num_commands; i++) {
        printf("\r\n执行命令 %d/%d: %s\r\n", i+1, num_commands, test_commands[i]);
        send_gcode(test_commands[i]);
        
        /* 等待命令处理完成 (简单延时，实际应用中可以等待响应) */
        HAL_Delay(3000);
        
        /* 切换LED指示灯状态 */
        LED2 = !LED2;
    }
    
    printf("\r\n自动测试序列完成\r\n");
}

/**
  * @brief  测试发送功能
  * @param  无
  * @retval 无
  */
void test_transmit(void)
{
    printf("\r\n测试发送功能...\r\n");
    
    /* 测试单字符发送 */
    for (char c = 'A'; c <= 'Z'; c++) {
        Usart_SendByte(c);
        HAL_Delay(100);
    }
    
    Usart_SendString((uint8_t*)"\r\n单字符发送测试完成\r\n");
    
    /* 测试字符串发送 */
    Usart_SendString((uint8_t*)"这是一个字符串发送测试\r\n");
    
    printf("发送测试完成\r\n");
}

/**
  * @brief  测试接收功能
  * @param  timeout_ms: 超时时间(毫秒)
  * @retval 无
  */
void test_receive(uint32_t timeout_ms)
{
    printf("\r\n测试接收功能，请在 %lu 毫秒内输入内容...\r\n", timeout_ms);
    
    uint32_t start_time = HAL_GetTick();
    
    /* 重置接收缓冲区 */
    rx_index = 0;
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    command_received = 0;
    
    /* 等待接收或超时 */
    while ((HAL_GetTick() - start_time < timeout_ms) && !command_received) {
        /* 这里可以添加从中断处理程序中接收字符的逻辑 */
        /* 简化起见，这里假设字符将通过中断接收并调用process_received_char */
        
        /* 闪烁LED指示等待状态 */
        if ((HAL_GetTick() - start_time) % 200 == 0) {
            LED3 = !LED3;
        }
    }
    
    if (command_received) {
        printf("\r\n接收到内容: %s\r\n", rx_buffer);
    } else {
        printf("\r\n接收超时\r\n");
    }
    
    /* 重置接收状态 */
    rx_index = 0;
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    command_received = 0;
}

/**
  * @brief  测试硬件流控制(如果支持)
  * @param  无
  * @retval 无
  */
void test_hardware_flow_control(void)
{
    printf("\r\n硬件流控测试未实现，该功能需要硬件支持\r\n");
    /* 这部分需要根据实际硬件实现添加相应代码 */
}

/**
  * @brief  测试中断模式
  * @param  无
  * @retval 无
  */
void test_interrupt_mode(void)
{
    printf("\r\n中断模式测试...\r\n");
    printf("请发送任意内容，将通过中断接收\r\n");
    
    /* 启用UART接收中断 */
    __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
    
    /* 等待接收完成 */
    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time < 5000) && !command_received) {
        /* 闪烁LED指示等待状态 */
        if ((HAL_GetTick() - start_time) % 200 == 0) {
            LED4 = !LED4;
        }
    }
    
    /* 禁用UART接收中断 */
    __HAL_UART_DISABLE_IT(&UartHandle, UART_IT_RXNE);
    
    if (command_received) {
        printf("\r\n中断模式成功接收: %s\r\n", rx_buffer);
    } else {
        printf("\r\n中断接收超时\r\n");
    }
    
    /* 重置接收状态 */
    rx_index = 0;
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    command_received = 0;
}

/**
  * @brief  与固件集成测试 - 发送G代码并处理
  * @param  无
  * @retval 无
  */
void integrated_test_with_firmware(void)
{
    printf("\r\n开始与固件集成测试...\r\n");
    
    /* 测试基本G代码命令 */
    const char* test_commands[] = {
        "G28",            // 回零
        "G1 X10 F1000",   // X轴移动
        "G1 Y10 F1000",   // Y轴移动
        "G1 Z5 F500",     // Z轴移动
        "M114"            // 查询位置
    };
    
    int num_commands = sizeof(test_commands) / sizeof(test_commands[0]);
    
    for (int i = 0; i < num_commands; i++) {
        printf("\r\n测试命令 %d/%d: %s\r\n", i+1, num_commands, test_commands[i]);
        
        /* 发送命令 */
        for (const char *p = test_commands[i]; *p; p++) {
            bsp_serial_isr(*p);
        }
        bsp_serial_isr('\n'); // 命令结束符
        
        /* 处理命令 */
        bsp_command_process();
        
        /* 等待命令执行完成 */
        HAL_Delay(2000);
    }
    
    printf("\r\n固件集成测试完成\r\n");
}

/**
  * @brief  串口中断处理函数
  * @param  无
  * @retval 无
  */
void uart_irq_handler(void)
{
    uint8_t ch;
    
    if(__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET) {
        ch = (uint8_t)(UartHandle.Instance->DR & 0xFF);
        process_received_char(ch);
        
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_RXNE);
    }
    
    /* 处理其他可能的错误标志 */
    if(__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_ORE);
        /* 处理过载错误 */
    }
}

/**
  * @brief  主测试函数
  * @param  无
  * @retval 无
  */
void uart_test_main(void)
{
    /* 初始化串口测试 */
    uart_test_init(115200);
    
    /* 测试发送功能 */
    test_transmit();
    HAL_Delay(1000);
    
    /* 测试接收功能 */
    test_receive(5000);
    HAL_Delay(1000);
    
    /* 测试中断模式 */
    test_interrupt_mode();
    HAL_Delay(1000);
    
    /* 执行自动测试序列 */
    run_auto_test_sequence();
    HAL_Delay(1000);
    
    /* 与固件集成测试 */
    integrated_test_with_firmware();
    
    printf("\r\n所有测试完成，开始监听命令...\r\n");
    
    /* 启用中断接收，进入命令监听模式 */
    __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
    
    /* 命令监听循环 */
    while(1) {
        /* 处理接收到的命令 */
        if (command_received) {
            process_command();
        }
        
        /* 运行固件的命令处理流程 */
        bsp_command_process();
        
        /* 状态指示 */
        HAL_Delay(100);
        if ((HAL_GetTick() % 1000) < 500) {
            LED4 = 1;
        } else {
            LED4 = 0;
        }
    }
}

/**
  * @brief  示例主函数
  * @param  无
  * @retval int
  */
int main(void)
{
    /* 系统初始化 */
    HAL_Init();
    
    /* 启动串口测试 */
    uart_test_main();
    
    /* 永不应该到达这里 */
    while(1) {}
}