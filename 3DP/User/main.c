/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   直流无刷电机-串口控制
  ******************************************************************************
  * @attention
  *
  * 实验平台 STM32 F407 开发板 

  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"
#include "led.h"
#include "usart.h"
#include "planner.h"
#include "gpio_config.h"
#include "leveling.h"

int pulse_num=0;

	
void Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}	
	
extern float angle_a,angle_b;



void test_motor(void)
{
  printf("测试B轴电机开始\r\n");
  
  // 使能X轴
  B_ENABLE_PIN = STEPPER_ENABLE_VOL;
  printf("B电机使能完成\r\n");
  
  // 设置正方向
  B_DIR_PIN = POSITIVE_B_DIR;
  printf("B方向设置完成\r\n");
  
  // 生成1000个脉冲
  for(int i=0; i<1000; i++)
  {
    B_STEP_PIN = 1;
    soft_delay_100us();  // 使用较长延迟确保脉冲足够宽
    B_STEP_PIN = 0;
    soft_delay_100us();
    
    if(i % 100 == 0)
      printf("已生成%d个脉冲\r\n", i);
  }
  
  printf("测试B轴电机结束\r\n");
	
	printf("测试A轴电机开始\r\n");
  
  // 使能X轴
  A_ENABLE_PIN = STEPPER_ENABLE_VOL;
  printf("A电机使能完成\r\n");
  
  // 设置正方向
  A_DIR_PIN = POSITIVE_B_DIR;
  printf("A方向设置完成\r\n");
  
  // 生成1000个脉冲
  for(int i=0; i<1000; i++)
  {
    A_STEP_PIN = 1;
    soft_delay_100us();  // 使用较长延迟确保脉冲足够宽
    A_STEP_PIN = 0;
    soft_delay_100us();
    
    if(i % 100 == 0)
      printf("已生成%d个脉冲\r\n", i);
  }
  
  printf("测试B轴电机结束\r\n");
}


/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void) 
{
  
	/* 初始化系统时钟为168MHz */
  SystemClock_Config();
  
  /* LED 灯初始化 */
  LED_GPIO_Config();
  
  /* 初始化BSP模块 */
  bsp_init();
	DWT_Init();
	
	
  SERIAL_USART_Config(115200);
	
  /* 打印欢迎信息 */
  printf("start\r\n");
  SERIAL_ECHO_START;
  printf(MACHINE_NAME);
	printf("\r\n");
  printf(FIRMWARE_URL);
  printf("\r\n");
  
  /* 规划器初始化 */
  plan_init();
	

  
  /* 运行串口通信测试 */
  //UART_Test();      // 基本串口通信测试
  
  /* 主循环 */
  while(1)
  {
   bsp_command_process();
	//test_motor();
	
  }
}







/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
	
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1) {};
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    while(1) {};
  }
}
	
// void SystemClock_Config(void)
//{
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;
//  RCC_OscInitTypeDef RCC_OscInitStruct;

//  /* Enable Power Control clock */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  
//  /* The voltage scaling allows optimizing the power consumption when the device is 
//     clocked below the maximum system frequency, to update the voltage scaling value 
//     regarding system frequency refer to product datasheet.  */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//  
//  /* Enable HSE Oscillator and activate PLL with HSE as source */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 8;
//  RCC_OscInitStruct.PLL.PLLN = 336;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 4;
//  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    while(1) {};
//  }
//  
//  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
//     clocks dividers */
//  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
//  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//  {
//    while(1) {};
//  }

//  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
//  if (HAL_GetREVID() == 0x1001)
//  {
//    /* Enable the Flash prefetch */
//    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
//  }
//	__HAL_RCC_GPIOH_CLK_ENABLE();
//}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
