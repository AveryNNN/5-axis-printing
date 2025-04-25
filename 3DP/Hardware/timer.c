/**
  ******************************************************************************
  * @file    bsp_motor_tim.c
  * @author  long
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   电机相关定时器配置
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "timer.h"
#include "marlin.h"
#include "stm32f4xx_hal.h"



static unsigned int time=0;

TIM_HandleTypeDef TIM_TimeMotionStructure;
TIM_HandleTypeDef TIM_TimeTimeStructure;
 /**
  * @brief  基本定时器 TIMx,x[6,7]中断优先级配置
  * @param  无
  * @retval 无
  */
static void TIMx_NVIC_Configuration(void)
{
	//设置抢占优先级，子优先级
	HAL_NVIC_SetPriority(MOTION_TIM_IRQn, 1, 3);
	HAL_NVIC_EnableIRQ(MOTION_TIM_IRQn);
	//设置抢占优先级，子优先级
	HAL_NVIC_SetPriority(TIME_TIM_IRQn, 2, 3);
	HAL_NVIC_EnableIRQ(TIME_TIM_IRQn);
}

/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{
	// 开启TIMx_CLK,x[6,7] 
	MOTION_TIM_CLK_ENABLE(); 
	TIME_TIM_CLK_ENABLE();

	TIM_TimeMotionStructure.Instance = MOTION_TIM;//0.03s进一次中断
	/* 累计 TIM_Period个后产生一个更新或者中断*/		
	//当定时器从0计数到4999，即为5000次，为一个定时周期
	TIM_TimeMotionStructure.Init.Period = 65536-1;       
	//定时器时钟源TIMxCLK = 2 * PCLK1  
	//				PCLK1 = HCLK / 4 
	//				=> TIMxCLK=HCLK/2=SystemCoreClock/2=84MHz
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10000Hz
	TIM_TimeMotionStructure.Init.Prescaler = 42-1;	
	HAL_TIM_Base_Init(&TIM_TimeMotionStructure);
	HAL_TIM_Base_Start_IT(&TIM_TimeMotionStructure);	
	
	TIM_TimeTimeStructure.Instance = TIME_TIM;
	/* 累计 TIM_Period个后产生一个更新或者中断*/		
	//当定时器从0计数到4999，即为5000次，为一个定时周期
	TIM_TimeTimeStructure.Init.Period = 1000-1;       
	//定时器时钟源TIMxCLK = 2 * PCLK1  
	//				PCLK1 = HCLK / 4 
	//				=> TIMxCLK=HCLK/2=SystemCoreClock/2=84MHz
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10000Hz
	TIM_TimeTimeStructure.Init.Prescaler = 84-1;	
	HAL_TIM_Base_Init(&TIM_TimeTimeStructure);
	HAL_TIM_Base_Start_IT(&TIM_TimeTimeStructure);
}

/**
  * @brief  初始化基本定时器定时，1ms产生一次中断
  * @param  无
  * @retval 无
  */
void timer_init(void)
{
	TIMx_NVIC_Configuration();	
  
	TIM_Mode_Config();
}

void  MOTION_TIM_IRQHandler (void)
{
	HAL_TIM_IRQHandler(&TIM_TimeMotionStructure);	 	
}

void  TIME_TIM_IRQHandler (void)
{
	HAL_TIM_IRQHandler(&TIM_TimeTimeStructure);	 	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&TIM_TimeMotionStructure))
    {
			bsp_stepper_timer_isr();
    }
		else if(htim==(&TIM_TimeTimeStructure))//1ms
		{
			time++;
		}
}

unsigned int get_timer_ms(void)
{
	return time;
}

void set_timer_autoload(unsigned short t)
{
	MOTION_TIM_SetAutoreload(&TIM_TimeMotionStructure,t);
}

/*********************************************END OF FILE**********************/
