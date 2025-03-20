#ifndef _TIMER_H
#define	_TIMER_H

#include "stm32f4xx.h"

// 使用SKR PRO上的定时器
// TIM3用于电机控制
#define MOTION_TIM           		 TIM3
#define MOTION_TIM_CLK_ENABLE()  __HAL_RCC_TIM3_CLK_ENABLE()

#define MOTION_TIM_IRQn					 TIM3_IRQn
#define MOTION_TIM_IRQHandler    TIM3_IRQHandler

// TIM4用于时间计时
#define TIME_TIM           		  TIM4
#define TIME_TIM_CLK_ENABLE()   __HAL_RCC_TIM4_CLK_ENABLE()

#define TIME_TIM_IRQn						TIM4_IRQn
#define TIME_TIM_IRQHandler    	TIM4_IRQHandler

#define MOTION_TIM_SetAutoreload	__HAL_TIM_SET_AUTORELOAD

extern TIM_HandleTypeDef TIM_TimeMotionStructure;
extern TIM_HandleTypeDef TIM_TimeTimeStructure;

void timer_init(void);
unsigned int get_timer_ms(void);
void set_timer_autoload(unsigned short t);

#endif /* _TIMER_H */
