#ifndef _TIMER_H
#define	_TIMER_H

#include "stm32f4xx.h"

#define MOTION_TIM           		 TIM3
#define MOTION_TIM_CLK_ENABLE()  __TIM3_CLK_ENABLE()

#define MOTION_TIM_IRQn					 TIM3_IRQn
#define MOTION_TIM_IRQHandler    TIM3_IRQHandler

#define TIME_TIM           		  TIM4
#define TIME_TIM_CLK_ENABLE()   __TIM4_CLK_ENABLE()

#define TIME_TIM_IRQn						TIM4_IRQn
#define TIME_TIM_IRQHandler    	TIM4_IRQHandler

#define MOTION_TIM_SetAutoreload	__HAL_TIM_SetAutoreload

extern TIM_HandleTypeDef TIM_TimeMotionStructure;
extern TIM_HandleTypeDef TIM_TimeTimeStructure;

void timer_init(void);
unsigned int get_timer_ms(void);
void set_timer_autoload(unsigned short t);




#endif /* __BSP_MOTOR_TIM_H */

