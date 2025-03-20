#include "stepper_gpio.h"
#include "gpio_config.h"
#include "stm32f4xx_hal.h"

/*
 ********************************************************************************************
 *函数名：stepper_gpio_init
 *描述  ：配置步进电机的GPIO口引脚
 *调用  ：
 ********************************************************************************************
*/
void stepper_gpio_init(void)
{
	//GPIO 初始化结构体
	GPIO_InitTypeDef  GPIO_InitStruct;

  /*开启GPIO外部时钟*/
  // X轴电机相关引脚时钟
  X_MOTION_STEP_GPIO_CLK_ENABLE();
  X_MOTION_DIR_GPIO_CLK_ENABLE();
  X_MOTION_ENA_GPIO_CLK_ENABLE();
	
  // Y轴电机相关引脚时钟
  Y_MOTION_STEP_GPIO_CLK_ENABLE();
  Y_MOTION_DIR_GPIO_CLK_ENABLE();
  Y_MOTION_ENA_GPIO_CLK_ENABLE();
	
  // Z轴电机相关引脚时钟
  Z_MOTION_STEP_GPIO_CLK_ENABLE();
  Z_MOTION_DIR_GPIO_CLK_ENABLE();
  Z_MOTION_ENA_GPIO_CLK_ENABLE();
	
  // A电机相关引脚时钟 (E0)
  A_MOTION_STEP_GPIO_CLK_ENABLE();
  A_MOTION_DIR_GPIO_CLK_ENABLE();
  A_MOTION_ENA_GPIO_CLK_ENABLE();
	
  // B电机相关引脚时钟 (E1)
  B_MOTION_STEP_GPIO_CLK_ENABLE();
  B_MOTION_DIR_GPIO_CLK_ENABLE();
  B_MOTION_ENA_GPIO_CLK_ENABLE();
	
  // E电机相关引脚时钟 (E2)
  E_MOTION_STEP_GPIO_CLK_ENABLE();
  E_MOTION_DIR_GPIO_CLK_ENABLE();
  E_MOTION_ENA_GPIO_CLK_ENABLE();
	
  // 气压模块引脚时钟
  PERSS_ENA_GPIO_CLK_ENABLE();
	
  // X轴限位开关引脚时钟
  X_MIN_LIMIT_GPIO_CLK_ENABLE();
  X_ZERO_LIMIT_GPIO_CLK_ENABLE();
  X_MAX_LIMIT_GPIO_CLK_ENABLE();
	
  // Y轴限位开关引脚时钟
  Y_MIN_LIMIT_GPIO_CLK_ENABLE();
  Y_ZERO_LIMIT_GPIO_CLK_ENABLE();
  Y_MAX_LIMIT_GPIO_CLK_ENABLE();
	
  // Z轴限位开关引脚时钟
  Z_MIN_LIMIT_GPIO_CLK_ENABLE();
  Z_ZERO_LIMIT_GPIO_CLK_ENABLE();
  Z_MAX_LIMIT_GPIO_CLK_ENABLE();
	
  // 运动部分初始化
  // 所有输出引脚配置为推挽输出、上拉、高速
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;  
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; 
  
  // X轴电机引脚配置
  GPIO_InitStruct.Pin = X_MOTION_STEP_PIN;	
  HAL_GPIO_Init(X_MOTION_STEP_PORT, &GPIO_InitStruct);	
  
  GPIO_InitStruct.Pin = X_MOTION_DIR_PIN;	
  HAL_GPIO_Init(X_MOTION_DIR_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = X_MOTION_ENA_PIN;	
  HAL_GPIO_Init(X_MOTION_ENA_PORT, &GPIO_InitStruct);
	
  // Y轴电机引脚配置
  GPIO_InitStruct.Pin = Y_MOTION_STEP_PIN;	
  HAL_GPIO_Init(Y_MOTION_STEP_PORT, &GPIO_InitStruct);	
  
  GPIO_InitStruct.Pin = Y_MOTION_DIR_PIN;	
  HAL_GPIO_Init(Y_MOTION_DIR_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = Y_MOTION_ENA_PIN;	
  HAL_GPIO_Init(Y_MOTION_ENA_PORT, &GPIO_InitStruct);
	
  // Z轴电机引脚配置
  GPIO_InitStruct.Pin = Z_MOTION_STEP_PIN;	
  HAL_GPIO_Init(Z_MOTION_STEP_PORT, &GPIO_InitStruct);	
  
  GPIO_InitStruct.Pin = Z_MOTION_DIR_PIN;	
  HAL_GPIO_Init(Z_MOTION_DIR_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = Z_MOTION_ENA_PIN;	
  HAL_GPIO_Init(Z_MOTION_ENA_PORT, &GPIO_InitStruct);
	
  // A电机引脚配置 (E0)
  GPIO_InitStruct.Pin = A_MOTION_STEP_PIN;	
  HAL_GPIO_Init(A_MOTION_STEP_PORT, &GPIO_InitStruct);	
  
  GPIO_InitStruct.Pin = A_MOTION_DIR_PIN;	
  HAL_GPIO_Init(A_MOTION_DIR_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = A_MOTION_ENA_PIN;	
  HAL_GPIO_Init(A_MOTION_ENA_PORT, &GPIO_InitStruct);
	
  // B电机引脚配置 (E1)
  GPIO_InitStruct.Pin = B_MOTION_STEP_PIN;	
  HAL_GPIO_Init(B_MOTION_STEP_PORT, &GPIO_InitStruct);	
  
  GPIO_InitStruct.Pin = B_MOTION_DIR_PIN;	
  HAL_GPIO_Init(B_MOTION_DIR_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = B_MOTION_ENA_PIN;	
  HAL_GPIO_Init(B_MOTION_ENA_PORT, &GPIO_InitStruct);
	
  // E电机引脚配置 (E2)
  GPIO_InitStruct.Pin = E_MOTION_STEP_PIN;	
  HAL_GPIO_Init(E_MOTION_STEP_PORT, &GPIO_InitStruct);	
  
  GPIO_InitStruct.Pin = E_MOTION_DIR_PIN;	
  HAL_GPIO_Init(E_MOTION_DIR_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = E_MOTION_ENA_PIN;	
  HAL_GPIO_Init(E_MOTION_ENA_PORT, &GPIO_InitStruct);
	
  // 气压模块引脚配置
  GPIO_InitStruct.Pin = PERSS_ENA_PIN;	
  HAL_GPIO_Init(PERSS_ENA_PORT, &GPIO_InitStruct);
	
  // 限位开关引脚配置 - 所有输入引脚配置为上拉输入
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  
  // X轴限位开关引脚配置
  GPIO_InitStruct.Pin = X_MIN_LIMIT_PIN; 
  HAL_GPIO_Init(X_MIN_LIMIT_PORT, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = X_ZERO_LIMIT_PIN; 
  HAL_GPIO_Init(X_ZERO_LIMIT_PORT, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = X_MAX_LIMIT_PIN; 
  HAL_GPIO_Init(X_MAX_LIMIT_PORT, &GPIO_InitStruct);
	
  // Y轴限位开关引脚配置
  GPIO_InitStruct.Pin = Y_MIN_LIMIT_PIN; 
  HAL_GPIO_Init(Y_MIN_LIMIT_PORT, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = Y_ZERO_LIMIT_PIN; 
  HAL_GPIO_Init(Y_ZERO_LIMIT_PORT, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = Y_MAX_LIMIT_PIN; 
  HAL_GPIO_Init(Y_MAX_LIMIT_PORT, &GPIO_InitStruct);
	
  // Z轴限位开关引脚配置
  GPIO_InitStruct.Pin = Z_MIN_LIMIT_PIN; 
  HAL_GPIO_Init(Z_MIN_LIMIT_PORT, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = Z_ZERO_LIMIT_PIN; 
  HAL_GPIO_Init(Z_ZERO_LIMIT_PORT, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = Z_MAX_LIMIT_PIN; 
  HAL_GPIO_Init(Z_MAX_LIMIT_PORT, &GPIO_InitStruct);
  
  // 初始化状态 - 禁用所有电机
  X_ENABLE_PIN = 1;  // 高电平禁用
  Y_ENABLE_PIN = 1;
  Z_ENABLE_PIN = 1;
  A_ENABLE_PIN = 1;
  B_ENABLE_PIN = 1;
  E0_ENABLE_PIN = 1;
}