#include "stepper_gpio.h"
#include "gpio_config.h"
#include "stm32f4xx_hal.h"



/*
 ********************************************************************************************
 *函数名：Stepper_GPIO_Config
 *描述  ：配置步进电机的GPIO口引脚
 *调用  ：
 ********************************************************************************************
*/
void stepper_gpio_init(void)
{
	//GPIO 初始化结构体
	GPIO_InitTypeDef  GPIO_InitStruct;

  /*开启GPIO外部时钟*/
  X_MOTION_STEP_GPIO_CLK_ENABLE();
  X_MOTION_DIR_GPIO_CLK_ENABLE();
  X_MOTION_ENA_GPIO_CLK_ENABLE();
	
  Y_MOTION_STEP_GPIO_CLK_ENABLE();
  Y_MOTION_DIR_GPIO_CLK_ENABLE();
  Y_MOTION_ENA_GPIO_CLK_ENABLE();
	
  Z_MOTION_STEP_GPIO_CLK_ENABLE();
  Z_MOTION_DIR_GPIO_CLK_ENABLE();
  Z_MOTION_ENA_GPIO_CLK_ENABLE();
	
  A_MOTION_STEP_GPIO_CLK_ENABLE();
  A_MOTION_DIR_GPIO_CLK_ENABLE();
  A_MOTION_ENA_GPIO_CLK_ENABLE();
	
  B_MOTION_STEP_GPIO_CLK_ENABLE();
  B_MOTION_DIR_GPIO_CLK_ENABLE();
  B_MOTION_ENA_GPIO_CLK_ENABLE();
	
  E_MOTION_STEP_GPIO_CLK_ENABLE();
  E_MOTION_DIR_GPIO_CLK_ENABLE();
  E_MOTION_ENA_GPIO_CLK_ENABLE();
	
	PERSS_ENA_GPIO_CLK_ENABLE();
	
	X_MIN_LIMIT_GPIO_CLK_ENABLE();
	X_ZERO_LIMIT_GPIO_CLK_ENABLE();
	X_MAX_LIMIT_GPIO_CLK_ENABLE();
	
	Y_MIN_LIMIT_GPIO_CLK_ENABLE();
	Y_ZERO_LIMIT_GPIO_CLK_ENABLE();
	Y_MAX_LIMIT_GPIO_CLK_ENABLE();
	
	Z_MIN_LIMIT_GPIO_CLK_ENABLE();
	Z_ZERO_LIMIT_GPIO_CLK_ENABLE();
	Z_MAX_LIMIT_GPIO_CLK_ENABLE();
	
	// 运动部分
	// X	
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;  
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; 								   
  GPIO_InitStruct.Pin = X_MOTION_STEP_PIN;	
	HAL_GPIO_Init(X_MOTION_STEP_PORT, &GPIO_InitStruct);	
			   
  GPIO_InitStruct.Pin = X_MOTION_DIR_PIN;	
	HAL_GPIO_Init(X_MOTION_DIR_PORT, &GPIO_InitStruct);
			   
  GPIO_InitStruct.Pin = X_MOTION_ENA_PIN;	
	HAL_GPIO_Init(X_MOTION_ENA_PORT, &GPIO_InitStruct);
	
	// Y 								   
  GPIO_InitStruct.Pin = Y_MOTION_STEP_PIN;	
	HAL_GPIO_Init(Y_MOTION_STEP_PORT, &GPIO_InitStruct);	
			   
  GPIO_InitStruct.Pin = Y_MOTION_DIR_PIN;	
	HAL_GPIO_Init(Y_MOTION_DIR_PORT, &GPIO_InitStruct);
			   
  GPIO_InitStruct.Pin = Y_MOTION_ENA_PIN;	
	HAL_GPIO_Init(Y_MOTION_ENA_PORT, &GPIO_InitStruct);
	
	// Z 								   
  GPIO_InitStruct.Pin = Z_MOTION_STEP_PIN;	
	HAL_GPIO_Init(Z_MOTION_STEP_PORT, &GPIO_InitStruct);	
			   
  GPIO_InitStruct.Pin = Z_MOTION_DIR_PIN;	
	HAL_GPIO_Init(Z_MOTION_DIR_PORT, &GPIO_InitStruct);
			   
  GPIO_InitStruct.Pin = Z_MOTION_ENA_PIN;	
	HAL_GPIO_Init(Z_MOTION_ENA_PORT, &GPIO_InitStruct);
	
	// A 								   
  GPIO_InitStruct.Pin = A_MOTION_STEP_PIN;	
	HAL_GPIO_Init(A_MOTION_STEP_PORT, &GPIO_InitStruct);	
			   
  GPIO_InitStruct.Pin = A_MOTION_DIR_PIN;	
	HAL_GPIO_Init(A_MOTION_DIR_PORT, &GPIO_InitStruct);
			   
  GPIO_InitStruct.Pin = A_MOTION_ENA_PIN;	
	HAL_GPIO_Init(A_MOTION_ENA_PORT, &GPIO_InitStruct);
	
	// B 								   
  GPIO_InitStruct.Pin = B_MOTION_STEP_PIN;	
	HAL_GPIO_Init(B_MOTION_STEP_PORT, &GPIO_InitStruct);	
			   
  GPIO_InitStruct.Pin = B_MOTION_DIR_PIN;	
	HAL_GPIO_Init(B_MOTION_DIR_PORT, &GPIO_InitStruct);
			   
  GPIO_InitStruct.Pin = B_MOTION_ENA_PIN;	
	HAL_GPIO_Init(B_MOTION_ENA_PORT, &GPIO_InitStruct);
	
	// E 								   
  GPIO_InitStruct.Pin = E_MOTION_STEP_PIN;	
	HAL_GPIO_Init(E_MOTION_STEP_PORT, &GPIO_InitStruct);	
			   
  GPIO_InitStruct.Pin = E_MOTION_DIR_PIN;	
	HAL_GPIO_Init(E_MOTION_DIR_PORT, &GPIO_InitStruct);
			   
  GPIO_InitStruct.Pin = E_MOTION_ENA_PIN;	
	HAL_GPIO_Init(E_MOTION_ENA_PORT, &GPIO_InitStruct);
	
	// 气压
	GPIO_InitStruct.Pin = PERSS_ENA_PIN;	
	HAL_GPIO_Init(PERSS_ENA_PORT, &GPIO_InitStruct);
	
	// 限位部分
	// X
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 
  GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin  = X_MIN_LIMIT_PIN; 
  HAL_GPIO_Init(X_MIN_LIMIT_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin  = X_ZERO_LIMIT_PIN; 
  HAL_GPIO_Init(X_ZERO_LIMIT_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin  = X_MAX_LIMIT_PIN; 
  HAL_GPIO_Init(X_MAX_LIMIT_PORT, &GPIO_InitStruct);
	
	// Y
	GPIO_InitStruct.Pin  = Y_MIN_LIMIT_PIN; 
  HAL_GPIO_Init(Y_MIN_LIMIT_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin  = Y_ZERO_LIMIT_PIN; 
  HAL_GPIO_Init(Y_ZERO_LIMIT_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin  = Y_MAX_LIMIT_PIN; 
  HAL_GPIO_Init(Y_MAX_LIMIT_PORT, &GPIO_InitStruct);
	
	// Z
	GPIO_InitStruct.Pin  = Z_MIN_LIMIT_PIN; 
  HAL_GPIO_Init(Z_MIN_LIMIT_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin  = Z_ZERO_LIMIT_PIN; 
  HAL_GPIO_Init(Z_ZERO_LIMIT_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin  = Z_MAX_LIMIT_PIN; 
  HAL_GPIO_Init(Z_MAX_LIMIT_PORT, &GPIO_InitStruct);
}



