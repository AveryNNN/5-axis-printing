#ifndef gpio_config_h
#define gpio_config_h

#include "stm32f4xx.h"

#define DEBUG 0

/*
**********************************************************************************************
*                              位带操作,实现51类似的GPIO控制功能
*
**********************************************************************************************
*/
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr (GPIOB_BASE+20) //0x40020414
#define GPIOC_ODR_Addr (GPIOC_BASE+20) //0x40020814
#define GPIOD_ODR_Addr (GPIOD_BASE+20) //0x40020C14
#define GPIOE_ODR_Addr (GPIOE_BASE+20) //0x40021014
#define GPIOF_ODR_Addr (GPIOF_BASE+20) //0x40021414
#define GPIOG_ODR_Addr (GPIOG_BASE+20) //0x40021814
#define GPIOH_ODR_Addr (GPIOH_BASE+20) //0x40021C14
#define GPIOI_ODR_Addr (GPIOI_BASE+20) //0x40022014

#define GPIOA_IDR_Addr (GPIOA_BASE+16) //0x40020010
#define GPIOB_IDR_Addr (GPIOB_BASE+16) //0x40020410
#define GPIOC_IDR_Addr (GPIOC_BASE+16) //0x40020810
#define GPIOD_IDR_Addr (GPIOD_BASE+16) //0x40020C10
#define GPIOE_IDR_Addr (GPIOE_BASE+16) //0x40021010
#define GPIOF_IDR_Addr (GPIOF_BASE+16) //0x40021410
#define GPIOG_IDR_Addr (GPIOG_BASE+16) //0x40021810
#define GPIOH_IDR_Addr (GPIOH_BASE+16) //0x40021C10
#define GPIOI_IDR_Addr (GPIOI_BASE+16) //0x40022010
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n) 	 BIT_ADDR(GPIOH_ODR_Addr,n) //输出 
#define PHin(n) 	 BIT_ADDR(GPIOH_IDR_Addr,n) //输入

#define PIout(n) 	 BIT_ADDR(GPIOI_ODR_Addr,n) //输出 
#define PIin(n) 	 BIT_ADDR(GPIOI_IDR_Addr,n) //输入

/*
**********************************************************************************************
*                                         IO接口定义
*
**********************************************************************************************
*/

/*
**********************************************************************************************
*                                         LED接口定义
*
**********************************************************************************************
*/
//LED1 - 使用PA8作为LED引脚，可以根据SKR PRO板实际情况调整
#define LED1_PIN                  GPIO_PIN_8                
#define LED1_GPIO_PORT            GPIOA                     
#define LED1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()

//LED2 - 使用PG4作为LED引脚，对应EXP1_01_PIN，可根据实际情况调整
#define LED2_PIN                  GPIO_PIN_4                 
#define LED2_GPIO_PORT            GPIOG                      
#define LED2_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()

//LED3 - 使用PD12作为LED引脚，对应HEATER_BED_PIN，可根据实际情况调整
#define LED3_PIN                  GPIO_PIN_12
#define LED3_GPIO_PORT            GPIOD                      
#define LED3_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOD_CLK_ENABLE()

//LED4 - 使用PB1作为LED引脚，对应HEATER_0_PIN，可根据实际情况调整
#define LED4_PIN                  GPIO_PIN_1
#define LED4_GPIO_PORT            GPIOB                      
#define LED4_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()

#define LED1			PAout(8)
#define LED2			PGout(4)
#define LED3			PDout(12)
#define LED4			PBout(1)

/*
**********************************************************************************************
*                                         步进电机接口定义
*
**********************************************************************************************
*/
//
// Limit Switches - 使用SKR PRO的限位开关引脚
// X
#define X_MIN_LIMIT_PORT                GPIOB
#define X_MIN_LIMIT_PIN                 GPIO_PIN_10  // X_MIN_PIN from pins_BTT_SKR_PRO_common.h
#define X_MIN_LIMIT_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

#define X_ZERO_LIMIT_PORT               GPIOE
#define X_ZERO_LIMIT_PIN                GPIO_PIN_15  // X_MAX_PIN from pins_BTT_SKR_PRO_common.h
#define X_ZERO_LIMIT_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()

#define X_MAX_LIMIT_PORT                GPIOE
#define X_MAX_LIMIT_PIN                 GPIO_PIN_15  // 使用相同引脚作为最大限位，可根据需要更改
#define X_MAX_LIMIT_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()

// Y
#define Y_MIN_LIMIT_PORT                GPIOE
#define Y_MIN_LIMIT_PIN                 GPIO_PIN_12  // Y_MIN_PIN from pins_BTT_SKR_PRO_common.h
#define Y_MIN_LIMIT_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()

#define Y_ZERO_LIMIT_PORT               GPIOE
#define Y_ZERO_LIMIT_PIN                GPIO_PIN_10  // Y_MAX_PIN from pins_BTT_SKR_PRO_common.h
#define Y_ZERO_LIMIT_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()

#define Y_MAX_LIMIT_PORT                GPIOE
#define Y_MAX_LIMIT_PIN                 GPIO_PIN_10  // 使用相同引脚作为最大限位，可根据需要更改
#define Y_MAX_LIMIT_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()

// Z
#define Z_MIN_LIMIT_PORT                GPIOG
#define Z_MIN_LIMIT_PIN                 GPIO_PIN_8   // Z_MIN_PIN from pins_BTT_SKR_PRO_common.h
#define Z_MIN_LIMIT_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOG_CLK_ENABLE()

#define Z_ZERO_LIMIT_PORT               GPIOG
#define Z_ZERO_LIMIT_PIN                GPIO_PIN_5   // Z_MAX_PIN from pins_BTT_SKR_PRO_common.h
#define Z_ZERO_LIMIT_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()

#define Z_MAX_LIMIT_PORT                GPIOG
#define Z_MAX_LIMIT_PIN                 GPIO_PIN_5   // 使用相同引脚作为最大限位，可根据需要更改
#define Z_MAX_LIMIT_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOG_CLK_ENABLE()

#define X_MIN_PIN                       PBin(10)
#define X_ZERO_PIN                      PEin(15)
#define X_MAX_PIN                       PEin(15)

#define Y_MIN_PIN                       PEin(12)
#define Y_ZERO_PIN                      PEin(10)
#define Y_MAX_PIN                       PEin(10)

#define Z_MIN_PIN                       PGin(8)
#define Z_ZERO_PIN                      PGin(5)
#define Z_MAX_PIN                       PGin(5)

//
// Steppers - 使用SKR PRO的电机控制引脚
//
// X
#define X_MOTION_STEP_PORT                GPIOE
#define X_MOTION_STEP_PIN                 GPIO_PIN_9  // X_STEP_PIN from pins_BTT_SKR_PRO_common.h
#define X_MOTION_STEP_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()

#define X_MOTION_DIR_PORT                 GPIOF
#define X_MOTION_DIR_PIN                  GPIO_PIN_1  // X_DIR_PIN from pins_BTT_SKR_PRO_common.h
#define X_MOTION_DIR_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()

#define X_MOTION_ENA_PORT                 GPIOF
#define X_MOTION_ENA_PIN                  GPIO_PIN_2  // X_ENABLE_PIN from pins_BTT_SKR_PRO_common.h
#define X_MOTION_ENA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()

// Y
#define Y_MOTION_STEP_PORT                GPIOE
#define Y_MOTION_STEP_PIN                 GPIO_PIN_11 // Y_STEP_PIN from pins_BTT_SKR_PRO_common.h
#define Y_MOTION_STEP_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()

#define Y_MOTION_DIR_PORT                 GPIOE
#define Y_MOTION_DIR_PIN                  GPIO_PIN_8  // Y_DIR_PIN from pins_BTT_SKR_PRO_common.h
#define Y_MOTION_DIR_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOE_CLK_ENABLE()

#define Y_MOTION_ENA_PORT                 GPIOD
#define Y_MOTION_ENA_PIN                  GPIO_PIN_7  // Y_ENABLE_PIN from pins_BTT_SKR_PRO_common.h
#define Y_MOTION_ENA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOD_CLK_ENABLE()

// Z
#define Z_MOTION_STEP_PORT                GPIOE
#define Z_MOTION_STEP_PIN                 GPIO_PIN_13 // Z_STEP_PIN from pins_BTT_SKR_PRO_common.h
#define Z_MOTION_STEP_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()

#define Z_MOTION_DIR_PORT                 GPIOC
#define Z_MOTION_DIR_PIN                  GPIO_PIN_2  // Z_DIR_PIN from pins_BTT_SKR_PRO_common.h
#define Z_MOTION_DIR_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()

#define Z_MOTION_ENA_PORT                 GPIOC
#define Z_MOTION_ENA_PIN                  GPIO_PIN_0  // Z_ENABLE_PIN from pins_BTT_SKR_PRO_common.h
#define Z_MOTION_ENA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()

// E0 (A)
#define A_MOTION_STEP_PORT                GPIOE
#define A_MOTION_STEP_PIN                 GPIO_PIN_14 // E0_STEP_PIN from pins_BTT_SKR_PRO_common.h
#define A_MOTION_STEP_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()

#define A_MOTION_DIR_PORT                 GPIOA
#define A_MOTION_DIR_PIN                  GPIO_PIN_0  // E0_DIR_PIN from pins_BTT_SKR_PRO_common.h
#define A_MOTION_DIR_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()

#define A_MOTION_ENA_PORT                 GPIOC
#define A_MOTION_ENA_PIN                  GPIO_PIN_3  // E0_ENABLE_PIN from pins_BTT_SKR_PRO_common.h
#define A_MOTION_ENA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()

// E1 (B)
#define B_MOTION_STEP_PORT                GPIOD
#define B_MOTION_STEP_PIN                 GPIO_PIN_15 // E1_STEP_PIN from pins_BTT_SKR_PRO_common.h
#define B_MOTION_STEP_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()

#define B_MOTION_DIR_PORT                 GPIOE
#define B_MOTION_DIR_PIN                  GPIO_PIN_7  // E1_DIR_PIN from pins_BTT_SKR_PRO_common.h
#define B_MOTION_DIR_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOE_CLK_ENABLE()

#define B_MOTION_ENA_PORT                 GPIOA
#define B_MOTION_ENA_PIN                  GPIO_PIN_3  // E1_ENABLE_PIN from pins_BTT_SKR_PRO_common.h
#define B_MOTION_ENA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()

// E2 (E)
#define E_MOTION_STEP_PORT                GPIOD
#define E_MOTION_STEP_PIN                 GPIO_PIN_13 // E2_STEP_PIN from pins_BTT_SKR_PRO_common.h
#define E_MOTION_STEP_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()

#define E_MOTION_DIR_PORT                 GPIOG
#define E_MOTION_DIR_PIN                  GPIO_PIN_9  // E2_DIR_PIN from pins_BTT_SKR_PRO_common.h
#define E_MOTION_DIR_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()

#define E_MOTION_ENA_PORT                 GPIOF
#define E_MOTION_ENA_PIN                  GPIO_PIN_0  // E2_ENABLE_PIN from pins_BTT_SKR_PRO_common.h
#define E_MOTION_ENA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()

// 气压模块引脚，使用一个未被占用的GPIO引脚
#define PERSS_ENA_PORT                    GPIOC
#define PERSS_ENA_PIN                     GPIO_PIN_6  // 可以选择一个未被占用的引脚
#define PERSS_ENA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOC_CLK_ENABLE()

// 步进电机控制引脚宏定义
#define X_STEP_PIN                        PEout(9)
#define X_DIR_PIN                         PFout(1)
#define X_ENABLE_PIN                      PFout(2)

#define Y_STEP_PIN                        PEout(11)
#define Y_DIR_PIN                         PEout(8)
#define Y_ENABLE_PIN                      PDout(7)

#define Z_STEP_PIN                        PEout(13)
#define Z_DIR_PIN                         PCout(2)
#define Z_ENABLE_PIN                      PCout(0)

#define A_STEP_PIN                        PEout(14)
#define A_DIR_PIN                         PAout(0)
#define A_ENABLE_PIN                      PCout(3)

#define B_STEP_PIN                        PDout(15)
#define B_DIR_PIN                         PEout(7)
#define B_ENABLE_PIN                      PAout(3)

#define E0_STEP_PIN                       PDout(13)
#define E0_DIR_PIN                        PGout(9)
#define E0_ENABLE_PIN                     PFout(0)

#define PRESS_STEP_PIN                    PCout(6)

/*
**********************************************************************************************
*                                         FPGA通信接口定义
*
**********************************************************************************************
*/

/*
**********************************************************************************************
*                                SHT温度 湿度传感器 接口定义
*
**********************************************************************************************
*/

#endif
