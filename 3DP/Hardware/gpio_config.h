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
//LED1
#define LED1_PIN                  GPIO_PIN_8                
#define LED1_GPIO_PORT            GPIOA                     
#define LED1_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()

//LED2
#define LED2_PIN                  GPIO_PIN_7                 
#define LED2_GPIO_PORT            GPIOA                      
#define LED2_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()

//LED3
#define LED3_PIN                  GPIO_PIN_6
#define LED3_GPIO_PORT            GPIOA                      
#define LED3_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()

//LED4
#define LED4_PIN                  GPIO_PIN_8
#define LED4_GPIO_PORT            GPIOB                      
#define LED4_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()
//
#define		LED1			PAout(8)
#define		LED2			PAout(7)
#define		LED3			PAout(6)
#define		LED4			PBout(8)
/*
**********************************************************************************************
*                                         步进电机接口定义
*
**********************************************************************************************
*/
//
// Limit Switches
// x
#define	X_MIN_LIMIT_PORT								GPIOG
#define	X_MIN_LIMIT_PIN									GPIO_PIN_2
#define	X_MIN_LIMIT_GPIO_CLK_ENABLE()		__GPIOG_CLK_ENABLE()

#define	X_ZERO_LIMIT_PORT								GPIOC
#define	X_ZERO_LIMIT_PIN								GPIO_PIN_8
#define	X_ZERO_LIMIT_GPIO_CLK_ENABLE()	__GPIOC_CLK_ENABLE()

#define	X_MAX_LIMIT_PORT								GPIOG
#define	X_MAX_LIMIT_PIN									GPIO_PIN_3
#define	X_MAX_LIMIT_GPIO_CLK_ENABLE()		__GPIOG_CLK_ENABLE()

// Y G4 G8 G5
#define	Y_MIN_LIMIT_PORT								GPIOG
#define	Y_MIN_LIMIT_PIN									GPIO_PIN_5
#define	Y_MIN_LIMIT_GPIO_CLK_ENABLE()		__GPIOG_CLK_ENABLE()

#define	Y_ZERO_LIMIT_PORT								GPIOG
#define	Y_ZERO_LIMIT_PIN								GPIO_PIN_8
#define	Y_ZERO_LIMIT_GPIO_CLK_ENABLE()	__GPIOG_CLK_ENABLE()

#define	Y_MAX_LIMIT_PORT								GPIOG
#define	Y_MAX_LIMIT_PIN									GPIO_PIN_4
#define	Y_MAX_LIMIT_GPIO_CLK_ENABLE()		__GPIOG_CLK_ENABLE()

// Z G6 D3 G7
#define	Z_MIN_LIMIT_PORT								GPIOG
#define	Z_MIN_LIMIT_PIN									GPIO_PIN_7
#define	Z_MIN_LIMIT_GPIO_CLK_ENABLE()		__GPIOG_CLK_ENABLE()

#define	Z_ZERO_LIMIT_PORT								GPIOD
#define	Z_ZERO_LIMIT_PIN								GPIO_PIN_3
#define	Z_ZERO_LIMIT_GPIO_CLK_ENABLE()	__GPIOD_CLK_ENABLE()

#define	Z_MAX_LIMIT_PORT								GPIOG
#define	Z_MAX_LIMIT_PIN									GPIO_PIN_6
#define	Z_MAX_LIMIT_GPIO_CLK_ENABLE()		__GPIOG_CLK_ENABLE()

#define X_MIN_PIN                      	    PGin(2)
#define X_ZERO_PIN                      	  PCin(8)
#define X_MAX_PIN                      	    PGin(3)

#define Y_MIN_PIN                      	    PGin(5)
#define Y_ZERO_PIN                      	  PGin(8)
#define Y_MAX_PIN                      	    PGin(4)

#define Z_MIN_PIN                      	    PGin(7)
#define Z_ZERO_PIN                      	  PDin(3)
#define Z_MAX_PIN                      	    PGin(6)
//
// Steppers
//
//X I5 F14 F15
#define	X_MOTION_STEP_PORT										GPIOF
#define	X_MOTION_STEP_PIN											GPIO_PIN_15
#define	X_MOTION_STEP_GPIO_CLK_ENABLE()				__GPIOF_CLK_ENABLE()

#define	X_MOTION_DIR_PORT											GPIOF
#define	X_MOTION_DIR_PIN											GPIO_PIN_14
#define	X_MOTION_DIR_GPIO_CLK_ENABLE()				__GPIOF_CLK_ENABLE()

#define	X_MOTION_ENA_PORT											GPIOI
#define	X_MOTION_ENA_PIN											GPIO_PIN_5
#define	X_MOTION_ENA_GPIO_CLK_ENABLE()				__GPIOI_CLK_ENABLE()
// Y I6 F12 F13
#define	Y_MOTION_STEP_PORT										GPIOF
#define	Y_MOTION_STEP_PIN											GPIO_PIN_13
#define	Y_MOTION_STEP_GPIO_CLK_ENABLE()				__GPIOF_CLK_ENABLE()

#define	Y_MOTION_DIR_PORT											GPIOF
#define	Y_MOTION_DIR_PIN											GPIO_PIN_12
#define	Y_MOTION_DIR_GPIO_CLK_ENABLE()				__GPIOF_CLK_ENABLE()

#define	Y_MOTION_ENA_PORT											GPIOI
#define	Y_MOTION_ENA_PIN											GPIO_PIN_6
#define	Y_MOTION_ENA_GPIO_CLK_ENABLE()				__GPIOI_CLK_ENABLE()
// Z I7 B2 F11
#define	Z_MOTION_STEP_PORT										GPIOF
#define	Z_MOTION_STEP_PIN											GPIO_PIN_11
#define	Z_MOTION_STEP_GPIO_CLK_ENABLE()				__GPIOF_CLK_ENABLE()

#define	Z_MOTION_DIR_PORT											GPIOB
#define	Z_MOTION_DIR_PIN											GPIO_PIN_2
#define	Z_MOTION_DIR_GPIO_CLK_ENABLE()				__GPIOB_CLK_ENABLE()

#define	Z_MOTION_ENA_PORT											GPIOI
#define	Z_MOTION_ENA_PIN											GPIO_PIN_7
#define	Z_MOTION_ENA_GPIO_CLK_ENABLE()				__GPIOI_CLK_ENABLE()
// A E8 E7 D1
#define	A_MOTION_STEP_PORT										GPIOE
#define	A_MOTION_STEP_PIN											GPIO_PIN_11
#define	A_MOTION_STEP_GPIO_CLK_ENABLE()				__GPIOE_CLK_ENABLE()

#define	A_MOTION_DIR_PORT											GPIOE
#define	A_MOTION_DIR_PIN											GPIO_PIN_10
#define	A_MOTION_DIR_GPIO_CLK_ENABLE()				__GPIOE_CLK_ENABLE()

#define	A_MOTION_ENA_PORT											GPIOE
#define	A_MOTION_ENA_PIN											GPIO_PIN_9
#define	A_MOTION_ENA_GPIO_CLK_ENABLE()				__GPIOE_CLK_ENABLE()
// B D0 D15 D14
#define	B_MOTION_STEP_PORT										GPIOE
#define	B_MOTION_STEP_PIN											GPIO_PIN_14
#define	B_MOTION_STEP_GPIO_CLK_ENABLE()				__GPIOE_CLK_ENABLE()

#define	B_MOTION_DIR_PORT											GPIOE
#define	B_MOTION_DIR_PIN											GPIO_PIN_13
#define	B_MOTION_DIR_GPIO_CLK_ENABLE()				__GPIOE_CLK_ENABLE()

#define	B_MOTION_ENA_PORT											GPIOE
#define	B_MOTION_ENA_PIN											GPIO_PIN_12
#define	B_MOTION_ENA_GPIO_CLK_ENABLE()				__GPIOE_CLK_ENABLE()
// E
#define	E_MOTION_STEP_PORT										GPIOC
#define	E_MOTION_STEP_PIN											GPIO_PIN_0
#define	E_MOTION_STEP_GPIO_CLK_ENABLE()				__GPIOC_CLK_ENABLE()

#define	E_MOTION_DIR_PORT											GPIOH
#define	E_MOTION_DIR_PIN											GPIO_PIN_5
#define	E_MOTION_DIR_GPIO_CLK_ENABLE()				__GPIOH_CLK_ENABLE()

#define	E_MOTION_ENA_PORT											GPIOH
#define	E_MOTION_ENA_PIN											GPIO_PIN_4
#define	E_MOTION_ENA_GPIO_CLK_ENABLE()				__GPIOH_CLK_ENABLE()


#define	PERSS_ENA_PORT											GPIOC
#define	PERSS_ENA_PIN											  GPIO_PIN_1
#define	PERSS_ENA_GPIO_CLK_ENABLE()				  __GPIOC_CLK_ENABLE()


#define X_STEP_PIN                          PFout(15)
#define X_DIR_PIN                           PFout(14)
#define X_ENABLE_PIN                        PIout(5)

#define Y_STEP_PIN                          PFout(13)
#define Y_DIR_PIN                           PFout(12)
#define Y_ENABLE_PIN                        PIout(6)

#define Z_STEP_PIN                          PFout(11)
#define Z_DIR_PIN                           PBout(2)
#define Z_ENABLE_PIN                        PIout(7)

#define A_STEP_PIN                          PEout(11)
#define A_DIR_PIN                           PEout(10)
#define A_ENABLE_PIN                        PDout(9)

#define B_STEP_PIN                          PEout(14)
#define B_DIR_PIN                           PEout(13)
#define B_ENABLE_PIN                        PEout(12)


#define E0_STEP_PIN                         PCout(0)
#define E0_DIR_PIN                          PHout(5)
#define E0_ENABLE_PIN                       PHout(4)

#define PRESS_STEP_PIN											PCout(1)

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
