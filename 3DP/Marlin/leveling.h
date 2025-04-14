#ifndef LEVELING_H_
#define LEVELING_H_

#include "stm32f4xx.h"

#define DEFAULT_A_STEPS_DEG_PER_DEG		625.0f		
#define DEFAULT_B_STEPS_DEG_PER_DEG		625.0f		

/******************************************************************************************/
/* RS485 引脚 和 串口 定义 
 * 默认是针对RS4852的.
 * 注意: 通过修改这10个宏定义, 可以支持UART1~UART7任意一个串口.
 */

//#define RS485_TX_GPIO_PORT                  GPIOB
//#define RS485_TX_GPIO_PIN                   GPIO_PIN_10
//#define RS485_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

//#define RS485_RX_GPIO_PORT                  GPIOB
//#define RS485_RX_GPIO_PIN                   GPIO_PIN_11
//#define RS485_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

//#define RS485_UX                            USART3
//#define RS485_UX_IRQn                       USART3_IRQn
//#define RS485_UX_IRQHandler                 USART3_IRQHandler
//#define RS485_UX_CLK_ENABLE()               do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)  /* USART3 时钟使能 */

/******************************************************************************************/

/* 控制RS485_RE脚, 控制RS485发送/接收状态
 * RS485_RE = 0, 进入接收模式
 * RS485_RE = 1, 进入发送模式
 */

typedef struct
{
	float p1_x;
	float p1_y;
	float p1_z;
	
	float p2_x;
	float p2_y;
	float p2_z;
	
	float p3_x;
	float p3_y;
	float p3_z;	
}measure_point_typedef;

#define RS485_REC_LEN               64          /* 定义最大接收字节数 64 */
#define RS485_EN_RX                 1           /* 使能（1）/禁止（0）RS485接收 */


extern unsigned char g_RS485_rx_buf[RS485_REC_LEN];   /* 接收缓冲,最大RS485_REC_LEN个字节 */
extern unsigned char g_RS485_rx_cnt;                  /* 接收数据长度 */


//void rs485_init( unsigned int baudrate);  /* RS485初始化 */
//void rs485_send_data(unsigned char *buf, unsigned int len);    /* RS485发送数据 */
//void rs485_receive_data(unsigned char *buf, unsigned char *len);/* RS485接收数据 */

void get_mp1_data(float cx,float cy);
void get_mp2_data(float cx,float cy);
void get_mp3_data(float cx,float cy);
void compute_angle(void);
void send_pluse_to_stepper(void);












#endif
