#ifndef LEVELING_H_
#define LEVELING_H_

#include "stm32f4xx.h"

#define DEFAULT_A_STEPS_DEG_PER_DEG		625.0f		
#define DEFAULT_B_STEPS_DEG_PER_DEG		625.0f		

/******************************************************************************************/
/* RS485 ���� �� ���� ���� 
 * Ĭ�������RS4852��.
 * ע��: ͨ���޸���10���궨��, ����֧��UART1~UART7����һ������.
 */

//#define RS485_TX_GPIO_PORT                  GPIOB
//#define RS485_TX_GPIO_PIN                   GPIO_PIN_10
//#define RS485_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB��ʱ��ʹ�� */

//#define RS485_RX_GPIO_PORT                  GPIOB
//#define RS485_RX_GPIO_PIN                   GPIO_PIN_11
//#define RS485_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PA��ʱ��ʹ�� */

//#define RS485_UX                            USART3
//#define RS485_UX_IRQn                       USART3_IRQn
//#define RS485_UX_IRQHandler                 USART3_IRQHandler
//#define RS485_UX_CLK_ENABLE()               do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)  /* USART3 ʱ��ʹ�� */

/******************************************************************************************/

/* ����RS485_RE��, ����RS485����/����״̬
 * RS485_RE = 0, �������ģʽ
 * RS485_RE = 1, ���뷢��ģʽ
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

#define RS485_REC_LEN               64          /* �����������ֽ��� 64 */
#define RS485_EN_RX                 1           /* ʹ�ܣ�1��/��ֹ��0��RS485���� */


extern unsigned char g_RS485_rx_buf[RS485_REC_LEN];   /* ���ջ���,���RS485_REC_LEN���ֽ� */
extern unsigned char g_RS485_rx_cnt;                  /* �������ݳ��� */


//void rs485_init( unsigned int baudrate);  /* RS485��ʼ�� */
//void rs485_send_data(unsigned char *buf, unsigned int len);    /* RS485�������� */
//void rs485_receive_data(unsigned char *buf, unsigned char *len);/* RS485�������� */

void get_mp1_data(float cx,float cy);
void get_mp2_data(float cx,float cy);
void get_mp3_data(float cx,float cy);
void compute_angle(void);
void send_pluse_to_stepper(void);












#endif
