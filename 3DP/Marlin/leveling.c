#include "leveling.h"
#include "delay.h" 
#include "marlin.h"
#include "gpio_config.h"

UART_HandleTypeDef g_rs458_handler;     /* RS485���ƾ��(����) */

#ifdef RS485_EN_RX /* ���ʹ���˽��� */

unsigned char g_RS485_rx_buf[RS485_REC_LEN]; /* ���ջ���, ��� RS485_REC_LEN ���ֽ�. */
unsigned char g_RS485_rx_cnt = 0;            /* ���յ������ݳ��� */

measure_point_typedef measure_point;

unsigned char is_return_flag = 0;//0��δ����  1�����ڲ���

unsigned char cmd_measure_str[9] = {0x02,0x4D,0x45,0x41,0x53,0x55,0x52,0x45,0x03};
float angle_a,angle_b;
void get_mp1_data(float cx,float cy)
{
	int i=0;
	//���Ͳɼ�����
	is_return_flag=0;
	rs485_send_data(cmd_measure_str,9);
	for(i=0;i<100;i++)
	{
		delay_ms(1);
		if(is_return_flag==1)break;
	}
	if(is_return_flag==0)return;
	measure_point.p1_x=cx;
	measure_point.p1_y=cy;
	measure_point.p1_z=(float)strtod((char *)&g_RS485_rx_buf[1], NULL);
	printf("MP1:X=%f Y=%f Z=%f\r\n",measure_point.p1_x,measure_point.p1_y,measure_point.p1_z);	
}
void get_mp2_data(float cx,float cy)
{
	int i=0;
	//���Ͳɼ�����
	is_return_flag=0;
	rs485_send_data(cmd_measure_str,9);
	for(i=0;i<100;i++)
	{
		delay_ms(1);
		if(is_return_flag==1)break;
	}
	if(is_return_flag==0)return;
	measure_point.p2_x=cx;
	measure_point.p2_y=cy;
	measure_point.p2_z=(float)strtod((char *)&g_RS485_rx_buf[1], NULL);
	printf("MP2:X=%f Y=%f Z=%f\r\n",measure_point.p2_x,measure_point.p2_y,measure_point.p2_z);	
}

void get_mp3_data(float cx,float cy)
{
	int i=0;
	//���Ͳɼ�����
	is_return_flag=0;
	rs485_send_data(cmd_measure_str,9);
	for(i=0;i<100;i++)
	{
		delay_ms(1);
		if(is_return_flag==1)break;
	}
	if(is_return_flag==0)return;
	measure_point.p3_x=cx;
	measure_point.p3_y=cy;
	measure_point.p3_z=(float)strtod((char *)&g_RS485_rx_buf[1], NULL);
	printf("MP3:X=%f Y=%f Z=%f\r\n",measure_point.p3_x,measure_point.p3_y,measure_point.p3_z);	
}

void compute_angle(void)
{
	//����˳ʱ�ӷ������
	float e1_x,e1_y,e1_z;
	float e2_x,e2_y,e2_z;
	float e1_norm,e2_norm;
	e1_x = -(measure_point.p1_x - measure_point.p2_x);
	e1_y = -(measure_point.p1_y - measure_point.p2_y);
	e1_z = -(measure_point.p1_z - measure_point.p2_z);
	e1_norm = sqrt(e1_x*e1_x + e1_y*e1_y + e1_z*e1_z);
	if(e1_x>0.0f)
	{
		angle_a = acos(e1_x/e1_norm)*180.0f/3.141592654f;
		if(e1_z<0.0f)angle_a=-angle_a;
	}
	else
	{
		printf("e1_x<=0.0f\r\n");
	}
	
	e2_x = -(measure_point.p3_x - measure_point.p2_x);
	e2_y = -(measure_point.p3_y - measure_point.p2_y);
	e2_z = (measure_point.p3_z - measure_point.p2_z);
	e2_norm = sqrt(e2_x*e2_x + e2_y*e2_y + e2_z*e2_z);
	
	if(e2_y>0.0f)
	{
		angle_b = acos(e2_y/e2_norm)*180.0f/3.141592654f;
		if(e2_z<0.0f)angle_b=-angle_b;
	}
	else
	{
		printf("e2_y<=0.0f\r\n");
	}
	//
	printf("A=%f[deg] B=%f[deg]\r\n",angle_a,angle_b);
}

void send_pluse_to_stepper(void)
{
	unsigned int a_axis_steps = abs(lround(angle_a*DEFAULT_A_STEPS_DEG_PER_DEG));
	unsigned int b_axis_steps = abs(lround(angle_b*DEFAULT_A_STEPS_DEG_PER_DEG));
	unsigned int max_steps = max(a_axis_steps,b_axis_steps);
	int i;
	if(angle_a>0.0f)A_DIR_PIN = 1;
	else A_DIR_PIN = 0;
	if(angle_b>0.0f)B_DIR_PIN = 1;
	else B_DIR_PIN = 0;
	delay_us(10);
	for(i=0;i<max_steps;i++)
	{
		if(i<a_axis_steps)A_STEP_PIN=1;
		if(i<b_axis_steps)B_STEP_PIN=1;
		delay_us(10);
		A_STEP_PIN = 0;
		B_STEP_PIN = 0;
		delay_ms(1);
	}
}

void output_a_axis_pluse_to_drive(void)
{
	
}

void RS485_UX_IRQHandler(void)
{
    uint8_t res;

    if ((__HAL_UART_GET_FLAG(&g_rs458_handler, UART_FLAG_RXNE) != RESET)) /* ���յ����� */
    {
        HAL_UART_Receive(&g_rs458_handler, &res, 1, 1000);

        if (g_RS485_rx_cnt < RS485_REC_LEN)         /* ������δ�� */
        {
            g_RS485_rx_buf[g_RS485_rx_cnt] = res;   /* ��¼���յ���ֵ */
            g_RS485_rx_cnt++;                       /* ������������1 */
        }
    }
		if(res==0x03)	
		{
			g_RS485_rx_buf[--g_RS485_rx_cnt] = 0x00;
			is_return_flag = 1;
		}
}

#endif

/**
 * @brief       RS485��ʼ������
 *   @note      �ú�����Ҫ�ǳ�ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @retval      ��
 */
void rs485_init(uint32_t baudrate)
{

    /* IO �� ʱ������ */
    RS485_TX_GPIO_CLK_ENABLE(); /* ʹ�� ����TX�� ʱ�� */
    RS485_RX_GPIO_CLK_ENABLE(); /* ʹ�� ����RX�� ʱ�� */
    RS485_UX_CLK_ENABLE();      /* ʹ�� ���� ʱ�� */

    GPIO_InitTypeDef gpio_initure;
    gpio_initure.Pin = RS485_TX_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_AF_PP;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_initure.Alternate = GPIO_AF7_USART3;               /* ����Ϊ����3 */
    HAL_GPIO_Init(RS485_TX_GPIO_PORT, &gpio_initure);       /* ����TX �� ģʽ���� */

    gpio_initure.Pin = RS485_RX_GPIO_PIN;
    HAL_GPIO_Init(RS485_RX_GPIO_PORT, &gpio_initure);       /* ����RX �� �������ó�����ģʽ */

    /* USART ��ʼ������ */
    g_rs458_handler.Instance = RS485_UX;                    /* ѡ��485��Ӧ�Ĵ��� */
    g_rs458_handler.Init.BaudRate = baudrate;               /* ������ */
    g_rs458_handler.Init.WordLength = UART_WORDLENGTH_8B;   /* �ֳ�Ϊ8λ���ݸ�ʽ */
    g_rs458_handler.Init.StopBits = UART_STOPBITS_1;        /* һ��ֹͣλ */
    g_rs458_handler.Init.Parity = UART_PARITY_NONE;         /* ����żУ��λ */
    g_rs458_handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;   /* ��Ӳ������ */
    g_rs458_handler.Init.Mode = UART_MODE_TX_RX;            /* �շ�ģʽ */
    HAL_UART_Init(&g_rs458_handler);                        /* ʹ�ܶ�Ӧ�Ĵ���, ����Msp */
    __HAL_UART_DISABLE_IT(&g_rs458_handler, UART_IT_TC);

#if RS485_EN_RX /* ���ʹ���˽��� */
                /* ʹ�ܽ����ж� */
    __HAL_UART_ENABLE_IT(&g_rs458_handler, UART_IT_RXNE);   /* ���������ж� */
    HAL_NVIC_EnableIRQ(RS485_UX_IRQn);                      /* ʹ��USART1�ж� */
    HAL_NVIC_SetPriority(RS485_UX_IRQn, 3, 3);              /* ��ռ���ȼ�3�������ȼ�3 */
#endif

}

/**
 * @brief       RS485����len���ֽ�
 * @param       buf     : �������׵�ַ
 * @param       len     : ���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ���� RS485_REC_LEN ���ֽ�)
 * @retval      ��
 */
void rs485_send_data(unsigned char *buf, unsigned int len)
{
    HAL_UART_Transmit(&g_rs458_handler, buf, len, 1000); /* ����2�������� */
    g_RS485_rx_cnt = 0;
}

/**
 * @brief       RS485��ѯ���յ�������
 * @param       buf     : ���ջ������׵�ַ
 * @param       len     : ���յ������ݳ���
 *   @arg               0   , ��ʾû�н��յ��κ�����
 *   @arg               ����, ��ʾ���յ������ݳ���
 * @retval      ��
 */
void rs485_receive_data(unsigned char *buf, unsigned char *len)
{
    uint8_t rxlen = g_RS485_rx_cnt;
    uint8_t i = 0;
    *len = 0;     /* Ĭ��Ϊ0 */
    delay_ms(10); /* �ȴ�10ms,��������10msû�н��յ�һ������,����Ϊ���ս��� */

    if (rxlen == g_RS485_rx_cnt && rxlen) /* ���յ�������,�ҽ�������� */
    {
        for (i = 0; i < rxlen; i++)
        {
            buf[i] = g_RS485_rx_buf[i];
        }

        *len = g_RS485_rx_cnt; /* ��¼�������ݳ��� */
        g_RS485_rx_cnt = 0;    /* ���� */
    }
}














