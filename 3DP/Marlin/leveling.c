#include "leveling.h"
#include "delay.h" 
#include "marlin.h"
#include "gpio_config.h"

UART_HandleTypeDef g_rs458_handler;     /* RS485控制句柄(串口) */

#ifdef RS485_EN_RX /* 如果使能了接收 */

unsigned char g_RS485_rx_buf[RS485_REC_LEN]; /* 接收缓冲, 最大 RS485_REC_LEN 个字节. */
unsigned char g_RS485_rx_cnt = 0;            /* 接收到的数据长度 */

measure_point_typedef measure_point;

unsigned char is_return_flag = 0;//0：未测量  1：正在测量

unsigned char cmd_measure_str[9] = {0x02,0x4D,0x45,0x41,0x53,0x55,0x52,0x45,0x03};
float angle_a,angle_b;
void get_mp1_data(float cx,float cy)
{
	int i=0;
	//发送采集命令
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
	//发送采集命令
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
	//发送采集命令
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
	//按照顺时钟方向进行
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

    if ((__HAL_UART_GET_FLAG(&g_rs458_handler, UART_FLAG_RXNE) != RESET)) /* 接收到数据 */
    {
        HAL_UART_Receive(&g_rs458_handler, &res, 1, 1000);

        if (g_RS485_rx_cnt < RS485_REC_LEN)         /* 缓冲区未满 */
        {
            g_RS485_rx_buf[g_RS485_rx_cnt] = res;   /* 记录接收到的值 */
            g_RS485_rx_cnt++;                       /* 接收数据增加1 */
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
 * @brief       RS485初始化函数
 *   @note      该函数主要是初始化串口
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @retval      无
 */
void rs485_init(uint32_t baudrate)
{

    /* IO 及 时钟配置 */
    RS485_TX_GPIO_CLK_ENABLE(); /* 使能 串口TX脚 时钟 */
    RS485_RX_GPIO_CLK_ENABLE(); /* 使能 串口RX脚 时钟 */
    RS485_UX_CLK_ENABLE();      /* 使能 串口 时钟 */

    GPIO_InitTypeDef gpio_initure;
    gpio_initure.Pin = RS485_TX_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_AF_PP;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_initure.Alternate = GPIO_AF7_USART3;               /* 复用为串口3 */
    HAL_GPIO_Init(RS485_TX_GPIO_PORT, &gpio_initure);       /* 串口TX 脚 模式设置 */

    gpio_initure.Pin = RS485_RX_GPIO_PIN;
    HAL_GPIO_Init(RS485_RX_GPIO_PORT, &gpio_initure);       /* 串口RX 脚 必须设置成输入模式 */

    /* USART 初始化设置 */
    g_rs458_handler.Instance = RS485_UX;                    /* 选择485对应的串口 */
    g_rs458_handler.Init.BaudRate = baudrate;               /* 波特率 */
    g_rs458_handler.Init.WordLength = UART_WORDLENGTH_8B;   /* 字长为8位数据格式 */
    g_rs458_handler.Init.StopBits = UART_STOPBITS_1;        /* 一个停止位 */
    g_rs458_handler.Init.Parity = UART_PARITY_NONE;         /* 无奇偶校验位 */
    g_rs458_handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;   /* 无硬件流控 */
    g_rs458_handler.Init.Mode = UART_MODE_TX_RX;            /* 收发模式 */
    HAL_UART_Init(&g_rs458_handler);                        /* 使能对应的串口, 调用Msp */
    __HAL_UART_DISABLE_IT(&g_rs458_handler, UART_IT_TC);

#if RS485_EN_RX /* 如果使能了接收 */
                /* 使能接收中断 */
    __HAL_UART_ENABLE_IT(&g_rs458_handler, UART_IT_RXNE);   /* 开启接收中断 */
    HAL_NVIC_EnableIRQ(RS485_UX_IRQn);                      /* 使能USART1中断 */
    HAL_NVIC_SetPriority(RS485_UX_IRQn, 3, 3);              /* 抢占优先级3，子优先级3 */
#endif

}

/**
 * @brief       RS485发送len个字节
 * @param       buf     : 发送区首地址
 * @param       len     : 发送的字节数(为了和本代码的接收匹配,这里建议不要超过 RS485_REC_LEN 个字节)
 * @retval      无
 */
void rs485_send_data(unsigned char *buf, unsigned int len)
{
    HAL_UART_Transmit(&g_rs458_handler, buf, len, 1000); /* 串口2发送数据 */
    g_RS485_rx_cnt = 0;
}

/**
 * @brief       RS485查询接收到的数据
 * @param       buf     : 接收缓冲区首地址
 * @param       len     : 接收到的数据长度
 *   @arg               0   , 表示没有接收到任何数据
 *   @arg               其他, 表示接收到的数据长度
 * @retval      无
 */
void rs485_receive_data(unsigned char *buf, unsigned char *len)
{
    uint8_t rxlen = g_RS485_rx_cnt;
    uint8_t i = 0;
    *len = 0;     /* 默认为0 */
    delay_ms(10); /* 等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束 */

    if (rxlen == g_RS485_rx_cnt && rxlen) /* 接收到了数据,且接收完成了 */
    {
        for (i = 0; i < rxlen; i++)
        {
            buf[i] = g_RS485_rx_buf[i];
        }

        *len = g_RS485_rx_cnt; /* 记录本次数据长度 */
        g_RS485_rx_cnt = 0;    /* 清零 */
    }
}














