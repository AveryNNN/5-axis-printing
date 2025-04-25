#ifndef connect_bsp_h
#define connect_bsp_h

//外部相关头文件
#include "usart.h"

void bsp_stepper_init(void);
	void bsp_set_timer_autoreload(unsigned short timer_count);
	// 电机使能
	void bsp_enable_x_stepper(void);
	void bsp_enable_y_stepper(void);
	void bsp_enable_z_stepper(void);
	void bsp_enable_e_stepper(void);
	// 电机禁止
	void bsp_disable_x_stepper(void);
	void bsp_disable_y_stepper(void);
	void bsp_disable_z_stepper(void);
	void bsp_disable_e_stepper(void);
	// 限位开关
	unsigned char bsp_get_x_axis_min_endstop(void);
	unsigned char bsp_get_x_axis_zero_endstop(void);
	unsigned char bsp_get_x_axis_max_endstop(void);
	unsigned char bsp_get_y_axis_min_endstop(void);
	unsigned char bsp_get_y_axis_zero_endstop(void);
	unsigned char bsp_get_y_axis_max_endstop(void);
	unsigned char bsp_get_z_axis_min_endstop(void);
	unsigned char bsp_get_z_axis_zero_endstop(void);
	unsigned char bsp_get_z_axis_max_endstop(void);
	// 输出脉冲
	void bsp_create_one_x_axis_step(void);
	void bsp_create_one_y_axis_step(void);
	void bsp_create_one_z_axis_step(void);
	void bsp_create_one_e_axis_step(void);
	// 运动方向
	void bsp_set_x_axis_dir(unsigned char dir);
	void bsp_set_y_axis_dir(unsigned char dir);
	void bsp_set_z_axis_dir(unsigned char dir);
	void bsp_set_e_axis_dir(unsigned char dir);
	
	void bsp_set_plan_line_num(unsigned int num);
	
	//=======================================usart===================================//
	void bsp_serial_init(unsigned int bound);
	unsigned int bsp_get_timer_ms(void);
	
	
//
	void bsp_init(void);
//外部串口中断调用函数
	void bsp_serial_isr(unsigned char ch);
	
	void bsp_command_process(void);
	
	void bsp_stepper_timer_isr(void);
	
	void soft_delay_us(void);
	void soft_delay_10us(void);
	void soft_delay_100us(void);
	// A 轴和 B 轴相关函数声明
void bsp_enable_a_stepper(void);
void bsp_enable_b_stepper(void);
void bsp_disable_a_stepper(void);
void bsp_disable_b_stepper(void);
void bsp_create_one_a_axis_step(void);
void bsp_create_one_b_axis_step(void);
void bsp_set_a_axis_dir(unsigned char dir);
void bsp_set_b_axis_dir(unsigned char dir);
#endif
