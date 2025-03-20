
#include "marlin.h"
#include "connect_bsp.h"
#include "timer.h"
#include "stepper_gpio.h"
#include "gpio_config.h"

//外部主进程调用模块插补函数
	void bsp_init(void)
	{
		//进行函数指针关联
		steppers.create_one_e_axis_step = bsp_create_one_e_axis_step;
		steppers.create_one_x_axis_step = bsp_create_one_x_axis_step;
		steppers.create_one_y_axis_step = bsp_create_one_y_axis_step;
		steppers.create_one_z_axis_step = bsp_create_one_z_axis_step;
		steppers.create_one_a_axis_step = bsp_create_one_a_axis_step;
		steppers.create_one_b_axis_step = bsp_create_one_b_axis_step;
		
		steppers.disable_e_stepper			=	bsp_disable_e_stepper;
		steppers.disable_x_stepper			=	bsp_disable_x_stepper;
		steppers.disable_y_stepper			=	bsp_disable_y_stepper;
		steppers.disable_z_stepper			=	bsp_disable_z_stepper;
		steppers.disable_a_stepper = bsp_disable_a_stepper;
		steppers.disable_b_stepper = bsp_disable_b_stepper;
		
		steppers.enable_e_stepper				=	bsp_enable_e_stepper;
		steppers.enable_x_stepper				=	bsp_enable_x_stepper;
		steppers.enable_y_stepper				=	bsp_enable_y_stepper;
		steppers.enable_z_stepper				=	bsp_enable_z_stepper;
		steppers.enable_a_stepper = bsp_enable_a_stepper;
		steppers.enable_b_stepper = bsp_enable_b_stepper;
		
		steppers.get_x_axis_max_endstop	=	bsp_get_x_axis_max_endstop;
		steppers.get_x_axis_zero_endstop=	bsp_get_x_axis_zero_endstop;
		steppers.get_x_axis_min_endstop	=	bsp_get_x_axis_min_endstop;
		
		steppers.get_y_axis_max_endstop	=	bsp_get_y_axis_max_endstop;
		steppers.get_y_axis_zero_endstop=	bsp_get_y_axis_zero_endstop;
		steppers.get_y_axis_min_endstop	=	bsp_get_y_axis_min_endstop;
		
		steppers.get_z_axis_max_endstop	=	bsp_get_z_axis_max_endstop;
		steppers.get_z_axis_zero_endstop=	bsp_get_z_axis_zero_endstop;
		steppers.get_z_axis_min_endstop	=	bsp_get_z_axis_min_endstop;
		
		steppers.set_e_axis_dir					=	bsp_set_e_axis_dir;
		steppers.set_x_axis_dir					=	bsp_set_x_axis_dir;
		steppers.set_y_axis_dir					=	bsp_set_y_axis_dir;
		steppers.set_z_axis_dir					=	bsp_set_z_axis_dir;
		steppers.set_a_axis_dir = bsp_set_a_axis_dir;
		steppers.set_b_axis_dir = bsp_set_b_axis_dir;
		
		steppers.set_plan_line_num			= bsp_set_plan_line_num;
		
		steppers.set_timer_autoreload		=	bsp_set_timer_autoreload;
		steppers.stepper_init						=	bsp_stepper_init;//中断初始化
		//关联串口操作函数
		serial.serial_init							=	bsp_serial_init;
		serial.get_timer_ms							=	bsp_get_timer_ms;
		
		
		//对模块初始化
		steppers.stepper_init();
		serial.serial_init(BAUDRATE);
		
	}
//外部串口中断调用函数
	void bsp_serial_isr(unsigned char ch)
	{
		serial_isr(ch);
	}
// 需要持续运行的函数
	void bsp_command_process(void)
	{
		start_command_process();
	}
// 外部定时器周期调用函数  联动插补电机运动
	void bsp_stepper_timer_isr(void)
	{
		timer_isr();
	}

//=======================================usart===================================//
	void bsp_serial_init(unsigned int bound)
	{
		//
		SERIAL_USART_Config(bound);
	}
//=======================================timer===================================//
	unsigned int bsp_get_timer_ms(void)
	{
		unsigned int temp;
		temp = get_timer_ms();
		return temp;
	}
//======================================stepper==================================//
	void bsp_stepper_init(void)
	{
		// 步进电机初始化
		// 初始化定时器
		timer_init();
		// 初始化引脚GPIO
		stepper_gpio_init();
		return;
	}
	void bsp_set_timer_autoreload(unsigned short timer_count)
	{
		set_timer_autoload(timer_count);
		return;
	}
	// 电机使能
	void bsp_enable_x_stepper(void)
	{
		X_ENABLE_PIN = STEPPER_ENABLE_VOL;
		return;
	}
	void bsp_enable_y_stepper(void)
	{
		Y_ENABLE_PIN = STEPPER_ENABLE_VOL;
		return;
	}
	void bsp_enable_z_stepper(void)
	{
		Z_ENABLE_PIN = STEPPER_ENABLE_VOL;
		return;
	}
	void bsp_enable_e_stepper(void)
	{
		E0_ENABLE_PIN = STEPPER_ENABLE_VOL;
		return;
	}
	void bsp_enable_a_stepper(void)
{
  A_ENABLE_PIN = STEPPER_ENABLE_VOL;
  return;
}

void bsp_enable_b_stepper(void)
{
  B_ENABLE_PIN = STEPPER_ENABLE_VOL;
  return;
}
	
	
	// 电机禁止
	void bsp_disable_x_stepper(void)
	{
		X_ENABLE_PIN = STEPPER_DISABLE_VOL;
		return;
	}
	void bsp_disable_y_stepper(void)
	{
		Y_ENABLE_PIN = STEPPER_DISABLE_VOL;
		return;
	}
	void bsp_disable_z_stepper(void)
	{
		Z_ENABLE_PIN = STEPPER_DISABLE_VOL;
		return;
	}
	void bsp_disable_e_stepper(void)
	{
		E0_ENABLE_PIN = STEPPER_DISABLE_VOL;
		return;
	}
	void bsp_disable_a_stepper(void)
{
  A_ENABLE_PIN = STEPPER_DISABLE_VOL;
  return;
}

void bsp_disable_b_stepper(void)
{
  B_ENABLE_PIN = STEPPER_DISABLE_VOL;
  return;
}
	
	// 限位开关
	unsigned char bsp_get_x_axis_min_endstop(void)
	{
		unsigned char temp;
		temp = X_MIN_PIN;
		if(temp == ENDSTOP_HIT_VOL)
		{
			return 1;
		}
		return 0;
	}
	unsigned char bsp_get_x_axis_zero_endstop(void)
	{
		unsigned char temp;
		temp = X_ZERO_PIN;
		if(temp == ENDSTOP_HIT_VOL)
		{
			return 1;
		}
		return 0;
	}
	unsigned char bsp_get_x_axis_max_endstop(void)
	{
		unsigned char temp;
		temp = X_MAX_PIN;
		if(temp == ENDSTOP_HIT_VOL)
		{
			return 1;
		}
		return 0;
	}
	unsigned char bsp_get_y_axis_min_endstop(void)
	{
		unsigned char temp;
		temp = Y_MIN_PIN;
		if(temp == ENDSTOP_HIT_VOL)
		{
			return 1;
		}
		return 0;
	}
	unsigned char bsp_get_y_axis_zero_endstop(void)
	{
		unsigned char temp;
		temp = Y_ZERO_PIN;
		if(temp == ENDSTOP_HIT_VOL)
		{
			return 1;
		}
		return 0;
	}
	unsigned char bsp_get_y_axis_max_endstop(void)
	{
		unsigned char temp;
		temp = Y_MAX_PIN;
		if(temp == ENDSTOP_HIT_VOL)
		{
			return 1;
		}
		return 0;
	}
	unsigned char bsp_get_z_axis_min_endstop(void)
	{
		unsigned char temp;
		temp = Z_MIN_PIN;
		if(temp == ENDSTOP_HIT_VOL)
		{
			return 1;
		}
		return 0;
	}
	unsigned char bsp_get_z_axis_zero_endstop(void)
	{
		unsigned char temp;
		temp = Z_ZERO_PIN;
		if(temp == ENDSTOP_HIT_VOL)
		{
			return 1;
		}
		return 0;
	}
	unsigned char bsp_get_z_axis_max_endstop(void)
	{
		unsigned char temp;
		temp = Z_MAX_PIN;
		if(temp == ENDSTOP_HIT_VOL)
		{
			return 1;
		}
		return 0;
	}
	// 输出脉冲
	void bsp_create_one_x_axis_step(void)
	{
		X_STEP_PIN = 1;
		soft_delay_us();
		X_STEP_PIN = 0;
		soft_delay_us();
		return;
	}
	void bsp_create_one_y_axis_step(void)
	{
		Y_STEP_PIN = 1;
		soft_delay_us();
		Y_STEP_PIN = 0;
		soft_delay_us();
		return;
	}
	void bsp_create_one_z_axis_step(void)
	{
		Z_STEP_PIN = 1;
		soft_delay_us();
		Z_STEP_PIN = 0;
		soft_delay_us();
		return;
	}
	void bsp_create_one_e_axis_step(void)
	{
		E0_STEP_PIN = !E0_STEP_PIN;
//		E0_STEP_PIN = 1;
//		soft_delay_us();
//		E0_STEP_PIN = 0;
//		soft_delay_us();
		return;
	}
	
	void bsp_create_one_a_axis_step(void)
{
  A_STEP_PIN = 1;
  soft_delay_us();
  A_STEP_PIN = 0;
  soft_delay_us();
  return;
}

void bsp_create_one_b_axis_step(void)
{
  B_STEP_PIN = 1;
  soft_delay_us();
  B_STEP_PIN = 0;
  soft_delay_us();
  return;
}
	// 运动方向
	void bsp_set_x_axis_dir(unsigned char dir)
	{
		if(dir)	X_DIR_PIN = 1;
		else 		X_DIR_PIN = 0;
		return;
	}
	void bsp_set_y_axis_dir(unsigned char dir)
	{
		if(dir)	Y_DIR_PIN = 1;
		else 		Y_DIR_PIN = 0;
		return;
	}
	void bsp_set_z_axis_dir(unsigned char dir)
	{
		if(dir)	Z_DIR_PIN = 1;
		else 		Z_DIR_PIN = 0;
		return;
	}
	void bsp_set_e_axis_dir(unsigned char dir)
	{
		if(dir)	E0_DIR_PIN = 1;
		else 		E0_DIR_PIN = 0;
		return;
	}
	void bsp_set_a_axis_dir(unsigned char dir)
{
  if(dir) A_DIR_PIN = 1;
  else    A_DIR_PIN = 0;
  return;
}

void bsp_set_b_axis_dir(unsigned char dir)
{
  if(dir) B_DIR_PIN = 1;
  else    B_DIR_PIN = 0;
  return;
}
	void bsp_set_plan_line_num(unsigned int num)
	{
		//用LED灯来指示
		switch(num % 4)
		{
			case 0: 
				LED1 = 1;
				LED2 = 0;
				LED3 = 0;
				LED4 = 0;
			break;
			case 1: 
				LED1 = 0;
				LED2 = 1;
				LED3 = 0;
				LED4 = 0;
			break;
			case 2: 
				LED1 = 0;
				LED2 = 0;
				LED3 = 1;
				LED4 = 0;
			break;
			case 3: 
				LED1 = 0;
				LED2 = 0;
				LED3 = 0;
				LED4 = 1;
			break;
		}
	}

void soft_delay_us(void)
{
	int temp = 500;
	while(temp--);
}







