/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef stepper_h
#define stepper_h 

#include "planner.h"




typedef struct
{
	void (*stepper_init)(void);
	void (*set_timer_autoreload)(unsigned short timer_count);
	// 电机使能
	void (*enable_x_stepper)(void);
	void (*enable_y_stepper)(void);
	void (*enable_z_stepper)(void);
	void (*enable_e_stepper)(void);
	void (*enable_a_stepper)(void);
  void (*enable_b_stepper)(void);
	// 电机禁止
	void (*disable_x_stepper)(void);
	void (*disable_y_stepper)(void);
	void (*disable_z_stepper)(void);
	void (*disable_e_stepper)(void);
	void (*disable_a_stepper)(void);
  void (*disable_b_stepper)(void);
	// 限位开关
	unsigned char (*get_x_axis_min_endstop)(void);//
	unsigned char (*get_x_axis_zero_endstop)(void);
	unsigned char (*get_x_axis_max_endstop)(void);
	unsigned char (*get_y_axis_min_endstop)(void);
	unsigned char (*get_y_axis_zero_endstop)(void);
	unsigned char (*get_y_axis_max_endstop)(void);
	unsigned char (*get_z_axis_min_endstop)(void);
	unsigned char (*get_z_axis_zero_endstop)(void);
	unsigned char (*get_z_axis_max_endstop)(void);
	// 输出脉冲
	void (*create_one_x_axis_step)(void);
	void (*create_one_y_axis_step)(void);
	void (*create_one_z_axis_step)(void);
	void (*create_one_e_axis_step)(void);
	void (*create_one_a_axis_step)(void);
  void (*create_one_b_axis_step)(void);
	// 运动方向
	void (*set_x_axis_dir)(unsigned char dir);
	void (*set_y_axis_dir)(unsigned char dir);
	void (*set_z_axis_dir)(unsigned char dir);
	void (*set_e_axis_dir)(unsigned char dir);
	void (*set_a_axis_dir)(unsigned char dir);
  void (*set_b_axis_dir)(unsigned char dir);
	// 运动块处理指令 状态指示
	void (*set_plan_line_num)(unsigned int num);
	
}stepper_func_typedef;


extern stepper_func_typedef steppers;

void endstops_hit_on_purpose(void);
void enable_endstops(unsigned char check);
void check_axes_activity(void);
void disable_all_stepper(void);
void enable_all_stepper(void);
void checkHitEndstops(void);
long st_get_position(uint8_t axis);
void timer_isr(void);
void plan_set_e_position(const float e);
void st_synchronize(void);

#endif
