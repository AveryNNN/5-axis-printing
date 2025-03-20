/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
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

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "Marlin.h"
#include "stepper.h"
#include "gpio_config.h"

#define MAX_STEP_FREQUENCY 20000 // Max step frequency:最大允许的脉冲数[step/sec]
//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block = NULL;  // A pointer to the block currently being traced
static char step_loops;
//步进电机
stepper_func_typedef steppers;
//
static long acceleration_time, deceleration_time;
static unsigned short timer_nominal;
static unsigned short acc_step_rate; // needed for deccelaration start point
static unsigned short step_loops_nominal;

static unsigned char out_bits=0;        // The next stepping-bits to be output
static long counter_x=0,       // Counter variables for the bresenham line tracer
            counter_y=0,
            counter_z=0,
            counter_e=0,
			counter_a=0, 
			counter_b=0;
volatile static unsigned long step_events_completed=0; // The number of step events executed in the current block
volatile signed char count_direction[NUM_AXIS] = {  1, 1, 1, 1, 1, 1};
volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0, 0, 0};

// 限位相关
unsigned char check_endstops = true;
volatile long endstops_trigsteps[3]={0,0,0};
static volatile uint8_t endstop_x_hit=false;
static volatile uint8_t endstop_y_hit=false;
static volatile uint8_t endstop_z_hit=false;

static uint8_t old_x_min_endstop=true;
static uint8_t old_x_max_endstop=true;
static uint8_t old_y_min_endstop=true;
static uint8_t old_y_max_endstop=true;
static uint8_t old_z_min_endstop=true;
static uint8_t old_z_max_endstop=true;

extern float axis_steps_per_unit[NUM_AXIS];
//
void checkHitEndstops(void)
{
	if( endstop_x_hit || endstop_y_hit || endstop_z_hit) 
	{
		SERIAL_ECHO_START;
		printf(MSG_ENDSTOPS_HIT);
		if(endstop_x_hit) 
		{
			printf(" X:%f",(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
		}
		if(endstop_y_hit) 
		{
			printf(" Y:%f",(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
		}
		if(endstop_z_hit)
		{
			printf(" Z:%f",(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
		}
		printf("\n");
		endstop_x_hit=false;
		endstop_y_hit=false;
		endstop_z_hit=false;
	}
}

// step_rate:[steps/sec]
unsigned short calc_timer(unsigned short step_rate) 
{
  unsigned short timer;
	//上限约束
  if(step_rate > MAX_STEP_FREQUENCY) 
	{
		step_rate = MAX_STEP_FREQUENCY;
  }
  if(step_rate > 20000) //高于20kHz
	{
    step_rate  = (step_rate >> 2)&0x3fff;//缩小4倍脉冲率，单次输出4个脉冲
    step_loops = 4;
  }
  else if(step_rate > 10000)
	{
    step_rate = (step_rate >> 1)&0x7fff;//缩小2倍脉冲率，单次输出2个脉冲
    step_loops = 2;
  }
  else//不缩小，单次输出1个脉冲
	{
    step_loops = 1;
  }
	//下限约束
  if(step_rate < 32) //不能太小
	{
		step_rate = 32;
	}
	//依据频率计算对应的定时器计数值
  timer = TIMER_FREQ/step_rate - 1;
	//定时器计数中断速度
  if(timer < 100)
	{ 
		timer = 100; 
		printf(MSG_STEPPER_TO_HIGH); 
		printf("%d",step_rate); 
	}//(20kHz this should never happen)
  return timer;
}

void trapezoid_generator_reset(void)
{
  deceleration_time = 0;
  // step_rate to timer interval
  timer_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate 		 = current_block->initial_rate;
  acceleration_time  = calc_timer(acc_step_rate);
  
  steppers.set_timer_autoreload(acceleration_time - 1);
}
//实现1个流水灯 用于表示查不块正在运动
static unsigned int led_count=0;
void timer_isr(void)
{
	if(current_block == NULL)
	{
		// 获取一个新的插补运动块
		current_block = plan_get_current_block();//获取新的插补运动块
		if(current_block != NULL)
		{
			steppers.set_plan_line_num(led_count++);
			// 成功获取得到块
			current_block->busy = true;
      trapezoid_generator_reset();
			counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0;
			if(current_block->steps_z > 0) 
			{
          steppers.enable_z_stepper();
          steppers.set_timer_autoreload(2000-1); //1ms wait
          return;
      }
		}
		else// 当前插补运动块环形缓存中没有待插补指令
		{
			steppers.set_timer_autoreload(2000-1);
		}
	}
	// 若当前插补块正在插补
	if(current_block != NULL)
	{
		// Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;
		// Set direction en check limit switches
		// X Axis
		if((out_bits & (1<<X_AXIS)) != 0)
		{
			steppers.set_x_axis_dir(NEGATIVE_X_DIR);
			count_direction[X_AXIS]=-1;
		}
		else
		{
			steppers.set_x_axis_dir(POSITIVE_X_DIR);
			count_direction[X_AXIS]=1;
		}
		// Y Axis
		if((out_bits & (1<<Y_AXIS)) != 0)
		{
			steppers.set_y_axis_dir(NEGATIVE_Y_DIR);
			count_direction[Y_AXIS]=-1;
		}
		else
		{
			steppers.set_y_axis_dir(POSITIVE_Y_DIR);
			count_direction[Y_AXIS]=1;
		}
		// Z Axis
		if((out_bits & (1<<Z_AXIS)) != 0)
		{
			steppers.set_z_axis_dir(NEGATIVE_Z_DIR);
			count_direction[Z_AXIS]=-1;
		}
		else
		{
			steppers.set_z_axis_dir(POSITIVE_Z_DIR);
			count_direction[Z_AXIS]=1;
		}
		// E Axis
		if((out_bits & (1<<E_AXIS)) != 0)
		{
			steppers.set_e_axis_dir(NEGATIVE_E_DIR);
			count_direction[E_AXIS]=-1;
		}
		else
		{
			steppers.set_e_axis_dir(POSITIVE_E_DIR);
			count_direction[Z_AXIS]=1;
		}
		// 设置A轴方向
    if((out_bits & (1<<A_AXIS)) != 0) {
      steppers.set_a_axis_dir(NEGATIVE_A_DIR);
      count_direction[A_AXIS]=-1;
    } else {
      steppers.set_a_axis_dir(POSITIVE_A_DIR);
      count_direction[A_AXIS]=1;
    }
    
    // 设置B轴方向
    if((out_bits & (1<<B_AXIS)) != 0) {
      steppers.set_b_axis_dir(NEGATIVE_B_DIR);
      count_direction[B_AXIS]=-1;
    } else {
      steppers.set_b_axis_dir(POSITIVE_B_DIR);
      count_direction[B_AXIS]=1;
    }
		// 检测限位
		// -X
		if ((out_bits & (1<<X_AXIS)) != 0)  // stepping along -X axis
		{  
			if(check_endstops)
			{
				uint8_t x_min_endstop = (steppers.get_x_axis_min_endstop() & 0x01);
				if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) 
				{
					endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
					endstop_x_hit=true;
					step_events_completed = current_block->step_event_count;
				}
				old_x_min_endstop = x_min_endstop;
			}
		}
		// +X
		else
		{
			if(check_endstops)
			{
				uint8_t x_max_endstop = (steppers.get_x_axis_max_endstop() & 0x01);
				if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0))
				{
					endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
					endstop_x_hit=true;
					step_events_completed = current_block->step_event_count;
				}
				old_x_max_endstop = x_max_endstop;
			}
		}
		// -Y
		if ((out_bits & (1<<Y_AXIS)) != 0)  // stepping along -Y axis
		{
			if(check_endstops)
			{			
				uint8_t y_min_endstop = (steppers.get_y_axis_min_endstop() & 0x01);
				if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) 
				{
					endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
					endstop_y_hit=true;
					step_events_completed = current_block->step_event_count;
				}
				old_y_min_endstop = y_min_endstop;
			}
		}
		// +Y
		else
		{
			if(check_endstops)
			{
				uint8_t y_max_endstop = (steppers.get_y_axis_max_endstop() & 0x01);
				if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0))
				{
					endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
					endstop_y_hit=true;
					step_events_completed = current_block->step_event_count;
				}
				old_y_max_endstop = y_max_endstop;
			}
		}
		// -Z
		if ((out_bits & (1<<Z_AXIS)) != 0)  // stepping along -Z axis
		{ 
			if(check_endstops)
			{
				uint8_t z_min_endstop = (steppers.get_z_axis_min_endstop() & 0x01);
				if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) 
				{
					endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
					endstop_z_hit=true;
					step_events_completed = current_block->step_event_count;
				}
				old_z_min_endstop = z_min_endstop;
			}
		}
		// +Z
		else
		{
			if(check_endstops)
			{
				uint8_t z_max_endstop = (steppers.get_z_axis_max_endstop() & 0x01);
				if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0))
				{
					endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
					endstop_z_hit=true;
					step_events_completed = current_block->step_event_count;
				}
				old_z_max_endstop = z_max_endstop;
			}
		}
		//输出运动脉冲
		
		uint8_t i;
    for(i=0; i < step_loops; i++) 
		{
			// X轴运动
			counter_x += current_block->steps_x;
			if (counter_x > 0) 
			{
				steppers.create_one_x_axis_step();
				counter_x -= current_block->step_event_count;
        count_position[X_AXIS]+=count_direction[X_AXIS];
			}
			// Y轴运动
			counter_y += current_block->steps_y;
			if (counter_y > 0) 
			{
				steppers.create_one_y_axis_step();
				counter_y -= current_block->step_event_count;
        count_position[Y_AXIS]+=count_direction[Y_AXIS];
			}
			// Z轴运动
			counter_z += current_block->steps_z;
			if(counter_z > 0)//说明需要输出一个运动脉冲
			{
				steppers.create_one_z_axis_step();
				counter_z -= current_block->step_event_count;
        count_position[Z_AXIS]+=count_direction[Z_AXIS];
			}
			// E轴运动
			if(current_block->steps_e > 0)
			{
				PRESS_STEP_PIN = 1;
			}	
			else
			{
				PRESS_STEP_PIN = 0;
			}
			counter_e += current_block->steps_e;
      if (counter_e > 0) 
			{
        steppers.create_one_e_axis_step();
        counter_e -= current_block->step_event_count;
        count_position[E_AXIS]+=count_direction[E_AXIS];
      }
			 counter_a += current_block->steps_a;
      if (counter_a > 0) {
        steppers.create_one_a_axis_step();
        counter_a -= current_block->step_event_count;
        count_position[A_AXIS] += count_direction[A_AXIS];
      }
      
      // B轴运动
      counter_b += current_block->steps_b;
      if (counter_b > 0) {
        steppers.create_one_b_axis_step();
        counter_b -= current_block->step_event_count;
        count_position[B_AXIS] += count_direction[B_AXIS];
      }
      
      step_events_completed += 1;
      if(step_events_completed >= current_block->step_event_count) break;
    }
			
		// 计算加减速
		unsigned short timer;
    unsigned short step_rate;
		if (step_events_completed <= (unsigned long int)current_block->accelerate_until) //加速
		{ 
			// 计算速度增量
			float delta_t  = (float)acceleration_time/TIMER_FREQ;
			acc_step_rate  = (long)(delta_t * current_block->acceleration_rate + 0.5f);
      acc_step_rate += current_block->initial_rate;
      
      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
			{
        acc_step_rate = current_block->nominal_rate;
			}
      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
			//printf("1:%ld\r\n",timer);//
			steppers.set_timer_autoreload(timer-1);
      acceleration_time += timer;
		}
		else if (step_events_completed > (unsigned long int)current_block->decelerate_after) //减速
		{
			float delta_t  = (float)deceleration_time/TIMER_FREQ;
			step_rate  = (long)(delta_t * current_block->acceleration_rate + 0.5f);
      if(step_rate > acc_step_rate) 
			{ // Check step_rate stays positive
				// 保证速度不能反向
	      step_rate = current_block->final_rate;//如果相对于块初速度减小量大于增加量
	    }
	    else
			{
	      step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
	    }
			// lower limit
	    if(step_rate < current_block->final_rate)//限制速度最小减到0
			{
	      step_rate = current_block->final_rate;
			}
	    // step_rate to timer interval
	    timer = calc_timer(step_rate);
			// printf("2:%ld\r\n",timer);
			steppers.set_timer_autoreload(timer-1);
	    deceleration_time += timer;
	  }
		else //匀速
		{
			// printf("3:%ld\r\n",TIME3_nominal);
		  steppers.set_timer_autoreload(timer_nominal-1);;
		  // ensure we're running at the correct step rate, even if we just came off an acceleration
		  step_loops = step_loops_nominal;
		}
	}
	// 检测当前插补运动块是否插补完成
	if(step_events_completed >= current_block->step_event_count) 
		{
			PRESS_STEP_PIN = 0;
      current_block = NULL;
      plan_discard_current_block();
    } 
}

void st_set_position(const long x, const long y, const long z, const long e, const long a, const long b)
{
  CRITICAL_SECTION_START;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
  count_position[A_AXIS] = a; // 添加A轴
  count_position[B_AXIS] = b; // 添加B轴
  CRITICAL_SECTION_END;
}

void endstops_hit_on_purpose(void)
{
  endstop_x_hit=false;
  endstop_y_hit=false;
  endstop_z_hit=false;
}

void enable_endstops(unsigned char check)
{
  check_endstops = check;
}

void disable_all_stepper(void)
{
	steppers.disable_x_stepper();
	steppers.disable_y_stepper();
	steppers.disable_z_stepper();
	steppers.disable_e_stepper();
	  steppers.enable_a_stepper(); // 添加A轴
  steppers.enable_b_stepper(); // 添加B轴
}

void enable_all_stepper(void)
{
	steppers.enable_x_stepper();
	steppers.enable_y_stepper();
	steppers.enable_z_stepper();
	steppers.enable_e_stepper();
	  steppers.enable_a_stepper(); // 添加A轴
  steppers.enable_b_stepper(); // 添加B轴
}

long st_get_position(uint8_t axis)
{
  long count_pos;
  CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

void st_synchronize(void)
{
    while( blocks_queued()|(current_block!=NULL)) 
	{
		
  }
}


