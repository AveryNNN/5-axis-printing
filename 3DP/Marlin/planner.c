/*
  planner.c - buffers movement commands and manages the acceleration profile plan
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

/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */

/*  
 Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 
 s == speed, a == acceleration, t == time, d == distance
 
 Basic definitions:
 
 Speed[s_, a_, t_] := s + (a*t) 
 Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 
 Distance to reach a specific speed with a constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 
 Speed after a given distance of travel with constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 m -> Sqrt[2 a d + s^2]    
 
 DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 
 When to start braking (di) to reach a specified destionation speed (s2) after accelerating
 from initial speed s1 without ever stopping at a plateau:
 
 Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 
 IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 */

#include "marlin.h"
#include "planner.h"




//===========================================================================
//=============================public variables ============================
//===========================================================================
unsigned long axis_steps_per_sqr_second[NUM_AXIS];
float axis_steps_per_unit[NUM_AXIS] = {DEFAULT_X_AXIS_STEPS_PER_UNIT,DEFAULT_Y_AXIS_STEPS_PER_UNIT,DEFAULT_Z_AXIS_STEPS_PER_UNIT,DEFAULT_A_AXIS_STEPS_PER_UNIT,DEFAULT_B_AXIS_STEPS_PER_UNIT,DEFAULT_E_AXIS_STEPS_PER_UNIT};
long position[NUM_AXIS] 		= {0l,0l,0l,0l,0l,0l};//当前位置/step
static float previous_unit_vec[3]={0.0,0.0,0.0};
static float previous_speed[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Speed of previous path line segment
static float previous_nominal_speed; // Nominal speed of previous path line segment
unsigned long max_acceleration_units_per_sq_second[NUM_AXIS]={DEFAULT_X_AXIS_MAX_ACCELERATION,DEFAULT_Y_AXIS_MAX_ACCELERATION,DEFAULT_Z_AXIS_MAX_ACCELERATION,DEFAULT_A_AXIS_MAX_ACCELERATION, DEFAULT_B_AXIS_MAX_ACCELERATION,DEFAULT_E_AXIS_MAX_ACCELERATION};

int extrudemultiply 				= 100;//100->1.0 200->2.0
const unsigned int dropsegments = DEFAULT_DROP_SEGMENTS;//可忽略运动步长
float delta_mm[NUM_AXIS] 		= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float max_feedrate[NUM_AXIS]= {DEFAULT_X_AXIS_MAX_FEEDRATE, DEFAULT_Y_AXIS_MAX_FEEDRATE, DEFAULT_Z_AXIS_MAX_FEEDRATE, DEFAULT_A_AXIS_MAX_FEEDRATE,DEFAULT_B_AXIS_MAX_FEEDRATE,DEFAULT_E_AXIS_MAX_FEEDRATE};

float minimumfeedrate				= DEFAULT_MINIMUMFEEDRATE;
float mintravelfeedrate			= DEFAULT_MINIMUMFEEDRATE;

float retract_acceleration 	= DEFAULT_ACCELERATION; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
float acceleration 					= DEFAULT_RETRACT_ACCELERATION;
float max_xy_jerk  					= DEFAULT_XYJERK;
float max_z_jerk 	 					= DEFAULT_ZJERK;
float max_e_jerk 	 					= DEFAULT_EJERK;

//===========================================================================
//=================semi-private variables, used in inline  functions    =====
//===========================================================================
block_t block_buffer[BLOCK_BUFFER_SIZE];            	// A ring buffer for motion instfructions
unsigned char block_buffer_head=0;           // Index of the next block to be pushed
unsigned char block_buffer_tail=0;           // Index of the block to process now

extern stepper_func_typedef steppers;
//===========================================================================
//=============================private variables ============================
//===========================================================================
int8_t next_block_index(int8_t block_index)
{
  block_index++;
  if(block_index == BLOCK_BUFFER_SIZE) { 
    block_index = 0; 
  }
  return(block_index);
}

// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) 
{
  if (block_index == 0) 
	{ 
    block_index = BLOCK_BUFFER_SIZE; 
  }
  block_index--;
  return(block_index);
}

float max_allowable_speed(float acceleration, float target_velocity, float distance) 
{
  return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
}
// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
  if (acceleration!=0) 
	{
    return((target_rate*target_rate-initial_rate*initial_rate)/(2.0f*acceleration));
  }
  else 
	{
    return 0.0f;  // acceleration was 0, set acceleration distance to 0
  }
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) 
{
  if (acceleration!=0) 
	{
    return((2.0f*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/ (4.0f*acceleration) );
  }
  else 
	{
    return 0.0f;  // acceleration was 0, set intersection distance to 0
  }
}
// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor) 
{
  unsigned long initial_rate = ceil(block->nominal_rate*entry_factor); // (step/min)
  unsigned long final_rate = ceil(block->nominal_rate*exit_factor); // (step/min)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  if(initial_rate <120) 
	{
    initial_rate=120; 
  }
  if(final_rate < 120) 
	{
    final_rate=120;  
  }
	long acceleration = block->acceleration_st;
	int32_t accelerate_steps = ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps = floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));
	// Calculate the size of Plateau of Nominal Rate.
  int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;
	// Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) //说明加速不到设定的匀速运动速度
	{
    accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count));
    accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps,block->step_event_count);//(We can cast here to unsigned, because the above line ensures that we are above zero)
    plateau_steps = 0;
  }
	 CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section禁用总中断
  if(block->busy == false) 
	{ // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps+plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
  }
  CRITICAL_SECTION_END;
}
// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) 
{
  if(!current) 
	{ 
    return; 
  }

  if (next) 
	{
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) 
		{

      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) 
			{
        current->entry_speed = min( current->max_entry_speed,max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
      } 
      else 
			{
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;
    }
  } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
void planner_reverse_pass()
{
  uint8_t block_index = block_buffer_head;
  
  //Make a local copy of block_buffer_tail, because the interrupt can alter it
  CRITICAL_SECTION_START;
  unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END
  
  if(((block_buffer_head-tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) 
	{
    block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
    block_t *block[3] = {NULL, NULL, NULL};
    while(block_index != tail) 
		{ 
      block_index = prev_block_index(block_index); 
      block[2] = block[1];
      block[1] = block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}
// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) 
{
  if(!previous) 
	{ 
    return; 
  }
  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) 
	{
    if (previous->entry_speed < current->entry_speed) 
		{
      double entry_speed = min( current->entry_speed,max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters));
      // Check for junction speed change
      if (current->entry_speed != entry_speed) 
			{
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}
// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
void planner_forward_pass(void) 
{
  uint8_t block_index = block_buffer_tail;
  block_t *block[3] = {NULL, NULL, NULL };

  while(block_index != block_buffer_head) 
	{
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1],block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}
// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
void planner_recalculate_trapezoids(void) 
{
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;

  while(block_index != block_buffer_head) 
	{
    current = next;
    next = &block_buffer[block_index];
    if (current) 
		{
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) 
			{
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed,
        next->entry_speed/current->nominal_speed);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if(next != NULL) 
	{
    calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed,
    MINIMUM_PLANNER_SPEED/next->nominal_speed);
    next->recalculate_flag = false;
  }
}
// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor) 
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if 
//     a. The speed increase within one block would require faster accelleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than 
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate(void) 
{   
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init(void) 
{
  block_buffer_head = 0;
  block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
  previous_nominal_speed = 0.0;
}

void plan_buffer_line(const float x, const float y, const float z, const float e, const float a, const float b, float feed_rate, const uint8_t extruder)
{
	
	
	/**
	*	@step:(1)监测插补块环形队列是否还有存储空间，并持续等待到有空间为止，此处后续可使用操作系统线程进行阻塞挂起
	*/
  // Calculate the buffer head after we push this byte
  int16_t next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Rest here until there is room in the buffer.
  while(block_buffer_tail == next_buffer_head)
  {
    ;
  }
	/**
	*	@step:(2)计算插补块物理坐标对应的脉冲坐标，并计算当前坐标相对于已有指令的偏移脉冲坐标
	*/
	// The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  long target[NUM_AXIS];
  target[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
	//printf("X steps: %ld for distance: %f (steps/mm: %f)\n", target[X_AXIS]-position[X_AXIS], delta_mm[X_AXIS], (float)(target[X_AXIS]-position[X_AXIS])/delta_mm[X_AXIS]);
  target[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  target[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
  target[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);
  target[A_AXIS] = lround(a*axis_steps_per_unit[A_AXIS]); // 添加A轴
	//printf("target[A_AXIS]: %ld", target[A_AXIS]);
  target[B_AXIS] = lround(b*axis_steps_per_unit[B_AXIS]); // 添加B轴
	
	// Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];
	// Mark block as not busy (Not executed by the stepper interrupt)
	block->busy = false;
	// these equations follow the form of the dA and dB equations on http://www.corexy.com/theory.html
	block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
	block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
	block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
	block->steps_e = labs(target[E_AXIS]-position[E_AXIS]);
  block->steps_a = labs(target[A_AXIS]-position[A_AXIS]); // 添加A轴
	block->steps_b = labs(target[B_AXIS]-position[B_AXIS]); // 添加B轴
	block->steps_e *= extrudemultiply;
	block->steps_e /= 100;
	/**
	*	@step:(3)计算得到所有插补运动轴中脉冲数最大的轴，并以此为参考插补其他轴
	*/
	//
	block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, max(block->steps_e, max(block->steps_a, block->steps_b)))));
	//Bail if this is a zero-length block
	if(block->step_event_count <= dropsegments)//若该运动块运动距离可忽略，则直接忽略，下一条插补块自动纳入
  { 
    return; 
  }
	/**
	*	@step:(4)计算各运动轴当前目标位置与前一插补块目标位置的相对关系，并据此标记运动轴方向Bit
	*/
	// Compute direction bits for this block 
  block->direction_bits = 0;
	if (target[X_AXIS] < position[X_AXIS])
  {
    block->direction_bits |= (1<<X_AXIS); //X轴对应位为1
  }
  if (target[Y_AXIS] < position[Y_AXIS])
  {
    block->direction_bits |= (1<<Y_AXIS); 
  }
	 if (target[Z_AXIS] < position[Z_AXIS])
  {
    block->direction_bits |= (1<<Z_AXIS); 
  }
  if (target[E_AXIS] < position[E_AXIS])
  {
    block->direction_bits |= (1<<E_AXIS); 
  }
   if (target[A_AXIS] < position[A_AXIS]) { // 添加A轴
    block->direction_bits |= (1<<A_AXIS); 
  }
  if (target[B_AXIS] < position[B_AXIS]) { // 添加B轴
    block->direction_bits |= (1<<B_AXIS); 
  }
	/**
	*	@step:(5)指定当前插补块使用的挤出头，并依据各轴是否需要运动来使能各轴电机
	*/
	// Set active extruder
  block->active_extruder = extruder;
	if(block->steps_x != 0) steppers.enable_x_stepper();
	if(block->steps_y != 0) steppers.enable_y_stepper();
	if(block->steps_z != 0) steppers.enable_z_stepper();
	if(block->steps_e != 0) steppers.enable_e_stepper();
	if(block->steps_a != 0) steppers.enable_a_stepper(); // 添加A轴
	if(block->steps_b != 0) steppers.enable_b_stepper(); // 添加B轴
	if(block->steps_e == 0)
  {
    if(feed_rate<mintravelfeedrate) feed_rate=mintravelfeedrate;
  }
  else
  {
    if(feed_rate<minimumfeedrate) feed_rate=minimumfeedrate;
  } 
	/**
	*	@step:(6)计算各轴物理运动量[mm]
	*/
	delta_mm[X_AXIS] = (target[X_AXIS]-position[X_AXIS])/axis_steps_per_unit[X_AXIS];
  delta_mm[Y_AXIS] = (target[Y_AXIS]-position[Y_AXIS])/axis_steps_per_unit[Y_AXIS];
	delta_mm[Z_AXIS] = (target[Z_AXIS]-position[Z_AXIS])/axis_steps_per_unit[Z_AXIS];
  delta_mm[E_AXIS] = ((target[E_AXIS]-position[E_AXIS])/axis_steps_per_unit[E_AXIS])*extrudemultiply/100.0f;
  delta_mm[A_AXIS] = (target[A_AXIS]-position[A_AXIS])/axis_steps_per_unit[A_AXIS]; // 添加A轴
	//printf("距离: %f\n", delta_mm[A_AXIS]);
  delta_mm[B_AXIS] = (target[B_AXIS]-position[B_AXIS])/axis_steps_per_unit[B_AXIS]; // 添加B轴
  
	/**
	*	@step:(7)检测XYZ三个运动轴是否需要运动,并据此计算运动所需时间
	*/
	if(block->steps_x <=dropsegments && block->steps_y <=dropsegments && block->steps_z <=dropsegments)
  {
    block->millimeters = fabs(delta_mm[E_AXIS]);//只需要运动E
  }
	else//需要运动则计算坐标距离
	{
		 block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
	}
	
	
	 
  // 计算 inverse_millimeters
  float inverse_millimeters = 1.0f / block->millimeters; 
	
	// 添加在运动规划时需要的 current_speed
  float current_speed[NUM_AXIS]; 
	
	
	
	 // **新增代码**: 计算速度百分比
  float speed_percentage = feed_rate / 100.0f; // F值作为百分比
  
  // 限制百分比范围
  if (speed_percentage > 1.0f) speed_percentage = 1.0f;
  if (speed_percentage < 0.05f) speed_percentage = 0.05f; // 最低5%速度
  
  block->speed_percentage = speed_percentage;
  
  // 计算各轴完成运动所需时间
  float time_x = (block->steps_x > 0) ? 
                fabs(delta_mm[X_AXIS]) / (max_feedrate[X_AXIS] * speed_percentage / 60.0f) : 0;
  float time_y = (block->steps_y > 0) ? 
                fabs(delta_mm[Y_AXIS]) / (max_feedrate[Y_AXIS] * speed_percentage / 60.0f) : 0;
  float time_z = (block->steps_z > 0) ? 
                fabs(delta_mm[Z_AXIS]) / (max_feedrate[Z_AXIS] * speed_percentage / 60.0f) : 0;
  float time_e = (block->steps_e > 0) ? 
                fabs(delta_mm[E_AXIS]) / (max_feedrate[E_AXIS] * speed_percentage / 60.0f) : 0;
  float time_a = (block->steps_a > 0) ? 
                fabs(delta_mm[A_AXIS]) / (max_feedrate[A_AXIS] * speed_percentage / 60.0f) : 0;
  float time_b = (block->steps_b > 0) ? 
                fabs(delta_mm[B_AXIS]) / (max_feedrate[B_AXIS] * speed_percentage / 60.0f) : 0;
  
  // 选择最长时间作为运动时间
  float max_time = max(time_x, max(time_y, max(time_z, max(time_e, max(time_a, time_b)))));
  
  // 如果所有时间都为0，设置一个最小值
  if (max_time < 0.001f) max_time = 0.001f;
  
  // 计算标称速率
  block->nominal_rate = ceil(block->step_event_count / max_time); // steps/s
  block->nominal_speed = block->millimeters / max_time; // mm/s
	 
  // 计算各轴速度
  for(int i=0; i < NUM_AXIS; i++) {
    current_speed[i] = delta_mm[i] / max_time; // mm/s
  }
  
			// 方向处理 - 确保方向位正确设置
		block->direction_bits = 0;
		if (target[X_AXIS] < position[X_AXIS]) block->direction_bits |= (1<<X_AXIS);
		if (target[Y_AXIS] < position[Y_AXIS]) block->direction_bits |= (1<<Y_AXIS);
		if (target[Z_AXIS] < position[Z_AXIS]) block->direction_bits |= (1<<Z_AXIS);
		if (target[E_AXIS] < position[E_AXIS]) block->direction_bits |= (1<<E_AXIS);
		if (target[A_AXIS] < position[A_AXIS]) block->direction_bits |= (1<<A_AXIS);
		if (target[B_AXIS] < position[B_AXIS]) block->direction_bits |= (1<<B_AXIS);
  // 计算 moves_queued
  int16_t moves_queued = (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1); 

	// 检查速度限制
  float speed_factor = 1.0;
  for (int i = 0; i < NUM_AXIS; i++) {
    float axis_speed = fabs(delta_mm[i]) / max_time; // mm/s
    if (axis_speed > max_feedrate[i]) {
      speed_factor = min(speed_factor, max_feedrate[i] / axis_speed);
    }
  }
  
  // 应用速度因子
  if (speed_factor < 1.0f) {
    block->nominal_rate *= speed_factor;
    block->nominal_speed *= speed_factor;
  }
  
  // 计算加速度
  float steps_per_mm = block->step_event_count / block->millimeters;
  if (block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0) {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm);
  } else {
    block->acceleration_st = ceil(acceleration * steps_per_mm);
    // 限制各轴加速度
    if (((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
    if (((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
    if (((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
    if (((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count) > axis_steps_per_sqr_second[Z_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
  }
  
  block->acceleration = block->acceleration_st / steps_per_mm;
  block->acceleration_rate = (long)((float)block->acceleration_st * 1.0f);
	
	
	/**
	*	@step:(12)进行速度前瞻准备，依据各轴的最大急停速度，确定一个满足各轴约束的最大块连接速度
	*/
	// Compute path unit vector
  double unit_vec[3];

  unit_vec[X_AXIS] = delta_mm[X_AXIS]*inverse_millimeters;//计算运动的单位向量
  unit_vec[Y_AXIS] = delta_mm[Y_AXIS]*inverse_millimeters;
  unit_vec[Z_AXIS] = delta_mm[Z_AXIS]*inverse_millimeters;

  // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
  // Let a circle be tangent to both previous and current path line segments, where the junction
  // deviation is defined as the distance from the junction to the closest edge of the circle,
  // colinear with the circle center. The circular segment joining the two paths represents the
  // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
  // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
  // path width or max_jerk in the previous grbl version. This approach does not actually deviate
  // from path, but used as a robust way to compute cornering speeds, as it takes into account the
  // nonlinearities of both the junction angle and junction velocity.
  float vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

  // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
  if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0f)) 
	{
    // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
    // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
		//previous_unit_vec[]之前路径的单位向量
    float cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
											- previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
											- previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS];

    // Skip and use default max junction speed for 0 degree acute junction.
    if (cos_theta < 0.95f) 
		{
      vmax_junction = min(previous_nominal_speed,block->nominal_speed);//180度
      // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
      if (cos_theta > -0.95f) //角度对速度的影响不能忽略
			{
        // Compute maximum junction velocity based on maximum acceleration and junction deviation
        float sin_theta_d2 = sqrt(0.5f*(1.0f-cos_theta)); // Trig half angle identity. Always positive.
        vmax_junction = min(vmax_junction,
        sqrt(block->acceleration * JUNCTION_DEVIATION * sin_theta_d2/(1.0f-sin_theta_d2)) );//圆弧过渡算法
      }
    }
  }
	
	//调试区
	
printf("axis_steps_per_unit[B_AXIS]: %f\n", axis_steps_per_unit[A_AXIS]);
printf("输入角度 b: %f\n", a);
printf("target[B_AXIS]: %ld\n", target[B_AXIS]);
printf("当前位置: %ld\n", position[B_AXIS]);
printf("delta_mm[B_AXIS]: %f\n", delta_mm[B_AXIS]);
	
//	// Start with a safe speed
//  float vmax_junction = max_xy_jerk/2; //块连接点最大速度初始化为xy急停速度[max_xy_jerk]的一半
//  float vmax_junction_factor = 1.0; 
//	if(fabs(current_speed[Z_AXIS]) > max_z_jerk/2) //current_speed是平均速度
//	{
//		//Z轴当前速度大于Z轴最大允许急停速度的一半，则max_z_jerk/2与max_xy_jerk/2相比取较小
//    vmax_junction = min(vmax_junction, max_z_jerk/2);//Z轴
//	}
//	if(fabs(current_speed[E_AXIS]) > max_e_jerk/2) //E轴 同Z
//	{
//    vmax_junction = min(vmax_junction, max_e_jerk/2);
//	}
//	vmax_junction = min(vmax_junction, block->nominal_speed);//取一个满足所有约束的最大速度，速度值可能会减小
	/**
	*	@step:(13)进行速度前瞻准备，依据各轴指令运速度进行幅值约束，与步骤12不同的是：步骤12使用的是各轴最大容许速度的一半，而步骤13是各轴最大容许速度
	*/
  float safe_speed = MINIMUM_PLANNER_SPEED;//先将满足各轴最大容许约束的速度作为一个安全速度
	float vmax_junction_factor = 1.0; 
	if ((moves_queued > 1) && (previous_nominal_speed > 0.0001f))
	{
		//计算在两条插补线连接点处的速度激变幅值，主要是XY平面内
    float jerk = sqrt(pow((current_speed[X_AXIS]-previous_speed[X_AXIS]), 2)+pow((current_speed[Y_AXIS]-previous_speed[Y_AXIS]), 2));
    
		vmax_junction = block->nominal_speed;//初值
		
    if (jerk > max_xy_jerk) //若连接处激变速度大于xy急停速度[max_xy_jerk]
		{
      vmax_junction_factor = (max_xy_jerk/jerk);//需要进行速度调整,vmax_junction_factor<1.0
    } 
    if(fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) //若Z方向的速度激变大于最大允许的急停速度
		{
      vmax_junction_factor= min(vmax_junction_factor, (max_z_jerk/fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
    } 
    if(fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk)//同Z轴
		{
      vmax_junction_factor = min(vmax_junction_factor, (max_e_jerk/fabs(current_speed[E_AXIS] - previous_speed[E_AXIS])));
    } 
		//限制速度为前一运动块的设定速度
    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
  }
  block->max_entry_speed = vmax_junction;//确定当前块的最大进入速度
	/**
	*	@step:(14)当前运动块是最新进入队列的，所以认为先认为其执行完成速度应该为最小的规划速度，结合加速度计算当前块目前(会结合后续插入的运动块重新规划)允许的最大进入速度
	*/ 
	// Initialize block entry speed. Compute based on deceleration to user defined MINIMUM_PLANNER_SPEED.
  double v_allowable = max_allowable_speed(-block->acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);
	if (block->nominal_speed <= v_allowable) 
	{ 
    block->nominal_length_flag = true; 
  }
  else 
	{ 
    block->nominal_length_flag = false; 
  }
  block->recalculate_flag = true; // Always calculate trapezoid for new block
	/**
	*	@step:(15)当前速度幅值给之前速度，以便新的运动块插入进行速度规划
	*/ 
	// Update previous path unit_vector and nominal speed
	for(unsigned char i=0; i < NUM_AXIS; i++)
	{
		previous_speed[i] = current_speed[i];
	}
	for(unsigned char i=0; i < 3; i++)
	{
		previous_unit_vec[i] = unit_vec[i];
	}
	previous_nominal_speed = block->nominal_speed;
	/**
	*	@step:(15)依据当前块的进入因子、退出因子(因为在梯形计算函数内部通过因子乘以脉冲速率来计算入口脉冲率和出口脉冲率)计算梯形加减速参数
	*/ 
	// 计算梯形加减速
	calculate_trapezoid_for_block(block, block->entry_speed/block->nominal_speed, safe_speed/block->nominal_speed);
	// Move buffer head
	/**
	*	@step:(16)更新插补环形队列头部指针，并更新块运位置
	*/
  block_buffer_head = next_buffer_head;
	for(int8_t i=0; i < NUM_AXIS; i++) {
    position[i] = target[i];
  }
	//基于当前新插补运动块的速度特征，重新规划环形队列里插补块的速度
	planner_recalculate();
	
	
}
// Calculate the steps/s^2 acceleration rates, based on the mm/s^s
void reset_acceleration_rates(void)
{	
	int8_t i;
	for(i=0; i < NUM_AXIS; i++)
  {
		axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
  }
}

block_t *plan_get_current_block() 
{
  if(block_buffer_head == block_buffer_tail) 
	{
    return(NULL); 
  }
  block_t *block = &block_buffer[block_buffer_tail];
  block->busy = true;
  return(block);
}

void plan_discard_current_block(void)  
{  
  if(block_buffer_head == block_buffer_tail)
	{
    return;
  }
	else
	{
		block_buffer_tail = (block_buffer_tail + 1) & (BLOCK_BUFFER_SIZE - 1); 
	}		
}

// Gets the current block. Returns NULL if buffer empty
uint8_t blocks_queued(void) 
{
  if (block_buffer_head == block_buffer_tail)
	{ 
    return false; 
  }
  else
    return true;
}

void plan_set_position(const float x, const float y, const float z, const float e, const float a, const float b)
{
  position[X_AXIS] = round(x*axis_steps_per_unit[X_AXIS]);
  position[Y_AXIS] = round(y*axis_steps_per_unit[Y_AXIS]);
  position[Z_AXIS] = round(z*axis_steps_per_unit[Z_AXIS]);     
  position[E_AXIS] = round(e*axis_steps_per_unit[E_AXIS]);  
  position[A_AXIS] = round(a*axis_steps_per_unit[A_AXIS]); // 添加A轴
  position[B_AXIS] = round(b*axis_steps_per_unit[B_AXIS]); // 添加B轴
  st_set_position(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], position[E_AXIS], position[A_AXIS], position[B_AXIS]);
previous_nominal_speed = 0.0f; // 重置规划器连接速度
  previous_speed[0] = 0.0f;
  previous_speed[1] = 0.0f;
  previous_speed[2] = 0.0f;
  previous_speed[3] = 0.0f;
  previous_speed[4] = 0.0f; // 添加A轴
  previous_speed[5] = 0.0f; // 添加B轴
}

void plan_set_e_position(const float e)
{
	  position[E_AXIS] = round(e*axis_steps_per_unit[E_AXIS]); 
  previous_speed[E_AXIS] = 0.0f;
}

void check_axes_activity(void)
{
  unsigned char x_active = 0;
  unsigned char y_active = 0;  
  unsigned char z_active = 0;
  unsigned char e_active = 0;
  unsigned char a_active = 0; // 添加A轴
  unsigned char b_active = 0; // 添加B轴
  block_t *block;

  if(block_buffer_tail != block_buffer_head)
  {
    uint8_t block_index = block_buffer_tail;
		while(block_index != block_buffer_head)
    {
      block = &block_buffer[block_index];
      if(block->steps_x != 0) x_active++;
      if(block->steps_y != 0) y_active++;
      if(block->steps_z != 0) z_active++;
      if(block->steps_e != 0) e_active++;
		if(block->steps_a != 0) a_active++; // 添加A轴
      if(block->steps_b != 0) b_active++; // 添加B轴
      block_index = (block_index+1) & (BLOCK_BUFFER_SIZE - 1);
    }
  }
  if((DISABLE_X) && (x_active == 0)) steppers.disable_x_stepper(); 
  if((DISABLE_Y) && (y_active == 0)) steppers.disable_y_stepper(); 
  if((DISABLE_Z) && (z_active == 0)) steppers.disable_z_stepper(); 
  if((DISABLE_E) && (e_active == 0)) steppers.disable_e_stepper(); 
  if((DISABLE_A) && (a_active == 0)) steppers.disable_a_stepper(); 
  if((DISABLE_B) && (b_active == 0)) steppers.disable_b_stepper();
}