/*
  motion_control.c - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon

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

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"

#define M_PI 3.14159265358979323846
// Arc interpretation settings:
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };


//void clamp_to_software_endstops(float target[3])
//{
//  if (min_software_endstops) 
//	{
//    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
//    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
//    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
//  }
//  if (max_software_endstops) 
//	{
//    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
//    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
//    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
//  }
//}
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each 
// segment is configured in settings.mm_per_arc_segment.  
void mc_arc(float *position, float *target, float *offset, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, float feed_rate, float radius, uint8_t isclockwise, uint8_t extruder)
{      
  //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
  //   plan_set_acceleration_manager_enabled(false);
	// disable acceleration management for the duration of the arc
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float linear_travel = target[axis_linear] - position[axis_linear];
  float extruder_travel = target[E_AXIS] - position[E_AXIS];
  float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;
    // 为A轴和B轴添加线性插值
  float a_per_segment = 0;
  float b_per_segment = 0;
	
	
	
  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (angular_travel < 0) 
	{ 
		angular_travel += 2*M_PI; 
	}
  if (isclockwise) 
	{
		angular_travel -= 2*M_PI; 
	}
  //20141002:full circle for G03 did not work, e.g. G03 X80 Y80 I20 J0 F2000 is giving an Angle of zero so head is not moving
  //to compensate when start pos = target pos && angle is zero -> angle = 2Pi
  if(position[axis_0] == target[axis_0] && position[axis_1] == target[axis_1] && angular_travel == 0)
  {
	  angular_travel += 2*M_PI;
  }
  //end fix G03
  
  float millimeters_of_travel = hypot(angular_travel*radius, fabs(linear_travel));
  if (millimeters_of_travel < 0.001f) 
	{ 
		return; 
	}
  uint16_t segments = floor(millimeters_of_travel/MM_PER_ARC_SEGMENT);
  if(segments == 0) segments = 1;
  
	 // 计算A轴和B轴每段的增量
  if (NUM_AXIS > 4) {
    a_per_segment = (target[A_AXIS] - position[A_AXIS]) / segments;
  }
  
  if (NUM_AXIS > 5) {
    b_per_segment = (target[B_AXIS] - position[B_AXIS]) / segments;
  }
  
  /*  
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
    // all segments.
    if (invert_feed_rate) { feed_rate *= segments; }
  */
  float theta_per_segment 		= angular_travel/segments;
  float linear_per_segment 		= linear_travel/segments;
  float extruder_per_segment 	= extruder_travel/segments;
  
  /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
         r_T = [cos(phi) -sin(phi);
                sin(phi)  cos(phi] * r ;
     
     For arc generation, the center of the circle is the axis of rotation and the radius vector is 
     defined from the circle center to the initial position. Each line segment is formed by successive
     vector rotations. This requires only two cos() and sin() computations to form the rotation
     matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     all double numbers are single precision on the Arduino. (True double precision will not have
     round off issues for CNC applications.) Single precision error can accumulate to be greater than
     tool precision in some cases. Therefore, arc path correction is implemented. 

     Small angle approximation may be used to reduce computation overhead further. This approximation
     holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
     theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for 
     numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     issue for CNC machines with the single precision Arduino calculations.
     
     This approximation also allows mc_arc to immediately insert a line segment into the planner 
     without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead. 
     This is important when there are successive arc motions. 
  */
  // Vector rotation matrix values
  float cos_T = 1-0.5f*theta_per_segment*theta_per_segment; // Small angle approximation
  float sin_T = theta_per_segment;
  
  float arc_target[NUM_AXIS];
  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  uint16_t i;
  int8_t count = 0;
  


  // Initialize the linear axis
  arc_target[axis_linear] = position[axis_linear];
  
  // Initialize the extruder axis
  arc_target[E_AXIS] = position[E_AXIS];

  if (NUM_AXIS > 4) arc_target[A_AXIS] = position[A_AXIS];
  if (NUM_AXIS > 5) arc_target[B_AXIS] = position[B_AXIS];

  for (i = 1; i<segments; i++) 
	{ // Increment (segments-1)
    if (count < N_ARC_CORRECTION) 
		{
      // Apply vector rotation matrix 
      r_axisi = r_axis0*sin_T + r_axis1*cos_T;
      r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
      r_axis1 = r_axisi;
      count++;
    } 
		else 
		{
      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti = cos(i*theta_per_segment);
      sin_Ti = sin(i*theta_per_segment);
      r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
      r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
      count = 0;
    }

    // Update arc_target location
    arc_target[axis_0] 			 = center_axis0 + r_axis0;
    arc_target[axis_1]			 = center_axis1 + r_axis1;
    arc_target[axis_linear] += linear_per_segment;
    arc_target[E_AXIS]		  += extruder_per_segment;

	   if (NUM_AXIS > 4) arc_target[A_AXIS] += a_per_segment;
    if (NUM_AXIS > 5) arc_target[B_AXIS] += b_per_segment;
	
    clamp_to_software_endstops(arc_target);
    plan_buffer_line(arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], 
                     arc_target[E_AXIS], 
                     (NUM_AXIS > 4) ? arc_target[A_AXIS] : 0,
                     (NUM_AXIS > 5) ? arc_target[B_AXIS] : 0,
                     feed_rate, extruder);
    
	
  }
  // Ensure last segment arrives at target location.
  plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], 
                  target[E_AXIS], 
                  (NUM_AXIS > 4) ? target[A_AXIS] : 0, 
                  (NUM_AXIS > 5) ? target[B_AXIS] : 0,
                  feed_rate, extruder);
	
}

