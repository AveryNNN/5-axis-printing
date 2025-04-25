#ifndef configuration_h
#define configuration_h

//======================================================================================//
#define BAUDRATE 115200

#define STEPPER_ENABLE_VOL  0
#define STEPPER_DISABLE_VOL	1

#define	ENDSTOP_HIT_VOL			1

//======================================================================================//
//2.插补运动块
//======================================================================================//
// The number of linear motions that can be in the plan at any give time
#define BLOCK_BUFFER_SIZE 16u			//注意 只能取2^n 如：2 4 8 16 32
// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                5.0f    // (mm/sec)
#define DEFAULT_ZJERK                 5.0f     // (mm/sec)
#define DEFAULT_EJERK                 40.0f    	// (mm/sec)
#define DEFAULT_AJERK                   5.0f
#define DEFAULT_BJERK                   5.0f


#define MINIMUM_PLANNER_SPEED					0.05f
#define JUNCTION_DEVIATION						0.02f    //junction_deviation (mm/sec)

#define DEFAULT_MINIMUMFEEDRATE       0.0f     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0f

#define DEFAULT_ACCELERATION					30			//[mm/sec^2]
#define DEFAULT_RETRACT_ACCELERATION	30			//[mm/sec^2]

#define DEFAULT_X_AXIS_STEPS_PER_UNIT	100		//[steps/mm]
#define DEFAULT_Y_AXIS_STEPS_PER_UNIT	100		//[steps/mm]
#define DEFAULT_Z_AXIS_STEPS_PER_UNIT	1600			//[steps/mm]
#define DEFAULT_E_AXIS_STEPS_PER_UNIT	200			//[steps/mm]
#define DEFAULT_A_AXIS_STEPS_PER_UNIT   1706/360.0f 	//[steps/degree]
#define DEFAULT_B_AXIS_STEPS_PER_UNIT   1706/360.0f		//[steps/degree]

#define DEFAULT_X_AXIS_MAX_FEEDRATE		1000			//[steps/sec]
#define DEFAULT_Y_AXIS_MAX_FEEDRATE		1000			//[steps/sec]
#define DEFAULT_Z_AXIS_MAX_FEEDRATE		1000			//[steps/sec]
#define DEFAULT_E_AXIS_MAX_FEEDRATE		1000			//[steps/sec]
#define DEFAULT_A_AXIS_MAX_FEEDRATE     2000
#define DEFAULT_B_AXIS_MAX_FEEDRATE     2000


#define DEFAULT_X_AXIS_MAX_ACCELERATION		20	//[mm/sec^2]
#define DEFAULT_Y_AXIS_MAX_ACCELERATION		20	//[mm/sec^2]
#define DEFAULT_Z_AXIS_MAX_ACCELERATION		40	//[mm/sec^2]
#define DEFAULT_E_AXIS_MAX_ACCELERATION		40	//[mm/sec^2]
#define DEFAULT_A_AXIS_MAX_ACCELERATION 	50
#define DEFAULT_B_AXIS_MAX_ACCELERATION 	50

#define DEFAULT_DROP_SEGMENTS					5				// 运动可忽略的步数

//======================================================================================//

#define X_AXIS	0
#define Y_AXIS	1
#define Z_AXIS	2
#define E_AXIS	5
#define A_AXIS 	3
#define B_AXIS 	4

// 通信数据
#define MAX_CMD_SIZE 128
#define BUFSIZE 8

#define ROTARY_SPEED_FACTOR 5.0f  // 旋转轴速度因子，调整旋转轴的速度

// 插补运动轴数
#define NUM_AXIS 6

#define TIMER_FREQ 2000000u

#define NEGATIVE_X_DIR 0    // for Mendel set to false, for Orca set to true
#define NEGATIVE_Y_DIR 0    // for Mendel set to true, for Orca set to false
#define NEGATIVE_Z_DIR 0    // for Mendel set to false, for Orca set to true
#define NEGATIVE_E_DIR 0    // for Mendel set to false, for Orca set to true
#define NEGATIVE_A_DIR 0
#define NEGATIVE_B_DIR 0

#define POSITIVE_X_DIR 1    // for Mendel set to false, for Orca set to true
#define POSITIVE_Y_DIR 1    // for Mendel set to true, for Orca set to false
#define POSITIVE_Z_DIR 1    // for Mendel set to false, for Orca set to true
#define POSITIVE_E_DIR 1    // for Mendel set to false, for Orca set to true
#define POSITIVE_A_DIR 1
#define POSITIVE_B_DIR 1

// 软件限位
#define min_software_endstops true  //If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  //If true, axis won't move to coordinates greater than the defined lengths below.
	
#define X_MAX_POS 150                                                                                         
#define X_MIN_POS -150
#define Y_MAX_POS 150
#define Y_MIN_POS -150
#define Z_MAX_POS 150
#define Z_MIN_POS -150


// 旋转轴角度限制
#define A_MIN_ANGLE -90.0f  // A轴最小角度
#define A_MAX_ANGLE 90.0f   // A轴最大角度
#define B_MIN_ANGLE -180.0f  // B轴最小角度
#define B_MAX_ANGLE 180.0f   // B轴最大角度



#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)


//default stepper release if idle
#define DEFAULT_STEPPER_DEACTIVE_TIME 60

// Disables axis when it's not being used. 
#define DISABLE_X 0
#define DISABLE_Y 0
#define DISABLE_Z 0
#define DISABLE_E 0 // For all extruders
#define DISABLE_A 0
#define DISABLE_B 0

#define EXTRUDERS 1
// homeaxis
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

#define HOMING_FEEDRATE {50*10, 50*10, 20*50, 0}  // set the homing speeds (mm/min)

//homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define X_HOME_RETRACT_MM 5 
#define Y_HOME_RETRACT_MM 5 
#define Z_HOME_RETRACT_MM 5 

#if X_HOME_DIR == -1
  #define X_HOME_POS X_MAX_LENGTH * 0.0  
#else    
  #define X_HOME_POS X_MAX_LENGTH * 1.0
#endif //X_HOME_DIR == -1
  
//Y axis
#if Y_HOME_DIR == -1
  #define Y_HOME_POS Y_MAX_LENGTH * 0.0
#else    
  #define Y_HOME_POS Y_MAX_LENGTH * 1.0
#endif //Y_HOME_DIR == -1
  
// Z axis
#if Z_HOME_DIR == -1 //BED_CENTER_AT_0_0 not used
  #define Z_HOME_POS -1.5//Z_MAX_LENGTH * 0.0
#else    
  #define Z_HOME_POS Z_MAX_LENGTH * 0.0
#endif //Z_HOME_DIR == -1

#endif
