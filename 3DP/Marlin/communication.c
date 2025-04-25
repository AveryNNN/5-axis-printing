#include "marlin.h"
#include "leveling.h"
// 定义指令字符缓存队列
ring_buffer_typedef rx_buffer = { { 0 }, 0, 0 };
serial_func_typedef serial;
static char serial_char;
static int serial_count = 0;
static char comment_mode = false;
static unsigned int previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;
// 运动指令
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'A', 'B','E'};
float offset[3] = {0.0, 0.0, 0.0};
float destination[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float current_position[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float add_homeing[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


extern float min_pos[5];
extern float max_pos[5];

#define AXIS_RELATIVE_MODES {false, false, false, false}

#define HOMEAXIS(X) homeaxis(X##_AXIS)

unsigned char axis_relative_modes[] = {false, false, false, false, false, false};
volatile static unsigned char relative_mode = false;  //Determines Absolute or Relative Coordinates

static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static float saved_feedmultiply=0.0f;

static unsigned char home_all_axis = true;

uint8_t Stopped=false;

uint8_t active_extruder = 0;
static unsigned char tmp_extruder;
int feedmultiply=100; //100->1 200->2

float homing_feedrate[] = HOMING_FEEDRATE;
float max_length[] =   {X_MAX_LENGTH,Y_MAX_LENGTH,Z_MAX_LENGTH};
signed char home_dir[] = {X_HOME_DIR,Y_HOME_DIR,Z_HOME_DIR};
float home_retract_mm[] = {X_HOME_RETRACT_MM,Y_HOME_RETRACT_MM,Z_HOME_RETRACT_MM};
float base_home_pos[]= {X_HOME_POS,Y_HOME_POS,Z_HOME_POS};
float base_min_pos[] = {X_MIN_POS, Y_MIN_POS, Z_MIN_POS};
float base_max_pos[] = {X_MAX_POS, Y_MAX_POS, Z_MAX_POS};

static void Uart_Hal_UART_Errorhandle(void);


void store_char(unsigned char c, ring_buffer_typedef *rx_buffer)
{
  int i = (rx_buffer->head + 1) % RX_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != rx_buffer->tail) //队头+1不等于队尾说明队列不满
	{
    rx_buffer->buffer[rx_buffer->head] = c;//串口读值入队
    rx_buffer->head = i;//移动队头
  }
}
// 返回缓存队列中的数据
unsigned int serial_available(void)
{
   return (unsigned int)(RX_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % RX_BUFFER_SIZE;
}
// 串口读字符
unsigned char serial_read(void)
{
	unsigned char ch;
	if(rx_buffer.head == rx_buffer.tail) //缓存队列是否为空
	{
    return 0;
  } 
	else 
	{
    ch = rx_buffer.buffer[rx_buffer.tail];//先入先出
    rx_buffer.tail = (unsigned int)(rx_buffer.tail + 1) % RX_BUFFER_SIZE;//元素出队,队尾移动
    return ch;
  }
}
// 清空串口
void serial_flush(void)
{
  rx_buffer.head = rx_buffer.tail;
}

void serial_isr(unsigned char ch)
{
	store_char(ch, &rx_buffer);
}


static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static int buflen = 0;
static int bufindr = 0;
static int bufindw = 0;
volatile static char fromsd[BUFSIZE];
static char *strchr_pointer;

static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

void ClearToSend()
{	
  printf(MSG_OK);
  printf("\n");
}

void FlushSerialRequestResend()
{
  serial_flush();
  printf(MSG_RESEND);
  printf("%d\n",(int)gcode_LastN + 1);
  ClearToSend();
}

void get_command(void)
{
	while( serial_available() > 0  && buflen < BUFSIZE)//#define BUFSIZE 8 指令缓存条数
  {
		serial_char = serial_read();
		if(serial_char == '\n' ||serial_char == '\r' ||(serial_char == ':' && comment_mode == false) ||serial_count >= (MAX_CMD_SIZE - 1) )//sanse 冒号
    {
			/*这个if没有运行????*/
			if(!serial_count)  //serial_count == 0 意味着空指令
	    {
        comment_mode = false; //for new command 做好处理新命令的准备
        return;
      }
			// 终止字符
			cmdbuffer[bufindw][serial_count] = 0; //terminate string
			if(!comment_mode)//新指令
	    {
				/*为啥要有这句*/
				comment_mode = false; //for new command
				/*这个数组什么作用*/
				fromsd[bufindw] = false;
				if(strchr(cmdbuffer[bufindw], 'N') != NULL)//在cmdbuffer中找到了"N"
				{
					strchr_pointer = strchr(cmdbuffer[bufindw], 'N');// 获得行号
					gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
					// 行号检测
					if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], PSTR("M110")) == NULL) )  //重发
		      {                                                                          
            SERIAL_ERROR_START;
            printf(MSG_ERR_LINE_NO);
            printf("%ld\n",gcode_LastN);
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }			
					
					
					// 校验*号检测
					if(strchr(cmdbuffer[bufindw], '*') != NULL)
          {
            uint8_t checksum = 0;
            uint8_t count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');                                      
            if((uint8_t)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum)
						{                                                                 
							SERIAL_ERROR_START;
							printf(MSG_ERR_CHECKSUM_MISMATCH);
							printf("%ld\n",gcode_LastN);
							FlushSerialRequestResend();
							serial_count = 0;
							return;
            }
            //if no errors, continue parsing
          }
					else // 指令中无*号校验码
          {
            SERIAL_ERROR_START;
            printf(MSG_ERR_NO_CHECKSUM);
            printf("%ld\n",gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
					gcode_LastN = gcode_N;
				}
				else  // if we don't receive 'N' but still see '*'
        {
          if((strchr(cmdbuffer[bufindw], '*') != NULL))                                                            
          {
            SERIAL_ERROR_START;
            printf(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            printf("%ld\n",gcode_LastN);
            serial_count = 0;
            return;
          }
        }
				// 处理G代码
				if((strchr(cmdbuffer[bufindw], 'G') != NULL))                                                                
		    {
					strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
					switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL))))
					{
						case 0:
						case 1:
						case 2:
						case 3:
							if(Stopped == true) 
							{ 
								printf(MSG_ERR_STOPPED);
								printf("\n");
							}
							break;
						default:
							break;
					}
				}
				bufindw = (bufindw + 1)%BUFSIZE;
        buflen += 1;//sanse
			}
			serial_count = 0; //clear buffer
		}
		else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
	}
	Uart_Hal_UART_Errorhandle();
}

float code_value(void)
{
	return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

unsigned char code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

void get_coordinates()
{ 
	int8_t i;
  volatile unsigned char seen[NUM_AXIS] = {false, false, false, false, false, false};
 for(i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
            if(i == A_AXIS || i == B_AXIS) {
                // 旋转轴使用角度值
                float angle_value = (float)code_value();
                
                // 处理角度值（角度可以为任意值，但实际上会考虑模360度）
                destination[i] = (axis_relative_modes[i] || relative_mode) ? 
                                 current_position[i] + angle_value : 
                                 angle_value;
            } else {
                // 线性轴使用距离值
                destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
            }
            seen[i]=true;
        }
        else destination[i] = current_position[i];
    }
  if(code_seen('F')) 
	{
    next_feedrate = code_value();
    if(next_feedrate > 0.0f) feedrate = next_feedrate;
    if(next_feedrate > 200.0f) feedrate = 200;
  }
}
void get_arc_coordinates(void)
{
  get_coordinates();

  if(code_seen('I')) 
	{
    offset[0] = code_value();
  }
  else 
	{
    offset[0] = 0.0;
  }
   if(code_seen('J')) 
	{
    offset[1] = code_value();
  }
  else 
	{
    offset[1] = 0.0;
  }
}
void clamp_to_software_endstops(float target[NUM_AXIS])
{
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
    // A轴和B轴通常是旋转轴，可能不需要软件限位，但如果需要可以添加
    if (target[A_AXIS] < A_MIN_ANGLE) target[A_AXIS] = A_MIN_ANGLE;
    if (target[B_AXIS] < B_MIN_ANGLE) target[B_AXIS] = B_MIN_ANGLE;
	}

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
    // A轴和B轴的最大限位同上
		if (target[A_AXIS] > A_MAX_ANGLE) target[A_AXIS] = A_MAX_ANGLE;
		if (target[B_AXIS] > B_MAX_ANGLE) target[B_AXIS] = B_MAX_ANGLE;
  }
}

void prepare_move(void)
{  
	int8_t i;
  clamp_to_software_endstops(destination);//限位
  // previous_millis_cmd = millis();
  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) //只有Z轴运动
	{
     plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], 
                  destination[E_AXIS], destination[A_AXIS], destination[B_AXIS], 
                  feedrate*feedmultiply/60/100.0f, active_extruder);
  }
  else //X,Y运动
	{
     plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], 
                  destination[E_AXIS], destination[A_AXIS], destination[B_AXIS], 
                  feedrate*feedmultiply/60/100.0f, active_extruder);
  }
  for(i=0; i < NUM_AXIS; i++) 
	{
    current_position[i] = destination[i];
  }
}

void prepare_arc_move(unsigned char isclockwise) 
{
	int8_t i;
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0f, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(i=0; i < NUM_AXIS; i++) 
	{
    current_position[i] = destination[i];
  }
  //previous_millis_cmd = millis();
}



void manage_inactivity(void)
{
  if( (serial.get_timer_ms() - previous_millis_cmd) >  max_inactive_time )
    if(max_inactive_time)
      //kill();
  if(stepper_inactive_time)  {
    if( (serial.get_timer_ms() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false) 
			{
        disable_all_stepper();
      }
    }
  }
  check_axes_activity();
}

long code_value_long(void)
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

void process_commands(void)
{ 
	unsigned long codenum; //throw away variable
	if(code_seen('G'))//在接收缓冲区中寻找G
	{
		switch((int)code_value())//提取G后数字
    {
			case 0: // G0 -> G1
			case 1: // G1直线
				if(Stopped == false) 
				{
					get_coordinates(); // For X Y Z E F
					prepare_move();
				}
				break;
			case 2://圆弧
				if(Stopped == false) 
				{
					get_arc_coordinates();
					prepare_arc_move(true);
				}
				break;
			case 3: // G3  - CCW ARC
				if(Stopped == false) 
				{
					get_arc_coordinates();
					prepare_arc_move(false);
				}
				break;
			case 4: // G4 dwell
				codenum = 0;
				if(code_seen('P')) codenum = code_value(); // milliseconds to wait //单位为ms
				if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait//单位为s
				//1.先让运动块缓存中的所有块执行完成
				while(blocks_queued())
				{
					// 添加阻塞期间需要执行的代码
				}
				codenum += serial.get_timer_ms();
				previous_millis_cmd = serial.get_timer_ms();
				while(serial.get_timer_ms() < codenum)
				{
					// 添加延时等候期间需要执行的代码
				}
				break;
			case 28://G28 Home all Axis one at a time
				saved_feedrate = feedrate;
				saved_feedmultiply = feedmultiply;
				feedmultiply = 100;
				previous_millis_cmd = serial.get_timer_ms();
				enable_endstops(true);
				uint8_t i;
				for(i=0; i < NUM_AXIS; i++) 
				{
					destination[i] = current_position[i];
				}
				feedrate = 0.0;
				home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || 
                    (code_seen(axis_codes[2])) || (code_seen(axis_codes[3])) || 
                    (code_seen(axis_codes[4])) || (code_seen(axis_codes[5]))); 
				 // X轴回零
				  if((home_all_axis) || (code_seen(axis_codes[X_AXIS]))) {
					HOMEAXIS(X);
				  }
				  
				  // Y轴回零
				  if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
					HOMEAXIS(Y);
				  }
				  
				  // Z轴回零
				  if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
					HOMEAXIS(Z);
				  }
				  
				  // A轴回零（如果需要）
				  if((home_all_axis) || (code_seen(axis_codes[A_AXIS]))) {
					HOMEAXIS(A);
				  }
				  
				  // B轴回零（如果需要）
				  if((home_all_axis) || (code_seen(axis_codes[B_AXIS]))) {
					HOMEAXIS(B);
				  }
				if(code_seen(axis_codes[X_AXIS]))
				{
					if(code_value_long() != 0) 
					{
						current_position[X_AXIS]=code_value()+add_homeing[0];
					}
				}

				if(code_seen(axis_codes[Y_AXIS]))
				{
					if(code_value_long() != 0) 
					{
						current_position[Y_AXIS]=code_value()+add_homeing[1];
					}
				}

				if(code_seen(axis_codes[Z_AXIS]))
				{
					if(code_value_long() != 0) 
					{
						current_position[Z_AXIS]=code_value()+add_homeing[2];
					}
				}
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], current_position[A_AXIS], current_position[B_AXIS]);;
				feedrate = saved_feedrate;
				feedmultiply = saved_feedmultiply;
				previous_millis_cmd = serial.get_timer_ms();
				endstops_hit_on_purpose();
			break;
			case 90: // G90
				relative_mode = false;
      break;
			case 91: // G91
				relative_mode = true;
      break;
			case 92: // G92 - 设置位置
			  if(!code_seen(axis_codes[E_AXIS])) {
				st_synchronize();
			  }
			  for(int8_t i=0; i < NUM_AXIS; i++) {
				if(code_seen(axis_codes[i])) {
				  if(i == E_AXIS) {
					current_position[i] = code_value();
					plan_set_e_position(current_position[E_AXIS]);
				  } else {
					current_position[i] = code_value();
					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], 
									 current_position[Z_AXIS], current_position[E_AXIS],
									 current_position[A_AXIS], current_position[B_AXIS]);
				  }
				}
			  }
			  break;
			default:
			break;
		}
	}
	else if(code_seen('M'))//查询
  {	
		switch( (int)code_value())
    { 
			case 17:
				get_coordinates(); // For X Y Z E F
				enable_all_stepper();
				break;
			case 101:
				//char j;
				for(char j=0; j < NUM_AXIS; j++) 
				{
					destination[j] = current_position[j];
				}
				if(code_seen(axis_codes[X_AXIS]))
				{
					if(code_value_long() != 0) 
					{
						current_position[X_AXIS]=code_value();
					}
				}

				if(code_seen(axis_codes[Y_AXIS])) 
				{
					if(code_value_long() != 0) 
					{
						current_position[Y_AXIS]=code_value();
					}
				}

				if(code_seen(axis_codes[Z_AXIS])) 
				{
					if(code_value_long() != 0) 
					{
						current_position[Z_AXIS]=code_value();
					}
				}
				if(code_seen(axis_codes[E_AXIS])) 
				{
					if(code_value_long() != 0) 
					{
						current_position[E_AXIS]=code_value();
					}
				}
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], current_position[A_AXIS], current_position[B_AXIS]);;
				break;
			case 104: // M104
		      break;
			case 140: 
		      break;
			case 105 : 
				printf("ok");
				printf(" T@:%.1f",100.0f);
				printf(" B@:%d\n\r",10);
			return;
			case 106:
				break;


			case 82:
	        axis_relative_modes[3] = false;
	        break;
	    case 83:
	        axis_relative_modes[3] = true;
	        break;
			case 114: // M114	 
				  printf("X:%f Y:%f Z:%f E:%f",current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS]);			
			    printf(MSG_COUNT_X);
				  printf("%f Y:%f Z:%f\n",((float)st_get_position(X_AXIS))/80,((float)st_get_position(Y_AXIS))/80,((float)st_get_position(Z_AXIS))/800);
		      break; 
			case 115: // M115
		      printf(MSG_M115_REPORT);
		      break;
		  case 120: // M120
		      enable_endstops(false) ;
		      break;
		  case 121: // M121
		      enable_endstops(true) ;
		      break;
			case 220: // M220 S<factor in percent>- set speed factor override percentage
			    {
			      if(code_seen('S'))
			      {
			        feedmultiply = code_value() ;
			      }
			    }
		      break;
		   case 221: // M221 S<factor in percent>- set extrude factor override percentage
			    {
			      if(code_seen('S'))
			      {
			        ;
			      }
			    }
		      break;
			case 999: // M999: Restart after being stopped
	      Stopped = false;
	    //  lcd_reset_alert_level();//////////////////////////////////////
	      gcode_LastN = Stopped_gcode_LastN;
	      FlushSerialRequestResend();
	    break;
			default:
			break;
		}
	}
	else if(code_seen('T'))//更换喷头
	{
		tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) 
		{
      SERIAL_ECHO_START;
      printf("T%d",tmp_extruder);
      printf(MSG_INVALID_EXTRUDER);
		}
		else
		{
			volatile unsigned char make_move = false;
      if(code_seen('F')) 
			{
        make_move = true;
        next_feedrate = code_value();
        if(next_feedrate > 0.0f) 
				{
          feedrate = next_feedrate;
        }
      }
			#if EXTRUDERS > 1
			if(tmp_extruder != active_extruder) 
			{
				// Save current position to return to after applying extruder offset
				memcpy(destination, current_position, sizeof(destination));
				// Offset extruder (only by XY)
				for(i = 0; i < 2; i++) 
				{
					current_position[i] = current_position[i] - extruder_offset[i][active_extruder] + extruder_offset[i][tmp_extruder];
				}
				// Set the new active extruder and position
				active_extruder = tmp_extruder;
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], current_position[A_AXIS], current_position[B_AXIS]);;
				// Move to the old position if 'F' was in the parameters
				if(make_move && Stopped == false) 
				{
						prepare_move();
				}
				}	//end   if(tmp_extruder != active_extruder) 
			#endif
			SERIAL_ECHO_START;
			printf(MSG_ACTIVE_EXTRUDER);
			printf("%d",active_extruder);
			printf("\n");
		}
	}
	else
  { 
		SERIAL_ECHO_START;
    printf(MSG_UNKNOWN_COMMAND);
    printf("%s",cmdbuffer[bufindr]);
    printf("\"");
  }
  ClearToSend();
}

void start_command_process(void)
{
	if(buflen < (BUFSIZE-1))
	{
		// 获取命令
    get_command();
	}
	if(buflen)//缓存中有命令
	{
		process_commands();
		buflen = (buflen-1);//已处理指令,标志位-1
    bufindr = (bufindr + 1)%BUFSIZE;//指向下一条指令
	}
	checkHitEndstops();//限位检测
}


static void axis_is_at_home(int axis) 
{
  current_position[axis] = base_home_pos[axis] + add_homeing[axis];
  min_pos[axis] =          base_min_pos[axis] + add_homeing[axis];
  max_pos[axis] =          base_max_pos[axis] + add_homeing[axis];
}


// 所有运动轴回零点
void homeaxis(int axis)
{
	#define HOMEAXIS_DO(LETTER) ((LETTER##_HOME_DIR==-1) || (LETTER##_HOME_DIR==1))

  if (axis==X_AXIS ? HOMEAXIS_DO(X) : axis==Y_AXIS ? HOMEAXIS_DO(Y) :axis==Z_AXIS ? HOMEAXIS_DO(Z) : 0) 
	{
		current_position[axis] = 0; 
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], current_position[A_AXIS], current_position[B_AXIS]);; 
		destination[axis] = 1.5f * max_length[axis] * home_dir[axis]; 
		feedrate = homing_feedrate[axis]; 
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], destination[A_AXIS], destination[B_AXIS], feedrate/60, active_extruder);;
		destination[axis]=0;
		st_synchronize(); 

		current_position[axis] = 0; 
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], current_position[A_AXIS], current_position[B_AXIS]);;
		destination[axis] = -home_retract_mm[axis] * home_dir[axis]; 
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], destination[A_AXIS], destination[B_AXIS], feedrate/60, active_extruder);;
		destination[axis]=0;
		st_synchronize();

		destination[axis] = 2*home_retract_mm[axis] * home_dir[axis];  
		feedrate = homing_feedrate[axis]/2 ;
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], destination[A_AXIS], destination[B_AXIS], feedrate/60, active_extruder);;
		destination[axis]=0;
		st_synchronize(); 
    
		axis_is_at_home(axis); 
		destination[axis] = current_position[axis]; 
		feedrate = 0.0;
		endstops_hit_on_purpose(); 
  }
}

static void Uart_Hal_UART_Errorhandle(void)
{
	UartHandle.Instance->CR1 |= (1U)<<5;
}










