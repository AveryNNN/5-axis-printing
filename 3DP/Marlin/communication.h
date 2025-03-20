#ifndef communication_h
#define communication_h

#define	RX_BUFFER_SIZE 128

#define SERIAL_ERROR_START	printf("Error:")
#define SERIAL_ECHO_START	printf("echo:")

// 缓存环形队列
typedef struct ring_buffer 
{
  unsigned char buffer[RX_BUFFER_SIZE];
  int head;
  int tail;
}ring_buffer_typedef;

typedef struct 
{
	void (*serial_init)(unsigned int bound);
	unsigned int (*get_timer_ms)(void);
	
}serial_func_typedef;

extern serial_func_typedef serial;

void clamp_to_software_endstops(float target[3]);
void start_command_process(void);
void serial_isr(unsigned char ch);
void homeaxis(int axis);

#endif
