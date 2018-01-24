#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>                     //Used for UART
#include <fcntl.h>                      //Used for UART
#include <termios.h>            //Used for UART
#include <sys/ioctl.h>
#define MC_MAGIC 0xdf

#include "motor.h"

#define MOROR_CONTROL_ONOFF _IOW(MC_MAGIC, 0x01, unsigned long)
#define MOTOR_CONTROL_SETTINGS  _IOW(MC_MAGIC, 0x02, unsigned long)
#define OFFLINE

#if 0
#define FORWARD 1
#define BACKWARD 2

#define APP_CMD_START_WORK      1
#define APP_CMD_STOP_WORK       2 
#define APP_CMD_SET_PARA        3
#endif

struct motor_control {
    int direction_value;
    int duty_value;
    int freq_value;
};

static int fd = 0;
static int status = APP_CMD_STOP_WORK;
static int dir = 0;

int motor_init(void)
{
#ifdef OFFLINE
    printf("Enter motor init  program!\n");
    fd = open("/dev/motor-control", O_RDWR);
    if (fd < 0)
    {
        printf("Open Dev motor control fail!\n");
        return -1;
    }


    return fd;
#else
    return 0;
#endif
    //printf("closd pwm device\n");
}

int motor_start()
{
    motor_control(APP_CMD_START_WORK, 0, 0, 0);
    return 0;
}

int motor_stop()
{
//    printf("%s, %d\n", __func__, __LINE__);
	printf("%s =================\n", __func__);
    motor_control(APP_CMD_STOP_WORK, 0, 0, 0);
    return 0;
}

int motor_control(int command, int direction, int duty, int freq)
{
#ifdef OFFLINE
    struct motor_control control;
    unsigned long arg = 0;
    int cmd;

    printf(" move %s cmd: %d, direction: %d, duty:%d, freq:%d\n", __func__,command, direction, duty, freq);
    switch(command)
    {
    case APP_CMD_START_WORK:
        //if (status != command) {
            cmd = MOROR_CONTROL_ONOFF;
            arg = 1;
            if(ioctl(fd, cmd, &arg) < 0)
            {
                printf("Call cmd MOROR_CONTROL_ONOFF fail\n");
                return -1;
            }
            printf("start motor control\n"); 
            status = 3;
        //}
        break;
    case APP_CMD_SET_PARA:
#if 0
        if (direction < 1 || direction > 2) {
            printf("Invalide direction\n");
            break;
        }
		if (dir != direction) {
			motor_stop();
		}
#endif

		if (dir != direction) {
			motor_stop();
			control.direction_value = direction;
			control.duty_value = duty;
			control.duty_value = 90;
			control.freq_value = freq;
			control.freq_value = 5;

			if(ioctl(fd, MOTOR_CONTROL_SETTINGS, &control) < 0)
			{
				printf("Call cmd MOTOR_CONTROL_SETTINGS fail\n");
				return -1;
			}
			//status = command;
			//} else {

			printf("==== change direction new: %d\n", direction);
//			if (status != APP_CMD_START_WORK) {
//				printf("s_ould start the motor fist\n");
			motor_start();
				//motor_control(APP_CMD_START_WORK, 0, 0, 0);
//			}
			status = direction;
			dir = direction;
		} else
			printf("==== keep moveing ...status:..%d, dir: %d\n", status, dir);
        //}
        break;
    case APP_CMD_STOP_WORK:
        if (status != command) {
            cmd  = MOROR_CONTROL_ONOFF;
            arg = 0;
            if(ioctl(fd, cmd, &arg) < 0)
            {   
                printf("Call cmd MOROR_CONTROL_ONOFF fail\n");
                return -1;
            }
            printf("move stop motor control, before status: %d\n", status);                   
            status = STOPED;
			dir = 0;
        }
        break;
    default:
            break;
    }

    return 0;
#else
    return 0;
#endif
}

int motor_status()
{
	return status;
}

int motor_deinit()
{

#ifdef OFFLINE
	int cmd = MOROR_CONTROL_ONOFF;
	int arg = 0;
	if(ioctl(fd, cmd, &arg) < 0)
	{   
		printf("Call cmd MOROR_CONTROL_ONOFF fail\n");
		return -1;
	}

    //motor_stop();
    close(fd);
    return 0;
#else
    return 0;
#endif
}

#if 0
int main()
{
	motor_init();
	printf("===");
	sleep(2);
	//motor_control(APP_CMD_SET_PARA, BACKWARD, 90, 5);
	motor_control(APP_CMD_SET_PARA, FORWARD, 90, 5);
	motor_start();
	sleep(2);
	motor_control(APP_CMD_SET_PARA, BACKWARD, 90, 5);
	sleep(2);
	motor_control(APP_CMD_SET_PARA, BACKWARD, 90, 5);
	sleep(1);
	printf("=====");
	motor_stop();

	return 0;
}
#endif
