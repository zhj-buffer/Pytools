#define STOPED 0
#define FORWARD 1
#define BACKWARD 2

#define APP_CMD_START_WORK      1
#define APP_CMD_STOP_WORK       2
#define APP_CMD_SET_PARA        3

int motor_init(void);
int motor_control(int command, int direction, int duty, int freq);
int motor_stop(void);
int motor_start(void);
int motor_deinit(void);
int motor_status(void);
