#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>  
#include <fcntl.h>  
#include <stdio.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <errno.h>  
#include <string.h>  


#define LED_BRIGHTNESS "/sys/devices/platform/pca9626-led.1/brightness"
#define LED_ENABLE "/sys/devices/platform/pca9626-led.1/enable_led"
// LED x 3 (24 + 24 + 12) == 60
#define LED_NUM 130
//static FILE *fb;
//static FILE *fe;
static int fb, fe;
//static char buf[LED_NUM];

int led_init()
{

	if((fb=open(LED_BRIGHTNESS, O_RDWR )) == NULL) {
		printf("file %s cannot be opened\n", LED_BRIGHTNESS);
		exit(0);
	} else
		printf("file opened for writing brightness\n");

	if((fe=open(LED_ENABLE, O_RDWR  )) == NULL) {  
		printf("file %s cannot be opened\n", LED_ENABLE);
		exit(0);
	} else
		printf("file opened for writing enable led\n");
}

int led_deinit()
{
	close(fb);
	close(fe);

	return 0;
}


int led_read_brightness(char * buf)
{
	int ret;

	memset(buf, '\0', 120);
	ret = lseek(fb,0L,SEEK_SET);
	if (ret != 0)
		printf("%s, seek error\n", __func__);

	ret = read(fb, buf, LED_NUM - 10);
	if (ret < 0)
		printf("%s, error\n", __func__);

	printf("after readbuf:ret: %d buf: %s\n",ret,  buf);
	return ret;
}

int led_write_brightness(char *wbuf)
{
	int ret;

	ret = lseek(fb,0L,SEEK_SET);
	if (ret != 0)
		printf("%s, seek error\n", __func__);
	printf("size of wbuf: %d\n", strlen(wbuf));
	ret = write(fb, wbuf, strlen(wbuf));
	if (ret < 0)
		printf("%s, error\n", __func__);
	printf("%s ret : %d , len: %d , write buf: %s\n",__func__, ret,  strlen(wbuf), wbuf);

	return 0;
}

int led_enable()
{
	int ret;

	ret = lseek(fe,0L,SEEK_SET);
	if (ret != 0)
		printf("%s, error\n", __func__);
	printf("seek ret: %d \n", ret);
	ret = write(fe, "1", 1);
	if (ret < 0)
		printf("%s, error\n", __func__);
	printf("ret: %d %s %d\n", ret, __func__, __LINE__);

	return 0;
}

int led_disable()
{
	int ret;

	ret = lseek(fe,0L,SEEK_SET);
	if (ret != 0)
		printf("%s, error\n", __func__);
	printf("seek ret%d \n", ret);
	ret = write(fe, "0", 1);
	if (ret < 0)
		printf("%s, error\n", __func__);
	printf("ret: %d %s %d\n", ret, __func__, __LINE__);

	return 0;
}

int led_state()
{
	int ret = 0;
	char state[10];
	ret = lseek(fe,0L,SEEK_SET);
	if (ret != 0)
		printf("%s, error\n", __func__);

	ret = read(fe, state, 1);
	if (ret < 0)
		printf("%s, error\n", __func__);

	printf("%s ret : %d state %s, c: %c\n", __func__, ret, state, state[0]);
	return state[0] - '0';
}

#if 0
int main(int argc, char **argv)
{
	char buf[120];
	led_init();
	printf("%d\n", led_state());
	led_enable();
	sleep(3);
	printf("state %d \n", led_state());
	sleep(3);
	printf("%s", led_read_brightness(buf));
	sleep(3);
	//led_read_brightness();
	printf("%d\n", __LINE__);
	if (argc > 1)
		led_write_brightness(argv[1]);
	sleep(3);
	printf("%d\n", __LINE__);
	printf("after w read %s", led_read_brightness(buf));
	sleep(3);
	printf("state %d \n", led_state());
	sleep(3);
#if 1
	led_disable();
#endif
	led_deinit();
}
#endif
