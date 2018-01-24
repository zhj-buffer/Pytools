#include <stdio.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>


//#define brightness_BRIGHTNESS "/sys/devices/platform/pca9626-brightness.1/brightness"
//#define brightness_ENABLE "/sys/devices/platform/pca9626-brightness.1/enable_brightness"
#define BRIGHTNESS "/sys/class/i2c-adapter/i2c-0/0-0039/iio_device/in_illuminance0_input"
#define BRIGHTNESS1 "/sys/class/i2c-adapter/i2c-1/1-0039/iio_device/in_illuminance0_input"
// brightness x 3 (24 + 24 + 12) == 60
#define brightness_NUM 120

static int fc, fc1;

int brightness_init()
{

    if((fc=open(BRIGHTNESS,O_RDWR)) == NULL) {
        printf("file %s cannot be opened\n", BRIGHTNESS);
	return -1;
    } else
        printf("remote_ctrl_thread file opened for writing aa brightness\n");

    if((fc1=open(BRIGHTNESS1,O_RDWR)) == NULL) {
        printf("file %s cannot be opened\n", BRIGHTNESS);
	return -1;
    } else
        printf("remote_ctrl_thread file opened for writing aa brightness\n");


#if 0
    fread(buf, 1, 400, fp);
#endif
//    printf("buf: %s\n", buf);
}

int brightness_deinit()
{
    close(fc);
    close(fc1);

    return 0;
}


int brightness_read(char *buf)
{
	int ret = 0;
	char str[20];
	char str1[20];
	
#if 1
	ret = lseek(fc,0L,SEEK_SET);
	if (ret != 0)
		printf("%s, error\n", __func__);
#endif
#if 1
	ret = lseek(fc1,0L,SEEK_SET);
	if (ret != 0)
		printf("%s, error\n", __func__);
#endif

//    ret = fread(buf, 1, 20, fb);
	ret = read(fc, str, 20);
	if (ret <= 0)
		printf("remote_ctrl_thread %s, error\n", __func__);


	ret = read(fc1, str1, 20);
	if (ret <= 0)
		printf("remote_ctrl_thread %s, error\n", __func__);
	sprintf(buf, "%d,%d",atoi(str), atoi(str1));
	printf("remote_ctrl_thread ret %d, buf: %s,  fucn: %s\n", ret, buf, __func__ );
    return ret;
}

#if 0
int main()
{
	brightness_init();
	char buf[20];
	brightness_read(buf);
	brightness_deinit();

	return 0;
}

int brightness_write(char *buf)
{
    fwrite(buf, 1, 20, fb);

    return 0;
}
int brightness_enable()
{
    fwrite("1", 1, 1, fe);

    return 0;
}
int brightness_disable()
{
    fwrite("0", 1, 1, fe);

    return 0;
}

int brightness_state()
{
    char buf;
    fread(&buf, 1, 1, fe);

    return buf - '0';
}
#endif
