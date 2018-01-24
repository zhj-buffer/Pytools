#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "image.h"
#include "log.h"

#include "image_core.h"
#include "deconvolutional_layer.h"
#include "darknet.h"

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <memory.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <malloc.h>
#include <poll.h>
#include <sys/mman.h>
#include <time.h>
#include <pthread.h>

#ifdef OPENCV
#include <cv.h>
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/videoio/videoio_c.h"
#endif

#define TIMEOUT (30 * 60)

int frame1;

typedef struct caminfo {
    int cam_idx;
    int fps;
    bool autoc;
    long long int count;
} caminfo;

typedef struct mnet {

    char **names;
    float hier_thresh;
    int classes;
    float thresh;
    float thresh_hand;

    IplImage *showImg;

    network *net;
    layer *l;
    box *boxes;
    float **probs;
    float nms;

    char output[30];
    bool debug;
    int cam_idx;
    int num;

    char coordinate[80];
    char msg[100];
    caminfo info[4];
//    int fps[4][2];
    pthread_mutex_t range_lock;
    pthread_cond_t range_cond;
    pthread_t range_handle;
    pthread_t ctrl_handle;
    pthread_t timer_handle;
    pthread_t led_handle;

    bool exit;
    int d0;
    int d1;
	int brightness;
	bool remote_motor;
	bool remote_led;

	int t_motor;
	int t_led;

        int w;
        int h;
} mnet, *pmnet;


int draw_vText(IplImage* img, const char* text, int x, int y);
int get_detections_result(pmnet mdata, IplImage *showImg);
int object_detect_run(pmnet mdata,  float *buf);
pmnet object_detect_init(int cam_idx);
void object_detect_deinit(pmnet mdata);

int get_control_msg(pmnet mdata);
int send_with_face_msg(pmnet mdata, char *msg);
int send_without_face_msg(pmnet mdata);
int append_with_face_msg(pmnet mdata, char *msg);
int send_dbus_msg(char *transfer, pmnet mdata);
int filter_detect_to_event(char *event_name);
int timeval_subtract(struct timeval* result, struct timeval* x, struct timeval* y);

#ifdef __cplusplus
};
#endif

#endif

