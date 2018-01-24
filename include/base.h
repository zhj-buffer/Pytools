#ifndef BASE
#define BASE

#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "queue.h"
#include "image.h"

#include "image_core.h"

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


#define FRAMES 3
#define FRAMES_MASK ((1 << FRAMES) - 1)

#define CAM_COUNT 4

#define QUEUES (CAM_COUNT * 3)
#define QUEUES_MASK ((1 << QUEUES) - 1)

#ifdef OPENCV
#include <cv.h>
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/videoio/videoio_c.h"
#endif

#define CAMERA_DEV "/dev/video0"
/* For camera */
#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define CAM_W   1920
#define CAM_H  1080
#define CAM_C  3

/**
 *  * If set, host memory is portable between CUDA contexts.
 *   * Flag for ::cuMemHostAlloc()
 *    */
#define CU_MEMHOSTALLOC_PORTABLE        0x01

/**
 *  * If set, host memory is mapped into CUDA address space and
 *   * ::cuMemHostGetDevicePointer() may be called on the host pointer.
 *    * Flag for ::cuMemHostAlloc()
 *     */
#define CU_MEMHOSTALLOC_DEVICEMAP       0x02

/**
 *  * If set, host memory is allocated as write-combined - fast to write,
 *   * faster to DMA, slow to read except via SSE4 streaming load instruction
 *    * (MOVNTDQA).
 *     * Flag for ::cuMemHostAlloc()
 *      */
#define CU_MEMHOSTALLOC_WRITECOMBINED   0x04

/**
 *  * If set, host memory is portable between CUDA contexts.
 *   * Flag for ::cuMemHostRegister()
 *    */
#define CU_MEMHOSTREGISTER_PORTABLE     0x01

/**
 *  * If set, host memory is mapped into CUDA address space and
 *   * ::cuMemHostGetDevicePointer() may be called on the host pointer.
 *    * Flag for ::cuMemHostRegister()
 *     */
#define CU_MEMHOSTREGISTER_DEVICEMAP    0x02

/**
 *  * If set, peer memory is mapped into CUDA address space and
 *   * ::cuMemPeerGetDevicePointer() may be called on the host pointer.
 *    * Flag for ::cuMemPeerRegister()
 *     */
#define CU_MEMPEERREGISTER_DEVICEMAP    0x02

#define MEMORY_ALIGNMENT  4096
#define ALIGN_UP(x,size) ( ((size_t)x+(size-1))&(~(size-1)) )

typedef void (*THREADP)(void * data);

struct buffer {
    void *      start;
    void *      align_addr;
    void *      d_addr;
    size_t      length;
};

typedef struct eye {
    int camera_fd[CAM_COUNT];
	char cameraName[CAM_COUNT][20];
    pthread_mutex_t mutex;
    pthread_cond_t cond_detect;
    pthread_t id_detect;


    pthread_t aquireFrame[CAM_COUNT];
    THREADP threadFunc[CAM_COUNT];

	char **names;
	image **alphabet;
	float hier_thresh;
	int classes;
    float thresh;
    float thresh_hand;

    struct v4l2_buffer *cur_buf;
    struct buffer   *buffers[CAM_COUNT];
    unsigned int    n_buffers;

	IplImage *showImg[CAM_COUNT];

	network *net;
    layer *l;
    box *boxes;
    float **probs;
    float nms;

    float *predictions[FRAMES];
    float *avg;

    char output[30];
    bool debug;

    image reim[QUEUES];
    image feedim;
    int detect_idx;

    struct v4l2_buffer *qbuf[CAM_COUNT];

    Queue detectQueue;

    int cam_w;
    int cam_h;
    int cam_c;
    int cam_fps;


#ifdef GPU
    float *dev_dst;
    float *dev_partdst;
    float *dev_redst;
    char *dev_src;
#endif

    bool ready;
    int ready_idx;
    image showim[CAM_COUNT];

} eye, *peye;

void fetch_detections_result(peye mdata, int num, float thresh, box *boxes, float **probs, char **names, int classes, int cam_idx, float thresh_hand);
void cvText(IplImage* img, const char* text, int x, int y);
int base(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, int classes, int frame_skip, char *prefix, float hier_thresh, float thresh_hand);

#endif
