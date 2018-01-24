#include "base.h"
#ifdef DBUS
#include "dbus.h"
#endif

static float *predictions[FRAMES];
static image images[FRAMES];
static float *avg;

#ifdef DBUS
DBusConnection * connection;
#endif

box *rbox;
//static char *lable_names[] = {"aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};
static char *lable_names[] = {"head", "bicycle", "bird", "hand", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};

static int init_device (peye mdata, int cam_idx)
{
    struct v4l2_requestbuffers req;
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min;
    int ret;
    struct v4l2_fmtdesc fmtdesc;
    int fd = mdata->camera_fd[cam_idx];
    int fps = mdata->cam_fps;

    if (-1 == ioctl (fd, VIDIOC_QUERYCAP, &cap)) {
        fprintf (stderr, "VIDIOC_QUERYCAP fail\n");
        goto err;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf (stderr, "VIDIOC_QUERYCAP fail\n");
        goto err;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        fprintf (stderr, "device does not support streaming i/o\n");
        goto err;
    }
    /* Print capability infomations */
    printf("\nCamera: %d=======================================\n", cam_idx);
    printf("Capability Informations:\n");
    printf(" driver: %s\n", cap.driver);
    printf(" card: %s\n", cap.card);
    printf(" bus_info: %s\n", cap.bus_info);
    printf(" version: %08X\n", cap.version);

    /* Select video input, video standard and tune here. */
    CLEAR (cropcap);
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 == ioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == ioctl (fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                    break;
            }
        }
    }
    /* enum video formats. */
    CLEAR(fmtdesc);
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Enum format:\n");
    while ((ret = ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) == 0)
    {
        fmtdesc.index++;
        printf(" <%d> pixelformat = \"%c%c%c%c\", description = %s\n",fmtdesc.index,
                fmtdesc.pixelformat & 0xFF,
                (fmtdesc.pixelformat >> 8) & 0xFF,
                (fmtdesc.pixelformat >> 16) & 0xFF,
                (fmtdesc.pixelformat >> 24) & 0xFF,
                fmtdesc.description);
    }

    /* set video formats. */
    CLEAR (fmt);
    char * p = (char *)(&fmt.fmt.pix.pixelformat);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl (fd, VIDIOC_G_FMT, &fmt) < 0) {
        /* Errors ignored. */
        printf("get fmt fail\n");
    }

    fmt.fmt.pix.width       = mdata->cam_w;
    fmt.fmt.pix.height      = mdata->cam_h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

    if (-1 == ioctl (fd, VIDIOC_S_FMT, &fmt)){
        printf("set format fail\n");
        return -1;
    }
    if (ioctl (fd, VIDIOC_G_FMT, &fmt) < 0) {
        /* Errors ignored. */
        printf("get fmt fail\n");
    }

    printf("fmt.type = %d\n", fmt.type);
    printf("fmt.width = %d\n", fmt.fmt.pix.width);
    printf("fmt.height = %d\n", fmt.fmt.pix.height);
    printf("fmt.format = %c%c%c%c\n", p[0], p[1], p[2], p[3]);
    printf("fmt.field = %d\n", fmt.fmt.pix.field);
    printf("fps = %d\n", fps);
    printf("=======================================\n");
    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;

    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    CLEAR (req);
    req.count   = FRAMES;
    req.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory  = V4L2_MEMORY_MMAP;

    if (-1 == ioctl (fd, VIDIOC_REQBUFS, &req)) {
        fprintf (stderr, "VIDIOC_QUERYCAP fail\n");
        goto err;
    }

    if (req.count < 2) {
        fprintf (stderr, "Insufficient buffer memory\n");
        return -1;
    }

    mdata->buffers[cam_idx] = (struct buffer *)calloc(req.count, sizeof(struct buffer));
    if (!mdata->buffers[cam_idx]) {
        fprintf (stderr, "Out of memory\n");
        return -1;
    }
    if (req.count != FRAMES) {
        printf("Quest %d buffers failed, actual %d\n",req.count, FRAMES);
        return -1;
    }

    mdata->n_buffers = req.count;

    for (int i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;

        if (-1 == ioctl (fd, VIDIOC_QUERYBUF, &buf)){
            fprintf (stderr, "VIDIOC_QUERYCAP fail\n");
            goto err;
        }

        mdata->buffers[cam_idx][i].length = buf.length;
        mdata->buffers[cam_idx][i].start = mmap (NULL /* start anywhere */,
                buf.length,
                PROT_READ | PROT_WRITE /* required */,
                MAP_SHARED /* recommended */,
                fd, buf.m.offset);

        if (MAP_FAILED == mdata->buffers[cam_idx][i].start){
            fprintf (stderr, "mmap fail\n");
            goto err;
        }

#if 0
        cudaError_t status = cudaHostGetDevicePointer((void **)&(mdata->buffers[cam_idx][i].d_addr),  (void *)mdata->buffers[cam_idx][i].start , 0);
        printf(" status %d\n", status);
        check_error(status);
#endif
    }

    return 0;

err:
    return -1;
}
static void errno_exit (const char *s)
{
    fprintf (stderr, "%s error %d, %s\n",s, errno, strerror (errno));
    exit (EXIT_FAILURE);
}
static void start_capturing(peye mdata, int cam_idx)
{
    unsigned int i,ret;
    int fd = mdata->camera_fd[cam_idx];
    enum v4l2_buf_type type;

    for (i = 0; i < mdata->n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;
        ret = ioctl(fd, VIDIOC_QBUF, &buf);
        if (-1 == ret)
            errno_exit ("VIDIOC_QBUF");
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd, VIDIOC_STREAMON, &type);
    if (-1 == ret) 
        errno_exit ("VIDIOC_STREAMON");
}
/*
 * read a frame of image from video device
 */
static int get_buffer(peye mdata, struct v4l2_buffer *qbuf, int cam_idx)
{
    if (ioctl(mdata->camera_fd[cam_idx], VIDIOC_DQBUF, qbuf))
        return -1;

    return 0;
}

/*
 * enqueue the frame again
 */
static int put_buffer(peye mdata, struct v4l2_buffer *qbuf, int cam_idx)
{
    return ioctl(mdata->camera_fd[cam_idx], VIDIOC_QBUF, qbuf);
}

int getBufferIdx(peye mdata, int *index)
{
    int idx = *index;
    int i = 0;
    while ((idx & 1) && (i <= QUEUES))
    {
        i++;
        idx = idx >> 1;

    }

    if (i == QUEUES) {
        printf(" No free idx : %x\n", i);
        return -1;
    }

//    printf("I; %d\n", i);
    *index = *index | (1 << i);
    return i;
}

void putBufferIdx(peye mdata, int* index, int idx)
{
    *index = *index & (~(1 << idx));
}


static void aquireFrameThread_3(void *data)
{
    //struct timeval tv0;
    //struct timeval tv1;
    //struct timezone tz;

    peye mdata = (peye)data;

    // Cam idx
    int cam_idx = 3;

    struct pollfd fds[1];
    int r;
    int buf_idx;

    fds[0].fd = mdata->camera_fd[cam_idx];
    fds[0].events = POLLIN;

    mdata->qbuf[cam_idx]->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mdata->qbuf[cam_idx]->memory = V4L2_MEMORY_MMAP;

    while (poll(fds, 1, 5000) > 0) {
        if (fds[0].revents & POLLIN) {
            int idx = -1;

            r = get_buffer(mdata, mdata->qbuf[cam_idx], cam_idx);
            if (r)
                printf("error get buffer from camera r1= %d\n",r);

            buf_idx = mdata->qbuf[cam_idx]->index;

            pthread_mutex_lock(&mdata->mutex);
            if ((mdata->detect_idx & QUEUES_MASK) != QUEUES_MASK) {
                idx = getBufferIdx(mdata, &mdata->detect_idx);
            }
            pthread_mutex_unlock(&mdata->mutex);

            if (idx != -1) {
                cuda_push_char_array(mdata->dev_src + (mdata->cam_w * mdata->cam_h * 2) * cam_idx, (char *)mdata->buffers[cam_idx][buf_idx].start, mdata->cam_w * mdata->cam_h * 2);
                cudayuv2rgb(mdata->dev_dst + (mdata->cam_w * mdata->cam_h * mdata->cam_c) * cam_idx, mdata->dev_src + (mdata->cam_w * mdata->cam_h * 2) * cam_idx, mdata->cam_w, mdata->cam_h);
            }

            pthread_mutex_lock(&mdata->mutex);
            if (idx != -1) {
                cuda_resize(mdata->dev_dst + (mdata->cam_w * mdata->cam_h * mdata->cam_c) * cam_idx, mdata->reim[idx].data, mdata->cam_w, mdata->cam_h, mdata->net->w, mdata->net->h);
                idx = idx << 8 | cam_idx;
                printf(" %s, idx: %d \n", __func__, idx >> 8);
                EnQueue(&mdata->detectQueue, idx);
                pthread_cond_signal(&mdata->cond_detect);
            }
            pthread_mutex_unlock(&mdata->mutex);

           r = put_buffer(mdata, mdata->qbuf[cam_idx], cam_idx);
            if (r)
                printf("error get buffer from camera r3= %d\n",r);

        } else {
            printf("Not Pollin event\n");
        }

    }
        printf("Error: waiting for buffer of camera: %d timeout\n", cam_idx);
}

static void aquireFrameThread_2(void *data)
{
    //struct timeval tv0;
    //struct timeval tv1;
    //struct timezone tz;

    peye mdata = (peye)data;

    // Cam idx
    int cam_idx = 2;

    struct pollfd fds[1];
    int r;
    int buf_idx;

    fds[0].fd = mdata->camera_fd[cam_idx];
    fds[0].events = POLLIN;

    mdata->qbuf[cam_idx]->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mdata->qbuf[cam_idx]->memory = V4L2_MEMORY_MMAP;

    while (poll(fds, 1, 5000) > 0) {
        if (fds[0].revents & POLLIN) {
            int idx = -1;

            r = get_buffer(mdata, mdata->qbuf[cam_idx], cam_idx);
            if (r)
                printf("error get buffer from camera r1= %d\n",r);

            buf_idx = mdata->qbuf[cam_idx]->index;

            pthread_mutex_lock(&mdata->mutex);
            if ((mdata->detect_idx & QUEUES_MASK) != QUEUES_MASK) {
                idx = getBufferIdx(mdata, &mdata->detect_idx);
            }
            pthread_mutex_unlock(&mdata->mutex);

            if (idx != -1) {
                cuda_push_char_array(mdata->dev_src + (mdata->cam_w * mdata->cam_h * 2) * cam_idx, (char *)mdata->buffers[cam_idx][buf_idx].start, mdata->cam_w * mdata->cam_h * 2);
                cudayuv2rgb(mdata->dev_dst + (mdata->cam_w * mdata->cam_h * mdata->cam_c) * cam_idx, mdata->dev_src + (mdata->cam_w * mdata->cam_h * 2) * cam_idx, mdata->cam_w, mdata->cam_h);
            }

            pthread_mutex_lock(&mdata->mutex);
            if (idx != -1) {
                cuda_resize(mdata->dev_dst + (mdata->cam_w * mdata->cam_h * mdata->cam_c) * cam_idx, mdata->reim[idx].data, mdata->cam_w, mdata->cam_h, mdata->net->w, mdata->net->h);
                idx = idx << 8 | cam_idx;
                printf(" %s, idx: %d \n", __func__, idx >> 8);
                EnQueue(&mdata->detectQueue, idx);
                pthread_cond_signal(&mdata->cond_detect);
            }
            pthread_mutex_unlock(&mdata->mutex);

           r = put_buffer(mdata, mdata->qbuf[cam_idx], cam_idx);
            if (r)
                printf("error get buffer from camera r3= %d\n",r);

        } else {
            printf("Not Pollin event\n");
        }

    }
        printf("Error: waiting for buffer of camera: %d timeout\n", cam_idx);
}

static void aquireFrameThread_1(void *data)
{
    struct timeval tv0;
    struct timeval tv1;
    struct timezone tz;

    peye mdata = (peye)data;

    // Cam idx
    int cam_idx = 1;

    struct pollfd fds[1];
    int r;
    int buf_idx;

    fds[0].fd = mdata->camera_fd[cam_idx];
    fds[0].events = POLLIN;

    mdata->qbuf[cam_idx]->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mdata->qbuf[cam_idx]->memory = V4L2_MEMORY_MMAP;

    while (poll(fds, 1, 5000) > 0) {
        if (fds[0].revents & POLLIN) {
            r = get_buffer(mdata, mdata->qbuf[cam_idx], cam_idx);
            if (r)
                printf("error get buffer from camera r1= %d\n",r);

            buf_idx = mdata->qbuf[cam_idx]->index;

//            printf(" %s, %d\n", __func__, __LINE__);
            gettimeofday(&tv0, &tz);
            //            for (int i = 0; i < CAM_COUNT; i++) {
            //            }
            gettimeofday(&tv1, &tz);
            //            printf(" %s, total cost time: %lu us, buf_idx: %d\n", __func__, (tv1.tv_usec - tv0.tv_usec), buf_idx);

            /* splice 0, 1, 2, 3 to 4 */
            //            cuda_splice(mdata->dev_src, mdata->cam_w, mdata->cam_h);
            //cudayuv2rgb(mdata->dev_dst, mdata->dev_src + mdata->cam_w * mdata->cam_h * 2 * 4, mdata->cam_w * 2, mdata->cam_h * 2);

            int idx = -1;
            pthread_mutex_lock(&mdata->mutex);
            printf(" %s, %d\n", __func__, __LINE__);
            if ((mdata->detect_idx & QUEUES_MASK) != QUEUES_MASK) {
                idx = getBufferIdx(mdata, &mdata->detect_idx);
            }
            pthread_mutex_unlock(&mdata->mutex);

            printf(" %s, %d\n", __func__, __LINE__);
            if (idx != -1) {
                printf("%s  get buff idx: %d ===== \n", __func__, idx);
                cuda_push_char_array(mdata->dev_src + (mdata->cam_w * mdata->cam_h * 2) * cam_idx, (char *)mdata->buffers[cam_idx][buf_idx].start, mdata->cam_w * mdata->cam_h * 2);
            printf(" %s, %d\n", __func__, __LINE__);
                cudayuv2rgb(mdata->dev_dst + (mdata->cam_w * mdata->cam_h * mdata->cam_c) * cam_idx, mdata->dev_src + (mdata->cam_w * mdata->cam_h * 2) * cam_idx, mdata->cam_w, mdata->cam_h);
            }
            printf(" %s, %d\n", __func__, __LINE__);

            pthread_mutex_lock(&mdata->mutex);
            if (idx != -1) {
            printf(" %s, %d\n", __func__, __LINE__);
                cuda_resize(mdata->dev_dst + (mdata->cam_w * mdata->cam_h * mdata->cam_c) * cam_idx, mdata->reim[idx].data, mdata->cam_w, mdata->cam_h, mdata->net->w, mdata->net->h);
                idx = idx << 8 | cam_idx;
                printf(" %s, idx: %d \n", __func__, idx >> 8);
                EnQueue(&mdata->detectQueue, idx);
                pthread_cond_signal(&mdata->cond_detect);
            }
            pthread_mutex_unlock(&mdata->mutex);

            printf(" %s, %d\n", __func__, __LINE__);
           r = put_buffer(mdata, mdata->qbuf[cam_idx], cam_idx);
            if (r)
                printf("error get buffer from camera r3= %d\n",r);

        } else {
            printf("Not Pollin event\n");
        }

    }
        printf("Error: waiting for buffer of camera: %d timeout\n", cam_idx);
}

static void aquireFrameThread_0(void *data)
{
    //struct timeval tv0;
    //struct timeval tv1;
    //struct timezone tz;

    peye mdata = (peye)data;

    // Cam idx
    int cam_idx = 0;

    struct pollfd fds[1];
    int r;
    int buf_idx;

    fds[0].fd = mdata->camera_fd[cam_idx];
    fds[0].events = POLLIN;

    mdata->qbuf[cam_idx]->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mdata->qbuf[cam_idx]->memory = V4L2_MEMORY_MMAP;

    while (poll(fds, 1, 5000) > 0) {
        if (fds[0].revents & POLLIN) {
            int idx = -1;

            r = get_buffer(mdata, mdata->qbuf[cam_idx], cam_idx);
            if (r)
                printf("error get buffer from camera r1= %d\n",r);

            buf_idx = mdata->qbuf[cam_idx]->index;

            pthread_mutex_lock(&mdata->mutex);
            if ((mdata->detect_idx & QUEUES_MASK) != QUEUES_MASK) {
                idx = getBufferIdx(mdata, &mdata->detect_idx);
            }
            pthread_mutex_unlock(&mdata->mutex);

            printf(" %s, %d buf_idx:%d, cam_idx:%d, idx: %d\n", __func__, __LINE__, buf_idx, cam_idx, idx);
            if (idx != -1) {
                cuda_push_char_array(mdata->dev_src + (mdata->cam_w * mdata->cam_h * 2) * cam_idx, (char *)mdata->buffers[cam_idx][buf_idx].start, mdata->cam_w * mdata->cam_h * 2);
                cudayuv2rgb(mdata->dev_dst + (mdata->cam_w * mdata->cam_h * mdata->cam_c) * cam_idx, mdata->dev_src + (mdata->cam_w * mdata->cam_h * 2) * cam_idx, mdata->cam_w, mdata->cam_h);
            }

            printf(" %s, %d, buf_idx:%d, cam_idx%d, idx: %d\n", __func__, __LINE__, buf_idx, cam_idx, idx);
            pthread_mutex_lock(&mdata->mutex);
            if (idx != -1) {
                cuda_resize(mdata->dev_dst + (mdata->cam_w * mdata->cam_h * mdata->cam_c) * cam_idx, mdata->reim[idx].data, mdata->cam_w, mdata->cam_h, mdata->net->w, mdata->net->h);
                idx = idx << 8 | cam_idx;
                printf(" %s, idx: %d \n", __func__, idx >> 8);
                EnQueue(&mdata->detectQueue, idx);
                pthread_cond_signal(&mdata->cond_detect);
            }
            pthread_mutex_unlock(&mdata->mutex);

            printf(" %s, %d buf_idx:%d, cam_idx: %d, idx: %d\n", __func__, __LINE__, buf_idx, cam_idx, idx >> 8);
           r = put_buffer(mdata, mdata->qbuf[cam_idx], cam_idx);
            if (r)
                printf("error get buffer from camera r3= %d\n",r);

        } else {
            printf("Not Pollin event\n");
        }

    }
        printf("Error: waiting for buffer of camera: %d timeout\n", cam_idx);
}

static void detect_thread(void *data)
{
    struct timeval tv0;
    struct timeval tv1;
    struct timezone tz;

    peye mdata = (peye)data;

    while(true) {

        int idx = -1;
        int cam_idx = -1;

        pthread_mutex_lock(&mdata->mutex);
        if ((mdata->detectQueue.size == 0)) {
            pthread_cond_wait(&mdata->cond_detect, &mdata->mutex);
        }

        idx = DeQueue(&mdata->detectQueue);
        cam_idx = idx & 0xFF;
        idx = idx >> 8;

        printf(" %d, camIdx: %d\n", idx, cam_idx);

       pthread_mutex_unlock(&mdata->mutex);
#if 0
        if (cam_idx == 0) {
            putBufferIdx(mdata, &mdata->detect_idx, idx);
            continue;
        }
#endif
        gettimeofday(&tv0, &tz);
            printf(" %s, %d\n", __func__, __LINE__);
        cudabgrtorgb(mdata->feedim.data, mdata->reim[idx].data, mdata->net->w, mdata->net->h);

            printf(" %s, %d\n", __func__, __LINE__);
        cudaswap(mdata->feedim.data, mdata->net->w, mdata->net->h);

            printf(" %s, %d\n", __func__, __LINE__);
        float *prediction = network_predict(*mdata->net, mdata->feedim.data);

            printf(" %s, %d\n", __func__, __LINE__);
        cudaShowCon(mdata->showImg[cam_idx]->imageData, mdata->reim[idx].data, mdata->net->w, mdata->net->h);

            printf(" %s, %d\n", __func__, __LINE__);
        pthread_mutex_lock(&mdata->mutex);
        putBufferIdx(mdata, &mdata->detect_idx, idx);
        pthread_mutex_unlock(&mdata->mutex);

#if 0
        memcpy(predictions[idx], prediction, mdata->l->outputs*sizeof(float));
        mean_arrays(predictions, FRAMES, mdata->l->outputs, avg);
        mdata->l->output = avg;
#endif
        mdata->l->output = prediction;

//        cudaDeviceSynchronize();
        if(mdata->l->type == DETECTION){
            get_detection_boxes(*mdata->l, 1, 1, mdata->thresh, mdata->probs, mdata->boxes, 0);
        } else if (mdata->l->type == REGION){
//            printf(" %s, %d\n", __func__, __LINE__);
            printf(" %s, %d\n", __func__, __LINE__);
           get_region_boxes(*mdata->l, 1, 1, mdata->thresh, mdata->probs, mdata->boxes, 0, 0, mdata->hier_thresh, mdata->thresh_hand);
        } else {
            error("Last layer must produce detections\n");
        }
        float nms = .4;
        if (mdata->nms > 0) ;
            printf(" %s, %d\n", __func__, __LINE__);
        do_nms(mdata->boxes, mdata->probs, mdata->l->w * mdata->l->h * mdata->l->n, mdata->l->classes, nms);
        //printf("\033[2J");
        //printf("\033[1;1H");
        //printf("\nFPS:%.1f\n",fps);
        printf("Objects:\n\n");


        gettimeofday(&tv1, &tz);
        fetch_detections_result(mdata,  mdata->l->w * mdata->l->h * mdata->l->n, mdata->thresh, mdata->boxes, mdata->probs, lable_names, mdata->classes, cam_idx, mdata->thresh_hand);

        char vname[20];
        sprintf(vname, "Camera: %d", cam_idx);
        cvText(mdata->showImg[cam_idx], vname, 8, 8);

        cvShowImage(mdata->cameraName[cam_idx], mdata->showImg[cam_idx]);
        cvWaitKey(1);

        printf(" %s, network_predict cost time: %lu us idx:%d, cam_idx: %d\n", __func__, (tv1.tv_usec - tv0.tv_usec), idx, cam_idx);
        if (mdata->debug)
            printf(" %s, network_predict cost time: %lu ms idx:%d\n", __func__, (tv1.tv_usec - tv0.tv_usec), idx);

    }

}

void draw_detections_rect(peye mdata, image a, box *rbox)
{
    int i , j;

	int x1, y1, x2, y2;
	int w = a.w * 0.012;
	int r = 178, g = 34, b = 34;
	x1 = rbox->x;
	y1 = rbox->y;
	x2 = x1 + rbox->w;
	y2 = y1 + rbox->h;
    printf(" %s x1 : %d, y1: %d, x2: %d, y2: %d, w: %d\n", __func__, x1, y1, x2, y2, w);

    if(x1 < 0) x1 = 0;
    if(x1 >= a.w) x1 = a.w-1;
    if(x2 < 0) x2 = 0;
    if(x2 >= a.w) x2 = a.w-1;

    if(y1 < 0) y1 = 0;
    if(y1 >= a.h) y1 = a.h-1;
    if(y2 < 0) y2 = 0;
    if(y2 >= a.h) y2 = a.h-1;

    for (j = x1; j < x2; j++) {
        for (i = 0; i < w; ++i)
        {
            a.data[j * a.c + (y1 + i) * a.w * a.c + 0] = r;
            a.data[j * a.c + (y1 + i) * a.w * a.c + 1] = g;
            a.data[j * a.c + (y1 + i) * a.w * a.c + 2] = b;
        }
        for (i = 0; i < w; ++i)
        {
            a.data[j * a.c + (y2 - i) * a.w * a.c + 0] = r;
            a.data[j * a.c + (y2 - i) * a.w * a.c + 1] = g;
            a.data[j * a.c + (y2 - i) * a.w * a.c + 2] = b;
        }
    }
    for (j = y1; j < y2; j++) {
        for (i = 0; i < w; ++i)
        {
            a.data[j * a.w * a.c + (x1 + i) * a.c + 0] = r;
            a.data[j * a.w * a.c + (x1 + i) * a.c + 1] = g;
            a.data[j * a.w * a.c + (x1 + i) * a.c + 2] = b;
        }
        for (i = 0; i < w; ++i)
        {
            a.data[j * a.w * a.c + (x2 - i) * a.c + 0] = r;
            a.data[j * a.w * a.c + (x2 - i) * a.c + 1] = g;
            a.data[j * a.w * a.c + (x2 - i) * a.c + 2] = b;
        }
    }
}

void cvText(IplImage* img, const char* text, int x, int y)  
{
    CvFont font;
    double hscale = 0.3;
    double vscale = 0.3;
    int linewidth = 0.05;

    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,hscale,vscale,0,linewidth, 0);
//    CvScalar textColor =cvScalar(100,240,106, 106);
    CvScalar textColor =CV_RGB(0,255,255);
    CvPoint textPos =cvPoint(x, y);
    cvPutText(img, text, textPos, &font,textColor);
}

void fetch_detections_result(peye mdata, int num, float thresh, box *boxes, float **probs, char **names, int classes, int cam_idx, float thresh_hand)
{
    int i, j = 0;
    int w = mdata->net->w;
    int h = mdata->net->h;

    for(i = 0; i < num; ++i){
        int class = max_index(probs[i], classes);
        float prob = probs[i][class];
//            printf(" %s, %d, w: %d, h: %d i:  %d class: %d, prob: %f\n", __func__, __LINE__, w, h, i, class, prob);
        if((prob > thresh) || (prob > thresh_hand && class ==3)){
            if ((prob*100 > 0) && (prob*100 <= 100)) {
                char temp[50];
                //printf("%s\t: %.0f%%, class:%d\n",names[class], prob*100, class);
                //printf("%s\t: %.0f%%\n",names[class], prob*100);
                if (class > sizeof(lable_names));

                printf("kunta:prob: %f, thresh: %f, thresh_hand: %f %s\t: %.0f%%\n", prob, thresh, thresh_hand, lable_names[class], prob*100);

#if 1
                box b = boxes[i];


                int left  = (b.x-b.w/2.)*w;
                int right = (b.x+b.w/2.)*w;
                int top   = (b.y-b.h/2.)*h;
                int bot   = (b.y+b.h/2.)*h;

				left += 4;
				top += 4;
				right += 4;
				bot += 4;
                if(left < 0) left = 0;
                if(right > w-1) right = w-1;
                if(top < 0) top = 0;
                if(bot > h-1) bot = h-1;

                sprintf(temp, "%d: %s:%d %.0f%% l:%d, r:%d, t:%d, b:%d", cam_idx, names[class],j++,  prob*100, left, right, top, bot);

#ifdef DBUS
                //camera_without_replay(connection, temp);
				send_object_messages(connection, temp);
#endif
//            printf(" %s, %d\n", __func__, __LINE__);
				char vname[20];
				sprintf(vname, "Camera: %d", cam_idx);
				cvText(mdata->showImg[cam_idx], vname, 8, 8);

                cvRectangle(mdata->showImg[cam_idx], cvPoint(left,top), cvPoint(right, bot), CV_RGB( rand()&255, rand()&255, rand()&255),2,1,0);
                if (top < 4)
                    top = 4;
                
                cvText(mdata->showImg[cam_idx], temp, left, top - 4);

#endif
            } else {
                printf("Error:  detection error\n");
            }

        }
    }
            printf(" %s, %d\n", __func__, __LINE__);
}

int v4l2_poll(peye mdata, struct v4l2_buffer *qbuf, int cam_idx) {
    struct pollfd fds[1];
    int r;

    fds[0].fd = mdata->camera_fd[cam_idx];
    fds[0].events = POLLIN;

    qbuf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    qbuf->memory = V4L2_MEMORY_MMAP;

    while (poll(fds, 1, 5000) > 0) {
        if (fds[0].revents & POLLIN) {


            r = get_buffer(mdata, qbuf, cam_idx);
            if (r)
                printf("error get buffer from camera r1= %d\n",r);

            return qbuf->index;

        } else {
            printf("Not Pollin event\n");
        }
    }

    printf("Error: waiting for buffer of camera: %d timeout\n", cam_idx);
    return -1;
}
  

#if 1
int base(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, int classes, int frame_skip, char *prefix, float hier_thresh, float thresh_hand)
{
    image **alphabet = load_alphabet();
    float nms = .5;

    printf("Demo\n");

    peye mdata = (peye)calloc(sizeof(eye), 1);


#ifdef DBUS
    if(init_session_bus(connection, "com.pi.cameraSensorConnect")){
        printf("bus connection fail\n");
        return -1;
    }
#endif

    network net = parse_network_cfg(cfgfile);
    mdata->net = &net;
    if(weightfile){
        load_weights(&net, weightfile);
    }
    set_batch_network(&net, 1);

    srand(2222222);

    layer l = net.layers[net.n-1];
    mdata->l = &l;


    avg = (float *) calloc(l.outputs, sizeof(float));
    for(int j = 0; j < FRAMES; ++j) predictions[j] = (float *) calloc(l.outputs, sizeof(float));
    for(int j = 0; j < FRAMES; ++j) images[j] = make_image(1,1,3);

    mdata->boxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
    mdata->probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
    for(int j = 0; j < l.w*l.h*l.n; ++j)
        mdata->probs[j] = (float *)calloc(l.classes, sizeof(float));


    for (int i = 0; i < QUEUES; i++) {
		//cudaMallocHost((void**)&(mdata->reim[i].data), net.w * net.h * net.c * sizeof(float));
        mdata->reim[i].data = cuda_make_float_array(net.w * net.h * net.c);
        mdata->reim[i].w = net.w;
        mdata->reim[i].h = net.h;
        mdata->reim[i].c = net.c;
    }

	//cudaMallocHost((void**)&(mdata->feedim.data), net.w * net.h * net.c * sizeof(float));
    mdata->feedim.data = cuda_make_float_array(net.w * net.h * net.c);
    mdata->feedim.w = net.w;
    mdata->feedim.h = net.h;
    mdata->feedim.c = net.c;
    //	cudaMallocHost((void**)&(mdata->showim.data), net.w * net.h * net.c * sizeof(float));
#if 0
    for (int k = 0; k < CAM_COUNT; k++) {
        mdata->showim[k].data = cuda_make_float_array(net.w * net.h * net.c);
        mdata->showim[k].w = net.w;
        mdata->showim[k].h = net.h;
        mdata->showim[k].c = net.c;
    }
#endif
    mdata->n_buffers = FRAMES; //default
    mdata->cam_h = CAM_H;
    mdata->cam_w = CAM_W;
    mdata->cam_c = CAM_C;
    mdata->cam_fps = 30;
    mdata->debug = 0;
    mdata->nms = nms;

    mdata->names = names;
    mdata->alphabet = alphabet;
    mdata->hier_thresh = hier_thresh;
    mdata->classes = classes;
    mdata->thresh = thresh;
    mdata->thresh_hand = thresh_hand;
    mdata->thresh_hand = 0.24;

    mdata->detect_idx = 0;


    InitCUDA();

    int ret, fd;

    for (int i = 0; i < CAM_COUNT; i++) {
        sprintf(mdata->cameraName[i], "%s%d","/dev/video" , i);

        fd =open(mdata->cameraName[i], O_RDWR /* required */ | O_NONBLOCK, 0);
        if(fd < 0){
            printf("open camera failed\n");
            close(fd);
            return -1;
        }

        mdata->qbuf[i] = (struct v4l2_buffer *)calloc(1 ,sizeof(struct v4l2_buffer));
        mdata->camera_fd[i] = fd;

        if (init_device(mdata, i))
            printf(" init camera failed\n");

        printf("start capturing camera: %d\n", fd);
        start_capturing(mdata, i);

#if 1
        cvNamedWindow(mdata->cameraName[i], CV_WINDOW_NORMAL); 
        cvMoveWindow(mdata->cameraName[i], 0, 0);
        cvResizeWindow(mdata->cameraName[i], mdata->net->w + 50, mdata->net->h + 50);
#endif
    }

    //struct timeval tv0;
    //struct timeval tv1;
    //1struct timezone tz;

    InitQueue(&mdata->detectQueue);

    printf("\ncam_W: %d cam_h: %d, net->w: %d net->h: %d, queue size: %u\n", mdata->cam_w, mdata->cam_h,net.w, net.h, FRAMES);

    for (int i = 0; i < CAM_COUNT; i++) {
        mdata->showImg[i] = cvCreateImage(cvSize(net.w, net.h), 8, 3);
        cudaMallocHost((void **)&mdata->showImg[i]->imageData, net.w * net.h * net.c);
    }

#if 0
    if(!prefix) {
        cvNamedWindow(mdata->cameraName, CV_WINDOW_NORMAL); 
        cvMoveWindow(mdata->cameraName, 0, 0);
        cvResizeWindow(mdata->cameraName, 1600, 900);
    }
#endif
    pthread_mutex_init(&mdata->mutex,NULL);
    pthread_cond_init(&mdata->cond_detect,   NULL);


    float *dev_dst = cuda_make_float_array(mdata->cam_w * mdata->cam_h * mdata->cam_c * CAM_COUNT);
    if (!dev_dst) {
        printf ("Malloc cuda buffer error!\n");
        return -1;
    }

    char *dev_src = cuda_make_char_array(mdata->cam_w * mdata->cam_h * 2 * CAM_COUNT);
    if (!dev_src) {
        printf ("Malloc cuda buffer error!\n");
        return -1;
    }

    mdata->dev_src = dev_src;
    mdata->dev_dst = dev_dst;

    ret=pthread_create(&mdata->id_detect,NULL,(void *) detect_thread, mdata);
    if(ret) {
        printf ("Create pthread error!\n");
        return ret;
    }

    mdata->ready = false;
    mdata->ready_idx = -1;

    mdata->threadFunc[0] = aquireFrameThread_0;
    mdata->threadFunc[1] = aquireFrameThread_1;
    mdata->threadFunc[2] = aquireFrameThread_2;
    mdata->threadFunc[3] = aquireFrameThread_3;
#if 1
    for (int i = 0; i < CAM_COUNT; i++) {
        ret=pthread_create(&mdata->aquireFrame[i],NULL, (void *)mdata->threadFunc[i], mdata);
        if(ret) {
            printf ("Create pthread error!\n");
            return ret;
        }
    }
#endif

    while(true);

    printf(" main exit \n");
    for(int j = 0; j < l.w*l.h*l.n; ++j)
        free(mdata->probs[j]);

    for (int i = 0; i < QUEUES; i++) {
		//cudaFreeHost(mdata->reim[i].data);
        cuda_free(mdata->reim[i].data);
    }
    cuda_free(mdata->feedim.data);
    for (int i = 0; i < CAM_COUNT; i++) {
        cuda_free(mdata->showim[i].data);
    }

    DelQueue(&mdata->detectQueue);

    for (int i = 0; i < CAM_COUNT; i++) {
        cuda_free(mdata->showim[i].data);
        cudaFreeHost(mdata->showImg[i]->imageData);
        cvReleaseImage(&mdata->showImg[i]);
    }

//	cudaFreeHost(mdata->showImg->imageData);
//    cvReleaseImage(&mdata->showImg);

    free(mdata->boxes);
    free(mdata->probs);
    free(mdata->buffers);

    cuda_free(mdata->dev_dst);
    cuda_char_free(mdata->dev_src);

    free(mdata);

    return 0;
}
#else
void base(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, int classes, int frame_skip, char *prefix, float hier_thresh)
{
    fprintf(stderr, "Demo needs OpenCV for webcam images.\n");
}
#endif

