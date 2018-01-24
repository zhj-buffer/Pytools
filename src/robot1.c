
#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include "parser.h"
#include "utils.h"
#include "cuda.h"
#include "blas.h"
#include "connected_layer.h"

#include "robot.h"
#include "dbus.h"

DBusConnection * connection;

#ifdef OPENCV
#include "opencv2/highgui/highgui_c.h"
#endif

static network net;
static float **probs;
static box *boxes;

//static char *voc_names[] = {"aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};
static char *voc_names[] = {"head", "bicycle", "bird", "hand", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};

int draw_vText(IplImage* img, const char* text, int x, int y)
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

int get_detections_result(pmnet mdata, IplImage *showImg)
    //peye mdata, int num, float thresh, box *boxes, float **probs, char **names, int classes, int cam_idx, float thresh_hand)
{
    int i, j = 0;
    int w = mdata->net->w;
    int h = mdata->net->h;
//    printf("%s, %d\n", __func__, __LINE__);
    box *boxes = mdata->boxes;
//    printf("%s, %d\n", __func__, __LINE__);
    float **probs = mdata->probs;
    char **names = mdata->names;
//    printf("%s, %d\n", __func__, __LINE__);
    int classes = mdata->classes;
//    printf("%s, %d\n", __func__, __LINE__);
    int cam_idx = mdata->cam_idx;
    float thresh_hand = mdata->thresh_hand;

//    layer l = net.layers[net.n-1];

//    printf("%s, %d, l.w: %d, l.h: %d, l.n: %d\n", __func__, __LINE__, l.w, net.h, net.n);
    int num = mdata->num;
//    printf("%s, %d, num: %d\n", __func__, __LINE__, num);
    float thresh = mdata->thresh;

    int k = 0;
    char temp[20 * 5 * 6];

//    printf(" %s, num : %d\n", __func__, num);
    for(i = 0; i < num; ++i){
        int class = max_index(probs[i], classes);
        float prob = probs[i][class];
        probs[i][class] = 0.;
       //             printf(" %s, %d, w: %d, h: %d i:  %d class: %d, prob: %f\n", __func__, __LINE__, w, h, i, class, prob);
        if((prob > thresh) || (prob > thresh_hand && class ==3)){
            if ((prob*100 > 0) && (prob*100 <= 100)) {
                //printf("%s\t: %.0f%%, class:%d\n",names[class], prob*100, class);
                //printf("%s\t: %.0f%%\n",names[class], prob*100);
                if (class > sizeof(names));

          //      printf("kunta:prob: %f, thresh: %f, thresh_hand: %f %s\t: %.0f%%\n", prob, thresh, thresh_hand, names[class], prob*100);

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

                sprintf(temp, "%d:%d:%d %.2f%% l:%d t:%d r:%d b:%d", cam_idx, names[class],j++,  prob*100, left, top, right, bot);
                
                printf("name: %s cam_idx: %d l:%d, t:%d, r:%d, b:%d\n", names[class], cam_idx, left, top, right, bot);
                if (k < 20 && !strcmp(names[class], "head")) {
                    mdata->coordinate[k * 4 + 0] = left;
                    mdata->coordinate[k * 4 + 1] = top;
                    mdata->coordinate[k * 4 + 2] = right - left;
                    mdata->coordinate[k * 4 + 3] = bot - top;
                    k++;
                }

                camera_without_replay(connection, temp);
                //            printf(" %s, %d\n", __func__, __LINE__);
#if 1
                char vname[20];
                sprintf(vname, "Camera: %d", cam_idx);

                printf("%s, class: %d, cmp: %d\n", names[class], class, strcmp(names[class], "hand"));
                if (!strcmp(names[class], "hand" )) {
                    cvText(showImg, vname, 8, 8);
                    cvRectangle(showImg, cvPoint(left,top), cvPoint(right, bot), CV_RGB( rand()&255, rand()&255, rand()&255),2,1,0);
                    if (top < 4)
                        top = 4;
                    cvText(showImg, temp, left, top - 4);
                }
#endif
#endif
            } else {
                printf("Error:  detection error\n");
            }

        }
    }
//    printf(" %s, %d\n", __func__, __LINE__);
}

int object_detect_run(pmnet mdata,  float *buf)
{
    layer l = net.layers[net.n-1];
#if 0
    printf(" %s. mdata : %p, %p, buf: %d\n", __func__, mdata, buf, buf[0]);
    for (int i = 0; i < 40; i++)
        printf("%f\n ", buf[i]);

    printf("%s , w:%d, h%d, net: %p, classes: %d\n", __func__, mdata->net->w, mdata->net->h, mdata->net, mdata->classes);
#endif
    float *prediction =  network_predict_gpu(net, buf);

    l.output_gpu = prediction;
    //memcpy(l.output_gpu, prediction, l.outputs*sizeof(float));

    //get_region_boxes(*mdata->l, 1, 1, mdata->thresh, mdata->probs, mdata->boxes, 0, 0, mdata->hier_thresh, mdata->thresh_hand);
    get_region_boxes(l, 1, 1, mdata->thresh, mdata->probs, mdata->boxes, 0, 0, mdata->hier_thresh, mdata->thresh_hand);

    do_nms(mdata->boxes, mdata->probs, l.w * l.h * l.n, l.classes, mdata->nms);
    memset(l.output_gpu, 0, l.outputs*sizeof(float));

    return 0;
}

pmnet object_detect_init(int cam_idx)
{

    char *cfgfile = "../net/fast-eye.cfg";
//    char *weightfile = "../net/fast-eye.weights";
    //char *weightfile = "../net/eye_hand.weights";
    char *weightfile = "../net/eye_hand_manual_v2.weights";
    float thresh_hand = 0.24;
    float hier_thresh = 0.5;
    float thresh = 0.24;
    float nms = 0.4;

    int classes = 20;

    pmnet mdata = (pmnet)calloc(sizeof(mnet), 1);

    connection = connect_dbus();
    if(connection == NULL) {
        printf("bus connection fail\n");
        return -1;
    }

    net = parse_network_cfg(cfgfile);
    mdata->net = &net;
    if(weightfile){
        load_weights(&net, weightfile);
    }
    set_batch_network(&net, 1);

    printf("net: %d, h: %d, net: %p, &net %p\n", net.w, mdata->net->h, mdata->net, &net);
    srand(2222222);

    layer l = net.layers[net.n-1];
    mdata->l = &l;


    mdata->num = mdata->l->w * mdata->l->h * mdata->l->n;

    printf("%s, %d, l.w: %d, l.h: %d, l.n: %d, num: %d\n", __func__, __LINE__, mdata->l->w, mdata->l->h, mdata->l->n, mdata->num);
    boxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
    probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
    for(int j = 0; j < l.w*l.h*l.n; ++j)
        probs[j] = (float *)calloc(l.classes, sizeof(float));

    mdata->boxes = boxes;
    mdata->probs = probs;
    mdata->names = voc_names;


    mdata->debug = 0;
    mdata->nms = nms;

    mdata->hier_thresh = hier_thresh;
    mdata->classes = classes;
    mdata->thresh = thresh;
    mdata->thresh_hand = thresh_hand;
    mdata->cam_idx = cam_idx;

    InitCUDA();

    printf("%s , w:%d, h%d, net: %p, classes: %d\n", __func__, mdata->net->w, mdata->net->h, mdata->net, mdata->classes);

    printf("%s, %p\n", __func__, mdata);
	return mdata;
}
#ifdef __cplusplus
};
#endif

