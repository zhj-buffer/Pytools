
#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <math.h>
#include <gio/gio.h>

#include "parser.h"
#include "utils.h"
#include "cuda.h"
#include "blas.h"
#include "connected_layer.h"

#include "robot.h"
#include "dbus.h"
#include "motor.h"
#include "queue.h"

DBusConnection  *connection;
DBusMessage     *message;

//#define LOCAL_FOLLOW
#define MAX_SIZE (PATH_MAX+1)
#ifdef OPENCV
#include "opencv2/highgui/highgui_c.h"
#endif

//#define AUTO_FRAMERATE

static network net;
static float **probs;
static box *boxes;

#define BUFMAX 500
#define OLD 0
#define LAST 1
static char *temp;
static char oname[50];
static int buf_count = 0;

//static char *voc_names[] = {"aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};
//static char *voc_names[] = {"head", "bicycle", "bird", "hand", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};

static char *voc_names[] = {"head", "hand", "person"};
typedef struct pspos {
    int x;
    int y;
    int x0;
    int y0;
    int w;
    int h;
    int fresh;
    bool has;
    int count;
    int rate;
}pspos;

typedef struct hands {
    int x;
    int y;
    int x0;
    int y0;
    int w;
    int h;
    int cam_idx;
    int status;
    int fresh;
    int count;
}hand;

typedef struct HEAD {
    int x;
    int y;
    int x0;
    int y0;
    int w;
    int h;
    int cam_idx;
    int status;
    int fresh;
    int count;
}head;

typedef struct lines {
    int x;
    int y;
    int cam_idx;
    int status;
    int direction;
    int count;
    int dup;
    int stay;
    int lost;
    int fresh;
    int start;
    int used;
}line;

static pspos pps[4][2][10];
static int spos[2];
static int pflags[4];
static int hflags[4];
static int mstatus = 0;
static int ta = 60; // center of the person
static hand hands[4][10];
static hand last_hands[4][10];
static head heads[4][10];
static head last_heads[4][10];

#define L_W 10
#define L_H 10
#define L_STEP (40)
static line lines[4][L_W][L_H];

pmnet mdata;

const int CACHE_SIZE = 3;
const int HUMAN_GONE_THRESHOLD = 300;
struct Stat
{
    bool human_exist;
    int human_gone_begin;
};

struct Stat human_exist_stat;
int last_pose;
void init_stat()
{
    human_exist_stat.human_exist = false;
    human_exist_stat.human_gone_begin = 0;
}
char *human_exist_name = "detect_human_exist";
char *human_gone_name = "human_gone";

Queue human_exist_cache;

FILE *fp;

static int init_lines()
{
    for (int k = 0; k < 4; k++)
        for (int i = 0; i < L_H; i++)
            for (int j = 0; j < L_W; j++) {
                lines[k][i][j].x = j * L_STEP + 5;
                lines[k][i][j].y = i * L_STEP + 5;

                lines[k][i][j].cam_idx = k;
                lines[k][i][j].status = -1;
                lines[k][i][j].direction = -1;
                lines[k][i][j].lost = -1;
                lines[k][i][j].dup = -1;
                lines[k][i][j].stay = -1;
                lines[k][i][j].fresh = 0;
            }

}

void drawCvText(IplImage* img, const char* text, int x, int y)
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
{
    int i;
    int w = mdata->net->w;
    int h = mdata->net->h;
    box *boxes = mdata->boxes;
    float **probs = mdata->probs;
    char **names = mdata->names;
    int classes = mdata->classes;
    int cam_idx = mdata->cam_idx;
    float thresh_hand = mdata->thresh_hand;

    int num = mdata->num;
    float thresh = mdata->thresh;

    int k = 0;
    bool hasperson = false;
    bool hashead = false;
    bool human_exist = false;

    memset(temp, '\0', buf_count);
    int ppscount = 0;

    pps[cam_idx][OLD][ppscount].has = false;
    pps[cam_idx][OLD][ppscount].count = 0;

    int hand_idx = 0;
    int head_idx = 0;
    hands[cam_idx][0].count = 0;
    heads[cam_idx][0].count = 0;
    double distance = 0;
    int area = 0;
    int area_error = 0;
    const int MIN_STATIC_DISTANCE = 2;
    const int MIN_MOVE_DISTANCE = 200;
    const int MAX_MOVE_DISTANCE = 200;
    const int MIN_MOVE_AREA_ERROR = 4;
    const int MAX_MOVE_AREA_ERROR = 5000;
    const int OTHER = 0;
    const int STAND = 1;
    const int SIT = 2;
    const int LIE = 3;
    const int MOVE = 4;
    int pose;
    int max_area_person_id = 0;
    int max_area_person_cam_id = 0;
    int max_area;
    int P_THRESHOLD = 5;
    int H_THRESHOLD = 2;
    int positive = 0;

    for(i = 0; i < num; ++i){
        int class = max_index(probs[i], classes);
        float prob = probs[i][class];
        probs[i][class] = 0.;


        if((prob > thresh) || (prob > thresh_hand && class == 1)){
            if ((prob*100 > 0) && (prob*100 <= 100)) {
                if (class > sizeof(names));

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

                if (temp[0] == '\0') {
                    sprintf(temp, "%d,%s,%.0f%%,%d,%d,%d,%d", cam_idx, names[class], prob*100, left, top, right, bot);
                } else {
                    if ((strlen(temp) + 50) <= BUFMAX) {
                        sprintf(oname, "%d,%s,%.0f%%,%d,%d,%d,%d", cam_idx, names[class], prob*100, left, top, right, bot);
                        sprintf(temp, "%s|%s", temp, oname);
                    } else
                        printf("Warning....too many objects\n");
                }
                //printf("name: %s cam_idx: %d l:%d, t:%d, r:%d, b:%d\n", names[class], cam_idx, left, top, right, bot);
                if (k < 20 && !strcmp(names[class], "head")) {
                    mdata->coordinate[k * 4 + 0] = left;
                    mdata->coordinate[k * 4 + 1] = top;
                    mdata->coordinate[k * 4 + 2] = right - left;
                    mdata->coordinate[k * 4 + 3] = bot - top;

                    k++;
                }
#if 1
                char vname[20];
                sprintf(vname, "Camera: %d", cam_idx);
                drawCvText(showImg, vname, 8, 8);

                if (!strcmp(names[class], "hand" ) ) {
                    hands[cam_idx][hand_idx].x = left;
                    hands[cam_idx][hand_idx].y = right;
                    hands[cam_idx][hand_idx].x0 = (right - left) / 2 + left;
                    hands[cam_idx][hand_idx].y0 = (bot - top) / 2 + top;
                    hands[cam_idx][hand_idx].w = right - left;
                    hands[cam_idx][hand_idx].h = bot - top;
                    hands[cam_idx][0].count++;

                    for (int n=0; n < last_hands[cam_idx][0].count; n++) {
                        distance = sqrt(pow(hands[cam_idx][hand_idx].x0 - last_hands[cam_idx][n].x0, 2) + pow(hands[cam_idx][hand_idx].y0 - last_hands[cam_idx][n].y0, 2));
                        if (distance < MIN_STATIC_DISTANCE) {
                            //printf("hand distance < MIN STATIC DISTANCE: distance: %f \n", distance);
                        } else {
                            human_exist = true;
                        }
                    }
                    last_hands[cam_idx][hand_idx].x = hands[cam_idx][hand_idx].x;
                    last_hands[cam_idx][hand_idx].y = hands[cam_idx][hand_idx].y;
                    last_hands[cam_idx][hand_idx].x0 = hands[cam_idx][hand_idx].x0;
                    last_hands[cam_idx][hand_idx].y0 = hands[cam_idx][hand_idx].y0;
                    last_hands[cam_idx][hand_idx].w = hands[cam_idx][hand_idx].w;
                    last_hands[cam_idx][hand_idx].h = hands[cam_idx][hand_idx].h;
                    last_hands[cam_idx][0].count = hands[cam_idx][0].count;

                    hand_idx++;


                    drawCvText(showImg, "hand", left, top - 4);
                    cvRectangle(showImg, cvPoint(left,top), cvPoint(right, bot), CV_RGB( 255, 0, 0),2,1,0);
                    if (top < 4)
                        top = 4;
                }
                if (!strcmp(names[class], "person" ) ) {

                    pps[cam_idx][OLD][ppscount].x = left;
                    pps[cam_idx][OLD][ppscount].y = top;
                    pps[cam_idx][OLD][ppscount].x0 = (right - left) / 2 + left;
                    pps[cam_idx][OLD][ppscount].y0 = (bot - top) / 2 + top;
                    pps[cam_idx][OLD][ppscount].w = right - left;
                    pps[cam_idx][OLD][ppscount].h = bot - top;
                    pps[cam_idx][OLD][ppscount].has = true;
                    pps[cam_idx][OLD][ppscount].rate = prob*100;
                    pps[cam_idx][OLD][0].count++;

                    //area = pps[cam_idx][OLD][ppscount].w * pps[cam_idx][OLD][ppscount].h;
                    if (pps[cam_idx][LAST][0].has) {
                        for (int n = 0; n < pps[cam_idx][LAST][0].count; n++){
                            if (pps[cam_idx][LAST][n].has) {
                                distance = sqrt(pow(pps[cam_idx][OLD][ppscount].x0 - pps[cam_idx][LAST][n].x0, 2) + pow(pps[cam_idx][OLD][ppscount].y0 - pps[cam_idx][LAST][n].y0, 2));
                                //area_error = abs(area - pps[cam_idx][LAST][n].w * pps[cam_idx][LAST][n].h);
                                if (distance < MIN_STATIC_DISTANCE) {
                                    //printf("person distance < MIN STATIC DISTANCE: distance: %f \n", distance);
                                //} else if (distance > MAX_MOVE_DISTANCE) {
                                //    printf("distance > MAX MOVE DISTANCE: distance: %f \n", distance);
                                //} else if ((area_error < MIN_MOVE_AREA_ERROR) || (area_error > MAX_MOVE_AREA_ERROR)){
                                //    printf("area error too small or too big: %d, last: %d x %d, now %d x %d \n", area_error, pps[cam_idx][LAST][n].w,  pps[cam_idx][LAST][n].h, pps[cam_idx][OLD][ppscount].w, pps[cam_idx][OLD][ppscount].h);
                                } else {
                                    hasperson = true;
                                    human_exist = true;
                                    area = pps[cam_idx][OLD][ppscount].w * pps[cam_idx][OLD][ppscount].h;
                                    if (area > max_area) {
                                        max_area = area;
                                        max_area_person_id = ppscount;
                                        max_area_person_cam_id = cam_idx;
                                    }
                                    if (pps[cam_idx][OLD][ppscount].w > pps[cam_idx][OLD][ppscount].h){
                                        pose = LIE;
                                    } else if (pps[cam_idx][OLD][ppscount].w * 2 < pps[cam_idx][OLD][ppscount].h){
                                        pose = STAND;
                                    } else if (pps[cam_idx][OLD][ppscount].w < pps[cam_idx][OLD][ppscount].h){
                                        pose = SIT;
                                    }
                                }
                            }
                        }
                    }
                    pps[cam_idx][LAST][ppscount].x = pps[cam_idx][OLD][ppscount].x;
                    pps[cam_idx][LAST][ppscount].y = pps[cam_idx][OLD][ppscount].y;
                    pps[cam_idx][LAST][ppscount].x0 = pps[cam_idx][OLD][ppscount].x0;
                    pps[cam_idx][LAST][ppscount].y0 = pps[cam_idx][OLD][ppscount].y0;
                    pps[cam_idx][LAST][ppscount].w = pps[cam_idx][OLD][ppscount].w;
                    pps[cam_idx][LAST][ppscount].h = pps[cam_idx][OLD][ppscount].h;
                    pps[cam_idx][LAST][ppscount].has = pps[cam_idx][OLD][ppscount].has;
                    pps[cam_idx][LAST][ppscount].rate = pps[cam_idx][OLD][ppscount].rate;
                    pps[cam_idx][LAST][0].count = pps[cam_idx][OLD][0].count;
                    ppscount++;
                    hasperson = true;
                    if (cam_idx == 0)
                        positive = 1;

                }

                if (!strcmp(names[class], "head" ) ) {
                    heads[cam_idx][head_idx].x = left;
                    heads[cam_idx][head_idx].y = right;
                    heads[cam_idx][head_idx].x0 = (right - left) / 2 + left;
                    heads[cam_idx][head_idx].y0 = (bot - top) / 2 + top;
                    heads[cam_idx][head_idx].w = right - left;
                    heads[cam_idx][head_idx].h = bot - top;
                    hands[cam_idx][0].count++;
                    for (int n=0; n < last_heads[cam_idx][0].count; n++) {
                        distance = sqrt(pow(heads[cam_idx][head_idx].x0 - last_heads[cam_idx][n].x0, 2) + pow(heads[cam_idx][head_idx].y0 - last_heads[cam_idx][n].y0, 2));
                        if (distance < MIN_STATIC_DISTANCE) {
                            //printf("head distance < MIN STATIC DISTANCE: distance: %f \n", distance);
                        } else {
                            human_exist = true;
                        }
                    }
                    last_heads[cam_idx][head_idx].x = heads[cam_idx][head_idx].x;
                    last_heads[cam_idx][head_idx].y = heads[cam_idx][head_idx].y;
                    last_heads[cam_idx][head_idx].x0 = heads[cam_idx][head_idx].x0;
                    last_heads[cam_idx][head_idx].y0 = heads[cam_idx][head_idx].y0;
                    last_heads[cam_idx][head_idx].w = heads[cam_idx][head_idx].w;
                    last_heads[cam_idx][head_idx].h = heads[cam_idx][head_idx].h;
                    last_heads[cam_idx][0].count = heads[cam_idx][0].count;
                    head_idx++;

                    drawCvText(showImg, "head", left, top - 4);
                    cvRectangle(showImg, cvPoint(left,top), cvPoint(right, bot), CV_RGB( 0, 255, 0),2,1,0);
                    if (top < 4)
                        top = 4;
		    hashead = true;
                    if (cam_idx == 0)
                        positive = 1;
		}

                if (!strcmp(names[class], "person" ) ) {
                    drawCvText(showImg, "person", left, top - 4);
                    cvRectangle(showImg, cvPoint(left,top), cvPoint(right, bot), CV_RGB( 0, 0, 255),2,1,0);
                    if (top < 4)
                        top = 4;
                }
#endif
            } else {
                printf("Error:  detection error\n");
            }

        }
    }

    EnQueue(&human_exist_cache, (int)human_exist);
    if (human_exist_cache.size >= CACHE_SIZE) {
        int c = 0;
        queueNode *cur = human_exist_cache.head->next;
        while(cur != human_exist_cache.tail){
            if (cur->data){
                c += 1;
            }
            cur = cur->next;
        }
        if (c * 2 > CACHE_SIZE){
            if (!human_exist_stat.human_exist){
                printf("stat -> human exist\n");
                filter_detect_to_event(human_exist_name);
                human_exist_stat.human_exist = true;
            }
            human_exist_stat.human_gone_begin = 0;
        } else {
            if (human_exist_stat.human_exist) {
                if (!human_exist_stat.human_gone_begin) {
                    human_exist_stat.human_gone_begin = (int)time(NULL);
                } else {
                    int now = (int)time(NULL);
                    int delta = now - human_exist_stat.human_gone_begin;
                    if (delta > HUMAN_GONE_THRESHOLD) {
                        printf("stat -> human gone\n");
                        filter_detect_to_event(human_gone_name);
                        human_exist_stat.human_exist = false;
                    }
                }
            }
        }
        DeQueue(&human_exist_cache);
    }
#ifdef AUTO_FRAMERATE
    char str[50];
    if (!hasperson) {
        if (mdata->info[cam_idx].count > 0) {
            mdata->info[cam_idx].count = 0;
            //mdata->info[cam_idx].count--;
        } else if (mdata->info[cam_idx].count < -999999){
            mdata->info[cam_idx].count = -1;
        } else
            mdata->info[cam_idx].count--;

        if ((mdata->info[cam_idx].count < -6) && (mdata->info[cam_idx].fps >= 3))
            mdata->info[cam_idx].fps = 2;

        pflags[cam_idx]++;
        if ((pflags[cam_idx] >= P_THRESHOLD)) {

            hflags[cam_idx] = 0;
        }
    } else {
        if (mdata->info[cam_idx].count < 0) {
            mdata->info[cam_idx].count = 0;
        } else if (mdata->info[cam_idx].count > 999999){
            mdata->info[cam_idx].count = 1;
        } else {
            mdata->info[cam_idx].count++;
        }

        if ((mdata->info[cam_idx].count >= 3))
            mdata->info[cam_idx].fps = 30;

        hflags[cam_idx]++;
        if ((hflags[cam_idx] >= H_THRESHOLD) && (pflags[cam_idx] > 0)) {
            pflags[cam_idx] = 0;
        }
    }

#endif

#ifdef LOCAL_FOLLOW
    char str[50];
    if (!hasperson) {
        if (mdata->info[cam_idx].count > 0) {
            mdata->info[cam_idx].count = 0;
            //mdata->info[cam_idx].count--;
        } else if (mdata->info[cam_idx].count < -999999){
            mdata->info[cam_idx].count = -1;
        } else
            mdata->info[cam_idx].count--;

        if ((mdata->info[cam_idx].count < -6) && (mdata->info[cam_idx].fps >= 3))
            mdata->info[cam_idx].fps = 2;

        pflags[cam_idx]++;
        //printf("cam: move %d no person %d times\n", cam_idx, pflags[cam_idx]);
        if (pflags[0] >= 3 && pflags[1] >= 3 && pflags[2] >= 3 && pflags[3] >= 3) {
            printf("cam: =========================  move  stop  %d no person %d times\n", cam_idx, pflags[cam_idx]);
            if (!mdata->remote_motor)
                send_control_command(connection, "2motw0");
            mstatus = 0;
            spos[0] = 0;
        }
        if ((pflags[cam_idx] >= P_THRESHOLD)) {

            hflags[cam_idx] = 0;
        }

    } else {

        if (mdata->info[cam_idx].count < 0) {
            mdata->info[cam_idx].count = 0;
        } else if (mdata->info[cam_idx].count > 999999){
            mdata->info[cam_idx].count = 1;
        } else {
            mdata->info[cam_idx].count++;
        }

        if ((mdata->info[cam_idx].count >= 3))
            mdata->info[cam_idx].fps = 30;

        hflags[cam_idx]++;
        if ((hflags[cam_idx] >= H_THRESHOLD) && (pflags[cam_idx] > 0)) {
            pflags[cam_idx] = 0;
            //printf("cam:  -------------- move %d has person %d times\n", cam_idx, hflags[cam_idx]);
        }

        if (hflags[0] >= H_THRESHOLD || hflags[1] >= H_THRESHOLD || hflags[2] >= H_THRESHOLD || hflags[3] >= H_THRESHOLD) {

            if (cam_idx == 0 || cam_idx == 1 || cam_idx == 2 || cam_idx == 3) {
                int pos0 = 0;
                int pos1 = 0;
                int pos2 = 0;
                int pos3 = 0;
                int rec[4] = {0, 0, 0, 0};
                int pos = 0;
                int tcount = 0;

                if (pps[0][OLD][0].has) {
                    tcount += pps[0][OLD][0].count;
                }
                if (pps[1][OLD][0].has) {
                    tcount += pps[1][OLD][0].count;
                }

                for (int i = 0; i < 4; i++) {
                    if (pps[i][OLD][0].has) {
                        int idx = 0;
                        int max = 0;
                        for (int j = 0; j < pps[i][OLD][0].count; j++) {
			//if (pps[i][OLD][j].rate > 40)
                            if (pps[i][OLD][j].w > max) {
                                max = pps[i][OLD][j].w;
                                idx = j;
                            }
                        }
                        rec[i] = pps[i][OLD][idx].x0;
                        printf("move: cam_idx: %d x: %d, x0: %d, w: %d\n", cam_idx, pps[i][OLD][idx].x, pps[i][OLD][idx].x0, pps[i][OLD][idx].w);
                    }
                }

                rec[0] = rec[0];
		if (rec[2] > 0)
                	rec[2] = 416 - rec[2];

		if (rec[1] > 0)
                	rec[1] = 416 - rec[1];
                rec[3] = rec[3];

                pos = (rec[0] - rec[2]) + (rec[1] - rec[3]);
		pos = pos / 2;
                printf("move:cur pos: %d, rec < %d, %d, %d, %d >, ta: %d, d0: %d, d1: %d\n", pos , rec[0], rec[1],  rec[2],  rec[3], ta, mdata->d0, mdata->d1);
                if (pos > ta) {
                    // backward, i2c0, led side
			if (pos > 300)
                    send_control_command(connection, "2motw2100");
			else if (pos > 150)
                    send_control_command(connection, "2motw295");
			else if (pos > 60)
                    send_control_command(connection, "2motw290");
                    mstatus = 1;
                    printf("left backward move <<<<<<<<<<<<<<<<<<<< mstatus: %d\n", mstatus);
                }

                if (pos < -ta) {
			if (-pos > 300)
                    send_control_command(connection, "2motw1100");
			else if (-pos > 150)
                    send_control_command(connection, "2motw195");
			else if (-pos > 60)
                    send_control_command(connection, "2motw190");
                    mstatus = 2;
                    printf("right forward  move >>>>>>>>>>>>>>>>>>  mstatus: %d\n", mstatus);
                }

                if (pos <= ta && pos >= -ta) {
                    send_control_command(connection, "2motw0");
                    printf("get there no need move =====================================\n");
                    // no need to move
                }
            }
        }// 4 times has person
    }

#endif // LOCAL_FOLLOW

#ifdef LOCAL_GESTURE

    for (int i = 0; i < L_H; i++)
        for (int j = 0; j < L_W; j++)
        {
            lines[cam_idx][i][j].fresh = 0;;
            lines[cam_idx][i][j].used = 0;;
        }

    //gesture rec
    if (hands[cam_idx][0].count > 0)
    {
        for (int i = 0; i < hands[cam_idx][0].count; i++)
        {

            int ix = hands[cam_idx][i].x0 / L_STEP;
            int iy = hands[cam_idx][i].y0 / L_STEP;
            int ox = hands[cam_idx][i].x0 % L_STEP;
            int oy = hands[cam_idx][i].y0 % L_STEP;


            if (lines[cam_idx][iy][ix].dup == -1)
                lines[cam_idx][iy][ix].dup = 1;
            else
                lines[cam_idx][iy][ix].dup++;

            lines[cam_idx][iy][ix].fresh = 1;;
        }

        for (int i = 0; i < L_H; i++)
        {
            for (int j = 0; j < L_W; j++)
            {
                if (lines[cam_idx][i][j].fresh == 0)
                {
                    if (lines[cam_idx][i][j].dup <= 5)
                        lines[cam_idx][i][j].dup = -1;

                    //if (lines[cam_idx][i][j].start == 0)
                        //lines[cam_idx][i][j].status = -1;
                } else {
                    if (lines[cam_idx][i][j].dup >= 5 && lines[cam_idx][i][j].status == -1)
                    {
                        for (int k = j + 1; k < L_W; k++)
                            if (lines[cam_idx][i][k].status == 3) {
                                lines[cam_idx][i][k].status = -1;
                                lines[cam_idx][i][k].dup = -1;
                                lines[cam_idx][i][j].dup = -1;
                                printf("gesture: 5 time same pos : %d,%d , stop status: %d\n", lines[cam_idx][i][j].x, lines[cam_idx][i][j].y, lines[cam_idx][i][j].status );
                                send_control_command(connection, "2led+0");
                            }
                        for (int k = j - 1; k > 0; k--)
                            if (lines[cam_idx][i][k].status == 1) {
                                printf("gesture: 5 time same pos : %d,%d , stop status: %d\n", lines[cam_idx][i][j].x, lines[cam_idx][i][j].y, lines[cam_idx][i][j].status );
                                lines[cam_idx][i][k].status = -1;
                                lines[cam_idx][i][k].dup = -1;
                                lines[cam_idx][i][j].dup = -1;
                                send_control_command(connection, "2led+0");
                            }
                        for (int k = i + 1; k < L_H; k++)
                            if (lines[cam_idx][k][j].status == 4) {
                                printf("gesture: 5 time same pos in the up : %d,%d , stop status: %d\n", lines[cam_idx][i][j].x, lines[cam_idx][i][j].y, lines[cam_idx][i][j].status );
                                lines[cam_idx][k][j].status = -1;
                                lines[cam_idx][k][j].dup = -1;
                                lines[cam_idx][i][j].dup = -1;
                                send_control_command(connection, "2led+0");
                            }
                        for (int k = i - 1; k > 0; k--)
                            if (lines[cam_idx][k][j].status == 2) {
                                printf("gesture: 5 time same pos in the down : %d,%d , stop status: %d\n", lines[cam_idx][i][j].x, lines[cam_idx][i][j].y, lines[cam_idx][i][j].status );
                                lines[cam_idx][k][j].status = -1;
                                lines[cam_idx][k][j].dup = -1;
                                lines[cam_idx][i][j].dup = -1;
                                send_control_command(connection, "2led+0");
                            }


                        if (lines[cam_idx][i][j].dup != -1) {
                            lines[cam_idx][i][j].status = 0;
                            printf("gesture: 5 time same pos : %d,%d , start status: %d\n", lines[cam_idx][i][j].x, lines[cam_idx][i][j].y, lines[cam_idx][i][j].status );
                            //lines[cam_idx][i][j].start = 1;
                        }
                    }
                }
            }
        }

        for (int i = 0; i < L_H; i++)
        {
            for (int j = 0; j < L_W; j++)
            {
                if ((lines[cam_idx][i][j].fresh == 1) && (lines[cam_idx][i][k].used == 0)) {

                    int dir = 0;
                   for (int k = j+1; k < L_W; k++) {
                        if ((lines[cam_idx][i][k].status == 0)) {
                            printf("gesture: begin left\n");
                            //lines[cam_idx][i][j].status = 3;
                            lines[cam_idx][i][k].status = 3;
                            lines[cam_idx][i][j].used = 1;
                            cvLine(showImg, cvPoint(lines[cam_idx][i][j].x * L_STEP, lines[cam_idx][i][j].y * L_STEP), cvPoint(lines[cam_idx][i][k].x * L_STEP, lines[cam_idx][i][k].y * L_STEP), CV_RGB(0,255,0),2, 1, 0 );
                            dir++;
                     //           send_control_command(connection, "2led+0");
                        } else if (lines[cam_idx][i][k].status == 3) {
                            printf("gesture: keep left\n");
                    //            send_control_command(connection, "2led+0");
                        }
                    }

                    for (int k = j - 1; k > 0; k--) {
                        if ((lines[cam_idx][i][k].status == 0)) {
                            printf("gesture: begin right\n");
                            //lines[cam_idx][i][j].status = 1;
                            lines[cam_idx][i][k].status = 1;
                            lines[cam_idx][i][j].used = 1;
                            cvLine(showImg, cvPoint(lines[cam_idx][i][j].x * L_STEP, lines[cam_idx][i][j].y * L_STEP), cvPoint(lines[cam_idx][i][k].x * L_STEP , lines[cam_idx][i][k].y * L_STEP), CV_RGB(0,255, 0),2, 1, 0 );
                            dir++;
                      //          send_control_command(connection, "2led+0");
                        } else if (lines[cam_idx][i][k].status == 1)
                            printf("gesture: keep right\n");
                        //        send_control_command(connection, "2led+0");
                    }

                    if (hasperson && hashead) {
                        for (int k = i+1; k < L_H; k++)
                            if ((lines[cam_idx][k][j].status == 0)) {
                                printf("gesture: begin up\n");
                                //lines[cam_idx][i][j].status = 4;
                                lines[cam_idx][k][j].status = 4;
                                lines[cam_idx][i][j].used = 1;
                                cvLine(showImg, cvPoint(lines[cam_idx][i][j].x * L_STEP, lines[cam_idx][i][j].y * L_STEP), cvPoint(lines[cam_idx][k][j].x * L_STEP , lines[cam_idx][k][j].y * L_STEP), CV_RGB(0,255,255),2, 1, 0 );
                                send_control_command(connection, "2led+20");
                            } else if (lines[cam_idx][k][j].status == 4) {
                                printf("gesture: keep up, distance: %d\n", k - i);
                                //send_control_command(connection, "2led+20");
                            }


                        for (int k = i - 1; k > 0; k--) {
                            if ((lines[cam_idx][k][j].status == 0)) {
                                printf("gesture: begin down\n");
                                //lines[cam_idx][i][j].status = 2;
                                lines[cam_idx][k][j].status = 2;
                                lines[cam_idx][i][j].used = 1;
                                cvLine(showImg, cvPoint(lines[cam_idx][i][j].x * L_STEP, lines[cam_idx][i][j].y * L_STEP), cvPoint(lines[cam_idx][k][j].x * L_STEP, lines[cam_idx][k][j].y * L_STEP), CV_RGB(255,255,0),2, 1, 0 );
                                send_control_command(connection, "2led-20");
                            } else if (lines[cam_idx][k][j].status == 2) {
                                printf("gesture: keep down, distance: %d\n", i - k);
                                //send_control_command(connection, "2led-20");
                            }
                        }
                    }
                }
            }
        }
    }

#endif // LOCAL_GESTURE
    return 0;

}

int get_control_msg(pmnet mdata)
{

    int try = 10;

#if 1
    memset(mdata->msg, '0', 10);
    while (try-- > 0) {
        dbus_connection_read_write (connection,0);
        message = dbus_connection_pop_message (connection);
        if(message == NULL){
            usleep(1000);
            continue;
        } else {
            if (dbus_message_is_signal(message, "com.pi.cameraSensorInterface", "change_model")) {
                DBusError error;
                char *s;

                dbus_error_init (&error);
                if (dbus_message_get_args(message, &error, DBUS_TYPE_STRING, &s, DBUS_TYPE_INVALID)) {

                    printf("client received singal: %s\n", s);
                    memcpy(mdata->msg, s, 10);
                    dbus_message_unref(message);
                    return 0;
                } else {
                    printf("client received, but error getting message: %s\n", error.message);
                    dbus_error_free (&error);
                    dbus_message_unref(message);
                    continue;
                }
            } else {
                dbus_message_unref(message);
                continue;
            }

        }
    }
#endif
    return -1;
}

int send_with_face_msg(pmnet mdata, char *msg)
{

    sprintf(temp, "%s|%s", temp, msg);

    if (temp[0] != '\0') {
        printf("%s\n", temp);
        send_object_messages(connection, temp);
    }

    return 0;
}


int send_without_face_msg(pmnet mdata)
{

    if (temp[0] != '\0') {
        printf("%s\n", temp);
        send_object_messages(connection, temp);
    }

    return 0;
}

int append_with_face_msg(pmnet mdata, char *msg)
{

    if ((strlen(temp) + 50) <= BUFMAX)
        sprintf(temp, "%s|%s", temp, msg);
    else
        printf("Warning too many objects, can not append face\n");

    return 0;
}

int send_dbus_msg(char *transfer, pmnet mdata)
{

    if (temp[0] != '\0') {
        //printf("%s\n", temp);
        memset(transfer, '\0', sizeof(transfer));
        transfer = strcpy(transfer, temp);
        send_object_messages(connection, temp);
        //printf("%s\n", transfer);
    }

    return 0;
}

int filter_detect_to_event(char *event_name)
{
    printf("%s\n", event_name);
    emit_event_signal(connection, event_name);

    return 0;
}

int object_detect_run(pmnet mdata,  float *buf)
{
    struct timeval consume_time;
    struct timeval start;
    struct timeval end;
    double over_time;
    gettimeofday(&start, NULL);

    layer l = net.layers[net.n-1];
    float *prediction =  network_predict_gpu(net, buf);
    gettimeofday(&end, NULL);
    timeval_subtract(&consume_time, &start, &end);
    over_time = (consume_time.tv_usec + consume_time.tv_sec * 1000000);
    log_debug("%s network_predict_gpu cost time: %f us, cam_idx: %d\n", __func__, over_time, mdata->cam_idx);

    l.output = prediction;

    get_region_boxes(l, net.w, net.h, net.w, net.h, mdata->thresh, mdata->probs, mdata->boxes, 0, 0, 0, mdata->hier_thresh,1, mdata->thresh_hand);

    //printf("%s , w:%d, h%d, net: %p, classes: %d\n", __func__, mdata->net->w, mdata->net->h, mdata->net, mdata->classes);
    do_nms(mdata->boxes, mdata->probs, l.w * l.h * l.n, l.classes, mdata->nms);
    //memset(l.output, 0, l.outputs*sizeof(float));

    return 0;
}


static void *timer_thread(void *arg)
{
    while (!mdata->exit) {
        if (mdata->remote_motor) {
            if (mdata->t_motor == 0)
                mdata->remote_motor = false;

            mdata->t_motor--;
        }

        if (mdata->remote_led) {
            if (mdata->t_led == 0)
                mdata->remote_led = false;

            mdata->t_led--;
        }

        sleep(1);
    }
}


char * get_absolute_path(char * related_path){
        char * full_path;
        char current_absolute_path[MAX_SIZE];
        //获取当前程序绝对路径
        int cnt = readlink("/proc/self/exe", current_absolute_path, MAX_SIZE);
        if (cnt < 0 || cnt >= MAX_SIZE)
        {
                printf("***Error***\n");
                exit(-1);
        }
        //获取当前目录绝对路径，即去掉程序名
        int i;
        for (i = cnt; i >=0; --i)
        {
                if (current_absolute_path[i] == '/')
                {
                        current_absolute_path[i+1] = '\0';
                        break;
                }
        }
        strcat(current_absolute_path, related_path);
        full_path = strdup(current_absolute_path);
        return full_path;

}

pmnet object_detect_init(int cam_idx)
{

    //char *cfgfile = "/home/ubuntu/tiny-yolo-three.cfg";
    //char *weightfile = "/home/ubuntu/tiny-yolo-three_200000.weights";
    char *cfgfile = get_absolute_path("../../net/tiny-yolo-three.cfg");
    char *weightfile = get_absolute_path("../../net/tiny-yolo-three_40000.weights");
    float thresh_hand = 0.25;
    float hier_thresh = 0.5;
    float thresh = 0.24;
    float nms = 0.4;
    int ret;

    int classes = 3;

    fp = fopen("/tmp/robot.log", "w+");
    log_set_fp(fp);
    log_set_quiet(1);
    temp = calloc(BUFMAX, 1);
    buf_count = BUFMAX;
    mdata = (pmnet)calloc(sizeof(mnet), 1);

    if (init_session_bus(&connection, "com.pi.cameraSensorConnect")){
        printf("bus connection fail\n");
        return NULL;
    }


    dbus_bus_add_match(connection,"type='signal',interface='com.pi.cameraSensorInterface'", NULL);

    mdata->remote_motor = false;
    mdata->remote_led = false;

    pthread_mutex_init(&mdata->range_lock, NULL);
    pthread_cond_init(&mdata->range_cond, NULL);
    mdata->exit = false;


    ret = pthread_create(&mdata->timer_handle,NULL,(void *) timer_thread, mdata);
    if(ret) {
        printf ("Create pthread error!\n");
        return NULL;
    }


    spos[0] = 0;
    spos[1] = 0;

    net = parse_network_cfg(cfgfile);
    mdata->net = &net;
    if(weightfile){
        load_weights(&net, weightfile);
    }
    set_batch_network(&net, 1);
    mdata->w = net.w;
    mdata->h = net.h;

    srand(2222222);

    layer l = net.layers[net.n-1];
    mdata->l = &l;

    mdata->info[0].fps = 30;
    mdata->info[1].fps = 30;
    mdata->info[2].fps = 30;
    mdata->info[3].fps = 30;

    for (int i = 0; i < 4; i++) {
        mdata->info[i].cam_idx = i;
        //mdata->info[i].fps = 30;
        mdata->info[i].autoc = true;
        mdata->info[i].count = 0;
    }

    mdata->num = mdata->l->w * mdata->l->h * mdata->l->n;

    //printf("%s, %d, l.w: %d, l.h: %d, l.n: %d, num: %d\n", __func__, __LINE__, mdata->l->w, mdata->l->h, mdata->l->n, mdata->num);
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

    init_lines();

    init_stat();
    InitQueue(&human_exist_cache);
    return mdata;
}

void object_detect_deinit(pmnet mdata)
{
    printf("%s =================\n", __func__);
    mdata->exit = true;
    sleep(1);

    fclose(fp);
    free(mdata);
}
#ifdef __cplusplus
};
#endif

