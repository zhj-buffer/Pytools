#include "network.h"
#include "region_layer.h"
//#include "detection_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "pipe_service.h"

#ifdef OPENCV
#include "opencv2/highgui/highgui_c.h"
#endif

void convert_detections(float *predictions, int classes, int num, int square, int side, int w, int h, float thresh, float **probs, box *boxes, int only_objectness)
{
    int i,j,n;
    //int per_cell = 5*num+classes;
    for (i = 0; i < side*side; ++i){
        int row = i / side;
        int col = i % side;
        for(n = 0; n < num; ++n){
            int index = i*num + n;
            int p_index = side*side*classes + i*num + n;
            float scale = predictions[p_index];
            int box_index = side*side*(classes + num) + (i*num + n)*4;
            boxes[index].x = (predictions[box_index + 0] + col) / side * w;
            boxes[index].y = (predictions[box_index + 1] + row) / side * h;
            boxes[index].w = pow(predictions[box_index + 2], (square?2:1)) * w;
            boxes[index].h = pow(predictions[box_index + 3], (square?2:1)) * h;
            for(j = 0; j < classes; ++j){
                int class_index = i*classes;
                float prob = scale*predictions[class_index+j];
                probs[index][j] = (prob > thresh) ? prob : 0;
            }
            if(only_objectness){
                probs[index][0] = scale;
            }
        }
    }
}

int eye(int argc, char **argv)
{

    float thresh = find_float_arg(argc, argv, "-thresh", .24);
    float hier_thresh = find_float_arg(argc, argv, "-hier", .5);
    int debug = find_int_arg(argc, argv, "-debug", 0);


    char *cfgfile = argv[1];
    char *weightfile = (argc > 2) ? argv[2] : 0;

    network net = parse_network_cfg(cfgfile);
    if(weightfile){
        load_weights(&net, weightfile);
    }
    layer l = net.layers[net.n-1];
    set_batch_network(&net, 1);
    srand(2222222);
    clock_t time;
    char buff[256];
    char *input = buff;
    int i,j;
    float nms=.4;
    box *boxes = calloc(l.w*l.h*l.n, sizeof(box));
    float **probs = calloc(l.w*l.h*l.n, sizeof(float *));
    for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = calloc(l.classes, sizeof(float *));

    // Size of image from camera
    int w = 640;
    int h = 480;
    int c = 3;

    /////////////////////////////////////////////////
    // Wait to get image from pipe
    /////////////////////////////////////////////////

    CPipe *pipe = NULL;

    // Create pipe
	pipe = create_pipe();
	if(pipe == NULL) {
		printf("create and init CPipe fail\n");
		return 0;
	}

    // Wait to receive from the python process
    char sendbuff[1024];
    char output[30];
    char recvbuff[MAXSIZE];
    fprintf(stderr, "Ready ...\n");
	while(1) {
	    if(debug)fprintf(stderr, "Waiting for the image from the pipe ...\n");
		memset(recvbuff, 0, sizeof(recvbuff));
		if(read(pipe->pipe_read, recvbuff, sizeof(recvbuff)) < 0) {
			printf("read from pipe error\n");
		}

        // Convert from byte array into image

        if(debug)fprintf(stderr, "Converting the received byte array into image ...\n");
        int tmp_i, tmp_j, tmp_k;
        image im = make_image(w, h, c);
        for(tmp_k = 0; tmp_k < c; ++tmp_k){
            for(tmp_j = 0; tmp_j < h; ++tmp_j){
                for(tmp_i = 0; tmp_i < w; ++tmp_i){
                    // int dst_index = i + w*j + w*h*k;
                    // int src_index = k + c*i + c*w*j;
                    int dst_index = tmp_i + w*tmp_j + w*h*tmp_k;
                    int src_index = tmp_k + c*tmp_i + c*w*tmp_j;
                    im.data[dst_index] = (float)recvbuff[src_index]/255.;
                }
            }
        }
        rgbgr_image(im);

        // Resize image
        image sized = resize_image(im, net.w, net.h);

        // Pointer to the image
        float *X = sized.data;

        // Make predictions
        if(debug)fprintf(stderr, "Predicting ...\n");
        network_predict(net, X);
        if(debug)fprintf(stderr, "Detected %d objects\n", l.w*l.h*l.n);
        // Get the list of outputs from predictions
        if(debug)fprintf(stderr, "Converting predictions ...\n");
        //convert_detections(predictions, l.classes, l.n, l.sqrt, l.side, 1, 1, thresh, probs, boxes, 0);
        get_region_boxes(l, 1, 1, thresh, probs, boxes, 0, 0, hier_thresh, 0.1);
        if(debug)fprintf(stderr, "Preparing buffer for sending outputs in byte array ...\n");
        memset(sendbuff, 0, sizeof(sendbuff));
        memset(output, 0, sizeof(output));

        // Post processing
        if(debug)fprintf(stderr, "Post processing ...\n");
        if (nms) do_nms_sort(boxes, probs, l.w*l.h*l.n, l.classes, nms);
       
        int not_found = 1;
        int first = 1;
        int num = l.w*l.h*l.n;
        int classes = 20;
        if(debug)fprintf(stderr, "After NMS left %d objects\n", l.w*l.h*l.n);
        for(i = 0; i < num; ++i)
        {
            int class = max_index(probs[i], classes);
            float prob = probs[i][class];
            //fprintf(stderr, "%f,",prob);
            if(prob > thresh)
            {
                not_found = 0;

                box b = boxes[i];
                int left  = (b.x-b.w/2.)*im.w;
                int right = (b.x+b.w/2.)*im.w;
                int top   = (b.y-b.h/2.)*im.h;
                int bot   = (b.y+b.h/2.)*im.h;

                snprintf(output, sizeof(output), "%d,%f,%d,%d,%d,%d", class, prob, top, left, bot, right);

                if(first)
                {
                    strcpy(sendbuff, output);
                    first = 0;
                }
                else
                {
                    strcat(sendbuff, "|");
                    strcat(sendbuff, output);
                }
            }
        }

        if(not_found)
        {
            snprintf(sendbuff, sizeof(output), "|%d,%d,%d,%d,%d,%d", -1, 0, 0, 0, 0, 0);
        }

        if(debug)
	    fprintf(stderr, "Sending the output: %s\n\n", sendbuff);
	
	for(i=strlen(sendbuff); i<sizeof(sendbuff); i++) {
	    sendbuff[i] = '\t';
	}
        if(write(pipe->pipe_write, sendbuff, sizeof(sendbuff)) < 0)
        {
            printf("write to pipe error\n");
        }

        free_image(im);
        free_image(sized);
        //free(boxes);
        //free_ptrs((void **)probs, l.w*l.h*l.n);
	}

	destroy_pipe(pipe);

}
