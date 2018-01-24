#ifndef _PIPE_SERVICE_H_HEADER
#define _PIPE_SERVICE_H_HEADER

#define WRITE_FIFO_PATH	"/tmp/result_fifo"
#define READ_FIFO_PATH	"/tmp/frame_fifo"
#define MAXSIZE 480*640*3

typedef struct _CPipe {
	int pipe_write;
	int pipe_read;
}CPipe;

extern CPipe * create_pipe();
extern void destroy_pipe(CPipe *pipe);

#endif
