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

static int pipe_init(const char *path, int mode) {
    int res, fd;

    if(path == NULL)
    	return -1;

    // Check pipe exist
    if(access(path, F_OK) == 0)
    	unlink(path);

    // Create pipe
    if( (res = mkfifo(path, O_CREAT|O_EXCL|0755)) < 0)
       return -2;
    
    // Open pipe
    fd = open(path, mode);
    if(fd < 0){
        unlink(path);
        return -3;
    }

    return fd;
}

int CPipe_init(CPipe *pipe) {
	int fd;

	if(pipe == NULL)
		return -1;

	fd = pipe_init(WRITE_FIFO_PATH, O_RDWR);
	if(fd < 0) {
		printf("create write pipe failed, errcode=%d\n", fd);
		return -2;
	}
	pipe->pipe_write = fd;

	fd = pipe_init(READ_FIFO_PATH, O_RDWR);
	if(fd < 0) {
		printf("create read pipe failed, errcode=%d\n", fd);
		return -3;
	}
	pipe->pipe_read = fd;
	return 0;
}

CPipe * create_pipe() {
	CPipe *pipe = (CPipe *)malloc(sizeof(CPipe));
	if(pipe != NULL) {
		memset(pipe, 0, sizeof(CPipe));
		if(CPipe_init(pipe) < 0) {
			free(pipe);
			pipe = NULL;
		}
	}
	return pipe;
}

void CPipe_cleanup(CPipe *pipe) {
	if(pipe != NULL) {
		if(pipe->pipe_write > 0) {
			close(pipe->pipe_write);
			unlink(WRITE_FIFO_PATH);
		}
		if(pipe->pipe_read > 0) {
			close(pipe->pipe_read);
			unlink(READ_FIFO_PATH);
		}
	}
}

void destroy_pipe(CPipe *pipe) {
	if(pipe != NULL) {
		CPipe_cleanup(pipe);
		free(pipe);
	}
}
