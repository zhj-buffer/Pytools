#include "cuda_runtime.h"           //CUDA运行时API  
// CUDA runtime
#include <cuda.h>
#include <cuda_runtime.h>

#include "device_launch_parameters.h"     
#include <malloc.h>
#include <stdio.h> 
#include <stdlib.h>
#include <sys/time.h>

#define CAM_W 1920
#define CAM_H 1080
#define BLOCK_NUM  64
#define THREAD_NUM 512

#if 1
static struct timeval tv0;
static struct timeval tv1;
static struct timezone tz;
#endif
__global__ void cudaResizeLinear(float *src, float *dst, int w0, int h0, int w1, int h1)
{
	const int tid = threadIdx.x;
	const int bid = blockIdx.x;
	int i;

	int y1,y2, x1,x2,  x, y;
	float fx1, fx2, fy1, fy2;

	for (i = bid * THREAD_NUM + tid; i < w1 * h1; i += BLOCK_NUM * THREAD_NUM)
	{
		x = (i) % w1;
		y = (i) / w1;

		x1 = (int)(x* ((float)w0 / (float)w1));
		x2 = (int)(x* ((float)w0 / (float)w1)) + 1;
		y1 = (int)(y* ((float)h0 / (float)h1));
		y2 = (int)(y* ((float)h0 / (float)h1)) + 1;

		fx1 = (((float)x* (((float)w0) / (float)w1))) - (int)(x * (((float)w0) / (float)w1));
		fx2 = 1.0f - fx1;
		fy1 = (((float)y* (((float)h0) / (float)h1))) - (int)(y * (((float)h0) / (float)h1));
		fy2 = 1.0f - fy1;

		float s1 = fx1*fy1;
		float s2 = fx2*fy1;
		float s3 = fx2*fy2;
		float s4 = fx1*fy2;

		dst[i * 3 + 0] = (src[y1 * w0 * 3 + x1 * 3 + 0]) * s3 + (src[y1 * w0 *3 + x2*3 + 0]) * s4 + (src[y2 * w0*3 + x1*3 + 0]) * s2 + (src[y2 * w0 *3 + x2 *3 + 0]) * s1;
		dst[i * 3 + 1] = (src[y1 * w0 * 3 + x1 * 3 + 1]) * s3 + (src[y1 * w0 *3 + x2*3 + 1]) * s4 + (src[y2 * w0*3 + x1*3 + 1]) * s2 + (src[y2 * w0 *3 + x2 *3 + 1]) * s1;
		dst[i * 3 + 2] = (src[y1 * w0 * 3 + x1 * 3 + 2]) * s3 + (src[y1 * w0 *3 + x2*3 + 2]) * s4 + (src[y2 * w0*3 + x1*3 + 2]) * s2 + (src[y2 * w0 *3 + x2 *3 + 2]) * s1;
	}
}

__global__ void addKernel(float *a,  const char *b, int w, int h)
{
    const int tid = threadIdx.x;
    const int bid = blockIdx.x;
    int i;

    for (i = bid * THREAD_NUM + tid; i < w * h / 2; i += BLOCK_NUM * THREAD_NUM) {

        a[i*6 + 0]=(10000*b[i*4 + 1]+14075*(b[i*4 + 2]-128))/10000;
        a[i*6 + 1]=(10000*b[i*4 + 1]-3455*( b[i*4 + 0]-128)-7169*(b[i*4 + 2]-128))/10000;
        a[i*6 + 2]=(10000*b[i*4 + 1]+17990*(b[i*4 + 0]-128))/10000;
        a[i*6 + 3]=(10000*b[i*4 + 3]+14075*(b[i*4 + 2]-128))/10000;
        a[i*6 + 4]=(10000*b[i*4 + 3]-3455*( b[i*4 + 0]-128)-7169*(b[i*4 + 2]-128))/10000;
        a[i*6 + 5]=(10000*b[i*4 + 3]+17990*(b[i*4 + 0]-128))/10000;

        if(a[i*6 + 0]>255) a[i*6 + 0]=255; if(a[i*6 + 0]<0) a[i*6 + 0]=0;
        if(a[i*6 + 1]>255) a[i*6 + 1]=255; if(a[i*6 + 1]<0) a[i*6 + 1]=0;
        if(a[i*6 + 2]>255) a[i*6 + 2]=255; if(a[i*6 + 2]<0) a[i*6 + 2]=0;
        if(a[i*6 + 3]>255) a[i*6 + 3]=255; if(a[i*6 + 3]<0) a[i*6 + 3]=0;
        if(a[i*6 + 4]>255) a[i*6 + 4]=255; if(a[i*6 + 4]<0) a[i*6 + 4]=0;
        if(a[i*6 + 5]>255) a[i*6 + 5]=255; if(a[i*6 + 5]<0) a[i*6 + 5]=0;

    }
}

__global__ void cudasplice(char *src0, char *src1, char *src2, char * src3, char *dst, int w, int h)
{
    const int tid = threadIdx.x;
    const int bid = blockIdx.x;
    int i, h1, w1;
    int size = w * h;
    int bw = 2 * w;

    for (i = bid * THREAD_NUM + tid; i < size * 4; i += BLOCK_NUM * THREAD_NUM) {
        w1 = i % bw;
        h1 = i / bw;
        if (((i % bw) < w ) && (i / bw < h)) {
            dst[i * 2 + 0] = src0[(h1 * w + w1) * 2 + 0];
            dst[i * 2 + 1] = src0[(h1 * w + w1) * 2 + 1];
        } else if ((i % bw >= w ) && (i / bw < h)) {
            dst[i * 2 + 0] = src1[(h1 * w + w1) * 2 + 0];
            dst[i * 2 + 1] = src1[(h1 * w + w1) * 2 + 1];
        } else if ((i % bw < w ) && (i / bw >= h)) {
            h1 = h1 - h;
            dst[i * 2 + 0] = src2[(h1 * w + w1) * 2 + 0];
            dst[i * 2 + 1] = src2[(h1 * w + w1) * 2 + 1];
        } else if ((i % bw >= w ) && (i / bw >= h)) {
            h1 = h1 - h;
            dst[i * 2 + 0] = src3[(h1 * w + w1) * 2 + 0];
            dst[i * 2 + 1] = src3[(h1 * w + w1) * 2 + 1];
        } else {
            printf(" Should not be here\n");
        }
    }
}
__global__ void cudaShowconvert(char *dst, const float *src, int w, int h)
{
    const int tid = threadIdx.x;
    const int bid = blockIdx.x;
    int i;

    for (i = bid * THREAD_NUM + tid; i < w * h; i += BLOCK_NUM * THREAD_NUM) {
        dst[i * 3 + 0] = src[i * 3 + 2];
        dst[i * 3 + 1] = src[i * 3 + 1];
        dst[i * 3 + 2] = src[i * 3 + 0];
    }
}

__global__ void cudabgr2rgb(float *dst, const float *src, int w, int h)
{
    const int tid = threadIdx.x;
    const int bid = blockIdx.x;
    int i, j, c;

	for (c = 0; c < 3; c++)
	for (j = 0; j < h; j++)
    for (i = bid * THREAD_NUM + tid; i < w; i += BLOCK_NUM * THREAD_NUM) {
        dst[i  + j *  w  + c * h * w] = src[i * 3 + 3 * w * j + c] / 255.;
    }
}


__global__ void cudaswapfloat(float *dst, int w, int h)
{
    const int tid = threadIdx.x;
    const int bid = blockIdx.x;
    int i;
	float swap;

    for (i = bid * THREAD_NUM + tid; i < w * h; i += BLOCK_NUM * THREAD_NUM) {
		swap = dst[i];
		dst[i] = dst[i + w*h*2];
		dst[i + w*h*2] = swap;
    }
}

extern "C"
bool InitCUDA(void)
{
    int count = 0;
    int i = 0;
    cudaGetDeviceCount(&count); //看看有多少个设备?
    if(count == 0)   //哈哈~~没有设备.
    {
        fprintf(stderr, "There is no device.\n");
        return false;
    }
    cudaDeviceProp prop;
    for(i = 0; i < count; i++)  //逐个列出设备属性:
    {
        if(cudaGetDeviceProperties(&prop, i) == cudaSuccess)
        {
            if(prop.major >= 1)
            {
                break;
            }
        }
    }
    if(i == count)
    {
        fprintf(stderr, "There is no device supporting CUDA.\n");
        return false;
    }
    cudaSetDevice(i);

    cudaDeviceProp sDevProp = prop;

    printf( "\n\nGPU Num: %d \n", i);
    printf( "Device name: %s\n", sDevProp.name );
    printf( "Device memory: %lu\n", sDevProp.totalGlobalMem );
    printf( "Memory per-block: %lu\n", sDevProp.sharedMemPerBlock );
    printf( "Register per-block: %u\n", sDevProp.regsPerBlock );
    printf( "Warp size: %u\n", sDevProp.warpSize );
    printf( "Memory pitch: %lu\n", sDevProp.memPitch );
    printf( "Constant Memory: %lu\n", sDevProp.totalConstMem );
    printf( "Max thread per-block: %u\n", sDevProp.maxThreadsPerBlock );
    printf( "Max thread dim: ( %d, %d, %d )\n", sDevProp.maxThreadsDim[0],
            sDevProp.maxThreadsDim[1], sDevProp.maxThreadsDim[2] );
    printf( "Max grid size: ( %d, %d, %d )\n", sDevProp.maxGridSize[0],  
            sDevProp.maxGridSize[1], sDevProp.maxGridSize[2] );
    printf( "Ver: %d.%d\n", sDevProp.major, sDevProp.minor );
    printf( "Clock: %d\n", sDevProp.clockRate );
    printf( "textureAlignment: %lu\n", sDevProp.textureAlignment );
    printf( "CUDART_VERSION: %d\n", CUDART_VERSION);

    if (!prop.canMapHostMemory)
    {
        printf("Device %d does not support mapping CPU host memory!\n", i);

    } else {
        printf("Device %d support mapping CPU host memory!\n", i);
    }

    cudaSetDeviceFlags(cudaDeviceMapHost);


    printf("\nCUDA initialized.\n\n");
    return true;
}

extern "C"
void cudayuv2rgb(float *dev_a, const char *dev_b, int w, int h)
{
    gettimeofday(&tv0, &tz);
	addKernel<<<BLOCK_NUM, THREAD_NUM>>>(dev_a, dev_b, w, h);
    gettimeofday(&tv1, &tz);
    //printf("\n kernel running Cost time :  %lu us\n", tv1.tv_usec - tv0.tv_usec);
}

extern "C"
void cuda_resize(float *src, float *dst, int src_w, int src_h, int dst_w, int dst_h)
{
    gettimeofday(&tv0, &tz);
	cudaResizeLinear<<<BLOCK_NUM, THREAD_NUM>>>(src, dst, src_w, src_h, dst_w, dst_h);
    gettimeofday(&tv1, &tz);
    //printf("\n kernel running Cost time :  %lu us\n", tv1.tv_usec - tv0.tv_usec);
}

extern "C"
void cudaShowCon(char *dst, const float *src, int w, int h)
{
    gettimeofday(&tv0, &tz);
    cudaShowconvert<<<BLOCK_NUM, THREAD_NUM>>>(dst, src, w, h);
    gettimeofday(&tv1, &tz);
//    printf("\n kernel show convert Cost time :  %lu us\n", tv1.tv_usec - tv0.tv_usec);
}
extern "C"
void cudabgrtorgb(float *dst, const float *src, int w, int h)
{
    gettimeofday(&tv0, &tz);
    cudabgr2rgb<<<BLOCK_NUM, THREAD_NUM>>>(dst, src, w, h);
    gettimeofday(&tv1, &tz);
//    printf("\n kernel show convert Cost time :  %lu us\n", tv1.tv_usec - tv0.tv_usec);
}
extern "C"
void cudaswap(float *dst, int w, int h)
{
    gettimeofday(&tv0, &tz);
    cudaswapfloat<<<BLOCK_NUM, THREAD_NUM>>>(dst,w, h);
    gettimeofday(&tv1, &tz);
//    printf("\n kernel show convert Cost time :  %lu us\n", tv1.tv_usec - tv0.tv_usec);
}

extern "C"
void cuda_splice(char *src, int w, int h)
{
    int offset = w * h * 2;
    gettimeofday(&tv0, &tz);
    cudasplice<<<BLOCK_NUM, THREAD_NUM>>>(src, src + offset * 1, src + offset * 2, src + offset * 3, src + offset * 4, w, h);
    gettimeofday(&tv1, &tz);
//    printf("\n kernel show convert Cost time :  %lu us\n", tv1.tv_usec - tv0.tv_usec);
}

extern "C"
void cuda_splice_four(char *src0, char *src1, char *src2, char *src3, char *dst, int w, int h)
{
    //int offset = w * h * 2;
    gettimeofday(&tv0, &tz);
    cudasplice<<<BLOCK_NUM, THREAD_NUM>>>(src0, src1, src2, src3, dst, w, h);
    gettimeofday(&tv1, &tz);
//    printf("\n kernel show convert Cost time :  %lu us\n", tv1.tv_usec - tv0.tv_usec);
}
