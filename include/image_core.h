#ifndef IMAGE_CORE_H
#define IMAGE_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool InitCUDA(void);
void cudayuv2rgb(float *dev_a, const char *dev_b, int w, int h);
void cuda_resize(float *src, float *dst, int src_w, int src_h, int dst_w, int dst_h);
void cudaShowCon(char *dst, const float *src, int w, int h);

void cudabgrtorgb(float *dst, const float *src, int w, int h);
void cudaswap(float *dst, int w, int h);

void cuda_splice(char *src, int w, int h);
void cuda_splice_four(char *src0, char *src1, char *src2, char *src3, char *dst, int w, int h);

#ifdef __cplusplus
};
#endif

#endif
