###############################################################################
#
# Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
###############################################################################

GPU=1
CUDNN=1
OPENCV=1
DEBUG=0

# Internal flags
NVCCFLAGS := --shared
CPPFLAGS += -fPIC
#CFLAGS+= -I ./../include
EXTRCFLAGS := -Wall -Wfatal-errors --shared -fPIC -Ofast

SRCS:= $(wildcard *.c)
SRCS+= $(wildcard *.cu)
OBJS:= $(SRCS:.c=.o)
OBJS+= $(SRCS:.cu=.o)

# Debug flags
ifneq ($(CUDA_DEBUG),)
NVCCFLAGS += -g -G
endif

ifeq ($(OPENCV), 1)
CFLAGS+= -DOPENCV \
		 `pkg-config --cflags opencv`
LDFLAGS+= `pkg-config --libs opencv`
endif

LDFLAGS+= `pkg-config --libs dbus-1 glib-2.0 dbus-glib-1`
CFLAGS+= `pkg-config --cflags dbus-1 glib-2.0 dbus-glib-1`
OBJ +=dbus.o

ifeq ($(CUDNN), 1)
CFLAGS+= -DCUDNN
LDFLAGS+= -lcudnn
endif

ifeq ($(GPU), 1)
CFLAGS+= -DGPU \
	 -I/usr/local/cuda/include
CFLAGS+= -DGPU
LDFLAGS+= -L /usr/local/cuda/lib64 -lcuda -lcudart -lcublas -lcurand -lrt -ldbus-1 -lglib-2.0
LDFLAGS+= -lstdc++
endif

ALL_CPPFLAGS :=
ALL_CPPFLAGS += $(NVCCFLAGS)
ALL_CPPFLAGS += $(addprefix -Xcompiler ,$(CPPFLAGS))
#ALL_CPPFLAGS += $(CFLAGS)


# Common includes and paths for CUDA
INCLUDES  := -I"./../include"

# CUDA code generation flags
GENCODE_SM53 := -gencode arch=compute_53,code=sm_53
GENCODE_SM62 := -gencode arch=compute_62,code=sm_62
GENCODE_FLAGS := $(GENCODE_SM53) $(GENCODE_SM62)

#OBJS += gemm.o utils.o cuda.o convolutional_layer.o list.o image.o activations.o im2col.o col2im.o blas.o crop_layer.o dropout_layer.o maxpool_layer.o softmax_layer.o data.o matrix.o network.oconnected_layer.o cost_layer.o parser.o option_list.o detection_layer.o captcha.o route_layer.o writing.o box.o nightmare.o normalization_layer.o avgpool_layer.o coco.o dice.o detector.o layer.ocompare.o classifier.o local_layer.o swag.o shortcut_layer.o activation_layer.o rnn_layer.o gru_layer.o rnn.o rnn_vid.o crnn_layer.o tag.o cifar.o go.o batchnorm_layer.o art.o region_layer.oreorg_layer.o super.o voxel.o tree.o pipe_service.o queue.o yolo.o eye_c.o base.o demo.o \
	convolutional_kernels.o activation_kernels.o im2col_kernels.o col2im_kernels.o blas_kernels.o crop_layer_kernels.o dropout_layer_kernels.o maxpool_layer_kernels.o network_kernels.oavgpool_layer_kernels.o image_core.o


# Target rules
all: $(OBJS)

%.o: %.c
	$(CC) $(INCLUDES) $(CFLAGS) $(EXTRCFLAGS) -o $@ -c $<

%.o: %.cu
	$(NVCC) -I/usr/local/cuda/include $(ALL_CPPFLAGS) $(GENCODE_FLAGS) -o $@ -c $<

clean:
	rm -rf *.o
