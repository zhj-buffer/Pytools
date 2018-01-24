GPU=1
CUDNN=0
OPENCV=1
DBUS=1
DEBUG=0

#ARCH= --gpu-architecture=compute_32 --gpu-code=compute_32

GENCODE_SM53 := -gencode arch=compute_53,code=sm_53
GENCODE_SM62 := -gencode arch=compute_61,code=sm_61
GENCODE_FLAGS := $(GENCODE_SM53)


VPATH=src:include
OBJDIR=obj/
#INCLUDES= -I./include

CC=gcc
NVCC=nvcc
OPTS=-Ofast
LDFLAGS= -lm -pthread
COMMON=
CFLAGS=-Wall -Wfatal-errors

#CFLAGS+= -I./include

ifeq ($(DEBUG), 1)
OPTS=-O0 -g
endif

CFLAGS+=$(OPTS)

ifeq ($(OPENCV), 1)
COMMON+= -DOPENCV
CFLAGS+= -DOPENCV
LDFLAGS+= `pkg-config --libs opencv`
COMMON+= `pkg-config --cflags opencv`
endif

ifeq ($(DBUS), 1)
CFLAGS+= -DBUS \
                `pkg-config --cflags dbus-1 glib-2.0 dbus-glib-1 gio-2.0`
LDFLAGS+=`pkg-config --libs dbus-1 glib-2.0 dbus-glib-1 gio-2.0`
endif

ifeq ($(GPU), 1)
COMMON+= -DGPU \
		-I./include \
		-I/usr/local/cuda/include/

CFLAGS+= -DGPU
LDFLAGS+= -L/usr/local/cuda/lib64 -lcuda -lcudart -lcublas -lcurand -lrt
endif

ifeq ($(CUDNN), 1)
COMMON+= -DCUDNN
CFLAGS+= -DCUDNN
LDFLAGS+= -lcudnn
endif

LFLAGS = -Wl,-rpath-link -Wl,`pkg-config --libs opencv`

OBJ=gemm.o utils.o cuda.o lstm_layer.o deconvolutional_layer.o convolutional_layer.o list.o image.o activations.o im2col.o col2im.o blas.o crop_layer.o dropout_layer.o maxpool_layer.o softmax_layer.o data.o matrix.o network.o connected_layer.o cost_layer.o  option_list.o detection_layer.o captcha.o route_layer.o writing.o box.o  normalization_layer.o avgpool_layer.o  dice.o layer.o compare.o local_layer.o swag.o shortcut_layer.o activation_layer.o rnn_layer.o gru_layer.o crnn_layer.o tag.o cifar.o batchnorm_layer.o art.o region_layer.o reorg_layer.o super.o voxel.o tree.o pipe_service.o parser.o queue.o dbus.o robot.o log.o# motor.o led.o brightness.o yolo.o demo.o coco.o detector.o classifier.o rnn_vid.o rnn.o go.o eye_c.o base.o
ifeq ($(GPU), 1)
LDFLAGS+= -lstdc++
OBJ+=deconvolutional_kernels.o convolutional_kernels.o activation_kernels.o im2col_kernels.o col2im_kernels.o blas_kernels.o crop_layer_kernels.o dropout_layer_kernels.o maxpool_layer_kernels.o network_kernels.o avgpool_layer_kernels.o image_core.o
endif

OBJS = $(addprefix $(OBJDIR), $(OBJ))
DEPS = $(wildcard include/*.h) Makefile

all: obj $(OBJS)

#$(EXEC): $(OBJS)
#	$(CC) $(COMMON) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(OBJDIR)%.o: %.c
	$(CC) $(COMMON) $(CFLAGS) -c $< -o $@

$(OBJDIR)%.o: %.cu $(DEPS)
	#$(NVCC) $(ARCH) $(COMMON) --compiler-options "$(CFLAGS)" -c $< -o $@
	$(NVCC)  $(GENCODE_FLAGS) $(COMMON) --compiler-options "$(CFLAGS)" -c $< -o $@


obj:
	mkdir -p obj

.PHONY: clean

clean:
	rm -rf obj
