# Makefile for kernel module
KERNEL_VERSION:=$(shell uname -r)
KERNEL_PATH?=/lib/modules/$(KERNEL_VERSION)/build
# ARCH?=$(shell uname -m)
# ARCH might be 'aarch64', but the linux lib directories might be named 'arm64'
ARCH?=$(shell if [ "$(shell uname -m)" = "aarch64" ] && [ ! -d $(KERNEL_PATH)/arch/$(shell uname -m) ]; then echo "arm64"; else echo $(shell uname -m); fi)

obj-m = limelitepcie.o
litepcie-objs = limelitepcie.ko

all: limelitepcie.ko

export EXTRA_CFLAGS := -std=gnu11 -Wno-declaration-after-statement

limelitepcie.ko: limelitepcie.c
	make -C $(KERNEL_PATH) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(shell pwd) modules

limelitepcie.ko: limelitepcie.h bsp/config.h bsp/flags.h bsp/csr.h bsp/soc.h

clean:
	make -C $(KERNEL_PATH) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(shell pwd) clean
	rm -f *~
