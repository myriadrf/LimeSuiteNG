# Makefile for kernel module
KERNEL_VERSION:=$(shell uname -r)
KERNEL_PATH?=/lib/modules/$(KERNEL_VERSION)/build
# ARCH?=$(shell uname -m)
# ARCH might be 'aarch64', but the linux lib directories might be named 'arm64'
ARCH?=$(shell if [ "$(shell uname -m)" = "aarch64" ] && [ ! -d $(KERNEL_PATH)/arch/$(shell uname -m) ]; then echo "arm64"; else echo $(shell uname -m); fi)

obj-m = litepcie.o
litepcie-objs = main.o

all: litepcie.ko

export EXTRA_CFLAGS := -std=gnu99 -Wno-declaration-after-statement

litepcie.ko: main.c
	make -C $(KERNEL_PATH) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(shell pwd) modules

litepcie.ko: litepcie.h config.h flags.h csr.h soc.h

clean:
	make -C $(KERNEL_PATH) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(shell pwd) clean
	rm -f *~
