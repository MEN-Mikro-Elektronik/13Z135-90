ifneq ($(KERNELRELEASE),)
	obj-m := men_16z135.o
	ccflags-y := -I/opt/menlinux/INCLUDE/NATIVE/
	ccflags-y += -I/opt/menlinux/INCLUDE/COM/
else
#	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	KERNELDIR ?= /usr/src/linux

default:
	$(MAKE) -C $(KERNELDIR) M=$$PWD modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$$PWD clean
endif
