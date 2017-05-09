obj-m := ioctl_dev.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

CCFLAGS := -I$(shell dirname $(PWD))/include/

default:
	$(MAKE) -C $(KDIR) M=$(PWD) ccflags-y=$(CCFLAGS) modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
