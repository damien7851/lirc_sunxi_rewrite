################################################################################
#
# Common Variables that already set:
#     LICHEE_KDIR
#     LICHEE_MOD_DIR
#     CROSS_COMPILE
#     ARCH
#
#################################################################################
ARCH:= arm
CROSS_COMPILE:= /usr/bin/arm-linux-gnueabihf-
PWD=$(shell pwd)
KDIR:= $(PWD)/..


all: build
#obj-m:=sunxi-lircnew.o
#obj-m+=sunxi-cir.o
#obj-m+=sunxi-lirc.o
obj-m+=sunxi_lirc_new.o
#install: build
#	cp spi-sunxi-slave.ko $(LICHEE_MOD_DIR)/

build:
	@echo $(LICHEE_KDIR)
#	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(LICHEE_KDIR) M=$(PWD) modules
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD)

clean:
	@rm -rf *.o *.ko .*.cmd *.mod.c *.order *.symvers .tmp_versions *~
