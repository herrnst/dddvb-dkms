ifneq ($(KERNELRELEASE),)

ddbridge-objs := ddbridge-core.o cxd2099.o

obj-$(CONFIG_DVB_DDBRIDGE) += ddbridge.o

EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core/ -Idrivers/media/dvb/frontends -g

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)

MODDEFS := CONFIG_DVB_DDBRIDGE=m

all: 
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) $(MODDEFS) modules

dep:
	DIR=`pwd`; (cd $(TOPDIR); make SUBDIRS=$$DIR dep)

install: all
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules_install

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions Module* modules*

endif

