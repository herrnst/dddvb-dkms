ifneq ($(KERNELRELEASE),)

ddbridge-objs := ddbridge-core.o
drxk-objs := drxk_hard.o 

obj-$(CONFIG_DVB_DDBRIDGE) += ddbridge.o
obj-$(CONFIG_DVB_DRXK) += drxk.o
obj-$(CONFIG_DVB_CXD2099) += cxd2099.o
obj-$(CONFIG_DVB_TDA18271C2DD) += tda18271c2dd.o

EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core/ -Idrivers/media/dvb/frontends -Idrivers/media/common/tuners -g

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)

MODDEFS := CONFIG_DVB_DDBRIDGE=m CONFIG_DVB_DRXK=m CONFIG_DVB_TDA18271C2DD=m CONFIG_DVB_CXD2099=m

all: 
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) $(MODDEFS) modules

dep:
	DIR=`pwd`; (cd $(TOPDIR); make SUBDIRS=$$DIR dep)

install: all
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules_install

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions Module* modules*

endif

