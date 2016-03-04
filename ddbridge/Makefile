ddbridge-objs := ddbridge-core.o ddbridge-mod.o

EXTRA_CFLAGS += -DCONFIG_DVB_LNBP21 -DCONFIG_DVB_STV090x -DCONFIG_DVB_STV6110x -g

obj-$(CONFIG_DVB_DDBRIDGE) += ddbridge.o 

EXTRA_CFLAGS += -I$(SUBDIRS)/dvb-core/ -I$(SUBDIRS)/frontends