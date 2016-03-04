#
# Makefile for the nGene device driver
#

ngene-objs := ngene-core.o ngene-i2c.o ngene-cards.o ngene-av.o \
	ngene-eeprom.o ngene-dvb.o

obj-$(CONFIG_DVB_NGENE) += ngene.o

EXTRA_CFLAGS += -I$(SUBDIRS)/dvb-core/
EXTRA_CFLAGS += -I$(SUBDIRS)/frontends/
