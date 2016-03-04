/*
 * ddbridge.h: Digital Devices PCIe bridge driver
 *
 * Copyright (C) 2010-2011 Digital Devices GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 * Or, point your browser to http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _DDBRIDGE_H_
#define _DDBRIDGE_H_

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <asm/dma.h>
#include <linux/dvb/frontend.h>
#include <linux/dvb/ca.h>
#include <linux/dvb/video.h>
#include <linux/dvb/audio.h>
#include <linux/socket.h>
#include <linux/device.h>
#include <linux/io.h>

#include "dmxdev.h"
#include "dvbdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"
#include "dvb_ringbuffer.h"
#include "dvb_ca_en50221.h"
#include "dvb_net.h"
#include "cxd2099.h"

#define DDB_MAX_I2C     4
#define DDB_MAX_PORT   10
#define DDB_MAX_INPUT   8
#define DDB_MAX_OUTPUT 10

struct ddb_info {
	int   type;
#define DDB_NONE         0
#define DDB_OCTOPUS      1
#define DDB_OCTOPUS_CI   2
#define DDB_MOD          3
	char *name;
	int   port_num;
	int   i2c_num;
	int   led_num;
	int   fan_num;
	int   temp_num;
	int   temp_bus;
};


/* DMA_SIZE MUST smaller than 256k and 
   MUST be divisible by 188 and 128 !!! */

#define DMA_MAX_BUFS 32      /* hardware table limit */

#define INPUT_DMA_BUFS 8
#define INPUT_DMA_SIZE (128*47*21)
#define INPUT_DMA_IRQ_DIV 1

#define OUTPUT_DMA_BUFS 8
#define OUTPUT_DMA_SIZE (128*47*21)
#define OUTPUT_DMA_IRQ_DIV 1

struct ddb;
struct ddb_port;

struct ddb_dma {
	void                  *io;
	u32                    nr;
	dma_addr_t             pbuf[DMA_MAX_BUFS];
	u8                    *vbuf[DMA_MAX_BUFS];
	u32                    num;
	u32                    size;
	u32                    div;
	u32                    bufreg;

	struct tasklet_struct  tasklet;
	spinlock_t             lock;
	wait_queue_head_t      wq;
	int                    running;
	u32                    stat;
	u32                    ctrl;
	u32                    cbuf;
	u32                    coff;
};

struct ddb_dvb {
	struct dvb_adapter    *adap;
	int                    adap_registered;
	struct dvb_device     *dev;
	struct dvb_frontend   *fe;
	struct dvb_frontend   *fe2;
	struct dmxdev          dmxdev;
	struct dvb_demux       demux;
	struct dvb_net         dvbnet;
	struct dmx_frontend    hw_frontend;
	struct dmx_frontend    mem_frontend;
	int                    users;
	int (*gate_ctrl)(struct dvb_frontend *, int);
	int                    attached;
};

struct ddb_ci {
	struct dvb_ca_en50221  en;
	struct ddb_port       *port;
	u32                    nr;
	struct mutex           lock;
};

struct ddb_io {
	struct ddb_port       *port;
	u32                    nr;
	struct ddb_dma        *dma;
	struct ddb_io         *redirect;
	struct ddb_io         *redo;
	struct ddb_io         *redi;
};

#if 0
struct ddb_input {
	struct ddb_port       *port;
	u32                    nr;
	struct ddb_dma        *dma;
	struct ddb_output     *redo;
	struct ddb_input      *redi;
};

struct ddb_output {
	struct ddb_port       *port;
	u32                    nr;
	struct ddb_dma        *dma;
	struct ddb_output     *redo;
	struct ddb_input      *redi;
};
#else
#define ddb_output ddb_io
#define ddb_input ddb_io
#endif

struct ddb_i2c {
	struct ddb            *dev;
	u32                    nr;
	struct i2c_adapter     adap;
	u32                    regs;
	u32                    rbuf;
	u32                    wbuf;
	int                    done;
	wait_queue_head_t      wq;
	struct mutex           lock;
};

struct ddb_port {
	struct ddb            *dev;
	u32                    nr;
	struct ddb_i2c        *i2c;
	struct mutex           i2c_gate_lock;
	u32                    class;
#define DDB_PORT_NONE           0
#define DDB_PORT_CI             1
#define DDB_PORT_TUNER          2
#define DDB_PORT_LOOP           3
#define DDB_PORT_MOD            4
	u32                    type;
#define DDB_TUNER_NONE          0
#define DDB_TUNER_DVBS_ST       1
#define DDB_TUNER_DVBS_ST_AA    2
#define DDB_TUNER_DVBCT_TR      3
#define DDB_TUNER_DVBCT_ST      4
#define DDB_CI_INTERNAL         5
#define DDB_CI_EXTERNAL_SONY    6
	u32                    adr;

	struct ddb_input      *input[2];
	struct ddb_output     *output;
	struct dvb_ca_en50221 *en;
	struct ddb_dvb         dvb[2];
	u32                    gap;
};

struct ddb {
	struct pci_dev        *pdev;
	const struct pci_device_id *id;
	struct ddb_info       *info;
	int                    msi;

	unsigned char         *regs;
	struct ddb_port        port[DDB_MAX_PORT];
	struct ddb_i2c         i2c[DDB_MAX_I2C];
	struct ddb_input       input[DDB_MAX_INPUT];
	struct ddb_output      output[DDB_MAX_OUTPUT];
	struct dvb_adapter     adap[DDB_MAX_INPUT];
	struct ddb_dma         dma[DDB_MAX_INPUT + DDB_MAX_OUTPUT];

	void                   (*handler[20])(unsigned long);
	unsigned long          handler_data[20];

	struct device         *ddb_dev;
	u32                    nr;
	u8                     iobuf[1028];

	u8                     leds;
	u32                    ts_irq;
	u32                    i2c_irq;

	u32                    hwid;
	u32                    regmap;

	u32                    rate_inc[10];
};


/******************************************************************************/

static inline void ddbwritel(struct ddb *dev, u32 val, u32 adr)
{
	writel(val, (char *) (dev->regs+(adr)));
}

static inline void ddbwritew(struct ddb *dev, u16 val, u32 adr)
{
	writew(val, (char *) (dev->regs+(adr)));
}

static inline u32 ddbreadl(struct ddb *dev, u32 adr)
{
	return readl((char *) (dev->regs+(adr)));
}

#define ddbcpyto(_dev, _adr, _src, _count)   memcpy_toio((char *) \
					(_dev->regs + (_adr)), (_src), (_count))

#define ddbcpyfrom(_dev, _dst, _adr, _count) memcpy_fromio((_dst), (char *) \
					 (_dev->regs + (_adr)), (_count))



#define dd_uint8    u8
#define dd_uint16   u16
#define dd_int16    s16
#define dd_uint32   u32
#define dd_int32    s32
#define dd_uint64   u64
#define dd_int64    s64

#define DDMOD_FLASH_START  0x1000

struct DDMOD_FLASH_DS {
	dd_uint32   Symbolrate;             // kSymbols/s
	dd_uint32   DACFrequency;           // kHz
	dd_uint16   FrequencyResolution;    // kHz
	dd_uint16   IQTableLength;
	dd_uint16   FrequencyFactor;
	dd_int16    PhaseCorr;              // TBD
	dd_uint32   Control2;
	dd_uint16   PostScaleI;   
	dd_uint16   PostScaleQ;
	dd_uint16   PreScale;
	dd_int16    EQTap[11];
	dd_uint16   FlatStart;   
	dd_uint16   FlatEnd;
	dd_uint32   FlashOffsetPrecalculatedIQTables;       // 0 = none
	dd_uint8    Reserved[28];

};

struct DDMOD_FLASH {
	dd_uint32   Magic;
	dd_uint16   Version;
	dd_uint16   DataSets;
	
	dd_uint16   VCORefFrequency;    // MHz
	dd_uint16   VCO1Frequency;      // MHz
	dd_uint16   VCO2Frequency;      // MHz
	
	dd_uint16   DACAux1;    // TBD
	dd_uint16   DACAux2;    // TBD
	
	dd_uint8    Reserved1[238];
	
	struct DDMOD_FLASH_DS DataSet[1];
};

#define DDMOD_FLASH_MAGIC   0x5F564d5F

#endif


