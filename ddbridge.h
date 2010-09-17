#ifndef _DDBRIDGE_H_
#define _DDBRIDGE_H_

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <asm/dma.h>
#include <asm/scatterlist.h>
#include <sound/core.h> 
#include <sound/initval.h> 
#include <sound/control.h> 
#include <sound/pcm.h> 
#include <sound/pcm_params.h> 

#include <linux/dvb/frontend.h>
#include <linux/dvb/ca.h>
#include <linux/dvb/video.h>
#include <linux/dvb/audio.h>

#include "dmxdev.h"
#include "dvbdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"
#include "dvb_ringbuffer.h"
#include "dvb_ca_en50221.h"
#include "cxd2099.h"

#define DDB_MAX_I2C     4
#define DDB_MAX_PORT    4
#define DDB_MAX_INPUT   8
#define DDB_MAX_OUTPUT  4

struct ddb_info {
	int   type;
#define DDB_OCTOPUS      0
	char *name;
	int   port_num;
};

#define INPUT_DMA_MAX_BUFS 32
#define INPUT_DMA_BUFS 4
#define INPUT_DMA_SIZE (128*47*21)
#define INPUT_DMA_TOTAL (INPUT_DMA_BUFS*INPUT_DMA_SIZE)

#define OUTPUT_DMA_MAX_BUFS 32
#define OUTPUT_DMA_BUFS 8
#define OUTPUT_DMA_SIZE (128*47*21)
#define OUTPUT_DMA_TOTAL (OUTPUT_DMA_BUFS*OUTPUT_DMA_SIZE)

struct ddb;
struct ddb_port;

struct ddb_input {
	struct ddb_port       *port;
	u32                    nr;
	dma_addr_t             pbuf[INPUT_DMA_MAX_BUFS];
	u8                    *vbuf[INPUT_DMA_MAX_BUFS];
	u32                    buf_size;
	u32                    dma_size;

	struct tasklet_struct  tasklet;
	spinlock_t             lock;
	wait_queue_head_t      wq;
	int                    running;
	u32                    stat;
	u32                    cbuf;
	u32                    coff;

	struct dvb_adapter     adap;
 	struct dvb_device     *dev;
	struct dvb_frontend   *fe;
	struct dmxdev          dmxdev;
	struct dvb_demux       demux;
	struct dmx_frontend    hw_frontend;
	struct dmx_frontend    mem_frontend;
	int                    users;
};

struct ddb_output {
	struct ddb_port       *port;
	u32                    nr;
	dma_addr_t             pbuf[OUTPUT_DMA_MAX_BUFS];
	u8                    *vbuf[OUTPUT_DMA_MAX_BUFS];
	u32                    buf_size;
	u32                    dma_size;
	struct tasklet_struct  tasklet;
	spinlock_t             lock;
	wait_queue_head_t      wq;
	int                    running;
	u32                    stat;
	u32                    cbuf;
	u32                    coff;

	struct dvb_adapter     adap;
 	struct dvb_device     *dev;

	u8                    *buf;
#define TSOUT_BUF_SIZE (512*188*8)
	struct dvb_ringbuffer  rbuf;
};

struct ddb_i2c {
	struct ddb            *dev;
	u32                    nr;
	struct i2c_adapter     adap;
	u32                    regs;
	u32                    rbuf;
	u32                    wbuf;
	int                    done;
	wait_queue_head_t      wq;
};

struct ddb_port {
	struct ddb            *dev;
	u32                    nr;
	struct ddb_i2c        *i2c;
	u32                    class;
#define DDB_PORT_NONE        0
#define DDB_PORT_CI          1
#define DDB_PORT_TUNER       2
	u32                    type;
#define DDB_TUNER_DVBS_ST    0 
#define DDB_TUNER_DVBCT_TR   1
#define DDB_TUNER_DVBCST_SL  2

	struct ddb_input      *input[2];
	struct ddb_output     *output;
	struct dvb_ca_en50221 *en;
};

struct ddb {
	struct pci_dev        *pdev;
	unsigned char         *regs;
	struct ddb_port        port[DDB_MAX_PORT];
	struct ddb_i2c         i2c[DDB_MAX_I2C];
	struct ddb_input       input[DDB_MAX_INPUT];
	struct ddb_output      output[DDB_MAX_OUTPUT];

	struct device         *ddb_dev;
	int                    nr;
	struct ddb_info       *info;
};

/****************************************************************************/

#define ddbwritel(_val, _adr)        writel((_val), \
                                     (char *) (dev->regs+(_adr)))
#define ddbreadl(_adr)               readl((char *) (dev->regs+(_adr)))
#define ddbcpyto(_adr,_src,_count)   memcpy_toio((char *)	\
				     (dev->regs+(_adr)),(_src),(_count))
#define ddbcpyfrom(_dst,_adr,_count) memcpy_fromio((_dst),(char *) \
                                     (dev->regs+(_adr)),(_count))

/****************************************************************************/



#endif
