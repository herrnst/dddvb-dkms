/*
 * ddbridge.c: 
 *
 * Copyright (C) 2010 Digital Devices UG
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <asm/io.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/smp_lock.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/swab.h>
#include <linux/vmalloc.h>

#include "ddbridge-regs.h"
#include "ddbridge.h"

#include "stv6110x.h"
#include "stv090x.h"
#include "lnbh24.h"

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static int i2c_read_reg(struct i2c_adapter *adapter, u8 adr, 
			u8 reg, u8 *val)
{
        struct i2c_msg msgs[2]={{.addr=adr, .flags=0, 
				 .buf=&reg, .len=1 },
				{.addr=adr, .flags=I2C_M_RD, 
				 .buf=val, .len=1 }};
	return (i2c_transfer(adapter, msgs, 2)==2) ? 0 : -1;
}

static int i2c_read_reg16(struct i2c_adapter *adapter, u8 adr, 
                          u16 reg, u8 *val)
{
        u8 msg[2]={reg>>8, reg&0xff};
        struct i2c_msg msgs[2]={{.addr=adr, .flags=0, 
                                 .buf=msg,  .len=2 },
                                {.addr=adr, .flags=I2C_M_RD, 
                                 .buf=val, .len=1 }};
        return (i2c_transfer(adapter, msgs, 2)==2) ? 0 : -1;
}

static int ddb_i2c_wait(struct ddb_i2c *i2c, u32 adr, u32 cmd)
{
	struct ddb *dev=i2c->dev;
	int stat;

	i2c->done=0;
	ddbwritel((adr<<9)|cmd, i2c->regs+I2C_COMMAND);
	stat=wait_event_timeout(i2c->wq, i2c->done==1, HZ);
	if (stat<=0)
		return -EIO;
	if (ddbreadl(i2c->regs+I2C_COMMAND)&0x70000)
		return -EIO;
	return 0;
}

static int ddb_i2c_master_xfer(struct i2c_adapter *adapter,
			       struct i2c_msg msg[], int num)
{
	struct ddb_i2c *i2c=(struct ddb_i2c *)i2c_get_adapdata(adapter);
	struct ddb *dev=i2c->dev;

	if (num == 2 && msg[1].flags & I2C_M_RD &&
	    !(msg[0].flags & I2C_M_RD)) {
		memcpy_toio(dev->regs+I2C_TASKMEM_BASE+i2c->wbuf,
			    msg[0].buf,msg[0].len);
		ddbwritel(msg[0].len|(msg[1].len<<16), 
			  i2c->regs+I2C_TASKLENGTH);
		if (!ddb_i2c_wait(i2c, msg[0].addr,1)) {
			memcpy_fromio(msg[1].buf, 
				      dev->regs+I2C_TASKMEM_BASE+i2c->rbuf, 
				   msg[1].len);
			return num;
		}
	}
	if (num == 1 && !(msg[0].flags & I2C_M_RD)) {
		ddbcpyto(I2C_TASKMEM_BASE+i2c->wbuf,msg[0].buf,msg[0].len);
		ddbwritel(msg[0].len, i2c->regs+I2C_TASKLENGTH);
		if (!ddb_i2c_wait(i2c, msg[0].addr,2))
			return num;
	}
	if (num == 1 && (msg[0].flags & I2C_M_RD)) {
		ddbwritel(msg[0].len<<16, i2c->regs+I2C_TASKLENGTH);
		if (!ddb_i2c_wait(i2c, msg[0].addr, 3)) {
			ddbcpyfrom(msg[0].buf, 
				   I2C_TASKMEM_BASE+i2c->rbuf,msg[0].len);
			return num;
		}
	}
	return -EIO;
}


static u32 ddb_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL;
}

struct i2c_algorithm ddb_i2c_algo = {
	.master_xfer  = ddb_i2c_master_xfer,
	.functionality = ddb_i2c_functionality,
};

static void ddb_i2c_release(struct ddb *dev)
{
	int i;
	struct ddb_i2c *i2c;
	struct i2c_adapter *adap;

	for (i=0; i<DDB_MAX_I2C; i++) {
		i2c=&dev->i2c[i];
		adap=&i2c->adap;
		i2c_del_adapter(adap);
	}
}

static int ddb_i2c_init(struct ddb *dev)
{
	int i, j, stat=0;
	struct ddb_i2c *i2c;
	struct i2c_adapter *adap;

	for (i=0; i<DDB_MAX_I2C; i++) {
		i2c=&dev->i2c[i];
		i2c->dev=dev;
		i2c->nr=i;
		i2c->wbuf=i*(I2C_TASKMEM_SIZE/4);
		i2c->rbuf=i2c->wbuf+(I2C_TASKMEM_SIZE/8);
		i2c->regs=0x80+i*0x20;
		ddbwritel(I2C_SPEED_100, i2c->regs+I2C_TIMING);
		ddbwritel((i2c->rbuf<<16)|i2c->wbuf, 
			  i2c->regs+I2C_TASKADDRESS);
		init_waitqueue_head(&i2c->wq);
		
		adap=&i2c->adap;
		i2c_set_adapdata(adap, i2c);
#ifdef I2C_ADAP_CLASS_TV_DIGITAL
		adap->class=I2C_ADAP_CLASS_TV_DIGITAL|I2C_CLASS_TV_ANALOG;
#else
		adap->class=I2C_CLASS_TV_ANALOG;
#endif
		strcpy(adap->name, "ddbridge");
		adap->algo=&ddb_i2c_algo;
		adap->algo_data=(void *)i2c;
		adap->dev.parent=&dev->pdev->dev;
		stat=i2c_add_adapter(adap);
		if (stat) 
			break;
	}
	if (stat)
		for (j=0; j<i; j++) {
			i2c=&dev->i2c[j];
			adap=&i2c->adap;
			i2c_del_adapter(adap);
		}
	return stat;
}


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static u8 fill_pack[4] = { 0x47, 0x1f, 0xff, 0x10 };

static void fill_tsbuf(u8 *p, int len)
{
	while (len>0) {
		memcpy(p, fill_pack, 4);
		memset(p+4, 0xff, 184);
		p+=188;
		len-=188;
	}
}

static void ddb_address_table(struct ddb *dev)
{
	u32 i, j, base;
	u64 mem;
	dma_addr_t *pbuf;
	
	for (i=0; i<DDB_MAX_INPUT; i++) {
		base=DMA_BASE_ADDRESS_TABLE+i*0x100;
		pbuf=dev->input[i].pbuf;
		for (j=0; j<INPUT_DMA_BUFS; j++) {
			mem=pbuf[j];
			ddbwritel(mem&0xffffffff, base+j*8);
			ddbwritel(mem>>32, base+j*8+4);
		}
	}
	for (i=0; i<DDB_MAX_OUTPUT; i++) {
		base=DMA_BASE_ADDRESS_TABLE+0x800+i*0x100;
		pbuf=dev->output[i].pbuf;
		for (j=0; j<OUTPUT_DMA_BUFS; j++) {
			mem=pbuf[j];
			ddbwritel(mem&0xffffffff, base+j*8);
			ddbwritel(mem>>32, base+j*8+4);
		}
	}
}

static void io_free(struct pci_dev *pdev, u8 **vbuf,
		    dma_addr_t *pbuf, u32 size, int num)
{
	if (!vbuf[0])
		return;
	if (size&0xfff)
		size=(size&0xfffff000)+0x1000;
	pci_free_consistent(pdev, size*num, vbuf[0], pbuf[0]);
}

static int io_alloc(struct pci_dev *pdev, u8 **vbuf,
		    dma_addr_t *pbuf, u32 size, int num)
{
	int i;

	if (size&0xfff)
		size=(size&0xfffff000)+0x1000;
	vbuf[0]=pci_alloc_consistent(pdev, num*size, &pbuf[0]);
	if (!vbuf[0])
		return -ENOMEM;
	//fill_tsbuf(vbuf[0], num*size);
	//memset(vbuf[0], 0xff, num*size);
	for (i=1; i<num; i++) {
		vbuf[i]=vbuf[0]+i*size;
		pbuf[i]=pbuf[0]+i*size;
	}
	return 0;
}

static int ddb_buffers_alloc(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i=0; i<DDB_MAX_PORT; i++) {
		port=&dev->port[i];
		switch (port->class) {
		case DDB_PORT_TUNER:
			if (io_alloc(dev->pdev, port->input[0]->vbuf, 
				     port->input[0]->pbuf, 
				     INPUT_DMA_SIZE, INPUT_DMA_BUFS)<0)
				return -1;
			if (io_alloc(dev->pdev, port->input[1]->vbuf, 
				     port->input[1]->pbuf, 
				     INPUT_DMA_SIZE, INPUT_DMA_BUFS)<0)
				return -1;
			break;
		case DDB_PORT_CI:
			if (io_alloc(dev->pdev, port->input[0]->vbuf, 
				     port->input[0]->pbuf, 
				     INPUT_DMA_SIZE, INPUT_DMA_BUFS)<0)
				return -1;
			if (io_alloc(dev->pdev, port->output->vbuf, 
				     port->output->pbuf, 
				     OUTPUT_DMA_SIZE, OUTPUT_DMA_BUFS)<0)
				return -1;
			port->output->buf=vmalloc(TSOUT_BUF_SIZE);
			if (!port->output->buf)
				return -ENOMEM;
			dvb_ringbuffer_init(&port->output->rbuf, 
					    port->output->buf, 
					    TSOUT_BUF_SIZE);

			break;
		default:
			break;
		}
	}
	ddb_address_table(dev);
	return 0;
}

static void ddb_buffers_free(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i=0; i<DDB_MAX_PORT; i++) {
		port=&dev->port[i];
		vfree(port->output->buf);
		io_free(dev->pdev, port->input[0]->vbuf, 
			port->input[0]->pbuf, 
			INPUT_DMA_SIZE, INPUT_DMA_BUFS);
		io_free(dev->pdev, port->input[1]->vbuf, 
			port->input[1]->pbuf, 
			INPUT_DMA_SIZE, INPUT_DMA_BUFS);
		io_free(dev->pdev, port->output->vbuf, 
			port->output->pbuf, 
			INPUT_DMA_SIZE, INPUT_DMA_BUFS);
	}
}

static void ddb_input_start(struct ddb_input *input)
{
	struct ddb *dev=input->port->dev;

	printk("start input %d\n", input->nr);
	spin_lock_irq(&input->lock);
	input->cbuf=0;
	input->coff=0;
	ddbwritel(0, TS_INPUT_CONTROL(input->nr));
	ddbwritel(2, TS_INPUT_CONTROL(input->nr));
	ddbwritel(0, TS_INPUT_CONTROL(input->nr));
	ddbwritel((2<<16)|(INPUT_DMA_BUFS<<11)|(INPUT_DMA_SIZE>>7),
		  DMA_BUFFER_SIZE(input->nr));
	ddbwritel(0, DMA_BUFFER_ACK(input->nr));
	
	ddbwritel(1, DMA_BASE_WRITE);
	ddbwritel(1, DMA_BUFFER_CONTROL(input->nr));
	ddbwritel(1, TS_INPUT_CONTROL(input->nr));
	input->running=1;
	spin_unlock_irq(&input->lock);
}

static void ddb_input_stop(struct ddb_input *input)
{
	struct ddb *dev=input->port->dev;

	printk("stop input %d\n", input->nr);
	spin_lock_irq(&input->lock);
	ddbwritel(0, TS_INPUT_CONTROL(input->nr));
	ddbwritel(0, DMA_BUFFER_CONTROL(input->nr));
	input->running=0;
	spin_unlock_irq(&input->lock);
}

static void ddb_output_start(struct ddb_output *output)
{
	struct ddb *dev=output->port->dev;
	int i;

	printk("start output %d\n", output->nr);
	
	for (i=0; i<OUTPUT_DMA_BUFS; i++)
		fill_tsbuf(output->vbuf[i], OUTPUT_DMA_SIZE);

	spin_lock_irq(&output->lock);
	output->cbuf=0;
	output->coff=0;
	ddbwritel(0, TS_OUTPUT_CONTROL(output->nr));
	ddbwritel(2, TS_OUTPUT_CONTROL(output->nr));
	ddbwritel(0, TS_OUTPUT_CONTROL(output->nr));
	ddbwritel(0x1c, TS_OUTPUT_CONTROL(output->nr));
	ddbwritel((4<<16)|(OUTPUT_DMA_BUFS<<11)|(OUTPUT_DMA_SIZE>>7),
		  DMA_BUFFER_SIZE(output->nr+8));
	ddbwritel(0, DMA_BUFFER_ACK(output->nr+8));
	
	ddbwritel(1, DMA_BASE_READ);
	ddbwritel(1, DMA_BUFFER_CONTROL(output->nr+8));
	ddbwritel(0x1d, TS_OUTPUT_CONTROL(output->nr));
	output->running=1;
	spin_unlock_irq(&output->lock);
}

static void ddb_output_stop(struct ddb_output *output)
{
	struct ddb *dev=output->port->dev;

	printk("stop output %d\n", output->nr);
	spin_lock_irq(&output->lock);
	ddbwritel(0, TS_OUTPUT_CONTROL(output->nr));
	ddbwritel(0, DMA_BUFFER_CONTROL(output->nr+8));
	output->running=0;
	spin_unlock_irq(&output->lock);
}

static u32 ddb_output_free(struct ddb_output *output)
{
	//struct ddb *dev=output->port->dev;
	u32 idx, off;

	idx=(output->stat>>11)&0x1f;
	off=output->stat&0x7ff;

	if (output->cbuf!=idx) 
		return 188;
	return 0;
}

static ssize_t ddb_output_write(struct ddb_output* output, 
				const u8 *buf, size_t count)
{
	return count;
}

static u32 ddb_input_avail(struct ddb_input *input)
{
	//struct ddb *dev=input->port->dev;
	u32 idx, off;

	idx=(input->stat>>11)&0x1f;
	off=input->stat&0x7ff;

	printk("buffer %d, off %d   %d %d \n", idx, off, input->cbuf, input->coff);
	if (input->cbuf!=idx) 
		return 188;
	return 0;
}

static size_t ddb_input_read(struct ddb_input *input,
			     u8 *buf, size_t count)
{
	struct ddb *dev=input->port->dev;
	u32 left=count;
	u32 idx, off, free;
	int ret;
	
	idx=(input->stat>>11)&0x1f;
	off=input->stat&0x7ff;

	while (left) {
		if (input->cbuf==idx)
			return count-left;
		//printk("buffer %d, off %d  cbuf %d coff %d\n", 
		//     idx, off, input->cbuf, input->coff);
		free=INPUT_DMA_SIZE-input->coff;
		if (free>left)
			free=left;
		ret=copy_to_user(buf, input->vbuf[input->cbuf]+input->coff, free);
		input->coff+=free;
		if (input->coff==INPUT_DMA_SIZE) {
			input->coff=0;
			input->cbuf=(input->cbuf+1)%INPUT_DMA_BUFS;
		}
		left-=free;
		ddbwritel((input->cbuf<<11)|input->coff, DMA_BUFFER_ACK(input->nr));
	}
	return count;
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static struct stv090x_config stv0900 = {
	.device         = STV0900,
	.demod_mode     = STV090x_DUAL,
	.clk_mode       = STV090x_CLK_EXT,

	.xtal           = 27000000,
	.address        = 0x69,

	.ts1_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,
	.ts2_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,

	.repeater_level = STV090x_RPTLEVEL_16,

	.adc1_range	= STV090x_ADC_1Vpp,
	.adc2_range	= STV090x_ADC_1Vpp,

	.diseqc_envelope_mode = true,
};

static struct stv6110x_config stv6110a = {
	.addr	= 0x60,
	.refclk	= 27000000,
	.clk_div = 1,
};

static struct stv6110x_config stv6110b = {
	.addr	= 0x63,
	.refclk	= 27000000,
	.clk_div = 1,
};

static int demod_attach_stv0900(struct ddb_input *input)
{
	struct i2c_adapter *i2c=&input->port->i2c->adap;
	struct stv090x_config *feconf=&stv0900;

	input->fe=dvb_attach(stv090x_attach, feconf, i2c,
			     (input->nr&1)?STV090x_DEMODULATOR_1
			     : STV090x_DEMODULATOR_0);
	if (!input->fe) {
		printk("No STV0900 found!\n");
		return -ENODEV;
	}
	if (!dvb_attach(lnbh24_attach, input->fe, i2c, 0,
			0, (input->nr&1) ? 0x09 : 0x0b)) {
		printk("No LNBH24 found!\n");
		dvb_frontend_detach(input->fe);
		input->fe=NULL;
		return -ENODEV;
	}
	return 0;
}


static int tuner_attach_stv6110(struct ddb_input *input)
{
	struct i2c_adapter *i2c=&input->port->i2c->adap;
	struct stv090x_config *feconf=&stv0900;
	struct stv6110x_config *tunerconf=(input->nr&1) ? &stv6110b : &stv6110a;
	struct stv6110x_devctl *ctl;

	ctl=dvb_attach(stv6110x_attach, input->fe, tunerconf, i2c);
	if (!ctl) {
		printk("No STV6110X found!\n");
		return -ENODEV;
	}
	
	feconf->tuner_init          = ctl->tuner_init;
	feconf->tuner_sleep         = ctl->tuner_sleep;
	feconf->tuner_set_mode      = ctl->tuner_set_mode;
	feconf->tuner_set_frequency = ctl->tuner_set_frequency;
	feconf->tuner_get_frequency = ctl->tuner_get_frequency;
	feconf->tuner_set_bandwidth = ctl->tuner_set_bandwidth;
	feconf->tuner_get_bandwidth = ctl->tuner_get_bandwidth;
	feconf->tuner_set_bbgain    = ctl->tuner_set_bbgain;
	feconf->tuner_get_bbgain    = ctl->tuner_get_bbgain;
	feconf->tuner_set_refclk    = ctl->tuner_set_refclk;
	feconf->tuner_get_status    = ctl->tuner_get_status;

	return 0;
}

int my_dvb_dmx_ts_card_init(struct dvb_demux *dvbdemux, char *id,
			    int (*start_feed)(struct dvb_demux_feed *),
			    int (*stop_feed)(struct dvb_demux_feed *),
			    void *priv)
{
	dvbdemux->priv = priv;

	dvbdemux->filternum = 256;
	dvbdemux->feednum = 256;
	dvbdemux->start_feed = start_feed;
	dvbdemux->stop_feed = stop_feed;
	dvbdemux->write_to_decoder = NULL;
	dvbdemux->dmx.capabilities = (DMX_TS_FILTERING |
				      DMX_SECTION_FILTERING |
				      DMX_MEMORY_BASED_FILTERING);
	return dvb_dmx_init(dvbdemux);
}

int my_dvb_dmxdev_ts_card_init(struct dmxdev *dmxdev,
			       struct dvb_demux *dvbdemux,
			       struct dmx_frontend *hw_frontend,
			       struct dmx_frontend *mem_frontend,
			       struct dvb_adapter *dvb_adapter)
{
	int ret;

	dmxdev->filternum = 256;
	dmxdev->demux = &dvbdemux->dmx;
	dmxdev->capabilities = 0;
	ret = dvb_dmxdev_init(dmxdev, dvb_adapter);
	if (ret < 0)
		return ret;

	hw_frontend->source = DMX_FRONTEND_0;
	dvbdemux->dmx.add_frontend(&dvbdemux->dmx, hw_frontend);
	mem_frontend->source = DMX_MEMORY_FE;
	dvbdemux->dmx.add_frontend(&dvbdemux->dmx, mem_frontend);
	return dvbdemux->dmx.connect_frontend(&dvbdemux->dmx, hw_frontend);
}

static int start_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx=dvbdmxfeed->demux;
	struct ddb_input *input=dvbdmx->priv;

	if (!input->users) 
		ddb_input_start(input);

	return ++input->users;
}

static int stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx=dvbdmxfeed->demux;
	struct ddb_input *input=dvbdmx->priv;

	if (--input->users)
		return input->users;

	ddb_input_stop(input);
	return 0;
}


static void dvb_input_detach(struct ddb_input *input)
{
	struct dvb_adapter *adap=&input->adap;
	struct dvb_demux *dvbdemux=&input->demux;

	if (input->fe) {
		dvb_unregister_frontend(input->fe);
		dvb_frontend_detach(input->fe);
		input->fe=NULL;
	}
	dvbdemux->dmx.close(&dvbdemux->dmx);
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx,
				      &input->hw_frontend);
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx,
				      &input->mem_frontend);
	dvb_dmxdev_release(&input->dmxdev);
	dvb_dmx_release(&input->demux);
	
	dvb_unregister_adapter(adap);
}

static int dvb_input_attach(struct ddb_input *input)
{
	int ret;
	struct ddb_port *port=input->port;
	struct dvb_adapter *adap=&input->adap;
	struct dvb_demux *dvbdemux=&input->demux;

	ret=dvb_register_adapter(adap, "DDBridge",THIS_MODULE,
				   &input->port->dev->pdev->dev,
				   adapter_nr);
	if (ret < 0)
		return ret;
	
	ret = my_dvb_dmx_ts_card_init(dvbdemux, "SW demux",
				      start_feed,
				      stop_feed, input);
	ret = my_dvb_dmxdev_ts_card_init(&input->dmxdev, &input->demux,
					 &input->hw_frontend,
					 &input->mem_frontend, adap);
	input->fe=0;
	switch (port->type) {
	case DDB_TUNER_DVBS_ST:
		demod_attach_stv0900(input);
		tuner_attach_stv6110(input);
		if (input->fe) {
			if (dvb_register_frontend(adap, input->fe) < 0)
				return -1;
		}
		break;
	}
	return 0;
}


/****************************************************************************/
/****************************************************************************/

static ssize_t ts_write(struct file *file, const char *buf, 
			size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev=file->private_data;
	struct ddb_output *output=dvbdev->priv;
#if 1
	if (wait_event_interruptible(output->rbuf.queue, 
				     dvb_ringbuffer_free
				     (&output->rbuf)>=count)<0)
		return 0;

	dvb_ringbuffer_write(&output->rbuf, buf, count);

	return count;
#else
	size_t left=count;
	while (left) {
		if (wait_event_interruptible(
			    output->wq, ddb_output_free(output)>=188)<0)
			return -EAGAIN;
		left-=ddb_output_write(output, buf, left);
	}
	return count;
#endif
}

static ssize_t ts_read(struct file *file, char *buf, 
		       size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev=file->private_data;
	struct ddb_output *output=dvbdev->priv;
	struct ddb_input *input=output->port->input[0];
	int left, read;

	left=count;
	while (left) {
		if (wait_event_interruptible(
			    input->wq, ddb_input_avail(input)>=188)<0)
			return -EAGAIN;
		read=ddb_input_read(input, buf, left);
		left-=read;
		buf+=read;
	}
	return count;
}

static struct file_operations ci_fops = {
	.owner=		THIS_MODULE,
        .read=		ts_read,
	.write=		ts_write,
	.ioctl=  	0,
	.open=		dvb_generic_open,
	.release=	dvb_generic_release,
	.poll=		0,
	.mmap=          0, 
};

static struct dvb_device dvbdev_ci = {
        .priv=     0,
        .readers= -1,
        .writers= -1,
        .users= -1,
        .fops=    &ci_fops,
};

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void input_tasklet(unsigned long data)
{
	struct ddb_input *input=(struct ddb_input *) data;
	struct ddb *dev=input->port->dev;
	u32 idx, off;

	spin_lock(&input->lock);
	if (!input->running) {
		spin_unlock(&input->lock);
		return;
	}
	input->stat=ddbreadl(DMA_BUFFER_CURRENT(input->nr));
	idx=(input->stat>>11)&0x1f;
	off=input->stat&0x7ff;
	//printk("buffer %d, off %d\n", idx, off);

	if (input->port->class==DDB_PORT_TUNER)
		while (input->cbuf!=idx) {
			dvb_dmx_swfilter_packets(&input->demux, 
						 input->vbuf[input->cbuf],
						 INPUT_DMA_SIZE/188);
			input->cbuf=(input->cbuf+1)%INPUT_DMA_BUFS;
			ddbwritel((input->cbuf<<11), DMA_BUFFER_ACK(input->nr));
		}
	if (input->port->class==DDB_PORT_CI) {
		wake_up(&input->wq);
	}
	spin_unlock(&input->lock);
}


static void output_buf(struct ddb_output *output, int bnum, int len)
{
	u32 alen;

	alen=dvb_ringbuffer_avail(&output->rbuf);
	alen-=alen%188;

	if (alen<len) 
		fill_tsbuf(output->vbuf[bnum]+alen, len-alen);
	else
		alen=len;
	dvb_ringbuffer_read(&output->rbuf, output->vbuf[bnum], alen);
	wake_up(&output->rbuf.queue);
}

static void output_tasklet(unsigned long data)
{
	struct ddb_output *output=(struct ddb_output *) data;
	struct ddb *dev=output->port->dev;
	u32 idx, off;

	spin_lock(&output->lock);
	if (!output->running) {
		spin_unlock(&output->lock);
		return;
	}
	output->stat=ddbreadl(DMA_BUFFER_CURRENT(output->nr+8));
	idx=(output->stat>>11)&0x1f;
	off=output->stat&0x7ff;
#if 0
	printk("buffer %d, off %d\n", idx, off);
	{
		u8 *buf =output->vbuf[output->cbuf];
		printk("%02x %02x %02x %02x \n", buf[0],buf[1],buf[2],buf[3]);
	}
#endif
#if 1
	while (output->cbuf!=idx) {
		output_buf(output, output->cbuf, OUTPUT_DMA_SIZE);
		output->cbuf=(output->cbuf+1)%OUTPUT_DMA_BUFS;
		ddbwritel((output->cbuf<<11), DMA_BUFFER_ACK(output->nr+8));
	}
	//wake_up(&output->wq);
#endif
	spin_unlock(&output->lock);
}

static int ddb_port_attach(struct ddb_port *port)
{
	int ret;

	switch (port->class) {
	case DDB_PORT_TUNER:
		dvb_input_attach(port->input[0]);
		dvb_input_attach(port->input[1]);
		break;
	case DDB_PORT_CI:
		ret=dvb_register_adapter(&port->output->adap, 
					 "DDBridge", 
					 THIS_MODULE,
					 &port->dev->pdev->dev,
					 adapter_nr);
		if (ret<0)
			return ret;
		port->en=cxd2099_attach(0x40, port, &port->i2c->adap);
		if (!port->en)
			return -ENODEV;
		ddb_input_start(port->input[0]);
		ddb_output_start(port->output);
		dvb_ca_en50221_init(&port->output->adap, 
				    port->en, 0, 1);
		dvb_register_device(&port->output->adap, &port->output->dev,
				    &dvbdev_ci, (void *) port->output, 
				    DVB_DEVICE_SEC);
		break;
	default:
		break;
		
	}
	return ret;
}

static int ddb_ports_attach(struct ddb *dev)
{
	int i, ret=0;
	struct ddb_port *port;

	for (i=0; i<DDB_MAX_PORT; i++) {
		port=&dev->port[i];
		ret=ddb_port_attach(port);
		if (ret<0)
			return ret;
	}
	return ret;
}

static void ddb_ports_detach(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i=0; i<DDB_MAX_PORT; i++) {
		port=&dev->port[i];
		switch (port->class) {
		case DDB_PORT_TUNER:
			dvb_input_detach(port->input[0]);
			dvb_input_detach(port->input[1]);
			break;
		case DDB_PORT_CI:
			ddb_input_stop(port->input[0]);
			ddb_output_stop(port->output);
			if (port->output->dev)
				dvb_unregister_device(port->output->dev);
			if (port->en) {
				dvb_ca_en50221_release(port->en);
				kfree(port->en);
				port->en=0;
				dvb_unregister_adapter(&port->output->adap);
			}
			break;
		}
	}
}
/****************************************************************************/
/****************************************************************************/

static int port_has_ci(struct ddb_port *port)
{
	u8 val;
	return (i2c_read_reg(&port->i2c->adap, 0x40, 0, &val) ? 0 : 1);
}

static int port_has_stv0900(struct ddb_port *port)
{
	u8 val;
	if (i2c_read_reg16(&port->i2c->adap, 0x69, 0xf100, &val)<0)
		return 0;
	//printk("STV0900 revision %d\n", val);
	return 1;
}

static void ddb_port_probe(struct ddb_port *port)
{
	port->class=DDB_PORT_NONE;
	if (port_has_ci(port)) {
		printk("Port %d: CI\n", port->nr);
		port->class=DDB_PORT_CI;
		ddbwritel(I2C_SPEED_100, i2c->regs+I2C_TIMING);
		return;
	}
	if (port_has_stv0900(port)) {
		printk("Port %d: DUAL DVB-S2\n", port->nr);
		port->class=DDB_PORT_TUNER;
		port->type=DDB_TUNER_DVBS_ST;
		return;
	}
}

static void ddb_input_init(struct ddb_port *port, int nr)
{
	struct ddb *dev=port->dev;
	struct ddb_input *input=&dev->input[nr];
	input->nr=nr;
	input->port=port;
	ddbwritel(0, TS_INPUT_CONTROL(nr));
	ddbwritel(2, TS_INPUT_CONTROL(nr));
	ddbwritel(0, TS_INPUT_CONTROL(nr));
	ddbwritel((2<<16)|(INPUT_DMA_BUFS<<11)|(INPUT_DMA_SIZE>>7),
		  DMA_BUFFER_SIZE(nr));
	ddbwritel(0, DMA_BUFFER_ACK(nr));
	tasklet_init(&input->tasklet, input_tasklet, (unsigned long) input);
	spin_lock_init(&input->lock);
	init_waitqueue_head(&input->wq);
}

static void ddb_output_init(struct ddb_port *port, int nr)
{
	struct ddb *dev=port->dev;
	struct ddb_output *output=&dev->output[nr];
	output->nr=nr;
	output->port=port;

	ddbwritel(0, TS_OUTPUT_CONTROL(nr));
	ddbwritel(2, TS_OUTPUT_CONTROL(nr));
	ddbwritel(0, TS_OUTPUT_CONTROL(nr));
	ddbwritel((2<<16)|(OUTPUT_DMA_BUFS<<11)|(OUTPUT_DMA_SIZE>>7),
		  DMA_BUFFER_SIZE(nr+8));
	tasklet_init(&output->tasklet, output_tasklet, (unsigned long) output);
	init_waitqueue_head(&output->wq);
}

static void ddb_ports_init(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i=0; i<DDB_MAX_PORT; i++) {
		port=&dev->port[i];
		port->dev=dev;
		port->nr=i;
		port->i2c=&dev->i2c[i];
		port->input[0]=&dev->input[2*i];
		port->input[1]=&dev->input[2*i+1];
		port->output=&dev->output[i];

		ddb_input_init(port, 2*i);
		ddb_input_init(port, 2*i+1);
		ddb_output_init(port, i);

		ddb_port_probe(port);
	}
}

static void ddb_ports_release(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i=0; i<DDB_MAX_PORT; i++) {
		port=&dev->port[i];
		port->dev=dev;
		tasklet_kill(&port->input[0]->tasklet);
		tasklet_kill(&port->input[1]->tasklet);
		tasklet_kill(&port->output->tasklet);
	}
}


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static int read_flash(struct ddb *dev, u32 address, u8 *buf, u32 buflen)
{
	u32 data=0;

	if (!buflen)
		return 0;

	ddbwritel(1, SPI_CONTROL);
	ddbwritel(0x03000000|(address&0xFFFFFF), SPI_DATA);
	
	while (ddbreadl(SPI_CONTROL)&0x0004);
	
	while (buflen>4) {
		ddbwritel(0xffffffff, SPI_DATA);
		while (ddbreadl(SPI_CONTROL)&0x0004);
		data=ddbreadl(SPI_DATA);
		*(u32 *) buf=swab32(data);
		buf+=4;
		buflen-=4;
	}
	ddbwritel(0x0003|((buflen<<(8+3))&0x1F00), SPI_CONTROL);
	ddbwritel(0xffffffff, SPI_DATA);
	while (ddbreadl(SPI_CONTROL)&0x0004);
	
	data=ddbreadl(SPI_DATA);
	ddbwritel(0, SPI_CONTROL);
	
	if (buflen<4)
		data<<=((4-buflen)*8);
	
	while (buflen>0) {
		*buf++=((data>>24)&0xff);
		data<<=8;
		buflen--;
	}
	return 0;
}

static int flashio(struct ddb *dev, u8 *wbuf, u32 wlen, u8 *rbuf, u32 rlen)
{
	u32 data, shift;
	
	if (wlen>4)
		ddbwritel(1, SPI_CONTROL);
	while (wlen>4) {
		data=swab32(*(u32 *)wbuf);
		wbuf+=4;
		wlen-=4;
		ddbwritel(data, SPI_DATA);
		while (ddbreadl(SPI_CONTROL)&0x0004);
	}
	
	if (rlen)
		ddbwritel(0x0001|((wlen<<(8+3))&0x1f00), SPI_CONTROL);
	else
		ddbwritel(0x0003|((wlen<<(8+3))&0x1f00), SPI_CONTROL);
	
	data=0;
	shift=((4-wlen)*8);
	while (wlen) {
		data<<=8;
		data|=*wbuf;
		wlen--;
		wbuf++;
	}
	if (shift)
		data<<=shift;
	ddbwritel(data, SPI_DATA);
	while (ddbreadl(SPI_CONTROL)&0x0004);
	
	if (!rlen) {
		ddbwritel(0, SPI_CONTROL);
		return 0;
	}
	if (rlen>4)
		ddbwritel(1, SPI_CONTROL);
	
	while (rlen>4) {
		ddbwritel(0xffffffff, SPI_DATA);
		while (ddbreadl(SPI_CONTROL)&0x0004);
		data=ddbreadl(SPI_DATA);
		*(u32 *)rbuf=swab32(data);
		rbuf+=4;
		rlen-=4;
	}
	ddbwritel(0x0003|((rlen<<(8+3))&0x1F00), SPI_CONTROL);
	ddbwritel(0xffffffff, SPI_DATA);
	while (ddbreadl(SPI_CONTROL)&0x0004);
	
	data=ddbreadl(SPI_DATA);
	ddbwritel(0, SPI_CONTROL);
	
	if (rlen<4)
		data<<=((4-rlen)*8);
	
	while (rlen>0) {
		*rbuf++=((data>>24)&0xff);
		data<<=8;
		rlen--;
	}
	return 0;
}

static void dump(u8 *buf, u32 len)
{
	u32 i, j;

	for (i=0; i<len; i+=8) {
		for (j=0; j<8 && i+j<len; j++) {
			printk("%02x ", buf[i+j]);
		}
		printk("\n");
	}
}

static void flash_test(struct ddb *dev) 
{
	u8 buf[256];

	read_flash(dev, 0x0, buf, sizeof(buf));
	dump(buf, sizeof(buf));
}


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void irq_handle_i2c(struct ddb *dev, int n)
{
	struct ddb_i2c *i2c=&dev->i2c[n];

	i2c->done=1;
	wake_up(&i2c->wq);
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	struct ddb *dev=(struct ddb *) dev_id;
	u32 istat=ddbreadl(INTERRUPT_STATUS);

	if (!istat)
		return IRQ_NONE;
	ddbwritel(istat, INTERRUPT_ACK);
	
	if (istat&0x00000001) irq_handle_i2c(dev, 0);
	if (istat&0x00000002) irq_handle_i2c(dev, 1);
	if (istat&0x00000004) irq_handle_i2c(dev, 2);
	if (istat&0x00000008) irq_handle_i2c(dev, 3);

	if (istat&0x00000100) tasklet_schedule(&dev->input[0].tasklet);
	if (istat&0x00000200) tasklet_schedule(&dev->input[1].tasklet);
	if (istat&0x00000400) tasklet_schedule(&dev->input[2].tasklet);
	if (istat&0x00000800) tasklet_schedule(&dev->input[3].tasklet);
	if (istat&0x00001000) tasklet_schedule(&dev->input[4].tasklet);
	if (istat&0x00002000) tasklet_schedule(&dev->input[5].tasklet);
	if (istat&0x00004000) tasklet_schedule(&dev->input[6].tasklet);
	if (istat&0x00008000) tasklet_schedule(&dev->input[7].tasklet);

	if (istat&0x00010000) tasklet_schedule(&dev->output[0].tasklet);
	if (istat&0x00020000) tasklet_schedule(&dev->output[1].tasklet);
	if (istat&0x00040000) tasklet_schedule(&dev->output[2].tasklet);
	if (istat&0x00080000) tasklet_schedule(&dev->output[3].tasklet);

	//if (istat&0x000f0000)	printk("%08x\n", istat);

	return IRQ_HANDLED;
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#define DDB_MAGIC 'd'

struct ddb_flashio {
	__u8 *write_buf;
	__u32 write_len;
	__u8 *read_buf;
	__u32 read_len;
};

#define IOCTL_DDB_FLASHIO  _IOWR(DDB_MAGIC, 0x00, struct ddb_flashio)



#define DDB_NAME "ddbridge"

static u32 ddb_num;
static struct ddb *ddbs[32];
static struct class *ddb_class;
static int ddb_major;

static int ddb_open(struct inode *inode, struct file *file)
{
	struct ddb *dev = ddbs[iminor(inode)];
	
	file->private_data=dev;
	return 0;
}

static int ddb_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	struct ddb *dev=file->private_data;
	void *parg=(void *)arg;
	int res=-EFAULT;

	switch (cmd) {
	case IOCTL_DDB_FLASHIO:
	{
		struct ddb_flashio fio;
		u8 *rbuf, *wbuf;
		if (copy_from_user(&fio, parg, sizeof(fio)))
			break;
		wbuf=vmalloc(fio.write_len+fio.read_len);
		if (!wbuf)
			return -ENOMEM;
		rbuf=wbuf+fio.write_len;
		if (copy_from_user(wbuf, fio.write_buf, fio.write_len)) {
			vfree(wbuf);
			break;
		}
#if 1
		res=flashio(dev, wbuf, fio.write_len,
			    rbuf, fio.read_len);
#else
		printk("flashio %d %d\n", fio.write_len, fio.read_len);
		dump(wbuf, (fio.write_len>20)?20:fio.write_len);
		if (wbuf[0]==0xd7)
			rbuf[0]=0x80;
		res=0;
#endif
		if (copy_to_user(fio.read_buf, rbuf, fio.read_len))
			res=-EFAULT;
		vfree(wbuf);
		break;
	}
	default:
		break;
	}
	return res;
}


static struct file_operations ddb_fops={
	.ioctl  =  ddb_ioctl,
	.open   =  ddb_open,
};

static char *ddb_devnode(struct device *device, mode_t *mode)
{
	struct ddb *dev=dev_get_drvdata(device);

	return kasprintf(GFP_KERNEL, "ddbridge/card%d", dev->nr);
}

static int ddb_class_create(void)
{
	if ((ddb_major=register_chrdev(0, DDB_NAME, &ddb_fops))<0)
		return ddb_major;
	
	ddb_class=class_create(THIS_MODULE, DDB_NAME);
	if (IS_ERR(ddb_class)) {
		unregister_chrdev(ddb_major, DDB_NAME);
		return -1;
	}
	ddb_class->devnode=ddb_devnode;
	return 0;
}

static void ddb_class_destroy(void)
{
	class_destroy(ddb_class);
	unregister_chrdev(ddb_major, DDB_NAME);
}

static int ddb_device_create(struct ddb *dev)
{
	dev->nr=ddb_num++;
	dev->ddb_dev=device_create(ddb_class, NULL, MKDEV(ddb_major, 0), 
				   dev, "ddbridge%d", dev->nr);
	ddbs[dev->nr]=dev;
	if (IS_ERR(dev->ddb_dev))
		return -1;
	return 0;
}

static void ddb_device_destroy(struct ddb *dev)
{
	ddb_num--;
	if (IS_ERR(dev->ddb_dev))
		return;
	device_destroy(ddb_class, MKDEV(ddb_major, 0));
}


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void ddb_unmap(struct ddb *dev)
{
	if (dev->regs)
		iounmap(dev->regs);
	vfree(dev);
}


static void __devexit ddb_remove(struct pci_dev *pdev)
{
	struct ddb *dev=(struct ddb *) pci_get_drvdata(pdev);

	ddb_ports_detach(dev);
	ddb_i2c_release(dev);

	ddbwritel(0, INTERRUPT_ENABLE);
	free_irq(dev->pdev->irq, dev);

	ddb_ports_release(dev);
	ddb_buffers_free(dev);
	ddb_device_destroy(dev);

	ddb_unmap(dev);
	pci_set_drvdata(pdev, 0);
	pci_disable_device(pdev);
}


static int __devinit ddb_probe(struct pci_dev *pdev,
			       const struct pci_device_id *id)
{
	struct ddb *dev;
	int stat=0;

	if (pci_enable_device(pdev)<0) 
		return -ENODEV;

	dev=vmalloc(sizeof(struct ddb));
	if (dev==NULL) 
		return -ENOMEM;
	memset(dev, 0, sizeof(struct ddb));

	dev->pdev=pdev;
	pci_set_drvdata(pdev, dev);
	dev->info=(struct ddb_info *) id->driver_data;
	printk("DDBridge driver detected: %s\n", dev->info->name);

	dev->regs=ioremap(pci_resource_start(dev->pdev,0),
			  pci_resource_len(dev->pdev,0));
	if (dev->regs==NULL) {
		stat=-ENOMEM;
		goto fail;
	}

	if ((stat=request_irq(dev->pdev->irq, irq_handler, 
			      IRQF_SHARED, "DDBridge", 
			      (void *) dev))<0) 
		goto fail1;
	ddbwritel(0, DMA_BASE_WRITE);
	ddbwritel(0, DMA_BASE_READ);
	ddbwritel(0xffffffff, INTERRUPT_ACK);
	ddbwritel(0xfff0f, INTERRUPT_ENABLE);
	ddbwritel(0, MSI1_ENABLE);

	if (ddb_i2c_init(dev)<0) 
		goto fail1;
	ddb_ports_init(dev);
	if (ddb_buffers_alloc(dev)<0) 
		goto fail2;
	if (ddb_ports_attach(dev)<0) {
		ddb_ports_detach(dev);
		goto fail3;
	}
	ddb_device_create(dev);
	return 0;
fail3:
	ddb_ports_release(dev);
fail2:
	ddb_buffers_free(dev);
fail1:
	free_irq(dev->pdev->irq, dev);
fail:
	ddb_unmap(dev);
	pci_set_drvdata(pdev, 0);
	pci_disable_device(pdev);
	return stat;
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static struct ddb_info ddb_octopus = { 
	.type         = DDB_OCTOPUS, 
	.name         = "Digital Devices Octopus DVB adapter",
	.port_num     = 4,
};

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#define DDB_ID(_vend, _dev, _subvend,_subdev,_driverdata) {	\
	.vendor=_vend, .device=_dev, \
	.subvendor=_subvend, .subdevice=_subdev, \
        .driver_data=(unsigned long) &_driverdata }

/****************************************************************************/

static const struct pci_device_id ddb_id_tbl[] __devinitdata = {
	DDB_ID( 0xdd00, 0x0002, 0xdd00, 0x0001, ddb_octopus ),
	DDB_ID( 0xdd01, 0x0002, 0xdd01, 0x0001, ddb_octopus ),
	DDB_ID( 0xdd01, 0x0003, 0xdd01, 0x0001, ddb_octopus ),
};

static struct pci_driver ddb_pci_driver = {
	.name        = "DDBridge",
	.id_table    = ddb_id_tbl,
	.probe       = ddb_probe,
	.remove      = ddb_remove,
};

static __init int module_init_ddbridge(void) {
    	printk("Digital Devices PCIE bridge driver, "
	       "Copyright (C) 2010 Digital Devices\n");
	if (ddb_class_create())
		return -1;
	return pci_register_driver(&ddb_pci_driver);
}

static __exit void module_exit_ddbridge(void)
{
	pci_unregister_driver(&ddb_pci_driver);
	ddb_class_destroy();
}

module_init(module_init_ddbridge);
module_exit(module_exit_ddbridge);

MODULE_DESCRIPTION("Digital Devices PCIe Bridge");
MODULE_AUTHOR("Ralph Metzler");
MODULE_LICENSE("GPL");
