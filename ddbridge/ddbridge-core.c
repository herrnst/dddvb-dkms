/*
 * ddbridge.c: Digital Devices PCIe bridge driver
 *
 * Copyright (C) 2010-2012 Digital Devices GmbH
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

//#define TEST_I2C_LOCK
#undef CONFIG_PCI_MSI

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/swab.h>
#include <linux/vmalloc.h>
#include "ddbridge.h"
#include "ddbridge-regs.h"

#include "tda18271c2dd.h"
#include "stv6110x.h"
#include "stv090x.h"
#include "lnbh24.h"
#include "drxk.h"
#include "stv0367.h"
#include "stv0367dd.h"
#include "tda18212.h"
#include "tda18212dd.h"

static int adapter_alloc;
module_param(adapter_alloc, int, 0444);
MODULE_PARM_DESC(adapter_alloc, "0-one adapter per io, 1-one per tab with io, 2-one per tab, 3-one for all");

static int ts_loop = -1;
module_param(ts_loop, int, 0444);
MODULE_PARM_DESC(ts_loop, "TS in/out test loop on port ts_loop");

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static struct ddb *ddbs[32];

DEFINE_MUTEX(redirect_lock);

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

static int i2c_write(struct i2c_adapter *adap, u8 adr, u8 *data, int len)
{
	struct i2c_msg msg = {.addr = adr, .flags = 0, .buf = data, .len = len};

	return (i2c_transfer(adap, &msg, 1) == 1) ? 0 : -1;
}

static int i2c_read(struct i2c_adapter *adapter, u8 adr, u8 *val)
{
	struct i2c_msg msgs[1] = {{.addr = adr,  .flags = I2C_M_RD,
				   .buf  = val,  .len   = 1 } };
	return (i2c_transfer(adapter, msgs, 1) == 1) ? 0 : -1;
}

static int i2c_read_regs(struct i2c_adapter *adapter,
			 u8 adr, u8 reg, u8 *val, u8 len)
{
	struct i2c_msg msgs[2] = {{.addr = adr,  .flags = 0,
				   .buf  = &reg, .len   = 1 },
				  {.addr = adr,  .flags = I2C_M_RD,
				   .buf  = val,  .len   = len } };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

static int i2c_read_regs16(struct i2c_adapter *adapter, 
			   u8 adr, u16 reg, u8 *val, u8 len)
{
	u8 reg16[2] = { reg >> 8, reg };
	struct i2c_msg msgs[2] = {{.addr = adr,  .flags = 0,
				   .buf  = reg16, .len   = 2 },
				  {.addr = adr,  .flags = I2C_M_RD,
				   .buf  = val,  .len   = len } };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

static int i2c_read_reg(struct i2c_adapter *adapter, u8 adr, u8 reg, u8 *val)
{
	struct i2c_msg msgs[2] = {{.addr = adr,  .flags = 0,
				   .buf  = &reg, .len   = 1},
				  {.addr = adr,  .flags = I2C_M_RD,
				   .buf  = val,  .len   = 1 } };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

static int i2c_read_reg16(struct i2c_adapter *adapter, u8 adr,
			  u16 reg, u8 *val)
{
	u8 msg[2] = {reg >> 8, reg & 0xff};
	struct i2c_msg msgs[2] = {{.addr = adr, .flags = 0,
				   .buf  = msg, .len   = 2},
				  {.addr = adr, .flags = I2C_M_RD,
				   .buf  = val, .len   = 1 } };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

static int i2c_write_reg16(struct i2c_adapter *adap, u8 adr,
			   u16 reg, u8 val)
{
	u8 msg[3] = {reg >> 8, reg & 0xff, val};

	return i2c_write(adap, adr, msg, 3);
}

static int i2c_write_reg8(struct i2c_adapter *adap, u8 adr,
			  u8 reg, u8 val)
{
	u8 msg[2] = {reg, val};

	return i2c_write(adap, adr, msg, 2);
}

static int ddb_i2c_cmd(struct ddb_i2c *i2c, u32 adr, u32 cmd)
{
	struct ddb *dev = i2c->dev;
	int stat;
	u32 val;

	i2c->done = 0;
	ddbwritel(dev, (adr << 9) | cmd, i2c->regs + I2C_COMMAND);
	stat = wait_event_timeout(i2c->wq, i2c->done == 1, HZ);
	if (stat <= 0) {
		printk(KERN_ERR "DDBridge I2C timeout, card %d, port %d\n",
		       dev->nr, i2c->nr);
		{ /* MSI debugging*/
			u32 istat = ddbreadl(dev, INTERRUPT_STATUS);
			printk(KERN_ERR "DDBridge IRS %08x\n", istat);
			ddbwritel(dev, istat, INTERRUPT_ACK);
		}
		return -EIO;
	}
	val = ddbreadl(dev, i2c->regs+I2C_COMMAND);
	if (val & 0x70000)
		return -EIO;
	return 0;
}

static int ddb_i2c_master_xfer(struct i2c_adapter *adapter,
			       struct i2c_msg msg[], int num)
{
	struct ddb_i2c *i2c = (struct ddb_i2c *) i2c_get_adapdata(adapter);
	struct ddb *dev = i2c->dev;
	u8 addr = 0;

	if (num)
		addr = msg[0].addr;
#ifdef TEST_I2C_LOCK
	if (!mutex_trylock(&i2c->lock)) {
		printk(KERN_ERR "I2C lock error\n");
		return -EBUSY;
	}
#endif
	if (num == 2 && msg[1].flags & I2C_M_RD &&
	    !(msg[0].flags & I2C_M_RD)) {
		memcpy_toio(dev->regs + I2C_TASKMEM_BASE + i2c->wbuf,
			    msg[0].buf, msg[0].len);
		ddbwritel(dev, msg[0].len|(msg[1].len << 16),
			  i2c->regs + I2C_TASKLENGTH);
		if (!ddb_i2c_cmd(i2c, addr, 1)) {
			memcpy_fromio(msg[1].buf,
				      dev->regs + I2C_TASKMEM_BASE + i2c->rbuf,
				      msg[1].len);
#ifdef TEST_I2C_LOCK
			mutex_unlock(&i2c->lock);
#endif
			return num;
		}
	}
	if (num == 1 && !(msg[0].flags & I2C_M_RD)) {
		ddbcpyto(dev, I2C_TASKMEM_BASE + i2c->wbuf, 
			 msg[0].buf, msg[0].len);
		ddbwritel(dev, msg[0].len, i2c->regs + I2C_TASKLENGTH);
		if (!ddb_i2c_cmd(i2c, addr, 2)) {
#ifdef TEST_I2C_LOCK
			mutex_unlock(&i2c->lock);
#endif
			return num;
		}
	}
	if (num == 1 && (msg[0].flags & I2C_M_RD)) {
		ddbwritel(dev, msg[0].len << 16, i2c->regs + I2C_TASKLENGTH);
		if (!ddb_i2c_cmd(i2c, addr, 3)) {
			ddbcpyfrom(dev, msg[0].buf,
				   I2C_TASKMEM_BASE + i2c->rbuf, msg[0].len);
#ifdef TEST_I2C_LOCK
			mutex_unlock(&i2c->lock);
#endif
			return num;
		}
	}
#ifdef TEST_I2C_LOCK
	mutex_unlock(&i2c->lock);
#endif
	return -EIO;
}


static u32 ddb_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL;
}

struct i2c_algorithm ddb_i2c_algo = {
	.master_xfer   = ddb_i2c_master_xfer,
	.functionality = ddb_i2c_functionality,
};

static void ddb_i2c_release(struct ddb *dev)
{
	int i;
	struct ddb_i2c *i2c;
	struct i2c_adapter *adap;

	for (i = 0; i < dev->info->i2c_num; i++) {
		i2c = &dev->i2c[i];
		adap = &i2c->adap;
		i2c_del_adapter(adap);
	}
}

static void i2c_handler(unsigned long priv)
{
	struct ddb_i2c *i2c = (struct ddb_i2c *) priv; 

	i2c->done = 1;
	wake_up(&i2c->wq);
}

static int ddb_i2c_init(struct ddb *dev)
{
	int i, j, stat = 0;
	struct ddb_i2c *i2c;
	struct i2c_adapter *adap;
	
	for (i = 0; i < dev->info->i2c_num; i++) {
		i2c = &dev->i2c[i];
		dev->handler[i] = i2c_handler;
		dev->handler_data[i] = (unsigned long) i2c;
		i2c->dev = dev;
		i2c->nr = i;
		i2c->wbuf = i * (I2C_TASKMEM_SIZE / 4);
		i2c->rbuf = i2c->wbuf + (I2C_TASKMEM_SIZE / 8);
		i2c->regs = 0x80 + i * 0x20;
		ddbwritel(dev, I2C_SPEED_100, i2c->regs + I2C_TIMING);
		ddbwritel(dev, (i2c->rbuf << 16) | i2c->wbuf,
			  i2c->regs + I2C_TASKADDRESS);
		init_waitqueue_head(&i2c->wq);
#ifdef TEST_I2C_LOCK
		mutex_init(&i2c->lock);
#endif
		adap = &i2c->adap;
		i2c_set_adapdata(adap, i2c);
#ifdef I2C_ADAP_CLASS_TV_DIGITAL
		adap->class = I2C_ADAP_CLASS_TV_DIGITAL|I2C_CLASS_TV_ANALOG;
#else
#ifdef I2C_CLASS_TV_ANALOG
		adap->class = I2C_CLASS_TV_ANALOG;
#endif
#endif
		strcpy(adap->name, "ddbridge");
		adap->algo = &ddb_i2c_algo;
		adap->algo_data = (void *)i2c;
		adap->dev.parent = &dev->pdev->dev;
		stat = i2c_add_adapter(adap);
		if (stat)
			break;
	}
	if (stat)
		for (j = 0; j < i; j++) {
			i2c = &dev->i2c[j];
			adap = &i2c->adap;
			i2c_del_adapter(adap);
		}
	return stat;
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

static void ddb_set_dma_table(struct ddb *dev, struct ddb_dma *dma)
{
	u32 i, base;
	u64 mem;

	if (!dma)
		return;
	base = DMA_BASE_ADDRESS_TABLE + dma->nr * 0x100;
	for (i = 0; i < dma->num; i++) {
		mem = dma->pbuf[i];
		ddbwritel(dev, mem & 0xffffffff, base + i * 8);
		ddbwritel(dev, mem >> 32, base + i * 8 + 4);
	}
	dma->bufreg = (dma->div << 16) | 
		((dma->num & 0x1f) << 11) | 
		((dma->size >> 7) & 0x7ff);
}

static void ddb_set_dma_tables(struct ddb *dev)
{
	u32 i;

	for (i = 0; i < dev->info->port_num * 2; i++)
		ddb_set_dma_table(dev, dev->input[i].dma);
	for (i = 0; i < dev->info->port_num; i++)
		ddb_set_dma_table(dev, dev->output[i].dma);
}

static void dma_free(struct pci_dev *pdev, struct ddb_dma *dma)
{
	int i;

	if (!dma)
		return;
	for (i = 0; i < dma->num; i++) {
		if (dma->vbuf[i]) {
			pci_free_consistent(pdev, dma->size,
					    dma->vbuf[i], dma->pbuf[i]);
			dma->vbuf[i] = 0;
		}
	}
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

static void ddb_redirect_dma(struct ddb *dev,
			     struct ddb_dma *sdma,
			     struct ddb_dma *ddma)
{
	u32 i, base;
	u64 mem;

	sdma->bufreg = ddma->bufreg;
	base = DMA_BASE_ADDRESS_TABLE + sdma->nr * 0x100;
	for (i = 0; i < ddma->num; i++) {
		mem = ddma->pbuf[i];
		ddbwritel(dev, mem & 0xffffffff, base + i * 8);
		ddbwritel(dev, mem >> 32, base + i * 8 + 4);
	}
}

static int ddb_unredirect(struct ddb_port *port)
{
	struct ddb_input *oredi, *iredi = 0;
	struct ddb_output *iredo = 0;
	
	printk("unredirect %d.%d\n", port->dev->nr, port->nr);
	mutex_lock(&redirect_lock);
	if (port->output->dma->running) {
		mutex_unlock(&redirect_lock);
		return -EBUSY;
	}
	oredi = port->output->redi;
	if (!oredi)
		goto done;
	if (port->input[0]) {
		iredi = port->input[0]->redi;
		iredo = port->input[0]->redo;
		
		if (iredo) {
			iredo->port->output->redi = oredi;
			if (iredo->port->input[0]) {
				iredo->port->input[0]->redi = iredi;
				ddb_redirect_dma(oredi->port->dev, 
						 oredi->dma, iredo->dma);
			}
			port->input[0]->redo = 0;
			ddb_set_dma_table(port->dev, port->input[0]->dma);
		} 
		oredi->redi = iredi;
		port->input[0]->redi = 0;
	}
	oredi->redo = 0;
	port->output->redi = 0;

	ddb_set_dma_table(oredi->port->dev, oredi->dma);
done:
	mutex_unlock(&redirect_lock);
	return 0;
}

static int ddb_redirect(u32 i, u32 p)
{
	struct ddb *idev = ddbs[(i >> 4) & 0x1f];
	struct ddb_input *input, *input2;
	struct ddb *pdev = ddbs[(p >> 4) & 0x1f];
	struct ddb_port *port;

	if (!idev || !pdev)
		return -EINVAL;

	port = &pdev->port[p & 0x0f];
	if (!port->output)
		return -EINVAL;
	if (ddb_unredirect(port))
		return -EBUSY;

	if (i == 8)
		return 0;

	input = &idev->input[i & 7];
	if (!input) 
		return -EINVAL;

	mutex_lock(&redirect_lock);
	if (port->output->dma->running || input->dma->running) {
		mutex_unlock(&redirect_lock);
		return -EBUSY;
	}
	if ((input2 = port->input[0])) {
		if (input->redi) {
			input2->redi = input->redi;
			input->redi = 0;
		} else
			input2->redi = input;
	}
	input->redo = port->output;
	port->output->redi = input;

	ddb_redirect_dma(input->port->dev, input->dma, port->output->dma);
	mutex_unlock(&redirect_lock);
	return 0;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

static int dma_alloc(struct pci_dev *pdev, struct ddb_dma *dma)
{
	int i;

	if (!dma)
		return 0;
	for (i = 0; i < dma->num; i++) {
		dma->vbuf[i] = pci_alloc_consistent(pdev, dma->size,
						    &dma->pbuf[i]);
		if (!dma->vbuf[i])
			return -ENOMEM;
	}
	return 0;
}

static int ddb_buffers_alloc(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		switch (port->class) {
		case DDB_PORT_TUNER:
			if (dma_alloc(dev->pdev, port->input[0]->dma) < 0)
				return -1;
			if (dma_alloc(dev->pdev, port->input[1]->dma) < 0)
				return -1;
			break;
		case DDB_PORT_CI:
		case DDB_PORT_LOOP:
			if (dma_alloc(dev->pdev, port->input[0]->dma) < 0)
				return -1;
		case DDB_PORT_MOD:
			if (dma_alloc(dev->pdev, port->output->dma) < 0)
				return -1;
			break;
		default:
			break;
		}
	}
	ddb_set_dma_tables(dev);
	return 0;
}

static void ddb_buffers_free(struct ddb *dev)
{
	int i;
	struct ddb_port *port;
	
	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		
		if (port->input[0])
			dma_free(dev->pdev, port->input[0]->dma);
		if (port->input[1])
			dma_free(dev->pdev, port->input[1]->dma);
		if (port->output)
			dma_free(dev->pdev, port->output->dma);
	}
}

static void ddb_input_start(struct ddb_input *input)
{
	struct ddb *dev = input->port->dev;

	spin_lock_irq(&input->dma->lock);
	input->dma->cbuf = 0;
	input->dma->coff = 0;

	ddbwritel(dev, 0, DMA_BUFFER_CONTROL(input->dma->nr));
	ddbwritel(dev, 0, TS_INPUT_CONTROL(input->nr));
	ddbwritel(dev, 2, TS_INPUT_CONTROL(input->nr));
	ddbwritel(dev, 0, TS_INPUT_CONTROL(input->nr));

	ddbwritel(dev, input->dma->bufreg, DMA_BUFFER_SIZE(input->dma->nr));
	ddbwritel(dev, 0, DMA_BUFFER_ACK(input->dma->nr));

	ddbwritel(dev, 1, DMA_BASE_WRITE);
	ddbwritel(dev, 3, DMA_BUFFER_CONTROL(input->dma->nr));
	ddbwritel(dev, 9, TS_INPUT_CONTROL(input->nr));
	input->dma->running = 1;
	spin_unlock_irq(&input->dma->lock);
	//printk("input_start %d.%d\n", dev->nr, input->nr);
}

static void ddb_input_stop(struct ddb_input *input)
{
	struct ddb *dev = input->port->dev;

	spin_lock_irq(&input->dma->lock);
	ddbwritel(dev, 0, TS_INPUT_CONTROL(input->nr));
	ddbwritel(dev, 0, DMA_BUFFER_CONTROL(input->dma->nr));
	input->dma->running = 0;
	spin_unlock_irq(&input->dma->lock);
	//printk("input_stop %d.%d\n", dev->nr, input->nr);
}

static void ddb_output_start(struct ddb_output *output)
{
	struct ddb *dev = output->port->dev;

	spin_lock_irq(&output->dma->lock);
	output->dma->cbuf = 0;
	output->dma->coff = 0;
	ddbwritel(dev, 0, DMA_BUFFER_CONTROL(output->dma->nr));
	if (output->port->class == DDB_PORT_MOD) {
		ddbwritel(dev, 0, CHANNEL_CONTROL(output->nr));
		ddbwritel(dev, 1, CHANNEL_CONTROL(output->nr));
		ddbwritel(dev, 0, CHANNEL_CONTROL(output->nr));
		ddbwritel(dev, 0x604, CHANNEL_SETTINGS(output->nr));
		ddbwritel(dev, 0x0e, CHANNEL_CONTROL(output->nr));
	} else {
		ddbwritel(dev, 0, TS_OUTPUT_CONTROL(output->nr));
		ddbwritel(dev, 2, TS_OUTPUT_CONTROL(output->nr));
		ddbwritel(dev, 0, TS_OUTPUT_CONTROL(output->nr));
		ddbwritel(dev, 0x3c, TS_OUTPUT_CONTROL(output->nr));
		ddbwritel(dev, output->port->gap, TS_OUTPUT_CONTROL2(output->nr));
	}
	ddbwritel(dev, output->dma->bufreg, DMA_BUFFER_SIZE(output->dma->nr));
	ddbwritel(dev, 0, DMA_BUFFER_ACK(output->dma->nr));
	if (output->port->class == DDB_PORT_MOD)
		ddbwritel(dev, 1, DMA_BASE_READ_MOD);
	else
		ddbwritel(dev, 1, DMA_BASE_READ);
	ddbwritel(dev, 3, DMA_BUFFER_CONTROL(output->dma->nr));
	if (output->port->class != DDB_PORT_MOD) {
		if (output->port->input[0]->port->class == DDB_PORT_LOOP)
			//ddbwritel(dev, 0x15, TS_OUTPUT_CONTROL(output->nr));
			ddbwritel(dev, 0x45, TS_OUTPUT_CONTROL(output->nr));
		else
			ddbwritel(dev, 0x1d, TS_OUTPUT_CONTROL(output->nr));
	}
	output->dma->running = 1;
	spin_unlock_irq(&output->dma->lock);
	printk("output_start %d.%d\n", dev->nr, output->nr);
}

static void ddb_output_stop(struct ddb_output *output)
{
	struct ddb *dev = output->port->dev;

	spin_lock_irq(&output->dma->lock);
	if (output->port->class == DDB_PORT_MOD) 
		ddbwritel(dev, 0, CHANNEL_CONTROL(output->nr));
	else
		ddbwritel(dev, 0, TS_OUTPUT_CONTROL(output->nr));
	ddbwritel(dev, 0, DMA_BUFFER_CONTROL(output->dma->nr));
	output->dma->running = 0;
	spin_unlock_irq(&output->dma->lock);
	mutex_unlock(&redirect_lock);
}

static void ddb_input_start_all(struct ddb_input *input)
{
	struct ddb_input *i = input;
	struct ddb_output *o;
	
	mutex_lock(&redirect_lock);
	while (i && (o = i->redo)) {
		ddb_output_start(o);
		if ((i = o->port->input[0]))
			ddb_input_start(i);
	}
	ddb_input_start(input);
	mutex_unlock(&redirect_lock);
}

static void ddb_input_stop_all(struct ddb_input *input)
{
	struct ddb_input *i = input;
	struct ddb_output *o;
	
	mutex_lock(&redirect_lock);
	ddb_input_stop(input);
	while (i && (o = i->redo)) {
		ddb_output_stop(o);
		if ((i = o->port->input[0]))
			ddb_input_stop(i);
	}
	mutex_unlock(&redirect_lock);
}

static u32 ddb_output_free(struct ddb_output *output)
{
	u32 idx, off, stat = output->dma->stat;
	s32 diff;

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;
 
	if (output->dma->cbuf != idx) {
		if ((((output->dma->cbuf + 1) % output->dma->num) == idx) &&
		    (output->dma->size - output->dma->coff <= 188))
			return 0;
		return 188;
	}
	diff = off - output->dma->coff;
	if (diff <= 0 || diff > 188)
		return 188;
	return 0;
}

static u32 ddb_dma_free(struct ddb_dma *dma)
{
	u32 idx, off, stat = dma->stat;
	s32 p1, p2, diff;

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	p1 = idx * dma->size + off;
	p2 = dma->cbuf * dma->size + dma->coff;

	diff = p1 - p2;
	if (diff <= 0) 
		diff += dma->num * dma->size;
	return diff;
}

static ssize_t ddb_output_write(struct ddb_output *output,
				const u8 *buf, size_t count)
{
	struct ddb *dev = output->port->dev;
	u32 idx, off, stat = output->dma->stat;
	u32 left = count, len;

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	while (left) {
		len = output->dma->size - output->dma->coff;
		if ((((output->dma->cbuf + 1) % output->dma->num) == idx) &&
		    (off == 0)) {
			if (len <= 188)
				break;
			len -= 188;
		}
		if (output->dma->cbuf == idx) {
			if (off > output->dma->coff) {
				len = off - output->dma->coff;
				len -= (len % 188);
				if (len <= 188)
					break;
				len -= 188;
			}
		}
		if (len > left)
			len = left;
		if (copy_from_user(output->dma->vbuf[output->dma->cbuf] +
				   output->dma->coff,
				   buf, len))
			return -EIO;
		left -= len;
		buf += len;
		output->dma->coff += len;
		if (output->dma->coff == output->dma->size) {
			output->dma->coff = 0;
			output->dma->cbuf = ((output->dma->cbuf + 1) %
					     output->dma->num);
		}
		ddbwritel(dev, 
			  (output->dma->cbuf << 11) | (output->dma->coff >> 7),
			  DMA_BUFFER_ACK(output->dma->nr));
	}
	return count - left;
}

static u32 ddb_input_free_bytes(struct ddb_input *input)
{
	struct ddb *dev = input->port->dev;
	u32 idx, off, stat = input->dma->stat;
	u32 ctrl = ddbreadl(dev, DMA_BUFFER_CONTROL(input->dma->nr));

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	if (ctrl & 4)
		return 0;
	if (input->dma->cbuf != idx)
		return 1;
	return 0;
}

static s32 ddb_output_used_bufs(struct ddb_output *output)
{
	u32 idx, off, stat, ctrl;
	s32 diff;

	spin_lock_irq(&output->dma->lock);
	stat = output->dma->stat;
	ctrl = output->dma->ctrl;
	spin_unlock_irq(&output->dma->lock);

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	if (ctrl & 4)
		return 0;
	diff = output->dma->cbuf - idx;
	if (diff == 0 && off < output->dma->coff)
		return 0;
	if (diff <= 0)
		diff += output->dma->num;
	return diff;
}

static s32 ddb_input_free_bufs(struct ddb_input *input)
{
	u32 idx, off, stat, ctrl;
	s32 free;

	spin_lock_irq(&input->dma->lock);
	ctrl = input->dma->ctrl;
	stat = input->dma->stat;
	spin_unlock_irq(&input->dma->lock);
	if (ctrl & 4)
		return 0;
	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;
	free = input->dma->cbuf - idx;
	if (free == 0 && off < input->dma->coff)
		return 0;
	if (free <= 0)
		free += input->dma->num;
	return free - 1;
}

static u32 ddb_output_ok(struct ddb_output *output)
{
	struct ddb_input *input = output->port->input[0];
	s32 diff;

	diff = ddb_input_free_bufs(input) - ddb_output_used_bufs(output);
	if (diff > 0)
		return 1;
	return 0;
}

static u32 ddb_input_avail(struct ddb_input *input)
{
	struct ddb *dev = input->port->dev;
	u32 idx, off, stat = input->dma->stat;
	u32 ctrl = ddbreadl(dev, DMA_BUFFER_CONTROL(input->dma->nr));

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	if (ctrl & 4) {
		printk(KERN_ERR "IA %d %d %08x\n", idx, off, ctrl);
		ddbwritel(dev, stat, DMA_BUFFER_ACK(input->dma->nr));
		return 0;
	}
	if (input->dma->cbuf != idx || off < input->dma->coff)
		return 188;
	return 0;
}

static size_t ddb_input_read(struct ddb_input *input, u8 *buf, size_t count)
{
	struct ddb *dev = input->port->dev;
	u32 left = count;
	u32 idx, off, free, stat = input->dma->stat;
	int ret;

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	while (left) {
		if (input->dma->cbuf == idx)
			return count - left;
		free = input->dma->size - input->dma->coff;
		if (free > left)
			free = left;
		ret = copy_to_user(buf, input->dma->vbuf[input->dma->cbuf] +
				   input->dma->coff, free);
		if (ret)
			return -EFAULT;
		input->dma->coff += free;
		if (input->dma->coff == input->dma->size) {
			input->dma->coff = 0;
			input->dma->cbuf = (input->dma->cbuf + 1) %
				input->dma->num;
		}
		left -= free;
		ddbwritel(dev, 
			  (input->dma->cbuf << 11) | (input->dma->coff >> 7),
			  DMA_BUFFER_ACK(input->dma->nr));
	}
	return count;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#if 0
static struct ddb_input *fe2input(struct ddb *dev, struct dvb_frontend *fe)
{
	int i;

	for (i = 0; i < dev->info->port_num * 2; i++) {
		if (dev->input[i].fe == fe)
			return &dev->input[i];
	}
	return NULL;
}
#endif

static int locked_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct ddb_input *input = fe->sec_priv;
	struct ddb_port *port = input->port;
	struct ddb_dvb *dvb = &port->dvb[input->nr & 1];
	int status;

	if (enable) {
		mutex_lock(&port->i2c_gate_lock);
		status = dvb->gate_ctrl(fe, 1);
	} else {
		status = dvb->gate_ctrl(fe, 0);
		mutex_unlock(&port->i2c_gate_lock);
	}
	return status;
}

static int demod_attach_drxk(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;

	fe = dvb->fe = dvb_attach(drxk_attach,
				  i2c, 0x29 + (input->nr&1),
				  &dvb->fe2);
	if (!fe) {
		printk(KERN_ERR "No DRXK found!\n");
		return -ENODEV;
	}
	fe->sec_priv = input;
	dvb->gate_ctrl = fe->ops.i2c_gate_ctrl;
	fe->ops.i2c_gate_ctrl = locked_gate_ctrl;
	return 0;
}

struct stv0367_config stv0367_0 = {
	.demod_address = 0x1f,
	.xtal = 27000000,
	.if_khz = 5000,
	.if_iq_mode = FE_TER_NORMAL_IF_TUNER,
	.ts_mode = STV0367_SERIAL_PUNCT_CLOCK,
	.clk_pol = STV0367_RISINGEDGE_CLOCK,
};

struct stv0367_config stv0367_1 = {
	.demod_address = 0x1e,
	.xtal = 27000000,
	.if_khz = 5000,
	.if_iq_mode = FE_TER_NORMAL_IF_TUNER,
	.ts_mode = STV0367_SERIAL_PUNCT_CLOCK,
	.clk_pol = STV0367_RISINGEDGE_CLOCK,
};


static int demod_attach_stv0367(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;

	fe = dvb->fe = dvb_attach(stv0367ter_attach,
				  (input->nr & 1) ? &stv0367_1 : &stv0367_0,
				  i2c);
	if (!dvb->fe) {
		printk(KERN_ERR "No stv0367 found!\n");
		return -ENODEV;
	}
	fe->sec_priv = input;
	dvb->gate_ctrl = fe->ops.i2c_gate_ctrl;
	fe->ops.i2c_gate_ctrl = locked_gate_ctrl;
	return 0;
}

struct stv0367_cfg stv0367dd_0 = {
	.adr = 0x1f,
	.xtal = 27000000,
};

struct stv0367_cfg stv0367dd_1 = {
	.adr = 0x1e,
	.xtal = 27000000,
};

static int demod_attach_stv0367dd(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;

	fe = dvb->fe = dvb_attach(stv0367_attach, i2c,
				  (input->nr & 1) ? &stv0367dd_1 : &stv0367dd_0,
				  &dvb->fe2);
	if (!dvb->fe) {
		printk(KERN_ERR "No stv0367 found!\n");
		return -ENODEV;
	}
	fe->sec_priv = input;
	dvb->gate_ctrl = fe->ops.i2c_gate_ctrl;
	fe->ops.i2c_gate_ctrl = locked_gate_ctrl;
	return 0;
}

static int tuner_attach_tda18271(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;

	if (dvb->fe->ops.i2c_gate_ctrl)
		dvb->fe->ops.i2c_gate_ctrl(dvb->fe, 1);
	fe = dvb_attach(tda18271c2dd_attach, dvb->fe, i2c, 0x60);
	if (dvb->fe->ops.i2c_gate_ctrl)
		dvb->fe->ops.i2c_gate_ctrl(dvb->fe, 0);
	if (!fe) {
		printk(KERN_ERR "No TDA18271 found!\n");
		return -ENODEV;
	}
	return 0;
}

static int tuner_attach_tda18212dd(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;

	fe = dvb_attach(tda18212dd_attach, dvb->fe, i2c,
			(input->nr & 1) ? 0x63 : 0x60);
	if (!fe) {
		printk(KERN_ERR "No TDA18212 found!\n");
		return -ENODEV;
	}
	return 0;
}

struct tda18212_config tda18212_0 = {
	.i2c_address = 0x60,
};

struct tda18212_config tda18212_1 = {
	.i2c_address = 0x63,
};

static int tuner_attach_tda18212(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;
	struct tda18212_config *cfg;

	cfg = (input->nr & 1) ? &tda18212_1 : &tda18212_0;
	fe = dvb_attach(tda18212_attach, dvb->fe, i2c, cfg);
	if (!fe) {
		printk(KERN_ERR "No TDA18212 found!\n");
		return -ENODEV;
	}
	return 0;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

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

static struct stv090x_config stv0900_aa = {
	.device         = STV0900,
	.demod_mode     = STV090x_DUAL,
	.clk_mode       = STV090x_CLK_EXT,

	.xtal           = 27000000,
	.address        = 0x68,

	.ts1_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,
	.ts2_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,

	.repeater_level = STV090x_RPTLEVEL_16,

	.adc1_range	= STV090x_ADC_1Vpp,
	.adc2_range	= STV090x_ADC_1Vpp,

	.diseqc_envelope_mode = true,
};

static struct stv6110x_config stv6110a = {
	.addr    = 0x60,
	.refclk	 = 27000000,
	.clk_div = 1,
};

static struct stv6110x_config stv6110b = {
	.addr    = 0x63,
	.refclk	 = 27000000,
	.clk_div = 1,
};

static int demod_attach_stv0900(struct ddb_input *input, int type)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct stv090x_config *feconf = type ? &stv0900_aa : &stv0900;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];

	dvb->fe = dvb_attach(stv090x_attach, feconf, i2c,
			     (input->nr & 1) ? STV090x_DEMODULATOR_1
			     : STV090x_DEMODULATOR_0);
	if (!dvb->fe) {
		printk(KERN_ERR "No STV0900 found!\n");
		return -ENODEV;
	}
	if (!dvb_attach(lnbh24_attach, dvb->fe, i2c, 0,
			0, (input->nr & 1) ?
			(0x09 - type) : (0x0b - type))) {
		printk(KERN_ERR "No LNBH24 found!\n");
		return -ENODEV;
	}
	return 0;
}

static int tuner_attach_stv6110(struct ddb_input *input, int type)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct stv090x_config *feconf = type ? &stv0900_aa : &stv0900;
	struct stv6110x_config *tunerconf = (input->nr & 1) ?
		&stv6110b : &stv6110a;
	struct stv6110x_devctl *ctl;

	ctl = dvb_attach(stv6110x_attach, dvb->fe, tunerconf, i2c);
	if (!ctl) {
		printk(KERN_ERR "No STV6110X found!\n");
		return -ENODEV;
	}
	printk(KERN_INFO "attach tuner input %d adr %02x\n",
	       input->nr, tunerconf->addr);

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

static int my_dvb_dmx_ts_card_init(struct dvb_demux *dvbdemux, char *id,
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

static int my_dvb_dmxdev_ts_card_init(struct dmxdev *dmxdev,
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
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct ddb_input *input = dvbdmx->priv;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];

	if (!dvb->users)
		ddb_input_start_all(input);

	return ++dvb->users;
}

static int stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct ddb_input *input = dvbdmx->priv;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];

	if (--dvb->users)
		return dvb->users;

	ddb_input_stop_all(input);
	return 0;
}


static void dvb_input_detach(struct ddb_input *input)
{
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_demux *dvbdemux = &dvb->demux;

	switch (dvb->attached) {
	case 6:
		if (dvb->fe2)
			dvb_unregister_frontend(dvb->fe2);
		if (dvb->fe)
			dvb_unregister_frontend(dvb->fe);
	case 5:
		dvb_frontend_detach(dvb->fe);
		dvb->fe = NULL;
	case 4:
		dvb_net_release(&dvb->dvbnet);
	case 3:
		dvbdemux->dmx.close(&dvbdemux->dmx);
		dvbdemux->dmx.remove_frontend(&dvbdemux->dmx,
					      &dvb->hw_frontend);
		dvbdemux->dmx.remove_frontend(&dvbdemux->dmx,
					      &dvb->mem_frontend);
		dvb_dmxdev_release(&dvb->dmxdev);
	case 2:
		dvb_dmx_release(&dvb->demux);
	case 1:
		break;
	}
	dvb->attached = 0;
}

static int dvb_register_adapters(struct ddb *dev)
{
	int i, ret = 0;
	struct ddb_port *port;
	struct dvb_adapter *adap;

	if (adapter_alloc == 3 || dev->info->type == DDB_MOD) {
		port = &dev->port[0];
		adap = port->dvb[0].adap;
		ret = dvb_register_adapter(adap, "DDBridge", THIS_MODULE,
					   &port->dev->pdev->dev,
					   adapter_nr);
		if (ret < 0)
			return ret;
		port->dvb[0].adap_registered = 1;
		for (i = 0; i < dev->info->port_num; i++) {
			port = &dev->port[i];
			port->dvb[0].adap = adap;
			port->dvb[1].adap = adap;
		}
		return 0;
	}

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		switch (port->class) {
		case DDB_PORT_TUNER:
			adap = port->dvb[0].adap;
			ret = dvb_register_adapter(adap, "DDBridge", 
						   THIS_MODULE,
						   &port->dev->pdev->dev,
						   adapter_nr);
			if (ret < 0)
				return ret;
			port->dvb[0].adap_registered = 1;

			if (adapter_alloc > 0) {
				port->dvb[1].adap = port->dvb[0].adap;
				break;
			}
			adap = port->dvb[1].adap;
			ret = dvb_register_adapter(adap, "DDBridge", 
						   THIS_MODULE,
						   &port->dev->pdev->dev,
						   adapter_nr);
			if (ret < 0)
				return ret;
			port->dvb[1].adap_registered = 1;
			break;

		case DDB_PORT_CI:
		case DDB_PORT_LOOP:
			adap = port->dvb[0].adap;
			ret = dvb_register_adapter(adap, "DDBridge",
						   THIS_MODULE,
						   &port->dev->pdev->dev,
						   adapter_nr);
			if (ret < 0)
				return ret;
			port->dvb[0].adap_registered = 1;
			break;
		default:
			if (adapter_alloc < 2)
				break;
			adap = port->dvb[0].adap;
			ret = dvb_register_adapter(adap, "DDBridge",
						   THIS_MODULE,
						   &port->dev->pdev->dev,
						   adapter_nr);
			if (ret < 0)
				return ret;
			port->dvb[0].adap_registered = 1;
			break;
		}
	}
	return ret;
}

static void dvb_unregister_adapters(struct ddb *dev)
{
	int i;
	struct ddb_port *port;
	struct ddb_dvb *dvb;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];

		dvb = &port->dvb[0];
		if (dvb->adap_registered)
			dvb_unregister_adapter(dvb->adap);
		dvb->adap_registered = 0;
		
		dvb = &port->dvb[1];
		if (dvb->adap_registered)
			dvb_unregister_adapter(dvb->adap);
		dvb->adap_registered = 0;
	}
}


static int dvb_input_attach(struct ddb_input *input)
{
	int ret = 0;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct ddb_port *port = input->port;
	struct dvb_adapter *adap = dvb->adap;
	struct dvb_demux *dvbdemux = &dvb->demux;

	dvb->attached = 1;

	ret = my_dvb_dmx_ts_card_init(dvbdemux, "SW demux",
				      start_feed,
				      stop_feed, input);
	if (ret < 0)
		return ret;
	dvb->attached = 2;

	ret = my_dvb_dmxdev_ts_card_init(&dvb->dmxdev,
					 &dvb->demux,
					 &dvb->hw_frontend,
					 &dvb->mem_frontend, adap);
	if (ret < 0)
		return ret;
	dvb->attached = 3;

	ret = dvb_net_init(adap, &dvb->dvbnet, dvb->dmxdev.demux);
	if (ret < 0)
		return ret;
	dvb->attached = 4;

	dvb->fe = 0;
	switch (port->type) {
	case DDB_TUNER_DVBS_ST:
		if (demod_attach_stv0900(input, 0) < 0)
			return -ENODEV;
		if (tuner_attach_stv6110(input, 0) < 0)
			return -ENODEV;
		break;
	case DDB_TUNER_DVBS_ST_AA:
		if (demod_attach_stv0900(input, 1) < 0)
			return -ENODEV;
		if (tuner_attach_stv6110(input, 1) < 0)
			return -ENODEV;
		break;
	case DDB_TUNER_DVBCT_TR:
		if (demod_attach_drxk(input) < 0)
			return -ENODEV;
		if (tuner_attach_tda18271(input) < 0)
			return -ENODEV;
		break;
	case DDB_TUNER_DVBCT_ST:
		if (demod_attach_stv0367dd(input) < 0)
			return -ENODEV;
		if (tuner_attach_tda18212dd(input) < 0)
			return -ENODEV;
		break;
	}
	dvb->attached = 5;
	if (dvb->fe) {
		if (dvb_register_frontend(adap, dvb->fe) < 0)
			return -ENODEV;
	}
	if (dvb->fe2) {
		if (dvb_register_frontend(adap, dvb->fe2) < 0)
			return -ENODEV;
		dvb->fe2->tuner_priv = dvb->fe->tuner_priv;
		memcpy(&dvb->fe2->ops.tuner_ops,
		       &dvb->fe->ops.tuner_ops,
		       sizeof(struct dvb_tuner_ops));
	}
	dvb->attached = 6;
	return 0;
}

/****************************************************************************/
/****************************************************************************/

static ssize_t ts_write(struct file *file, const char *buf,
			size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	size_t left = count;
	int stat;

	while (left) {
		if (ddb_output_free(output) < 188) {
			if (file->f_flags & O_NONBLOCK) 
				break;
			if (wait_event_interruptible(
				    output->dma->wq,
				    ddb_output_free(output) >= 188) < 0) 
				break;
		}
		stat = ddb_output_write(output, buf, left);
		if (stat < 0)
			return stat;
		buf += stat;
		left -= stat;
	}
	return (left == count) ? -EAGAIN : (count - left);
}

static ssize_t ts_read(struct file *file, char *buf,
		       size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb_input *input = output->port->input[0];
	int left, read;

	count -= count % 188;
	left = count;
	while (left) {
		if (ddb_input_avail(input) < 188) {
			if (file->f_flags & O_NONBLOCK)
				break;
			if (wait_event_interruptible(
				    input->dma->wq, 
				    ddb_input_avail(input) >= 188) < 0)
				break;
		}
		read = ddb_input_read(input, buf, left);
		left -= read;
		buf += read;
	}
	return (left == count) ? -EAGAIN : (count - left);
}

static unsigned int ts_poll(struct file *file, poll_table *wait)
{
	/*
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb_input *input = output->port->input[0];
	*/
	unsigned int mask = 0;

#if 0
	if (data_avail_to_read)
		mask |= POLLIN | POLLRDNORM;
	if (data_avail_to_write)
		mask |= POLLOUT | POLLWRNORM;

	poll_wait(file, &read_queue, wait);
	poll_wait(file, &write_queue, wait);
#endif
	return mask;
}

static int ts_release(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb_input *input = output->port->input[0];
	
	if ((file->f_flags & O_ACCMODE) == O_RDONLY)
		ddb_input_stop(input);
	else
		ddb_output_stop(output);
	return dvb_generic_release(inode, file);
}

static int ts_open(struct inode *inode, struct file *file)
{
	int err;
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb_input *input = output->port->input[0];
	
	if (input->redo || input->redi)
		return -EBUSY;

	if ((file->f_flags & O_ACCMODE) == O_RDONLY) {
		if (!input)
			return -EINVAL;
	} else {
		if (!output)
			return -EINVAL;
	}

	if ((err = dvb_generic_open(inode, file)) < 0)
		return err;
	
	if ((file->f_flags & O_ACCMODE) == O_RDONLY) 
		ddb_input_start(input);
	else
		ddb_output_start(output);
	return err;
}

static const struct file_operations ci_fops = {
	.owner   = THIS_MODULE,
	.read    = ts_read,
	.write   = ts_write,
	.open    = ts_open,
	.release = ts_release,
	.poll    = ts_poll,
	.mmap    = 0,
};

static struct dvb_device dvbdev_ci = {
	.priv    = 0,
	.readers = 1,
	.writers = 1,
	.users   = 2,
	.fops    = &ci_fops,
};

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void input_write_output(struct ddb_input *input,
			       struct ddb_output *output)
{
	ddbwritel(output->port->dev,
		  input->dma->stat, DMA_BUFFER_ACK(output->dma->nr));
	output->dma->cbuf = (input->dma->stat >> 11) & 0x1f;
	output->dma->coff = (input->dma->stat & 0x7ff) << 7;
}

static void output_ack_input(struct ddb_output *output,
			     struct ddb_input *input)
{
	ddbwritel(input->port->dev,
		  output->dma->stat, DMA_BUFFER_ACK(input->dma->nr));
}

static void input_write_dvb(struct ddb_input *input, 
			    struct ddb_input *input2)
{
	struct ddb_dvb *dvb = &input2->port->dvb[input2->nr & 1];
	struct ddb_dma *dma, *dma2;
	struct ddb *dev = input->port->dev;
	int noack = 0;

	dma = dma2 = input->dma;
	if (input->redo) {
		dma2 = input->redo->dma;
		noack = 1;
	}
	while (dma->cbuf != ((dma->stat >> 11) & 0x1f)
	       || (4 & dma->ctrl)) {
		if (4 & dma->ctrl) {
			printk(KERN_ERR "Overflow dma %d\n", dma->nr);
			if (noack) 
				noack = 0;
		}
		dvb_dmx_swfilter_packets(&dvb->demux,
					 dma2->vbuf[dma->cbuf],
					 dma2->size / 188);
		dma->cbuf = (dma->cbuf + 1) % dma2->num;
		if (!noack)
			ddbwritel(dev, (dma->cbuf << 11),  
				  DMA_BUFFER_ACK(dma->nr));
		dma->stat = ddbreadl(dev, DMA_BUFFER_CURRENT(dma->nr));
		dma->ctrl = ddbreadl(dev, DMA_BUFFER_CONTROL(dma->nr));
	}
}

static void input_tasklet(unsigned long data)
{
	struct ddb_input *input = (struct ddb_input *) data;
	struct ddb_dma *dma = input->dma;
	struct ddb *dev = input->port->dev;

	spin_lock(&dma->lock);
	if (!dma->running) {
		spin_unlock(&dma->lock);
		return;
	}
	dma->stat = ddbreadl(dev, DMA_BUFFER_CURRENT(dma->nr));
	dma->ctrl = ddbreadl(dev, DMA_BUFFER_CONTROL(dma->nr));

	//printk(KERN_ERR "IT %d.%d %08x\n", dev->nr, dma->nr, dma->ctrl);
	if (4 & dma->ctrl) 
		printk(KERN_ERR "Overflow dma %d\n", dma->nr);
	if (input->redi)
		input_write_dvb(input, input->redi);
	if (input->redo)
		input_write_output(input, input->redo);	
	wake_up(&dma->wq);
	spin_unlock(&dma->lock);
}

static void input_handler(unsigned long data)
{
	struct ddb_input *input = (struct ddb_input *) data;
	struct ddb_dma *dma = input->dma;

	if (input->redi)
		tasklet_schedule(&dma->tasklet);
	else 
		input_tasklet(data);
}

static void output_tasklet(unsigned long data)
{
}

static int mod_set_rateinc(struct ddb *dev, u32 chan, u32 inc);

static void handle_rate2(struct ddb_output *output) 
{
	static u32 c=0, opr = 50;
	struct ddb_dma *dma = output->dma;
	s32 last_free;
	s32 total = dma->num * dma->size;
	s32 free = ddb_dma_free(output->dma);
	s32 pr = (free*100)/total;
	s32 d; 
	struct ddb *dev = output->port->dev;
	u32 bi, bo, bdo, bdi;
	static u32 lbo, lbi, to, ti;

#if 0
	opr = (pr + opr) / 2;
	d = (opr - 50);
	d = dev->rate_inc[output->nr] + d;
	mod_set_rateinc(dev, output->nr, d);
#endif
	bo = (dma->stat >> 11) & 0x1f;
	if (output->redi)
		bi = (output->redi->dma->stat >> 11) & 0x1f;
	bdo = (bo - lbo) & 0x1f;
	bdi = (bi - lbi) & 0x1f;
	lbo = bo;
	lbi = bi;
	to += bdo;
	ti += bdi;

	c++;
	if (!(c & 63)) {
#if 0
		printk("%d  %d  %08x\n", free, pr,
			dev->rate_inc[output->nr]);
		if (output->redi)
			printk("count %d  %d\n", dma->count,
			       output->redi->dma->count);
#else

		printk("%d  %d  %d %d %d\n", bo, lbo, bi, lbi, bo-lbo);
		if (to)
			printk("%d  %d %d\n", to, ti, to - ti);

#endif
	}
}

#define RateIncrementScale 0x1000000
#define NIntegrate 100
#define InverseDGain 1
#define InverseIGain 10

static void handle_rate(struct ddb_output *output) 
{
	static s32 i=-10000, c=0, d=0, diffsum=0, diffsumlast=0;
	static target;
	struct ddb_dma *dma = output->dma;
	struct ddb *dev = output->port->dev;
	s32 last_free;
	s32 total = dma->num * dma->size;
	s32 free = ddb_dma_free(output->dma);
	s32 pr = (free*100)/total;

	if (i==-10000)
		target = total / 2;
	if (i > -9950 && i < -NIntegrate) {
		i++;
		if (pr > 98 && !(i&3)) {
			d+=0x1000;
		}
		if (free < target) {
			i = -NIntegrate;
			diffsum = 0;
		}
	} else if (i >= NIntegrate && i < 0) {
		i++;
		diffsum += free - target;
		printk("ds = %d  %d %d\n", diffsum, free, target);
		if (i == 0) {
			printk("ds = %d\n", diffsum);
			target += diffsum / NIntegrate;
			diffsum = 0;
		}
	} else {
		i++;
		diffsum += free - target;
		if (i==NIntegrate) {
			s32 icorr = diffsum / (NIntegrate * InverseIGain);
			s32 dcorr = (diffsum - diffsumlast) / (NIntegrate * InverseDGain);
			
			printk("ds = %d\n", diffsum);
			diffsumlast = diffsum;
			d += icorr + dcorr;
			i = 0;
			diffsum = 0;
		}
	}
	mod_set_rateinc(dev, output->nr, d);

	c++;
	if (!(c & 63)) {
		printk("%08x %d %d %d\n",dev->rate_inc[output->nr], pr, target, i);
	}
}

static void output_handler(unsigned long data)
{
	struct ddb_output *output = (struct ddb_output *) data;
	struct ddb_dma *dma = output->dma;
	struct ddb *dev = output->port->dev;

	spin_lock(&dma->lock);
	if (!dma->running) {
		spin_unlock(&dma->lock);
		return;
	}
	dma->stat = ddbreadl(dev, DMA_BUFFER_CURRENT(dma->nr));
	dma->ctrl = ddbreadl(dev, DMA_BUFFER_CONTROL(dma->nr));
	if (output->redi) 
		output_ack_input(output, output->redi);
	wake_up(&dma->wq);
	spin_unlock(&dma->lock);

	//if (output->port->class == DDB_PORT_MOD)
	//handle_rate(output);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void mod_write_dac_register(struct ddb *dev, u8 Index, u8 Value)
{
	u32 RegValue = 0;
	
	ddbwritel(dev, Value, DAC_BASE + 4);
	ddbwritel(dev, 0x100 | Index, DAC_BASE);
	do {
		RegValue = ddbreadl(dev, DAC_BASE);
	} while ((RegValue & 0x100) != 0 );
}

static void mod_write_dac_register2(struct ddb *dev, u8 Index, u16 Value)
{
	u32 RegValue = 0;
	
	ddbwritel(dev, Value, DAC_BASE + 4);
	ddbwritel(dev, 0x120 | Index, DAC_BASE);
	do {
		RegValue = ddbreadl(dev, DAC_BASE);
	} while ((RegValue & 0x100) != 0 );
}

static int mod_read_dac_register(struct ddb *dev, u8 Index, u8 *pValue)
{
	u32 RegValue = 0;
	
	ddbwritel(dev, 0x180 | Index, DAC_BASE);
	do {
		RegValue = ddbreadl(dev, DAC_BASE);
	} while( (RegValue & 0x100) != 0 );
	
	RegValue = ddbreadl(dev, DAC_BASE + 8);
	*pValue = (u8) RegValue;
	return 0;
}

static void mod_set_up_converter_vco1(struct ddb *dev, u32 Value)
{
	u32 RegValue = 0;

	if( (Value & 0x03) == 0x02 ) {
		/* Extra delay before writing N divider */
		msleep(50);
	}
	
	do {
		RegValue = ddbreadl(dev, VCO1_BASE);
	} while( (RegValue & 1) != 0 ); 
	
	if ((RegValue & 2) == 0) {
		RegValue |= 2;
		ddbwritel(dev, RegValue, VCO1_BASE);
		msleep(10);
	}
	
	ddbwritel(dev, Value, VCO1_BASE + 4);
	ddbwritel(dev, RegValue | 0x01, VCO1_BASE);
}

static void mod_set_up_converter_vco2(struct ddb *dev, u32 Value)
{
	u32 RegValue = 0;

	if( (Value & 0x03) == 0x02 ) {
		/* Extra delay before writing N divider */
		msleep(50);
	}
	
	do {
		RegValue = ddbreadl(dev, VCO2_BASE);
	} while( (RegValue & 1) != 0 ); 
	
	if ((RegValue & 2) == 0) {
		RegValue |= 2;
		ddbwritel(dev, RegValue, VCO2_BASE);
		msleep(10);
	}
	
	ddbwritel(dev, Value, VCO2_BASE + 4);
	ddbwritel(dev, RegValue | 0x01, VCO2_BASE);
}

static void mod_set_down_converter_vco(struct ddb *dev, u32 Value)
{
	u32 RegValue = 0;

	do {
		RegValue = ddbreadl(dev, VCO3_BASE);
	} while( (RegValue & 1) != 0 ); 
	
	if ((RegValue & 2) == 0) {
		RegValue |= 2;
		ddbwritel(dev, RegValue, VCO3_BASE);
		msleep(10);
	}
	ddbwritel(dev, Value, VCO3_BASE + 4);
	ddbwritel(dev, RegValue | 0x01, VCO3_BASE);
}

static int mod_set_attenuator(struct ddb *dev, u32 Value)
{
	if (Value > 31)
		return -EINVAL;
	ddbwritel(dev, Value, RF_ATTENUATOR);
	return 0;
}

static void mod_si598_readreg(struct ddb *dev, u8 index, u8 *val)
{
	ddbwritel(dev, index, CLOCKGEN_INDEX);
	ddbwritel(dev, 1, CLOCKGEN_CONTROL);
	msleep(5);
	*val = ddbreadl(dev, CLOCKGEN_READDATA);
}

static void mod_si598_writereg(struct ddb *dev, u8 index, u8 val)
{
	ddbwritel(dev, index, CLOCKGEN_INDEX);
	ddbwritel(dev, val, CLOCKGEN_WRITEDATA);
	ddbwritel(dev, 3, CLOCKGEN_CONTROL);
	msleep(5);
}

static int mod_set_si598(struct ddb *dev, u32 freq)
{
	int hr;
	u8 Data[7];
	u64 fDCO = 0;
	u64 RFreq = 0;
	u32 fOut = 10000000;
	u64 m_fXtal = 0;
	u32 N = 0;
	u32 HSDiv = 0;

	u32 fxtal;
	u64 MinDiv, MaxDiv, Div;
	u64 RF;

	if (freq < 10000000 || freq > 525000000 )
		return -EINVAL;
	mod_si598_writereg(dev, 137, 0x10);

	if (m_fXtal == 0) {
		mod_si598_writereg(dev, 135, 0x01);
		mod_si598_readreg(dev, 7, &Data[0]);
		mod_si598_readreg(dev, 8, &Data[1]);
		mod_si598_readreg(dev, 9, &Data[2]);
		mod_si598_readreg(dev, 10, &Data[3]);
		mod_si598_readreg(dev, 11, &Data[4]);
		mod_si598_readreg(dev, 12, &Data[5]);
		
		printk(" Data = %02x %02x %02x %02x %02x %02x\n",
		       Data[0],Data[1],Data[2],Data[3],Data[4],Data[5]);
		RFreq = (((u64)Data[1] & 0x3F) << 32) | ((u64)Data[2] << 24) |
			((u64)Data[3] << 16) | ((u64)Data[4] << 8) | ((u64)Data[5]);
		
		HSDiv = ((Data[0] & 0xE0) >> 5) + 4;
		if (HSDiv == 8 || HSDiv == 10 ) 
			return -EINVAL;
		N = (((u32)(Data[0] & 0x1F) << 2) | ((u32)(Data[1] & 0xE0) >> 6)) + 1;
		fDCO = fOut * (u64)(HSDiv * N);  
		m_fXtal = fDCO << 28;
		printk("fxtal %016llx  rfreq %016llx\n", m_fXtal, RFreq);
		
		m_fXtal += RFreq >> 1;
		m_fXtal = div64_u64(m_fXtal, RFreq);
		
		printk(" fOut = %d fXtal = %d fDCO = %d HDIV = %2d, N = %3d \n",
		       (u32) fOut,(u32) m_fXtal,(u32) fDCO,HSDiv,N);
	}
	
	fOut = freq;
	MinDiv = 4850000000ULL; do_div(MinDiv, freq); MinDiv += 1;
	MaxDiv = 5670000000ULL; do_div(MaxDiv, freq);
	Div    = 5260000000ULL; do_div(Div, freq);

	if( Div < MinDiv ) 
		Div = Div + 1;
	//printk(" fOut = %d MinDiv = %4d MaxDiv = %4d StartDiv = %d\n", fOut,MinDiv,MaxDiv,Div);
	
	if( Div <= 11 ) {
		N = 1;
		HSDiv = Div;
	} else {
		int retry = 100;
		while(retry > 0) {
			N = 0;
			HSDiv = Div;
			while( (HSDiv > 11) /*|| ((HSDiv * N) != Div)*/ ) {
				N = N + 2;
				//HSDiv = Div / N;
				HSDiv = Div;
				do_div(HSDiv, N);
				if (N > 128) 
					break;
			}
			//printk(" %3d: %4d %4d %2d %3d\n",retry,Div,HSDiv*N,HSDiv,N);
			if (HSDiv * N < MinDiv) {
				Div = Div + 2;
			} else if (HSDiv * N > MaxDiv) {
				Div = Div - 2;
			} else
				break;
			retry = retry - 1;
		}
		if( retry == 0 ) {
			printk(" FAIL \n");
			return -EINVAL;
		}
	}
	
	if (HSDiv == 8 || HSDiv == 10)	{
		HSDiv = HSDiv/2;
		N = N * 2;
	}
	
	if (HSDiv < 4)
		return -EINVAL;
	

	fDCO = (u64)fOut * (u64)N * (u64)HSDiv;
	printk("fdco %16llx\n", fDCO);
	RFreq = fDCO<<28;
	printk("%16llx %16llx\n", fDCO, RFreq);

	fxtal = m_fXtal; 
	do_div(RFreq, fxtal);
	printk("%16llx %d\n", RFreq, fxtal);
	RF = RFreq;
	
	printk(" fOut = %d fXtal = %d fDCO = %d HDIV = %d, N = %d, RFreq = %d\n",fOut,m_fXtal,fDCO,HSDiv,N,RFreq);
	printk("%16llx\n", RF);

	Data[0] = (u8)( ((HSDiv - 4) << 5) | ((N - 1) >> 2) );
	Data[1] = (u8)( (((N - 1) & 0x03) << 6) | (( RF >> 32 ) & 0x3F ) );
	Data[2] = (u8)( (RF >> 24) & 0xFF );
	Data[3] = (u8)( (RF >> 16) & 0xFF );
	Data[4] = (u8)( (RF >>  8) & 0xFF );
	Data[5] = (u8)( (RF      ) & 0xFF );
	
	printk(" Data = %02x %02x %02x %02x %02x %02x\n",Data[1],Data[2],Data[3],Data[4],Data[5], Data[6]);
	mod_si598_writereg(dev, 7, Data[0]);
	mod_si598_writereg(dev, 8, Data[1]);
	mod_si598_writereg(dev, 9, Data[2]);
	mod_si598_writereg(dev, 10, Data[3]);
	mod_si598_writereg(dev, 11, Data[4]);
	mod_si598_writereg(dev, 12, Data[5]);

	mod_si598_writereg(dev, 137, 0x00);
	mod_si598_writereg(dev, 135, 0x40);
	return hr;
}


static void mod_bypass_equalizer(struct ddb *dev, int bypass)
{
	u32  RegValue;
	
	RegValue = ddbreadl(dev, IQOUTPUT_BASE);
	RegValue &= ~0x10;
	RegValue |= (bypass ? 0x10 : 0x00);
	ddbwritel(dev, RegValue, IQOUTPUT_BASE);
}

static int mod_set_equalizer(struct ddb *dev, u32 Num, s16 *cTable)
{
	u32 i, adr = IQOUTPUT_EQUALIZER_0;
	
	if (Num > 11) 
		return -EINVAL;

	for (i = 0; i < 11 - Num; i += 1) {
		ddbwritel(dev, 0, adr);
		adr += 4;
	}
	for (i = 0; i < Num; i += 1) {
		ddbwritel(dev, (u32) cTable[i], adr);
		adr += 4;
	}
	return 0;
}

static void mod_peak(struct ddb *dev, u32 Time, s16 *pIPeak, s16 *pQPeak)
{
	u32 val;
	
	val = ddbreadl(dev, IQOUTPUT_BASE);
	val &= ~0x0C;
	ddbwritel(dev, val, IQOUTPUT_BASE);
	ddbwritel(dev, val | 4, IQOUTPUT_BASE);
	msleep(10);
	ddbwritel(dev, val, IQOUTPUT_BASE);
	ddbwritel(dev, val | 8, IQOUTPUT_BASE);
	msleep(Time);
	ddbwritel(dev, val, IQOUTPUT_BASE);
	val = ddbreadl(dev, IQOUTPUT_BASE + 8);
	
	*pIPeak = val & 0xffff;
	*pQPeak = (val >> 16) & 0xffff;
}

static int mod_init_dac_input(struct ddb *dev)
{
	u8 Set = 0;
	u8 Hld = 0;
	u8 Sample = 0; 
	
	u8 Seek = 0;
	u8 ReadSeek = 0;
	
	u8 SetTable[32];
	u8 HldTable[32];
	u8 SeekTable[32];

	u8 Sample1 = 0xFF;
	u8 Sample2 = 0xFF;
	
	u8 SelectSample = 0xFF;
	u8 DiffMin = 0xFF;

	for (Sample = 0; Sample < 32; Sample++ ) {
		Set = 0;
		Hld = 0;
		
		mod_write_dac_register(dev, 0x04, Set << 4 | Hld);
		mod_write_dac_register(dev, 0x05, Sample);
		mod_read_dac_register(dev, 0x06, &ReadSeek);
		Seek = ReadSeek & 0x01;
		SeekTable[Sample] = Seek;
		
		HldTable[Sample] = 15;

		for (Hld = 1; Hld < 16; Hld += 1) {
			mod_write_dac_register(dev, 0x04, Set << 4 | Hld);
			mod_read_dac_register(dev, 0x06, &ReadSeek);
			
			if ((ReadSeek & 0x01) != Seek) 
			{
				HldTable[Sample] = Hld;
				break;
			}
		}
		
		Hld = 0;
		SetTable[Sample] = 15;
		for (Set = 1; Set < 16; Set += 1) {
			mod_write_dac_register(dev, 0x04, Set << 4 | Hld);
			mod_read_dac_register(dev, 0x06, &ReadSeek);
			
			if( (ReadSeek & 0x01) != Seek ) {
				SetTable[Sample] = Set;
				break;
			}
		}
	}
	
	Seek = 1;
	for (Sample = 0; Sample < 32; Sample += 1 ) {
		//printk(" %2d: %d %2d %2d\n", Sample,SeekTable[Sample],SetTable[Sample],HldTable[Sample]);
		
		if (Sample1 == 0xFF && SeekTable[Sample] == 1 && Seek == 0 ) 
			Sample1 = Sample;
		if (Sample1 != 0xFF && Sample2 == 0xFF && SeekTable[Sample] == 0 && Seek == 1 )
			Sample2 = Sample;
		Seek = SeekTable[Sample];
	}
	
	if (Sample1 == 0xFF || Sample2 == 0xFF ) {
		printk(" No valid window found\n");
		return -EINVAL;
	}
	
	printk(" Window = %d - %d\n", Sample1, Sample2);

	for (Sample = Sample1; Sample < Sample2; Sample += 1) {
		if (SetTable[Sample] < HldTable[Sample]) {
			if (HldTable[Sample] - SetTable[Sample] < DiffMin) {
				DiffMin = HldTable[Sample] - SetTable[Sample];
				SelectSample = Sample;
			}
		}
	}
	
	printk("Select Sample %d\n", SelectSample);
	
	if (SelectSample == 0xFF) {
		printk("No valid sample found\n");
		return -EINVAL;
	}
	
	if (HldTable[SelectSample] + SetTable[SelectSample] < 8 ) {
		printk("Too high jitter\n");
		return -EINVAL;
	}
	
	mod_write_dac_register(dev, 0x04, 0x00);
	mod_write_dac_register(dev, 0x05, (SelectSample - 1) & 0x1F);
	mod_read_dac_register(dev, 0x06, &Seek);
	mod_write_dac_register(dev, 0x05, (SelectSample + 1) & 0x1F);
	mod_read_dac_register(dev, 0x06,&ReadSeek);
	Seek &= ReadSeek;
	
	mod_write_dac_register(dev, 0x05, SelectSample);
	mod_read_dac_register(dev, 0x06, &ReadSeek);
	Seek &= ReadSeek;
	if( (Seek & 0x01) == 0 ) {
		printk("Insufficient timing margin\n");
		return -EINVAL;
	}
	printk("Done\n");
	return 0;
}

static void mod_set_up1(struct ddb *dev, u32 Frequency, u32 Ref, u32 Ext)
{
	u32 RDiv = Ext / Ref;
	Frequency = Frequency / Ref;
	
	mod_set_up_converter_vco1(dev, 0x360001 | (RDiv << 2)); 
	mod_set_up_converter_vco1(dev, 0x0ff128);
	mod_set_up_converter_vco1(dev, 0x02 | (Frequency << 8));
}

static void mod_set_up2(struct ddb *dev, u32 Frequency, u32 Ref, u32 Ext)
{
	u32 Rdiv = Ext / Ref;
	u32 PreScale = 8;
	Frequency = Frequency / Ref;

	mod_set_up_converter_vco2(dev, 0x360001 | (Rdiv << 2));
	mod_set_up_converter_vco2(dev, 0x0fc128 | (((PreScale - 8) / 8) << 22));
	mod_set_up_converter_vco2(dev, 0x02 | ((Frequency / PreScale) << 8) 
				  | (Frequency & (PreScale - 1)) << 2);
}

static void mod_set_down(struct ddb *dev, u32 Frequency, u32 Ref, u32 Ext)
{
	u32 BandSelect = Ref * 8;
	u32 RefMul = 2;
	u32 RefDiv2 = 1;
	u32 RefDiv = Ext * RefMul / (Ref * RefDiv2);
	
	if (Frequency < 2200 || Frequency > 4000) 
		return -EINVAL; 

	Frequency = Frequency / Ref;

	mod_set_down_converter_vco(dev, 0x0080003C | ((BandSelect & 0xFF) << 12));
	mod_set_down_converter_vco(dev, 0x00000003);
	mod_set_down_converter_vco(dev, 0x18001E42 | ((RefMul-1) << 25) | ((RefDiv2-1) << 24) | (RefDiv << 14) );
	mod_set_down_converter_vco(dev, 0x08008021);
	mod_set_down_converter_vco(dev, Frequency << 15);
}

static int mod_set_dac_clock(struct ddb *dev, u32 Frequency)
{
	int hr, i;

	if (Frequency) {
		ddbwritel(dev, 0x200, DAC_BASE);
		msleep(10);
		if (mod_set_si598(dev, Frequency))
			return -1;
		msleep(50);
		ddbwritel(dev, 0x000, DAC_BASE);
		msleep(10);
		mod_write_dac_register(dev, 0, 0x02);
	}
	
	for (i = 0; i < 10; i++) { 
		hr = mod_init_dac_input(dev);
		if (hr == 0)
			break;
		msleep(100);
	}
	return hr;
}

static void mod_set_dac_current(struct ddb *dev, u32 Current1, u32 Current2)
{
	mod_write_dac_register2(dev, 0x0b, Current1 & 0x3ff);
	mod_write_dac_register2(dev, 0x0f, Current2 & 0x3ff);
}

static void mod_output_enable(struct ddb *dev, int enable)
{

	u32  RegValue;
	
	RegValue = ddbreadl(dev, IQOUTPUT_BASE);
	RegValue &= ~0x03;
	ddbwritel(dev, RegValue, IQOUTPUT_BASE);

	if (enable) {
		ddbwritel(dev, RegValue | 1, IQOUTPUT_BASE);
		msleep(10);
		ddbwritel(dev, RegValue, IQOUTPUT_BASE);
		ddbwritel(dev, RegValue | 2, IQOUTPUT_BASE);
	}
}

static void mod_set_iq(struct ddb *dev, u32 steps, u32 chan, u32 freq)
{
	int i, j, k, fac = 8;
	int s1 = 22, s2 = 33;
	u64 amp = (1 << 17) - 1; 
	u64 s = 0, c = (amp << s1), ss;
	u64 frq = 0xC90FDAA22168C234;
	u32 *iqtab;

	iqtab = kmalloc((steps + 1) * 4, GFP_KERNEL);
	if (!iqtab)
		return -ENOMEM;
	frq = div64_u64(frq, steps * fac) >> (61 - s2);
	for (i = 0; i <= steps * fac / 4; i++) {
		if (!(i & (fac - 1))) {
			j = i / fac;
			ss = s >> s1;
			//ss = ((s >> (s1 - 1)) + 1) >> 1;
					
			iqtab[j] = iqtab[steps / 2 - j] = ss;
			iqtab[steps / 2 + j] = iqtab[steps - j] = -ss;
		}
		c -= ((s * frq) >> s2);
		s += ((c * frq) >> s2);
	}

	ddbwritel(dev, chan & 0x0f, MODULATOR_IQTABLE_INDEX);
	for (i = j = 0, k = steps / 4; i < steps; i++) {
		ddbwritel(dev, iqtab[j], ITABLE_BASE + i * 4);
		ddbwritel(dev, iqtab[k], QTABLE_BASE + i * 4);
		j += freq;
		j %= steps;
		k += freq;
		k %= steps;
	}
	ddbwritel(dev, steps - 1, CHANNEL_IQTABLESIZE(chan));
	kfree(iqtab);
}

u32 eqtab[] = {
	0x0000FFDB, 0x00000121, 0x0000FF0A, 0x000003D7, 
	0x000001C4, 0x000005A5, 0x000009CC, 0x0000F50D, 
	0x00001B23, 0x0000EEB7, 0x00006A28
};

static int mod_set_modulation(struct ddb *dev, int chan, int mod)
{
	static u32 setting[5] = { 0x600, 0x601, 0x602, 0x903, 0x604 };

	if (mod > 4) 
		return -EINVAL;
	ddbwritel(dev, setting[mod] , CHANNEL_SETTINGS(chan));
	return 0;
}

static int mod_set_rateinc(struct ddb *dev, u32 chan, u32 inc)
{
	dev->rate_inc[chan] = inc;
	ddbwritel(dev, inc, CHANNEL_RATE_INCR(chan));
	return 0;
}

static void mod_set_channelsumshift(struct ddb *dev, u32 shift)
{
	ddbwritel(dev, (shift & 3) << 2, MODULATOR_CONTROL);
}

static void mod_pre_eq_gain(struct ddb *dev, u16 gain)
{
	ddbwritel(dev, gain, IQOUTPUT_PRESCALER);
}

static void mod_post_eq_gain(struct ddb *dev, u16 igain, u16 qgain)
{
	ddbwritel(dev, ((u32)qgain << 16) | igain, IQOUTPUT_POSTSCALER);
}


static int flashread(struct ddb *dev, u8 *buf, u32 addr, u32 len);

static int mod_init(struct ddb *dev)
{
	int stat = 0;
	u8 *buffer;
	struct DDMOD_FLASH *flash;
	u32 Ext = 40;
        u32 UP1Frequency = 290;
        u32 UP2Frequency = 1896;
	u32 Frequency = 722000000;
	u32 DownFrequency;
        u32 FrequencyCH10;
	u32 iqfreq, iqsteps, i;

	buffer = kmalloc(4096, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	flash = (struct DDMOD_FLASH *) buffer;
	
	flashread(dev, buffer, DDMOD_FLASH_START, 4096);
	
	if (flash->Magic != DDMOD_FLASH_MAGIC && flash->Magic != 1) {
		stat = -EINVAL;
		goto fail;
	}
	printk("srate = %d\n", flash->DataSet[0].Symbolrate * 1000);

	mod_output_enable(dev, 0);
	mod_set_dac_clock(dev, flash->DataSet[0].DACFrequency * 1000);
	mod_set_dac_current(dev, 512, 512);

	ddbwritel(dev, flash->DataSet[0].Control2, IQOUTPUT_CONTROL2);	

	mod_set_up1(dev, UP1Frequency, 5, Ext);
	mod_set_up2(dev, UP2Frequency, 8, Ext);
	
	Frequency /= 1000000;
        FrequencyCH10 = flash->DataSet[0].FlatStart + 4;
	DownFrequency = Frequency + 9 * 8 + FrequencyCH10 + UP1Frequency + UP2Frequency;
	printk("CH10 = %d, Down = %d\n", FrequencyCH10, DownFrequency);


        if ((FrequencyCH10 + 9 * 8) > (flash->DataSet[0].FlatEnd - 4)) {
		printk("Frequency out of range %d\n", FrequencyCH10);
		stat = -EINVAL;
		goto fail;
        }

        if( DownFrequency % 8 != 0 ) {
		printk(" Invalid Frequency %d\n", DownFrequency);
		stat = -EINVAL;
		goto fail;
        }

	mod_set_down(dev, DownFrequency, 8, Ext);

	for (i = 0; i < 10; i++) {
		ddbwritel(dev, 0, CHANNEL_CONTROL(i));
		
		iqfreq = flash->DataSet[0].FrequencyFactor * (FrequencyCH10 + (9 - i) * 8);
		iqsteps = flash->DataSet[0].IQTableLength;
		mod_set_iq(dev, iqsteps, i, iqfreq);
	}
	
	mod_bypass_equalizer(dev, 1);
	mod_set_equalizer(dev, 11, flash->DataSet[0].EQTap);
	mod_bypass_equalizer(dev, 0);
	mod_post_eq_gain(dev, flash->DataSet[0].PostScaleI, flash->DataSet[0].PostScaleQ);
	//mod_pre_eq_gain(dev, flash->DataSet[0].PreScale);
	mod_pre_eq_gain(dev, 0x0680);
	printk("prescaler %04x\n", flash->DataSet[0].PreScale);
	mod_set_channelsumshift(dev, 2);
	mod_output_enable(dev, 1);

	//mod_set_attenuator(dev, 10);

	mod_set_rateinc(dev, 0, 0);
	//mod_set_rateinc(dev, 0, 0x299e45);
	
fail:
	kfree(buffer);
	return stat;
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static int wait_ci_ready(struct ddb_ci *ci)
{
	u32 count = 10;
	
	ndelay(500);
	do {
		if (ddbreadl(ci->port->dev, 
			     CI_CONTROL(ci->nr)) & CI_READY)
			break;
		usleep_range(1, 2);
		if ((--count) == 0)
			return -1;
	} while (1);
	return 0;
}

static int read_attribute_mem(struct dvb_ca_en50221 *ca,
			      int slot, int address)
{
	struct ddb_ci *ci = ca->data;
	u32 val, off = (address >> 1) & (CI_BUFFER_SIZE-1);

	if (address > CI_BUFFER_SIZE)
		return -1;
	ddbwritel(ci->port->dev, CI_READ_CMD | (1 << 16) | address,
		  CI_DO_READ_ATTRIBUTES(ci->nr));
	wait_ci_ready(ci);
	val = 0xff & ddbreadl(ci->port->dev, CI_BUFFER(ci->nr) + off);
	return val;
}

static int write_attribute_mem(struct dvb_ca_en50221 *ca, int slot,
			       int address, u8 value)
{
	struct ddb_ci *ci = ca->data;

	ddbwritel(ci->port->dev, CI_WRITE_CMD | (value << 16) | address,
		  CI_DO_ATTRIBUTE_RW(ci->nr));
	wait_ci_ready(ci);
	return 0;
}

static int read_cam_control(struct dvb_ca_en50221 *ca,
			    int slot, u8 address)
{
	u32 count = 100;
	struct ddb_ci *ci = ca->data;
	u32 res;

	ddbwritel(ci->port->dev, CI_READ_CMD | address,
		  CI_DO_IO_RW(ci->nr));
	ndelay(500);
	do {
		res = ddbreadl(ci->port->dev, CI_READDATA(ci->nr));
		if (res & CI_READY)
			break;
		usleep_range(1, 2);
		if ((--count) == 0)
			return -1;
	} while (1);
	return (0xff & res);
}

static int write_cam_control(struct dvb_ca_en50221 *ca, int slot,
			     u8 address, u8 value)
{
	struct ddb_ci *ci = ca->data;

	ddbwritel(ci->port->dev, CI_WRITE_CMD | (value << 16) | address,
		  CI_DO_IO_RW(ci->nr));
	wait_ci_ready(ci);
	return 0;
}

static int slot_reset(struct dvb_ca_en50221 *ca, int slot)
{
	struct ddb_ci *ci = ca->data;

	ddbwritel(ci->port->dev, CI_POWER_ON,
		  CI_CONTROL(ci->nr));
	msleep(100);
	ddbwritel(ci->port->dev, CI_POWER_ON | CI_RESET_CAM,
		  CI_CONTROL(ci->nr));
	ddbwritel(ci->port->dev, CI_ENABLE | CI_POWER_ON | CI_RESET_CAM,
		  CI_CONTROL(ci->nr));
	udelay(20);
	ddbwritel(ci->port->dev, CI_ENABLE | CI_POWER_ON,
		  CI_CONTROL(ci->nr));
	return 0;
}

static int slot_shutdown(struct dvb_ca_en50221 *ca, int slot)
{
	struct ddb_ci *ci = ca->data;

	ddbwritel(ci->port->dev, 0, CI_CONTROL(ci->nr));
	msleep(300);
	return 0;
}

static int slot_ts_enable(struct dvb_ca_en50221 *ca, int slot)
{
	struct ddb_ci *ci = ca->data;
	u32 val = ddbreadl(ci->port->dev, CI_CONTROL(ci->nr));

	ddbwritel(ci->port->dev, val | CI_BYPASS_DISABLE,
		  CI_CONTROL(ci->nr));
	return 0;
}

static int poll_slot_status(struct dvb_ca_en50221 *ca, int slot, int open)
{
	struct ddb_ci *ci = ca->data;
	u32 val = ddbreadl(ci->port->dev, CI_CONTROL(ci->nr));
	int stat = 0;
	
	if (val & CI_CAM_DETECT)
		stat |= DVB_CA_EN50221_POLL_CAM_PRESENT;
	if (val & CI_CAM_READY)
		stat |= DVB_CA_EN50221_POLL_CAM_READY;
	return stat;
}

static struct dvb_ca_en50221 en_templ = {
	.read_attribute_mem  = read_attribute_mem,
	.write_attribute_mem = write_attribute_mem,
	.read_cam_control    = read_cam_control,
	.write_cam_control   = write_cam_control,
	.slot_reset          = slot_reset,
	.slot_shutdown       = slot_shutdown,
	.slot_ts_enable      = slot_ts_enable,
	.poll_slot_status    = poll_slot_status,
};

static void ci_attach(struct ddb_port *port)
{
	struct ddb_ci *ci = 0;

	ci = kzalloc(sizeof(*ci), GFP_KERNEL);
	if (!ci)
		return;
	memcpy(&ci->en, &en_templ, sizeof(en_templ));
	ci->en.data = ci;
	port->en = &ci->en;
	ci->port = port;
	ci->nr = port->nr - 2;
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/


struct cxd2099_cfg cxd_cfg = {
	.bitrate =  62000,
	.adr     =  0x40,
	.polarity = 1,
	.clock_mode = 1,
};

static int ddb_ci_attach(struct ddb_port *port)
{
	if (port->type == DDB_CI_EXTERNAL_SONY) {
		port->en = cxd2099_attach(&cxd_cfg, port, &port->i2c->adap);
		if (!port->en) 
			return -ENODEV;
		dvb_ca_en50221_init(port->dvb[0].adap,
				    port->en, 0, 1);
	}
	if (port->type == DDB_CI_INTERNAL) {
		ci_attach(port);
		if (!port->en) 
			return -ENODEV;
		dvb_ca_en50221_init(port->dvb[0].adap, port->en, 0, 1);
	}
	return 0;
}

static int ddb_port_attach(struct ddb_port *port)
{
	int ret = 0;

	switch (port->class) {
	case DDB_PORT_TUNER:
		ret = dvb_input_attach(port->input[0]);
		if (ret < 0)
			break;
		ret = dvb_input_attach(port->input[1]);
		if (ret < 0)
			break;
		port->input[0]->redi = port->input[0];
		port->input[1]->redi = port->input[1];
		break;
	case DDB_PORT_CI:
		ret = ddb_ci_attach(port);
		if (ret < 0)
			break;
	case DDB_PORT_LOOP:
	case DDB_PORT_MOD:
		ret = dvb_register_device(port->dvb[0].adap,
					  &port->dvb[0].dev,
					  &dvbdev_ci, (void *) port->output,
					  DVB_DEVICE_SEC);
		break;
	default:
		break;
	}
	if (ret < 0)
		printk(KERN_ERR "port_attach on port %d failed\n", port->nr);
	return ret;
}

static int ddb_ports_attach(struct ddb *dev)
{
	int i, ret = 0;
	struct ddb_port *port;

	ret = dvb_register_adapters(dev);
	if (ret < 0)
		return ret;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		ret = ddb_port_attach(port);
		if (ret < 0)
			break;
	}
	return ret;
}

static void ddb_ports_detach(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
	
		switch (port->class) {
		case DDB_PORT_TUNER:
			dvb_input_detach(port->input[0]);
			dvb_input_detach(port->input[1]);
			break;
		case DDB_PORT_CI:
		case DDB_PORT_LOOP:
			if (port->dvb[0].dev)
				dvb_unregister_device(port->dvb[0].dev);
			if (port->en) {
				dvb_ca_en50221_release(port->en);
				kfree(port->en);
				port->en = 0;
			}
			break;
		case DDB_PORT_MOD:
			if (port->dvb[0].dev)
				dvb_unregister_device(port->dvb[0].dev);
			break;
		}
	}
	dvb_unregister_adapters(dev);
}

/****************************************************************************/
/****************************************************************************/

static int port_has_cxd(struct ddb_port *port)
{
	u8 val;
	return i2c_read_reg(&port->i2c->adap, 0x40, 0, &val) ? 0 : 1;
}

static int port_has_stv0900(struct ddb_port *port)
{
	u8 val;
	if (i2c_read_reg16(&port->i2c->adap, 0x69, 0xf100, &val) < 0)
		return 0;
	return 1;
}

static int port_has_stv0900_aa(struct ddb_port *port)
{
	u8 val;
	if (i2c_read_reg16(&port->i2c->adap, 0x68, 0xf100, &val) < 0)
		return 0;
	return 1;
}

static int port_has_drxks(struct ddb_port *port)
{
	u8 val;
	if (i2c_read(&port->i2c->adap, 0x29, &val) < 0)
		return 0;
	if (i2c_read(&port->i2c->adap, 0x2a, &val) < 0)
		return 0;
	return 1;
}

static int port_has_stv0367(struct ddb_port *port)
{
	u8 val;

	if (i2c_read_reg16(&port->i2c->adap, 0x1e, 0xf000, &val) < 0)
		return 0;
	if (val != 0x60)
		return 0;
	if (i2c_read_reg16(&port->i2c->adap, 0x1f, 0xf000, &val) < 0)
		return 0;
	if (val != 0x60)
		return 0;
	return 1;
}

static void ddb_port_probe(struct ddb_port *port)
{
	struct ddb *dev = port->dev;
	char *modname = "NO MODULE";

	port->class = DDB_PORT_NONE;

	if (dev->info->type == DDB_MOD) {
		modname = "MOD";
		port->class = DDB_PORT_MOD;
		printk(KERN_INFO "Port %d: MOD\n", port->nr);
		return;
	}

	if (port->nr > 1 && dev->info->type == DDB_OCTOPUS_CI) {
		modname = "CI internal";
		port->class = DDB_PORT_CI;
		port->type = DDB_CI_INTERNAL;
	} else if (port_has_cxd(port)) {
		modname = "CI";
		port->class = DDB_PORT_CI;
		port->type = DDB_CI_EXTERNAL_SONY;
		ddbwritel(dev, I2C_SPEED_400, port->i2c->regs + I2C_TIMING);
	} else if (port_has_stv0900(port)) {
		modname = "DUAL DVB-S2";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBS_ST;
		ddbwritel(dev, I2C_SPEED_100, port->i2c->regs + I2C_TIMING);
	} else if (port_has_stv0900_aa(port)) {
		modname = "DUAL DVB-S2";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBS_ST_AA;
		ddbwritel(dev, I2C_SPEED_100, port->i2c->regs + I2C_TIMING);
	} else if (port_has_drxks(port)) {
		modname = "DUAL DVB-C/T";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBCT_TR;
		ddbwritel(dev, I2C_SPEED_400, port->i2c->regs + I2C_TIMING);
	} else if (port_has_stv0367(port)) {
		modname = "DUAL DVB-C/T";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBCT_ST;
		ddbwritel(dev, I2C_SPEED_100, port->i2c->regs + I2C_TIMING);
	} else if (port->nr == ts_loop) {
		modname = "TS LOOP";
		port->class = DDB_PORT_LOOP;
	}
	printk(KERN_INFO "Port %d (TAB %d): %s\n", port->nr, port->nr + 1, modname);
}

static void ddb_dma_init(struct ddb_dma *dma, int nr, void *io, int out)
{
	unsigned long priv = (unsigned long) io;

	dma->io = io;
	dma->nr = nr;
	spin_lock_init(&dma->lock);
	init_waitqueue_head(&dma->wq);
	if (out) {
		tasklet_init(&dma->tasklet, output_tasklet, priv);
		dma->num = OUTPUT_DMA_BUFS;
		dma->size = OUTPUT_DMA_SIZE;
		dma->div = OUTPUT_DMA_IRQ_DIV;
	} else {
		tasklet_init(&dma->tasklet, input_tasklet, priv);
		dma->num = INPUT_DMA_BUFS;
		dma->size = INPUT_DMA_SIZE;
		dma->div = INPUT_DMA_IRQ_DIV;
	}
}

static void ddb_input_init(struct ddb_port *port, int nr, int pnr, int dma_nr)
{
	struct ddb *dev = port->dev;
	struct ddb_input *input = &dev->input[nr];
	
	dev->handler[dma_nr + 8] = input_handler;
	dev->handler_data[dma_nr + 8] = (unsigned long) input;
	port->input[pnr] = input;
	input->nr = nr;
	input->port = port;
	input->dma = &dev->dma[dma_nr];
	ddb_dma_init(input->dma, dma_nr, (void *) input, 0);
	ddbwritel(dev, 0, TS_INPUT_CONTROL(nr));
	ddbwritel(dev, 2, TS_INPUT_CONTROL(nr));
	ddbwritel(dev, 0, TS_INPUT_CONTROL(nr));
	ddbwritel(dev, 0, DMA_BUFFER_ACK(input->dma->nr));
}

static void ddb_output_init(struct ddb_port *port, int nr, int dma_nr)
{
	struct ddb *dev = port->dev;
	struct ddb_output *output = &dev->output[nr];

	dev->handler[dma_nr + 8] = output_handler;
	dev->handler_data[dma_nr + 8] = (unsigned long) output;
	port->output = output;
	output->nr = nr;
	output->port = port;
	output->dma = &dev->dma[dma_nr];
	ddb_dma_init(output->dma, dma_nr, (void *) output, 1);
	if (output->port->class == DDB_PORT_MOD) {
		//ddbwritel(dev, 0, CHANNEL_CONTROL(output->nr));
	} else {
		ddbwritel(dev, 0, TS_OUTPUT_CONTROL(nr));
		ddbwritel(dev, 2, TS_OUTPUT_CONTROL(nr));
		ddbwritel(dev, 0, TS_OUTPUT_CONTROL(nr));
	}
	ddbwritel(dev, 0, DMA_BUFFER_ACK(output->dma->nr));
}

static void ddb_ports_init(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		port->dev = dev;
		port->nr = i;
		port->i2c = &dev->i2c[i];
		port->gap = 4;
		mutex_init(&port->i2c_gate_lock);
		ddb_port_probe(port);
		port->dvb[0].adap = &dev->adap[2 * i];
		port->dvb[1].adap = &dev->adap[2 * i + 1];

		if ((dev->info->type == DDB_OCTOPUS_CI) || 
		    (dev->info->type == DDB_OCTOPUS)) {
			if (i >= 2 && dev->info->type == DDB_OCTOPUS_CI) {
				ddb_input_init(port, 2 + i, 0, 2 + i);
				ddb_input_init(port, 4 + i, 1, 4 + i);
			} else {
				ddb_input_init(port, 2 * i, 0, 2 * i);
				ddb_input_init(port, 2 * i + 1, 1, 2 * i + 1);
			}
			ddb_output_init(port, i, i + 8);
		} 
		if (dev->info->type == DDB_MOD) {
			ddb_output_init(port, i, i);
		}
	}
}

static void ddb_ports_release(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		if (port->input[0])
			tasklet_kill(&port->input[0]->dma->tasklet);
		if (port->input[1])
			tasklet_kill(&port->input[1]->dma->tasklet);
		if (port->output)
			tasklet_kill(&port->output->dma->tasklet);
	}
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#define IRQ_HANDLE(_nr) if ((s & (1UL << _nr)) && dev->handler[_nr]) \
		dev->handler[_nr](dev->handler_data[_nr]);

static void irq_handle_msg(struct ddb *dev, u32 s)
{
	dev->i2c_irq++;
	IRQ_HANDLE(0);
	IRQ_HANDLE(1);
	IRQ_HANDLE(2);
	IRQ_HANDLE(3);
}

static void irq_handle_io(struct ddb *dev, u32 s)
{
	dev->ts_irq++;
	IRQ_HANDLE(8);
	IRQ_HANDLE(9);
	IRQ_HANDLE(10);
	IRQ_HANDLE(11);
	IRQ_HANDLE(12);
	IRQ_HANDLE(13);
	IRQ_HANDLE(14);
	IRQ_HANDLE(15);
	IRQ_HANDLE(16);
	IRQ_HANDLE(17);
	IRQ_HANDLE(18);
	IRQ_HANDLE(19);
}

static irqreturn_t irq_handler0(int irq, void *dev_id)
{
	struct ddb *dev = (struct ddb *) dev_id;
	u32 s = ddbreadl(dev, INTERRUPT_STATUS);

	do {
		if (s & 0x80000000)
			return IRQ_NONE;
		if (!(s & 0xfff00))
			return IRQ_NONE;
		ddbwritel(dev, s, INTERRUPT_ACK);
		irq_handle_io(dev, s);
	} while ((s = ddbreadl(dev, INTERRUPT_STATUS)));
	
	return IRQ_HANDLED;
}

static irqreturn_t irq_handler1(int irq, void *dev_id)
{
	struct ddb *dev = (struct ddb *) dev_id;
	u32 s = ddbreadl(dev, INTERRUPT_STATUS);

	do {
		if (s & 0x80000000)
			return IRQ_NONE;
		if (!(s & 0x0000f))
			return IRQ_NONE;
		ddbwritel(dev, s, INTERRUPT_ACK);
		irq_handle_msg(dev, s);
	} while ((s = ddbreadl(dev, INTERRUPT_STATUS)));
	
	return IRQ_HANDLED;
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	struct ddb *dev = (struct ddb *) dev_id;
	u32 s = ddbreadl(dev, INTERRUPT_STATUS);

	if (!s)
		return IRQ_NONE;
	do {
		if (s & 0x80000000)
			return IRQ_NONE;
		ddbwritel(dev, s, INTERRUPT_ACK);
		
		if (s & 0x0000000f)
			irq_handle_msg(dev, s);
		if (s & 0x000fff00)
			irq_handle_io(dev, s);
	} while ((s = ddbreadl(dev, INTERRUPT_STATUS)));
	
	return IRQ_HANDLED;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

static int flashio(struct ddb *dev, u8 *wbuf, u32 wlen, u8 *rbuf, u32 rlen)
{
	u32 data, shift;

	if (wlen > 4)
		ddbwritel(dev, 1, SPI_CONTROL);
	while (wlen > 4) {
		/* FIXME: check for big-endian */
		data = swab32(*(u32 *)wbuf);
		wbuf += 4;
		wlen -= 4;
		ddbwritel(dev, data, SPI_DATA);
		while (ddbreadl(dev, SPI_CONTROL) & 0x0004)
			;
	}

	if (rlen)
		ddbwritel(dev, 0x0001 | ((wlen << (8 + 3)) & 0x1f00), SPI_CONTROL);
	else
		ddbwritel(dev, 0x0003 | ((wlen << (8 + 3)) & 0x1f00), SPI_CONTROL);

	data = 0;
	shift = ((4 - wlen) * 8);
	while (wlen) {
		data <<= 8;
		data |= *wbuf;
		wlen--;
		wbuf++;
	}
	if (shift)
		data <<= shift;
	ddbwritel(dev, data, SPI_DATA);
	while (ddbreadl(dev, SPI_CONTROL) & 0x0004)
		;

	if (!rlen) {
		ddbwritel(dev, 0, SPI_CONTROL);
		return 0;
	}
	if (rlen > 4)
		ddbwritel(dev, 1, SPI_CONTROL);

	while (rlen > 4) {
		ddbwritel(dev, 0xffffffff, SPI_DATA);
		while (ddbreadl(dev, SPI_CONTROL) & 0x0004)
			;
		data = ddbreadl(dev, SPI_DATA);
		*(u32 *) rbuf = swab32(data);
		rbuf += 4;
		rlen -= 4;
	}
	ddbwritel(dev, 0x0003 | ((rlen << (8 + 3)) & 0x1F00), SPI_CONTROL);
	ddbwritel(dev, 0xffffffff, SPI_DATA);
	while (ddbreadl(dev, SPI_CONTROL) & 0x0004)
		;

	data = ddbreadl(dev, SPI_DATA);
	ddbwritel(dev, 0, SPI_CONTROL);

	if (rlen < 4)
		data <<= ((4 - rlen) * 8);

	while (rlen > 0) {
		*rbuf = ((data >> 24) & 0xff);
		data <<= 8;
		rbuf++;
		rlen--;
	}
	return 0;
}

static int flashread(struct ddb *dev, u8 *buf, u32 addr, u32 len)
{
	u8 cmd[4] = {0x03, (addr>>16)&0xff, (addr>>8)&0xff, addr&0xff};
	
	return flashio(dev, cmd, 4, buf, len);
}

#define DDB_MAGIC 'd'

struct ddb_flashio {
	__u8 *write_buf;
	__u32 write_len;
	__u8 *read_buf;
	__u32 read_len;
};

struct ddb_gpio {
	__u32 mask;
	__u32 data;
};

struct ddb_id {
	__u16 vendor;
	__u16 device;
	__u16 subvendor;
	__u16 subdevice;
	__u32 hw;
	__u32 regmap;
};

#define IOCTL_DDB_FLASHIO  _IOWR(DDB_MAGIC, 0x00, struct ddb_flashio)
#define IOCTL_DDB_GPIO_IN  _IOWR(DDB_MAGIC, 0x01, struct ddb_gpio)
#define IOCTL_DDB_GPIO_OUT _IOWR(DDB_MAGIC, 0x02, struct ddb_gpio)
#define IOCTL_DDB_ID       _IOR(DDB_MAGIC, 0x03, struct ddb_id)

#define DDB_NAME "ddbridge"

static u32 ddb_num;
static int ddb_major;
static DEFINE_MUTEX(ddb_mutex);

static int ddb_open(struct inode *inode, struct file *file)
{
	struct ddb *dev = ddbs[iminor(inode)];

	file->private_data = dev;
	return 0;
}

static long ddb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ddb *dev = file->private_data;
	void *parg = (void *)arg;
	int res;

	switch (cmd) {
	case IOCTL_DDB_FLASHIO:
	{
		struct ddb_flashio fio;
		u8 *rbuf, *wbuf;

		if (copy_from_user(&fio, parg, sizeof(fio)))
			return -EFAULT;

		if (fio.write_len > 1028 || fio.read_len > 1028)
			return -EINVAL;
		if (fio.write_len + fio.read_len > 1028)
			return -EINVAL;

		wbuf = &dev->iobuf[0];
		rbuf = wbuf + fio.write_len;

		if (copy_from_user(wbuf, fio.write_buf, fio.write_len))
			return -EFAULT;
		res = flashio(dev, wbuf, fio.write_len, rbuf, fio.read_len);
		if (res)
			return res;
		if (copy_to_user(fio.read_buf, rbuf, fio.read_len))
			return -EFAULT;
		break;
	}
	case IOCTL_DDB_GPIO_OUT:
	{
		struct ddb_gpio gpio;
		if (copy_from_user(&gpio, parg, sizeof(gpio))) 
			return -EFAULT;
		ddbwritel(dev, gpio.mask, GPIO_DIRECTION);
		ddbwritel(dev, gpio.data, GPIO_OUTPUT);
		break;
	}
	case IOCTL_DDB_ID:
	{
		struct ddb_id ddbid;
		
		ddbid.vendor = dev->id->vendor;
		ddbid.device = dev->id->device;
		ddbid.subvendor = dev->id->subvendor;
		ddbid.subdevice = dev->id->subdevice;
		ddbid.hw = ddbreadl(dev, 0);
		ddbid.regmap = ddbreadl(dev, 4);
		if (copy_to_user(parg, &ddbid, sizeof(ddbid))) 
			return -EFAULT;
		break;
	}
	default:
		return -ENOTTY;
	}
	return 0;
}

static const struct file_operations ddb_fops = {
	.unlocked_ioctl = ddb_ioctl,
	.open           = ddb_open,
};

static char *ddb_devnode(struct device *device, mode_t *mode)
{
	struct ddb *dev = dev_get_drvdata(device);

	return kasprintf(GFP_KERNEL, "ddbridge/card%d", dev->nr);
}

static ssize_t ports_show(struct device *device, struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", dev->info->port_num);
}

static ssize_t ts_irq_show(struct device *device, struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", dev->ts_irq);
}

static ssize_t i2c_irq_show(struct device *device, struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", dev->i2c_irq);
}

static char *class_name[] = {
	"NONE", "CI", "TUNER", "LOOP"
};

static char *type_name[] = {
	"NONE", "DVBS_ST", "DVBS_ST_AA", "DVBCT_TR", "DVBCT_ST", "INTERNAL", "CXD2099", 
};

static ssize_t fan_show(struct device *device, struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	u32 val;

	val = ddbreadl(dev, GPIO_OUTPUT) & 1;
	return sprintf(buf, "%d\n", val);
}

static ssize_t fan_store(struct device *device, struct device_attribute *d,
			 const char *buf, size_t count)
{
	struct ddb *dev = dev_get_drvdata(device);
	unsigned val;

	if (sscanf(buf, "%u\n", &val) != 1)
		return -EINVAL;
	ddbwritel(dev, 1, GPIO_DIRECTION);
	ddbwritel(dev, val & 1, GPIO_OUTPUT);
	return count;
}

static ssize_t temp_show(struct device *device, struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	struct i2c_adapter *adap;
	int temp, temp2;
	u8 tmp[2];

	if (dev->info->type == DDB_MOD) {
		ddbwritel(dev, 1, TEMPMON_CONTROL);
		msleep(5);
		temp = ddbreadl(dev, TEMPMON_SENSOR1);
		temp2 = ddbreadl(dev, TEMPMON_SENSOR2);
		temp = (temp * 1000) >> 8;
		temp2 = (temp2 * 1000) >> 8;
		return sprintf(buf, "%d %d\n", temp, temp2);
	}
	if (!dev->info->temp_num)
		return sprintf(buf, "no sensor\n");
	adap = &dev->i2c[dev->info->temp_bus].adap;
	if (i2c_read_regs(adap, 0x48, 0, tmp, 2) < 0)
		return sprintf(buf, "read_error\n");
	temp = (tmp[0] << 3) | (tmp[1] >> 5);
	temp *= 125;
	if (dev->info->temp_num == 2) {
		if (i2c_read_regs(adap, 0x49, 0, tmp, 2) < 0)
			return sprintf(buf, "read_error\n");
		temp2 = (tmp[0] << 3) | (tmp[1] >> 5);
		temp2 *= 125;
		return sprintf(buf, "%d %d\n", temp, temp2);
	}
	return sprintf(buf, "%d\n", temp);
}


static ssize_t rinc_store(struct device *device, struct device_attribute *d,
			  const char *buf, size_t count)
{
	struct ddb *dev = dev_get_drvdata(device);
	unsigned int val;
	
	if (sscanf(buf, "%x\n", &val) != 1)
		return -EINVAL;
	mod_set_rateinc(dev, 0, val);
	return count;
}


static ssize_t qam_show(struct device *device, struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	struct i2c_adapter *adap;
	u8 tmp[4];
	s16 i, q;

	adap = &dev->i2c[1].adap;
	if (i2c_read_regs16(adap, 0x1f, 0xf480, tmp, 4) < 0) 
		return sprintf(buf, "read_error\n");
	i = (s16) (((u16) tmp[1]) << 14) | (((u16) tmp[0]) << 6);
	q = (s16) (((u16) tmp[3]) << 14) | (((u16) tmp[2]) << 6);

	return sprintf(buf, "%d %d\n", i, q);
}

static ssize_t buf_show(struct device *device, struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", ddb_dma_free(&dev->dma[0]));
}

static ssize_t mod_show(struct device *device, struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	int num = attr->attr.name[3] - 0x30;

	return sprintf(buf, "%s:%s\n",
		       class_name[dev->port[num].class],
		       type_name[dev->port[num].type]);
}

static ssize_t led_show(struct device *device, struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	int num = attr->attr.name[3] - 0x30;

	return sprintf(buf, "%d\n", dev->leds & (1 << num) ? 1 : 0);
}


static void ddb_set_led(struct ddb *dev, int num, int val)
{
	if (!dev->info->led_num)
		return;
	switch (dev->port[num].class) {
	case DDB_PORT_TUNER:
		switch (dev->port[num].type) {
		case DDB_TUNER_DVBS_ST:
			i2c_write_reg16(&dev->i2c[num].adap,
					0x69, 0xf14c, val ? 2 : 0);
			break;
		case DDB_TUNER_DVBCT_ST:
			i2c_write_reg16(&dev->i2c[num].adap, 
					0x1f, 0xf00e, 0);
			i2c_write_reg16(&dev->i2c[num].adap, 
					0x1f, 0xf00f, val ? 1 : 0);
			break;
		}
		break;
	default:
		break;
	}
}

static ssize_t led_store(struct device *device, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct ddb *dev = dev_get_drvdata(device);
	int num = attr->attr.name[3] - 0x30;
	unsigned val;

	if (sscanf(buf, "%u\n", &val) != 1)
		return -EINVAL;
	if (val)
		dev->leds |= (1 << num);
	else
		dev->leds &= ~(1 << num);
	ddb_set_led(dev, num, val);
	return count;
}

static ssize_t snr_show(struct device *device, struct device_attribute *attr, char *buf) 
{
	struct ddb *dev = dev_get_drvdata(device); 
	char snr[32];
	int num = attr->attr.name[3] - 0x30;
	
	/* serial number at 0x100-0x11f */
	if (i2c_read_regs16(&dev->i2c[num].adap, 0x50, 0x100, snr, 32) < 0)
		if (i2c_read_regs16(&dev->i2c[num].adap, 0x57, 0x100, snr, 32) < 0)
			return sprintf(buf, "NO SNR\n");
	snr[31]=0; /* in case it is not terminated on EEPROM */
	return sprintf(buf, "%s\n", snr); 
}


static ssize_t snr_store(struct device *device, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct ddb *dev = dev_get_drvdata(device); 
	int num = attr->attr.name[3] - 0x30;
	u8 snr[34] = { 0x01, 0x00 };
	
	if (count > 31)
		return -EINVAL;
	memcpy(snr + 2, buf, count);
	i2c_write(&dev->i2c[num].adap, 0x57, snr, 34);
	i2c_write(&dev->i2c[num].adap, 0x50, snr, 34);
	return count;
}

static ssize_t bsnr_show(struct device *device, struct device_attribute *attr, char *buf) 
{
	struct ddb *dev = dev_get_drvdata(device); 
	char snr[16];

	flashread(dev, snr, 0x10, 15);
	snr[15]=0; /* in case it is not terminated on EEPROM */
	return sprintf(buf, "%s\n", snr); 
}

static ssize_t redirect_show(struct device *device, struct device_attribute *attr, char *buf) 
{
	return 0;
}

static ssize_t redirect_store(struct device *device, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int i, p;
	int res;
	
	if (sscanf(buf, "%x %x\n", &i, &p) != 2)
		return -EINVAL;
	printk(KERN_INFO "redirect: %02x, %02x\n", i, p);
	res = ddb_redirect(i, p);
	if (res < 0)
		return res;
	return count;
}

static ssize_t gap_show(struct device *device, struct device_attribute *attr, char *buf) 
{
	struct ddb *dev = dev_get_drvdata(device); 
	int num = attr->attr.name[3] - 0x30;
	
	return sprintf(buf, "%d\n", dev->port[num].gap);
}

static ssize_t gap_store(struct device *device, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct ddb *dev = dev_get_drvdata(device); 
	int num = attr->attr.name[3] - 0x30;
	unsigned int val;
	
	if (sscanf(buf, "%u\n", &val) != 1)
		return -EINVAL;
	if (val > 20)
		return -EINVAL;
	dev->port[num].gap = val;
	return count;
}

static ssize_t version_show(struct device *device, struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	
	return sprintf(buf, "%08x %08x\n", ddbreadl(dev, 0), ddbreadl(dev, 4));
}

static ssize_t hwid_show(struct device *device,
			 struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);

	return sprintf(buf, "0x%08X\n", dev->hwid);
}

static ssize_t regmap_show(struct device *device, 
			   struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	
	return sprintf(buf, "0x%08X\n", dev->regmap);
}

#define __ATTR_MRO(_name, _show) {				\
	.attr	= { .name = __stringify(_name), .mode = 0444 },	\
	.show	= _show,					\
}

#define __ATTR_MWO(_name, _store) {				\
	.attr	= { .name = __stringify(_name), .mode = 0222 },	\
	.store	= _store,					\
}

struct device_attribute ddb_attrs[] = {
	__ATTR_RO(version),
	__ATTR_RO(ports),
	__ATTR_RO(ts_irq),
	__ATTR_RO(i2c_irq),
	__ATTR_RO(temp),
	__ATTR_RO(qam),
	__ATTR_RO(buf),
	__ATTR_RO(hwid),
	__ATTR_RO(regmap),
	__ATTR_MWO(rinc, rinc_store),
	__ATTR_MRO(mod0, mod_show),
	__ATTR_MRO(mod1, mod_show),
	__ATTR_MRO(mod2, mod_show),
	__ATTR_MRO(mod3, mod_show),
	__ATTR(fan, 0666, fan_show, fan_store),
	__ATTR(led0, 0666, led_show, led_store),
	__ATTR(led1, 0666, led_show, led_store),
	__ATTR(led2, 0666, led_show, led_store),
	__ATTR(led3, 0666, led_show, led_store),
	__ATTR(snr0, 0666, snr_show, snr_store),
	__ATTR(snr1, 0666, snr_show, snr_store),
	__ATTR(snr2, 0666, snr_show, snr_store),
	__ATTR(snr3, 0666, snr_show, snr_store),
	__ATTR_MRO(snr,  bsnr_show),
	__ATTR(redirect, 0666, redirect_show, redirect_store),
	__ATTR(gap0, 0666, gap_show, gap_store),
	__ATTR(gap1, 0666, gap_show, gap_store),
	__ATTR(gap2, 0666, gap_show, gap_store),
	__ATTR(gap3, 0666, gap_show, gap_store),
	__ATTR_NULL
};

static struct class ddb_class = {
	.name		= "ddbridge",
	.owner          = THIS_MODULE,
	.dev_attrs	= ddb_attrs,
	.devnode        = ddb_devnode,
};

static int ddb_class_create(void)
{
	ddb_major = register_chrdev(0, DDB_NAME, &ddb_fops);
	if (ddb_major < 0)
		return ddb_major;
	if (class_register(&ddb_class) < 0)
		return -1;
	return 0;
}

static void ddb_class_destroy(void)
{
	class_unregister(&ddb_class);
	unregister_chrdev(ddb_major, DDB_NAME);
}

static int ddb_device_create(struct ddb *dev)
{
	mutex_lock(&ddb_mutex);
	dev->nr = ddb_num++;
	ddbs[dev->nr] = dev;
	mutex_unlock(&ddb_mutex);
	dev->ddb_dev = device_create(&ddb_class, &dev->pdev->dev,
				     MKDEV(ddb_major, dev->nr),
				     dev, "ddbridge%d", dev->nr);
	if (IS_ERR(dev->ddb_dev))
		return -1;
	return 0;
}

static void ddb_device_destroy(struct ddb *dev)
{
	if (IS_ERR(dev->ddb_dev))
		return;
	device_destroy(&ddb_class, MKDEV(ddb_major, dev->nr));
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
	struct ddb *dev = (struct ddb *) pci_get_drvdata(pdev);

	ddb_ports_detach(dev);
	ddb_i2c_release(dev);

	ddbwritel(dev, 0, INTERRUPT_ENABLE);
	if (dev->msi == 2)
		free_irq(dev->pdev->irq + 1, dev);
	free_irq(dev->pdev->irq, dev);
#ifdef CONFIG_PCI_MSI
	if (dev->msi)
		pci_disable_msi(dev->pdev);
#endif
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
	int stat = 0;
	int irq_flag = IRQF_SHARED;

	if (pci_enable_device(pdev) < 0)
		return -ENODEV;

	dev = vzalloc(sizeof(struct ddb));
	if (dev == NULL)
		return -ENOMEM;

	dev->pdev = pdev;
	pci_set_drvdata(pdev, dev);
	dev->id = id;
	dev->info = (struct ddb_info *) id->driver_data;
	printk(KERN_INFO "DDBridge driver detected: %s\n", dev->info->name);

	dev->regs = ioremap(pci_resource_start(dev->pdev, 0),
			    pci_resource_len(dev->pdev, 0));
	if (!dev->regs) {
		stat = -ENOMEM;
		goto fail;
	}
	if (ddbreadl(dev, 0) == 0xffffffff) {
		stat = -ENODEV;
		goto fail;
	}

	dev->hwid = ddbreadl(dev, 0);
	dev->regmap = ddbreadl(dev, 4);
	
	printk(KERN_INFO "HW %08x REGMAP %08x\n",
	       dev->hwid, dev->regmap);

#ifdef CONFIG_PCI_MSI
	if (pci_msi_enabled()) {
		stat = pci_enable_msi_block(dev->pdev, 2);
		if (stat == 0) {
			dev->msi = 1;
			printk("DDBrige using 2 MSI interrupts\n");
		}
		if (stat == 1) 
			stat = pci_enable_msi(dev->pdev);
		if (stat < 0) {
			printk(KERN_INFO ": MSI not available.\n");
		} else {
			irq_flag = 0;
			dev->msi++;
		}
	}
#endif
	if (dev->msi == 2) {
		stat = request_irq(dev->pdev->irq, irq_handler0,
				   0, "DDBridge", (void *) dev);
		if (stat < 0)
			goto fail0;
		stat = request_irq(dev->pdev->irq + 1, irq_handler1,
				   0, "DDBridge", (void *) dev);
		if (stat < 0) {
			free_irq(dev->pdev->irq, dev);
			goto fail0;
		} 
	} else {
		stat = request_irq(dev->pdev->irq, irq_handler,
				   irq_flag, "DDBridge", (void *) dev);
		if (stat < 0)
			goto fail0;
	}
	if (dev->info->type == DDB_MOD) {
		ddbwritel(dev, 0, DMA_BASE_READ_MOD);
	} else {
		ddbwritel(dev, 0, DMA_BASE_READ);
		ddbwritel(dev, 0, DMA_BASE_WRITE);
	}
	ddbwritel(dev, 0xffffffff, INTERRUPT_ACK);

	if (dev->msi == 2) {
		ddbwritel(dev, 0xfff00, INTERRUPT_ENABLE);
		ddbwritel(dev, 0x0000f, MSI1_ENABLE);
	} else {
		ddbwritel(dev, 0xfff0f, INTERRUPT_ENABLE);
		ddbwritel(dev, 0x00000, MSI1_ENABLE);
	}
	if (ddb_i2c_init(dev) < 0)
		goto fail1;
	ddb_ports_init(dev);
	if (ddb_buffers_alloc(dev) < 0) {
		printk(KERN_INFO ": Could not allocate buffer memory\n");
		goto fail2;
	}
	if (ddb_ports_attach(dev) < 0)
		goto fail3;

	ddb_device_create(dev);
	if (dev->info->fan_num)	{
		ddbwritel(dev, 1, GPIO_DIRECTION);
		ddbwritel(dev, 1, GPIO_OUTPUT);
	}

	if (dev->info->type == DDB_MOD)
		mod_init(dev);

	return 0;

fail3:
	ddb_ports_detach(dev);
	printk(KERN_ERR "fail3\n");
	ddb_ports_release(dev);
fail2:
	printk(KERN_ERR "fail2\n");
	ddb_buffers_free(dev);
	ddb_i2c_release(dev);
fail1:
	printk(KERN_ERR "fail1\n");
	free_irq(dev->pdev->irq, dev);
	if (dev->msi == 2) 
		free_irq(dev->pdev->irq + 1, dev);
fail0:
	if (dev->msi)
		pci_disable_msi(dev->pdev);
fail:
	printk(KERN_ERR "fail\n");
	ddb_unmap(dev);
	pci_set_drvdata(pdev, 0);
	pci_disable_device(pdev);
	return -1;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

static struct ddb_info ddb_none = {
	.type     = DDB_NONE,
	.name     = "Digital Devices PCIe bridge",
};

static struct ddb_info ddb_octopus = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus DVB adapter",
	.port_num = 4,
	.i2c_num  = 4,
};

static struct ddb_info ddb_octopus_le = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus LE DVB adapter",
	.port_num = 2,
	.i2c_num  = 2,
};

static struct ddb_info ddb_octopus_oem = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus OEM",
	.port_num = 4,
	.i2c_num  = 4,
	.led_num  = 1,
	.fan_num  = 1,
	.temp_num = 1,
	.temp_bus = 0,
};

static struct ddb_info ddb_octopus_mini = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus Mini",
	.port_num = 4,
	.i2c_num  = 4,
};

static struct ddb_info ddb_v6 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V6 DVB adapter",
	.port_num = 3,
	.i2c_num  = 3,
};

static struct ddb_info ddb_satixS2v3 = {
	.type     = DDB_OCTOPUS,
	.name     = "Mystique SaTiX-S2 V3 DVB adapter",
	.port_num = 3,
	.i2c_num  = 3,
};

static struct ddb_info ddb_ci = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI",
	.port_num = 4,
	.i2c_num  = 2,
};

static struct ddb_info ddb_cis = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI single",
	.port_num = 3,
	.i2c_num  = 2,
};

static struct ddb_info ddb_dvbct = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices DVBCT V6.1 DVB adapter",
	.port_num = 3,
	.i2c_num  = 3,
};

static struct ddb_info ddb_mod = {
	.type     = DDB_MOD,
	.name     = "Digital Devices DVB-C modulator",
	.port_num = 10,
};


#define DDVID 0xdd01 /* Digital Devices Vendor ID */

#define DDB_ID(_vend, _dev, _subvend, _subdev, _driverdata) { \
	.vendor      = _vend,    .device    = _dev, \
	.subvendor   = _subvend, .subdevice = _subdev, \
	.driver_data = (unsigned long)&_driverdata }

static const struct pci_device_id ddb_id_tbl[] __devinitdata = {
	DDB_ID(DDVID, 0x0002, DDVID, 0x0001, ddb_octopus),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0001, ddb_octopus),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0002, ddb_octopus_le),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0003, ddb_octopus_oem),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0010, ddb_octopus_mini),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0020, ddb_v6),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0030, ddb_dvbct),
	DDB_ID(DDVID, 0x0003, DDVID, 0xdb03, ddb_satixS2v3),
	DDB_ID(DDVID, 0x0011, DDVID, 0x0040, ddb_ci),
	DDB_ID(DDVID, 0x0011, DDVID, 0x0041, ddb_cis),
	DDB_ID(DDVID, 0x0201, DDVID, 0x0001, ddb_mod),
	/* in case sub-ids got deleted in flash */
	DDB_ID(DDVID, 0x0003, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0011, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0201, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	{0}
};
MODULE_DEVICE_TABLE(pci, ddb_id_tbl);


static struct pci_driver ddb_pci_driver = {
	.name        = "DDBridge",
	.id_table    = ddb_id_tbl,
	.probe       = ddb_probe,
	.remove      = ddb_remove,
};

static __init int module_init_ddbridge(void)
{
	int stat;

	printk(KERN_INFO "Digital Devices PCIE bridge driver, "
	       "Copyright (C) 2010-12 Digital Devices GmbH\n");
	if (ddb_class_create())
		return -1;
	stat = pci_register_driver(&ddb_pci_driver);
	if (stat < 0)
		ddb_class_destroy();
	return stat;
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
MODULE_VERSION("0.9");
