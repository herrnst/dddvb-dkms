/* (C) 2010  Digital Devices UG  */

#include <linux/version.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <asm/io.h>

#include "cxd2099.h"

#define MAX_BUFFER_SIZE 248

struct cxd {
	struct dvb_ca_en50221 en;

	struct i2c_adapter *i2c;
	u8     adr;
	u8     regs[0x23];
	u8     lastaddress;
	u8     clk_reg_f;
	u8     clk_reg_b;
	int    mode;
	u32    bitrate;
	int    ready;
	int    dr;
	int    slot_stat;

	u8     amem[1024];
	int    amem_read;
	
	int    cammode;
	struct mutex lock;
};

static int i2c_write_reg(struct i2c_adapter *adapter, u8 adr, 
			 u8 reg, u8 data)
{
        u8 m[2]={reg, data};
        struct i2c_msg msg={.addr=adr, .flags=0, .buf=m, .len=2};
	
        if (i2c_transfer(adapter, &msg, 1)!=1) {
		printk("Failed to write to I2C register %02x@%02x!\n",
		       reg, adr);
		return -1;
	}
        return 0;
}

static int i2c_write(struct i2c_adapter *adapter, u8 adr, 
		     u8 *data, u8 len)
{
        struct i2c_msg msg={.addr=adr, .flags=0, .buf=data, .len=len};
	
        if (i2c_transfer(adapter, &msg, 1)!=1) {
		printk("Failed to write to I2C!\n");
		return -1;
	}
        return 0;
}

static int i2c_read_reg(struct i2c_adapter *adapter, u8 adr, 
			u8 reg, u8 *val)
{
        struct i2c_msg msgs[2]={{.addr=adr, .flags=0, 
				 .buf=&reg, .len=1 },
				{.addr=adr, .flags=I2C_M_RD, 
				 .buf=val, .len=1 }};
	
        if (i2c_transfer(adapter, msgs, 2)!=2) {
		printk("error in i2c_read_reg\n");
		return -1;
	}
	return 0;
}

static int i2c_read(struct i2c_adapter *adapter, u8 adr, 
		    u8 reg, u8 *data, u8 n)
{
        struct i2c_msg msgs[2]={{.addr=adr, .flags=0, 
				 .buf=&reg, .len=1 },
				{.addr=adr, .flags=I2C_M_RD, 
				 .buf=data, .len=n }};
	
        if (i2c_transfer(adapter, msgs, 2)!=2) {
		printk("error in i2c_read_reg\n");
		return -1;
	}
	return 0;
}

static int read_block(struct cxd *ci, u8 adr, u8 *data, u8 n)
{
	int status;

	status=i2c_write_reg(ci->i2c, ci->adr, 0, adr);
	if (!status) {
		ci->lastaddress=adr;
		status=i2c_read(ci->i2c, ci->adr, 1, data, n);
	}
	return status;
}

static int read_reg(struct cxd *ci, u8 reg, u8 *val)
{
	return read_block(ci, reg, val, 1);
}


static int read_pccard(struct cxd *ci, u16 address, u8 *data, u8 n)
{
	int status;
	u8 addr[3]={ 2, address&0xff, address>>8 };

	status=i2c_write(ci->i2c, ci->adr, addr, 3);  
	if (!status) 
		status=i2c_read(ci->i2c, ci->adr, 3, data, n);  
	return status;
}

static int write_pccard(struct cxd *ci, u16 address, u8 *data, u8 n)
{
	int status;
	u8 addr[3]={ 2, address&0xff, address>>8 };

	status=i2c_write(ci->i2c, ci->adr, addr, 3);  
	if (!status) {
		u8 buf[256]={3};
		memcpy(buf+1, data, n);
		status=i2c_write(ci->i2c, ci->adr, buf, n+1);  
	}
	return status;
}

static int read_io(struct cxd *ci, u16 address, u8 *val)
{
	int status;
	u8 addr[3]={ 2, address&0xff, address>>8 };

	status=i2c_write(ci->i2c, ci->adr, addr, 3);  
	if (!status) 
		status=i2c_read(ci->i2c, ci->adr, 3, val, 1);  
	return status;
}

static int write_io(struct cxd *ci, u16 address, u8 val)
{
	int status;
	u8 addr[3]={ 2, address&0xff, address>>8 };
	u8 buf[2]={ 3, val};

	status=i2c_write(ci->i2c, ci->adr, addr, 3);  
	if (!status) {
		status=i2c_write(ci->i2c, ci->adr, buf, 2);  
	}
	return status;
}

static int read_io_data(struct cxd *ci, u8 *data, u8 n)
{
	int status;
	u8 addr[3]={ 2, 0, 0 };

	status=i2c_write(ci->i2c, ci->adr, addr, 3);  
	if (!status) 
		status=i2c_read(ci->i2c, ci->adr, 3, data, n);  
	return 0;
}

static int write_io_data(struct cxd *ci, u8 *data, u8 n)
{
	int status;
	u8 addr[3]={ 2, 0, 0 };

	status=i2c_write(ci->i2c, ci->adr, addr, 3);  
	if (!status) {
		u8 buf[256]={3};
		memcpy(buf+1, data, n);
		status=i2c_write(ci->i2c, ci->adr, buf, n+1);  
	}
	return 0;
}

static int write_regm(struct cxd *ci, u8 reg, u8 val, u8 mask)
{
	int status;

	status=i2c_write_reg(ci->i2c, ci->adr, 0, reg);
	if (!status && reg>=6 && reg <=8 && mask!=0xff) 
		status=i2c_read_reg(ci->i2c, ci->adr, 1, &ci->regs[reg]);
	ci->regs[reg]=(ci->regs[reg]&(~mask))|val;
	if (!status) {
		ci->lastaddress=reg;
		status=i2c_write_reg(ci->i2c, ci->adr, 1, ci->regs[reg]);
	}
	if (reg==0x20)
		ci->regs[reg]&=0x7f;
	return status;
}

static int write_reg(struct cxd *ci, u8 reg, u8 val)
{
	return write_regm(ci, reg, val, 0xff);
}

static int write_block(struct cxd *ci, u8 adr, u8 *data, int n)
{
	int status;
	u8 buf[256]={1};

	status=i2c_write_reg(ci->i2c, ci->adr, 0, adr);
	if (!status) {
		ci->lastaddress=adr;
		memcpy(buf+1, data, n);
		status=i2c_write(ci->i2c, ci->adr, buf, n+1);
	}
	return status;
}

static void set_mode(struct cxd *ci, int mode)
{
	if (mode==ci->mode)
		return;

	switch (mode) {
	case 0x00: //IO mem
		write_regm(ci, 0x06,0x00,0x07);
		break;
	case 0x01: //ATT mem
		write_regm(ci, 0x06,0x02,0x07);
		break;
	default:
		break;
	}
	//printk("set_mode %d\n", mode);
	ci->mode=mode;
}

static void cam_mode(struct cxd *ci, int mode)
{
	if (mode==ci->cammode)
		return;

	switch (mode) {
	case 0x00: 
		write_regm(ci, 0x20, 0x80, 0x80);
		break;
	case 0x01:
#ifdef BUFFER_MODE
		if (!ci->en.read_data)
			return;
		//printk("enable cam buffer mode\n");
		//write_reg(ci, 0x0d, 0x00);
		//write_reg(ci, 0x0e, 0x01);
		write_regm(ci, 0x08, 0x40, 0x40);
		//read_reg(ci, 0x12, &dummy);
		write_regm(ci, 0x08, 0x80, 0x80);
#endif
		break;
	default:
		break;
	}
	ci->cammode=mode;
}



#define CHK_ERROR(s) if( (status = s) ) break

static int init(struct cxd *ci)
{
	int status;
	
	mutex_lock(&ci->lock);
	ci->mode=-1;
	do {
		CHK_ERROR(write_reg(ci, 0x00,0x00));
		CHK_ERROR(write_reg(ci, 0x01,0x00));
		CHK_ERROR(write_reg(ci, 0x02,0x10));
		CHK_ERROR(write_reg(ci, 0x03,0x00));
		CHK_ERROR(write_reg(ci, 0x05,0xFF));
		CHK_ERROR(write_reg(ci, 0x06,0x1F)); // CAM power off SLot A
		CHK_ERROR(write_reg(ci, 0x07,0x1F)); // CAM power off SLot B (just in case)
		CHK_ERROR(write_reg(ci, 0x08,0x28));
		CHK_ERROR(write_reg(ci, 0x14,0x20));  // PCMCIA Timeout = 1.2 msec
		
		// TS Interface -> Internal loopback
		
		//CHK_ERROR(write_reg(ci, 0x09,0x4D)); // Input Mode C, BYPass Serial, TIVAL = low, MSB
		CHK_ERROR(write_reg(ci, 0x09,0x47)); // Input Mode C, BYPass Serial, TIVAL = low, MSB
		CHK_ERROR(write_reg(ci, 0x0A,0xA7)); // TOSTRT = 8, Mode B (gated clock), falling Edge, Serial, POL=HIGH, MSB
		
		// Sync detector
		CHK_ERROR(write_reg(ci, 0x0B,0x33)); 
		CHK_ERROR(write_reg(ci, 0x0C,0x33)); 
		
		CHK_ERROR(write_regm(ci, 0x14,0x00,0x0F)); 
		CHK_ERROR(write_reg(ci, 0x15, ci->clk_reg_b)); 
		CHK_ERROR(write_regm(ci, 0x16, 0x00,0x0F)); 
		CHK_ERROR(write_reg(ci, 0x17,ci->clk_reg_f)); 
		
		CHK_ERROR(write_reg(ci, 0x20,0x28)); // Integer Divider, Falling Edge, Internal Sync,
		CHK_ERROR(write_reg(ci, 0x21,0x00)); // MCLKI = TICLK/8
		CHK_ERROR(write_reg(ci, 0x22,0x07)); // MCLKI = TICLK/8
		
		
		CHK_ERROR(write_regm(ci, 0x20,0x80,0x80)); // Reset CAM state machine
		
		CHK_ERROR(write_regm(ci, 0x03,0x02,02));  // Enable IREQA Interrupt
		CHK_ERROR(write_reg(ci, 0x01,0x04));  // Enable CD Interrupt
		CHK_ERROR(write_reg(ci, 0x00,0x31));  // Enable TS1,Hot Swap,Slot A
		CHK_ERROR(write_regm(ci, 0x09,0x08,0x08));  // Put TS in bypass
		ci->cammode=-1;
		cam_mode(ci, 0);
	} while(0);
	mutex_unlock(&ci->lock);
	
	return 0;
}

static int shutdown(struct cxd *ci)
{
	int status;
	
	do {
		CHK_ERROR(write_reg(ci, 0x00,0x00)); // Disable TS, CD, Hot Swap, Slot A
		CHK_ERROR(write_reg(ci, 0x01,0x00)); // Hot Swap int
		CHK_ERROR(write_reg(ci, 0x03,0x00));
		CHK_ERROR(write_reg(ci, 0x05,0xFF));
		CHK_ERROR(write_reg(ci, 0x06,0x1F)); // CAM power off SLot A
		CHK_ERROR(write_reg(ci, 0x07,0x1F)); // CAM power off SLot B (just in case)
	} while(0);
	
	return 0;
}


static int read_attribute_mem(struct dvb_ca_en50221 *ca, 
			      int slot, int address)
{
	struct cxd *ci = ca->data;
#if 0
	if (ci->amem_read) {
		if (address <=0 || address>1024)
			return -EIO;
		return ci->amem[address];
	}
	
	mutex_lock(&ci->lock);
	write_regm(ci, 0x06,0x00,0x05);
	read_pccard(ci, 0, &ci->amem[0], 128);
	read_pccard(ci, 128, &ci->amem[0], 128);
	read_pccard(ci, 256, &ci->amem[0], 128);
	read_pccard(ci, 384, &ci->amem[0], 128);
        write_regm(ci, 0x06,0x05,0x05);
	mutex_unlock(&ci->lock);
	return ci->amem[address];
#else
	u8 val;
	mutex_lock(&ci->lock);
	set_mode(ci, 1);
	read_pccard(ci, address, &val, 1);
	mutex_unlock(&ci->lock);
	//printk("%02x:%02x\n", address,val);
	return val;
#endif
}

static void dump_attribute_mem(struct cxd *ci)
{
	u8 val;
	int i;

	mutex_lock(&ci->lock);
	for (i=0; i<128; i+=2) {
		if (!(i&31))
			printk("\n%04x: ", i);
		set_mode(ci, 1);
		read_pccard(ci, i, &val, 1);
		printk("%02x ", val);
	}
	mutex_unlock(&ci->lock);
	printk("\n");
}



static int write_attribute_mem(struct dvb_ca_en50221 *ca, int slot, 
			       int address, u8 value)
{
	struct cxd *ci = ca->data;

	mutex_lock(&ci->lock);
	set_mode(ci, 1);
	write_pccard(ci, address, &value, 1);
	mutex_unlock(&ci->lock);
	return 0;
}

static int read_cam_control(struct dvb_ca_en50221 *ca, 
			    int slot, u8 address)
{
	struct cxd *ci = ca->data;
	u8 val;
	
	mutex_lock(&ci->lock);
	set_mode(ci, 0);
	read_io(ci, address, &val);
	mutex_unlock(&ci->lock);
	return val;
}

static int write_cam_control(struct dvb_ca_en50221 *ca, int slot,
			     u8 address, u8 value)
{
	struct cxd *ci = ca->data;
	
	mutex_lock(&ci->lock);
	set_mode(ci, 0);
	write_io(ci, address, value);
	mutex_unlock(&ci->lock);
	return 0;
}

static int slot_reset(struct dvb_ca_en50221 *ca, int slot)
{
	struct cxd *ci = ca->data;

	mutex_lock(&ci->lock);
#if 0
	write_reg(ci, 0x00,0x21);
	write_reg(ci, 0x06,0x1F);
	write_reg(ci, 0x00,0x31);
#else
#if 0
	write_reg(ci, 0x06,0x1F);
	write_reg(ci, 0x06,0x2F);
#else
	cam_mode(ci, 0);
	write_reg(ci, 0x00,0x21);
	write_reg(ci, 0x06,0x1F);
	write_reg(ci, 0x00,0x31);
	write_regm(ci, 0x20, 0x80, 0x80);
	write_reg(ci, 0x03, 0x02);
	ci->ready=0;
#endif
#endif
	ci->mode=-1;
	{
		int i;
		u8 val;
		for (i=0; i<100;i++) {
			msleep(10);
#if 0
			read_reg(ci,0x06,&val);
			printk("%d:%02x\n", i, val);
			if (!(val&0x10))
				break;
#else
			if (ci->ready)
				break;
#endif
		}
	}
	mutex_unlock(&ci->lock);
	//msleep(500);
	return 0;
}

static int slot_shutdown(struct dvb_ca_en50221 *ca, int slot)
{
	struct cxd *ci = ca->data;
	
	//printk("slot_shutdown\n");
	mutex_lock(&ci->lock);
	//write_regm(ci, 0x09,0x08,0x08);
        write_regm(ci, 0x20,0x80,0x80); // Reset CAM Mode
        write_regm(ci, 0x06,0x07,0x07); // Clear IO Mode
	ci->mode=-1;
	mutex_unlock(&ci->lock);
	return 0;// shutdown(ci);
}

static int slot_ts_enable(struct dvb_ca_en50221 *ca, int slot)
{
	struct cxd *ci = ca->data;

	//printk("TS ENABLE\n");
	mutex_lock(&ci->lock);
	write_regm(ci, 0x09,0x00,0x08);
	set_mode(ci, 0);
#ifdef BUFFER_MODE
	cam_mode(ci, 1);
#endif
	mutex_unlock(&ci->lock);
	return 0;
}


static int campoll(struct cxd *ci)
{
	u8 slotstat;
	u8 istat;
	int status;
	
	read_reg(ci, 0x04, &istat);
	if (!istat)
		return 0;
	write_reg(ci, 0x05, istat);

	if (istat&0x40) {
		ci->dr=1;
		printk("DR\n");
	}
	if (istat&0x20) {
		printk("WC\n");
	}
	if (istat&2) {
		u8 slotstat;
		
		read_reg(ci, 0x01, &slotstat);
		if (!(2&slotstat)) {
			if (!ci->slot_stat) {
				ci->slot_stat|=DVB_CA_EN50221_POLL_CAM_PRESENT;
				write_regm(ci, 0x03, 0x08, 0x08);
			}
			
		} else { 
			if (ci->slot_stat) {
				ci->slot_stat=0;
				write_regm(ci, 0x03, 0x00, 0x08);
				printk("NO CAM\n");
				ci->ready=0;
			}
		}
		if (istat&8 && ci->slot_stat==DVB_CA_EN50221_POLL_CAM_PRESENT) {
			ci->ready=1;
			ci->slot_stat|=DVB_CA_EN50221_POLL_CAM_READY;
		}
	}
	return 0;
}
	

static int poll_slot_status(struct dvb_ca_en50221 *ca, int slot, int open)
{
	struct cxd *ci = ca->data;
	u8 slotstat;
	int stat=0;
	
	mutex_lock(&ci->lock);
	campoll(ci);
	read_reg(ci, 0x01, &slotstat);
	mutex_unlock(&ci->lock);

	return ci->slot_stat;
}

#ifdef BUFFER_MODE
static int read_data(struct dvb_ca_en50221* ca, int slot, u8 *ebuf, int ecount)
{
	struct cxd *ci = ca->data;
	u8 msb, lsb;
	u16 len;

	mutex_lock(&ci->lock);
	campoll(ci);
	mutex_unlock(&ci->lock);

	printk("read_data\n");
	if (!ci->dr)
		return 0;

	mutex_lock(&ci->lock);
	read_reg(ci, 0x0f, &msb);
	read_reg(ci, 0x10, &lsb);
	len=(msb<<8)|lsb;
	read_block(ci, 0x12, ebuf, len);
	ci->dr=0;
	mutex_unlock(&ci->lock);

	return len;
}

static int write_data(struct dvb_ca_en50221* ca, int slot, u8 * ebuf, int ecount)
{
	struct cxd *ci = ca->data;

	mutex_lock(&ci->lock);
	printk("write_data %d\n", ecount);
        write_reg(ci, 0x0d, ecount>>8);
        write_reg(ci, 0x0e, ecount&0xff);
	write_block(ci, 0x11, ebuf, ecount);
	mutex_unlock(&ci->lock);
	return ecount;
}
#endif

static struct dvb_ca_en50221 en_templ = {
	.read_attribute_mem  = read_attribute_mem,
	.write_attribute_mem = write_attribute_mem,
	.read_cam_control    = read_cam_control,
	.write_cam_control   = write_cam_control,
	.slot_reset          = slot_reset,
	.slot_shutdown       = slot_shutdown,
	.slot_ts_enable      = slot_ts_enable,
	.poll_slot_status    = poll_slot_status,
#ifdef BUFFER_MODE
	.read_data           = read_data,
	.write_data          = write_data,
#endif

};

struct dvb_ca_en50221 *cxd2099_attach(u8 adr, void *priv, 
				      struct i2c_adapter *i2c)
{
	struct cxd *ci = 0;
	u32 bitrate=62000000;
	u8 val;

	if (i2c_read_reg(i2c, adr, 0, &val)<0) {
		printk("No CXD2099 detected at %02x\n", adr);
		return 0;
	}
	
	ci = kmalloc(sizeof(struct cxd), GFP_KERNEL);
	if (!ci)
		return 0;
	memset(ci, 0, sizeof(*ci));

	mutex_init(&ci->lock);
	ci->i2c = i2c;
	ci->adr = adr;
	ci->lastaddress=0xff;
	ci->clk_reg_b=0x4a;
	ci->clk_reg_f=0x1b;
	ci->bitrate=bitrate;
	
	memcpy(&ci->en, &en_templ, sizeof(en_templ));
	ci->en.data=ci;
	init(ci);
	printk("Attached CXD2099AR at %02x\n", ci->adr);
	
	//dump_attribute_mem(ci);
	return &ci->en;
} 

EXPORT_SYMBOL(cxd2099_attach);

MODULE_DESCRIPTION("cxd2099");
MODULE_AUTHOR("Ralph Metzler");
MODULE_LICENSE("GPL");
