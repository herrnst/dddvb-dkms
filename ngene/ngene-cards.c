/*
 * ngene-cards.c: nGene PCIe bridge driver - card specific info
 *
 * Copyright (C) 2005-2007 Micronas
 *
 * Copyright (C) 2008-2009 Ralph Metzler <rjkm@metzlerbros.de>
 *                         Modifications for new nGene firmware,
 *                         support for EEPROM-copying,
 *                         support for new dual DVB-S2 card prototype
 *
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
#include <linux/pci.h>
#include <linux/pci_ids.h>

#include "ngene.h"

/* demods/tuners */
#include "stv6110x.h"
#include "stv090x.h"
#include "lnbh24.h"


/****************************************************************************/
/* Demod/tuner attachment ***************************************************/
/****************************************************************************/

static int tuner_attach_stv6110(struct ngene_channel *chan)
{
	struct i2c_adapter *i2c;
	struct stv090x_config *feconf = (struct stv090x_config *)
		chan->dev->card_info->fe_config[chan->number];
	struct stv6110x_config *tunerconf = (struct stv6110x_config *)
		chan->dev->card_info->tuner_config[chan->number];
	struct stv6110x_devctl *ctl;

	/* tuner 1+2: i2c adapter #0, tuner 3+4: i2c adapter #1 */
	if (chan->number < 2)
		i2c = &chan->dev->channel[0].i2c_adapter;
	else
		i2c = &chan->dev->channel[1].i2c_adapter;

	ctl = dvb_attach(stv6110x_attach, chan->fe, tunerconf, i2c);
	if (ctl == NULL) {
		printk(KERN_ERR	DEVICE_NAME ": No STV6110X found!\n");
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

#if 0
static int tuner_attach_mt2060(struct ngene_channel *chan)
{
	struct ngene *dev = chan->dev;
	void *tconf = dev->card_info->tuner_config[chan->number];
	u8 drxa = dev->card_info->demoda[chan->number];
	struct dvb_frontend *fe = chan->fe, *fe2;

	fe->sec_priv = chan;
	fe->ops.i2c_gate_ctrl = dev->card_info->gate_ctrl;

	dev->card_info->gate_ctrl(fe, 1);
	fe2 = mt2060_attach(fe, &chan->i2c_adapter, tconf, 1220);
	dev->card_info->gate_ctrl(fe, 0);

	i2c_write_register(&chan->i2c_adapter, drxa, 3, 4);
	write_demod(&chan->i2c_adapter, drxa, 0x1012, 15);
	write_demod(&chan->i2c_adapter, drxa, 0x1007, 0xc27);
	write_demod(&chan->i2c_adapter, drxa, 0x0020, 0x003);

	return fe2 ? 0 : -ENODEV;
}

static int tuner_attach_xc3028(struct ngene_channel *chan)
{
	struct ngene *dev = chan->dev;
	void *tconf = dev->card_info->tuner_config[chan->number];
	struct dvb_frontend *fe = chan->fe, *fe2;

	fe->sec_priv = chan;
	fe->ops.i2c_gate_ctrl = dev->card_info->gate_ctrl;

	dev->card_info->gate_ctrl(fe, 1);
	fe2 = xc3028_attach(fe, &chan->i2c_adapter, tconf);
	dev->card_info->gate_ctrl(fe, 0);

	/*chan->fe->ops.tuner_ops.set_frequency(chan->fe,231250000);*/

	return fe2 ? 0 : -ENODEV;
}

static int demod_attach_drxd(struct ngene_channel *chan)
{
	void *feconf = chan->dev->card_info->fe_config[chan->number];

	chan->fe = drxd_attach(feconf,
			       chan, &chan->i2c_adapter,
			       &chan->dev->pci_dev->dev);
	return (chan->fe) ? 0 : -ENODEV;
}

static int demod_attach_drxh(struct ngene_channel *chan)
{
	void *feconf = chan->dev->card_info->fe_config[chan->number];

	chan->fe = drxh_attach(feconf, chan,
			       &chan->i2c_adapter, &chan->dev->pci_dev->dev);
	return (chan->fe) ? 0 : -ENODEV;
}

static int demod_attach_stb0899(struct ngene_channel *chan)
{
	void *feconf = chan->dev->card_info->fe_config[chan->number];

	chan->fe = stb0899_attach(feconf,
				  chan, &chan->i2c_adapter,
				  &chan->dev->pci_dev->dev);
	if (chan->fe) {
		chan->set_tone = chan->fe->ops.set_tone;
		chan->fe->ops.set_tone = lnbh21_set_tone;
		chan->fe->ops.set_voltage = lnbh21_set_voltage;
	}

	return (chan->fe) ? 0 : -ENODEV;
}
#endif

static int demod_attach_stv0900(struct ngene_channel *chan)
{
	struct i2c_adapter *i2c;
	struct stv090x_config *feconf = (struct stv090x_config *)
		chan->dev->card_info->fe_config[chan->number];

	/* tuner 1+2: i2c adapter #0, tuner 3+4: i2c adapter #1 */
	/* Note: Both adapters share the same i2c bus, but the demod     */
	/*       driver requires that each demod has its own i2c adapter */
	if (chan->number < 2)
		i2c = &chan->dev->channel[0].i2c_adapter;
	else
		i2c = &chan->dev->channel[1].i2c_adapter;

	chan->fe = dvb_attach(stv090x_attach, feconf, i2c,
			(chan->number & 1) == 0 ? STV090x_DEMODULATOR_0
						: STV090x_DEMODULATOR_1);
	if (chan->fe == NULL) {
		printk(KERN_ERR	DEVICE_NAME ": No STV0900 found!\n");
		return -ENODEV;
	}

	/* store channel info */
	if (feconf->tuner_i2c_lock)
		chan->fe->analog_demod_priv = chan;

	if (!dvb_attach(lnbh24_attach, chan->fe, i2c, 0,
			0, chan->dev->card_info->lnb[chan->number])) {
		printk(KERN_ERR DEVICE_NAME ": No LNBH24 found!\n");
		dvb_frontend_detach(chan->fe);
		chan->fe = NULL;
		return -ENODEV;
	}

	return 0;
}

static void cineS2_tuner_i2c_lock(struct dvb_frontend *fe, int lock)
{
	struct ngene_channel *chan = fe->analog_demod_priv;

	if (lock)
		down(&chan->dev->pll_mutex);
	else
		up(&chan->dev->pll_mutex);
}

static int cineS2_probe(struct ngene_channel *chan)
{
	struct i2c_adapter *i2c;
	struct stv090x_config *fe_conf;
	u8 buf[3];
	struct i2c_msg i2c_msg = { .flags = 0, .buf = buf };
	int rc;

	/* tuner 1+2: i2c adapter #0, tuner 3+4: i2c adapter #1 */
	if (chan->number < 2)
		i2c = &chan->dev->channel[0].i2c_adapter;
	else
		i2c = &chan->dev->channel[1].i2c_adapter;

	fe_conf = chan->dev->card_info->fe_config[chan->number];
	i2c_msg.addr = fe_conf->address;

	/* probe demod */
	i2c_msg.len = 2;
	buf[0] = 0xf1;
	buf[1] = 0x00;
	rc = i2c_transfer(i2c, &i2c_msg, 1);
	if (rc != 1)
		return -ENODEV;

	/* demod found, attach it */
	rc = demod_attach_stv0900(chan);
	if (rc < 0 || chan->number < 2)
		return rc;

	/* demod #2: reprogram outputs DPN1 & DPN2 */
	i2c_msg.len = 3;
	buf[0] = 0xf1;
	switch (chan->number)
	{
	case 2:
		buf[1] = 0x5c;
		buf[2] = 0xc2;
		break;
	case 3:
		buf[1] = 0x61;
		buf[2] = 0xcc;
		break;
	default:
		return -ENODEV;
	}
	rc = i2c_transfer(i2c, &i2c_msg, 1);
	if (rc != 1) {
		printk(KERN_ERR DEVICE_NAME ": could not setup DPNx\n");
		return -EIO;
	}

	return 0;
}


/****************************************************************************/
/* Switch control (I2C gates, etc.) *****************************************/
/****************************************************************************/

#if 0
static int avf_output(struct ngene_channel *chan, int state)
{
	if (chan->dev->card_info->avf[chan->number])
		i2c_write_register(&chan->i2c_adapter,
				   chan->dev->card_info->avf[chan->number],
				   0xf2, state ? 0x89 : 0x80);
	return 0;
}

/* Viper expander: sw11,sw12,sw21,sw22,i2csw1,i2csw2,tsen1,tsen2 */

static int exp_set(struct ngene *dev)
{
	return i2c_write(&dev->channel[0].i2c_adapter,
			 dev->card_info->exp, dev->exp_val);
}

static int exp_init(struct ngene *dev)
{
	if (!dev->card_info->exp)
		return 0;
	dev->exp_val = dev->card_info->exp_init;
	return exp_set(dev);
}

static int exp_set_bit(struct ngene *dev, int bit, int val)
{
	if (val)
		set_bit(bit, &dev->exp_val);
	else
		clear_bit(bit, &dev->exp_val);
	return exp_set(dev);
}

static int viper_switch_ctrl(struct ngene_channel *chan, int type, int val)
{
	switch (type) {
	case 0: /* I2C tuner gate on/off */
		return exp_set_bit(chan->dev, 4 + chan->number, val);
	case 1: /* Stream: 0=TS 1=ITU */
		avf_output(chan, val);
		return exp_set_bit(chan->dev, 6 + chan->number, val);
	case 2: /* Input: 0=digital 1=analog antenna input */
		exp_set_bit(chan->dev, 0 + chan->number * 2, val ? 0 : 1);
		exp_set_bit(chan->dev, 1 + chan->number * 2, val ? 1 : 0);
		break;
	}
	return 0;
}

static int viper_switch_ctrl2(struct ngene_channel *chan, int type, int val)
{
	switch (type) {
	case 0: /* I2C tuner gate on/off */
		return exp_set_bit(chan->dev, 4 + chan->number, val);
	case 1: /* Stream: 0=TS 1=ITU */
		avf_output(chan, val);
		return exp_set_bit(chan->dev, 6 + chan->number, val);
	case 2: /* Input: 0=digital 1=analog antenna input */
		exp_set_bit(chan->dev, 0 + chan->number * 2, val ? 0 : 1);
		exp_set_bit(chan->dev, 1 + chan->number * 2, 0);
		break;
	}
	return 0;
}

static int viper_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	/* Well, just abuse sec :-) */
	struct ngene_channel *chan = fe->sec_priv;
	struct ngene *dev = chan->dev;

	return dev->card_info->switch_ctrl(chan, 0, enable);
}

static int python_switch_ctrl(struct ngene_channel *chan, int type, int val)
{
	switch (type) {
	case 0: /* I2C tuner gate on/off */
		if (chan->number > 1)
			return -EINVAL;
		return ngene_command_gpio_set(chan->dev, 3 + chan->number, val);
	case 1: /* Stream: 0=TS 1=ITU */
		avf_output(chan, val);
		return 0;
	}
	return 0;
}

static int viper_reset_xc(struct dvb_frontend *fe)
{
	struct ngene_channel *chan = fe->sec_priv;
	struct ngene *dev = chan->dev;

	printk(KERN_INFO DEVICE_NAME ": Reset XC3028\n");

	if (chan->number > 1)
		return -EINVAL;

	ngene_command_gpio_set(dev, 3 + chan->number, 0);
	msleep(150);
	ngene_command_gpio_set(dev, 3 + chan->number, 1);
	return 0;
}

static int python_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct ngene_channel *chan = fe->sec_priv;
	struct ngene *dev = chan->dev;

	if (chan->number == 0)
		return ngene_command_gpio_set(dev, 3, enable);
	if (chan->number == 1)
		return ngene_command_gpio_set(dev, 4, enable);
	return -EINVAL;
}
#endif

static struct stv090x_config fe_cineS2 = {
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

	.tuner_i2c_lock = cineS2_tuner_i2c_lock,
};

static struct stv090x_config fe_cineS2_2 = {
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

	.tuner_i2c_lock = cineS2_tuner_i2c_lock,
};

static struct stv6110x_config tuner_cineS2_0 = {
	.addr	= 0x60,
	.refclk	= 27000000,
	.clk_div = 1,
};

static struct stv6110x_config tuner_cineS2_1 = {
	.addr	= 0x63,
	.refclk	= 27000000,
	.clk_div = 1,
};

static struct ngene_info ngene_info_cineS2 = {
	.type		= NGENE_SIDEWINDER,
	.name		= "Linux4Media cineS2 DVB-S2 Twin Tuner",
	.io_type	= {NGENE_IO_TSIN, NGENE_IO_TSIN},
	.demod_attach	= {demod_attach_stv0900, demod_attach_stv0900},
	.tuner_attach	= {tuner_attach_stv6110, tuner_attach_stv6110},
	.fe_config	= {&fe_cineS2, &fe_cineS2},
	.tuner_config	= {&tuner_cineS2_0, &tuner_cineS2_1},
	.lnb		= {0x0b, 0x08},
	.tsf		= {3, 3},
	.fw_version	= 18,
	.msi_supported	= true,
};

static struct ngene_info ngene_info_satixS2 = {
	.type		= NGENE_SIDEWINDER,
	.name		= "Mystique SaTiX-S2 Dual",
	.io_type	= {NGENE_IO_TSIN, NGENE_IO_TSIN},
	.demod_attach	= {demod_attach_stv0900, demod_attach_stv0900},
	.tuner_attach	= {tuner_attach_stv6110, tuner_attach_stv6110},
	.fe_config	= {&fe_cineS2, &fe_cineS2},
	.tuner_config	= {&tuner_cineS2_0, &tuner_cineS2_1},
	.lnb		= {0x0b, 0x08},
	.tsf		= {3, 3},
	.fw_version	= 18,
	.msi_supported	= true,
};

static struct ngene_info ngene_info_satixS2v2 = {
	.type		= NGENE_SIDEWINDER,
	.name		= "Mystique SaTiX-S2 Dual (v2)",
	.io_type	= {NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN,
			   NGENE_IO_TSOUT},
	.demod_attach	= {demod_attach_stv0900, demod_attach_stv0900, cineS2_probe, cineS2_probe},
	.tuner_attach	= {tuner_attach_stv6110, tuner_attach_stv6110, tuner_attach_stv6110, tuner_attach_stv6110},
	.fe_config	= {&fe_cineS2, &fe_cineS2, &fe_cineS2_2, &fe_cineS2_2},
	.tuner_config	= {&tuner_cineS2_0, &tuner_cineS2_1, &tuner_cineS2_0, &tuner_cineS2_1},
	.lnb		= {0x0a, 0x08, 0x0b, 0x09},
	.tsf		= {3, 3},
	.fw_version	= 18,
	.msi_supported	= true,
};

static struct ngene_info ngene_info_cineS2v5 = {
	.type		= NGENE_SIDEWINDER,
	.name		= "Linux4Media cineS2 DVB-S2 Twin Tuner (v5)",
	.io_type	= {NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN,
			   NGENE_IO_TSOUT},
	.demod_attach	= {demod_attach_stv0900, demod_attach_stv0900, cineS2_probe, cineS2_probe},
	.tuner_attach	= {tuner_attach_stv6110, tuner_attach_stv6110, tuner_attach_stv6110, tuner_attach_stv6110},
	.fe_config	= {&fe_cineS2, &fe_cineS2, &fe_cineS2_2, &fe_cineS2_2},
	.tuner_config	= {&tuner_cineS2_0, &tuner_cineS2_1, &tuner_cineS2_0, &tuner_cineS2_1},
	.lnb		= {0x0a, 0x08, 0x0b, 0x09},
	.tsf		= {3, 3},
	.fw_version	= 18,
	.msi_supported	= true,
};


static struct ngene_info ngene_info_duoFlexS2 = {
	.type           = NGENE_SIDEWINDER,
	.name           = "Digital Devices DuoFlex S2 miniPCIe",
	.io_type        = {NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN,
			   NGENE_IO_TSOUT},
	.demod_attach   = {cineS2_probe, cineS2_probe, cineS2_probe, cineS2_probe},
	.tuner_attach   = {tuner_attach_stv6110, tuner_attach_stv6110, tuner_attach_stv6110, tuner_attach_stv6110},
	.fe_config      = {&fe_cineS2, &fe_cineS2, &fe_cineS2_2, &fe_cineS2_2},
	.tuner_config   = {&tuner_cineS2_0, &tuner_cineS2_1, &tuner_cineS2_0, &tuner_cineS2_1},
	.lnb            = {0x0a, 0x08, 0x0b, 0x09},
	.tsf            = {3, 3},
	.fw_version     = 18,
	.msi_supported	= true,
};

/****************************************************************************/
/* PCI Subsystem ID *********************************************************/
/****************************************************************************/

#define NGENE_ID(_subvend, _subdev, _driverdata) { \
	.vendor = NGENE_VID, .device = NGENE_PID, \
	.subvendor = _subvend, .subdevice = _subdev, \
	.driver_data = (unsigned long) &_driverdata }

/****************************************************************************/

static const struct pci_device_id ngene_id_tbl[] __devinitdata = {
	NGENE_ID(0x18c3, 0xabc3, ngene_info_cineS2),
	NGENE_ID(0x18c3, 0xabc4, ngene_info_cineS2),
	NGENE_ID(0x18c3, 0xdb01, ngene_info_satixS2),
	NGENE_ID(0x18c3, 0xdb02, ngene_info_satixS2v2),
	NGENE_ID(0x18c3, 0xdd00, ngene_info_cineS2v5),
	NGENE_ID(0x18c3, 0xdd10, ngene_info_duoFlexS2),
	NGENE_ID(0x18c3, 0xdd20, ngene_info_duoFlexS2),
	{0}
};
MODULE_DEVICE_TABLE(pci, ngene_id_tbl);

/****************************************************************************/
/* Init/Exit ****************************************************************/
/****************************************************************************/

static pci_ers_result_t ngene_error_detected(struct pci_dev *dev,
					     enum pci_channel_state state)
{
	printk(KERN_ERR DEVICE_NAME ": PCI error\n");
	if (state == pci_channel_io_perm_failure)
		return PCI_ERS_RESULT_DISCONNECT;
	if (state == pci_channel_io_frozen)
		return PCI_ERS_RESULT_NEED_RESET;
	return PCI_ERS_RESULT_CAN_RECOVER;
}

static pci_ers_result_t ngene_link_reset(struct pci_dev *dev)
{
	printk(KERN_INFO DEVICE_NAME ": link reset\n");
	return 0;
}

static pci_ers_result_t ngene_slot_reset(struct pci_dev *dev)
{
	printk(KERN_INFO DEVICE_NAME ": slot reset\n");
	return 0;
}

static void ngene_resume(struct pci_dev *dev)
{
	printk(KERN_INFO DEVICE_NAME ": resume\n");
}

static struct pci_error_handlers ngene_errors = {
	.error_detected = ngene_error_detected,
	.link_reset = ngene_link_reset,
	.slot_reset = ngene_slot_reset,
	.resume = ngene_resume,
#if 0
	int (*mmio_enabled)(struct pci_dev *dev);
#endif
};

static struct pci_driver ngene_pci_driver = {
	.name        = "ngene",
	.id_table    = ngene_id_tbl,
	.probe       = ngene_probe,
	.remove      = __devexit_p(ngene_remove),
	.err_handler = &ngene_errors,
	.shutdown    = ngene_shutdown,
};

static __init int module_init_ngene(void)
{
	printk(KERN_INFO
	       "nGene PCIE bridge driver, Copyright (C) 2005-2007 Micronas\n");
	return pci_register_driver(&ngene_pci_driver);
}

static __exit void module_exit_ngene(void)
{
	pci_unregister_driver(&ngene_pci_driver);
}

module_init(module_init_ngene);
module_exit(module_exit_ngene);

MODULE_DESCRIPTION("nGene");
MODULE_AUTHOR("Micronas, Ralph Metzler, Manfred Voelkel");
MODULE_LICENSE("GPL");
