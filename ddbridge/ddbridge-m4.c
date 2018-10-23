/*
 * ddbridge-m4.c: Digital Devices MAX M4 driver
 *
 * Copyright (C) 2018 Digital Devices GmbH
 *                    Marcus Metzler <mocm@metzlerbros.de>
 *                    Ralph Metzler <rjkm@metzlerbros.de>
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
 * along with this program; if not, point your browser to
 * http://www.gnu.org/copyleft/gpl.html
 */

#include "ddbridge.h"
#include "ddbridge-io.h"
#include "ddbridge-i2c.h"
#include "ddbridge-mci.h"

struct m4_base {
	struct mci_base mci_base;

};

struct m4 {
	struct mci  mci;

	int         started;
	int         t2_signalling_valid;
	int         iq_constellation_point;
	int         iq_constellation_point_max;
	int         iq_constellation_tap;
	int         first_time_lock;
};

static int stop(struct dvb_frontend *fe)
{
	struct m4 *state = fe->demodulator_priv;
	struct mci_command cmd;
	struct mci_base *mci_base = state->mci.base;

	if (!state->started)
		return -1;
	state->started = 0;
	state->t2_signalling_valid = 0;
	memset(&cmd, 0, sizeof(cmd));
	cmd.command = MCI_CMD_STOP;
	cmd.demod = state->mci.demod;
	ddb_mci_cmd(&state->mci, &cmd, NULL);
	return 0;
}

static int search_s2(struct dvb_frontend *fe)
{
	struct m4 *state = fe->demodulator_priv;
	struct mci_base *mci_base = state->mci.base;
	struct m4_base *m4_base = (struct m4_base *) mci_base;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct mci_command cmd;
	int stat;

	memset(&cmd, 0, sizeof(cmd));
	cmd.command = MCI_CMD_SEARCH_DVBS;
	cmd.dvbs2_search.flags = 3;
	cmd.dvbs2_search.s2_modulation_mask = 3;
	cmd.dvbs2_search.retry = 2;
	cmd.dvbs2_search.frequency = p->frequency * 1000;
	cmd.dvbs2_search.symbol_rate = p->symbol_rate;
	cmd.dvbs2_search.scrambling_sequence_index = 0; //p->scrambling_sequence_index;
	cmd.dvbs2_search.input_stream_id = p->stream_id;
	cmd.tuner = state->mci.nr;
	cmd.demod = state->mci.tuner;
	cmd.output = state->mci.nr;

	stat = ddb_mci_cmd(&state->mci, &cmd, NULL);
	if (stat)
		stop(fe);
	return stat;
}

static int search_c(struct dvb_frontend *fe)
{
	struct m4 *state = fe->demodulator_priv;
	struct mci_base *mci_base = state->mci.base;
	struct m4_base *m4_base = (struct m4_base *) mci_base;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct mci_command cmd;
	int stat;

	memset(&cmd, 0, sizeof(cmd));
	cmd.command = MCI_CMD_SEARCH_DVBC;
	switch (p->bandwidth_hz) {
	case 6000000:
		cmd.dvbc_search.bandwidth = MCI_BANDWIDTH_6MHZ;
		break;
	case 7000000:
		cmd.dvbc_search.bandwidth = MCI_BANDWIDTH_7MHZ;
		break;
	default:
		cmd.dvbc_search.bandwidth = MCI_BANDWIDTH_8MHZ;
		break;
	}
	cmd.dvbc_search.retry = 2;
	cmd.dvbc_search.frequency = p->frequency;
	cmd.tuner = state->mci.tuner;
	cmd.demod = state->mci.demod;
	cmd.output = state->mci.nr;

	stat = ddb_mci_cmd(&state->mci, &cmd, NULL);
	if (stat)
		stop(fe);
	return stat;
}

static int search_t(struct dvb_frontend *fe)
{
	struct m4 *state = fe->demodulator_priv;
	struct mci_base *mci_base = state->mci.base;
	struct m4_base *m4_base = (struct m4_base *) mci_base;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct mci_command cmd;
	int stat;

	memset(&cmd, 0, sizeof(cmd));
	cmd.command = MCI_CMD_SEARCH_DVBT;
	switch (p->bandwidth_hz) {
	case 5000000:
		cmd.dvbt_search.bandwidth = MCI_BANDWIDTH_5MHZ;
		break;
	case 6000000:
		cmd.dvbt_search.bandwidth = MCI_BANDWIDTH_6MHZ;
		break;
	case 7000000:
		cmd.dvbt_search.bandwidth = MCI_BANDWIDTH_7MHZ;
		break;
	default:
		cmd.dvbt_search.bandwidth = MCI_BANDWIDTH_8MHZ;
		break;
	}
	cmd.dvbt_search.retry = 2;
	cmd.dvbt_search.frequency = p->frequency;
	cmd.tuner = state->mci.tuner;
	cmd.demod = state->mci.demod;
	cmd.output = state->mci.nr;

	stat = ddb_mci_cmd(&state->mci, &cmd, NULL);
	if (stat)
		stop(fe);
	return stat;
}

static int search_t2(struct dvb_frontend *fe)
{
	struct m4 *state = fe->demodulator_priv;
	struct mci_base *mci_base = state->mci.base;
	struct m4_base *m4_base = (struct m4_base *) mci_base;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct mci_command cmd;
	int stat;
	u32 flags = 0;

	memset(&cmd, 0, sizeof(cmd));
	cmd.command = MCI_CMD_SEARCH_DVBT2;
	switch (p->bandwidth_hz) {
	case 1700000:
		cmd.dvbt_search.bandwidth = MCI_BANDWIDTH_1_7MHZ;
		break;
	case 5000000:
		cmd.dvbt_search.bandwidth = MCI_BANDWIDTH_5MHZ;
		break;
	case 6000000:
		cmd.dvbt_search.bandwidth = MCI_BANDWIDTH_6MHZ;
		break;
	case 7000000:
		cmd.dvbt_search.bandwidth = MCI_BANDWIDTH_7MHZ;
		break;
	default:
		cmd.dvbt_search.bandwidth = MCI_BANDWIDTH_8MHZ;
		break;
	}
	cmd.dvbt2_search.retry = 2;
	cmd.dvbt2_search.frequency = p->frequency;
	if (p->stream_id != NO_STREAM_ID_FILTER) {
		cmd.dvbt2_search.plp = p->stream_id & 0xff;
		cmd.dvbt2_search.flags |= 0x80;
		cmd.dvbt2_search.flags |= (p->stream_id >> 8) & 1;
	}
	cmd.tuner = state->mci.tuner;
	cmd.demod = state->mci.demod;
	cmd.output = state->mci.nr;

	stat = ddb_mci_cmd(&state->mci, &cmd, NULL);
	if (stat)
		stop(fe);
	return stat;
}

static int search_c2(struct dvb_frontend *fe)
{
	struct m4 *state = fe->demodulator_priv;
	struct mci_base *mci_base = state->mci.base;
	struct m4_base *m4_base = (struct m4_base *) mci_base;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct mci_command cmd;
	int stat;
	u32 flags = 0;

	memset(&cmd, 0, sizeof(cmd));
	cmd.command = MCI_CMD_SEARCH_DVBC2;
	switch (p->bandwidth_hz) {
	case 6000000:
		cmd.dvbc2_search.bandwidth = MCI_BANDWIDTH_6MHZ;
		break;
	default:
		cmd.dvbc2_search.bandwidth = MCI_BANDWIDTH_8MHZ;
		break;
	}
	cmd.dvbc2_search.retry = 2;
	cmd.dvbc2_search.frequency = p->frequency;
	if (p->stream_id != NO_STREAM_ID_FILTER) {
		cmd.dvbc2_search.plp = p->stream_id & 0xff;
		cmd.dvbc2_search.data_slice = (p->stream_id >> 8) & 0xff;
	}
	cmd.tuner = state->mci.tuner;
	cmd.demod = state->mci.demod;
	cmd.output = state->mci.nr;

	stat = ddb_mci_cmd(&state->mci, &cmd, NULL);
	if (stat)
		stop(fe);
	return stat;
}

static int search_isdbt(struct dvb_frontend *fe)
{
	struct m4 *state = fe->demodulator_priv;
	struct mci_base *mci_base = state->mci.base;
	struct m4_base *m4_base = (struct m4_base *) mci_base;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct mci_command cmd;
	int stat;
	u32 flags = 0;

	memset(&cmd, 0, sizeof(cmd));
	cmd.command = MCI_CMD_SEARCH_ISDBT;
	switch (p->bandwidth_hz) {
	case 8000000:
		cmd.isdbt_search.bandwidth = MCI_BANDWIDTH_8MHZ;
		break;
	case 7000000:
		cmd.isdbt_search.bandwidth = MCI_BANDWIDTH_7MHZ;
		break;
	default:
		cmd.isdbt_search.bandwidth = MCI_BANDWIDTH_6MHZ;
		break;
	}
	cmd.isdbt_search.retry = 2;
	cmd.isdbt_search.frequency = p->frequency;
	cmd.tuner = state->mci.tuner;
	cmd.demod = state->mci.demod;
	cmd.output = state->mci.nr;

	stat = ddb_mci_cmd(&state->mci, &cmd, NULL);
	if (stat)
		stop(fe);
	return stat;
}


static int set_parameters(struct dvb_frontend *fe)
{
	struct m4 *state = fe->demodulator_priv;
	int res;

	stop(fe);

	state->t2_signalling_valid = 0;
	state->iq_constellation_point = 0;
	state->iq_constellation_point_max = 0;

	state->iq_constellation_tap = 0;
	switch (fe->dtv_property_cache.delivery_system) {
	case SYS_DVBS:
	case SYS_DVBS2:
		res = search_s2(fe);
		break;
	case SYS_DVBC_ANNEX_A:
		res = search_c(fe);
		break;
	case SYS_DVBT:
		state->iq_constellation_tap = 5;
		res = search_t(fe);
		break;
	case SYS_DVBT2:
		res = search_t2(fe);
		break;
	case SYS_DVBC2:
		res = search_c2(fe);
		break;
	case SYS_ISDBT:
		res = search_isdbt(fe);
		break;
	default:
		return -EINVAL;
	}
	if (!res) {
		state->started = 1;
		state->first_time_lock = 1;
	}
	return res;
}

static int read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	int stat;
	struct m4 *state = fe->demodulator_priv;
	struct mci_result res;

	stat = ddb_mci_get_status(&state->mci, &res);
	if (stat)
		return stat;
	*status = 0x00;
	ddb_mci_get_info(&state->mci);
	ddb_mci_get_strength(fe);
	if (res.status == MCI_DEMOD_WAIT_SIGNAL)
		*status = 0x01;
	if (res.status == MCI_DEMOD_LOCKED) {
		*status = 0x1f;
		ddb_mci_get_snr(fe);
	}
	return stat;
}

static int tune(struct dvb_frontend *fe, bool re_tune,
		unsigned int mode_flags,
		unsigned int *delay, enum fe_status *status)
{
	int r;

	if (re_tune) {
		r = set_parameters(fe);
		if (r)
			return r;
	}
	r = read_status(fe, status);
	if (r)
		return r;

	if (*status & FE_HAS_LOCK)
		return 0;
	*delay = HZ / 10;
	return 0;
}

static int sleep(struct dvb_frontend *fe)
{
	struct m4 *state = fe->demodulator_priv;

	if (state->started)
		stop(fe);
	return 0;
}

static void release(struct dvb_frontend *fe)
{
	struct m4 *state = fe->demodulator_priv;
	struct mci_base *mci_base = state->mci.base;

	mci_base->count--;
	if (mci_base->count == 0) {
		list_del(&mci_base->mci_list);
		kfree(mci_base);
	}
	kfree(state);
}

static int get_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

static int get_frontend(struct dvb_frontend *fe, struct dtv_frontend_properties *p)
{
	struct m4 *state = fe->demodulator_priv;

	ddb_mci_proc_info(&state->mci, p);
	return 0;
}

static struct dvb_frontend_ops m4_ops = {
	.delsys = { SYS_DVBC_ANNEX_A, SYS_DVBT, SYS_DVBT2, SYS_DVBC2, SYS_ISDBT,
		    SYS_DVBS, SYS_DVBS2, },
	.info = {
		.name = "M4",
		.frequency_min = 950000,	/* DVB-T: 47125000 */
		.frequency_max = 865000000,	/* DVB-C: 862000000 */
		.symbol_rate_min = 100000,
		.symbol_rate_max = 100000000,
		.frequency_stepsize	= 0,
		.frequency_tolerance	= 0,
		.caps = FE_CAN_QPSK | FE_CAN_QAM_16 | FE_CAN_QAM_32 |
		        FE_CAN_QAM_64 | FE_CAN_QAM_128 | FE_CAN_QAM_256 |
		        FE_CAN_QAM_AUTO | 
			FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			FE_CAN_FEC_4_5 |
			FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO | FE_CAN_HIERARCHY_AUTO |
			FE_CAN_RECOVER | FE_CAN_MUTE_TS | FE_CAN_2G_MODULATION
	},
	.release                        = release,
	.get_frontend_algo              = get_algo,
	.get_frontend                   = get_frontend,
	.read_status                    = read_status,
	.tune                           = tune,
	.sleep                          = sleep,
};

static int init(struct mci *mci)
{
	//struct m4 *state = (struct m4 *) mci;

	return 0;
}

static int base_init(struct mci_base *mci_base)
{
	//struct m4_base *base = (struct m4_base *) mci_base;

	return 0;
}

struct mci_cfg ddb_max_m4_cfg = {
	.type = 0,
	.fe_ops = &m4_ops,
	.base_size = sizeof(struct m4_base),
	.state_size = sizeof(struct m4),
	.init = init,
	.base_init = base_init,
};
