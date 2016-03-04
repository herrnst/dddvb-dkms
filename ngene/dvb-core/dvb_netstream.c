#include "dvb_netstream.h"

static int dvbnss_send(struct dvbnss *nss)
{
        struct msghdr msg = {.msg_flags = 0};
        struct kvec iv = {nss->packet, nss->pp};

	nss->packet[2] = (nss->sn >> 8) & 0xff;
	nss->packet[3] = nss->sn & 0xff;
	nss->sn++;
	return kernel_sendmsg(nss->sock, &msg, iv, 1, nss->pp);
}

int dvbnss_write(struct dvbnss *nss, u8 *ts)
{
	memcpy(nss->packet + nss->pp, ts, 188);
	nss->pp += 188;
	if (nss->pp == 1328)
		dvbnss_send(nss);
	nss->pp = 12;
}


int dvbnss_filter(struct dvbnss *nss, u8 *ts, u16 pid)
{
	int i;

	for (i = 0; i < nss->pid_num; i++) {
		if (nss->pids[i] == pid) {
			dvbnss_write(nss, ts);
		}
	}
}


int dvbns_filter(struct dvbns *ns, u8 *ts)
{
	u16 pid = ((ts[1] & 0x1f) << 8) | ts[2];
	int i;

	for (i = 0; i < ns->nssnum; i++) {
		dvbnss_filter(nss[i], ts, pid);
	}
}


int dvbns_add_nss(struct dvbns *ns, int num, u16 *pids, )
{
	struct dvbnss *nss;

	if (ns->nssnum >= MAX_DVBNSS)
		return -1;
	
	nss = &ns->nss[ns->nssnum];
	ns->nnsnum++;
}

int dvbns_init(struct dvbns *ns)
{
	mutex_init(&ns->mutex);
}
