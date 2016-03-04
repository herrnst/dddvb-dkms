#ifndef _DVB_NETSTREAM_H_
#define _DVB_NETSTREAM_H_

#include <linux/socket.h>

#define DVBNS_MAXPIDS 32

struct dvbnss {
	u32 pid_num;
	u16 pids[DVBNS_MAXPIDS];
	u8  packet[1328];
	u32 pp;
	
	struct socket *sock;
	struct sockaddr_in sadr;
	u32    sn;
};

#define MAX_DVBNSS 32

struct dvbns {
	u32    nssnum;
	struct nss[MAX_DVBNSS];
};

#endif
