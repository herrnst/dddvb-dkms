#ifndef _UAPI_DVBNS_H_
#define _UAPI_DVBNS_H_

#include <linux/types.h>

struct dvb_ns_params {
	__u8     smac[6];
	__u8     dmac[6];
	__u8     sip[16];
	__u8     dip[16];
	__u16    sport;
	__u16    dport;
	__u16    sport2;
	__u16    dport2;
	__u8     ssrc[8];
	__u8     flags;
	__u8     qos;
	__u16    vlan;
	__u8     ttl;
};

#define DVB_NS_IPV6    1
#define DVB_NS_RTP     2
#define DVB_NS_RTCP    4
#define DVB_NS_RTP_TO  8

struct dvb_ns_rtcp {
	__u8    *msg;
	__u16    len;
};

struct dvb_ns_packet {
	__u8    *buf;
	__u8     count;
};

#define NS_SET_NET               _IOW('o', 192, struct dvb_ns_params)
#define NS_START                 _IO('o', 193)
#define NS_STOP                  _IO('o', 194)
#define NS_SET_PID               _IOW('o', 195, __u16)
#define NS_SET_PIDS              _IOW('o', 196, __u8 *)
#define NS_SET_RTCP_MSG          _IOW('o', 197, struct dvb_ns_rtcp)
#define NS_SET_PACKETS           _IOW('o', 199, struct dvb_ns_packet)
#define NS_INSERT_PACKETS	 _IOW('o', 200, __u8)
#define NS_SET_CI	         _IOW('o', 201, __u8)


struct dvb_nsd_ts {
	__u16    pid;
	__u16    num;
	__u16    input;
	__u16    timeout;
	__u16    len;
	__u8    *ts;
	__u8     mode;
	__u8     table;
	__u8     section;
};

#define NSD_GET_TS               _IOWR('o', 198, struct dvb_nsd_ts)

#endif /*_UAPI_DVBNS_H_*/
