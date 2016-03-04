#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/signal.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/types.h>

#include <linux/net.h>
#include <net/sock.h>
#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/in.h>
#include <linux/tcp.h>
#include <asm/uaccess.h>
#include <linux/file.h>
#include <linux/socket.h>
#include <linux/smp_lock.h>
#include <linux/slab.h>

struct ddb_net {
        struct task_struct *thread;
	struct socket *sock;
        struct sockaddr_in addr;
};

int ksocket_send(struct socket *sock, struct sockaddr_in *addr, 
		 unsigned char *buf, int len)
{
        struct msghdr msg;
        struct iovec iov;
	
        if (!sock->sk)
		return 0;
	
        iov.iov_base = buf;
        iov.iov_len = len;
	
        msg.msg_flags = 0;
        msg.msg_name = addr;
        msg.msg_namelen = sizeof(struct sockaddr_in);
        msg.msg_control = NULL;
        msg.msg_controllen = 0;
        msg.msg_control = NULL;
	
        return kernel_sendmsg(sock, &msg, (struct kvec *) &iov, 1, len);
}

int ksocket_receive(struct socket *sock, struct sockaddr_in *addr, 
		    unsigned char *buf, int len)
{
        struct msghdr msg;
        struct iovec iov;
	
        if (!sock->sk) 
	        return 0;
	
        iov.iov_base = buf;
        iov.iov_len = len;
	
        msg.msg_flags = 0;
        msg.msg_name = addr;
        msg.msg_namelen = sizeof(*addr);
        msg.msg_control = NULL;
        msg.msg_controllen = 0;
        msg.msg_control = NULL;
	
        return kernel_recvmsg(sock, &msg, (struct kvec *) &iov, 
			      1, len, msg.msg_flags);
}

#define DDB_PORT 5555

static int kds_thread(void *data)
{
	struct kds *kds = data;
	int stat;
        int size;
        int bufsize = 10;
        unsigned char buf[bufsize+1];

        if (((stat = sock_create(AF_INET, SOCK_DGRAM, 
				 IPPROTO_UDP, &kds->sock)) < 0))
                goto error;
	
        memset(&kds->addr, 0, sizeof(struct sockaddr));
        memset(&kds->addr_send, 0, sizeof(struct sockaddr));
        kds->addr.sin_family = AF_INET;
        kds->addr_send.sin_family = AF_INET;
	
        kds->addr.sin_addr.s_addr = htonl(INADDR_ANY);
        kds->addr_send.sin_addr.s_addr = htonl(INADDR_SEND);
	
        kds->addr.sin_port = htons(DEFAULT_PORT);
        kds->addr_send.sin_port = htons(CONNECT_PORT);

        if (((stat = kernel_bind(kds->sock, (struct sockaddr *)&kds->addr, 
				 sizeof(struct sockaddr))) < 0))
		goto out;

        printk("ddb_port %d\n", DDB_PORT);
	
	while (!kthread_should_stop()) {
	        msleep(100);
		
		memset(&buf, 0, bufsize+1);
		size = ksocket_receive(kds->sock, &kds->addr, buf, bufsize);
		
		if (size < 0)
		        printk("error getting datagram, sock_recvmsg error = %d\n", size);
                else {
		        printk("received %d bytes\n", size);
                        /* data processing */
                        printk("\n data: %s\n", buf);

                        memset(&buf, 0, bufsize+1);
                        strcat(buf, "testing...");
                        ksocket_send(kds->sock, &kds->addr_send, buf, strlen(buf));
                }
        }
	
out:
        sock_release(kds->sock);
        kds->sock = NULL;
error:
        kds->thread = NULL;
	return 0;
}

static int ddb_net_init(struct ddb_net *net)
{
	memset(net, 0, sizeof(*net));
	net->thread = kthread_run(net_thread, net, "ddb_net");
	if (IS_ERR(net->thread)) {
		printk("no thread\n");
		return -1;
	}
	return 0;
}

static void ddb_net_exit(struct ddb_net *net)
{
	kthread_stop(net->thread);
	return;
}
