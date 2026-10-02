// SPDX-License-Identifier: GPL-2.0-only
/*
 * IP heartbeat: outgoing TCP keepalives held until DSME's heartbeat, so the radio wakes once for all
 * of them (/dev/iphb). Nokia's net/ipv4/netfilter/iphb.c from the N9's Harmattan kernel (Jukka
 * Rissanen, 2008-2010), on 6.18's netfilter: a keepalive is stolen at POST_ROUTING and sent later
 * through its hook state's okfn, from a work item. They go out when DSME writes to the device or
 * closes it, when any packet comes in (the radio is on then), when other traffic goes out, when 20
 * are queued, or after the period DSME wrote (at most 180 s). Poll wakes DSME at most once per
 * period after a flush; a read acknowledges it.
 *
 * Copyright (C) 2008-2010 Nokia Corporation. All rights reserved.
 */

#include <linux/fs.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tcp.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <net/ip.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include <net/tcp.h>

#define MAX_KEEPALIVES		20
#define MAX_FLUSH_VALUE		180	/* s */
#define MAX_RECV_LEN		15

struct keepalive {
	struct list_head list;
	struct sk_buff *skb;
	struct nf_hook_state state;	/* sk and out held */
};

static LIST_HEAD(queued);	/* held keepalives */
static LIST_HEAD(sending);	/* released, for send_work */
static int num_queued;
static DEFINE_SPINLOCK(lock);

static time64_t last_notification;
static unsigned int flush_period = MAX_FLUSH_VALUE;
static bool trigger_poll;
static DECLARE_WAIT_QUEUE_HEAD(iphb_pollq);
static bool iphb_open;

static void send_keepalives(struct work_struct *work);
static DECLARE_WORK(send_work, send_keepalives);
static void timed_flush(struct timer_list *t);
static struct timer_list flush_timer;

static void send_keepalives(struct work_struct *work)
{
	struct keepalive *k, *n;
	LIST_HEAD(list);

	spin_lock_bh(&lock);
	list_splice_init(&sending, &list);
	spin_unlock_bh(&lock);

	list_for_each_entry_safe(k, n, &list, list) {
		list_del(&k->list);
		local_bh_disable();
		rcu_read_lock();
		k->state.okfn(k->state.net, k->state.sk, k->skb);
		rcu_read_unlock();
		local_bh_enable();
		if (k->state.sk)
			sock_put(k->state.sk);
		dev_put(k->state.out);
		kfree(k);
	}
}

/* Release the held keepalives; with notify, wake DSME if a period has passed since the last time */
static void flush_keepalives(bool notify)
{
	if (notify) {
		time64_t now = ktime_get_seconds();

		if (now > last_notification + flush_period) {
			last_notification = now;
			trigger_poll = true;
			wake_up_interruptible(&iphb_pollq);
		}
	}

	spin_lock_bh(&lock);
	if (num_queued) {
		list_splice_tail_init(&queued, &sending);
		num_queued = 0;
		schedule_work(&send_work);
	}
	spin_unlock_bh(&lock);

	/* a new keepalive starts it again */
	timer_delete(&flush_timer);
}

static void timed_flush(struct timer_list *t)
{
	flush_keepalives(true);
}

static int iphbd_open(struct inode *inode, struct file *file)
{
	if (iphb_open)
		return -EBUSY;
	iphb_open = true;
	return 0;
}

static __poll_t iphbd_poll(struct file *file, poll_table *wait)
{
	poll_wait(file, &iphb_pollq, wait);
	return trigger_poll ? EPOLLIN | EPOLLRDNORM | EPOLLOUT | EPOLLWRNORM : 0;
}

static int iphbd_release(struct inode *inode, struct file *file)
{
	flush_keepalives(false);
	last_notification = 0;
	trigger_poll = false;
	iphb_open = false;
	return 0;
}

/* The flush period in seconds (DSME writes it at open); any write flushes */
static ssize_t iphbd_write(struct file *file, const char __user *buf, size_t len, loff_t *off)
{
	char received[MAX_RECV_LEN + 1];
	size_t n = min_t(size_t, MAX_RECV_LEN, len);
	long val;

	if (copy_from_user(received, buf, n))
		return -EFAULT;
	received[n] = 0;

	/* as Nokia's: a conversion error just flushes */
	if (!kstrtol(strim(received), 10, &val) && val > 0)
		flush_period = min_t(long, val, MAX_FLUSH_VALUE);
	flush_keepalives(false);
	return len;
}

/* Acknowledges the poll event */
static ssize_t iphbd_read(struct file *file, char __user *buf, size_t len, loff_t *off)
{
	trigger_poll = false;
	return 0;
}

/* A TCP keepalive: an ACK of seq snd_nxt - 1 with no or one byte of data */
static bool is_keepalive(struct sk_buff *skb, u8 pf)
{
	struct sock *sk = skb->sk;
	unsigned int len;
	struct tcphdr *tcp;

	if (!sk || !sk_fullsock(sk) || sk->sk_protocol != IPPROTO_TCP)
		return false;

	if (pf == NFPROTO_IPV4) {
		if (ip_hdr(skb)->protocol != IPPROTO_TCP)
			return false;
		len = ntohs(ip_hdr(skb)->tot_len) - ip_hdrlen(skb);
	} else {
		if (ipv6_hdr(skb)->nexthdr != IPPROTO_TCP)
			return false;
		len = ntohs(ipv6_hdr(skb)->payload_len);
	}

	tcp = tcp_hdr(skb);
	len -= tcp->doff << 2;
	return tcp->ack && len <= 1 && !(tcp->syn || tcp->fin || tcp->rst) &&
	       ntohl(tcp->seq) == tcp_sk(sk)->snd_nxt - 1;
}

static unsigned int net_out_hook(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	struct keepalive *k;

	if (!iphb_open)
		return NF_ACCEPT;

	if (num_queued >= MAX_KEEPALIVES || !is_keepalive(skb, state->pf)) {
		flush_keepalives(true);
		return NF_ACCEPT;
	}

	k = kzalloc(sizeof(*k), GFP_ATOMIC);
	if (!k)
		return NF_ACCEPT;
	k->skb = skb;
	k->state = *state;
	if (k->state.sk)
		sock_hold(k->state.sk);
	dev_hold(k->state.out);

	spin_lock_bh(&lock);
	list_add_tail(&k->list, &queued);
	num_queued++;
	spin_unlock_bh(&lock);

	if (!timer_pending(&flush_timer))
		mod_timer(&flush_timer, round_jiffies(jiffies + flush_period * HZ));

	return NF_STOLEN;
}

/* Anything coming in means the radio is on: send what's held */
static unsigned int net_in_hook(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	if (iphb_open)
		flush_keepalives(true);
	return NF_ACCEPT;
}

static const struct file_operations iphb_fops = {
	.owner		= THIS_MODULE,
	.write		= iphbd_write,
	.read		= iphbd_read,
	.poll		= iphbd_poll,
	.open		= iphbd_open,
	.release	= iphbd_release,
};

static struct miscdevice iphb_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "iphb",
	.fops	= &iphb_fops,
};

static const struct nf_hook_ops iphb_ops[] = {
	{ .hook = net_out_hook, .pf = NFPROTO_IPV4, .hooknum = NF_INET_POST_ROUTING,
	  .priority = NF_IP_PRI_LAST },
	{ .hook = net_out_hook, .pf = NFPROTO_IPV6, .hooknum = NF_INET_POST_ROUTING,
	  .priority = NF_IP6_PRI_LAST },
	{ .hook = net_in_hook, .pf = NFPROTO_IPV4, .hooknum = NF_INET_PRE_ROUTING,
	  .priority = NF_IP_PRI_FIRST },
	{ .hook = net_in_hook, .pf = NFPROTO_IPV6, .hooknum = NF_INET_PRE_ROUTING,
	  .priority = NF_IP6_PRI_FIRST },
};

static int __init iphb_init(void)
{
	int r;

	timer_setup(&flush_timer, timed_flush, TIMER_DEFERRABLE);

	r = misc_register(&iphb_misc);
	if (r)
		return r;

	r = nf_register_net_hooks(&init_net, iphb_ops, ARRAY_SIZE(iphb_ops));
	if (r)
		misc_deregister(&iphb_misc);
	return r;
}

static void __exit iphb_exit(void)
{
	nf_unregister_net_hooks(&init_net, iphb_ops, ARRAY_SIZE(iphb_ops));
	misc_deregister(&iphb_misc);
	iphb_open = false;
	flush_keepalives(false);
	timer_shutdown_sync(&flush_timer);
	flush_work(&send_work);
}

module_init(iphb_init);
module_exit(iphb_exit);

MODULE_AUTHOR("Jukka Rissanen <jukka.rissanen@nokia.com>");
MODULE_DESCRIPTION("netfilter module for delaying TCP keepalive packets");
MODULE_LICENSE("GPL");
