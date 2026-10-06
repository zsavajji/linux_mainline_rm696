// SPDX-License-Identifier: GPL-2.0-only OR GPL-3.0-only
/*
 *	"condition" match extension for Xtables
 *
 *	Description: This module allows firewall rules to match using
 *	condition variables available through procfs.  It also allows
 *	target rules to set the condition variable.
 *
 *	Authors:
 *	Stephane Ouellette <ouellettes [at] videotron ca>, 2002-10-22
 *	Massimiliano Hofer <max [at] nucleus it>, 2006-05-15
 *	Luciano Coelho <luciano.coelho@nokia.com>, 2010-08-11
 *
 *	This program is free software; you can redistribute it and/or modify it
 *	under the terms of the GNU General Public License; either version 2
 *	or 3 of the License, as published by the Free Software Foundation.
 *
 *	Portion Copyright 2010 Nokia Corporation and/or its subsidiary(-ies).
 *	File modified on 2010-08-11.
 *
 * Nokia's N9 kernel's version (the values and the CONDITION target are Nokia's), which Harmattan's
 * iptables extensions and wlancond's throughput rules (/etc/network/wlan_iptables.sh) use.
 * ponytail: the variables live in the initial namespace's /proc/net/nf_condition, as Nokia's rules
 * used them from any namespace; per-namespace lists if containers ever need them.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/capability.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_condition.h>
#include <net/net_namespace.h>

MODULE_AUTHOR("Stephane Ouellette <ouellettes@videotron.ca>");
MODULE_AUTHOR("Massimiliano Hofer <max@nucleus.it>");
MODULE_AUTHOR("Jan Engelhardt <jengelh@medozas.de>");
MODULE_AUTHOR("Luciano Coelho <luciano.coelho@nokia.com>");
MODULE_DESCRIPTION("Allows rules to set and match condition variables");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ipt_condition");
MODULE_ALIAS("ip6t_condition");

struct condition_variable {
	struct list_head list;
	unsigned int refcount;
	u32 value;
	char name[XT_CONDITION_MAX_NAME_SIZE];
};

static LIST_HEAD(conditions);
static DEFINE_MUTEX(conditions_lock);	/* the list and its proc files */
static struct proc_dir_entry *proc_dir;

static int condition_proc_show(struct seq_file *m, void *v)
{
	const struct condition_variable *var = m->private;

	seq_printf(m, "%u\n", READ_ONCE(var->value));
	return 0;
}

static int condition_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, condition_proc_show, pde_data(inode));
}

static ssize_t condition_proc_write(struct file *file, const char __user *input,
				    size_t length, loff_t *ppos)
{
	struct condition_variable *var = pde_data(file_inode(file));
	u32 value;
	int ret;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	ret = kstrtou32_from_user(input, length, 0, &value);
	if (ret)
		return ret;

	WRITE_ONCE(var->value, value);
	return length;
}

static const struct proc_ops condition_proc_ops = {
	.proc_open	= condition_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= condition_proc_write,
};

/* The variable named, created at its first rule */
static struct condition_variable *condition_get(const char *name)
{
	struct condition_variable *var;

	/* Forbid certain names */
	if (*name == '\0' || *name == '.' || name[XT_CONDITION_MAX_NAME_SIZE - 1] != '\0' ||
	    memchr(name, '/', XT_CONDITION_MAX_NAME_SIZE)) {
		pr_info("name not allowed or too long: \"%.*s\"\n",
			XT_CONDITION_MAX_NAME_SIZE, name);
		return ERR_PTR(-EINVAL);
	}

	guard(mutex)(&conditions_lock);
	list_for_each_entry(var, &conditions, list) {
		if (strcmp(name, var->name) == 0) {
			var->refcount++;
			return var;
		}
	}

	var = kzalloc(sizeof(*var), GFP_KERNEL);
	if (!var)
		return ERR_PTR(-ENOMEM);

	strscpy(var->name, name);
	if (!proc_create_data(name, 0644, proc_dir, &condition_proc_ops, var)) {
		kfree(var);
		return ERR_PTR(-ENOMEM);
	}
	var->refcount = 1;
	list_add(&var->list, &conditions);
	return var;
}

static void condition_put(struct condition_variable *var)
{
	guard(mutex)(&conditions_lock);
	if (--var->refcount)
		return;

	list_del(&var->list);
	remove_proc_entry(var->name, proc_dir);
	kfree(var);
}

static bool condition_mt(const struct sk_buff *skb, struct xt_action_param *par)
{
	const struct xt_condition_mtinfo *info = par->matchinfo;
	const struct condition_variable *var = info->condvar;

	return (READ_ONCE(var->value) == info->value) ^ info->invert;
}

static int condition_mt_check(const struct xt_mtchk_param *par)
{
	struct xt_condition_mtinfo *info = par->matchinfo;
	struct condition_variable *var = condition_get(info->name);

	if (IS_ERR(var))
		return PTR_ERR(var);

	info->condvar = var;
	return 0;
}

static void condition_mt_destroy(const struct xt_mtdtor_param *par)
{
	const struct xt_condition_mtinfo *info = par->matchinfo;

	condition_put(info->condvar);
}

static unsigned int condition_tg(struct sk_buff *skb, const struct xt_action_param *par)
{
	const struct condition_tginfo *info = par->targinfo;
	struct condition_variable *var = info->condvar;

	WRITE_ONCE(var->value, info->value);
	return XT_CONTINUE;
}

static int condition_tg_check(const struct xt_tgchk_param *par)
{
	struct condition_tginfo *info = par->targinfo;
	struct condition_variable *var = condition_get(info->name);

	if (IS_ERR(var))
		return PTR_ERR(var);

	info->condvar = var;
	return 0;
}

static void condition_tg_destroy(const struct xt_tgdtor_param *par)
{
	const struct condition_tginfo *info = par->targinfo;

	condition_put(info->condvar);
}

static struct xt_match condition_mt_reg __read_mostly = {
	.name		= "condition",
	.revision	= 2,
	.family		= NFPROTO_UNSPEC,
	.match		= condition_mt,
	.checkentry	= condition_mt_check,
	.destroy	= condition_mt_destroy,
	.matchsize	= sizeof(struct xt_condition_mtinfo),
	.usersize	= offsetof(struct xt_condition_mtinfo, condvar),
	.me		= THIS_MODULE,
};

static struct xt_target condition_tg_reg __read_mostly = {
	.name		= "CONDITION",
	.revision	= 0,
	.family		= NFPROTO_UNSPEC,
	.target		= condition_tg,
	.checkentry	= condition_tg_check,
	.destroy	= condition_tg_destroy,
	.targetsize	= sizeof(struct condition_tginfo),
	.usersize	= offsetof(struct condition_tginfo, condvar),
	.me		= THIS_MODULE,
};

static int __init condition_init(void)
{
	int ret;

	proc_dir = proc_mkdir("nf_condition", init_net.proc_net);
	if (!proc_dir)
		return -ENOMEM;

	ret = xt_register_match(&condition_mt_reg);
	if (ret)
		goto err_dir;

	ret = xt_register_target(&condition_tg_reg);
	if (ret)
		goto err_match;

	return 0;

err_match:
	xt_unregister_match(&condition_mt_reg);
err_dir:
	remove_proc_entry("nf_condition", init_net.proc_net);
	return ret;
}

static void __exit condition_exit(void)
{
	xt_unregister_target(&condition_tg_reg);
	xt_unregister_match(&condition_mt_reg);
	remove_proc_entry("nf_condition", init_net.proc_net);
}

module_init(condition_init);
module_exit(condition_exit);
