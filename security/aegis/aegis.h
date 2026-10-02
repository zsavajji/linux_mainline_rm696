/*
 * n9-qemu: shared by the Aegis components ported to Linux 6.x.
 *
 * On Linux 2.6.32 Aegis was the only security module and used the
 * cred/inode/file/super_block ->security pointers directly. Today the LSM
 * framework shares those blobs between modules; these return Aegis's slot.
 */
#ifndef _SECURITY_AEGIS_AEGIS_H
#define _SECURITY_AEGIS_AEGIS_H

#include <linux/lsm_hooks.h>
#include <linux/cred.h>
#include <linux/fs.h>

extern struct lsm_blob_sizes aegis_blob_sizes;

/* credp: trust-zone pointer, low bit = "mark" flag */
static inline unsigned long *aegis_cred(const struct cred *cred)
{
	return cred->security + aegis_blob_sizes.lbs_cred;
}

/* validator: count of exec mappings holding off writers */
static inline unsigned long *aegis_inode(const struct inode *inode)
{
	return inode->i_security + aegis_blob_sizes.lbs_inode;
}

/* validator: 1 while this file holds an exec mapping count */
static inline unsigned long *aegis_file(const struct file *file)
{
	return file->f_security + aegis_blob_sizes.lbs_file;
}

/* validator: the volume's reference hashlist */
static inline void **aegis_sb(const struct super_block *sb)
{
	return sb->s_security + aegis_blob_sizes.lbs_superblock;
}

#endif
