/*
 * This file is part of Aegis Validator
 *
 * Copyright (C) 2003 Ericsson, Inc
 * Copyright (C) 2008-2010 Nokia Corporation and/or its subsidiary(-ies).
 *
 * Contact: Markku Kylänpää <ext-markku.kylanpaa@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Authors: Axelle Apvrille, 2003
 *          David Gordon, 2003
 *          Makan Pourzandi, 2003
 *          Chris Wright, 2004
 *          Markku Kylänpää, 2008-2010
 */

/*
 * SHA1 measurement of files for Aegis Validator.
 *
 * n9-qemu: Linux 6.x has a synchronous SHA-1 library, so this replaces
 * Nokia's asynchronous crypto API code (double-buffered ahash requests).
 * The results, and therefore the validation decisions, are the same.
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <crypto/sha1.h>
#include "validator.h"
#include "verify.h"

/* Read buffer size (as Nokia's first buffer) */
#define VERIFY_BUFFER_SIZE (8 * PAGE_SIZE)

/**
 * validator_verify_refhash() - Compare SHA1 hash of a file to a reference
 * @file:    File to be measured
 * @refhash: Reference SHA1 hash
 *
 * Return zero if the hash matches, -EFAULT if it does not, and another
 * negative value (e.g. -EINTR) if the file could not be read.
 */
int validator_verify_refhash(struct file *file, char *refhash)
{
	struct sha1_ctx ctx;
	u8 digest[SHA1_DIGEST_SIZE];
	loff_t pos = 0;
	loff_t size = i_size_read(file_inode(file));
	char *buffer = kmalloc(VERIFY_BUFFER_SIZE, GFP_KERNEL);
	ssize_t n;

	if (!buffer) {
		pr_err("Aegis: cannot get free page\n");
		return -ENOMEM;
	}
	sha1_init(&ctx);
	while (pos < size) {
		n = kernel_read(file, buffer,
				min_t(loff_t, size - pos, VERIFY_BUFFER_SIZE), &pos);
		if (n <= 0) {
			pr_err("Aegis: read error during measurement (%zd %s)\n",
			       n, current->comm);
			kfree(buffer);
			return n < 0 ? n : -EFAULT;
		}
		sha1_update(&ctx, buffer, n);
	}
	sha1_final(&ctx, digest);
	kfree(buffer);
	return memcmp(digest, refhash, SHA1_HASH_LENGTH) ? -EFAULT : 0;
}

/**
 * validator_sha1() - Calculate SHA1 hash of a memory buffer
 * @vbuf:   Buffer
 * @len:    Buffer length
 * @digest: Result (SHA1_HASH_LENGTH bytes)
 *
 * Return zero.
 */
int validator_sha1(const void *vbuf, unsigned long len, unsigned char *digest)
{
	sha1(vbuf, len, digest);
	return 0;
}
