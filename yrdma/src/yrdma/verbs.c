/*
 * Xilinx FPGA Xilinx ERNIC Infiniband Driver
 *
 * Copyright (c) 2019 Xilinx Pvt., Ltd
 *
 * Author: Syed S <syeds@xilinx.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/of_platform.h>
#include <linux/module.h>
#include <rdma/ib_umem.h>
#include <rdma/ib_cache.h>
#include <rdma/ib_pack.h>
#include <rdma/ib_mad.h>
#include <linux/etherdevice.h>
//TODO:
// #include <rdma/xib-abi.h>
#include <asm/page.h>
#include "xib.h"
#include "ib_verbs.h"

/* In existing code, instead of using the user space buffers
 * for RDMA_READ, the code uses SGL memory configured when create qp is
 * called, and thus the data rx in READ response is being filled in the
 * SGL memory but not in the user space buffer directly. The fix, that's
 * being made here works only for 32Bit machines */
#define XIB_SGL_FIX 1
#define IB_QP_CREATE_HW_OFLD IB_QP_CREATE_RESERVED_START

int xib_bmap_alloc(struct xib_bmap *bmap, u32 max_count, char *name)
{
	unsigned long *bitmap;

	bitmap = kcalloc(BITS_TO_LONGS(max_count), sizeof(long),
			GFP_KERNEL);
	if(!bitmap)
		return -ENOMEM;

	bmap->bitmap = bitmap;
	bmap->max_count = max_count;
	snprintf(bmap->name, XIB_MAX_BMAP_NAME, "%s", name);

	return 0;
}

int xib_bmap_alloc_id(struct xib_bmap *bmap, u32 *id_num)
{
	*id_num = find_first_zero_bit(bmap->bitmap, bmap->max_count);
	if (*id_num > bmap->max_count)
		return -EINVAL;

	__set_bit(*id_num, bmap->bitmap);

	return 0;
}

void xib_bmap_release_id(struct xib_bmap *bmap, u32 id_num)
{
	// bool b_acquired;

	test_and_clear_bit(id_num, bmap->bitmap);
#if 0
	if (!b_acquired) {
		dev_err(NULL, "%s bitmap: id %d already released\n",
				bmap->name, id_num);
		return;
	}
#endif
}
