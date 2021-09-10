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
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/list.h>
#include "xib.h"

static struct device *xib_get_alloc_dev(char *from, struct xilinx_ib_dev *xib)
{
	struct device *dev;

	dev = &xib->pdev->dev;
	return dev;
}

void *xib_alloc_coherent(char *from, void *xib_dev,
		size_t len, u64 *dma_handle, gfp_t flags)
{
	struct device *dev;

	struct xilinx_ib_dev *xib = (struct xilinx_ib_dev *)xib_dev;

	dev = xib_get_alloc_dev(from, xib);
	if (dev)
		return dma_alloc_coherent(dev, len, (dma_addr_t *)dma_handle, flags);
	else
		return NULL;
}

void xib_free_coherent(void *xib_dev,
		 u64 size, void *cpu_addr, u64 dma_handle)
{
	struct device *dev;

	struct xilinx_ib_dev *xib = (struct xilinx_ib_dev *)xib_dev;

	dev = xib_get_alloc_dev("pl", xib);
	if (dev)
		dma_free_coherent(dev, size, cpu_addr, dma_handle);
}