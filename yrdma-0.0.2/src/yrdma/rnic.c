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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <asm/byteorder.h>
#include <rdma/ib_addr.h>
#include <rdma/ib_smi.h>
#include <rdma/ib_user_verbs.h>
#include <rdma/ib_cache.h>
#include "rnic.h"
#include "xib.h"
#include "ib_verbs.h"
//TODO:check...
//#include "xib_kmm_export.h"

#define DEBUG

void config_raw_ip(struct xrnic_local *xl, u32 base, u32 *ip, bool is_ipv6)
{
	u32 val = 0, i;

	if (!is_ipv6) {
		val = cpu_to_be32(*ip);
		xrnic_iow(xl, base, val);
	} else {
		for (i = 0; i < 4; i++) {
			val = cpu_to_be32(ip[i]);
			xrnic_iow(xl, base + (3 - i) * 4, val);
		}
	}
}

void xrnic_set_mac(struct xrnic_local *xl, u8 *mac)
{
	u32 val;

	val = mac[5] | (mac[4] << 8) |
		(mac[3] << 16) | (mac[2] << 24);

	xrnic_iow(xl, XRNIC_MAC_ADDR_LO, val);

  	wmb();
  	val =  mac[1] | (mac[0] << 8);
	xrnic_iow(xl, XRNIC_MAC_ADDR_HI, val);
}

/*
 *
 */
int xrnic_start(struct xrnic_local *xl)
{
	u32 val;

	val = ( XRNIC_EN | (1 << 5) |
		((xl->qps_enabled & XRNIC_NUM_QP_MASK) << XRNIC_NUM_QP_SHIFT) |
		((xl->udp_sport & XRNIC_UDP_SPORT_MASK) <<
		XRNIC_UDP_SPORT_SHIFT) );
	xrnic_iow(xl, XRNIC_CONF, val);

	return 0;
}

/*
 *
 */
dma_addr_t xrnic_buf_alloc(struct xrnic_local *xl, u32 size, u32 count)
{
	int order;
	void *buf;
	dma_addr_t addr;

	order = get_order(PAGE_ALIGN(size * count));
	buf = (void *)__get_free_pages(GFP_KERNEL, order);
	if (!buf) {
		dev_err(&xl->pdev->dev, "failed to alloc xrnic buffers, order\
				:%d\n", order);
		return 0;
	}

	addr = dma_map_single(&xl->pdev->dev, buf,
				PAGE_ALIGN(size * count),
				DMA_FROM_DEVICE);

	if (dma_mapping_error(&xl->pdev->dev, addr)) {
		dev_err(&xl->pdev->dev, "failed to alloc xrnic buffers, order\
				:%d\n", order);
		return 0;
	}
	return addr;
}

/*
 *
 */
static void xrnic_set_bufs(struct platform_device *pdev, struct xrnic_local *xl)
{
	dma_addr_t addr;
	u32 val;

	printk("enter xrnic_set_bufs\n");

	addr = xrnic_buf_alloc(xl, XRNIC_SIZE_OF_ERROR_BUF, XRNIC_NUM_OF_ERROR_BUF);
	if (!addr) {
		dev_err(&pdev->dev, "xrnic_set_bufs: Failed to allocate err bufs\n");
		return;
	}
	xrnic_iow(xl, XRNIC_ERR_BUF_BASE_LSB, addr);
	xrnic_iow(xl, XRNIC_ERR_BUF_BASE_MSB, UPPER_32_BITS(addr));
	wmb();
	val = XRNIC_NUM_OF_ERROR_BUF | ( XRNIC_SIZE_OF_ERROR_BUF << 16);
	xrnic_iow(xl, XRNIC_ERR_BUF_SZ, val);
	wmb();

	addr = xrnic_buf_alloc(xl, PAGE_SIZE, 4);
	if (!addr) {
		dev_err(&pdev->dev, "xrnic_set_bufs: Failed to allocate incg pkt err bufs\n");
		return;
	}
	xrnic_iow(xl, XRNIC_INCG_PKT_ERRQ_BASE_LSB, addr);
	xrnic_iow(xl, XRNIC_INCG_PKT_ERRQ_BASE_MSB, UPPER_32_BITS(addr));
	wmb();
	xrnic_iow(xl, XRNIC_INCG_PKT_ERRQ_SZ, XRNIC_IN_ERRST_Q_NUM_ENTRIES);
	wmb();

	addr = xrnic_buf_alloc(xl, XRNIC_RESP_ERR_BUF_SIZE, XRNIC_RESP_ERR_BUF_DEPTH);
	xrnic_iow(xl, XRNIC_RSP_ERR_BUF_BA_LSB, addr);
	xrnic_iow(xl, XRNIC_RSP_ERR_BUF_BA_MSB, UPPER_32_BITS(addr));
	wmb();
	xrnic_iow(xl, XRNIC_RSP_ERR_BUF_DEPTH, (XRNIC_RESP_ERR_BUF_DEPTH << 16 | XRNIC_RESP_ERR_BUF_SIZE));
	wmb();
}

/*
 *
 */
struct xrnic_local *xrnic_hw_init(struct platform_device *pdev, struct xilinx_ib_dev *xib)
{
	struct xrnic_local *xl;
	struct resource *res;
	u32 *db_buf;
	u64 db_buf_pa;
	// int err;

	pr_info("enter xrnic_hw_init\n");

	xl = kzalloc(sizeof(*xl), GFP_KERNEL);
	if (!xl) {
		dev_err(&pdev->dev, "memory alloc failed\n");
		return NULL;
	}

	xl->pdev = pdev;
	xl->retry_buf_va = NULL;
	xl->in_pkt_err_va = NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "memory resource not found\n");
		goto fail;
	}
	xl->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(xl->reg_base)) {
		dev_err(&pdev->dev, "cant ioreamp resource\n");
		goto fail;
	}

	/* store pa of reg for db access */
	xl->db_pa = (res->start + 0x20000);
	xl->db_size = (res->end - (res->start + 0x20000) + 1);

	// dev_dbg(&pdev->dev, "xl->reg_base: %lx\n", *(xl->reg_base));

	xl->irq = platform_get_irq_byname(pdev, "xrnic_intr0");
	if (xl->irq <= 0) {
		dev_err(&pdev->dev, "platform get of irq failed!\n");
		goto fail;
	}

	/* set default hw config */
	xrnic_set_bufs(pdev, xl);

	/* enable all interrupts */
	xrnic_iow(xl, XRNIC_INT_EN, 0xff);

	/* allocate memory for rq and sq doorbells for all QPs */
	/* allocate 4 * qpn */
	db_buf = xib_alloc_coherent("pl", xl->xib,
				PAGE_SIZE, (dma_addr_t *)&db_buf_pa,
				GFP_KERNEL);

	if (!db_buf) {
		printk("failed to alloc db mem %s:%d\n",
							__func__, __LINE__);
		goto fail;
	}

	xl->qp1_rq_db_p = db_buf_pa;
	xl->qp1_rq_db_v = db_buf;

	db_buf = dma_alloc_coherent(&pdev->dev, PAGE_SIZE, (dma_addr_t *)&db_buf_pa,
			GFP_KERNEL);
	if (!db_buf) {
		dev_err(&pdev->dev, "failed to alloc db mem\n");
		goto fail;
	}

	/* 512 bytes for all DBs for all QPs? TODO */
	xl->qp1_sq_db_v = db_buf;
	xl->qp1_sq_db_p = db_buf_pa;

	return xl;
fail:
	kfree(xl);
	return NULL;
}

/*
 *
 */
void xrnic_hw_deinit(struct xilinx_ib_dev *xib)
{
	struct xrnic_local *xl = xib->xl;

	xib_free_coherent(xib, PAGE_SIZE, xl->qp1_sq_db_v,
			xl->qp1_sq_db_p);
	xib_free_coherent(xib, PAGE_SIZE, xl->qp1_rq_db_v,
			xl->qp1_rq_db_p);
	free_irq(xl->irq, xib);
	kfree(xl);
}

#define DEBUG