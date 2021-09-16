// SPDX-License-Identifier: GPL-2.0 or Linux-OpenIB
/* Copyright (c) 2015 - 2021 Intel Corporation */

#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <rdma/ib_addr.h>
#include <rdma/ib_smi.h>
#include <rdma/ib_user_verbs.h>
#include <rdma/ib_cache.h>
#include <rdma/ib_umem.h>
//TODO: add kernel patch for below file
// #include <rdma/xib-abi.h>
#include <rdma/ib_verbs.h>
#include <rdma/uverbs_ioctl.h>
#include <net/addrconf.h>
#include <linux/of_address.h>
#include <linux/jiffies.h>
#include "ysn3.h"
#include "main.h"
#include "xib.h"
#include "yusur_roce_device.h"

#define DRV_VER_MAJOR 0
#define DRV_VER_MINOR 0
#define DRV_VER_BUILD 2
#define DRV_VER	__stringify(DRV_VER_MAJOR) "."		\
	__stringify(DRV_VER_MINOR) "." __stringify(DRV_VER_BUILD)

struct xilinx_ib_dev *ibdev;
const char* ifname = "eth0";

unsigned int app_qp_cnt = 10;
module_param_named(max_app_qp, app_qp_cnt, int, 0644);
unsigned int retry_time = 0xD;
module_param_named(rtime, retry_time, int, 0644);

unsigned int app_qp_depth = 16;
module_param_named(max_q_depth, app_qp_depth, int, 0644);

unsigned int max_rq_sge = 16;
module_param(max_rq_sge, int, 0644);


static const struct pci_device_id yusur_roce_hw_pci_tbl[] = {
	{PCI_DEVICE(VENDOR_ID, DEVICE_ID), 0},
	/* required last entry */
	{0, }
};

static int yusur_roce_hw_init_instance(struct ysn3_handle *handle)
{
	int ret;
	const struct pci_device_id *id;

	pr_info("%s: %d.%d.%d\n", __func__,DRV_VER_MAJOR,
		DRV_VER_MINOR, DRV_VER_BUILD);

	handle->rinfo.instance_state = YUSUR_ROCE_STATE_INIT;

	id = pci_match_id(yusur_roce_hw_pci_tbl, handle->pdev);
	if (!id) {
		dev_warn(ibdev->dev, "%s: <---------- \n", __func__);
		return 0;
	}

	dev_dbg(&handle->pdev->dev, "%s : <---------- \n", __func__);

	ibdev = (struct xilinx_ib_dev *)ib_alloc_device(xilinx_ib_dev, ib_dev);
	if (!ibdev)
		return -ENOMEM;

	ibdev->pci_dev = handle->pdev;
	ibdev->dev = &handle->pdev->dev;
	handle->priv = ibdev;

	pr_info("%s: alloc ib device done\n", __func__);

	ret = yusur_roce_init(ibdev);
	if (ret) {
		dev_err(ibdev->dev, "Yusur RoCE Engine HW init failed!\n");
		handle->rinfo.instance_state = YUSUR_ROCE_STATE_NON_INIT;
		goto error_failed_kzalloc;
	}

	handle->rinfo.instance_state = YUSUR_ROCE_STATE_INITED;

	return 0;

//TODO:
// error_failed_get_cfg:
// 	kfree(hr_dev->priv);

error_failed_kzalloc:
	ib_dealloc_device(&ibdev->ib_dev);

	return ret;
}

static void yusur_roce_hw_uninit_instance(struct ysn3_handle *handle,
					   bool reset)
{
	struct xilinx_ib_dev *ibdev = (struct xilinx_ib_dev *)handle->priv;

	pr_info("yusur_roce_hw_uninit_instance: %d.%d.%d\n", DRV_VER_MAJOR,
		DRV_VER_MINOR, DRV_VER_BUILD);

	dev_dbg(&handle->pdev->dev, "%s : <---------- \n", __func__);

	if (!ibdev)
		return;

	yusur_roce_exit(ibdev);

	//TODO:
	// kfree(ibdev->priv);
	ib_dealloc_device(&ibdev->ib_dev);
	pr_info("%s: dealloc ib device done\n", __func__);
}

static int yusur_roce_hw_reset_notify(struct ysn3_handle *handle,
				       enum ysn3_reset_notify_type type)
{
	pr_info("yusur_roce_hw_reset_notify, reset type: %d\n", type);
	return 0;
}

static const struct ysn3_client_ops ysn3_roce_hw_ops = {
	.init_instance = yusur_roce_hw_init_instance,
	.uninit_instance = yusur_roce_hw_uninit_instance,
	.reset_notify = yusur_roce_hw_reset_notify,
};

static struct ysn3_client yusur_roce_hw_client = {
	.name = "yusur_roce_hw",
	.type = YSN3_CLIENT_ROCE2,
	.ops = &ysn3_roce_hw_ops,
};

static int __init yrdma_init_module(void)
{
	pr_info("%s: yrdma driver version: %d.%d.%d\n", __func__, DRV_VER_MAJOR,
		DRV_VER_MINOR, DRV_VER_BUILD);

	return ysn3_register_client(&yusur_roce_hw_client);
}

static void __exit yrdma_exit_module(void)
{
	ysn3_unregister_client(&yusur_roce_hw_client);
}

MODULE_ALIAS("yrdma");
MODULE_AUTHOR("Wentao Cui <cuiwt@yusur.tech>");
MODULE_DESCRIPTION("Yusur RoCE Driver for RDMA");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRV_VER);

module_init(yrdma_init_module);
module_exit(yrdma_exit_module);
