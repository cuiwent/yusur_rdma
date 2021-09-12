// SPDX-License-Identifier: GPL-2.0 or Linux-OpenIB
/* Copyright (c) 2015 - 2021 Intel Corporation */

#include "ysn3.h"
#include "main.h"
#include "xib.h"

#define DRV_VER_MAJOR 0
#define DRV_VER_MINOR 0
#define DRV_VER_BUILD 1
#define DRV_VER	__stringify(DRV_VER_MAJOR) "."		\
	__stringify(DRV_VER_MINOR) "." __stringify(DRV_VER_BUILD)

/****************************************************************************************************************/



// static struct notifier_block yrdma_inetaddr_notifier = {
// 	.notifier_call = yrdma_inetaddr_event
// };

// static struct notifier_block yrdma_net_notifier = {
// 	.notifier_call = yrdma_net_event
// };

// static struct notifier_block yrdma_netdevice_notifier = {
// 	.notifier_call = yrdma_netdevice_event
// };

// static void yrdma_register_notifiers(void)
// {
// 	register_inetaddr_notifier(&yrdma_inetaddr_notifier);
// 	register_netevent_notifier(&yrdma_net_notifier);
// 	register_netdevice_notifier(&yrdma_netdevice_notifier);
// }

// static void yrdma_unregister_notifiers(void)
// {
// 	unregister_netevent_notifier(&yrdma_net_notifier);
// 	unregister_inetaddr_notifier(&yrdma_inetaddr_notifier);
// 	unregister_netdevice_notifier(&yrdma_netdevice_notifier);
// }

static int yusur_roce_init(struct xilinx_ib_dev *ibdev)
{
	// int err;
	// struct device_node *net_node;
	// struct device_node *np;
	struct xrnic_local *xl;
	struct pci_dev *pci_dev;
	// unsigned long debug_phys;
	// struct net_device *netdev;
	// struct resource resource;ß
	// u32 qpn, rtr_count;
	// u64 rtr_addr = 0;

	if(ibdev->pci_dev) {
		dev_dbg(ibdev->dev, "%s: <---------- \n", __func__);
		pci_dev = ibdev->pci_dev;
		ibdev->mtu = QP_PMTU_4096;
		//TODO: check driver data
		// platform_set_drvdata(pdev, ibdev);

		xl = xrnic_hw_init(pci_dev, ibdev);
		if (!xl) {
			dev_err(ibdev->dev, "xrnic init failed\n");
			return -ENODEV;
		}
		ibdev->xl = xl;
		xl->xib = ibdev;

	} else {
		pr_info("%s: pci_dev is NULL!!!", __func__);
		return -1;
	}

	return 0;
}

static void yusur_roce_exit(struct xilinx_ib_dev *ib_dev)
{

}

static int yusur_roce_hw_init_instance(struct ysn3_handle *handle)
{
	struct xilinx_ib_dev *ibdev;
	int ret;

	pr_info("%s: %d.%d.%d\n", __func__,DRV_VER_MAJOR,
		DRV_VER_MINOR, DRV_VER_BUILD);

	if(&handle->pdev->dev)
		dev_dbg(&handle->pdev->dev, "%s : <---------- \n", __func__);
	//TODO:
	//else
	//	no pci device handle

	// ibdev = (struct xilinx_ib_dev *)ib_alloc_device(sizeof(*ibdev));
	ibdev = (struct xilinx_ib_dev *)ib_alloc_device(xilinx_ib_dev, ib_dev);
	if (!ibdev)
		return -ENOMEM;

	ibdev->pci_dev = handle->pdev;
	ibdev->dev = &handle->pdev->dev;
	handle->priv = ibdev;

	pr_info("%s: alloc ib device done\n", __func__);

	ret = yusur_roce_init(ibdev);
	if (ret) {
		dev_err(ibdev->dev, "Yusur RoCE Engine init failed!\n");
		goto error_failed_kzalloc;
	}

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

	if(&handle->pdev->dev)
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
