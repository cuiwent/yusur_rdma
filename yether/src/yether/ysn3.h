/* SPDX-License-Identifier: GPL-2.0+ */
// Copyright (c) 2016-2017 Hisilicon Limited.
// Copyright (c) 2018-2021 Yusur Tech.

#ifndef __YSN3_H
#define __YSN3_H

#include <linux/acpi.h>
#include <linux/dcbnl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/types.h>

/* Device IDs */
#define VENDOR_ID 0x10EE  // Xilinx Vendor ID
#define DEVICE_ID 0x903f

enum ysn3_client_type {
	YSN3_CLIENT_ROCE2,
};

enum ysn3_reset_type {
	YSN3_FUNC_RESET,
};

enum ysn3_reset_notify_type {
	YSN3_UP_CLIENT,
	YSN3_DOWN_CLIENT,
	YSN3_INIT_CLIENT,
	YSN3_UNINIT_CLIENT,
};

struct ysn3_roce_private_info {
	struct net_device *netdev;
	void __iomem *roce_io_base;
	int base_vector;
	int num_vectors;
	/* The below attributes defined for RoCE client
	 * initial values to them, and RoCE client can modify and use
	 * them.
	 */
	unsigned long reset_state;
	unsigned long instance_state;
	unsigned long state;
};

struct ysn3_handle;

struct ysn3_client_ops {
	int (*init_instance)(struct ysn3_handle *handle);
	void (*uninit_instance)(struct ysn3_handle *handle, bool reset);
	void (*link_status_change)(struct ysn3_handle *handle, bool state);
	int (*setup_tc)(struct ysn3_handle *handle, u8 tc);
	int (*reset_notify)(struct ysn3_handle *handle,
			    enum ysn3_reset_notify_type type);
};

#define YSN3_CLIENT_NAME_LENGTH 16
struct ysn3_client {
	char name[YSN3_CLIENT_NAME_LENGTH];
	unsigned long state;
	enum ysn3_client_type type;
	const struct ysn3_client_ops *ops;
	struct list_head node;
};

struct ysn3_handle {
	struct ysn3_client *client;
	struct pci_dev *pdev;
	void *priv;

	unsigned long last_reset_time;
	enum ysn3_reset_type reset_level;

	union {
		struct net_device *netdev; /* first member */
		struct ysn3_roce_private_info rinfo;
	};
};

int ysn3_register_client(struct ysn3_client *client);
void ysn3_unregister_client(struct ysn3_client *client);
#endif
