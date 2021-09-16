/* SPDX-License-Identifier: GPL-2.0 or Linux-OpenIB */
/* Copyright (c) 2015 - 2021 Intel Corporation */
#ifndef YRDMA_MAIN_H
#define YRDMA_MAIN_H

#define DEBUG

#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/if_vlan.h>
#include <net/addrconf.h>
#include <net/netevent.h>
#include <net/tcp.h>
#include <net/ip6_route.h>
#include <net/flow.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/crc32c.h>
#include <linux/kthread.h>
#ifndef CONFIG_64BIT
#include <linux/io-64-nonatomic-lo-hi.h>
#endif
#include <linux/platform_device.h>
#ifndef RDMA_MMAP_DB_SUPPORT
#include <linux/random.h>
#endif
#include <linux/configfs.h>
#ifdef __OFED_4_8__
#include <linux/hashtable.h>
#endif /* __OFED_4_8__ */
#include <crypto/hash.h>
#include <rdma/ib_smi.h>
#include <rdma/ib_verbs.h>
#include <rdma/ib_pack.h>
#include <rdma/rdma_cm.h>
#include <rdma/iw_cm.h>
#include <rdma/ib_user_verbs.h>
#include <rdma/ib_umem.h>
#include <rdma/ib_cache.h>
#include "status.h"
#include "xib.h"

#define YRDMA_SN_PDEV_NAME	"rnicpci"

int yusur_roce_init(struct xilinx_ib_dev *ibdev);
void yusur_roce_exit(struct xilinx_ib_dev *ib_dev);
void xib_set_dev_caps(struct ib_device *ibdev);
int update_mtu(struct net_device *dev);
int set_ip_address(struct net_device *dev, u32 is_ipv4);
#endif /* YRDMA_MAIN_H */
