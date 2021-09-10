// SPDX-License-Identifier: GPL-2.0 or Linux-OpenIB
/* Copyright (c) 2018 - 2021 Intel Corporation */
#include "main.h"
#include "xib.h"

const char* ifname = "eth0";
struct xilinx_ib_dev *ibdev;

static int handle_netdev_notifier(struct notifier_block *notifier,
                               unsigned long event, void *ptr);
struct notifier_block cmac_netdev_notifier = {
	.notifier_call = handle_netdev_notifier
};

static int cmac_inetaddr_event(struct notifier_block *notifier,
                               unsigned long event, void *data);
static int cmac_inet6addr_event(struct notifier_block *notifier,
                               unsigned long event, void *data);
struct notifier_block cmac_inetaddr_notifier = {
	.notifier_call = cmac_inetaddr_event
};

struct notifier_block cmac_inet6addr_notifier = {
        .notifier_call = cmac_inet6addr_event
};

// static char rtr_mem[8] = "pl";

// static void xib_get_guid(u8 *dev_addr, u8 *guid)
// {
// 	u8 mac[ETH_ALEN];

// 	/* MAC-48 to EUI-64 mapping */
// 	memcpy(mac, dev_addr, ETH_ALEN);
// 	guid[0] = mac[0] ^ 2;
// 	guid[1] = mac[1];
// 	guid[2] = mac[2];
// 	guid[3] = 0xff;
// 	guid[4] = 0xfe;
// 	guid[5] = mac[3];
// 	guid[6] = mac[4];
// 	guid[7] = mac[5];
// }

int update_mtu(struct net_device *dev)
{
	u32 mtu;

	switch (dev->mtu) {
	case 340:
		mtu = QP_PMTU_256;
		break;
	case 592:
		mtu = QP_PMTU_512;
		break;
	case 1500:
		mtu = QP_PMTU_1024;
		break;
	case 2200:
		mtu = QP_PMTU_2048;
		break;
	case 4200:
		mtu = QP_PMTU_4096;
		break;
	default:
		mtu = QP_PMTU_4096;
		break;
	}

	/* update ib dev structure with mtu */
	pr_debug("Updating MTU to %d\n", mtu);
	ibdev->mtu = mtu;
	return 0;
}

int set_ip_address(struct net_device *dev, u32 is_ipv4)
{
	char ip_addr[16];
	int ret = 0;

	if (!dev) {
		pr_err("Dev is null\n");
		return 0;
	}
	if (is_ipv4) {
		u32 ipv4_addr = 0;
		struct in_device *inet_dev = (struct in_device *)dev->ip_ptr;

		if (!inet_dev) {
			pr_err("inet dev is null\n");
			return -EFAULT;
		}
		if (inet_dev->ifa_list) {
			ipv4_addr = inet_dev->ifa_list->ifa_address;
			if (!ipv4_addr) {
				pr_err("ifa_address not available\n");
				return -EINVAL;
			}

			if (!ibdev)
				return -EFAULT;

			if (!ibdev->xl)
				return -EFAULT;

			config_raw_ip(ibdev->xl, XRNIC_IPV4_ADDR, (u32 *)&ipv4_addr, 0);
			snprintf(ip_addr, 16, "%pI4", &ipv4_addr);
			pr_info("IP address is :%s\n", ip_addr);
		} else {
			pr_info("IP address not available at present\n");
			return -EFAULT;
		}
	} else {
		struct inet6_dev *idev;
		struct inet6_ifaddr *ifp, *tmp;
		u32 i, ip_avail = 0;

		idev = __in6_dev_get(dev);
		if (!idev) {
			pr_err("ipv6 inet device not found\n");
			return -EFAULT;
		}

		list_for_each_entry_safe(ifp, tmp, &idev->addr_list, if_list) {
			pr_info("IP=%pI6, MAC=%pM\n", &ifp->addr, dev->dev_addr);
			for (i = 0; i < 16; i++) {
				pr_info("IP=%x\n", ifp->addr.s6_addr[i]);
				ip_addr[15 - i] = ifp->addr.s6_addr[i];
			}
			ip_avail = 1;
			config_raw_ip(ibdev->xl, XRNIC_IPV6_ADD_1, (u32 *)ifp->addr.s6_addr, 1);
		}

		if (!ip_avail) {
			pr_info("IPv6 address not available at present\n");
			return 0;
		}
	}
	ret = update_mtu(dev);
	return ret;
}

static int handle_netdev_notifier(struct notifier_block *notifier,
				       unsigned long event, void *ptr)
{
	struct net_device *dev;
	int ret = 0;

	dev = netdev_notifier_info_to_dev(ptr);
	if (!dev) {
		pr_err("Failed to get the net device");
		ret = -EINVAL;
		return ret;
	}

	switch (event) {
		case NETDEV_CHANGEADDR:
			pr_info("eth0 mac changed\n");
			xrnic_set_mac(ibdev->xl, dev->dev_addr);
			break;
		case NETDEV_CHANGEMTU:
			pr_info("MTU changed\n");
			update_mtu(dev);
			break;
	}
	return ret;
}


static int handle_inetaddr_notification(struct notifier_block *notifier,
				       unsigned long event, void *data, u32 is_ipv4)
{
	struct in_ifaddr *ifa = data;
	struct net_device *event_netdev;
	struct net_device *dev = __dev_get_by_name(&init_net, ifname);
	int ret = 0;

	if (!ifa) {
		pr_err("ifaddr is NULL\n");
		ret = -EINVAL;
		return ret;
	}

	if (!dev) {
		pr_err("Failed to get the net device");
		ret = -EINVAL;
		return ret;
	}

	if (is_ipv4) {
		event_netdev = ifa->ifa_dev->dev;

		/* Check whether notification is for eth0 or not*/
		if (event_netdev != dev)
			return ret;
	}

	switch (event) {
	case NETDEV_DOWN:
		pr_info("eth0 link down\n");
		break;
	case NETDEV_UP:
		pr_info("eth0 link up\n");
		ret = set_ip_address(dev, is_ipv4);
		break;
	}
	return ret;
}


static int cmac_inetaddr_event(struct notifier_block *notifier,
			       unsigned long event, void *data)
{
	int ret = 0;
	ret = handle_inetaddr_notification(notifier, event, data, 1);
	return ret;
}

static int cmac_inet6addr_event(struct notifier_block *notifier,
			       unsigned long event, void *data)
{
	int ret = 0;
	ret = handle_inetaddr_notification(notifier, event, data, 0);
	return ret;
}

//TODO:
// static ssize_t show_roce_pfc_enable(struct kobject *kobj,
// 		        struct kobj_attribute *attr, char *buf)
// {
// 	if(xrnic_ior(ibdev->xl, XRNIC_PAUSE_CONF) &
// 			(1U << XRNIC_ROCE_PFC_EN_BIT))
// 		return sprintf(buf, "1\n");
// 	else
// 		return sprintf(buf, "0\n");
// }

// static ssize_t show_non_roce_pfc_enable(struct kobject *kobj,
// 		        struct kobj_attribute *attr, char *buf)
// {
// 	if(xrnic_ior(ibdev->xl, XRNIC_PAUSE_CONF) &
// 			(1U << XRNIC_NON_ROCE_PFC_EN_BIT))
// 		return sprintf(buf, "1\n");
// 	else
// 		return sprintf(buf, "0\n");
// }

// static ssize_t store_roce_pfc_enable(struct kobject *kobj,
// 		        struct kobj_attribute *attr,
// 			const char *buf, size_t count)
// {
// 	u32 val, en;
// 	val = xrnic_ior(ibdev->xl, XRNIC_PAUSE_CONF);
// 	en = simple_strtol(buf, NULL, 10);

// 	if (!en) {
// 		val &= ~(1U << XRNIC_ROCE_PFC_EN_BIT);
// 		xrnic_iow(ibdev->xl, XRNIC_PAUSE_CONF, val);
// 	} else if (en == 1) {
// 		val |= (1U << XRNIC_ROCE_PFC_EN_BIT);
// 		xrnic_iow(ibdev->xl, XRNIC_PAUSE_CONF, val);
// 	} else {
// 		pr_err("Error: Write 1 or 0 to enable/disable PFC.\n", val);
// 	}
// 	return count;
// }

// static ssize_t store_non_roce_pfc_enable(struct kobject *kobj,
// 		        struct kobj_attribute *attr,
// 			const char *buf, size_t count)
// {
// 	u32 val, en;
// 	val = xrnic_ior(ibdev->xl, XRNIC_PAUSE_CONF);
// 	en = simple_strtol(buf, NULL, 10);

// 	if (!en) {
// 		val &= ~(1U << XRNIC_NON_ROCE_PFC_EN_BIT);
// 		xrnic_iow(ibdev->xl, XRNIC_PAUSE_CONF, val);
// 	} else if (en == 1) {
// 		val |= (1U << XRNIC_NON_ROCE_PFC_EN_BIT);
// 		xrnic_iow(ibdev->xl, XRNIC_PAUSE_CONF, val);
// 	} else {
// 		pr_err("Error: Write 1 or 0 to enable/disable PFC.\n", val);
// 	}
// 	return count;
// }

// static ssize_t show_roce_pfc_priority(struct kobject *kobj,
// 		        struct kobj_attribute *attr, char *buf)
// {
// 	u32 val;
// 	val = xrnic_ior(ibdev->xl, XRNIC_PAUSE_CONF);

// 	return sprintf(buf, "%u\n", (val >> XRNIC_ROCE_PFC_PRIO_BIT) &
// 				XRNIC_PFC_PRIO_BIT_MASK);
// }

// static ssize_t show_non_roce_pfc_priority(struct kobject *kobj,
// 		        struct kobj_attribute *attr, char *buf)
// {
// 	u32 val;
// 	val = xrnic_ior(ibdev->xl, XRNIC_PAUSE_CONF);

// 	return sprintf(buf, "%u\n", (val >> XRNIC_NON_ROCE_PFC_PRIO_BIT) &
// 				XRNIC_PFC_PRIO_BIT_MASK);
// }

// static ssize_t store_roce_pfc_priority(struct kobject *kobj,
// 			struct kobj_attribute *attr,
// 			const char *buf, size_t count)
// {
// 	u32 val, prio;
// 	val = xrnic_ior(ibdev->xl, XRNIC_PAUSE_CONF);
// 	prio = simple_strtol(buf, NULL, 10);

// 	if (prio >= 0 && prio <= XRNIC_PFC_GLOBAL_PRIOIRTY) {
// 		val &= ~(XRNIC_PFC_PRIO_BIT_MASK << XRNIC_ROCE_PFC_PRIO_BIT);
// 		val |= (prio << XRNIC_ROCE_PFC_PRIO_BIT);
// 		xrnic_iow(ibdev->xl, XRNIC_PAUSE_CONF, val);
// 	} else {
// 		pr_err("Error: Priority value must be 0 to 8.\n");
// 	}
// 	return count;
// }


// static ssize_t store_non_roce_pfc_priority(struct kobject *kobj,
// 			struct kobj_attribute *attr,
// 			const char *buf, size_t count)
// {
// 	u32 val, prio;
// 	val = xrnic_ior(ibdev->xl, XRNIC_PAUSE_CONF);
// 	prio = simple_strtol(buf, NULL, 10);

// 	if (prio >= 0 && prio <= XRNIC_PFC_GLOBAL_PRIOIRTY) {
// 		val &= ~(XRNIC_PFC_PRIO_BIT_MASK << XRNIC_NON_ROCE_PFC_PRIO_BIT);
// 		val |= (prio << XRNIC_NON_ROCE_PFC_PRIO_BIT);
// 		xrnic_iow(ibdev->xl, XRNIC_PAUSE_CONF, val);
// 	} else {
// 		pr_err("Error: Priority value must be 0 to 8.\n");
// 	}
// 	return count;
// }

// static ssize_t show_roce_pfc_xon(struct kobject *kobj,
// 		        struct kobj_attribute *attr, char *buf)
// {
// 	u32 val;
// 	val = xrnic_ior(ibdev->xl, XRNIC_ROCE_PAUSE_OFFSET);
// 	return sprintf(buf, "%u\n", val & 0xFFFF);
// }

// static ssize_t show_non_roce_pfc_xon(struct kobject *kobj,
// 		        struct kobj_attribute *attr, char *buf)
// {
// 	u32 val;
// 	val = xrnic_ior(ibdev->xl, XRNIC_NON_ROCE_PAUSE_OFFSET);
// 	return sprintf(buf, "%u\n", val & 0xFFFF);
// }

// static ssize_t store_roce_pfc_xon(struct kobject *kobj,
// 			struct kobj_attribute *attr,
// 			const char *buf, size_t count)
// {
// 	u32 val, xon;
// 	val = xrnic_ior(ibdev->xl, XRNIC_ROCE_PAUSE_OFFSET);
// 	xon = simple_strtol(buf, NULL, 10);

// 	if (xon >= PFC_XON_XOFF_MIN && xon <= PFC_XON_XOFF_MAX) {
// 		xrnic_iow(ibdev->xl, XRNIC_ROCE_PAUSE_OFFSET, xon | (val & 0xFFFF0000));
// 	} else {
// 		pr_err("Error: XON threshold must be 0 to 512.\n", val);
// 	}
// 	return count;
// }

// static ssize_t store_non_roce_pfc_xon(struct kobject *kobj,
// 			struct kobj_attribute *attr,
// 			const char *buf, size_t count)
// {
// 	u32 val, xon;
// 	val = xrnic_ior(ibdev->xl, XRNIC_NON_ROCE_PAUSE_OFFSET);
// 	xon = simple_strtol(buf, NULL, 10);

// 	if (xon >= PFC_XON_XOFF_MIN && xon <= PFC_XON_XOFF_MAX) {
// 		xrnic_iow(ibdev->xl, XRNIC_NON_ROCE_PAUSE_OFFSET, xon | (val & 0xFFFF0000));
// 	} else {
// 		pr_err("Error: XON threshold must be 0 to 512.\n", val);
// 	}
// 	return count;
// }

// static ssize_t show_non_roce_pfc_xoff(struct kobject *kobj,
// 		        struct kobj_attribute *attr, char *buf)
// {
// 	u32 val;
// 	val = xrnic_ior(ibdev->xl, XRNIC_NON_ROCE_PAUSE_OFFSET);
// 	return sprintf(buf, "%u\n", val >> 16);
// }

// static ssize_t show_roce_pfc_xoff(struct kobject *kobj,
// 		        struct kobj_attribute *attr, char *buf)
// {
// 	u32 val;
// 	val = xrnic_ior(ibdev->xl, XRNIC_ROCE_PAUSE_OFFSET);
// 	return sprintf(buf, "%u\n", val >> 16);
// }

// static ssize_t store_roce_pfc_xoff(struct kobject *kobj,
// 			struct kobj_attribute *attr,
// 			const char *buf, size_t count)
// {
// 	u32 val, xoff;
// 	val = xrnic_ior(ibdev->xl, XRNIC_ROCE_PAUSE_OFFSET);
// 	xoff = simple_strtol(buf, NULL, 10);
// 	if (xoff >= PFC_XON_XOFF_MIN && xoff <= PFC_XON_XOFF_MAX) {
// 		xrnic_iow(ibdev->xl, XRNIC_ROCE_PAUSE_OFFSET, (xoff << 16) | (val & 0xFFFF));
// 	} else {
// 		pr_err("Error: XOFF threshold must be 0 to 512.\n", val);
// 	}
// 	return count;
// }


// static ssize_t store_non_roce_pfc_xoff(struct kobject *kobj,
// 			struct kobj_attribute *attr,
// 			const char *buf, size_t count)
// {
// 	u32 val, xoff;
// 	val = xrnic_ior(ibdev->xl, XRNIC_NON_ROCE_PAUSE_OFFSET);
// 	xoff = simple_strtol(buf, NULL, 10);
// 	if (xoff >= PFC_XON_XOFF_MIN && xoff <= PFC_XON_XOFF_MAX) {
// 		xrnic_iow(ibdev->xl, XRNIC_NON_ROCE_PAUSE_OFFSET, (xoff << 16) | (val & 0xFFFF));
// 	} else {
// 		pr_err("Error: XOFF threshold must be 0 to 512.\n", val);
// 	}
// 	return count;
// }

// static ssize_t show_pfc_priority_check(struct kobject *kobj,
// 		        struct kobj_attribute *attr, char *buf)
// {
// 	u32 val;

// 	val = xrnic_ior(ibdev->xl, XRNIC_PAUSE_CONF);
// 	return sprintf(buf, "%u\n", (val >> XRNIC_DIS_PRIO_CHECK_BIT) & 1);
// }

// static ssize_t store_pfc_priority_check(struct kobject *kobj,
// 			struct kobj_attribute *attr,
// 			const char *buf, size_t count)
// {
// 	u32 val, temp;

// 	val = xrnic_ior(ibdev->xl, XRNIC_PAUSE_CONF);
// 	temp = simple_strtol(buf, NULL, 10);

// 	if (temp == 1) {
// 		val |= (1 << XRNIC_DIS_PRIO_CHECK_BIT);
// 		xrnic_iow(ibdev->xl, XRNIC_PAUSE_CONF, val);
// 	} else if (!temp) {
// 		val &= ~(1 << XRNIC_DIS_PRIO_CHECK_BIT);
// 		xrnic_iow(ibdev->xl, XRNIC_PAUSE_CONF, val);
// 	} else {
// 		pr_err("Error: value must be either a 1 or 0\n");
// 	}
// 	return count;
// }

// static struct kobj_attribute disable_priory_check_attr =
// __ATTR(dis_prioirty_check, 0660, show_pfc_priority_check, store_pfc_priority_check);

// /* Non-RoCE PFC configuration */
// static struct kobj_attribute non_roce_pfc_enable_attr =
// __ATTR(en_non_roce_pfc, 0660, show_non_roce_pfc_enable, store_non_roce_pfc_enable);

// static struct kobj_attribute non_roce_pfc_priority_attr =
// __ATTR(non_roce_pfc_priority, 0660, show_non_roce_pfc_priority, store_non_roce_pfc_priority);

// static struct kobj_attribute non_roce_pfc_xon_attr =
// __ATTR(non_roce_xon_threshold, 0660, show_non_roce_pfc_xon, store_non_roce_pfc_xon);

// static struct kobj_attribute non_roce_pfc_xoff_attr =
// __ATTR(non_roce_xoff_threshold, 0660, show_non_roce_pfc_xoff, store_non_roce_pfc_xoff);


// /* RoCE PFC configuration */
// static struct kobj_attribute roce_pfc_enable_attr =
// __ATTR(en_roce_pfc, 0660, show_roce_pfc_enable, store_roce_pfc_enable);

// static struct kobj_attribute roce_pfc_priority_attr =
// __ATTR(roce_pfc_priority, 0660, show_roce_pfc_priority, store_roce_pfc_priority);

// static struct kobj_attribute roce_pfc_xon_attr =
// __ATTR(roce_xon_threshold, 0660, show_roce_pfc_xon, store_roce_pfc_xon);

// static struct kobj_attribute roce_pfc_xoff_attr =
// __ATTR(roce_xoff_threshold, 0660, show_roce_pfc_xoff, store_roce_pfc_xoff);

// static struct attribute *pfc_attrs[] = {
// 	&roce_pfc_enable_attr.attr,
// 	&roce_pfc_priority_attr.attr,
// 	&roce_pfc_xon_attr.attr,
// 	&roce_pfc_xoff_attr.attr,

// 	&non_roce_pfc_enable_attr.attr,
// 	&non_roce_pfc_priority_attr.attr,
// 	&non_roce_pfc_xon_attr.attr,
// 	&non_roce_pfc_xoff_attr.attr,
// 	&disable_priory_check_attr.attr,
// 	NULL,
// };

// struct attribute_group pfc_attr_group = {
// 	.attrs = pfc_attrs,
// };

// static int pfc_create_sysfs_entries(const char *name,
// 				struct kobject *parent,
// 				const struct attribute_group *grp,
// 				struct kobject **kobj)
// {
// 	int ret = 0;

// 	*kobj = kobject_create_and_add(name, parent);

// 	if (!*kobj) {
// 		pr_err("pfc %s sysfs create failed", name);
// 		return -ENOMEM;
// 	}

// 	ret = sysfs_create_group(*kobj, grp);
// 	if (ret < 0) {
// 		pr_err("%s unable to create pfc %s sysfs entries\n",
// 				__func__, name);
// 		return ret;
// 	}

// 	return ret;
// }

// static const struct ib_device_ops xib_dev_ops = {
// TODO:
// 	.owner	= THIS_MODULE,
// 	.driver_id = RDMA_DRIVER_XLNX,
// 	.uverbs_abi_ver	= 1,

// 	.query_device	= xib_query_device,
// 	.query_port	= xib_query_port,
// 	.query_pkey	= xib_query_pkey,
// 	.alloc_ucontext	= xib_alloc_ucontext,
// 	.dealloc_ucontext = xib_dealloc_ucontext,
// 	.mmap	= xib_mmap,
// //	.query_gid	= xib_query_gid,
// 	.add_gid	= xib_add_gid,
// 	.del_gid	= xib_del_gid,
// 	.alloc_pd	= xib_alloc_pd,
// 	.alloc_mr	= xib_alloc_mr,
// 	.map_mr_sg	= xib_map_mr_sge,
// 	.dealloc_pd	= xib_dealloc_pd,
// 	.get_link_layer	= xib_get_link_layer,
// 	.get_netdev	= xib_get_netdev,
// 	.create_ah	= xib_create_ah,
// 	.destroy_ah	= xib_destroy_ah,
// 	.create_qp	= xib_create_qp,
// 	.modify_qp	= xib_modify_qp,
// 	.query_qp	= xib_query_qp,
// 	.destroy_qp	= xib_destroy_qp,
// 	.post_send	= xib_post_send,
// 	.drain_sq	= xib_drain_sq,
// 	.drain_rq	= xib_drain_rq,
// 	.post_recv	= xib_post_recv,
// 	.create_cq	= xib_create_cq,
// 	.destroy_cq	= xib_destroy_cq,
// 	.modify_cq	= xib_modify_cq,
// 	.poll_cq	= xib_poll_cq,
// 	.req_notify_cq = xib_arm_cq,
// 	.get_dma_mr	= xib_get_dma_mr,
// 	.dereg_mr	= xib_dereg_mr,
// 	.get_port_immutable	= xib_port_immutable,

// 	.reg_user_mr	= xib_reg_user_mr,
// 	.reg_user_mr_ex	= xib_reg_user_mr,
// 	INIT_RDMA_OBJ_SIZE(ib_ah, xib_ah, ib_ah),
//         INIT_RDMA_OBJ_SIZE(ib_cq, xib_cq, ib_cq),
// 	INIT_RDMA_OBJ_SIZE(ib_pd, xib_pd, ib_pd),
// 	INIT_RDMA_OBJ_SIZE(ib_ucontext, xib_ucontext, ib_uc),
// };

void xib_set_dev_caps(struct ib_device *ibdev)
{
	ibdev->phys_port_cnt		= 1;
	ibdev->num_comp_vectors		= 1;
	ibdev->local_dma_lkey		= 0;
	ibdev->uverbs_cmd_mask		=
		(1ULL << IB_USER_VERBS_CMD_GET_CONTEXT) |
		(1ULL << IB_USER_VERBS_CMD_QUERY_DEVICE) |
		(1ULL << IB_USER_VERBS_CMD_QUERY_PORT) |
		(1ULL << IB_USER_VERBS_CMD_ALLOC_PD) |
		(1ULL << IB_USER_VERBS_CMD_DEALLOC_PD) |
		(1ULL << IB_USER_VERBS_CMD_REG_MR) |
		//TODO:
		// (1ULL << IB_USER_VERBS_CMD_REG_MR_EX) |
		(1ULL << IB_USER_VERBS_CMD_DEREG_MR) |
		(1ULL << IB_USER_VERBS_CMD_CREATE_COMP_CHANNEL) |
		(1ULL << IB_USER_VERBS_CMD_CREATE_CQ) |
		(1ULL << IB_USER_VERBS_CMD_DESTROY_CQ) |
		(1ULL << IB_USER_VERBS_CMD_CREATE_QP) |
		(1ULL << IB_USER_VERBS_CMD_MODIFY_QP) |
		(1ULL << IB_USER_VERBS_CMD_QUERY_QP) |
		(1ULL << IB_USER_VERBS_CMD_DESTROY_QP);
}

/**
 * yrdma_probe - yusur rdma driver probe
 * @pdev: platform device
 *
 * Create device resources, set up queues, pble and hmc objects.
 * Return 0 if successful, otherwise return error
 */
int yrdma_probe(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s : <---------- \n", __func__);
	return 0;
// 	int err;
// 	struct device_node *net_node;
// 	struct device_node *np;
// 	struct xrnic_local *xl;
// 	// unsigned long debug_phys;
// 	struct net_device *netdev;
// 	// struct resource resource;
// 	u32 qpn, rtr_count;
// 	u64 rtr_addr = 0;

// 	dev_dbg(&pdev->dev, "%s : <---------- \n", __func__);

// 	pr_debug("yrdma_probe debug: 222\n");

// 	ibdev = (struct xilinx_ib_dev *)ib_alloc_device(xilinx_ib_dev, ib_dev);
// 	if(!ibdev) {
// 		dev_err(&pdev->dev, "cant alloc ibdev\n");
// 		return -ENOMEM;
// 	}

// 	ibdev->mtu = QP_PMTU_4096;
// 	ibdev->pdev = pdev;
// 	platform_set_drvdata(pdev, ibdev);

// 	xl = xrnic_hw_init(pdev, ibdev);
// 	if (!xl) {
// 		dev_err(&pdev->dev, "xrnic init failed\n");
// 		return -ENODEV;
// 	}
// 	ibdev->xl = xl;
// 	xl->xib = ibdev;

// 	net_node = of_parse_phandle(pdev->dev.of_node, "eth-handle", 0);

// 	ibdev->netdev = of_find_net_device_by_node(net_node);

// 	if(!ibdev->netdev) {
// 		dev_err(&pdev->dev, "no netdev found\n");
// 		return -EINVAL;
// 	}

// 	err = update_mtu(ibdev->netdev);
//         np = of_find_node_by_name(NULL, "ernic");
//         if (!np) {
//                 dev_err(&pdev->dev, "Unable to find the ernic node\n");
//                 return -EFAULT;
//         }

// 	netdev = ibdev->netdev;

// 	xib_get_guid(netdev->dev_addr, (u8 *)&ibdev->ib_dev.node_guid);


// 	err = of_property_read_u32(pdev->dev.of_node, "xlnx,num-qp",
// 			&ibdev->dev_attr.max_qp);
//         if (err) {
//                 ibdev->dev_attr.max_qp = 32;
//                 pr_debug("num-qp entry not found. Configuring max qp's to [%d]\n",
//                         ibdev->dev_attr.max_qp);
//         }

// 	err = of_property_read_u32(pdev->dev.of_node, "xlnx,num-pd",
// 			&ibdev->dev_attr.max_pd);
// 	if (err) {
// 		ibdev->dev_attr.max_pd = 256;
// 	}

// 	err = of_property_read_u32(pdev->dev.of_node, "xlnx,max-sgl-depth",
// 			&ibdev->dev_attr.max_send_sge);

// 	dev_dbg(&pdev->dev, "%s: qp:%d pd:%d sgl_depth:%d \n", __func__, ibdev->dev_attr.max_qp,
// 			ibdev->dev_attr.max_pd,	ibdev->dev_attr.max_send_sge);

// 	ibdev->dev_attr.max_cq_wqes	= 1024;
// 	ibdev->dev_attr.max_mr		= ibdev->dev_attr.max_pd;

// //TODO: continue...
// 		/* initialize retry bufs
// 	* qp 0 doesnt exist
// 	* qp 1 is UD so no rtr buf needed
// 	*/
// #if 0
// 	rtr_count = (ibdev->dev_attr.max_qp - 2 )*16;
// #else
// 	rtr_count = 1024;
// #endif
// 	xl->retry_buf_va = xib_alloc_coherent(rtr_mem, ibdev,
// 		(rtr_count * XRNIC_SIZE_OF_DATA_BUF),
// 		&rtr_addr,
// 		GFP_KERNEL);
// 	if (!rtr_addr) {
// 		dev_err(&pdev->dev, "Failed to allocate rtr bufs\n");
// 		return -ENOMEM;
// 	}
// 	xl->retry_buf_pa = rtr_addr;

// 	xrnic_iow(xl, XRNIC_DATA_BUF_BASE_LSB, rtr_addr);
// 	xrnic_iow(xl, XRNIC_DATA_BUF_BASE_MSB, UPPER_32_BITS(rtr_addr));
// 	wmb();
// 	xrnic_iow(xl, XRNIC_DATA_BUF_SZ,
// 		rtr_count | ( XRNIC_SIZE_OF_DATA_BUF << 16));
// 	wmb();

// 	/* initialize in_pkt_errq bufs */
// 	xl->in_pkt_err_va = dma_alloc_coherent(&pdev->dev,
// 			(XRNIC_IN_PKT_ERRQ_DEPTH * 8), &xl->in_pkt_err_ba, GFP_KERNEL);
// 	if (!xl->in_pkt_err_ba) {
// 		dev_err(&pdev->dev, "Failed to allocate in_pkt_err bufs\n");
// 		return -ENOMEM;
// 	}
// 	xrnic_iow(xl, XRNIC_INCG_PKT_ERRQ_BASE_LSB, xl->in_pkt_err_ba);
// 	xrnic_iow(xl, XRNIC_INCG_PKT_ERRQ_BASE_MSB,
// 					UPPER_32_BITS(xl->in_pkt_err_ba));
// 	wmb();
// 	/* each entry is 8 bytes, max sz = 64  */
// 	xrnic_iow(xl, XRNIC_INCG_PKT_ERRQ_SZ, XRNIC_IN_PKT_ERRQ_DEPTH);
// 	wmb();

// 	spin_lock_init(&ibdev->lock);

// 	/* allocate qp list */
// 	ibdev->qp_list = (struct xib_qp **) kzalloc
// 			((sizeof(struct xib_qp *) * ibdev->dev_attr.max_qp), GFP_KERNEL);

// 	/* alloc pd bmap */
// 	xib_bmap_alloc(&ibdev->pd_map, ibdev->dev_attr.max_pd, "PD");
// 	/* alloc qp bmap */
// 	xib_bmap_alloc(&ibdev->qp_map, ibdev->dev_attr.max_qp, "QP");
// 	/* alloc mr bmap */
// 	xib_bmap_alloc(&ibdev->mr_map, ibdev->dev_attr.max_mr, "MR");

// 	/*
// 	*  TODO do we need to pass PL DDR for infiniband core
// 	* allocations?
// 	*/
// 	ibdev->ib_dev.dev.parent = &pdev->dev;

// 	strlcpy(ibdev->ib_dev.name, "xib_%d", IB_DEVICE_NAME_MAX);
// 	ibdev->ib_dev.node_type	= RDMA_NODE_IB_CA;

// 	xib_set_dev_caps(&ibdev->ib_dev);
// 	ib_set_device_ops(&ibdev->ib_dev, &xib_dev_ops);

// 	err = ib_register_device(&ibdev->ib_dev, ibdev->ib_dev.name);
// 	if (err) {
// 		dev_err(&pdev->dev, "failed to regiser xib device\n");
// 		goto err_1;
// 	}

// 	/* set dma mask */
// 	/* TODO set relevant mask for 64bit and 32 bit */
// 	err = dma_set_mask_and_coherent(&ibdev->pdev->dev, DMA_BIT_MASK(32));
// 	if (err != 0)
// 		dev_err(&pdev->dev, "unable to set dma mask\n");

// 	/* the phy attached to 40G is not giving out active speed or width
// 	 * since we are using 40G in the design default to 40G */ /*TODO */
// 	#if 0
// 	ib_get_eth_speed(&ibdev->ib_dev, 1, &ibdev->active_speed,
// 			&ibdev->active_width);
// 	#endif
// 	ibdev->active_speed = IB_SPEED_FDR10;
// 	ibdev->active_width = IB_WIDTH_4X;

// 	/* set the mac address */
// 	xrnic_set_mac(xl, netdev->dev_addr);

// 	err = set_ip_address(netdev, 1);
// 	//TODO: not suport ipv6 for now...
// 	err = set_ip_address(netdev, 0);

// 	/* pre-reserve QP1
// 	 * there is no QP0 in ernic HW
// 	 */
// 	spin_lock_bh(&ibdev->lock);
// 	xib_bmap_alloc_id(&ibdev->qp_map, &qpn);
// 	spin_unlock_bh(&ibdev->lock);

// 	dev_dbg(&pdev->dev, "gsi qpn: %d\n", qpn);

// 	//TODO: check
// 	// if(pfc_create_sysfs_entries("pfc", &ibdev->ib_dev.dev.kobj,
// 	// 		&pfc_attr_group, &ibdev->pfc_kobj)) {
// 	// 	dev_err(&pdev->dev, "Failed to create PFC sysfs entry\n");
// 	// 	goto err_2;
// 	// }

// 	/* register irq */
// 	err = request_irq(xl->irq, xib_irq, IRQF_SHARED, "xrnic_intr0",
// 			(void *)ibdev);
// 	if (err) {
// 		dev_err(&pdev->dev, "request irq error!\n");
// 		goto err_3;
// 	}

// 	/* start ernic HW */
// 	xl->qps_enabled = (ibdev->dev_attr.max_qp - 1);
// 	xl->udp_sport = 0x8cd1;
// 	xrnic_start(xl);
// 	/* register for net dev notifications */
// 	register_netdevice_notifier(&cmac_netdev_notifier);
// 	register_inetaddr_notifier(&cmac_inetaddr_notifier);
// 	register_inet6addr_notifier(&cmac_inet6addr_notifier);

// 	//TODO:
// 	//axidma_init(ibdev, pdev->dev.of_node);
// 	return 0;
// err_3:
// 	kobject_put(ibdev->pfc_kobj);
// // err_2:
// 	ib_unregister_device(&ibdev->ib_dev);
// err_1:
// 	return err;
}

/**
 * yrdma_remove - GEN_2 driver remove
 * @pdev: platform device
 */
int yrdma_remove(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s : <---------- \n", __func__);

	return 0;
}

/* TODO: Remove when iidc merged */
int yrdma_suspend(struct platform_device *pdev, pm_message_t state)
{
    dev_dbg(&pdev->dev, "%s : <---------- \n", __func__);
	return yrdma_remove(pdev);
}