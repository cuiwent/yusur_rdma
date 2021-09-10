#include <linux/pci.h>
#include <linux/vermagic.h>
#include "sn_ether.h"
#include "ysn3.h"

static const char sn_driver_name[] = "yusur dummy ethernet";
const char sn_driver_version[] = VERMAGIC_STRING;
static const char sn_driver_string[] =
			"Yusur Dummy Ethernet Network Driver for u200";
static const char yusur_copyright[] = "Copyright (c) 2021 Yusur tech";

#define VENDOR_ID 0x10EE  // Xilinx Vendor ID
#define DEVICE_ID 0x903f

/* global dummy device */
struct dummy_device roce_dummy_device;

/*
 *
 */
static int __init sn_init_module(void)
{
	pr_info("%s: %s - version\n", sn_driver_name, sn_driver_string);
	pr_info("%s: %s\n", sn_driver_name, yusur_copyright);

	pr_info("sn_init_module\n");

	//Just tor test: init dummy device based deive and venor id
	roce_dummy_device.dev = pci_get_device(VENDOR_ID, DEVICE_ID, roce_dummy_device.dev);
	if (roce_dummy_device.dev) {
		pr_info("Init: u200 board found\n");
	} else {
        pr_info("Init: u200 board NOT found!!\n");
		roce_dummy_device.dev = NULL;
	}

	return 0;
}

module_init(sn_init_module);

/*
 * sn_exit_module - Driver exit cleanup routine
*/
static void __exit sn_exit_module(void)
{
	pr_info("%s: %s - version\n", sn_driver_name, sn_driver_string);
	pr_info("%s: %s\n", sn_driver_name, yusur_copyright);

	pr_info("sn_exit_module\n");

}
module_exit(sn_exit_module);

MODULE_ALIAS("yether");
MODULE_DESCRIPTION("Yusur u200 Dummy Ethernet Driver");
MODULE_AUTHOR("Wentao,Cui <cuiwt@yusur.tech>.");
MODULE_LICENSE("GPL");
MODULE_VERSION(YUSUR_KMOD_VERSION);