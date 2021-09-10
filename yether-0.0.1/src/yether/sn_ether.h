#ifndef __SN_ETHER_H
#define __SN_ETHER_H

#include "ysn3.h"

#define YUSUR_KMOD_VERSION "1.0"

//just for test
struct dummy_device {
	struct pci_dev *dev;
	struct ysn3_handle roce; /* RoCE handle*/
};

#endif
