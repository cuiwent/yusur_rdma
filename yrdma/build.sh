# SPDX-License-Identifier: GPL-2.0 or Linux-OpenIB
#!/bin/bash
# Copyright (c) 2015 - 2021 Intel Corporation

print_usage() {
	echo
	echo "usage: $0 {ofed}"
	echo "          ofed - Compile using OFED 4.17 or above modules."
	echo
	exit 1
}

get_suse_local_ver() {
	CONFIG_SUSE_KERNEL=`grep " CONFIG_SUSE_KERNEL " $1 | cut -d' ' -f3`
	if [ "$CONFIG_SUSE_KERNEL" == "1" ]; then
		LV=`grep " CONFIG_LOCALVERSION " $1 | cut -d'-' -f2 | sed 's/\.g[[:xdigit:]]\{7\}//'`
		LV_A=`echo $LV | cut -d'.' -f1`
		LV_B=`echo $LV | cut -s -d'.' -f2`
		LV_C=`echo $LV | cut -s -d'.' -f3`
		SLE_LOCALVERSION_CODE=$((LV_A * 65536 + LV_B * 256 + LV_C))
	else
		SLE_LOCALVERSION_CODE=0
	fi
}

cmd_initrd() {
	echo "Updating initramfs..."
	if which dracut > /dev/null 2>&1 ; then
		echo "dracut --force"
		dracut --force
	elif which update-initramfs > /dev/null 2>&1 ; then
		echo "update-initramfs -u"
		update-initramfs -u
	else
		echo "Unable to update initramfs. You may need to do this manually."
	fi
}

# Use KSRC if defined.
if [ -z "$KSRC" ]; then

	if [ -z "$BUILD_KERNEL" ]; then
		BUILD_KERNEL=`uname -r`
	fi

	if [ -e /usr/src/kernels/linux-$BUILD_KERNEL/include/config ]; then
		KSRC="/usr/src/kernels/linux-$BUILD_KERNEL/"
	elif [ -e /usr/src/kernels/$BUILD_KERNEL/include/config ]; then
		KSRC="/usr/src/kernels/$BUILD_KERNEL/"
	elif [ -e /lib/modules/$BUILD_KERNEL/build/include/config ]; then
		KSRC="/lib/modules/$BUILD_KERNEL/build/"
	fi

	if [ -z "$KSRC" ]; then
		BUILD_KERNEL=`uname -r | sed 's/\([0-9]*\.[0-9]*\)\..*/\1/'`
		if [ -e /usr/src/kernels/linux-$BUILD_KERNEL/include/config ]; then
			KSRC="/usr/src/kernels/linux-$BUILD_KERNEL/"
		elif [ -e /usr/src/kernels/$BUILD_KERNEL/include/config ]; then
			KSRC="/usr/src/kernels/$BUILD_KERNEL/"
		elif [ -e /lib/modules/$BUILD_KERNEL/build/include/config ]; then
			KSRC="/lib/modules/$BUILD_KERNEL/build/"
		fi
	fi
	export KSRC
fi


if [ -e ${KSRC}/include/linux/kconfig.h ]; then
	INCLUDE_KCONF_HDR="-include ${KSRC}/include/linux/kconfig.h"
	export INCLUDE_KCONF_HDR
fi

if [ -e ${KSRC}/include/generated/autoconf.h ]; then
	INCLUDE_AUTOCONF_HDR="-include ${KSRC}/include/generated/autoconf.h"
	export INCLUDE_AUTOCONF_HDR
	get_suse_local_ver "${KSRC}/include/generated/autoconf.h"
elif [ -e ${KSRC}/include/linux/autoconf.h ]; then
	INCLUDE_AUTOCONF_HDR="-include ${KSRC}/include/linux/autoconf.h"
	export INCLUDE_AUTOCONF_HDR
	get_suse_local_ver "${KSRC}/include/linux/autoconf.h"
fi

if [ -e ${KSRC}/include/generated/utsrelease.h ]; then
	UTSRELEASE_HDR="-include ${KSRC}/include/generated/utsrelease.h"
	export UTSRELEASE_HDR
fi


make -C $KSRC M=$PWD/src/yrdma clean

which nproc > /dev/null 2>&1
if [ $? -ne 0 ]; then
	nproc=1
else
	nproc=`nproc`
fi

if [ "$1" == "ofed" ]; then
	if [ -z "$OFED_OPENIB_PATH" ]; then
	        OFED_OPENIB_PATH="/usr/src/openib"
	fi
	if [ ! -e $OFED_OPENIB_PATH ]; then
		echo "Please install OFED development package"
		print_usage
	fi

	if [ -z "$OFED_VERSION_CODE" ]; then
		V1=$(ofed_info | head -1 | cut -d '-' -f 2 | cut -d '.' -f 1)
		V2=$(ofed_info | head -1 | cut -d '-' -f 2 | cut -d '.' -f 2 | cut -d ':' -f 1)
		OFED_VERSION_CODE=$(( ($V1 << 16) + ($V2 << 8) ))
	fi

	if [ ${OFED_VERSION_CODE} -lt $(( (4 << 16) + (8 << 8) )) ]; then
		echo "Unsupported OFED version installed, requires 4.8 or above"
		exit 1
	fi

	KBUILD_EXTRA_SYMBOLS="$OFED_OPENIB_PATH/Module.symvers"
	export KBUILD_EXTRA_SYMBOLS

	INCLUDE_COMPAT_HDR="-include $OFED_OPENIB_PATH/include/linux/compat-2.6.h -I$OFED_OPENIB_PATH/include -I$OFED_OPENIB_PATH/include/uapi"
	export INCLUDE_COMPAT_HDR

	if [ ${OFED_VERSION_CODE} == $(( (4 << 16) + (8 << 8) )) ]; then
		make "CFLAGS_MODULE=-DMODULE -DSLE_LOCALVERSION_CODE=${SLE_LOCALVERSION_CODE} -D__OFED_4_8__ -DOFED_VERSION_CODE=${OFED_VERSION_CODE}" -j$nproc -C $KSRC M=$PWD/src/yrdma W=1
	else
		make "CFLAGS_MODULE=-DMODULE -DSLE_LOCALVERSION_CODE=${SLE_LOCALVERSION_CODE} -D__OFED_BUILD__ -DOFED_VERSION_CODE=${OFED_VERSION_CODE}" -j$nproc -C $KSRC M=$PWD/src/yrdma W=1
	fi
else
	make "CFLAGS_MODULE=-DMODULE -DSLE_LOCALVERSION_CODE=${SLE_LOCALVERSION_CODE} -DOFED_VERSION_CODE=${OFED_VERSION_CODE}" -j$nproc -C $KSRC M=$PWD/src/yrdma W=1
fi

if [ $? -eq 0 ]; then
	if [ "$2" != "noinstall" ]; then
		#make -C $KSRC M=$PWD/src/yrdma INSTALL_MOD_DIR=updates/drivers/infiniband/hw/yrdma modules_install
		if [ $? -eq 0 ]; then
			depmod -a
			#cmd_initrd
		else
			exit 1
		fi
	fi
else
	exit 1
fi
