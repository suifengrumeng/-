/*
 * Configuration settings for new Allwinner F-series (suniv) CPU
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_SUNXI_USB_PHYS 1

/*
 * Include common sunxi configuration where most the settings are
 */
#include <configs/sunxi-common.h>

#define CONFIG_BOOTARGS "root=/dev/mtdblock0 rw lpj=5931008 mem=32m console=ttyS0,115200n8 hdb=noprobe ide0=noprobe"

#endif /* __CONFIG_H */
