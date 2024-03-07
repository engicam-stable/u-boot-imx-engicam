/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Engicam icorem6
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __ICOREM6_COMMON_CONFIG_H
#define __ICOREM6_COMMON_CONFIG_H

#include "mx6_common.h"
#include <linux/sizes.h>

#include <asm/arch/imx-regs.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* MMC Configs */
#define CFG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_BOUNCE_BUFFER

#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1

#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                        115200

/* Command definition */
#undef CONFIG_CMD_IMLS

#define CFG_EXTRA_ENV_SETTINGS \
	EXTRA_ENV_SETTINGS_ICORE \
	"fdt_high=0xffffffff\0"

/* Miscellaneous configurable options */
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

/* Physical Memory Map */
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CFG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CFG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CFG_SYS_INIT_RAM_SIZE       IRAM_SIZE

/* Framebuffer */
/*#define CONFIG_VIDEO_LOGO Abilitare per splashscreen UBOOT*/
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

#endif                         /* __ICOREM6_COMMON_CONFIG_H */
