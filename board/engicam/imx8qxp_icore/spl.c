/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <spl.h>
#include <asm/global_data.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/sci/sci.h>
#include <asm/arch/imx8-pins.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <bootm.h>

DECLARE_GLOBAL_DATA_PTR;

#define GPIO_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | \
			 (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) | \
			 (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | \
			 (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define USDHC2_SD_PWR IMX_GPIO_NR(4, 19)
static iomux_cfg_t usdhc2_sd_pwr[] = {
	SC_P_USDHC1_RESET_B | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

void spl_board_init(void)
{
	struct udevice *dev;
	int node, ret;

	node = fdt_node_offset_by_compatible(gd->fdt_blob, -1, "fsl,imx8-mu");

	ret = uclass_get_device_by_of_offset(UCLASS_MISC, node, &dev);
	if (ret) {
		return;
	}
	device_probe(dev);

	uclass_find_first_device(UCLASS_MISC, &dev);
	for (; dev; uclass_find_next_device(&dev)) {
		if (device_probe(dev))
			continue;
	}

	board_early_init_f();

	timer_init();

	imx8_iomux_setup_multiple_pads(usdhc2_sd_pwr, ARRAY_SIZE(usdhc2_sd_pwr));
	gpio_direction_output(USDHC2_SD_PWR, 0);


	preloader_console_init();

	puts("Normal Boot\n");
}

void spl_board_prepare_for_boot(void)
{
	board_quiesce_devices();
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* Just empty function now - can't decide what to choose */
	debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	board_init_r(NULL, 0);
}
