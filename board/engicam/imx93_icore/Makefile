#
# Copyright 2022 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx93_icore.o
obj-y += ../../freescale/common/

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
ifdef CONFIG_IMX9_LPDDR4X_1GB
	obj-y += lpddr4x_1gb_timing.o
else ifdef CONFIG_IMX9_LPDDR4X
	obj-y += lpddr4x_timing.o
endif

endif
