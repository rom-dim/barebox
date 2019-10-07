/*
 * Copyright (C) 2017 Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation.
 *
 */

#include <common.h>
#include <init.h>
#include <mach/bbu.h>
#include <linux/phy.h>
#include <environment.h>
#include <bootsource.h>

#include <linux/micrel_phy.h>

static int ksz8081_phy_fixup(struct phy_device *dev)
{
	phy_write(dev, 0x1f, 0x8190);
	phy_write(dev, 0x16, 0x202);

	return 0;
}

static int myir_mys6ul_init(void)
{
	int ret;
	char *environment_path, *default_environment_path;
	char *envdev, *default_envdev;

	if (!of_machine_is_compatible("myir,imx6ul-mys6ul"))
		return 0;

	phy_register_fixup_for_uid(PHY_ID_KSZ8081, MICREL_PHY_ID_MASK,
			ksz8081_phy_fixup);

	default_environment_path = "/chosen/environment-nand";
	default_envdev = "NAND flash";

	switch (bootsource_get()) {
	case BOOTSOURCE_MMC:
		environment_path = basprintf("/chosen/environment-sd%d",
					       bootsource_get_instance() + 1);
		envdev = "MMC";
		break;
	case BOOTSOURCE_NAND:
		environment_path = basprintf("/chosen/environment-nand");
		envdev = "NAND flash";
		break;
	case BOOTSOURCE_SPI_NOR:
		environment_path = basprintf("/chosen/environment-spinor");
		envdev = "SPI NOR flash";
		break;
	default:
		environment_path = basprintf(default_environment_path);
		envdev = default_envdev;
		break;
	}

	printf("Using environment in %s\n", envdev);
	if (environment_path) {
		ret = of_device_enable_path(environment_path);
		if (ret < 0)
			pr_warn("Failed to enable environment partition '%s' (%d)\n",
				environment_path, ret);
		free(environment_path);
	}

    imx6_bbu_nand_register_handler("nand", BBU_HANDLER_FLAG_DEFAULT);

	barebox_set_hostname("mys6ul");

	return 0;
}
device_initcall(myir_mys6ul_init);
