/*
 * archos-lcd-BOE.c
 *
 *  Created on: mar 15, 2010
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <mach/gpio.h>
#include <plat/archos-gpio.h>
#include <asm/mach-types.h>
#include <plat/board.h>
#include <mach/board-archos.h>
#include <linux/delay.h>

#include "clock.h"
#include "mux.h"
#include <plat/display.h>

static struct archos_disp_conf display_gpio;
static int panel_state;
static bool have_panel;

static int __init panel_init(void)
{
	if (!have_panel)
		return -ENODEV;
	
	pr_debug("lcd_boe_wsvga10 panel_init\n");
	
	archos_gpio_init_output( display_gpio.lcd_pwon, "lcd_pwon" );
	archos_gpio_init_output( display_gpio.lvds_en, "lvds_en" );

	if (gpio_is_valid(display_gpio.bkl_en))
		archos_gpio_init_output( display_gpio.bkl_en, "bkl_en" );

	if (gpio_is_valid(display_gpio.lcd_pwon))
		gpio_set_value( display_gpio.lcd_pwon, 0);
	if (gpio_is_valid(display_gpio.lvds_en))
		gpio_set_value( display_gpio.lvds_en, 0);

	return 0;
}

static int panel_enable(struct omap_dss_device *disp)
{
	pr_info("panel_enable [%s]\n", disp->name);

	if ( panel_state == 1)
		return 0;

	if (gpio_is_valid(display_gpio.lcd_pwon)) {
		gpio_set_value( display_gpio.lcd_pwon, 1 );
		msleep(50);
	}
	if (gpio_is_valid(display_gpio.lvds_en)) {
		gpio_set_value( display_gpio.lvds_en, 1 );
		msleep(100);
	}
	if (gpio_is_valid(display_gpio.bkl_en))
		gpio_set_value( display_gpio.bkl_en, 0 );

	panel_state = 1;
	return 0;
}

static void panel_disable(struct omap_dss_device *disp)
{
	pr_debug("panel_disable [%s]\n", disp->name);

	if (panel_state == 0)
		return;
	
	if (gpio_is_valid(display_gpio.bkl_en))
		gpio_set_value( display_gpio.bkl_en, 1 );

	if (gpio_is_valid(display_gpio.lvds_en))
		gpio_set_value( display_gpio.lvds_en, 0 );
	if (gpio_is_valid(display_gpio.lcd_pwon))
		gpio_set_value( display_gpio.lcd_pwon, 0 );

	panel_state = 0;
}


static struct omap_dss_device boe_wsvga_10_panel = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "boe_wsvga_10",
	.phy.dpi.data_lines = 18,
	.phy.dpi.dither = OMAP_DSS_DITHER_SPATIAL,
	.platform_enable = panel_enable,
	.platform_disable = panel_disable,
};

int __init panel_boe_wsvga_10_init(struct omap_dss_device *disp_data)
{
	const struct archos_display_config *disp_cfg;
	const struct archos_disp_conf *cfg;
	int ret = -ENODEV;

	disp_cfg = omap_get_config( ARCHOS_TAG_DISPLAY, struct archos_display_config );
	if (disp_cfg == NULL)
		return ret;

	cfg = hwrev_ptr(disp_cfg, hardware_rev);
	if (IS_ERR(cfg)) {
		pr_err("%s: hardware_rev (%i) >= nrev (%i)\n",
			__func__, hardware_rev, disp_cfg->nrev);
		return ret;
	}

	display_gpio = *cfg;

	*disp_data = boe_wsvga_10_panel;
	have_panel = true;
	
	return 0;
}

device_initcall(panel_init);
