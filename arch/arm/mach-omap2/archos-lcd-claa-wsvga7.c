/*
 * archos-lcd-claa-wsvga7.c
 *
 *  Created on: feb 12, 2011
 *      Author: Robic Yvon <robic@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>

#include <mach/gpio.h>
#include <plat/archos-gpio.h>
#include <asm/mach-types.h>
#include <plat/board.h>
#include <mach/board-archos.h>
#include <plat/display.h>
#include <plat/dmtimer.h>

#include "clock.h"
#include "mux.h"

static struct archos_disp_conf display_gpio;
static int panel_state;
static int vcom_val = 140;
static struct omap_dm_timer *vcom_timer;
static bool have_panel;

static int __init panel_init(void)
{
	if (!have_panel)
		return -ENODEV;
	
	archos_gpio_init_output(display_gpio.lcd_pwon, "lcd_pwon");
	archos_gpio_init_output(display_gpio.lcd_rst, "lcd_rst");
	archos_gpio_init_output(display_gpio.lvds_en, "lvds_en");
	archos_gpio_init_output(display_gpio.lcd_avdd_en, "lcd_avdd_en");

	if (gpio_is_valid(display_gpio.lcd_pwon))
		gpio_set_value(display_gpio.lcd_pwon, 0);

	if (gpio_is_valid(display_gpio.lcd_rst))
		gpio_set_value(display_gpio.lcd_rst, 1);

	if (gpio_is_valid(display_gpio.lcd_avdd_en))
		gpio_set_value(display_gpio.lcd_avdd_en, 1);

	if (gpio_is_valid(display_gpio.lvds_en))
		gpio_set_value(display_gpio.lvds_en, 0);

	// vcom
	if (display_gpio.vcom_pwm.timer != -1) {
		vcom_timer = omap_dm_timer_request_specific(display_gpio.vcom_pwm.timer);

		if (vcom_timer != NULL) {
			omap_dm_timer_disable(vcom_timer);
		} else {
			printk( "failed to request vcom pwm timer %d \n", display_gpio.vcom_pwm.timer);
		}
	}

	msleep(10);
	return 0;
}

static void pwm_set_speed(struct omap_dm_timer *gpt,
		int frequency, int duty_cycle)
{
	u32 val;
	u32 period;
	struct clk *timer_fclk;

	/* and you will have an overflow in 1 sec         */
	/* so,                              */
	/* freq_timer     -> 1s             */
	/* carrier_period -> 1/carrier_freq */
	/* => carrier_period = freq_timer/carrier_freq */

	timer_fclk = omap_dm_timer_get_fclk(gpt);
	period = clk_get_rate(timer_fclk) / frequency;
	val = 0xFFFFFFFF+1-period;
	omap_dm_timer_set_load(gpt, 1, val);

	val = 0xFFFFFFFF+1-(period*duty_cycle/256);
	omap_dm_timer_set_match(gpt, 1, val);

	/* assume overflow first: no toogle if first trig is match */
	omap_dm_timer_write_counter(gpt, 0xFFFFFFFE);
}

static int panel_enable(struct omap_dss_device *disp)
{
	pr_info("panel_enable [%s]\n", disp->name);

	if (gpio_is_valid(display_gpio.lcd_pwon))
		gpio_set_value(display_gpio.lcd_pwon, 1 );

	msleep(50);

	if (gpio_is_valid(display_gpio.lcd_rst))
		gpio_set_value(display_gpio.lcd_rst, 0);

	if (gpio_is_valid(display_gpio.lcd_avdd_en))
		gpio_set_value(display_gpio.lcd_avdd_en, 0);

	msleep(35);

	if (gpio_is_valid(display_gpio.lvds_en))
		gpio_set_value(display_gpio.lvds_en, 1);

	msleep(10);

	panel_state = 1;

	if (vcom_timer != NULL) {
		omap_dm_timer_set_source(vcom_timer, OMAP_TIMER_SRC_SYS_CLK);

		omap_dm_timer_set_pwm( vcom_timer, 1, 1, OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

		pwm_set_speed(vcom_timer, 30000, vcom_val);
		omap_dm_timer_start(vcom_timer);
	}

	return 0;
}

static void panel_disable(struct omap_dss_device *disp)
{
	pr_info("panel_disable [%s]\n", disp->name);

	if (gpio_is_valid(display_gpio.lvds_en))
		gpio_set_value(display_gpio.lvds_en, 0);

	if (gpio_is_valid(display_gpio.lcd_avdd_en))
		gpio_set_value(display_gpio.lcd_avdd_en, 1);

	msleep(10);

	if (gpio_is_valid(display_gpio.lcd_pwon))
		gpio_set_value(display_gpio.lcd_pwon, 0 );

	if (gpio_is_valid(display_gpio.lcd_rst))
		gpio_set_value(display_gpio.lcd_rst, 1);

	if (vcom_timer != NULL) {
		omap_dm_timer_stop(vcom_timer);
	}
	panel_state = 0;
}

#if 0
static int panel_set_vcom(struct omap_dss_device *disp, u32 vcom)
{
	pr_debug("panel_set_vcom [%s]\n", disp->name);

	vcom_val = vcom;
	if ((panel_state == 1) && (vcom_timer != NULL))
		pwm_set_speed(vcom_timer, 30000, vcom_val);

	return 0;
}

static int panel_get_vcom(struct omap_dss_device *disp)
{
	pr_debug("panel_get_vcom [%s]\n", disp->name);

	return vcom_val;
}
#endif

static struct platform_device lcd_device = {
	.name           = "lcd_panel",
	.id             = 0,
	.num_resources   = 0,
};

static struct omap_dss_device claa_wsvga_7_panel = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "claa_wsvga_7",
	.phy.dpi.data_lines	= 24,
	.platform_enable	= panel_enable,
	.platform_disable	= panel_disable,
	.channel		= OMAP_DSS_CHANNEL_LCD,
};

int __init panel_claa_wsvga_7_init(struct omap_dss_device *disp_data)
{
	const struct archos_display_config *disp_cfg;
	const struct archos_disp_conf * conf;
	int ret = -ENODEV;

	printk(KERN_INFO "panel_claa_wsvga_7_init\n");

	disp_cfg = omap_get_config( ARCHOS_TAG_DISPLAY, struct archos_display_config );
	if (disp_cfg == NULL) {
		pr_info("%s: no display config found\n", __func__);
		return ret;
	}

	conf = hwrev_ptr(disp_cfg, hardware_rev);
	if (IS_ERR(conf)) {
		pr_info("%s: no display config for hwrev %i\n", 
				__func__, hardware_rev);
		return ret;
	}

	display_gpio = *conf;
	vcom_timer = NULL;
	
	if (IS_ERR_VALUE(platform_device_register(&lcd_device)))
		pr_info("%s: cannot register lcd_panel device\n", __func__);
	
	*disp_data = claa_wsvga_7_panel;
	have_panel = true;
	
	return 0;
}

device_initcall(panel_init);
