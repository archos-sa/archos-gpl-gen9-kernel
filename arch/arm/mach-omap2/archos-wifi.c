/*
 * archos-wifi.c
 *
 *  Created on: Feb 2, 2011
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 *
 * Based on mach-omap2/board-4430sdp-wifi.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/wifi_tiwlan.h>

#include <mach/board-archos.h>

#include "mux.h"

struct wifi_mux {
	char *mode;
	unsigned int flags;
};

static int wl1271_wifi_cd;		/* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static struct archos_wifi_bt_dev_conf wl1271_conf;

#ifdef CONFIG_WIFI_CONTROL_FUNC
#define WIFI_STATIC static
#else
#define WIFI_STATIC
#endif

static const struct wifi_mux sdio_mux_omap4[] __initdata = 
{
	{ "mcspi4_clk.sdmmc4_clk",	OMAP_PIN_INPUT_PULLUP },
	{ "mcspi4_somi.sdmmc4_dat0",	OMAP_PIN_INPUT_PULLUP },
	{ "mcspi4_simo.sdmmc4_cmd",	OMAP_PIN_INPUT_PULLUP },
	{ "mcspi4_cs0.sdmmc4_dat3",	OMAP_PIN_INPUT_PULLUP },
	{ "uart4_rx.sdmmc4_dat2",	OMAP_PIN_INPUT_PULLUP },
	{ "uart4_tx.sdmmc4_dat1",	OMAP_PIN_INPUT_PULLUP },
	{ NULL, 0 }
};

static const struct wifi_mux sdio_mux_omap3[] __initdata = 
{
	{ "etk_clk.sdmmc3_clk",		OMAP_PIN_INPUT_PULLUP },
	{ "etk_d4.sdmmc3_dat0",		OMAP_PIN_INPUT_PULLUP },
	{ "etk_ctl.sdmmc3_cmd",		OMAP_PIN_INPUT_PULLUP },
	{ "etk_d3.sdmmc3_dat3",		OMAP_PIN_INPUT_PULLUP },
	{ "etk_d6.sdmmc3_dat2",		OMAP_PIN_INPUT_PULLUP },
	{ "etk_d5.sdmmc3_dat1",		OMAP_PIN_INPUT_PULLUP },
	{ NULL, 0 }
};

static void __init config_wlan_mux(const struct wifi_mux sdio_mux[])
{
	int i;
	
	if (wl1271_conf.wifi_irq_signal)
		omap_mux_init_signal(wl1271_conf.wifi_irq_signal, OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_WAKEUPENABLE);
	else
		omap_mux_init_gpio(wl1271_conf.wifi_irq, OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_WAKEUPENABLE);
	if (wl1271_conf.wifi_power_signal)
		omap_mux_init_signal(wl1271_conf.wifi_power_signal, OMAP_PIN_OUTPUT);
	else
		omap_mux_init_gpio(wl1271_conf.wifi_power, OMAP_PIN_OUTPUT);
	
	for (i = 0; sdio_mux[i].mode; i++)
		omap_mux_init_signal(sdio_mux[i].mode, sdio_mux[i].flags);
}

int omap_wifi_status_register(void (*callback)(int card_present,
						void *dev_id), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;

	wifi_status_cb_devid = dev_id;

	return 0;
}

int omap_wifi_status(struct device *dev, int slot)
{
	return wl1271_wifi_cd;
}

WIFI_STATIC int wl1271_wifi_set_carddetect(int val)
{
	printk(KERN_WARNING"%s: %d\n", __func__, val);
	wl1271_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(wl1271_wifi_set_carddetect);
#endif

static int wl1271_wifi_power_state;

WIFI_STATIC int wl1271_wifi_power(int on)
{
	printk(KERN_WARNING"%s: %d\n", __func__, on);
	gpio_set_value(wl1271_conf.wifi_power, on);
	wl1271_wifi_power_state = on;
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(wl1271_wifi_power);
#endif

static int wl1271_wifi_reset_state;
WIFI_STATIC int wl1271_wifi_reset(int on)
{
	printk(KERN_WARNING"%s: %d\n", __func__, on);
	wl1271_wifi_reset_state = on;
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(wl1271_wifi_reset);
#endif

struct wifi_platform_data wl1271_wifi_control = {
	.set_power	= wl1271_wifi_power,
	.set_reset	= wl1271_wifi_reset,
	.set_carddetect	= wl1271_wifi_set_carddetect,
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct resource wl1271_wifi_resources[] = {
	[0] = {
		.name		= "device_wifi_irq",
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device wl1271_wifi_device = {
	.name           = "device_wifi",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(wl1271_wifi_resources),
	.resource       = wl1271_wifi_resources,
	.dev            = {
		.platform_data = &wl1271_wifi_control,
	},
};
#endif

static int __init wl1271_wifi_init(void)
{
	const struct archos_wifi_bt_config *wifi_bt_cfg;
	const struct archos_wifi_bt_dev_conf *conf;
	int ret;

	printk(KERN_WARNING"%s: start\n", __func__);

	wifi_bt_cfg = omap_get_config(ARCHOS_TAG_WIFI_BT, 
			struct archos_wifi_bt_config);
	conf = hwrev_ptr(wifi_bt_cfg, hardware_rev);
	if (IS_ERR(conf)) {
		pr_err("%s: no device configuration for hardware_rev %i\n",
				__func__, hardware_rev);
		return -ENODEV;
	}
	
	wl1271_conf = *conf;

	ret = gpio_request(wl1271_conf.wifi_power, "wifi_pmena");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
			wl1271_conf.wifi_power);
		goto out;
	}
	gpio_direction_output(wl1271_conf.wifi_power, 0);

	ret = gpio_request(wl1271_conf.wifi_irq, "wifi_irq");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			wl1271_conf.wifi_irq);
		goto out;
	}
	gpio_direction_input(wl1271_conf.wifi_irq);
	
	if (cpu_is_omap44xx())
		config_wlan_mux(sdio_mux_omap4);
	else
		config_wlan_mux(sdio_mux_omap3);
	
#ifdef CONFIG_WIFI_CONTROL_FUNC
	wl1271_wifi_resources[0].start = gpio_to_irq(wl1271_conf.wifi_irq);
	wl1271_wifi_resources[0].end   = gpio_to_irq(wl1271_conf.wifi_irq);
	
	ret = platform_device_register(&wl1271_wifi_device);
	if (!ret) {
		gpio_export(wl1271_conf.wifi_power, false);
		gpio_export_link(&wl1271_wifi_device.dev, "wifi_pmena", wl1271_conf.wifi_power);
	}
#endif
out:
	return ret;
}

device_initcall(wl1271_wifi_init);
