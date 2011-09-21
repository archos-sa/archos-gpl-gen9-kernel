/*
 *    archos-usb-touchscreen.c : 07/06/2010
 *    g.revaillot, revaillot@archos.com
 */

#define DEBUG
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <plat/archos-gpio.h>
#include <mach/board-archos.h>

#include <linux/delay.h>
#include <linux/err.h>

static int gpio_tsp_enable = -EINVAL;
static int gpio_tsp_reset = -EINVAL;

static int tsp_is_enabled = -1;
static int tsp_is_reset = -1;

static struct usb_tsp_platform_data tsp_platform_data;

static struct platform_device usb_tsp_device = {
	.name = "usb_tsp",
	.id = -1,
	.dev.platform_data = &tsp_platform_data,
};

static void usb_tsp_enable(int on_off)
{
	if (on_off == tsp_is_enabled)
		return;

	gpio_set_value(gpio_tsp_enable, on_off);

	tsp_is_enabled = on_off;
}

static void usb_tsp_reset(int on_off)
{
	if (on_off == tsp_is_reset)
		return;

	gpio_set_value(gpio_tsp_reset, on_off);

	tsp_is_reset = on_off;
}

static int __init archos_usb_tsp_init(void)
{
	int ret;
	const struct archos_usb_tsp_config *usb_tsp_config = 
			omap_get_config(ARCHOS_TAG_USB_TSP, struct archos_usb_tsp_config);
	const struct archos_usb_tsp_conf* conf;
	
	pr_debug("%s\n", __func__);
	
	conf = hwrev_ptr(usb_tsp_config, hardware_rev);
	if (IS_ERR(conf)) {
		pr_err("archos_usb_tsp_init: no configuration for hardware_rev %d\n", 
				hardware_rev);
		return -ENODEV;
	}
	
	tsp_platform_data.flags = conf->suspend_flags;
	tsp_platform_data.x_scale = conf->x_scale;
	tsp_platform_data.x_offset = conf->x_offset;
	tsp_platform_data.y_scale = conf->y_scale;
	tsp_platform_data.y_offset = conf->y_offset;
	
	ret = platform_device_register(&usb_tsp_device);
	if (ret < 0)
		return -ENODEV;

	if (gpio_is_valid(conf->enable)) {
		gpio_tsp_enable = conf->enable;
		archos_gpio_init_output(gpio_tsp_enable, "tsp_enable");
		gpio_set_value(gpio_tsp_enable, 0);
		gpio_export(gpio_tsp_enable, false);
		gpio_export_link(&usb_tsp_device.dev, "tsp_enable", gpio_tsp_enable);

		tsp_platform_data.panel_power = usb_tsp_enable;
		usb_tsp_enable(1);
	}

	if (gpio_is_valid(conf->reset)) {
		gpio_tsp_reset = conf->reset;
		archos_gpio_init_output(gpio_tsp_reset, "tsp_reset");
		gpio_set_value(gpio_tsp_reset, 1);
		gpio_export(gpio_tsp_reset, false);
		gpio_export_link(&usb_tsp_device.dev, "tsp_reset", gpio_tsp_reset);

		usb_tsp_reset(1);
		msleep(200);
		usb_tsp_reset(0);
	}

	return 0;
}

device_initcall(archos_usb_tsp_init);
