/*
 *    archos-usb-ehci.c : 29/03/2011
 *    g.revaillot, revaillot@archos.com
 */
#define DEBUG
#include <linux/delay.h>
#include <plat/usb.h>
#include <mach/board-archos.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#include "mux.h"

static unsigned int gpio_fsusb_pwron = -EINVAL;
static unsigned int gpio_fsusb_suspend = -EINVAL;

static struct usbhs_omap_platform_data usbhs_pdata __initconst = {
	.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

/* FSUSB_5V power rail */
static struct regulator_consumer_supply board_fixed_fsusb_5v_supply[] = {
	REGULATOR_SUPPLY("hsusb1", "uhhtll-omap"),
};
static struct regulator_init_data board_fixed_fsusb_5v_init = {
	.constraints = {
		.name 			= "FSUSB_5V",
		.min_uV			= 5000000,
		.max_uV			= 5000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL |
					  REGULATOR_CHANGE_STATUS,
		.always_on		= 0,
		.boot_on		= 0,
	},
	.supply_regulator		= "5V",
	.num_consumer_supplies	= ARRAY_SIZE(board_fixed_fsusb_5v_supply),
	.consumer_supplies	= board_fixed_fsusb_5v_supply,
};
static struct fixed_voltage_config board_fixed_fsusb_5v_config = {
	.supply_name	= "FSUSB_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &board_fixed_fsusb_5v_init,
};
static struct platform_device board_fixed_fsusb_5v_device = {
	.name		= "reg-fixed-voltage",
	.id		= 4,
	.dev.platform_data = &board_fixed_fsusb_5v_config,
};

static int __init fsusb_init(void)
{
	const struct archos_fsusb_config * usb_cfg;
	const struct archos_fsusb_conf * cfg;
	int r;
	
	usb_cfg = omap_get_config( 
			ARCHOS_TAG_FSUSB, struct archos_fsusb_config);
	cfg = hwrev_ptr(usb_cfg, hardware_rev);
	if (IS_ERR(cfg)) {
		pr_err("%s: no device configuration for hardware_rev %i\n",
				__func__, hardware_rev);	
		return -ENODEV;
	}

	gpio_fsusb_pwron = cfg->enable_usb_ohci;
	gpio_fsusb_suspend = cfg->suspend;
	
	pr_debug("%s: fsusb_pwron: gpio%d, fsusb_susp: gpio%d\n",
			__func__, gpio_fsusb_pwron, gpio_fsusb_suspend);
	
	if (gpio_is_valid(gpio_fsusb_pwron)) {
		board_fixed_fsusb_5v_config.gpio = gpio_fsusb_pwron;
		omap_mux_init_gpio(gpio_fsusb_pwron, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
		platform_device_register(&board_fixed_fsusb_5v_device);
	}

	if (gpio_is_valid(gpio_fsusb_suspend)) {
		r = gpio_request(gpio_fsusb_suspend, "fsusb_susp");
		if (IS_ERR_VALUE(r)) {
			pr_err("%s: cannot get gpio%d for fsusb_susp\n", 
					__func__, gpio_fsusb_suspend);
			goto err2;
		}
		gpio_direction_output(gpio_fsusb_suspend, 0);
		omap_mux_init_gpio(gpio_fsusb_suspend, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
		gpio_export(gpio_fsusb_suspend, false);
	}
	
	return 0;

 err2:
	return -ENODEV;

}

int __init archos_omap3_uhhtll_init(void)
{
	int r;
	
	r = fsusb_init();
	if (IS_ERR_VALUE(r))
		return r;
	
	usb_uhhtll_init(&usbhs_pdata);
	return 0;
}
