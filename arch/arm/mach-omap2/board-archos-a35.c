/*
 * board-archos-a35.c
 *
 *  Created on: Dec 16, 2010
 *      Author: Niklas Schroeter <schroeter@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/ads7846.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/i2c/twl.h>
#include <linux/mmc/host.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/gpio.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <plat/timer-gp.h>
#include <plat/display.h>
#include <plat/control.h>
#include <plat/voltage.h>
#include <plat/opp_twl_tps.h>
#include <plat/mmc.h>
#include <linux/leds.h>
#include <linux/mma7660fc.h>
#include <plat/archos-audio-wm8988.h>
#include <mach/board-archos.h>
#include <asm/feature_list.h>
#include <plat/usb.h>
#include <linux/usb/gpio_vbus.h>

#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif

#include "mux.h"
#include "omap3-opp.h"
#include "hsmmc.h"
//#include "sdram-elpida-edk2132c2pb.h"
#include "sdram-micron-mt46h32m32lf-6.h"

static struct mma7660fc_pdata board_mma7660fc_pdata;
static struct gpio_vbus_mach_info archos_vbus_info;

extern int __init archos_audio_gpio_init(void);

static void __init board_init_irq(void)
{
//	omap2_init_common_hw(edk2132c2pd_50_sdrc_params, omap3630_mpu_rate_table,
//			     omap3630_dsp_rate_table, omap3630_l3_rate_table);

	omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
			     mt46h32m32lf6_sdrc_params);
	omap_init_irq();
//#ifdef CONFIG_OMAP_32K_TIMER
//	omap2_gp_clockevent_set_gptimer(12);
//#else
	omap2_gp_clockevent_set_gptimer(1);
//#endif
}


static struct archos_display_config display_config __initdata = {
	.nrev = 10,
	.rev[0] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	49,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[1] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	49,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[2] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	49,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[3] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	49,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[4] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	49,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[5] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	49,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[6] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	49,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[7] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	49,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[8] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	49,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[9] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	49,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
};

static struct archos_tsp_config tsp_config __initdata = {
	.nrev = 10,
	.rev[0] = {
		.irq_gpio = 24,
		.pwr_gpio = 51,
		.bus_num = 2,
		.x_plate_ohms = 745,
		.pressure_max = 2000,
		.inversion_flags = XY_SWAP | X_INV,
	},
	.rev[1] = {
		.irq_gpio = 24,
		.pwr_gpio = 51,
		.bus_num = 2,
		.x_plate_ohms = 745,
		.pressure_max = 2000,
		.inversion_flags = XY_SWAP | X_INV,
	},
	.rev[2] = {
		.irq_gpio = 24,
		.pwr_gpio = 51,
		.bus_num = 2,
		.x_plate_ohms = 745,
		.pressure_max = 2000,
		.inversion_flags = XY_SWAP | X_INV,
	},
	.rev[3] = {
		.irq_gpio = 24,
		.pwr_gpio = 51,
		.bus_num = 2,
		.x_plate_ohms = 745,
		.pressure_max = 2000,
		.inversion_flags = XY_SWAP | X_INV,
	},
	.rev[4] = {
		.irq_gpio = 24,
		.pwr_gpio = 51,
		.bus_num = 2,
		.x_plate_ohms = 745,
		.pressure_max = 2000,
		.inversion_flags = XY_SWAP | X_INV,
	},
	.rev[5] = {
		.irq_gpio = 24,
		.pwr_gpio = 51,
		.bus_num = 2,
		.x_plate_ohms = 745,
		.pressure_max = 2000,
		.inversion_flags = XY_SWAP | X_INV,
	},
	.rev[6] = {
		.irq_gpio = 24,
		.pwr_gpio = 51,
		.bus_num = 2,
		.x_plate_ohms = 745,
		.pressure_max = 2000,
		.inversion_flags = XY_SWAP | X_INV,
	},
	.rev[7] = {
		.irq_gpio = 24,
		.pwr_gpio = 51,
		.bus_num = 2,
		.x_plate_ohms = 745,
		.pressure_max = 2000,
		.inversion_flags = XY_SWAP | X_INV,
	},
	.rev[8] = {
		.irq_gpio = 24,
		.pwr_gpio = 51,
		.bus_num = 2,
		.x_plate_ohms = 745,
		.pressure_max = 2000,
		.inversion_flags = XY_SWAP | X_INV,
	},
	.rev[9] = {
		.irq_gpio = 24,
		.pwr_gpio = 51,
		.bus_num = 2,
		.x_plate_ohms = 745,
		.pressure_max = 2000,
		.inversion_flags = XY_SWAP | X_INV,
	},
};

static struct archos_audio_config audio_config __initdata = {
	.nrev = 10,
	.rev[0] = {
		.spdif = UNUSED_GPIO,
		.hp_on = UNUSED_GPIO,
		.headphone_plugged = UNUSED_GPIO,
		.clk_mux = -1,
	},
	.rev[1] = {
		.spdif = UNUSED_GPIO,
		.hp_on = UNUSED_GPIO,
		.headphone_plugged = UNUSED_GPIO,
		.clk_mux = -1,
	},
	.rev[2] = {
		.spdif = UNUSED_GPIO,
		.hp_on = UNUSED_GPIO,
		.headphone_plugged = UNUSED_GPIO,
		.clk_mux = -1,
	},
	.rev[3] = {
		.spdif = UNUSED_GPIO,
		.hp_on = UNUSED_GPIO,
		.headphone_plugged = UNUSED_GPIO,
		.clk_mux = -1,
	},
	.rev[4] = {
		.spdif = UNUSED_GPIO,
		.hp_on = UNUSED_GPIO,
		.headphone_plugged = UNUSED_GPIO,
		.clk_mux = -1,
	},
	.rev[5] = {
		.spdif = UNUSED_GPIO,
		.hp_on = UNUSED_GPIO,
		.headphone_plugged = UNUSED_GPIO,
		.clk_mux = -1,
	},
	.rev[6] = {
		.spdif = UNUSED_GPIO,
		.hp_on = UNUSED_GPIO,
		.headphone_plugged = UNUSED_GPIO,
		.clk_mux = -1,
	},
	.rev[7] = {
		.spdif = UNUSED_GPIO,
		.hp_on = UNUSED_GPIO,
		.headphone_plugged = UNUSED_GPIO,
		.clk_mux = -1,
	},
	.rev[8] = {
		.spdif = UNUSED_GPIO,
		.hp_on = UNUSED_GPIO,
		.headphone_plugged = UNUSED_GPIO,
		.clk_mux = -1,
	},
	.rev[9] = {
		.spdif = UNUSED_GPIO,
		.hp_on = UNUSED_GPIO,
		.headphone_plugged = UNUSED_GPIO,
		.clk_mux = -1,
	},
};
static struct archos_accel_config accel_config __initdata = {
	.nrev = 10,
	.rev[0] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[1] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[2] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[3] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[4] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[5] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[6] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[7] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[8] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[9] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
};

static struct archos_leds_config leds_config __initdata = {
	.nrev = 10,
	.rev[0] = {
		.power_led = 50,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 210,
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
	.rev[1] = {
		.power_led = 50,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 210,
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
	.rev[2] = {
		.power_led = 50,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 210,
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
	.rev[3] = {
		.power_led = 50,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 210,
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
	.rev[4] = {
		.power_led = 50,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 210,
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
	.rev[5] = {
		.power_led = 50,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 210,
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
	.rev[6] = {
		.power_led = 50,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 210,
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
	.rev[7] = {
		.power_led = 50,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 210,
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
	.rev[8] = {
		.power_led = 50,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 210,
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
	.rev[9] = {
		.power_led = 50,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 210,
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
};

static struct archos_usb_gadget_config gadget_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.product_name = "A35",
		.product_id = 0x142a,
		.ums_luns = 2,
	},
	.rev[1] = {
		.product_name = "A35",
		.product_id = 0x142a,
		.ums_luns = 2,		
	},
};

static struct omap_board_config_kernel board_config[] __initdata = {
	{ ARCHOS_TAG_TSP,		&tsp_config },
	{ ARCHOS_TAG_AUDIO,		&audio_config},
	{ ARCHOS_TAG_ACCEL,		&accel_config},
	{ ARCHOS_TAG_DISPLAY,		&display_config},
	{ ARCHOS_TAG_LEDS,		&leds_config},
	{ ARCHOS_TAG_USB_GADGET,	&gadget_config},
};

static struct omap_dss_device board_lcd_device;

static struct omap_dss_device *board_dss_devices[] = {
	&board_lcd_device,
};

static struct omap_dss_board_info board_dss_data = {
	.num_devices	=	1,
	.devices	=	&board_dss_devices,
	.default_device	=	NULL,
};

static struct platform_device board_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev            = {
		.platform_data = &board_dss_data,
	},
};

static int __init omap3_leds_init(void)
{
	omap_mux_init_signal("gpmc_ncs7.gpt8_pwm_evt", OMAP_PIN_OUTPUT);

	return 0;
}


static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable   = true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp        = -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{}	/* Terminator */
};

static int archos_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

#if 0
	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
#endif
	return ret;
}

static __init void archos_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev)
		return;

	pdata = dev->platform_data;
	pdata->init = archos_hsmmc_late_init;
}

#define GPIO_MMC1_POWER		26
#define GPIO_MMC1_DETECT	25
#define GPIO_MMC1_PREWARN	-1
static int archos_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
 	struct omap2_hsmmc_info *c;
#if 0
	int ret;
#endif	
	printk(KERN_DEBUG "%s\n", __FUNCTION__);

#if 0
	ret = gpio_request(GPIO_MMC1_POWER, "mmc1_power");
	if (ret) {
		pr_err("Cannot request GPIO %d\n", GPIO_MMC1_POWER);
		goto error1;
	}
	
	gpio_export(GPIO_MMC1_POWER, 0);
	gpio_direction_output(GPIO_MMC1_POWER, 0);
	
	gpio_free(GPIO_MMC1_POWER);
#endif	

#if 0
	ret = gpio_request(GPIO_MMC1_DETECT, "mmc1_detect");
	if (ret) {
		pr_err("Cannot request GPIO %d\n", GPIO_MMC1_DETECT);
		goto error1;
	}
	
	gpio_export(GPIO_MMC1_DETECT, 0);
	gpio_direction_input(GPIO_MMC1_DETECT);
#endif
	/* card detect */
	controllers[1].gpio_cd = GPIO_MMC1_DETECT;	

	omap2_hsmmc_init(controllers);

	for (c = controllers; c->mmc; c++)
		archos_hsmmc_set_late_init(c->dev);

	return 0;
#if 0
error1:
	return -1;
#endif
}

void keyboard_mux_init(void)
{

}

static struct regulator_consumer_supply board_vdda_dac_supply = {
	.supply		= "vdda_dac",
#ifdef CONFIG_OMAP2_DSS
	.dev		= &board_dss_device.dev,
#endif
};

static struct regulator_consumer_supply board_avcc_supply = {
	.supply		= "avcc",
};

static struct regulator_init_data board_vdac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &board_vdda_dac_supply,
};

static struct regulator_init_data board_vmmc1 = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= 1,
		.apply_uV		= 1,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &board_avcc_supply,
};

static struct regulator_consumer_supply board_vdds_dsi_supply = {
	.supply	= "vdds_dsi",
};

static struct regulator_init_data vdds_dsi_initdata = {
	.consumer_supplies = &board_vdds_dsi_supply,
	.num_consumer_supplies = 1,
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static struct fixed_voltage_config board_vdds_dsi = {
	.supply_name		= "vdds_dsi",
	.microvolts		= 1800000,
	.gpio			= -1,
	.enable_high		= 1,
	.enabled_at_boot	= 1,
	.init_data		= &vdds_dsi_initdata,
};

static struct regulator_consumer_supply board_vmmc_ext_supply = {
	.supply	= "vmmc",
};

static struct regulator_init_data vmmc_ext_initdata = {
	.consumer_supplies = &board_vmmc_ext_supply,
	.num_consumer_supplies = 1,
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static struct fixed_voltage_config board_vmmc_ext_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3000000,
	.gpio			= GPIO_MMC1_POWER,
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &vmmc_ext_initdata,
};

static struct platform_device board_vmmc_ext_device = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &board_vmmc_ext_config,
	},
};

static struct platform_device board_vdds_dsi_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev.platform_data = &board_vdds_dsi,
};

#if 0
/* VOLUME UP/DOWN */
static int board_twl4030_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(1, 0, KEY_VOLUMEDOWN),
	0
};

static struct twl4030_keypad_data board_kp_twl4030_data = {
	.rows		= 1,
	.cols		= 2,
	.keymap		= board_twl4030_keymap,
	.keymapsize	= ARRAY_SIZE(board_twl4030_keymap),
	.rep		= 1,
};
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode			= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 100,
};

static struct twl4030_usb_data board_usb_data = {
	.platform = &archos_vbus_info,
	.usb_mode	= T2_USB_MODE_ULPI,
	.name = "archos_twl4030_usb_xceiv",
	.caps = TWL_USB_CAPS_CHARGING, 
};

static struct twl4030_madc_platform_data board_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_resconfig twl4030_rconfig[] = {
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3, .type = -1,
		.type2 = -1 },
/* XXX removed, breaks booting after power-off
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1, .type = -1,
		.type2 = -1 },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1, .type = -1,
		.type2 = -1 },
*/
	{ 0, 0},
};

static struct twl4030_ins __initdata sleep_on_seq[] = {

	/* Turn OFF VAUX2 */
	{MSG_SINGULAR(DEV_GRP_P1, RES_VAUX2, RES_STATE_OFF), 2},
	/* Turn off HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, RES_HFCLKOUT, RES_STATE_OFF), 2},
	/* Turn OFF VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD1, RES_STATE_OFF), 2},
	/* Turn OFF VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD2, RES_STATE_OFF), 2},
	/* Turn OFF VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, RES_VPLL1, RES_STATE_OFF), 2},
};

static struct twl4030_script sleep_on_script __initdata = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_p12_seq[] __initdata = {
	/* Turn on VAUX2 */
	{MSG_SINGULAR(DEV_GRP_P1, RES_VAUX2, RES_STATE_ACTIVE), 2},
	/* Turn on HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, RES_HFCLKOUT, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD1, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD2, RES_STATE_ACTIVE), 2},
	/* Turn ON VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, RES_VPLL1, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p12_script __initdata = {
	.script = wakeup_p12_seq,
	.size   = ARRAY_SIZE(wakeup_p12_seq),
	.flags  = TWL4030_WAKEUP12_SCRIPT,
};

static struct twl4030_ins wakeup_p3_seq[] __initdata = {
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p3_script __initdata = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset VDD1 regulator.
 * Reset VDD2 regulator.
 * Reset VPLL1 regulator.
 * Enable sysclk output.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD1, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD2, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VPLL1, RES_STATE_WRST), 0x60},
	{MSG_SINGULAR(DEV_GRP_P1, RES_HFCLKOUT, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VAUX2, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] __initdata = {
	&sleep_on_script,
	&wakeup_p12_script,
	&wakeup_p3_script,
	&wrst_script,
};

static struct twl4030_power_data board_t2scripts_data __initdata = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

static struct twl4030_platform_data board_tps65921_pdata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &board_madc_data,
	.usb		= &board_usb_data,
	//.keypad	= &board_kp_twl4030_data,
	.power		= &board_t2scripts_data,
	.vmmc1          = &board_vmmc1, /* used for VACC :-( */
	.vdac		= &board_vdac,
};

static struct omap_i2c_bus_board_data __initdata archos_i2c_bus_pdata;

static struct i2c_board_info __initdata board_i2c_bus1_info[] = {
	{
		I2C_BOARD_INFO("tps65921", 0x48),	// id to fix
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &board_tps65921_pdata,
	},
};

static struct i2c_board_info __initdata board_i2c_bus2_info[] = {
	[0] = {
		I2C_BOARD_INFO("wm8988", 0x1a),
		.flags = I2C_CLIENT_WAKE,
	},
	[1] = {
		I2C_BOARD_INFO("mma7660fc", 0x4c),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_mma7660fc_pdata,
	},
};

static int __init omap3_i2c_init(void)
{
	/* Disable OMAP 3630 internal pull-ups for I2Ci */
	if (cpu_is_omap3630()) {

		u32 prog_io;

		prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
		/* Program (bit 19)=1 to disable internal pull-up on I2C1 */
		prog_io |= OMAP3630_PRG_I2C1_PULLUPRESX;
		/* Program (bit 0)=1 to disable internal pull-up on I2C2 */
		prog_io |= OMAP3630_PRG_I2C2_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO2);
		/* Program (bit 7)=1 to disable internal pull-up on I2C3 */
		prog_io |= OMAP3630_PRG_I2C3_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO2);
		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO_WKUP1);
		/* Program (bit 5)=1 to disable internal pull-up on I2C4(SR) */
		prog_io |= OMAP3630_PRG_SR_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO_WKUP1);
	}

	omap_register_i2c_bus(1, 400, &archos_i2c_bus_pdata, board_i2c_bus1_info,
			ARRAY_SIZE(board_i2c_bus1_info));
	omap_register_i2c_bus(2, 100, &archos_i2c_bus_pdata, board_i2c_bus2_info,
			ARRAY_SIZE(board_i2c_bus2_info));

	return 0;
}

static struct platform_device *archos_devices[] __initdata = {
	&board_vmmc_ext_device,
	&board_vdds_dsi_device,
};

static void enable_board_wakeup_source(void)
{
	omap_mux_init_signal("sys_nirq",
		OMAP_WAKEUP_EN | OMAP_PIN_INPUT_PULLUP);
}

#if defined CONFIG_OMAP2_DSS
static void archos_init_dss(void)
{

}
#endif /* CONFIG_OMAP2_DSS */

#ifdef CONFIG_SERIAL_OMAP
static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
#if defined(CONFIG_SERIAL_OMAP_UART1_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART1_DMA,
		.dma_rx_buf_size = CONFIG_SERIAL_OMAP_UART1_RXDMA_BUFSIZE,
		.dma_rx_timeout	= CONFIG_SERIAL_OMAP_UART1_RXDMA_TIMEOUT,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART1_DMA */
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
#if defined(CONFIG_SERIAL_OMAP_UART2_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART2_DMA,
		.dma_rx_buf_size = CONFIG_SERIAL_OMAP_UART2_RXDMA_BUFSIZE,
		.dma_rx_timeout	= CONFIG_SERIAL_OMAP_UART2_RXDMA_TIMEOUT,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART2_DMA */
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
#if defined(CONFIG_SERIAL_OMAP_UART3_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART3_DMA,
		.dma_rx_buf_size = CONFIG_SERIAL_OMAP_UART3_RXDMA_BUFSIZE,
		.dma_rx_timeout	= CONFIG_SERIAL_OMAP_UART3_RXDMA_TIMEOUT,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART3_DMA */
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
#if defined(CONFIG_SERIAL_OMAP_UART4_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART4_DMA,
		.dma_rx_buf_size = CONFIG_SERIAL_OMAP_UART4_RXDMA_BUFSIZE,
		.dma_rx_timeout	= CONFIG_SERIAL_OMAP_UART4_RXDMA_TIMEOUT,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART3_DMA */
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.flags		= 0
	}
};
#endif /* CONFIG_SERIAL_OMAP */

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct omap_volt_pmic_info omap_pmic_core = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x61,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x18,
	.vp_vlimitto_vddmax = 0x30,
};

static struct omap_volt_pmic_info omap_pmic_mpu = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x55,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x18,
	.vp_vlimitto_vddmax = 0x3c,
};

static struct omap_volt_pmic_info omap_pmic_iva = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x5b,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x18,
	.vp_vlimitto_vddmax = 0x3c,
};

static struct omap_volt_vc_data vc_config = {
	.vdd0_on = 1350000,        /* 1.35v */
	.vdd0_onlp = 1350000,      /* 1.35v */
	.vdd0_ret = 837500,       /* 0.8375v */
	.vdd0_off = 600000,       /* 0.6v */
	.vdd1_on = 1100000,        /* 1.1v */
	.vdd1_onlp = 1100000,      /* 1.1v */
	.vdd1_ret = 837500,       /* 0.8375v */
	.vdd1_off = 600000,       /* 0.6v */
	.vdd2_on = 1100000,        /* 1.1v */
	.vdd2_onlp = 1100000,      /* 1.1v */
	.vdd2_ret = 837500,       /* .8375v */
	.vdd2_off = 600000,       /* 0.6v */
};

static void __init board_init(void)
{
	/* only an example for testing. we not really need the mac address here :-) */
	struct feature_tag_product_mac_address *mac_address = get_feature_tag(FTAG_PRODUCT_MAC_ADDRESS, feature_tag_size(feature_tag_product_mac_address));
	if (mac_address)
		printk(KERN_INFO "MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n", 
				mac_address->addr[0], mac_address->addr[1],
				mac_address->addr[2], mac_address->addr[3],
				mac_address->addr[4], mac_address->addr[5]);
	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

	omap3_mux_init(board_mux, OMAP_PACKAGE_CYN);

	if (panel_a35_init(&board_lcd_device) == 0)
		board_dss_data.default_device = &board_lcd_device;

	archos_accel_mma7660fc_init(&board_mma7660fc_pdata);
	omap3_i2c_init();

	platform_add_devices(archos_devices, ARRAY_SIZE(archos_devices));

	omap_serial_init(omap_serial_platform_data);
	ads7846_dev_init();

	omap3_leds_init();
	
#if defined CONFIG_OMAP2_DSS
	archos_init_dss();
#endif	
	archos_hsmmc_init(mmc);

	enable_board_wakeup_source();

	omap_display_init(&board_dss_data);

	archos_usb_musb_init(&archos_vbus_info);
	usb_musb_init(&musb_board_data);
	
	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
	omap_voltage_init_vc(&vc_config);

	archos_audio_wm8988_init();
}

static void __init board_map_io(void)
{
	omap2_set_globals_343x();
	omap34xx_map_common_io();
}

MACHINE_START(ARCHOS_A35, "Archos A35 board")
	.phys_io	= 0x48000000,
	.io_pg_offst    = ((0xfa000000) >> 18) & 0xfffc, 
	.boot_params	= 0x80000100,
	.map_io		= board_map_io,
	.init_irq	= board_init_irq,
	.init_machine	= board_init,
	.timer		= &omap_timer,
MACHINE_END
