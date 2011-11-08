/*
 * board-archos-a10it.c
 *
 *  Created on: Mar 15, 2010
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/board-archos.h>

#include <plat/common.h>
#include <plat/board.h>
#include <plat/usb.h>
#include <plat/opp_twl_tps.h>
#include <plat/timer-gp.h>
#include <plat/control.h>
#include <plat/omap-serial.h>
#include <plat/mmc.h>
#include <plat/display.h>
#include <plat/archos-audio-wm8988.h>

#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/i2c/twl.h>
#include <linux/mmc/host.h>

#include <mach/gpio.h>

#include <asm/io.h>
#include <asm/delay.h>

#include <linux/mma7660fc.h>
#include <linux/usb/gpio_vbus.h>

#include "mux.h"
#include "omap3-opp.h"
#include "smartreflex-class3.h"
#include "hsmmc.h"
//#include "sdram-elpida-edk2132c2pb.h"
#include "sdram-micron-mt46h32m32lf-6.h"

#define GPIO_VBUS_MUSB_PWRON    56	/* fixme: from config tags? */

#ifdef CONFIG_PM
static struct omap_volt_vc_data vc_config = {
	/* MPU */
	.vdd0_on	= 1200000, /* 1.2v */
	.vdd0_onlp	= 1000000, /* 1.0v */
	.vdd0_ret	=  975000, /* 0.975v */
	.vdd0_off	=  600000, /* 0.6v */
	/* CORE */
	.vdd1_on	= 1150000, /* 1.15v */
	.vdd1_onlp	= 1000000, /* 1.0v */
	.vdd1_ret	=  975000, /* 0.975v */
	.vdd1_off	=  600000, /* 0.6v */

	.clksetup	= 0xff,
	.voltoffset	= 0xff,
	.voltsetup2	= 0xff,
	.voltsetup_time1 = 0xfff,
	.voltsetup_time2 = 0xfff,
};

#ifdef CONFIG_TWL4030_CORE
static struct omap_volt_pmic_info omap_pmic_mpu = { /* and iva */
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x0, /* (vdd0) VDD1 -> VDD1_CORE -> VDD_MPU */
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
	.vp_vlimitto_vddmin = 0x14,
	.vp_vlimitto_vddmax = 0x44,
};

static struct omap_volt_pmic_info omap_pmic_core = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x1, /* (vdd1) VDD2 -> VDD2_CORE -> VDD_CORE */
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
	.vp_vlimitto_vddmax = 0x42,
};
#endif /* CONFIG_TWL4030_CORE */
#endif /* CONFIG_PM */

static struct mma7660fc_pdata board_mma7660fc_pdata;
static struct gpio_vbus_mach_info archos_vbus_info;
static struct archos_musb_config musb_config __initdata;

extern int __init archos_audio_gpio_init(void);

static void __init board_map_io(void)
{
	omap2_set_globals_36xx();
	omap34xx_map_common_io();
}
static void __init board_init_irq(void)
{
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
			     mt46h32m32lf6_sdrc_params);

	omap_init_irq();
	omap2_gp_clockevent_set_gptimer(1);
	sr_class3_init();
}

static struct archos_display_config display_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.lcd_pwon = 	157,
		.lvds_en = 	14,		// lvds_en
		.bkl_en = 	UNUSED_GPIO,
		.hdmi_pwr = 	21,
		.hdmi_int = 	110,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[1] = {
		.lcd_pwon = 	157,
		.lvds_en = 	14,
		.bkl_en = 	146,		// bkl_en
		.hdmi_pwr = 	21,
		.hdmi_int = 	110,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[2] = {
		.lcd_pwon = 	157,
		.lvds_en = 	14,
		.bkl_en = 	146,		// bkl_en
		.hdmi_pwr = 	21,
		.hdmi_int = 	110,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[3] = {
		.lcd_pwon = 	157,
		.lvds_en = 	14,
		.bkl_en = 	146,		// bkl_en
		.hdmi_pwr = 	21,
		.hdmi_int = 	110,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[4] = {
		.lcd_pwon = 	157,
		.lvds_en = 	14,
		.bkl_en = 	146,		// bkl_en
		.hdmi_pwr = 	21,
		.hdmi_int = 	110,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[5] = {
		.lcd_pwon = 	157,
		.lvds_en = 	14,
		.bkl_en = 	146,		// bkl_en
		.hdmi_pwr = 	21,
		.hdmi_int = 	110,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
        },
	.rev[6] = {
		.lcd_pwon = 	157,
		.lvds_en = 	14,
		.bkl_en = 	146,		// bkl_en
		.hdmi_pwr = 	21,
		.hdmi_int = 	110,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
};

static struct archos_audio_config audio_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[1] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[2] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[3] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[4] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[5] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[6] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
};

static struct archos_charge_config charge_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type   = CHARGER_DCIN,
	},
	.rev[1] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type   = CHARGER_DCIN,
	},
	.rev[2] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type   = CHARGER_DCIN,
	},
	.rev[3] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type   = CHARGER_DCIN,
	},
	.rev[4] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type   = CHARGER_DCIN,
	},
	.rev[5] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type   = CHARGER_DCIN,
	},
	.rev[6] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type   = CHARGER_DCIN,
	},
};

static struct archos_wifi_bt_config wifi_bt_dev_conf __initdata = {
	.nrev = 7,
	.rev[0] = {
		.wifi_power 	= 111,
		.wifi_irq 	= 114,
		.bt_power 	= 162,
	},
	.rev[1] = {
		.wifi_power 	= 111,
		.wifi_irq 	= 114,
		.bt_power 	= 162,
	},
	.rev[2] = {
		.wifi_power 	= 111,
		.wifi_irq 	= 114,
		.bt_power 	= 162,
	},
	.rev[3] = {
		.wifi_power 	= 111,
		.wifi_irq 	= 114,
		.bt_power 	= 162,
	},
	.rev[4] = {
		.wifi_power 	= 111,
		.wifi_irq 	= 114,
		.bt_power 	= 162,
		.wifi_pa_type	= PA_TYPE_TQM67002_NEW_BOM,
	},
	.rev[5] = {
		.wifi_power 	= 111,
		.wifi_irq 	= 114,
		.bt_power 	= 162,
		.wifi_pa_type	= PA_TYPE_RF3482,
	},
	.rev[6] = {
		.wifi_power 	= 111,
		.wifi_irq 	= 114,
		.bt_power 	= 162,
		.wifi_pa_type	= PA_TYPE_TQM67002A,
	},
};

static struct archos_accel_config accel_config __initdata = {
	.nrev = 7,
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
};

static struct archos_camera_config camera_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.pwr_down = 15,
		.reset = 16,
	},
	.rev[1] = {
		.pwr_down = 15,
		.reset = 16,	},
	.rev[2] = {
		.pwr_down = 15,
		.reset = 16,	},
	.rev[3] = {
		.pwr_down = 15,
		.reset = 16,	},
	.rev[4] = {
		.pwr_down = 15,
		.reset = 16,	},
	.rev[5] = {
		.pwr_down = 15,
		.reset = 16,	},
	.rev[6] = {
		.pwr_down = 15,
		.reset = 16,
	},
};

static struct archos_leds_config leds_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.backlight_power = 23,
		.bkl_freq = 200,
		.bkl_max = 254,
		.pwr_invert = 1,
		.bkl_invert = 1,
		.status_led = UNUSED_GPIO,
	},
	.rev[1] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.backlight_power = 23,
		.bkl_freq = 20000,
		.bkl_max = 254,
		.pwr_invert = 1,
		.bkl_invert = 1,
		.status_led = UNUSED_GPIO,
	},
	.rev[2] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.backlight_power = 23,
		.bkl_freq = 20000,
		.bkl_max = 254,
		.bkl_invert = 1,
		.pwr_invert = 1,
		.status_led = UNUSED_GPIO,
	},
	.rev[3] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.backlight_power = 23,
		.bkl_freq = 20000,
		.bkl_max = 254,
		.bkl_invert = 1,
		.pwr_invert = 1,
		.status_led = UNUSED_GPIO,
	},
	.rev[4] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.backlight_power = 23,
		.bkl_freq = 20000,
		.bkl_max = 254,
		.bkl_invert = 1,
		.pwr_invert = 1,
		.status_led = UNUSED_GPIO,
	},
	.rev[5] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.backlight_power = 23,
		.bkl_freq = 20000,
		.bkl_max = 254,
		.bkl_invert = 1,
		.pwr_invert = 1,
		.status_led = UNUSED_GPIO,
	},
	.rev[6] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.backlight_power = 23,
		.bkl_freq = 20000,
		.bkl_max = 254,
		.bkl_invert = 1,
		.pwr_invert = 1,
		.status_led = UNUSED_GPIO,
	},
};

static struct archos_sd_config sd_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.sd_power 	= 158,
		.sd_detect 	= 65,
		.sd_prewarn 	= 58,
	},
	.rev[1] = {
		.sd_power 	= 158,
		.sd_detect 	= 65,
		.sd_prewarn 	= 58,
	},
	.rev[2] = {
		.sd_power 	= 158,
		.sd_detect 	= 65,
		.sd_prewarn 	= 58,
	},
	.rev[3] = {
		.sd_power 	= 158,
		.sd_detect 	= 65,
		.sd_prewarn 	= 58,
	},
	.rev[4] = {
		.sd_power 	= 158,
		.sd_detect 	= 65,
		.sd_prewarn 	= 58,
	},
	.rev[5] = {
		.sd_power 	= 158,
		.sd_detect 	= 65,
		.sd_prewarn 	= 58,
	},
	.rev[6] = {
		.sd_power 	= 158,
		.sd_detect 	= 65,
		.sd_prewarn 	= 58,
	},
};

static struct archos_fsusb_config fsusb_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.suspend = UNUSED_GPIO,
		.enable_usb_ohci = UNUSED_GPIO,
	},
	.rev[1] = {
		.suspend = 27,
		.enable_usb_ohci = UNUSED_GPIO,
	},
	.rev[2] = {
		.suspend = 27,
		.enable_usb_ohci = UNUSED_GPIO,
	},
	.rev[3] = {
		.suspend = 27,
		.enable_usb_ohci = UNUSED_GPIO,
	},
	.rev[4] = {
		.suspend = 27,
		.enable_usb_ohci = UNUSED_GPIO,
	},
	.rev[5] = {
		.suspend = 27,
		.enable_usb_ohci = UNUSED_GPIO,
	},
	.rev[6] = {
		.suspend = 27,
		.enable_usb_ohci = UNUSED_GPIO,
	},
};

static struct archos_usb_tsp_config usb_tsp_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.enable = UNUSED_GPIO,
		.reset = UNUSED_GPIO,
	},
	.rev[1] = {
		.enable = 26,
		.reset = 25,
		.suspend_flags = USB_TSP_FLAGS_EARLY_POWER_OFF | USB_TSP_FLAGS_RELEASE_WA,
	},
	.rev[2] = {
		.enable = 26,
		.reset = 25,
		.suspend_flags = USB_TSP_FLAGS_EARLY_POWER_OFF | USB_TSP_FLAGS_RELEASE_WA,
	},
	.rev[3] = {
		.enable = 26,
		.reset = 25,
		.suspend_flags = USB_TSP_FLAGS_EARLY_POWER_OFF,
	},
	.rev[4] = {
		.enable = 26,
		.reset = 25,
		.suspend_flags = USB_TSP_FLAGS_EARLY_POWER_OFF,
	},
	.rev[5] = {
		.enable = 26,
		.reset = 25,
		.suspend_flags = USB_TSP_FLAGS_EARLY_POWER_OFF,
	},
	.rev[6] = {
		.enable = 26,
		.reset = 25,
		.suspend_flags = USB_TSP_FLAGS_EARLY_POWER_OFF,
	},
};

static struct archos_usb_config usb_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.usb_max_power = 500,
	},
	.rev[1] = {
		.usb_max_power = 500,
	},
	.rev[2] = {
		.usb_max_power = 500,
	},
	.rev[3] = {
		.usb_max_power = 500,
	},
	.rev[4] = {
		.usb_max_power = 500,
	},
	.rev[5] = {
		.usb_max_power = 500,
	},
	.rev[6] = {
		.usb_max_power = 500,
	},
};

static struct archos_usb_gadget_config gadget_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.product_name = "A101IT",
		.product_id = 0x1419,
		.ums_luns = 2,
	},
	.rev[1] = {
		.product_name = "A101IT",
		.product_id = 0x1419,
		.ums_luns = 2,
	},
	.rev[2] = {
		.product_name = "A101IT",
		.product_id = 0x1419,
		.ums_luns = 2,
	},
	.rev[3] = {
		.product_name = "A101IT",
		.product_id = 0x1419,
		.ums_luns = 2,
	},
	.rev[4] = {
		.product_name = "A101IT",
		.product_id = 0x1419,
		.ums_luns = 2,
	},
	.rev[5] = {
		.product_name = "A101IT",
		.product_id = 0x1419,
		.ums_luns = 2,
	},
	.rev[6] = {
		.product_name = "A101IT",
		.product_id = 0x1419,
		.ums_luns = 2,
	},
	.rev[7] = {
		.product_name = "A101IT",
		.product_id = 0x1419,
		.ums_luns = 2,
	},
};

static struct omap_board_config_kernel board_config[] __initdata = {
	{ ARCHOS_TAG_DISPLAY,	&display_config },
	{ ARCHOS_TAG_CHARGE,    &charge_config},
	{ ARCHOS_TAG_AUDIO,     &audio_config},
	{ ARCHOS_TAG_WIFI_BT,	&wifi_bt_dev_conf},
	{ ARCHOS_TAG_ACCEL,	&accel_config},
	{ ARCHOS_TAG_CAMERA,	&camera_config},
	{ ARCHOS_TAG_LEDS,	&leds_config},
	{ ARCHOS_TAG_SD,	&sd_config},
	{ ARCHOS_TAG_USB_TSP,	&usb_tsp_config},
	{ ARCHOS_TAG_USB,       &usb_config},
	{ ARCHOS_TAG_FSUSB,	&fsusb_config},
	{ ARCHOS_TAG_USB_GADGET, &gadget_config},
	{ ARCHOS_TAG_MUSB,	&musb_config},
};

#ifdef CONFIG_OMAP2_DSS
static struct omap_dss_device board_lcd_device;

static struct omap_dss_device board_hdmi_device = {
	.name = "hdmi",
	.driver_name = "nxp_hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.platform_enable = NULL,
	.platform_disable = NULL,
};

static struct omap_dss_device *board_dss_devices[] = {
	&board_lcd_device,
	&board_hdmi_device,
};

static struct omap_dss_board_info board_dss_data = {
	.num_devices = ARRAY_SIZE(board_dss_devices),
	.devices = board_dss_devices,
	.default_device = &board_lcd_device,
};
#endif

static struct regulator_consumer_supply board_vdda_dac_supply = {
	.supply		= "vdda_dac",
#ifdef CONFIG_OMAP2_DSS
	/*FIXME: .dev		= &board_dss_device.dev, */
#endif
};

static struct regulator_consumer_supply board_avcc_supply = {
	.supply		= "avcc",
};

static struct regulator_consumer_supply board_vdd_cam_supply[] = {
	REGULATOR_SUPPLY("vdd_cam", "omap-ov7675-isp"),
};

/* consumer for vdds_dsi which is permanently enabled */
static struct regulator_consumer_supply board_vdds_dsi_supply = {
	.supply		= "vdds_dsi",
#ifdef CONFIG_OMAP2_DSS
	/*FIXME .dev		= &board_dss_device.dev, */
#endif
};

/* ------ TPS65921 init data ---------- */

#define TWL4030_MSECURE_GPIO	159

static int __init msecure_init(void)
{
	int ret = 0;

#ifdef CONFIG_RTC_DRV_TWL4030
	void __iomem *msecure_pad_config_reg =
		omap_ctrl_base_get() + 0x192;
	int mux_mask = 0x04;
	u16 tmp;

	ret = gpio_request(TWL4030_MSECURE_GPIO, "msecure");
	if (ret < 0) {
		printk(KERN_ERR "msecure_init: can't"
			"reserve GPIO:%d !\n", TWL4030_MSECURE_GPIO);
		goto out;
	}
	/*
	  * TWL4030 will be in secure mode if msecure line from OMAP
	  * is low. Make msecure line high in order to change the
	  * TWL4030 RTC time and calender registers.
	  */

	tmp = __raw_readw(msecure_pad_config_reg);
	tmp &= 0xF8;	/* To enable mux mode 03/04 = GPIO_RTC */
	tmp |= mux_mask;/* To enable mux mode 03/04 = GPIO_RTC */
	__raw_writew(tmp, msecure_pad_config_reg);

	gpio_direction_output(TWL4030_MSECURE_GPIO, 1);
out:
#endif
	return ret;
}

/* VOLUME UP/DOWN */
static int board_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_VOLUMEDOWN),
};

static struct matrix_keymap_data board_map_data = {
	.keymap		= board_keymap,
	.keymap_size	= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data board_kp_twl4030_data = {
	.keymap_data	= &board_map_data,
	.rows		= 1,
	.cols		= 2,
	.rep		= 1,
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

static struct regulator_init_data board_vaux2 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= 1,
		.apply_uV		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(board_vdd_cam_supply),
	.consumer_supplies	= board_vdd_cam_supply,
};

static struct regulator_init_data board_vintana1 = {
	.constraints = {
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.apply_uV		= 1,
	},
};

static struct twl4030_usb_data board_usb_data = {
	.platform	= &archos_vbus_info,
	.usb_mode	= T2_USB_MODE_ULPI,
	.name		= "archos_twl4030_usb_xceiv", 
	.caps 		= 0, 
};

static int archos_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	// dc_detect is pmic's gpio_0
	struct archos_charge_conf *chg_conf;
	chg_conf = hwrev_ptr(&charge_config, hardware_rev);

	chg_conf->gpio_dc_detect = gpio + 0;

	/* USB_ID */
	if ( hardware_rev > 1 ) {
		/* USB_ID also on PMIC GPIO_1 */		
		/* we seem to be able to modify the plat data from here */
		musb_config.nrev = 7;
		musb_config.rev[hardware_rev].gpio_id = gpio + 1;
		musb_config.rev[hardware_rev].gpio_vbus_detect = UNUSED_GPIO;
		musb_config.rev[hardware_rev].gpio_vbus_flag = UNUSED_GPIO;
	
		archos_usb_musb_init(&archos_vbus_info);
	}

	return 0;
}

static struct twl4030_gpio_platform_data board_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= archos_twl_gpio_setup,
};

static struct twl4030_madc_platform_data board_madc_data = {
	.irq_line	= 1,
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
	{MSG_SINGULAR(DEV_GRP_P1, RES_HFCLKOUT, RES_STATE_ACTIVE), 2},
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

static struct twl4030_resconfig twl4030_rconfig_vdd_cam_off[] = {
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3, .type = -1,
		.type2 = -1 },
 	{ .resource = RES_VAUX2, .devgroup = DEV_GRP_P1, .type = -1,
		.type2 = -1 },
	{ 0, 0},
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
	.gpio		= &board_gpio_data,
	.keypad		= &board_kp_twl4030_data,
	.power		= &board_t2scripts_data,
	.vmmc1          = &board_vmmc1,
	.vdac		= &board_vdac,
	.vaux2          = &board_vaux2,
 	.vintana1       = &board_vintana1,
};

static void remux_regulator_gpio(int gpio)
{
	omap_mux_init_gpio(gpio, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
}

/*
 * supply vbus_musb, consumer of 5V
 */

static struct regulator_consumer_supply fixed_reg_vbus_musb_consumer[] = {
	REGULATOR_SUPPLY("vbus_musb", "archos_twl4030_usb_xceiv"),
};

static struct regulator_init_data fixed_reg_vbus_musb_initdata = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	
	.supply_regulator 	= "5V", 

	.consumer_supplies      = fixed_reg_vbus_musb_consumer,
	.num_consumer_supplies  = ARRAY_SIZE(fixed_reg_vbus_musb_consumer),
};

static struct fixed_voltage_config fixed_reg_vbus_musb = {
	.supply_name		= "vbus_musb",
	.microvolts		= 5000000,
	.gpio			= GPIO_VBUS_MUSB_PWRON,
	.enable_high		= 0,
	.enabled_at_boot	= 0,
	.init_data		= &fixed_reg_vbus_musb_initdata,
	.remux			= remux_regulator_gpio,
};

static struct platform_device fixed_supply_vbus_musb = {
	.name	= "reg-fixed-voltage",
	.id	= 4,
	.dev	= {
		.platform_data = &fixed_reg_vbus_musb,
	},
};

/* fixed dummy regulator for 1.8v vdds_dsi rail */
static struct regulator_init_data board_vdds_dsi = {
	.constraints = {
		.name			= "vdds_dsi",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.always_on		= 1,
		.boot_on		= 1,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &board_vdds_dsi_supply,
};
static struct fixed_voltage_config board_vdds_dsi_config = {
	.supply_name	= "vdds_dsi",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &board_vdds_dsi,
};
static struct platform_device board_vdds_dsi_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev.platform_data = &board_vdds_dsi_config,
};

static struct regulator_consumer_supply board_vmmc_ext_supply[] = {
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.1"),
};
static struct regulator_init_data board_vmmc_ext = {
	.constraints = {
		.name 			= "vmmc",
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.always_on		= 0,
		.boot_on		= 0,
	},
	.num_consumer_supplies	= ARRAY_SIZE(board_vmmc_ext_supply),
	.consumer_supplies	= board_vmmc_ext_supply,
};
static struct fixed_voltage_config board_vmmc_ext_config = {
	.supply_name	= "vmmc_ext",
	.microvolts	= 3000000,
	.gpio		= -EINVAL,
	.init_data	= &board_vmmc_ext,
};
static struct platform_device board_vmmc_ext_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev.platform_data = &board_vmmc_ext_config
};

static struct regulator_consumer_supply board_vmmc2_supply[] = {
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.0"),
};
static struct regulator_init_data board_vmmc2 = {
	.constraints = {
		.name 			= "vmmc",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.always_on		= 1,
		.boot_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(board_vmmc2_supply),
	.consumer_supplies	= board_vmmc2_supply,
};
static struct fixed_voltage_config board_vmmc2_config = {
	.supply_name	= "vmmc2",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &board_vmmc2,
};
static struct platform_device board_vmmc2_device = {
	.name		= "reg-fixed-voltage",
	.id		= 2,
	.dev.platform_data = &board_vmmc2_config,
};

/* 5V power rail */
#define GPIO_5V_ON	144
static struct regulator_consumer_supply board_fixed_5v_supply[] = {
	REGULATOR_SUPPLY("hsusb1", "uhhtll-omap"),
};
static struct regulator_init_data board_fixed_5v_init = {
	.constraints = {
		.name 			= "5V",
		.min_uV			= 5000000,
		.max_uV			= 5000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= 0,
		.boot_on		= 0,
	},
	.num_consumer_supplies	= ARRAY_SIZE(board_fixed_5v_supply),
	.consumer_supplies	= board_fixed_5v_supply,
};
static struct fixed_voltage_config board_fixed_5v_config = {
	.supply_name	= "5V",
	.microvolts	= 5000000,
	.gpio		= GPIO_5V_ON,
	.enable_high	= 1,
	.enabled_at_boot= 0,
	.init_data	= &board_fixed_5v_init,
	.remux		= remux_regulator_gpio,
};
static struct platform_device board_fixed_5v_device = {
	.name		= "reg-fixed-voltage",
	.id		= 3,
	.dev.platform_data = &board_fixed_5v_config,
};

static struct i2c_board_info __initdata board_i2c_bus1_info[] = {
	{
		I2C_BOARD_INFO("tps65921", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &board_tps65921_pdata,
	},
};

static void archos_hdmi_power(struct i2c_client *cl, int on_off)
{
	struct device *dev = &cl->dev;
	struct archos_hdmi_platform_data *pdata = dev->platform_data;
	
	if (on_off)
		gpio_direction_output(pdata->pwr_gpio, 1);
	else
		gpio_direction_input(pdata->pwr_gpio);
}

static struct archos_hdmi_platform_data board_hdmi_pdata = {
	.hdmi_pwr = archos_hdmi_power,
};

static struct i2c_board_info __initdata board_i2c_bus2_info[] = {
	[0] = {
		I2C_BOARD_INFO("wm8988", 0x1a),
		.flags = I2C_CLIENT_WAKE,
	},
	[1] = {
		I2C_BOARD_INFO("tda998X", 0x70),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_hdmi_pdata,
	},
	[2] = {
		I2C_BOARD_INFO("tda99Xcec", 0x34),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_hdmi_pdata,
	},
	[3] = {
		I2C_BOARD_INFO("mma7660fc", 0x4c),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_mma7660fc_pdata,
	}
};

static struct i2c_board_info __initdata board_i2c_bus3_info[] = {
	// nothing on i2c3
};

static struct omap_i2c_bus_board_data __initdata archos_i2c_bus_pdata;

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
	omap_register_i2c_bus(3, 100, &archos_i2c_bus_pdata, board_i2c_bus3_info,
			ARRAY_SIZE(board_i2c_bus3_info));

	return 0;
}

static struct platform_device *board_devices[] __initdata = {
	&board_vdds_dsi_device,
	&board_vmmc_ext_device,
	&board_vmmc2_device,
	&board_fixed_5v_device,
	&fixed_supply_vbus_musb,
};

static void __init archos_hdmi_gpio_init(
		const struct archos_disp_conf* disp_conf)
{
	int ret;
	/* driver will manage the GPIO, just apply the pin multiplexing
	 * archos_gpio_init_input(&disp_conf->hdmi_int, "hdmi irq"); */
	omap_mux_init_gpio(disp_conf->hdmi_int, OMAP_PIN_INPUT);
	
	/* FIXME: make userspace configurable */
	ret = gpio_request(disp_conf->hdmi_pwr, "hdmi_pwr");
	if (!IS_ERR_VALUE(ret))
		gpio_direction_output(disp_conf->hdmi_pwr, 1);
	/* FIXME: make userspace configurable */
	gpio_set_value(disp_conf->hdmi_pwr, 0);

	/* patch power gpio into platform data */
	board_hdmi_pdata.pwr_gpio = disp_conf->hdmi_pwr;
	
	/* patch IRQ into HDMI I2C bus info */
	board_i2c_bus2_info[1].irq = gpio_to_irq(disp_conf->hdmi_int);
	board_i2c_bus2_info[2].irq = gpio_to_irq(disp_conf->hdmi_int);
}

static int __init wl127x_vio_leakage_fix(void)
{
	int ret = 0;
	const struct archos_wifi_bt_config *conf = &wifi_bt_dev_conf;
	const struct archos_wifi_bt_dev_conf *cfg;
	int bten_gpio;
	
	cfg = hwrev_ptr(conf, hardware_rev);
	if (IS_ERR(cfg))
		return -ENODEV;
	
	bten_gpio = cfg->bt_power;
	
	ret = gpio_request(bten_gpio, "wl127x_bten");
	if (IS_ERR_VALUE(ret)) {
		pr_err("wl127x_bten gpio_%d request fail", bten_gpio);
		goto fail;
	}

	gpio_direction_output(bten_gpio, 1);
	mdelay(10);
	gpio_direction_output(bten_gpio, 0);
	udelay(64);

	gpio_free(bten_gpio);

fail:
	return ret;
}

static int __init omap3_leds_init(void)
{
	omap_mux_init_signal("gpmc_ncs7.gpt8_pwm_evt", 
			OMAP_PIN_OUTPUT);

	return 0;
}

static struct omap2_hsmmc_info mmc[] __initdata = {
	{
		.mmc		= 2,
		.name		= "internal",
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.nonremovable	= true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif		
	},
	{
		.mmc		= 1,
		.name		= "external",
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif		
	},
	{
		.mmc		= 3,
		.name		= "sdio",
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif		
	},
	{}      /* Terminator */
};

static int archos_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

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

static int archos_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
 	const struct archos_sd_conf *cfg;
 	struct omap2_hsmmc_info *c;

 	cfg = hwrev_ptr(&sd_config, hardware_rev);
	if (IS_ERR(cfg)) {
		pr_err("%s: no board configuration found\n", __func__);
		return -ENODEV;
	}
	
	controllers[1].gpio_cd = cfg->sd_detect;
	
	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		archos_hsmmc_set_late_init(c->dev);


	return 0;
}

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

#ifdef CONFIG_SERIAL_OMAP
static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
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
	OMAP3_MUX(DSS_HSYNC, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_PCLK, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_VSYNC, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_ACBIAS, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	/* safe_mode */
	OMAP3_MUX(DSS_DATA0, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7),
	/* safe_mode */
	OMAP3_MUX(DSS_DATA1, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7),
	/* safe_mode */
	OMAP3_MUX(DSS_DATA2, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7),
	/* safe_mode */
	OMAP3_MUX(DSS_DATA3, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7),
	/* safe_mode */
	OMAP3_MUX(DSS_DATA4, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7),
	/* safe_mode */
	OMAP3_MUX(DSS_DATA5, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7),
	OMAP3_MUX(DSS_DATA6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_DATA7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_DATA8, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_DATA9, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_DATA10, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_DATA11, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_DATA12, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_DATA13, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_DATA14, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_DATA15, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_DATA16, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP3_MUX(DSS_DATA17, OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	/* dss_data0 */
	OMAP3_MUX(DSS_DATA18, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dss_data1 */
	OMAP3_MUX(DSS_DATA19, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dss_data2 */
	OMAP3_MUX(DSS_DATA20, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dss_data3 */
	OMAP3_MUX(DSS_DATA21, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dss_data4 */
	OMAP3_MUX(DSS_DATA22, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3), 
	/* dss_data5 */
	OMAP3_MUX(DSS_DATA23, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dss_data18 */
	OMAP3_MUX(SYS_BOOT0, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dss_data19 */
	OMAP3_MUX(SYS_BOOT1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dss_data20 */
	OMAP3_MUX(SYS_BOOT3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dss_data21 */
	OMAP3_MUX(SYS_BOOT4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dss_data22 */
	OMAP3_MUX(SYS_BOOT5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dss_data23 */
	OMAP3_MUX(SYS_BOOT6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static void enable_board_wakeup_source(void)
{
	omap_mux_init_signal("sys_nirq", 
			OMAP_WAKEUP_EN | OMAP_PIN_INPUT_PULLUP);
}

static void board_offmode_config(void)
{
	gpio_request(161, "pwren2");
	//omap_cfg_reg( K26_OMAP34XX_GPIO161_OFF_IPU ); // PWREN2 test
	gpio_direction_input(161); // we have a pull-down while active and a pull-up while in OFF mode

	/* if we can turn off the 1.8V, we need to turn off the VDD_CAM, too */
	board_tps65921_pdata.power->resource_config = twl4030_rconfig_vdd_cam_off;
}

static void __init board_init(void)
{
	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);

	omap3_leds_init();	// set it here mask hugly transitions

	/* before omap_i2c_init() or IRQ will not forwarded to driver */
	if (display_config.nrev > hardware_rev)
		archos_hdmi_gpio_init(&display_config.rev[hardware_rev]);

	msecure_init();

	omap3_i2c_init();

	panel_boe_wsvga_10_init(&board_lcd_device);

#ifdef CONFIG_SERIAL_OMAP
	omap_serial_init(omap_serial_platform_data);
#endif
	usb_musb_init(&musb_board_data);

	archos_accel_mma7660fc_init(&board_mma7660fc_pdata);

	archos_camera_ov7675_init();

	/* before hsubs init, otherwise 5V regulator is no found */
	platform_add_devices(board_devices, ARRAY_SIZE(board_devices));
	
	omap_display_init(&board_dss_data);
	archos_hsmmc_init(mmc);
	archos_omap3_uhhtll_init();
	archos_audio_wm8988_init();
	enable_board_wakeup_source();

#ifdef CONFIG_PM
#ifdef CONFIG_TWL4030_CORE
	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
#endif
	omap_voltage_init_vc(&vc_config);
#endif

	/* Fix to prevent VIO leakage on wl127x */
	wl127x_vio_leakage_fix();
}

/* OPP MPU/IVA Clock Frequency */
struct opp_frequencies {
	unsigned long mpu;
	unsigned long iva;
};

static struct opp_frequencies opp_freq_add_table[] __initdata = {
  {
	.mpu = 800000000,
	.iva = 660000000,
  },
  {
	.mpu = 1000000000,
	.iva =  800000000,
  },
#if 0
  1.2GHz has been observed to cause issues on ES1.1 boards and requires
  further investigation.
  {
	.mpu = 1200000000,
	.iva =   65000000,
  },
#endif

  { 0, 0 },
};

/* must be called after omap2_common_pm_init() */
static int __init a10it_opp_init(void)
{
	struct omap_hwmod *mh, *dh;
	struct omap_opp *mopp, *dopp;
	struct device *mdev, *ddev;
	struct opp_frequencies *opp_freq;


	if (!cpu_is_omap3630())
		return 0;

	mh = omap_hwmod_lookup("mpu");
	if (!mh || !mh->od) {
		pr_err("%s: no MPU hwmod device.\n", __func__);
		return 0;
	}

	dh = omap_hwmod_lookup("iva");
	if (!dh || !dh->od) {
		pr_err("%s: no DSP hwmod device.\n", __func__);
		return 0;
	}

	mdev = &mh->od->pdev.dev;
	ddev = &dh->od->pdev.dev;

	/* add MPU and IVA clock frequencies */
	for (opp_freq = opp_freq_add_table; opp_freq->mpu; opp_freq++) {
		/* check enable/disable status of MPU frequecy setting */
		mopp = opp_find_freq_exact(mdev, opp_freq->mpu, false);
		if (IS_ERR(mopp))
			mopp = opp_find_freq_exact(mdev, opp_freq->mpu, true);
		if (IS_ERR(mopp)) {
			pr_err("%s: MPU does not support %lu MHz\n", __func__, opp_freq->mpu / 1000000);
			continue;
		}

		/* check enable/disable status of IVA frequency setting */
		dopp = opp_find_freq_exact(ddev, opp_freq->iva, false);
		if (IS_ERR(dopp))
			dopp = opp_find_freq_exact(ddev, opp_freq->iva, true);
		if (IS_ERR(dopp)) {
			pr_err("%s: DSP does not support %lu MHz\n", __func__, opp_freq->iva / 1000000);
			continue;
		}

		/* try to enable MPU frequency setting */
		if (opp_enable(mopp)) {
			pr_err("%s: OPP cannot enable MPU:%lu MHz\n", __func__, opp_freq->mpu / 1000000);
			continue;
		}

		/* try to enable IVA frequency setting */
		if (opp_enable(dopp)) {
			pr_err("%s: OPP cannot enable DSP:%lu MHz\n", __func__, opp_freq->iva / 1000000);
			opp_disable(mopp);
			continue;
		}

		/* verify that MPU and IVA frequency settings are available */
		mopp = opp_find_freq_exact(mdev, opp_freq->mpu, true);
		dopp = opp_find_freq_exact(ddev, opp_freq->iva, true);
		if (!mopp || !dopp) {
			pr_err("%s: OPP requested MPU: %lu MHz and DSP: %lu MHz not found\n",
				__func__, opp_freq->mpu / 1000000, opp_freq->iva / 1000000);
			continue;
		}

		dev_info(mdev, "OPP enabled %lu MHz\n", opp_freq->mpu / 1000000);
		dev_info(ddev, "OPP enabled %lu MHz\n", opp_freq->iva / 1000000);
	}

	return 0;
}
device_initcall(a10it_opp_init);


MACHINE_START(ARCHOS_A101IT, "Archos A101IT board")
	.phys_io	= 0x48000000,
	.io_pg_offst    = ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= board_map_io,
	.fixup		= fixup_archos,
	.init_irq	= board_init_irq,
	.init_machine	= board_init,
	.timer		= &omap_timer,
MACHINE_END
