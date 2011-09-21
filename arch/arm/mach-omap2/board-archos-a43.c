/*
 * board-archos-a43.c
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
#include <plat/archos-audio-wm8988.h>
#include <mach/board-archos.h>

#include <linux/mma7660fc.h>

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

static struct archos_audio_config audio_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 170,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[1] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 170,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[2] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 170,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[3] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 170,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[4] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 170,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[5] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 170,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
};

static struct archos_accel_config accel_config __initdata = {
	.nrev = 6,
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
};

static struct archos_camera_config camera_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.pwr_down =144,
		.reset = 178,
	},
	.rev[1] = {
		.pwr_down =129,
		.reset = 178,
	},
	.rev[2] = {
		.pwr_down =144,
		.reset = 178,
	},
	.rev[3] = {
		.pwr_down =144,
		.reset = 178,
	},
	.rev[4] = {
		.pwr_down =144,
		.reset = 178,
	},
	.rev[5] = {
		.pwr_down =144,
		.reset = 178,
	},
};
static struct archos_leds_config leds_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.backlight_power = UNUSED_GPIO,
	},
	.rev[1] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 200,
		.backlight_power = UNUSED_GPIO,
	},
	.rev[2] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 200,
		.backlight_power = UNUSED_GPIO,
	},
	.rev[3] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 200,
		.backlight_power = UNUSED_GPIO,
	},
	.rev[4] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 200,
		.backlight_power = UNUSED_GPIO,
	},
	.rev[5] = {
		.power_led = 57,
		.backlight_led = { .timer = 8, .mux_cfg = -1 },
		.bkl_max = 200,
		.backlight_power = UNUSED_GPIO,
	},
};

static struct omap_board_config_kernel board_config[] __initdata = {
	{ ARCHOS_TAG_AUDIO,     &audio_config},
	{ ARCHOS_TAG_ACCEL,	&accel_config},
	{ ARCHOS_TAG_CAMERA,	&camera_config},
	{ ARCHOS_TAG_LEDS,	&leds_config},
};

#ifdef CONFIG_OMAP2_DSS
#define LCD_PANEL_ENABLE_GPIO		157
#define LCD_PANEL_RESET_GPIO		25

struct board_dss_board_info {
	int gpio_flag;
};

static int board_panel_power_enable(int enable)
{
	int ret;
	struct regulator *vdds_dsi_reg;

	vdds_dsi_reg = regulator_get(NULL, "vdds_dsi");
	if (IS_ERR(vdds_dsi_reg)) {
		pr_err("Unable to get vdds_dsi regulator\n");
		return PTR_ERR(vdds_dsi_reg);
	}

	if (enable)
		ret = regulator_enable(vdds_dsi_reg);
	else
		ret = regulator_disable(vdds_dsi_reg);

	return ret;
}

static int board_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	int ret;
	struct board_dss_board_info *pdata;

	pr_info("board_panel_enable_lcd [%s]\n", dssdev->name);

	ret = board_panel_power_enable(1);
	if (ret < 0)
		return ret;
	pdata = dssdev->dev.platform_data;
	if (pdata->gpio_flag == 0) {
		ret = gpio_request(LCD_PANEL_ENABLE_GPIO, "lcd enable");
		if (ret) {
			pr_err("Failed to get LCD_PANEL_ENABLE_GPIO.\n");
			return ret;
		}
		gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 0);
		ret = gpio_request(LCD_PANEL_RESET_GPIO, "lcd reset");
		if (ret) {
			pr_err("Failed to get LCD_PANEL_ENABLE_GPIO.\n");
			return ret;
		}
		gpio_direction_output(LCD_PANEL_RESET_GPIO, 1);
		pdata->gpio_flag = 1;
	}

	gpio_set_value(LCD_PANEL_RESET_GPIO, 1);
	gpio_set_value(LCD_PANEL_ENABLE_GPIO, 1);
	msleep(50);
	gpio_set_value(LCD_PANEL_RESET_GPIO, 0);
	msleep(50);		// min 10ms
	gpio_set_value(LCD_PANEL_RESET_GPIO, 1);
	msleep(50);		// min 10ms

	return 0;
}

static void board_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	pr_debug("board_panel_disable_lcd [%s]\n", dssdev->name);

	board_panel_power_enable(0);
	gpio_set_value(LCD_PANEL_RESET_GPIO, 0);
	gpio_set_value(LCD_PANEL_ENABLE_GPIO, 0);
}

static struct board_dss_board_info board_dss_lcd_data = {
	.gpio_flag = 0,
};

static struct omap_dss_device board_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DSI,
	.name = "lcd",
	.driver_name = "lg_fwvga_43",
	.phy.dsi = {
		.clk_lane = 1,			// at pos 1
		.clk_pol = 0,			// dx = + , dy = -
		.data1_lane = 3,		// dx2 = + , dy2 = -
		.data1_pol = 0,			// dx = + , dy = -
		.data2_lane = 2,		// dx1 = + , dy1 = -
		.data2_pol = 0,			// dx = + , dy = -
//		.lp_clk_hz = 5400000,		
//		.ddr_clk_hz = 162000000, 	/* for NDL=2, 24bpp */
		.div		= {
			.lck_div	= 1,
			.pck_div	= 5,
			.regm		= 150,
			.regn		= 17,
			.regm_dispc	= 4,
			.regm_dsi	= 4,
			.lp_clk_div	= 8,
		},
	},
	.ctrl = {
		.pixel_size = 24,
	},
	.panel = {
		.config = (OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_IVS |
				OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF),
		/* timings for NDL=2, RGB888 */
		.timings = {
			.x_res		= 480,
			.y_res		= 854,
			// 60 hz
			.pixel_clock	= 27000,	/* 27Mhz, (ddr_clk_hz/4)* 2/3 */
			.hsw		= 0,		/* horizontal sync pulse width */
			.hfp		= 25,		/* horizontal front porch */
			.hbp		= 30,		/* horizontal back porch */
			.vsw		= 4,		/* vertical sync pulse width */
			.vfp		= 12,		/* vertical front porch */
			.vbp		= 8,		/* vertical back porch */		
		},
	},
	.platform_enable = board_panel_enable_lcd,
	.platform_disable = board_panel_disable_lcd,
	.dev = {
		.platform_data = &board_dss_lcd_data,
	},
};

static struct omap_dss_device *board_dss_devices[] = {
	&board_lcd_device,
};

static struct omap_dss_board_info board_dss_data = {
	.num_devices	=	ARRAY_SIZE(board_dss_devices),
	.devices	=	board_dss_devices,
	.default_device	=	&board_lcd_device,
};

static struct platform_device board_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev            = {
		.platform_data = &board_dss_data,
	},
};

#define GPIO_POWER_LED			57
#define GPIO_LCD_BACKLIGHT_LED_TIMER	8
#define GPIO_LCD_BACKLIGHT_MAX		200
	
static struct gpio_led gpio_leds[] = {
	{
		.name			= "power",
		//.default_trigger	= "default-on",
		.gpio			= GPIO_POWER_LED,
		.active_low		= 0,
	},
};

static void bkl_set_power(struct omap_pwm_led_platform_data *self, int on_off)
{
	//gpio_set_value(GPIO_LCD_BACKLIGHT_POWER, on_off);
}

static struct omap_pwm_led_platform_data board_backlight_data = {
	.name			= "lcd-backlight",
	.default_trigger	= "backlight",
	.intensity_timer 	= GPIO_LCD_BACKLIGHT_LED_TIMER,
	.bkl_max 		= GPIO_LCD_BACKLIGHT_MAX,
	.bkl_freq 		= 0,
	.invert 		= 0,
	.set_power 		= &bkl_set_power,
};

static struct gpio_led_platform_data board_led_data = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device board_led_device = {
	.name		= "leds-gpio",
	.id		= 0,
	.dev            = {
		.platform_data = &board_led_data,
	},
};

static struct platform_device board_backlight_device = {
	.name		= "omap_pwm_led",
	.id		= 0,
	.dev.platform_data = &board_backlight_data,
};


static int __init omap3_leds_init(void)
{
	omap_mux_init_signal("gpt8_pwm_evt", OMAP_PIN_OUTPUT);

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

#define GPIO_MMC1_POWER		158
#define GPIO_MMC1_DETECT	65
#define GPIO_MMC1_PREWARN	58
static int archos_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
 	struct omap2_hsmmc_info *c;
	int ret;
	
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

	ret = gpio_request(GPIO_MMC1_PREWARN, "mmc1_prewarn");
	if (ret) {
		pr_err("Cannot request GPIO %d\n", GPIO_MMC1_PREWARN);
		goto error1;
	}
	
	gpio_export(GPIO_MMC1_PREWARN, 0);
	gpio_direction_input(GPIO_MMC1_PREWARN);
	
	omap2_hsmmc_init(controllers);

	for (c = controllers; c->mmc; c++)
		archos_hsmmc_set_late_init(c->dev);

	return 0;
error1:
	return -1;
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

extern struct platform_device a43_camera_device;

static struct regulator_consumer_supply board_2v8d_cam_supply = {
	.supply 	= "2v8d_cam",
	.dev		= &a43_camera_device.dev,
};

#endif /* CONFIG_OMAP2_DSS */


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
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &board_2v8d_cam_supply,
};

static struct regulator_consumer_supply board_vdds_dsi_supply = {
	.supply	= "vdds_dsi",
};

struct regulator_init_data vdds_dsi_initdata = {
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
#if 0
static struct twl4030_usb_data board_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
	.enable_charge_detect = 1, 
};
#endif
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
	//.usb		= &board_usb_data,
	//.keypad		= &board_kp_twl4030_data,
	.power		= &board_t2scripts_data,
	.vmmc1          = &board_vmmc1, /* used for VACC :-( */
	.vdac		= &board_vdac,
	.vaux2          = &board_vaux2,
};

static struct i2c_board_info __initdata board_i2c_bus1_info[] = {
	{
		I2C_BOARD_INFO("tps65921", 0x48),	// id to fix
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &board_tps65921_pdata,
	},
};

static void archos_hdmi_power(struct i2c_client *cl, int on_off)
{
	struct device *dev = &cl->dev;
	struct archos_hdmi_platform_data *pdata = dev->platform_data;
	
#if 0
	if (on_off)
		gpio_direction_output(pdata->pwr_gpio, 1);
	else
		gpio_direction_input(pdata->pwr_gpio);
#endif
}

#if 0
static struct archos_hdmi_platform_data board_hdmi_pdata = {
	.hdmi_pwr = archos_hdmi_power,
};
#endif
static struct omap_i2c_bus_board_data __initdata archos_i2c_bus_pdata;

static struct i2c_board_info __initdata board_i2c_bus2_info[] = {
	[0] = {
		I2C_BOARD_INFO("wm8988", 0x1a),
		.flags = I2C_CLIENT_WAKE,
	},
#if 0
	[1] = {
		I2C_BOARD_INFO("tda998X", 0x70),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_hdmi_pdata,
	},
	[2] = {
		I2C_BOARD_INFO("tda99Xcec", 0x34),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_hdmi_pdata,
	}
#endif
};

static struct i2c_board_info __initdata board_i2c_bus3_info[] = {
	{
		I2C_BOARD_INFO("mma7660fc", 0x4c),	// to fix
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_mma7660fc_pdata,
	},
	{
		I2C_BOARD_INFO("akm8973", 0x1c),
		.flags = I2C_CLIENT_WAKE,
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
	omap_register_i2c_bus(3, 100, &archos_i2c_bus_pdata, board_i2c_bus3_info,
			ARRAY_SIZE(board_i2c_bus3_info));

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

#ifdef CONFIG_PM
#ifdef CONFIG_TWL4030_CORE
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
#endif

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
#endif

static void __init board_init(void)
{
	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);

	archos_accel_mma7660fc_init(&board_mma7660fc_pdata);
	omap3_i2c_init();

	platform_add_devices(archos_devices, ARRAY_SIZE(archos_devices));

	omap_serial_init(omap_serial_platform_data);

	omap3_leds_init();
	
#if defined CONFIG_OMAP2_DSS
	archos_init_dss();
#endif	
	archos_hsmmc_init(mmc);

	enable_board_wakeup_source();

	omap_display_init(&board_dss_data);

#ifdef CONFIG_PM
#ifdef CONFIG_TWL4030_CORE
	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
#endif
	omap_voltage_init_vc(&vc_config);
#endif
	
	archos_camera_mt9d113_init();
	archos_audio_wm8988_init();
}

static void __init board_map_io(void)
{
	omap2_set_globals_343x();
	omap34xx_map_common_io();
}

MACHINE_START(ARCHOS_A43, "Archos A43 board")
	.phys_io	= 0x48000000,
	.io_pg_offst    = ((0xfa000000) >> 18) & 0xfffc, 
	.boot_params	= 0x80000100,
	.map_io		= board_map_io,
	.init_irq	= board_init_irq,
	.init_machine	= board_init,
	.timer		= &omap_timer,
MACHINE_END
