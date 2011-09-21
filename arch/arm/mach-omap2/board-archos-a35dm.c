/*
 * board-archos-a35dm.c
 *
 *  Created on: Oct 26, 2010
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
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
#include <plat/board-archos.h>
#include <asm/feature_list.h>

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

static struct archos_display_config display_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	25,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.cvbs_on =	UNUSED_GPIO,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[1] = {
		.lcd_pwon = 	157,
		.lcd_rst = 	25,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_pwr = 	UNUSED_GPIO,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.cvbs_on =	UNUSED_GPIO,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
};

static int __init archos_lcd_panel_init(struct omap_dss_device *disp_data)
{
	switch (hardware_rev) {
	default:
		return panel_a35_init(disp_data);
	}
}

static struct omap_uart_config uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | /*(1 << 1) |*/ (1 << 2)),
};

static struct archos_tsp_config tsp_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.irq_gpio = 24,
		.pwr_gpio = 175,
		.x_plate_ohms = 250,
		.pressure_max = 1500,
		.inversion_flags = XY_SWAP | X_INV,
	},
	.rev[1] = {
		.irq_gpio = 24,
		.pwr_gpio = 175,
		.x_plate_ohms = 250,
		.pressure_max = 1500,
		.inversion_flags = XY_SWAP | X_INV,
	},
};

static struct archos_audio_config audio_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 142,
		.headphone_plugged = 109,
//		.clk_mux = AE22_3430_CLKOUT2,
		.vamp_vbat = 143,
		.vamp_dc = 29,
	},
	.rev[1] = {
		.spdif = UNUSED_GPIO,
		.hp_on = 142,
		.headphone_plugged = 109,
//		.clk_mux = AE22_3430_CLKOUT2,
		.vamp_vbat = 143,
		.vamp_dc = 29,
	},
};

static struct archos_charge_config charge_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.charge_enable 	= 27,
		.charge_high	= 28,
		.charge_low 	= 22,
	},
	.rev[1] = {
		.charge_enable 	= 27,
		.charge_high	= 28,
		.charge_low 	= 22,
	},
};

static struct archos_wifi_bt_config wifi_bt_dev_conf __initdata = {
	.nrev = 2,
	.rev[0] = {
		.wifi_power 	= 111,
		.wifi_irq 	= 114,
		.bt_power 	= 162,
		.wifi_pa_type	= PA_TYPE_RF3482,
	},
	.rev[1] = {
		.wifi_power 	= 111,
		.wifi_irq 	= 114,
		.bt_power 	= 162,
		.wifi_pa_type	= PA_TYPE_TQM67002A,
	},
};

static struct archos_accel_config accel_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[1] = {
		.accel_int1 = 115,
		.accel_int2 = UNUSED_GPIO,
	},
};

static struct archos_camera_config camera_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.pwr_down = 23,
		.reset = 26, 
	},
	.rev[1] = {
		.pwr_down = 23,
		.reset = 26, 
	},
};

static struct archos_leds_config leds_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.power_led = 57,
//		.backlight_led = { .timer = 8, .mux_cfg = V3_3430_GPT08 },
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
	.rev[1] = {
		.power_led = 57,
//		.backlight_led = { .timer = 8, .mux_cfg = V3_3430_GPT08 },
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
	},
};

static struct archos_sd_config sd_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.sd_power = 158,
		.sd_detect = 65,
		.sd_prewarn = UNUSED_GPIO,
	},
	.rev[1] = {
		.sd_power = 158,
		.sd_detect = 65,
		.sd_prewarn = UNUSED_GPIO,
	},
};

static struct archos_keys_config keys_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.power = 64,
	},
	.rev[1] = {
		.power = 64,
	},
};

static struct omap_board_config_kernel board_config[] __initdata = {
//	{ OMAP_TAG_UART,	&uart_config },
	{ ARCHOS_TAG_DISPLAY,	&display_config },
//	{ ARCHOS_TAG_TSP,	&tsp_config },
//	{ ARCHOS_TAG_CHARGE,	&charge_config},
//	{ ARCHOS_TAG_AUDIO,     &audio_config},
//	{ ARCHOS_TAG_WIFI_BT,	&wifi_bt_dev_conf},
//	{ ARCHOS_TAG_ACCEL,	&accel_config},
//	{ ARCHOS_TAG_CAMERA,	&camera_config},
//	{ ARCHOS_TAG_LEDS,	&leds_config},
//	{ ARCHOS_TAG_KEYS,	&keys_config},
//	{ ARCHOS_TAG_SD,    	&sd_config},
};

#if 0
#ifdef CONFIG_WL127X_RFKILL
static struct wl127x_rfkill_platform_data wl127x_plat_data = {
	.bt_nshutdown_gpio = 162, 	/* Bluetooth Enable GPIO */
	.fm_enable_gpio = 0,		/* FM Enable GPIO */
};

static struct platform_device board_wl127x_device = {
	.name           = "wl127x-rfkill",
	.id             = -1,
	.dev.platform_data = &wl127x_plat_data,
};
#endif
#endif

static struct omap_dss_device board_lcd_device;

static struct omap_dss_device *board_dss_devices[1];

static struct omap_dss_board_info board_dss_data = {
	.num_devices = 1,
	.devices = &board_dss_devices,
	.default_device = NULL,
};

static struct platform_device board_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev            = {
		.platform_data = &board_dss_data,
	},
};

#if 0
static struct resource board_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};

#ifdef CONFIG_PM
struct vout_platform_data a35dm_vout_data = {
//	.set_min_bus_tput = omap_pm_set_min_bus_tput,
	.set_max_mpu_wakeup_lat =  omap_pm_set_max_mpu_wakeup_lat,
	.set_vdd1_opp = omap_pm_set_min_mpu_freq,
	.set_cpu_freq = omap_pm_cpu_set_freq,
};
#endif

static struct platform_device board_vout_device = {
	.name		= "omap_vout",
	.num_resources	= ARRAY_SIZE(board_vout_resource),
	.resource 	= &board_vout_resource[0],
	.id		= -1,
#ifdef CONFIG_PM
	.dev		= {
		.platform_data = &a35dm_vout_data,
	}
#else
	.dev		= {
		.platform_data = NULL,
	}
#endif
};
#endif

static struct regulator_consumer_supply board_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &board_dss_device.dev,
};

static struct regulator_consumer_supply board_avcc_supply = {
	.supply		= "avcc",
};

//extern struct platform_device camera_device;

static struct regulator_consumer_supply board_vdd_cam_supply = {
	.supply 	= "vdd_cam",
//	.dev		= &camera_device.dev,
};

/* consumer for vdds_dsi which is permanently enabled */
static struct regulator_consumer_supply board_vdds_dsi_supply = {
	.supply		= "vdds_dsi",
	.dev		= &board_dss_device.dev,
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
	.consumer_supplies	= &board_vdd_cam_supply,
};

static struct twl4030_usb_data board_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
//	.enable_charge_detect = 1, 
};

/* TODO: might not be needed
static struct twl4030_gpio_platform_data board_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= zoom2_twl_gpio_setup,
};
*/
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

static struct twl4030_resconfig twl4030_rconfig[] = {
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3, .type = -1,
		.type2 = -1 },
/* FIXME: adding VDD1/2 to DEV_GRP_P1 causes device to hang after power-off.
 * 	commenting this fixes it but what is the impact on power consumption 
 * 	in OFF mode?
 * 
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
	.power		= &board_t2scripts_data,
	.vmmc1          = &board_vmmc1,
	.vdac		= &board_vdac,
	.vaux2          = &board_vaux2,
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

static struct platform_device board_vdds_dsi_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev.platform_data = &board_vdds_dsi,
};

static struct regulator_consumer_supply board_vmmc_ext_supply = {
	.supply	= "vmmc",
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
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &board_vmmc_ext_supply,
};

static struct platform_device board_vmmc_ext_device = {
	.name		= "reg-fsw-voltage",
	.id		= 0,
	.dev.platform_data = &board_vmmc_ext,
};

static struct regulator_consumer_supply board_vmmc2_supply = {
	.supply	= "vmmc",
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
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &board_vmmc2_supply,
};

static struct platform_device board_vmmc2_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev.platform_data = &board_vmmc2,
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
	{
		I2C_BOARD_INFO("wm8988", 0x1a),
		.flags = I2C_CLIENT_WAKE,
	}
};

static struct i2c_board_info __initdata board_i2c_bus3_info[] = {
	{
		I2C_BOARD_INFO("mma7660fc", 0x4c),	// to fix
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
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
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
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
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
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
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
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
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
	int num_displays = 0;
	
	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);
	omap3_mux_init(board_mux, OMAP_PACKAGE_CYN);


	if (panel_a35_init(&board_lcd_device) == 0)
		board_dss_data.default_device = &board_lcd_device;

	archos_accel_mma7660fc_init(&board_mma7660fc_pdata);
	omap3_i2c_init();

	platform_add_devices(archos_devices, ARRAY_SIZE(archos_devices));

	omap_serial_init(omap_serial_platform_data);
//	ads7846_dev_init();

//	archos_leds_init();

	archos_init_dss();

//	archos_hsmmc_init(mmc);

	enable_board_wakeup_source();

	omap_display_init(&board_dss_data);

	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
	omap_voltage_register_pmic(&omap_pmic_iva, "iva");
	omap_voltage_init_vc(&vc_config);

	archos_audio_wm8988_init();
}

static void __init board_map_io(void)
{
	omap2_set_globals_343x();
	omap34xx_map_common_io();
}

MACHINE_START(ARCHOS_A35DM, "Archos A35DM board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= board_map_io,
	.init_irq	= board_init_irq,
	.init_machine	= board_init,
	.timer		= &omap_timer,
MACHINE_END
