/*
 * Board support file for OMAP4430 SDP.
 *
 * Copyright (C) 2011 Texas Instruments
 *

 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/bq2415x.h>
#include <linux/i2c/bma180.h>
#include <linux/i2c/mpu3050.h>
#include <linux/i2c/tsl2771.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/tps6130x.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/twl6040-vib.h>
#include <linux/wl12xx.h>
#include <linux/cdc_tcxo.h>
#include <linux/mfd/twl6040-codec.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/dmm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/timer-gp.h>
#include <plat/display.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif
#include <plat/opp_twl_tps.h>
#include <plat/mmc.h>
#include <plat/temperature_sensor.h>
#include <plat/hwspinlock.h>
#include <plat/nokia-dsi-panel.h>
#include <plat/toshiba-dsi-panel.h>
#include <linux/wakelock.h>
#include <linux/qtouch_obp_ts.h>
#include <linux/gpio_keys.h>
#include "mux.h"
#include "hsmmc.h"
#include "smartreflex-class3.h"
#include "board-4430sdp-wifi.h"
#include "omap_tps6236x.h"

#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <mach/omap4-common.h>

#define ETH_KS8851_IRQ			34
#define ETH_KS8851_POWER_ON		48
#define ETH_KS8851_QUART		138

#define OMAP4_TOUCH_IRQ_1		35
#define OMAP4_BMA180ACCEL_GPIO		178
#define OMAP4_MPU3050GYRO_GPIO		2
#define OMAP4_TSL2771_INT_GPIO		184
#define OMAP4_TSL2771_PWR_GPIO		188
#define OMAP4SDP_MDM_PWR_EN_GPIO	157

#define LED_PWM2ON		0x03
#define LED_PWM2OFF		0x04
#define LED_TOGGLE3		0x92

#define GPIO_WIFI_PMENA		54
#define GPIO_WIFI_IRQ		53

#define TWL6030_RTC_GPIO 6
#define BLUETOOTH_UART UART2
#define CONSOLE_UART UART3

static struct wake_lock uart_lock;
static struct platform_device sdp4430_hdmi_audio_device = {
	.name		= "hdmi-dai",
	.id		= -1,
};


/* GPIO_KEY for Tablet */
static struct gpio_keys_button tablet_gpio_keys_buttons[] = {
	[0] = {
		.code			= KEY_VOLUMEUP,
		.gpio			= 43,
		.desc			= "SW1",
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 5,
	},
	[1] = {
		.code			= KEY_HOME,
		.gpio			= 46,
		.desc			= "SW2",
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 5,
	},
	[2] = {
		.code			= KEY_VOLUMEDOWN,
		.gpio			= 47,
		.desc			= "SW3",
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 5,
		},
	};

static struct gpio_keys_platform_data tablet_gpio_keys = {
	.buttons		= tablet_gpio_keys_buttons,
	.nbuttons		= ARRAY_SIZE(tablet_gpio_keys_buttons),
	.rep			= 0,
};

static struct platform_device tablet_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &tablet_gpio_keys,
	},
};

static struct spi_board_info sdp4430_spi_board_info[] __initdata = {
	{
		.modalias               = "ks8851",
		.bus_num                = 1,
		.chip_select            = 0,
		.max_speed_hz           = 24000000,
		.irq                    = ETH_KS8851_IRQ,
	},
};

static int omap_ethernet_init(void)
{
	int status;

	/* Request of GPIO lines */

	status = gpio_request(ETH_KS8851_POWER_ON, "eth_power");
	if (status) {
		pr_err("Cannot request GPIO %d\n", ETH_KS8851_POWER_ON);
		return status;
	}

	status = gpio_request(ETH_KS8851_QUART, "quart");
	if (status) {
		pr_err("Cannot request GPIO %d\n", ETH_KS8851_QUART);
		goto error1;
	}

	status = gpio_request(ETH_KS8851_IRQ, "eth_irq");
	if (status) {
		pr_err("Cannot request GPIO %d\n", ETH_KS8851_IRQ);
		goto error2;
	}

	/* Configuration of requested GPIO lines */

	status = gpio_direction_output(ETH_KS8851_POWER_ON, 1);
	if (status) {
		pr_err("Cannot set output GPIO %d\n", ETH_KS8851_IRQ);
		goto error3;
	}

	status = gpio_direction_output(ETH_KS8851_QUART, 1);
	if (status) {
		pr_err("Cannot set output GPIO %d\n", ETH_KS8851_QUART);
		goto error3;
	}

	status = gpio_direction_input(ETH_KS8851_IRQ);
	if (status) {
		pr_err("Cannot set input GPIO %d\n", ETH_KS8851_IRQ);
		goto error3;
	}

	return 0;

error3:
	gpio_free(ETH_KS8851_IRQ);
error2:
	gpio_free(ETH_KS8851_QUART);
error1:
	gpio_free(ETH_KS8851_POWER_ON);
	return status;
}

static struct gpio_led sdp4430_gpio_leds[] = {
	{
		.name	= "tablet:led_gp_4_c",
		.gpio	= 50,
	},
	{
		.name	= "blue",
		.default_trigger = "timer",
		.gpio	= 169,
	},
	{
		.name	= "red",
		.default_trigger = "timer",
		.gpio	= 170,
	},
	{
		.name	= "green",
		.default_trigger = "timer",
		.gpio	= 139,
	},
};

static struct gpio_led_platform_data sdp4430_led_data = {
	.leds	= sdp4430_gpio_leds,
	.num_leds	= ARRAY_SIZE(sdp4430_gpio_leds),
};

static struct platform_device sdp4430_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &sdp4430_led_data,
	},
};

static void __init sdp4430_init_display_led(void)
{
	twl_i2c_write_u8(TWL_MODULE_PWM, 0xFF, LED_PWM2ON);
	twl_i2c_write_u8(TWL_MODULE_PWM, 0x7F, LED_PWM2OFF);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TWL6030_TOGGLE3);
}

/*
* Recalculation of brightness in new range using formula:
* MIN_BRIGHT, MAX_BRIGHT - brightness range (changes form 0 to 127).
*/

#define MIN_BRIGHT	112
#define MAX_BRIGHT	0

static void sdp4430_set_primary_brightness(u8 brightness)
{
	if (brightness > 1) {
		s32 new_brightness = ((MAX_BRIGHT -
					     MIN_BRIGHT) << 8) * brightness;
		new_brightness += ((MIN_BRIGHT) << 16);
		new_brightness >>= 16;

		brightness = (u8)new_brightness;

		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TWL6030_TOGGLE3);
		twl_i2c_write_u8(TWL_MODULE_PWM, brightness, LED_PWM2ON);
	} else {
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x08, TWL6030_TOGGLE3);
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x38, TWL6030_TOGGLE3);
	}
}

static struct omap4430_sdp_disp_led_platform_data sdp4430_disp_led_data = {
	.flags = LEDS_CTRL_AS_ONE_DISPLAY,
	.display_led_init = sdp4430_init_display_led,
	.primary_display_set = sdp4430_set_primary_brightness,
};

static struct platform_device sdp4430_disp_led = {
	.name	=	"display_led",
	.id	=	-1,
	.dev	= {
		.platform_data = &sdp4430_disp_led_data,
	},
};

static struct bma180accel_platform_data bma180accel_platform_data = {
	.ctrl_reg0	= 0x11,
	.g_range	= BMA_GRANGE_2G,
	.bandwidth	= BMA_BW_10HZ,
	.mode		= BMA_MODE_LOW_NOISE,
	.bit_mode	= BMA_BITMODE_14BITS,
	.smp_skip	= 1,
	.def_poll_rate	= 200,
	.fuzz_x		= 25,
	.fuzz_y		= 25,
	.fuzz_z		= 25,
};

/* TSL2771 ALS/Prox Begin */

static void omap_tsl2771_power(int state)
{
	gpio_set_value(OMAP4_TSL2771_PWR_GPIO, state);
}

static void blaze_tablet_tsl2771_init(void)
{
	/* TO DO: Not sure what the use case of the proximity is on a tablet
	 * but the interrupt may need to be wakeable if and only if proximity
	 * is enabled but for now leave it alone */
	gpio_request(OMAP4_TSL2771_PWR_GPIO, "tsl2771_power");
	gpio_direction_output(OMAP4_TSL2771_PWR_GPIO, 0);

	gpio_request(OMAP4_TSL2771_INT_GPIO, "tsl2771_interrupt");
	gpio_direction_input(OMAP4_TSL2771_INT_GPIO);
}

/* TO DO: Need to create a interrupt threshold table here */

struct tsl2771_platform_data tsl2771_data = {
	.irq_flags	= (IRQF_TRIGGER_LOW | IRQF_ONESHOT),
	.flags		= (TSL2771_USE_ALS | TSL2771_USE_PROX),
	.def_enable			= 0x0,
	.als_adc_time 			= 0xdb,
	.prox_adc_time			= 0xff,
	.wait_time			= 0x00,
	.als_low_thresh_low_byte	= 0x0,
	.als_low_thresh_high_byte	= 0x0,
	.als_high_thresh_low_byte	= 0x0,
	.als_high_thresh_high_byte	= 0x0,
	.prox_low_thresh_low_byte	= 0x0,
	.prox_low_thresh_high_byte	= 0x0,
	.prox_high_thresh_low_byte	= 0x0,
	.prox_high_thresh_high_byte	= 0x0,
	.interrupt_persistence		= 0xf6,
	.config				= 0x00,
	.prox_pulse_count		= 0x03,
	.gain_control			= 0xE0,
	.glass_attn			= 0x01,
	.device_factor			= 0x34,
	.tsl2771_pwr_control		= omap_tsl2771_power,
};
/* TSL2771 ALS/Prox End */

/* MPU3050 Gyro Begin */

static void omap_mpu3050_init(void)
{
	if (gpio_request(OMAP4_MPU3050GYRO_GPIO, "mpu3050") < 0) {
		pr_err("%s: MPU3050 GPIO request failed\n", __func__);
		return;
	}
	gpio_direction_input(OMAP4_MPU3050GYRO_GPIO);
}

static struct mpu3050gyro_platform_data mpu3050_platform_data = {
	.irq_flags = (IRQF_TRIGGER_HIGH | IRQF_ONESHOT),
	.slave_i2c_addr = 0x40,
	.sample_rate_div = 0x00,
	.dlpf_fs_sync = 0x11,
	.interrupt_cfg = (MPU3050_INT_CFG_OPEN | MPU3050_INT_CFG_LATCH_INT_EN |
		MPU3050_INT_CFG_MPU_RDY_EN | MPU3050_INT_CFG_RAW_RDY_EN),
};

/* MPU3050 Gyro End */
/* Atmel MXT224 TouchScreen Begin */
static struct qtm_touch_keyarray_cfg blaze_tablet_key_array_data[] = {
	{
		.ctrl = 0,
		.x_origin = 0,
		.y_origin = 0,
		.x_size = 0,
		.y_size = 0,
		.aks_cfg = 0,
		.burst_len = 0,
		.tch_det_thr = 0,
		.tch_det_int = 0,
		.rsvd1 = 0,
	},
};

static void blaze_tablet_touch_init(void)
{
	gpio_request(OMAP4_TOUCH_IRQ_1, "atmel touch irq");
	gpio_direction_input(OMAP4_TOUCH_IRQ_1);
	omap_mux_init_signal("gpmc_ad11.gpio_35",
			OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE3 );
}

static struct qtouch_ts_platform_data atmel_mxt224_ts_platform_data = {
	.irqflags	= (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW),
	.flags		= (QTOUCH_USE_MULTITOUCH | QTOUCH_FLIP_Y |
			   QTOUCH_CFG_BACKUPNV),
	.abs_min_x	= 0,
	.abs_max_x	= 768,
	.abs_min_y	= 0,
	.abs_max_y	= 1024,
	.abs_min_p	= 0,
	.abs_max_p	= 255,
	.abs_min_w	= 0,
	.abs_max_w	= 15,
	.x_delta	= 1024,
	.y_delta	= 768,
	.nv_checksum	= 0x187a,
	.fuzz_x		= 0,
	.fuzz_y		= 0,
	.fuzz_p		= 2,
	.fuzz_w		= 2,
	.hw_reset	= NULL,
	.power_cfg	= {
		.idle_acq_int	= 0x58,
		.active_acq_int	= 0xff,
		.active_idle_to	= 0x0a,
	},
	.acquire_cfg	= {
		.charge_time	= 0x20,
		.atouch_drift	= 0x05,
		.touch_drift	= 0x14,
		.drift_susp	= 0x14,
		.touch_autocal	= 0x4b,
		.sync		= 0,
		.cal_suspend_time = 0x09,
		.cal_suspend_thresh = 0x23,
	},
	.multi_touch_cfg	= {
		.ctrl		= 0x83,
		.x_origin	= 0,
		.y_origin	= 0,
		.x_size		= 0x11,
		.y_size		= 0x0d,
		.aks_cfg	= 0x0,
		.burst_len	= 0x01,
		.tch_det_thr	= 0x30,
		.tch_det_int	= 0x2,
		.mov_hyst_init	= 0x0,
		.mov_hyst_next	= 0x0,
		.mov_filter	= 0x30,
		.num_touch	= 1,
		.orient		= 0x00,
		.mrg_timeout	= 0x01,
		.merge_hyst	= 0x0a,
		.merge_thresh	= 0x0a,
		.amp_hyst 	= 0x0a,
		.x_res 		= 0x02ff,
		.y_res 		= 0x03ff,
		.x_low_clip	= 0x00,
		.x_high_clip 	= 0x00,
		.y_low_clip 	= 0x00,
		.y_high_clip 	= 0x00,
	},
	.key_array      = {
		.cfg		= blaze_tablet_key_array_data,
		.num_keys   = ARRAY_SIZE(blaze_tablet_key_array_data),
	},
	.grip_suppression_cfg = {
		.ctrl		= 0x01,
		.xlogrip	= 0x00,
		.xhigrip	= 0x00,
		.ylogrip	= 0x00,
		.yhigrip	= 0x00,
		.maxtchs	= 0x00,
		.reserve0	= 0x00,
		.szthr1		= 0x50,
		.szthr2		= 0x28,
		.shpthr1	= 0x04,
		.shpthr2	= 0x0f,
		.supextto	= 0x0a,
	},
	.noise0_suppression_cfg = {
		.ctrl		= 0x05,
		.reserved	= 0x0000,
		.gcaf_upper_limit = 0x000a,
		.gcaf_lower_limit = 0xfff6,
		.gcaf_valid	= 0x04,
		.noise_thresh	= 0x20,
		.reserved1	= 0x00,
		.freq_hop_scale = 0x01,
		.burst_freq_0	= 0x0a,
		.burst_freq_1 = 0x0f,
		.burst_freq_2 = 0x14,
		.burst_freq_3 = 0x19,
		.burst_freq_4 = 0x1e,
		.num_of_gcaf_samples = 0x04,
	},
	.spt_cte_cfg = {
		.ctrl = 0x00,
		.command = 0x00,
		.mode = 0x01,
		.gcaf_idle_mode = 0x04,
		.gcaf_actv_mode = 0x08,
	},
};
/* End Atmel Touch screen */

static struct toshiba_dsi_panel_data blazetablet_dsi_panel = {
	.name	= "d2l",
	.reset_gpio	= 102,
	.use_ext_te	= false,
	.ext_te_gpio	= 101,
	.use_esd_check	= false,
	.set_backlight	= NULL,
};

static struct omap_dss_device blazetablet_lcd_device = {
	.name			= "lcd",
	.driver_name		= "d2l",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.data			= &blazetablet_dsi_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,
		.data3_lane	= 4,
		.data3_pol	= 0,
		.data4_lane	= 5,
		.data4_pol	= 0,
		.div		= {
			.lck_div	= 1,	/* LCD */
			.pck_div	= 2,	/* PCD */
			.regm		= 394,	/* DSI_PLL_REGM */
			.regn		= 38,	/* DSI_PLL_REGN */
			.regm_dispc	= 6,	/* PLL_CLK1 (M4) */
			.regm_dsi	= 9,	/* PLL_CLK2 (M5) */
			.lp_clk_div	= 5,	/* LPDIV */
		},
		.xfer_mode = OMAP_DSI_XFER_VIDEO_MODE,
	},
	.panel			= {
		.width_in_mm = 210,
		.height_in_mm = 158,
	},
	.channel		= OMAP_DSS_CHANNEL_LCD,
};

#ifdef CONFIG_OMAP2_DSS_HDMI
static int sdp4430_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	gpio_request(HDMI_GPIO_60 , "hdmi_gpio_60");
	gpio_request(HDMI_GPIO_41 , "hdmi_gpio_41");
	gpio_direction_output(HDMI_GPIO_60, 1);
	gpio_direction_output(HDMI_GPIO_41, 1);
	gpio_set_value(HDMI_GPIO_60, 0);
	gpio_set_value(HDMI_GPIO_41, 0);
	mdelay(5);
	gpio_set_value(HDMI_GPIO_60, 1);
	gpio_set_value(HDMI_GPIO_41, 1);
	return 0;
}

static void sdp4430_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
}

static __attribute__ ((unused)) void __init sdp4430_hdmi_init(void)
{
}

static struct omap_dss_device sdp4430_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.phy.dpi.data_lines = 24,
	.platform_enable = sdp4430_panel_enable_hdmi,
	.platform_disable = sdp4430_panel_disable_hdmi,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};
#endif /* CONFIG_OMAP2_DSS_HDMI */

#ifdef CONFIG_PANEL_PICO_DLP
static int sdp4430_panel_enable_pico_DLP(struct omap_dss_device *dssdev)
{
	/* int i = 0; */

	gpio_request(DLP_4430_GPIO_45, "DLP PARK");
	gpio_direction_output(DLP_4430_GPIO_45, 0);
	gpio_request(DLP_4430_GPIO_40, "DLP PHY RESET");
	gpio_direction_output(DLP_4430_GPIO_40, 0);
	/* gpio_request(DLP_4430_GPIO_44, "DLP READY RESET");
	gpio_direction_input(DLP_4430_GPIO_44); */
	mdelay(500);

	gpio_set_value(DLP_4430_GPIO_45, 1);
	mdelay(1000);

	gpio_set_value(DLP_4430_GPIO_40, 1);
	mdelay(1000);

	/* FIXME with the MLO gpio changes,
		gpio read is not retuning correct value even though
		it is  set in hardware so the check is comment
		till the problem is fixed */
	/* while (i == 0)
		i = gpio_get_value(DLP_4430_GPIO_44); */

	mdelay(2000);
	return 0;
}

static void sdp4430_panel_disable_pico_DLP(struct omap_dss_device *dssdev)
{
	gpio_set_value(DLP_4430_GPIO_40, 0);
	gpio_set_value(DLP_4430_GPIO_45, 0);
}

static struct omap_dss_device sdp4430_picoDLP_device = {
	.name			= "pico_DLP",
	.driver_name		= "picoDLP_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= sdp4430_panel_enable_pico_DLP,
	.platform_disable	= sdp4430_panel_disable_pico_DLP,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};
#endif /* CONFIG_PANEL_PICO_DLP */

/* PARADE DP501, DisplayPort chip */
static int sdp4430_panel_enable_displayport(struct omap_dss_device *dssdev)
{
	printk(KERN_DEBUG "sdp4430_panel_enable_displayport is called\n");
	gpio_request(DP_4430_GPIO_59, "DISPLAYPORT POWER DOWN");
	gpio_direction_output(DP_4430_GPIO_59, 0);
	mdelay(100);
	gpio_set_value(DP_4430_GPIO_59, 1);
	mdelay(100);
	return 0;
}

static void sdp4430_panel_disable_displayport(struct omap_dss_device *dssdev)
{
	printk(KERN_DEBUG "sdp4430_panel_disable_displayport is called\n");
	gpio_set_value(DP_4430_GPIO_59, 0);
}

static struct omap_dss_device sdp4430_displayport_device = {
	.name				= "DP501",
	.driver_name			= "displayport_panel",
	.type				= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines		= 24,
	.platform_enable		= sdp4430_panel_enable_displayport,
	.platform_disable		= sdp4430_panel_disable_displayport,
	.channel			= OMAP_DSS_CHANNEL_LCD2,
};

static struct omap_dss_device *blazetablet_dss_devices[] = {
	&blazetablet_lcd_device,
#ifdef CONFIG_PANEL_DP501
	&sdp4430_displayport_device,
#endif
#ifdef CONFIG_OMAP2_DSS_HDMI
	&sdp4430_hdmi_device,
#endif
#ifdef CONFIG_PANEL_PICO_DLP
	&sdp4430_picoDLP_device,
#endif
};

static struct omap_dss_board_info blazetablet_dss_data = {
	.num_devices	=	ARRAY_SIZE(blazetablet_dss_devices),
	.devices	=	blazetablet_dss_devices,
	.default_device =	&blazetablet_lcd_device,
};

static unsigned long retry_suspend;
static int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct kim_data_s *kim_gdata;
	struct st_data_s *core_data;
	kim_gdata = dev_get_drvdata(&pdev->dev);
	core_data = kim_gdata->core_data;
	if (st_ll_getstate(core_data) != ST_LL_INVALID) {
		/*Prevent suspend until sleep indication from chip*/
		while (st_ll_getstate(core_data) != ST_LL_ASLEEP &&
			(retry_suspend++ < 5)) {
			return -1;
		}
	}
	return 0;
}
static int plat_kim_resume(struct platform_device *pdev)
{
	retry_suspend = 0;
	return 0;
}

static int plat_kim_chip_enable(struct kim_data_s *kim_data)
{
	printk(KERN_INFO"%s\n", __func__);
	/* Configure BT nShutdown to HIGH state */
	gpio_set_value(kim_data->nshutdown, GPIO_LOW);
	mdelay(5);      /* FIXME: a proper toggle */
	gpio_set_value(kim_data->nshutdown, GPIO_HIGH);
	mdelay(100);
	/* Call to black DPLL when BT/FM is in use */
	dpll_cascading_blocker_hold(&kim_data->kim_pdev->dev);
	return 0;
}

static int plat_kim_chip_disable(struct kim_data_s *kim_data)
{
	printk(KERN_INFO"%s\n", __func__);
	/* By default configure BT nShutdown to LOW state */
	gpio_set_value(kim_data->nshutdown, GPIO_LOW);
	mdelay(1);
	gpio_set_value(kim_data->nshutdown, GPIO_HIGH);
	mdelay(1);
	gpio_set_value(kim_data->nshutdown, GPIO_LOW);
	/* Release DPLL cascading blockers when we are done with BT/FM */
	dpll_cascading_blocker_release(&kim_data->kim_pdev->dev);
	return 0;
}

/* wl128x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 55,
	.dev_name = "/dev/ttyO1",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
	.chip_enable = plat_kim_chip_enable,
	.chip_disable = plat_kim_chip_disable,
};
static struct platform_device wl128x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct platform_device *blazetablet_devices[] __initdata = {
	&sdp4430_disp_led,
	/* TODO. Review button LEDs functionality
	&sdp4430_leds_pwm, */
	&sdp4430_leds_gpio,
	&wl128x_device,
	&btwilink_device,
	&sdp4430_hdmi_audio_device,
	&tablet_gpio_keys_device,
};

static void __init omap_4430sdp_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
	gic_init_irq();
	sr_class3_init();
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode			= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 200,
};

static int wifi_set_power(struct device *dev, int slot, int power_on, int vdd)
{
	static int power_state;

	pr_debug("Powering %s wifi", (power_on ? "on" : "off"));

	if (power_on == power_state)
		return 0;
	power_state = power_on;

	if (power_on) {
		gpio_set_value(GPIO_WIFI_PMENA, 1);
		mdelay(15);
		gpio_set_value(GPIO_WIFI_PMENA, 0);
		mdelay(1);
		gpio_set_value(GPIO_WIFI_PMENA, 1);
		mdelay(70);
	} else {
		gpio_set_value(GPIO_WIFI_PMENA, 0);
	}

	return 0;
}

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
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
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_wp	= -EINVAL,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
	{}	/* Terminator */
};

static struct wl12xx_platform_data omap4_panda_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
};

static struct regulator_consumer_supply sdp4430_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "mmci-omap-hs.0",
	},
};

static struct regulator_consumer_supply sdp4430_cam2_supply[] = {
	{
		.supply = "cam2pwr",
	},
};
static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}

	/* Set the MMC5 (wlan) power function */
	if (pdev->id == 4)
		pdata->slots[0].set_power = wifi_set_power;

	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev)
		return;

	pdata = dev->platform_data;
	pdata->init = omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static struct regulator_init_data sdp4430_vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct regulator_consumer_supply sdp4430_vaux2_supply[] = {
	REGULATOR_SUPPLY("av-switch", "soc-audio"),
};

static struct regulator_init_data sdp4430_vaux2 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= sdp4430_vaux2_supply,
};

static struct regulator_init_data sdp4430_vaux3 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = sdp4430_cam2_supply,
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data sdp4430_vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = sdp4430_vmmc_supply,
};

static struct regulator_init_data sdp4430_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct regulator_init_data sdp4430_vusim = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct regulator_init_data sdp4430_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct regulator_init_data sdp4430_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct regulator_init_data sdp4430_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct regulator_init_data sdp4430_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct twl4030_madc_platform_data sdp4430_gpadc_data = {
	.irq_line	= 1,
};

static int sdp4430_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};

static struct twl4030_bci_platform_data sdp4430_bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= sdp4430_batt_table,
	.tblsize			= ARRAY_SIZE(sdp4430_batt_table),
};

static void omap4_audio_conf(void)
{
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP);
}

static int tps6130x_enable(int on)
{
	u8 val = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &val, TWL6040_REG_GPOCTL);
	if (ret < 0) {
		pr_err("%s: failed to read GPOCTL %d\n", __func__, ret);
		return ret;
	}

	/* TWL6040 GPO2 connected to TPS6130X NRESET */
	if (on)
		val |= TWL6040_GPO2;
	else
		val &= ~TWL6040_GPO2;

	ret = twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, val, TWL6040_REG_GPOCTL);
	if (ret < 0)
		pr_err("%s: failed to write GPOCTL %d\n", __func__, ret);

	return ret;
}

static struct tps6130x_platform_data tps6130x_pdata = {
	.chip_enable    = tps6130x_enable,
};

static struct regulator_consumer_supply twl6040_vddhf_supply[] = {
	REGULATOR_SUPPLY("vddhf", "twl6040-codec"),
};

static struct regulator_init_data twl6040_vddhf = {
	.constraints = {
		.min_uV                 = 4075000,
		.max_uV                 = 4950000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(twl6040_vddhf_supply),
	.consumer_supplies      = twl6040_vddhf_supply,
	.driver_data            = &tps6130x_pdata,
};

static struct twl4030_codec_audio_data twl6040_audio = {
	.vddhf_uV       = 4075000,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	.max_timeout	= 15000,
	.initial_vibrate = 0,
};

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};

static struct twl4030_platform_data sdp4430_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &sdp4430_vmmc,
	.vpp		= &sdp4430_vpp,
	.vusim		= &sdp4430_vusim,
	.vana		= &sdp4430_vana,
	.vcxio		= &sdp4430_vcxio,
	.vdac		= &sdp4430_vdac,
	.vusb		= &sdp4430_vusb,
	.vaux1		= &sdp4430_vaux1,
	.vaux2		= &sdp4430_vaux2,
	.vaux3		= &sdp4430_vaux3,
	.usb		= &omap4_usbphy_data,
	.madc           = &sdp4430_gpadc_data,
	.bci            = &sdp4430_bci_data,

	/* children */
	.codec          = &twl6040_codec,
};

static struct bq2415x_platform_data sdp4430_bqdata = {
	.max_charger_voltagemV = 4200,
	.max_charger_currentmA = 1550,
};

/*
 * The Clock Driver Chip (TCXO) on OMAP4 based SDP needs to
 * be programmed to output CLK1 based on REQ1 from OMAP.
 * By default CLK1 is driven based on an internal REQ1INT signal
 * which is always set to 1.
 * Doing this helps gate sysclk (from CLK1) to OMAP while OMAP
 * is in sleep states.
 */

static struct cdc_tcxo_platform_data sdp4430_cdc_data = {
	.buf = {
		CDC_TCXO_REQ4INT | CDC_TCXO_REQ1INT |
		CDC_TCXO_REQ4POL | CDC_TCXO_REQ3POL |
		CDC_TCXO_REQ2POL | CDC_TCXO_REQ1POL,
		CDC_TCXO_MREQ4 | CDC_TCXO_MREQ3 |
		CDC_TCXO_MREQ2 | CDC_TCXO_MREQ1,
		0, 0 },
};

static struct pico_platform_data picodlp_platform_data[] = {
	[0] = { /* DLP Controller */
		.gpio_intr = 40,
	},
};

static struct i2c_board_info __initdata tablet_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &sdp4430_twldata,
	},
	{
		I2C_BOARD_INFO("bq24156", 0x6a),
		.platform_data = &sdp4430_bqdata,
	},
	{
		I2C_BOARD_INFO("cdc_tcxo_driver", 0x6c),
		.platform_data = &sdp4430_cdc_data,
	},
	{
		I2C_BOARD_INFO("tps6130x", 0x33),
		.platform_data = &twl6040_vddhf,
	},
};

static struct i2c_board_info __initdata tablet_i2c_2_boardinfo[] = {
	{
		I2C_BOARD_INFO("DP501_i2c_driver", 0x08),
	},
	{
		I2C_BOARD_INFO("picoDLP_i2c_driver", 0x1b),
		.platform_data = &picodlp_platform_data[0],
	},
	{
		I2C_BOARD_INFO("d2l_i2c_driver", 0x0f),
	},
};

static struct i2c_board_info __initdata tablet_i2c_3_boardinfo[] = {
	{
		I2C_BOARD_INFO("tmp105", 0x48),
	},
};

static struct i2c_board_info __initdata tablet_i2c_4_boardinfo[] = {
	{
		I2C_BOARD_INFO(QTOUCH_TS_NAME, 0x4b),
		.platform_data = &atmel_mxt224_ts_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP4_TOUCH_IRQ_1),
	},
	{
		I2C_BOARD_INFO("bmp085", 0x77),
	},
	{
		I2C_BOARD_INFO("hmc5843", 0x1e),
	},
	{
		I2C_BOARD_INFO("bma180_accel", 0x40),
		.platform_data = &bma180accel_platform_data,
	},
	{
		I2C_BOARD_INFO("mpu3050_gyro", 0x68),
		.platform_data = &mpu3050_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP4_MPU3050GYRO_GPIO),
	},
	{
		I2C_BOARD_INFO(TSL2771_NAME, 0x39),
		.platform_data = &tsl2771_data,
		.irq = OMAP_GPIO_IRQ(OMAP4_TSL2771_INT_GPIO),
	},
};

static struct usbhs_omap_platform_data usbhs_pdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static struct omap_i2c_bus_board_data __initdata tablet_i2c_bus_pdata;
static struct omap_i2c_bus_board_data __initdata tablet_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata tablet_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata tablet_i2c_4_bus_pdata;

/*
 * LPDDR2 Configeration Data:
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices = {
	.cs0_device = &elpida_2G_S4,
	.cs1_device = &elpida_2G_S4
};

static void __init omap_i2c_hwspinlock_init(int bus_id, unsigned int
			spinlock_id, struct omap_i2c_bus_board_data *pdata)
{
	pdata->handle = hwspinlock_request_specific(spinlock_id);
	if (pdata->handle != NULL) {
		pdata->hwspinlock_lock = hwspinlock_lock;
		pdata->hwspinlock_unlock = hwspinlock_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", bus_id);
	}
}
static int __init tablet_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &tablet_i2c_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &tablet_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &tablet_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &tablet_i2c_4_bus_pdata);
	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	omap_register_i2c_bus(1, 400, &tablet_i2c_bus_pdata,
		tablet_i2c_boardinfo, ARRAY_SIZE(tablet_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, &tablet_i2c_2_bus_pdata,
		tablet_i2c_2_boardinfo, ARRAY_SIZE(tablet_i2c_2_boardinfo));
	omap_register_i2c_bus(3, 400, &tablet_i2c_3_bus_pdata,
		tablet_i2c_3_boardinfo, ARRAY_SIZE(tablet_i2c_3_boardinfo));
	omap_register_i2c_bus(4, 400, &tablet_i2c_4_bus_pdata,
		tablet_i2c_4_boardinfo, ARRAY_SIZE(tablet_i2c_4_boardinfo));

	/* Setup pull-up resistor for I2C-3 */
	omap2_i2c_pullup(3, I2C_PULLUP_STD_860_OM_FAST_500_OM);

	return 0;
}

static void __init omap4_display_init(void)
{
	void __iomem *phymux_base = NULL;
	u32 val = 0xFFFFC000;

	phymux_base = ioremap(0x4A100000, 0x1000);

	/* Turning on DSI PHY Mux*/
	__raw_writel(val, phymux_base + 0x618);

	/* Set mux to choose GPIO 101 for Taal 1 ext te line*/
	val = __raw_readl(phymux_base + 0x90);
	val = (val & 0xFFFFFFE0) | 0x11B;
	__raw_writel(val, phymux_base + 0x90);

	/* Set mux to choose GPIO 103 for Taal 2 ext te line*/
	val = __raw_readl(phymux_base + 0x94);
	val = (val & 0xFFFFFFE0) | 0x11B;
	__raw_writel(val, phymux_base + 0x94);

	iounmap(phymux_base);

	/* Panel D2L reset and backlight GPIO init */
	gpio_request(blazetablet_dsi_panel.reset_gpio, "dsi1_en_gpio");
	gpio_direction_output(blazetablet_dsi_panel.reset_gpio, 0);

}


/*
 * As OMAP4430 mux HSI and USB signals, when HSI is used (for instance HSI
 * modem is plugged) we should configure HSI pad conf and disable some USB
 * configurations.
 * HSI usage is declared using bootargs variable:
 * board-4430sdp.modem_ipc=hsi
 * Any other or missing value will not setup HSI pad conf, and port_mode[0]
 * will be used by USB.
 * Variable modem_ipc is used to catch bootargs parameter value.
 */

static char *modem_ipc = "n/a";

module_param(modem_ipc, charp, 0);
MODULE_PARM_DESC(modem_ipc, "Modem IPC setting");

static void omap_4430hsi_pad_conf(void)
{
	/*
	 * HSI pad conf: hsi1_ca/ac_wake/flag/data/ready
	 * Also configure gpio_92/95/157/187 used by modem
	 */

	/* hsi1_cawake */
	omap_mux_init_signal("usbb1_ulpitll_clk.hsi1_cawake", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE | \
		OMAP_PIN_OFF_WAKEUPENABLE);
	/* hsi1_caflag */
	omap_mux_init_signal("usbb1_ulpitll_dir.hsi1_caflag", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_cadata */
	omap_mux_init_signal("usbb1_ulpitll_stp.hsi1_cadata", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acready */
	omap_mux_init_signal("usbb1_ulpitll_nxt.hsi1_acready", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acwake */
	omap_mux_init_signal("usbb1_ulpitll_dat0.hsi1_acwake", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acdata */
	omap_mux_init_signal("usbb1_ulpitll_dat1.hsi1_acdata", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acflag */
	omap_mux_init_signal("usbb1_ulpitll_dat2.hsi1_acflag", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_caready */
	omap_mux_init_signal("usbb1_ulpitll_dat3.hsi1_caready", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
	/* gpio_92 */
	omap_mux_init_signal("usbb1_ulpitll_dat4.gpio_92", \
		OMAP_PULL_ENA);
	/* gpio_95 */
	omap_mux_init_signal("usbb1_ulpitll_dat7.gpio_95", \
		OMAP_PIN_INPUT_PULLDOWN | \
		OMAP_PIN_OFF_NONE);
	/* gpio_157 */
	omap_mux_init_signal("usbb2_ulpitll_clk.gpio_157", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* gpio_187 */
	omap_mux_init_signal("sys_boot3.gpio_187", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
}

static void enable_board_wakeup_source(void)
{
	/* Android does not have touchscreen as wakeup source */
#if !defined(CONFIG_ANDROID)
	gpio_val = omap_mux_get_gpio(OMAP4_TOUCH_IRQ_1);
	if ((gpio_val & OMAP44XX_PADCONF_WAKEUPENABLE0) == 0) {
		gpio_val |= OMAP44XX_PADCONF_WAKEUPENABLE0;
		omap_mux_set_gpio(gpio_val, OMAP4_TOUCH_IRQ_1);
	}

#endif

	omap_mux_init_signal("gpmc_a22.gpio_46",
			OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_MUX_MODE3 |
			OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_a23.gpio_47",
			OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_MUX_MODE3 |
			OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_a19.gpio_43",
			OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_MUX_MODE3 |
			OMAP_INPUT_EN);
	/*
	 * Enable IO daisy for sys_nirq1/2, to be able to
	 * wakeup from interrupts from PMIC/Audio IC.
	 * Needed only in Device OFF mode.
	 */
	omap_mux_enable_wakeup("sys_nirq1");
	omap_mux_enable_wakeup("sys_nirq2");

	if (!strcmp(modem_ipc, "hsi")) {
		/*
		 * Enable IO daisy for HSI CAWAKE line, to be able to
		 * wakeup from interrupts from Modem.
		 * Needed only in Device OFF mode.
		 */
		omap_mux_enable_wakeup("usbb1_ulpitll_clk.hsi1_cawake");
	}

}

static struct omap_volt_pmic_info omap4430_pmic_core = {
	.name = "twl",
	.slew_rate = 8000,
	.step_size = 12660,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x61,
	.i2c_cmdreg = 0x62,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x08,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x8,
	.vp_vlimitto_vddmax = 0x26,
};

static struct omap_volt_pmic_info omap4460_pmic_core = {
	.name = "twl",
	.slew_rate = 8000,
	.step_size = 12660,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x55,
	.i2c_cmdreg = 0x56,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x08,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x8,
	.vp_vlimitto_vddmax = 0x26,
};

static struct omap_volt_pmic_info omap_pmic_mpu = {
	.name = "twl",
	.slew_rate = 8000,
	.step_size = 12660,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x55,
	.i2c_cmdreg = 0x56,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x08,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x8,
	.vp_vlimitto_vddmax = 0x35,
};

static struct omap_volt_pmic_info omap_pmic_iva = {
	.name = "twl",
	.slew_rate = 8000,
	.step_size = 12660,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x5b,
	.i2c_cmdreg = 0x5c,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x08,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x8,
	.vp_vlimitto_vddmax = 0x2B,
};

static struct omap_volt_vc_data vc443x_config = {
	.vdd0_on	= 1388000,	/* 1.388v */
	.vdd0_onlp	= 1025000,	/* 1.025v */
	.vdd0_ret	=  750000,	/* 0.75v  */
	.vdd0_off	=	0,	/* 0 v    */
	.vdd1_on	= 1291000,	/* 1.291v */
	.vdd1_onlp	=  950000,	/* 0.95v  */
	.vdd1_ret	=  750000,	/* 0.75v  */
	.vdd1_off	=	0,	/* 0 v    */
	.vdd2_on	= 1127000,	/* 1.127v */
	.vdd2_onlp	=  962000,	/* 0.962v */
	.vdd2_ret	=  750000,	/* 0.75v  */
	.vdd2_off	=	0,	/* 0 v    */
};

static struct omap_volt_vc_data vc443x_config_old = {
	.vdd0_on	= 1350000,	/* 1.35v   */
	.vdd0_onlp	= 1350000,	/* 1.35v   */
	.vdd0_ret	=  837500,	/* 0.8375v */
	.vdd0_off	=	0,	/* 0 v     */
	.vdd1_on	= 1100000,	/* 1.1v    */
	.vdd1_onlp	= 1100000,	/* 1.1v    */
	.vdd1_ret	=  837500,	/* 0.8375v */
	.vdd1_off	=	0,	/* 0 v     */
	.vdd2_on	= 1100000,	/* 1.1v    */
	.vdd2_onlp	= 1100000,	/* 1.1v    */
	.vdd2_ret	=  837500,	/* 0.8375v */
	.vdd2_off	=	0,	/* 0 v     */
};

static struct omap_volt_vc_data vc446x_config = {
	.vdd0_on	= 1380000,	/* 1.38v  */
	.vdd0_onlp	= 1025000,	/* 1.025v */
	.vdd0_ret	=  750000,	/* 0.75v  */
	.vdd0_off	=	0,	/* 0 v    */
	.vdd1_on	= 1375000,	/* 1.375v */
	.vdd1_onlp	=  950000,	/* 0.95v  */
	.vdd1_ret	=  750000,	/* 0.75v  */
	.vdd1_off	=	0,	/* 0 v    */
	.vdd2_on	= 1127000,	/* 1.127v */
	.vdd2_onlp	=  962000,	/* 0.962v */
	.vdd2_ret	=  750000,	/* 0.75v  */
	.vdd2_off	=	0,	/* 0 v    */
};

static void plat_hold_wakelock(void *up, int flag)
{
	struct uart_omap_port *up2 = (struct uart_omap_port *)up;
	/*Specific wakelock for bluetooth usecases*/
	if ((up2->pdev->id == BLUETOOTH_UART)
		&& ((flag == WAKELK_TX) || (flag == WAKELK_RX)))
		wake_lock_timeout(&uart_lock, 2*HZ);

	/*Specific wakelock for console usecases*/
	if ((up2->pdev->id == CONSOLE_UART)
		&& ((flag == WAKELK_IRQ) || (flag == WAKELK_RESUME)))
		wake_lock_timeout(&uart_lock, 5*HZ);
	return;
}

static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = NULL,
		.rts_padconf	= 0,
		.rts_override	= 0,
		.padconf	= OMAP4_CTRL_MODULE_PAD_SDMMC1_CMD_OFFSET,
		.padconf_wake_ev = 0,
		.wk_mask	= 0,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = plat_hold_wakelock,
		.rts_padconf	= OMAP4_CTRL_MODULE_PAD_UART2_RTS_OFFSET,
		.rts_override	= 0,
		.padconf	= OMAP4_CTRL_MODULE_PAD_UART2_RX_OFFSET,
		.padconf_wake_ev =
			OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_3,
		.wk_mask	=
			OMAP4_UART2_TX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART2_RX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART2_RTS_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART2_CTS_DUPLICATEWAKEUPEVENT_MASK,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = plat_hold_wakelock,
		.rts_padconf	= 0,
		.rts_override	= 0,
		.padconf	= OMAP4_CTRL_MODULE_PAD_UART3_RX_IRRX_OFFSET,
		.padconf_wake_ev =
			OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_4,
		.wk_mask	=
			OMAP4_UART3_TX_IRTX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART3_RX_IRRX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART3_RTS_SD_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART3_CTS_RCTX_DUPLICATEWAKEUPEVENT_MASK,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = NULL,
		.rts_padconf	= 0,
		.rts_override	= 0,
		.padconf	= OMAP4_CTRL_MODULE_PAD_UART4_RX_OFFSET,
		.padconf_wake_ev =
			OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_4,
		.wk_mask	=
			OMAP4_UART4_TX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART4_RX_DUPLICATEWAKEUPEVENT_MASK,
	},
	{
		.flags		= 0
	}
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* WLAN IRQ - GPIO 53 */
	OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT |
		  OMAP_PIN_OFF_WAKEUPENABLE),
	/* WLAN_EN - GPIO 54 */
	OMAP4_MUX(GPMC_NWP, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* WLAN SDIO: MMC5 CMD */
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 CLK */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 DAT[0-3] */
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(USBB1_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static void enable_rtc_gpio(void){
	/* To access twl registers we enable gpio6
	 * we need this so the RTC driver can work.
	 */
	gpio_request(TWL6030_RTC_GPIO, "h_SYS_DRM_MSEC");
	gpio_direction_output(TWL6030_RTC_GPIO, 1);

	omap_mux_init_signal("fref_clk0_out.gpio_wk6", \
		OMAP_PIN_OUTPUT | OMAP_PIN_OFF_NONE);
	return;
}

static void omap4_4430sdp_wifi_init(void)
{
	if (gpio_request(GPIO_WIFI_PMENA, "wl12xx") ||
	    gpio_direction_output(GPIO_WIFI_PMENA, 0))
		pr_err("Error initializing up WLAN_EN\n");
	if (wl12xx_set_platform_data(&omap4_panda_wlan_data))
		pr_err("Error setting wl12xx data\n");
}

static void __init tps62361_board_init(void)
{
	int  error;

	omap_mux_init_gpio(TPS62361_GPIO, OMAP_PIN_OUTPUT);
	error = gpio_request(TPS62361_GPIO, "tps62361");
	if (error < 0) {
		pr_err("%s:failed to request GPIO %d, error %d\n",
			__func__, TPS62361_GPIO, error);
		return;
	}

	error = gpio_direction_output(TPS62361_GPIO , 1);
	if (error < 0) {
		pr_err("%s: GPIO configuration failed: GPIO %d,error %d\n",
			__func__, TPS62361_GPIO, error);
		gpio_free(TPS62361_GPIO);
	}
}

static int omap_tshut_init(void)
{
	int status;

	/* Request for gpio_86 line */
	status = gpio_request(OMAP_TSHUT_GPIO, "tshut");
	if (status < 0) {
		pr_err("Failed to request TSHUT GPIO: %d\n", OMAP_TSHUT_GPIO);
		return status;
	}
	status = gpio_direction_input(OMAP_TSHUT_GPIO);
	if (status < 0) {
		pr_err(" GPIO configuration failed for TSHUT GPIO: %d\n",
							OMAP_TSHUT_GPIO);
		gpio_free(OMAP_TSHUT_GPIO);
		return status;
	}

	return 0;
}



static void __init omap_44xxtablet_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, package);

	omap_emif_setup_device_details(&emif_devices, &emif_devices);
	omap_init_emif_timings();
	omap_init_board_version(0);
	omap4_create_board_props();
	enable_rtc_gpio();
	omap4_audio_conf();
	tablet_i2c_init();

	blaze_tablet_touch_init();
	omap_dmm_init();
	omap4_display_init();

	platform_add_devices(blazetablet_devices,
		ARRAY_SIZE(blazetablet_devices));
	printk(KERN_INFO "Configuring Blaze Tablet, Board Revision %d\n",
			system_rev);

	wake_lock_init(&uart_lock, WAKE_LOCK_SUSPEND, "uart_wake_lock");
	omap_serial_init(omap_serial_platform_data);
	omap4_twl6030_hsmmc_init(mmc);
	omap_mpu3050_init();
	blaze_tablet_tsl2771_init();

	omap4_4430sdp_wifi_init();

	/* Power on the ULPI PHY */
	if (gpio_is_valid(OMAP4SDP_MDM_PWR_EN_GPIO)) {
		/* FIXME: Assumes pad is muxed for GPIO mode */
		gpio_request(OMAP4SDP_MDM_PWR_EN_GPIO, "USBB1 PHY VMDM_3V3");
		gpio_direction_output(OMAP4SDP_MDM_PWR_EN_GPIO, 1);
	}

	/*
	 * Test board-4430sdp.modem_ipc bootargs value to detect if HSI pad
	 * conf is required
	 */
	pr_info("Configured modem_ipc: %s", modem_ipc);
	if (!strcmp(modem_ipc, "hsi")) {
		pr_info("Modem HSI detected, set USB port_mode[0] as UNUSED");
		/* USBB1 I/O pads conflict with HSI1 port */
		usbhs_pdata.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED;
		/* Setup HSI pad conf for OMAP4430 platform */
		omap_4430hsi_pad_conf();
	} else
		pr_info("Modem HSI not detected");

	usb_uhhtll_init(&usbhs_pdata);
	usb_musb_init(&musb_board_data);

	status = omap_ethernet_init();
	if (status) {
		pr_err("Ethernet initialization failed: %d\n", status);
	} else {
		sdp4430_spi_board_info[0].irq = gpio_to_irq(ETH_KS8851_IRQ);
		spi_register_board_info(sdp4430_spi_board_info,
				ARRAY_SIZE(sdp4430_spi_board_info));
	}

	omap_display_init(&blazetablet_dss_data);

	enable_board_wakeup_source();

	if (cpu_is_omap443x()) {
		omap_voltage_register_pmic(&omap4430_pmic_core, "core");
		omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
	} else if (cpu_is_omap446x()) {
		omap_voltage_register_pmic(&omap4460_pmic_core, "core");
		tps62361_board_init();
		omap4_tps62361_init();
		status = omap_tshut_init();
		if (status)
			pr_err("TSHUT gpio initialization failed\n");
	}

	omap_voltage_register_pmic(&omap_pmic_iva, "iva");

	if (cpu_is_omap446x())
		omap_voltage_init_vc(&vc446x_config);
	else {
		if (omap_rev() <= OMAP4430_REV_ES2_1)
			omap_voltage_init_vc(&vc443x_config_old);
		else
			omap_voltage_init_vc(&vc443x_config);
	}
}

static void __init omap_4430sdp_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(OMAP_BLAZE, "OMAP4430")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_4430sdp_map_io,
	.init_irq	= omap_4430sdp_init_irq,
	.init_machine	= omap_44xxtablet_init,
	.timer		= &omap_timer,
MACHINE_END
