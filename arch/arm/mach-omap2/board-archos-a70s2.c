/*
 *    board-archos-a70s2.c
 *
 *  Created on: Feb 15, 2011
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
#include <asm/feature_list.h>

#include <linux/mma7660fc.h>
#include <linux/mma8453q.h>

#include <linux/input/goodix-gt80x.h>
#include <linux/input/tr16c0-i2c.h>

#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>

#include "mux.h"
#include "omap3-opp.h"
#include "hsmmc.h"
#include "sdram-micron-mt46h32m32lf-6.h"

#define BLUETOOTH_UART UART1

#define GPIO_HDMI_5V    	21	/* fixme: from config tags? */
#define GPIO_1V8PLUS		161

static struct wake_lock uart_lock;

static void remux_regulator_gpio(int gpio)
{
	switch (gpio) {
	case GPIO_1V8PLUS:
		omap_mux_init_gpio(161, OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_INPUT_PULLUP);
		gpio_direction_input(161); // we have a pull-down while active and a pull-up while in OFF mode
		break;
	default:
		omap_mux_init_gpio(gpio, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
		break;
	}
}

static struct regulator_consumer_supply fixed_reg_1v8_consumer[] = {
	REGULATOR_SUPPLY("ACCEL_1V8", "2-004c"),
	REGULATOR_SUPPLY("hsusb0", "uhhtll-omap"),
};
static struct regulator_init_data fixed_reg_1v8_initdata = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = fixed_reg_1v8_consumer,
	.num_consumer_supplies = ARRAY_SIZE(fixed_reg_1v8_consumer),
};
static struct fixed_voltage_config fixed_reg_1v8 = {
	.supply_name	= "1V8",
	.microvolts	= 1800000,
	.gpio		= GPIO_1V8PLUS,
	.remux		= remux_regulator_gpio,
	.enable_high	= false,
	.enabled_at_boot= true,
	.init_data	= &fixed_reg_1v8_initdata,
};
static struct platform_device fixed_supply_1v8 = {
	.name 	= "reg-fixed-voltage",
	.id	= 0,
	.dev.platform_data = &fixed_reg_1v8,
};

static struct regulator_consumer_supply fixed_reg_vcc_consumer[] = {
	REGULATOR_SUPPLY("ACCEL_VCC", "2-004c"),
	REGULATOR_SUPPLY("hsusb_opt0", "uhhtll-omap"),
};
static struct regulator_init_data fixed_reg_vcc_initdata = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.supply_regulator 	= "1V8",
	.consumer_supplies = fixed_reg_vcc_consumer,
	.num_consumer_supplies = ARRAY_SIZE(fixed_reg_vcc_consumer),
};
static struct fixed_voltage_config fixed_reg_vcc = {
	.supply_name	= "VCC",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.enabled_at_boot= 1,
	.init_data	= &fixed_reg_vcc_initdata,
};
static struct platform_device fixed_supply_vcc = {
	.name 	= "reg-fixed-voltage",
	.id	= 1,
	.dev.platform_data = &fixed_reg_vcc,
};

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
static struct mma8453q_pdata board_mma8453q_pdata;

static int board_panel_enable_hdmi(struct omap_dss_device *dssdev);

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
}

static struct archos_display_config display_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.lcd_pwon 	= 157,
		.lcd_rst 	= UNUSED_GPIO,
		.lcd_pci 	= UNUSED_GPIO,
		.hdmi_pwr 	= 21,
		.hdmi_int 	= 110,
		.vcom_pwm 	= { .timer = 10, .mux_cfg = -1 },
		.lvds_en 	= 28,
		.lcd_avdd_en 	= 29,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[1] = {
		.lcd_pwon 	= 157,
		.lcd_rst 	= UNUSED_GPIO,
		.lcd_pci 	= UNUSED_GPIO,
		.hdmi_pwr 	= 21,
		.hdmi_int 	= 110,
		.vcom_pwm 	= { .timer = 10, .mux_cfg = -1 },
		.lvds_en 	= 28,
		.lcd_avdd_en 	= 29,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
        },
	.rev[2] = {
		.lcd_pwon 	= 157,
		.lcd_rst 	= UNUSED_GPIO,
		.lcd_pci 	= UNUSED_GPIO,
		.hdmi_pwr 	= 21,
		.hdmi_int 	= 110,
		.vcom_pwm 	= { .timer = 10, .mux_cfg = -1 },
		.lvds_en 	= 28,
		.lcd_avdd_en 	= 29,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[3] = {
		.lcd_pwon 	= 157,
		.lcd_rst 	= UNUSED_GPIO,
		.lcd_pci 	= UNUSED_GPIO,
		.hdmi_pwr 	= 21,
		.hdmi_int 	= 110,
		.vcom_pwm 	= { .timer = 10, .mux_cfg = -1 },
		.lvds_en 	= 28,
		.lcd_avdd_en 	= 29,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
        },
	.rev[4] = {
		.lcd_pwon 	= 157,
		.lcd_rst 	= UNUSED_GPIO,
		.lcd_pci 	= UNUSED_GPIO,
		.hdmi_pwr 	= 21,
		.hdmi_int 	= 110,
		.vcom_pwm 	= { .timer = 10, .mux_cfg = -1 },
		.lvds_en 	= 28,
		.lcd_avdd_en 	= 29,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
        },
};

static struct archos_i2c_tsp_config i2c_tsp_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.irq_gpio	= 156,
		.pwr_gpio	= 160,
		.shtdwn_gpio	= 27,
	},
	.rev[1] = {
		.irq_gpio	= 156,
		.pwr_gpio	= 160,
		.shtdwn_gpio	= 27,
	},
	.rev[2] = {
		.irq_gpio	= 156,
		.pwr_gpio	= 160,
		.shtdwn_gpio	= 27,
	},
	.rev[3] = {
		.irq_gpio	= 156,
		.pwr_gpio	= 160,
		.shtdwn_gpio	= 27,
	},
	.rev[4] = {
		.irq_gpio	= 156,
		.pwr_gpio	= 160,
		.shtdwn_gpio	= 27,
	},
};

static struct archos_audio_config audio_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.vamp_vbat = UNUSED_GPIO,
		.vamp_dc = UNUSED_GPIO,

		.spdif	= UNUSED_GPIO,
		.hp_on	= 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[1] = {
		.vamp_vbat = UNUSED_GPIO,
		.vamp_dc = UNUSED_GPIO,

		.spdif	= UNUSED_GPIO,
		.hp_on	= 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[2] = {
		.vamp_vbat = UNUSED_GPIO,
		.vamp_dc = UNUSED_GPIO,

		.spdif	= UNUSED_GPIO,
		.hp_on	= 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[3] = {
		.vamp_vbat = UNUSED_GPIO,
		.vamp_dc = UNUSED_GPIO,

		.spdif	= UNUSED_GPIO,
		.hp_on	= 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
	.rev[4] = {
		.vamp_vbat = UNUSED_GPIO,
		.vamp_dc = UNUSED_GPIO,

		.spdif	= UNUSED_GPIO,
		.hp_on	= 22,
		.headphone_plugged = 109,
		.clk_mux = -1,
	},
};

static struct archos_charge_config charge_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.charge_enable 	= 26,
		.charge_high	= 24,
		.charge_low 	= 25,
		.charger_type	= CHARGER_ISL9220,
	},
	.rev[1] = {
		.charge_enable 	= 26,
		.charge_high	= 24,
		.charge_low 	= 25,
		.charger_type	= CHARGER_ISL9220,
	},
	.rev[2] = {
		.charge_enable 	= 26,
		.charge_high	= 24,
		.charge_low 	= 25,
		.charger_type	= CHARGER_ISL9220,
	},
	.rev[3] = {
		.charge_enable 	= 26,
		.charge_high	= 24,
		.charge_low 	= 25,
		.charger_type	= CHARGER_ISL9220,
	},
	.rev[4] = {
		.charge_enable 	= 26,
		.charge_high	= 24,
		.charge_low 	= 25,
		.charger_type	= CHARGER_ISL9220,
	},
};

static struct archos_wifi_bt_config wifi_bt_dev_conf __initdata = {
	.nrev = 5,
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
	},
};

/* wl127x BT, FM, GPS connectivity chip */

static unsigned long retry_suspend = 0;
static int platform_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct kim_data_s *kim_gdata;
	struct st_data_s *core_data;
	kim_gdata = dev_get_drvdata(&pdev->dev);
	core_data = kim_gdata->core_data;
	 if (st_ll_getstate(core_data) != ST_LL_INVALID) {
		 /*Prevent suspend until sleep indication from chip*/
		   while(st_ll_getstate(core_data) != ST_LL_ASLEEP &&
				   (retry_suspend++ < 5)) {
			   return -1;
		   }
	 }
	return 0;
}
static int platform_kim_resume(struct platform_device *pdev)
{
	retry_suspend = 0;
	return 0;
}

static int plat_kim_chip_enable(struct kim_data_s *kim_data)
{
	printk(KERN_INFO"%s\n", __func__);
	/* Configure BT nShutdown to HIGH statbt_powere */
	gpio_set_value(kim_data->nshutdown, GPIO_LOW);
	mdelay(5);      /* FIXME: a proper toggle */
	gpio_set_value(kim_data->nshutdown, GPIO_HIGH);
	mdelay(100);
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
	return 0;
}


static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = -1,
	.dev_name = "/dev/ttyO0",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = platform_kim_suspend,
	.resume = platform_kim_resume,
	.chip_enable = plat_kim_chip_enable,
	.chip_disable = plat_kim_chip_disable,
};

static int __init wlan_1271_config(void)
{
	const struct archos_wifi_bt_dev_conf *conf_ptr;
	
	conf_ptr = hwrev_ptr(&wifi_bt_dev_conf, hardware_rev);
	if (IS_ERR(conf_ptr))
		return -EINVAL;
	
	wilink_pdata.nshutdown_gpio = conf_ptr->bt_power;

	return 0;
}

static struct platform_device wl127x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};
static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct archos_accel_config accel_config __initdata = {
	.nrev = 5,
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
};

static struct archos_camera_config camera_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.pwr_down = 15,
		.reset    = 16,
	},
	.rev[1] = {
		.pwr_down = 15,
		.reset    = 16,
	},
	.rev[2] = {
		.pwr_down = 15,
		.reset    = 16,
	},
	.rev[3] = {
		.pwr_down = 15,
		.reset    = 16,
	},
	.rev[4] = {
		.pwr_down = 15,
		.reset    = 16,
	},
};

/* OPP MPU/IVA Clock Frequency */
struct opp_frequencies {
	unsigned long mpu;
	unsigned long iva;
	unsigned long ena;
};

static struct opp_frequencies opp_freq_add_table[] __initdata = {
  {
	.mpu = 800000000,
	.iva = 660000000,
	.ena = OMAP3630_CONTROL_FUSE_OPP120_VDD1,
  },
  {
	.mpu = 1000000000,
	.iva =  800000000,
	.ena = OMAP3630_CONTROL_FUSE_OPP120_VDD1,
  },
  {
	.mpu = 1200000000,
	.iva =   65000000,
	.ena = OMAP3630_CONTROL_FUSE_OPP120_VDD1,
  },

  { 0, 0, 0 },
};

static struct archos_leds_config leds_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.power_led = 57,
		.pwr_invert = 1,
		.status_led = UNUSED_GPIO,
		.backlight_led = {
			.src = OMAP_DM_PWM,
			.timer = 9,
			.signal = "uart2_cts.gpt9_pwm_evt",
			.signal_off = "uart2_cts.safe_mode",
		},
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
		.bkl_freq = 30000,
		.bkl_regulator_name = "hdmi_5v",
		.bkl_max = 254,
	},
	.rev[1] = {
		.power_led = 57,
		.pwr_invert = 1,
		.status_led = UNUSED_GPIO,
		.backlight_led = {
			.src = OMAP_DM_PWM,
			.timer = 9,
			.signal = "uart2_cts.gpt9_pwm_evt",
			.signal_off = "uart2_cts.safe_mode",
		},
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
		.bkl_freq = 30000,
		.bkl_regulator_name = "hdmi_5v",
		.bkl_max = 254,
	},
	.rev[2] = {
		.power_led = 57,
		.pwr_invert = 1,
		.status_led = UNUSED_GPIO,
		.backlight_led = {
			.src = OMAP_DM_PWM,
			.timer = 9,
			.signal = "uart2_cts.gpt9_pwm_evt",
			.signal_off = "uart2_cts.safe_mode",
		},
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
		.bkl_freq = 30000,
		.bkl_regulator_name = "hdmi_5v",
		.bkl_max = 254,
	},
	.rev[3] = {
		.power_led = 57,
		.pwr_invert = 1,
		.status_led = UNUSED_GPIO,
		.backlight_led = {
			.src = OMAP_DM_PWM,
			.timer = 9,
			.signal = "uart2_cts.gpt9_pwm_evt",
			.signal_off = "uart2_cts.safe_mode",
		},
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
		.bkl_freq = 30000,
		.bkl_regulator_name = "hdmi_5v",
		.bkl_max = 254,
	},
	.rev[4] = {
		.power_led = 57,
		.pwr_invert = 1,
		.status_led = UNUSED_GPIO,
		.backlight_led = {
			.src = OMAP_DM_PWM,
			.timer = 9,
			.signal = "uart2_cts.gpt9_pwm_evt",
			.signal_off = "uart2_cts.safe_mode",
		},
		.backlight_power = UNUSED_GPIO,
		.bkl_invert = 1,
		.bkl_freq = 30000,
		.bkl_regulator_name = "hdmi_5v",
		.bkl_max = 254,
	},
};

static struct archos_sd_config sd_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.sd_power   = 158,
		.sd_detect  = 65,
		.sd_prewarn = 58,
	},
	.rev[1] = {
		.sd_power   = 158,
		.sd_detect  = 65,
		.sd_prewarn = 58,
	},
	.rev[2] = {
		.sd_power   = 158,
		.sd_detect  = 65,
		.sd_prewarn = 58,
	},
	.rev[3] = {
		.sd_power   = 158,
		.sd_detect  = 65,
		.sd_prewarn = 58,
	},
	.rev[4] = {
		.sd_power   = 158,
		.sd_detect  = 65,
		.sd_prewarn = 58,
	},
};


static struct archos_usb_gadget_config gadget_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.product_name = "A70S2",
		.product_id = 0x1560,
		.ums_luns = 2,
	},
	.rev[1] = {
		.product_name = "A70S2",
		.product_id = 0x1560,
		.ums_luns = 2,
	},
	.rev[2] = {
		.product_name = "A70S2",
		.product_id = 0x1560,
		.ums_luns = 2,
	},
	.rev[3] = {
		.product_name = "A70S2",
		.product_id = 0x1560,
		.ums_luns = 2,
	},
	.rev[4] = {
		.product_name = "A70S2",
		.product_id = 0x1560,
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
	{ ARCHOS_TAG_I2C_TSP,	&i2c_tsp_config},
	{ ARCHOS_TAG_USB_GADGET, &gadget_config},
};

#ifdef CONFIG_OMAP2_DSS
static struct omap_dss_device board_lcd_device;

static int board_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	return 0;
}

static void board_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device board_hdmi_device = {
	.name               = "hdmi",
	.driver_name        = "hdmi_panel",
	.type               = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.platform_enable    = board_panel_enable_hdmi,
	.platform_disable   = board_panel_disable_hdmi,
	.channel            = OMAP_DSS_CHANNEL_LCD,
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

static struct regulator_consumer_supply board_vdds_dsi_supply =
	REGULATOR_SUPPLY("vdds_dsi", "dss_dispc");

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
		.always_on		= true,
		.apply_uV		= true,
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
//		.always_on		= true,
		.apply_uV		= true,
	},
	.num_consumer_supplies	= ARRAY_SIZE(board_vdd_cam_supply),
	.consumer_supplies	= board_vdd_cam_supply,
};

static struct twl4030_usb_data board_usb_data = {
	.platform = NULL, /*&archos_vbus_info, */
	.usb_mode	= T2_USB_MODE_ULPI,
	.name = "archos_twl4030_usb_xceiv",
	.caps = TWL_USB_CAPS_CHARGING, 
};

static struct twl4030_gpio_platform_data board_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
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
};

/* fixed dummy regulator for 1.8v vdds_dsi rail */
static struct regulator_init_data board_vdds_dsi = {
	.constraints = {
		.name			= "vdds_dsi",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
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
	.id		= 2,
	.dev.platform_data = &board_vdds_dsi_config,
};

static struct regulator_consumer_supply board_vmmc_ext_supply[] = {
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.0"),
};
static struct regulator_init_data board_vmmc_ext = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
	},
	.supply_regulator 	= "VCC",
	.num_consumer_supplies	= ARRAY_SIZE(board_vmmc_ext_supply),
	.consumer_supplies	= board_vmmc_ext_supply,
};
static struct fixed_voltage_config board_vmmc_ext_config = {
	.supply_name		= "vmmc_ext",
	.microvolts		= 3000000,
	.gpio			= -EINVAL,
	.enable_high		= true,
	.enabled_at_boot	= true,
	.init_data		= &board_vmmc_ext,
};
static struct platform_device board_vmmc_ext_device = {
	.name		= "reg-fixed-voltage",
	.id		= 3,
	.dev.platform_data = &board_vmmc_ext_config
};

static struct regulator_consumer_supply board_vmmc2_supply[] = {
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.1"),
};

static struct regulator_init_data board_vmmc2 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
	},
	.supply_regulator = "1V8",
	.consumer_supplies	= board_vmmc2_supply,
	.num_consumer_supplies	= ARRAY_SIZE(board_vmmc2_supply),
};
static struct fixed_voltage_config board_vmmc2_config = {
	.supply_name		= "vmmc2",
	.microvolts		= 1800000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= true,
	.init_data		= &board_vmmc2,
};
static struct platform_device board_vmmc2_device = {
	.name		= "reg-fixed-voltage",
	.id		= 4,
	.dev.platform_data = &board_vmmc2_config,
};

static struct regulator_consumer_supply fixed_reg_emmc_consumer[] = {
	REGULATOR_SUPPLY("vmmc_aux", "mmci-omap-hs.1"),
};
static struct regulator_init_data fixed_reg_emmc_initdata = {
	.constraints = {
		.min_uV 		= 3300000,
		.max_uV 		= 3300000,
		.valid_ops_mask 	= REGULATOR_CHANGE_STATUS,
	},
	.supply_regulator = "VCC",
	.consumer_supplies = fixed_reg_emmc_consumer,
	.num_consumer_supplies = ARRAY_SIZE(fixed_reg_emmc_consumer),
};
static struct fixed_voltage_config fixed_reg_emmc = {
	.supply_name		= "vmmc2_aux",
	.microvolts		= 3300000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= true,
	.init_data		= &fixed_reg_emmc_initdata,
};
static struct platform_device board_emmc_device = {
	.name 	= "reg-fixed-voltage",
	.id	= 5,
	.dev.platform_data = &fixed_reg_emmc,
};

static struct regulator_consumer_supply fixed_reg_hdmi_5v_consumer[] = {
	REGULATOR_SUPPLY("hdmi_5v", "omap_pwm_led.0"),
	REGULATOR_SUPPLY("hdmi_5v", "2-0070"),
	REGULATOR_SUPPLY("hdmi_5v", "3-0034"),
};

static struct regulator_init_data fixed_reg_hdmi_5v_initdata = {
	.constraints = {
		.min_uV 		= 5000000,
		.max_uV 		= 5000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask 	= REGULATOR_CHANGE_STATUS,
		.boot_on		= true,
	},
	.consumer_supplies = fixed_reg_hdmi_5v_consumer,
	.num_consumer_supplies = ARRAY_SIZE(fixed_reg_hdmi_5v_consumer),
};
static struct fixed_voltage_config fixed_reg_hdmi_5v = {
	.supply_name	= "hdmi_5v",
	.microvolts	= 5000000,
	.gpio		= GPIO_HDMI_5V,
	.enable_high	= 1,
	.init_data	= &fixed_reg_hdmi_5v_initdata,
	.remux		= remux_regulator_gpio,
};

static struct platform_device fixed_supply_hdmi_5v = {
	.name 	= "reg-fixed-voltage",
	.id	= 6,
	.dev.platform_data = &fixed_reg_hdmi_5v,
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

	if (!pdata->regulator) {
		pdata->regulator = regulator_get(dev, "hdmi_5v");
		if (IS_ERR(pdata->regulator)) {
			dev_err(dev, "Unable to get \"hdmi_5v\" regulator\n");
			pdata->regulator = NULL;
		}
	}
	
	if (pdata->regulator) {
		if (on_off)
			regulator_enable(pdata->regulator);
		else
			regulator_disable(pdata->regulator);
	}
}

static struct archos_hdmi_platform_data board_hdmi_pdata[] = {
	{
		.hdmi_pwr = archos_hdmi_power,
	},
	{
		.hdmi_pwr = archos_hdmi_power,
	},
};

static struct i2c_board_info __initdata board_i2c_bus2_info[] = {
	[0] = {
		I2C_BOARD_INFO("wm8988", 0x1a),
		.flags = I2C_CLIENT_WAKE,
	},
	[1] = {
		I2C_BOARD_INFO("tda998X", 0x70),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_hdmi_pdata[0],
	},
	[2] = {
		I2C_BOARD_INFO("tda99Xcec", 0x34),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_hdmi_pdata[1],
	},
};

static struct i2c_board_info __initdata mma7660fc_accel_i2c_boardinfo = {
	I2C_BOARD_INFO("mma7660fc", 0x4c),
	.flags = I2C_CLIENT_WAKE,
	.platform_data = &board_mma7660fc_pdata,
};

static struct i2c_board_info __initdata mma8453fc_accel_i2c_boardinfo = {
	I2C_BOARD_INFO("mma8453q", 0x1c),
	.flags = I2C_CLIENT_WAKE,
	.platform_data = &board_mma8453q_pdata,
};

static struct goodix_gt80x_platform_data board_goodix_pdata = {
	.flags = GOODIX_GT80X_FLAGS_XY_SWAP,
	.init_version = 2,
};

static struct i2c_board_info __initdata goodix_tsp_i2c_boardinfo[] = {
	[0] = {
		I2C_BOARD_INFO(GOODIX_GT801_NAME, GOODIX_GT801_ADDR),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_goodix_pdata,
	}
};

static struct tr16c0_platform_data board_tr16c0_pdata;

static struct tr16c0_platform_data board_tr16c0_pdata = {
	.x_max = 1024,
	.y_max = 600,
	.flags = TR16C0_FLAGS_INV_Y,
};

static struct i2c_board_info __initdata tr16c0_tsp_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(TR16C0_NAME, TR16C0_ADDR),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_tr16c0_pdata,
	},
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
	omap_register_i2c_bus(3, 200, &archos_i2c_bus_pdata, NULL, 0);

	return 0;
}

static struct platform_device *board_devices[] __initdata = {
	&wl127x_device,
	&btwilink_device, 
	&fixed_supply_1v8,
	&fixed_supply_vcc,
	&board_vdds_dsi_device,
	&board_vmmc_ext_device,
	&board_vmmc2_device,
	&board_emmc_device,
	&fixed_supply_hdmi_5v,
};

static void __init archos_hdmi_gpio_init(
		const struct archos_disp_conf* disp_conf)
{
	/* driver will manage the GPIO, just apply the pin multiplexing
	 * archos_gpio_init_input(&disp_conf->hdmi_int, "hdmi irq"); */
	omap_mux_init_gpio(disp_conf->hdmi_int, OMAP_PIN_INPUT);

	/* patch IRQ into HDMI I2C bus info */
	board_i2c_bus2_info[1].irq = gpio_to_irq(disp_conf->hdmi_int);
	board_i2c_bus2_info[2].irq = gpio_to_irq(disp_conf->hdmi_int);
}

static struct omap2_hsmmc_info mmc[] __initdata = {
	{
		.mmc		= 2,
		.name		= "internal",
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.nonremovable	= true,
		.max_freq	= 26000000,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif		
		.vcc_aux_disable_is_sleep = true,
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
		.power_saving	= false,
#endif		
	},
	{}      /* Terminator */
};

static void archos_hsmmc_gpio_preinit(struct omap2_hsmmc_info *mmc)
{
	//int detect = 0;
	int ret = 0;
	struct archos_sd_conf *cfg;

	cfg = hwrev_ptr(&sd_config, hardware_rev);
	if (IS_ERR(cfg))
		return;

	/* Set correct card detect gpio for the external mmc */
	mmc->gpio_cd = cfg->sd_detect;

	/* dummy request. Still not used */
	ret = gpio_request(cfg->sd_prewarn, "sd_prewarn");
	if (!ret) {
		gpio_direction_input(cfg->sd_prewarn);
	}

	/*  Set gpio for sdcard power regulator */
	board_vmmc_ext_config.gpio = cfg->sd_power;
}

static int archos_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
#if 0
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;
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
#elif defined(CONFIG_USB_MUSB_DUAL_ROLE)
	.mode			= MUSB_PERIPHERAL,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode			= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 100,
};

#ifdef CONFIG_SERIAL_OMAP
static void plat_hold_wakelock(void *up, int flag)
{
	struct uart_omap_port *up2 = (struct uart_omap_port *)up;
	/*Specific wakelock for bluetooth usecases*/
	if ((up2->pdev->id == BLUETOOTH_UART)
		&& ((flag == WAKELK_TX) || (flag == WAKELK_RX)))
		wake_lock_timeout(&uart_lock, 2*HZ);

	/*Specific wakelock for console usecases*/
	if ((up2->pdev->id != BLUETOOTH_UART)
		&& ((flag == WAKELK_IRQ) || (flag == WAKELK_RESUME)))
		wake_lock_timeout(&uart_lock, 5*HZ);
	return;
}

static struct omap_uart_port_info omap_serial_platform_data[] = {
	/* ttyO0 */
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = plat_hold_wakelock,
	},
	/* ttyO1 */
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = plat_hold_wakelock,
	},
	/* ttyO2 */
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = plat_hold_wakelock,
	},
	/* ttyO3 */
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = plat_hold_wakelock,
	},
	{
		.flags		= 0
	}
};
#endif /* CONFIG_SERIAL_OMAP */

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* DSS */
	OMAP3_MUX(DSS_PCLK, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_HSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_VSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_ACBIAS, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA0, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA1, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA3, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA4, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA5, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA6, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA7, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA8, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA9, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA10, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA11, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA12, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA13, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA14, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA15, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA16, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA17, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA18, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA19, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA20, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA21, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA22, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA23, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SYS_BOOT0, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SYS_BOOT1, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SYS_BOOT3, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SYS_BOOT4, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SYS_BOOT5, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SYS_BOOT6, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART3_RX_IRRX, OMAP_PIN_INPUT_PULLUP | 
			OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE0),
	

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
#if 0
	gpio_request(161, "pwren2");
	omap_mux_init_gpio(161, OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_INPUT_PULLUP);
	gpio_direction_input(161); // we have a pull-down while active and a pull-up while in OFF mode
#endif
	/* if we can turn off the 1.8V, we need to turn off the VDD_CAM, too */
	board_tps65921_pdata.power->resource_config = twl4030_rconfig_vdd_cam_off;
}

static void __init board_init(void)
{
	struct feature_tag_touchscreen * tsp;

	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);

	/* before omap_i2c_init() or IRQ will not forwarded to driver */
	if (display_config.nrev > hardware_rev)
		archos_hdmi_gpio_init(&display_config.rev[hardware_rev]);

	msecure_init();

	/* offmode config, before I2C config! */
	board_offmode_config();
	omap3_i2c_init();

	wlan_1271_config();

	panel_claa_wsvga_7_init(&board_lcd_device);

#ifdef CONFIG_SERIAL_OMAP
	wake_lock_init(&uart_lock, WAKE_LOCK_SUSPEND, "uart_wake_lock");

	omap_serial_init(omap_serial_platform_data);
#endif
	usb_musb_init(&musb_board_data);

	if (hardware_rev < 4) {
		i2c_register_board_info(2, &mma7660fc_accel_i2c_boardinfo, 1);
		archos_accel_mma7660fc_init(&board_mma7660fc_pdata);
	} else {
		i2c_register_board_info(2, &mma8453fc_accel_i2c_boardinfo, 1);
		archos_accel_mma8453q_init(&board_mma8453q_pdata);
	}

	tsp = get_feature_tag(FTAG_HAS_TOUCHSCREEN, feature_tag_size(feature_tag_touchscreen));

	if (!tsp || (tsp->vendor == 0)) {
		i2c_register_board_info(3, goodix_tsp_i2c_boardinfo, 1);
	} else {
		i2c_register_board_info(3, tr16c0_tsp_i2c_boardinfo, 1);
	}


	//archos_audio_gpio_init();
	
	archos_camera_ov7675_init();
	//archos_keys_init();
	
	omap_display_init(&board_dss_data);

	archos_hsmmc_gpio_preinit(&mmc[1]);
	archos_hsmmc_init(mmc);

	archos_audio_wm8988_init();
	enable_board_wakeup_source();

	regulator_has_full_constraints();

#ifdef CONFIG_PM
#ifdef CONFIG_TWL4030_CORE
	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
#endif
	omap_voltage_init_vc(&vc_config);
#endif

	platform_add_devices(board_devices, ARRAY_SIZE(board_devices));

	/* once VCC regulator has been registered only. */
	if (!tsp || (tsp->vendor == 0)) {
		archos_touchscreen_goodix_init(&board_goodix_pdata);
	} else {
		archos_touchscreen_tr16c0_init(&board_tr16c0_pdata);
	}
}

/* must be called after omap2_common_pm_init() */
static int __init a70s2_opp_init(void)
{
	struct omap_hwmod *mh, *dh;
	struct omap_opp *mopp, *dopp;
	struct device *mdev, *ddev;
	struct opp_frequencies *opp_freq;
	unsigned long hw_support;


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
		hw_support = omap_ctrl_readl(opp_freq->ena);

		if (IS_ERR(mopp))
			mopp = opp_find_freq_exact(mdev, opp_freq->mpu, true);
		if (IS_ERR(mopp) || !hw_support) {
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
device_initcall(a70s2_opp_init);

MACHINE_START(ARCHOS_A70S2, "Archos A70S2 board")
	.phys_io	= 0x48000000,
	.io_pg_offst    = ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= board_map_io,
	.fixup		= fixup_archos,
	.init_irq	= board_init_irq,
	.init_machine	= board_init,
	.timer		= &omap_timer,
MACHINE_END
