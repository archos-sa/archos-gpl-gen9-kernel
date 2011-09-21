/*
 * board-archos-a80h.c
 *
  *  Created on: Jan 17, 2011
 *      Author: Niklas Schroeter <schroeter@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/gpio_keys.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/lpddr2-micron.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/feature_list.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/timer-gp.h>
#include <plat/display.h>
#include <linux/delay.h>
#include <plat/usb.h>
#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <plat/mmc.h>
#include <linux/wakelock.h>
#include <plat/opp_twl_tps.h>
#include <plat/hwspinlock.h>

#include <linux/input/cypress-tma340.h>

#include <plat/archos-audio-twl6040.h>
#include <mach/board-archos.h>
#include <linux/usb/gpio_vbus.h>
#include <plat/temperature_sensor.h>

#include <linux/twl6030-vib.h>

#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>

#include "mux.h"
#include "hsmmc.h"
#include "smartreflex-class3.h"
#include "omap_tps6236x.h"

static struct mma8453q_pdata board_mma8453q_pdata;
static struct akm8975_platform_data board_akm8975_pdata;
static struct gpio_vbus_mach_info archos_vbus_info;
static struct wake_lock uart_lock;

#define BLUETOOTH_UART UART2

#define GPIO_5V_PWRON            36	/* fixme: from config tags? */
#define GPIO_1V8_PWRON           34	/* fixme: from config tags? */
#define GPIO_VCC_PWRON           35	/* fixme: from config tags? */
#define GPIO_VBUS_MUSB_PWRON    111	/* fixme: from config tags? */

static void remux_regulator_gpio(int gpio)
{
	switch (gpio) {
	default:
		omap_mux_init_gpio(gpio, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
		break;

	case 104:
		omap_mux_init_signal("gpmc_ncs7.gpio_104", OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
		break;
	}
}


static struct regulator_consumer_supply fixed_reg_5v_consumer[] = {
	REGULATOR_SUPPLY("hsusb_vbus0", "uhhtll-omap"),
	REGULATOR_SUPPLY("5V", "vbus_musb"),
	REGULATOR_SUPPLY("5V", "omap_pwm_led.0"),
};
static struct regulator_init_data fixed_reg_5v_initdata = {
	.constraints = {
		.min_uV = 5000000,
		.max_uV = 5000000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies      = fixed_reg_5v_consumer,
	.num_consumer_supplies  = ARRAY_SIZE(fixed_reg_5v_consumer),
};
static struct fixed_voltage_config fixed_reg_5v = {
	.supply_name	= "5V",
	.microvolts	= 5000000,
	.gpio		= GPIO_5V_PWRON,
	.enable_high	= 1,
	.enabled_at_boot= 0,
	.init_data	= &fixed_reg_5v_initdata,
	.remux		= remux_regulator_gpio,
};
static struct platform_device fixed_supply_5v = {
	.name 	= "reg-fixed-voltage",
	.id	= 0,
	.dev.platform_data = &fixed_reg_5v,
};

static struct regulator_consumer_supply fixed_reg_1v8_consumer[] = {
	REGULATOR_SUPPLY("GPS_1V8", "nl5550.0"),
	REGULATOR_SUPPLY("LCD_1V8", "lcd_panel.0"),
	REGULATOR_SUPPLY("ACCEL_1V8", "3-001c"),
	REGULATOR_SUPPLY("COMPASS_1V8", "3-000c"),
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
	.gpio		= GPIO_1V8_PWRON,
	.enable_high	= 1,
	.enabled_at_boot= 1,
	.init_data	= &fixed_reg_1v8_initdata,
	.remux		= remux_regulator_gpio,
};
static struct platform_device fixed_supply_1v8 = {
	.name 	= "reg-fixed-voltage",
	.id	= 1,
	.dev.platform_data = &fixed_reg_1v8,
};

static struct regulator_consumer_supply fixed_reg_vcc_consumer[] = {
	REGULATOR_SUPPLY("LCD_VCC", "lcd_panel.0"),
	REGULATOR_SUPPLY("ACCEL_VCC", "3-001c"),
	REGULATOR_SUPPLY("COMPASS_VCC", "3-000c"),
	REGULATOR_SUPPLY("hsusb_opt0", "uhhtll-omap"),
};
static struct regulator_init_data fixed_reg_vcc_initdata = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = fixed_reg_vcc_consumer,
	.num_consumer_supplies = ARRAY_SIZE(fixed_reg_vcc_consumer),
};
static struct fixed_voltage_config fixed_reg_vcc = {
	.supply_name	= "VCC",
	.microvolts	= 3300000,
	.gpio		= GPIO_VCC_PWRON,
	.enable_high	= 1,
	.enabled_at_boot= 1,
	.init_data	= &fixed_reg_vcc_initdata,
	.remux		= remux_regulator_gpio,
};
static struct platform_device fixed_supply_vcc = {
	.name 	= "reg-fixed-voltage",
	.id	= 2,
	.dev.platform_data = &fixed_reg_vcc,
};

/*
 * supply vbus_musb, consumer of 5V
 */

static struct regulator_consumer_supply fixed_reg_vbus_musb_consumer[] = {
	REGULATOR_SUPPLY("vbus_musb", "archos_usb_xceiv"),
	REGULATOR_SUPPLY("vbus_musb", "archos_twl6030_usb_xceiv"),
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
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &fixed_reg_vbus_musb_initdata,
	.remux			= remux_regulator_gpio,
};

static struct platform_device fixed_supply_vbus_musb = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &fixed_reg_vbus_musb,
	},
};

/* MMC1 "virtual" regulators */
static struct regulator_consumer_supply fixed_reg_vmmc_consumer[] = {
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.0"),
};
static struct regulator_init_data fixed_reg_vmmc_initdata = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
	},
	.supply_regulator = "1V8",
	.consumer_supplies = fixed_reg_vmmc_consumer,
	.num_consumer_supplies = ARRAY_SIZE(fixed_reg_vmmc_consumer),
};
static struct fixed_voltage_config fixed_reg_vmmc = {
	.supply_name		= "vmmc1",
	.microvolts		= 1800000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &fixed_reg_vmmc_initdata,
};
static struct platform_device fixed_supply_vmmc = {
	.name 	= "reg-fixed-voltage",
	.id	= 4,
	.dev.platform_data = &fixed_reg_vmmc,
};

static struct regulator_consumer_supply fixed_reg_vmmc_aux_consumer[] = {
	REGULATOR_SUPPLY("vmmc_aux", "mmci-omap-hs.0"),
};
static struct regulator_init_data fixed_reg_vmmc_aux_initdata = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
	},
	.supply_regulator = "VCC",
	.consumer_supplies = fixed_reg_vmmc_aux_consumer,
	.num_consumer_supplies = ARRAY_SIZE(fixed_reg_vmmc_aux_consumer),
};
static struct fixed_voltage_config fixed_reg_vmmc_aux = {
	.supply_name		= "vmmc1_aux",
	.microvolts		= 3300000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &fixed_reg_vmmc_aux_initdata,
};
static struct platform_device fixed_supply_vmmc_aux = {
	.name 	= "reg-fixed-voltage",
	.id	= 5,
	.dev.platform_data = &fixed_reg_vmmc_aux,
};

static struct archos_usb_config usb_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.enable_usb_ehci = 42,
		.enable_usb_hub = 60,
		.enable_5v = 40,
	},
	.rev[1] = {
		.enable_usb_ehci = 42,
		.enable_usb_hub = 60,
		.enable_5v = 40,
	},
	.rev[2] = {
		.enable_usb_ehci = 42,
		.enable_usb_hub = 60,
		.enable_5v = 40,
	},
	.rev[3] = {
		.enable_usb_ehci = 42,
		.enable_usb_hub = 60,
		.enable_5v = 40,
	},
	.rev[4] = {
		.enable_usb_ehci = 42,
		.enable_usb_hub = 60,
		.enable_5v = 40,
	},
	.rev[5] = {
		.enable_usb_ehci = 42,
		.enable_usb_hub = 60,
		.enable_5v = 40,
	},
};

static struct archos_sata_config sata_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.sata_power = 135,
		.sata_ready = UNUSED_GPIO,
		.hdd_power  = 100,
		.hdd_power_mux = "gpmc_wait2.gpio_100",
	},
	.rev[1] = {
		.sata_power = 135,
		.sata_ready = UNUSED_GPIO,
		.hdd_power  = 100,
		.hdd_power_mux = "gpmc_wait2.gpio_100",
	},
	.rev[2] = {
		.sata_power = 135,
		.sata_ready = UNUSED_GPIO,
		.hdd_power  = 100,
		.hdd_power_mux = "gpmc_wait2.gpio_100",
		.usb_suspend = 1,
	},
	.rev[3] = {
		.sata_power = 135,
		.sata_ready = UNUSED_GPIO,
		.hdd_power  = 100,
		.hdd_power_mux = "gpmc_wait2.gpio_100",
		.usb_suspend = 1,
	},
	.rev[4] = {
		.sata_power = 135,
		.sata_ready = UNUSED_GPIO,
		.hdd_power  = 100,
		.hdd_power_mux = "gpmc_wait2.gpio_100",
		.usb_suspend = 1,
	},
	.rev[5] = {
		.sata_power = 135,
		.sata_ready = UNUSED_GPIO,
		.hdd_power  = 100,
		.hdd_power_mux = "gpmc_wait2.gpio_100",
		.usb_suspend = 1,
	},
};

static struct archos_i2c_tsp_config i2c_tsp_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.irq_gpio = 112,
		.irq_signal = "abe_mcbsp2_dx.gpio_112",
		.pwr_gpio = 110,
		.shtdwn_gpio = 0,
	},
	.rev[1] = {
		.irq_gpio = 112,
		.irq_signal = "abe_mcbsp2_dx.gpio_112",
		.pwr_gpio = 110,
		.shtdwn_gpio = 0,
	},
	.rev[2] = {
		.irq_gpio = 112,
		.irq_signal = "abe_mcbsp2_dx.gpio_112",
		.pwr_gpio = 110,
		.shtdwn_gpio = 0,
	},
	.rev[3] = {
		.irq_gpio = 112,
		.irq_signal = "abe_mcbsp2_dx.gpio_112",
		.pwr_gpio = 110,
		.shtdwn_gpio = 0,
	},
	.rev[4] = {
		.irq_gpio = 112,
		.irq_signal = "abe_mcbsp2_dx.gpio_112",
		.pwr_gpio = 110,
		.shtdwn_gpio = 0,
	},
	.rev[5] = {
		.irq_gpio = 112,
		.irq_signal = "abe_mcbsp2_dx.gpio_112",
		.pwr_gpio = 110,
		.shtdwn_gpio = 0,
	},
};

static struct archos_gps_config gps_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.gps_enable = 41,
		.gps_int    = UNUSED_GPIO,
		.gps_reset  = UNUSED_GPIO,
	},
	.rev[1] = {
		.gps_enable = 41,
		.gps_int    = UNUSED_GPIO,
		.gps_reset  = UNUSED_GPIO,
	},
	.rev[2] = {
		.gps_enable = 41,
		.gps_int    = UNUSED_GPIO,
		.gps_reset  = UNUSED_GPIO,
	},
	.rev[3] = {
		.gps_enable = 41,
		.gps_int    = UNUSED_GPIO,
		.gps_reset  = UNUSED_GPIO,
	},
	.rev[4] = {
		.gps_enable = 41,
		.gps_int    = UNUSED_GPIO,
		.gps_reset  = UNUSED_GPIO,
	},
	.rev[5] = {
		.gps_enable = 41,
		.gps_int    = UNUSED_GPIO,
		.gps_reset  = UNUSED_GPIO,
	},
};

static struct archos_wifi_bt_config board_wifi_bt_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.wifi_power = 103,
		.wifi_power_signal = "gpmc_ncs6.gpio_103",
		.wifi_irq   = 102,
		.wifi_irq_signal = "gpmc_ncs5.gpio_102",
		.bt_power   = 104,
		.fm_power   = UNUSED_GPIO,
		.gps_power  = UNUSED_GPIO,
	},
	.rev[1] = {
		.wifi_power = 103,
		.wifi_power_signal = "gpmc_ncs6.gpio_103",
		.wifi_irq   = 102,
		.wifi_irq_signal = "gpmc_ncs5.gpio_102",
		.bt_power   = 104,
		.fm_power   = UNUSED_GPIO,
		.gps_power  = UNUSED_GPIO,
	},
	.rev[2] = {
		.wifi_power = 103,
		.wifi_power_signal = "gpmc_ncs6.gpio_103",
		.wifi_irq   = 102,
		.wifi_irq_signal = "gpmc_ncs5.gpio_102",
		.bt_power   = 104,
		.fm_power   = UNUSED_GPIO,
		.gps_power  = UNUSED_GPIO,
	},
	.rev[3] = {
		.wifi_power = 103,
		.wifi_power_signal = "gpmc_ncs6.gpio_103",
		.wifi_irq   = 102,
		.wifi_irq_signal = "gpmc_ncs5.gpio_102",
		.bt_power   = 104,
		.fm_power   = UNUSED_GPIO,
		.gps_power  = UNUSED_GPIO,
	},
	.rev[4] = {
		.wifi_power = 103,
		.wifi_power_signal = "gpmc_ncs6.gpio_103",
		.wifi_irq   = 102,
		.wifi_irq_signal = "gpmc_ncs5.gpio_102",
		.bt_power   = 104,
		.fm_power   = UNUSED_GPIO,
		.gps_power  = UNUSED_GPIO,
	},
	.rev[5] = {
		.wifi_power = 103,
		.wifi_power_signal = "gpmc_ncs6.gpio_103",
		.wifi_irq   = 102,
		.wifi_irq_signal = "gpmc_ncs5.gpio_102",
		.bt_power   = 104,
		.fm_power   = UNUSED_GPIO,
		.gps_power  = UNUSED_GPIO,
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


static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = -1,
	.dev_name = "/dev/ttyO1",
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
	
	conf_ptr = hwrev_ptr(&board_wifi_bt_config, hardware_rev);
	if (IS_ERR(conf_ptr))
		return -EINVAL;
	
	wilink_pdata.nshutdown_gpio = conf_ptr->bt_power;
	remux_regulator_gpio(conf_ptr->bt_power);

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

// camera
static struct i2c_board_info __initdata board_i2c_2_boardinfo[] = {
// FIXME

};

// accel compas
static struct i2c_board_info __initdata board_i2c_3_boardinfo[] = {
	{
		I2C_BOARD_INFO("mma8453q", 0x1c),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_mma8453q_pdata,
	},
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_akm8975_pdata,
	},
// FIXME

};

static struct cypress_tma340_platform_data board_tma340_pdata = {
	.flags = CYPRESS_TMA340_FLAGS_INV_Y,
};

// tsp
static struct i2c_board_info __initdata board_i2c_4_boardinfo[] = {
	{
		I2C_BOARD_INFO(CYPRESS_TMA340_NAME, CYPRESS_TMA340_ADDR),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_tma340_pdata,
	},
};

static struct archos_accel_config accel_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.accel_int1 = 45,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[1] = {
		.accel_int1 = 45,
		.accel_int2 = 174,
	},
	.rev[2] = {
		.accel_int1 = 45,
		.accel_int2 = 174,
	},
	.rev[3] = {
		.accel_int1 = 45,
		.accel_int2 = 174,
	},
	.rev[4] = {
		.accel_int1 = 45,
		.accel_int2 = 174,
	},
	.rev[5] = {
		.accel_int1 = 45,
		.accel_int2 = 174,
	},
};

static struct archos_compass_config compass_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.data_ready = 51,
	},
	.rev[1] = {
		.data_ready = 51,
	},
	.rev[2] = {
		.data_ready = 51,
	},
	.rev[3] = {
		.data_ready = 51,
	},
	.rev[4] = {
		.data_ready = 51,
	},
	.rev[5] = {
		.data_ready = 51,
	},
};

static struct archos_audio_twl6040_config audio_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.power_on = 127,
	},
	.rev[1] = {
		.power_on = 127,
	},
	.rev[2] = {
		.power_on = 127,
	},
	.rev[3] = {
		.power_on = 127,
	},
	.rev[4] = {
		.power_on = 127,
	},
	.rev[5] = {
		.power_on = 127,
	},
};

static struct archos_charge_config charge_config __initdata = {
	.nrev = 6,
	.rev[0] = { 
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type	= CHARGER_TWL6030USB_DC,
	},
	.rev[1] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type	= CHARGER_TWL6030USB_DC,
	},
	.rev[2] = { 
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type	= CHARGER_TWL6030USB_DC,
	},
	.rev[3] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type	= CHARGER_TWL6030USB_DC,
	},
	.rev[4] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type	= CHARGER_TWL6030USB_DC,
	},
	.rev[5] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type	= CHARGER_TWL6030USB_DC,
	},
};

static struct archos_display_config display_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.lcd_pwon = 	38,
		.lcd_rst = 	53,
		.lcd_pci = 	UNUSED_GPIO,
		.lvds_en = 	39,
		.lcd_stdby = 	101,
		.lcd_avdd_en = 	12,
		.hdmi_pwr = 	37,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{
			.src = OMAP_DM_PWM,
			.timer = 11,
			.signal ="abe_dmic_din2.dmtimer11_pwm_evt",
		},
		.bkl_en = 	122,
		.bkl_pwr =	UNUSED_GPIO,
		.use_fixed_bkl = 1,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[1] = {
		.lcd_pwon = 	38 ,
		.lcd_rst = 	53 ,
		.lcd_pci = 	UNUSED_GPIO,
		.lvds_en = 	39 ,
		.lcd_stdby = 	101,
		.lcd_avdd_en = 	12 ,
		.hdmi_pwr = 	37 ,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{
			.src = OMAP_DM_PWM,
			.timer = 11,
			.signal ="abe_dmic_din2.dmtimer11_pwm_evt",
		},
		.bkl_en = 	122,
		.bkl_pwr =	UNUSED_GPIO,
		.use_fixed_bkl = 1,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[2] = {
		.lcd_pwon = 	38 ,
		.lcd_rst = 	53 ,
		.lcd_pci = 	UNUSED_GPIO,
		.lvds_en = 	39 ,
		.lcd_stdby = 	101,
		.lcd_avdd_en = 	12 ,
		.hdmi_pwr = 	37 ,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{
			.src = TWL6030_PWM,
			.timer = 2 // Pmic's PWM1 -> id = 2.
		},
		.bkl_en = 	122,
		.bkl_pwr =	UNUSED_GPIO,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[3] = {
		.lcd_pwon = 	38 ,
		.lcd_rst = 	53 ,
		.lcd_pci = 	UNUSED_GPIO,
		.lvds_en = 	39 ,
		.lcd_stdby = 	101,
		.lcd_avdd_en = 	12 ,
		.hdmi_pwr = 	37 ,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{
			.src = TWL6030_PWM,
			.timer = 2 // Pmic's PWM1 -> id = 2.
		},
		.bkl_en = 	122,
		.bkl_pwr =	UNUSED_GPIO,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[4] = {
		.lcd_pwon = 	38 ,
		.lcd_rst = 	53 ,
		.lcd_pci = 	UNUSED_GPIO,
		.lvds_en = 	39 ,
		.lcd_stdby = 	101,
		.lcd_avdd_en = 	12 ,
		.hdmi_pwr = 	37 ,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{
			.src = TWL6030_PWM,
			.timer = 2 // Pmic's PWM1 -> id = 2.
		},
		.bkl_en = 	122,
		.bkl_pwr =	UNUSED_GPIO,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[5] = {
		.lcd_pwon = 	38 ,
		.lcd_rst = 	53 ,
		.lcd_pci = 	UNUSED_GPIO,
		.lvds_en = 	39 ,
		.lcd_stdby = 	101,
		.lcd_avdd_en = 	12 ,
		.hdmi_pwr = 	37 ,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{
			.src = TWL6030_PWM,
			.timer = 2 // Pmic's PWM1 -> id = 2.
		},
		.bkl_en = 	122,
		.bkl_pwr =	UNUSED_GPIO,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
};

static struct archos_leds_config leds_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.power_led = UNUSED_GPIO,
		.backlight_led = { .timer = -1 },
		.bkl_invert = 1,
		.backlight_power = UNUSED_GPIO,
		.bkl_max = 178,
		.bkl_regulator_name = "5V",
		.bkl_freq = 30000,
	},
	.rev[1] = {
		.power_led = UNUSED_GPIO,
		.backlight_led = { .timer = -1 },
		.bkl_invert = 1,
		.backlight_power = UNUSED_GPIO,
		.bkl_max = 178,
		.bkl_regulator_name = "5V",
		.bkl_freq = 30000,
	},
	.rev[2] = {
		.power_led = UNUSED_GPIO,
		.backlight_led = {
			.src = OMAP_DM_PWM,
			.timer = 8,
			.signal = "uart3_rx_irrx.dmtimer8_pwm_evt",
			.signal_off = "uart3_rx_irrx.safe_mode",
		},
		.bkl_invert = 1,
		.backlight_power = UNUSED_GPIO,
		.bkl_max = 254,	//200,
		.bkl_regulator_name = "5V",
		.bkl_freq = 30000,
	},
	.rev[3] = {
		.power_led = UNUSED_GPIO,
		.backlight_led = {
			.src = OMAP_DM_PWM,
			.timer = 8,
			.signal = "uart3_rx_irrx.dmtimer8_pwm_evt",
			.signal_off = "uart3_rx_irrx.safe_mode",
		},
		.bkl_invert = 1,
		.backlight_power = UNUSED_GPIO,
		.bkl_max = 254,	//200,
		.bkl_regulator_name = "5V",
		.bkl_freq = 30000,
	},
	.rev[4] = {
		.power_led = UNUSED_GPIO,
		.backlight_led = {
			.src = OMAP_DM_PWM,
			.timer = 8,
			.signal = "uart3_rx_irrx.dmtimer8_pwm_evt",
			.signal_off = "uart3_rx_irrx.safe_mode",
		},
		.bkl_invert = 1,
		.backlight_power = UNUSED_GPIO,
		.bkl_max = 254,	//200,
		.bkl_regulator_name = "5V",
		.bkl_freq = 30000,
	},
	.rev[5] = {
		.power_led = UNUSED_GPIO,
		.backlight_led = {
			.src = OMAP_DM_PWM,
			.timer = 8,
			.signal = "uart3_rx_irrx.dmtimer8_pwm_evt",
			.signal_off = "uart3_rx_irrx.safe_mode",
		},
		.bkl_invert = 1,
		.backlight_power = UNUSED_GPIO,
		.bkl_max = 254,	//200,
		.bkl_regulator_name = "5V",
		.bkl_freq = 30000,
	},
};

static struct archos_musb_config musb_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.gpio_vbus_detect = 48,
		.gpio_vbus_flag = 113,
		.gpio_id = 49,
	},
	.rev[1] = {
		.gpio_vbus_detect = 48,
		.gpio_vbus_flag = 113,
		.gpio_id = 49,
	},
	.rev[2] = {
		.gpio_vbus_detect = 48,
		.gpio_vbus_flag = 113,
		.gpio_id = 49,
	},
	.rev[3] = {
		.gpio_vbus_detect = 48,
		.gpio_vbus_flag = 113,
		.gpio_id = 49,
	},
	.rev[4] = {
		.gpio_vbus_detect = 48,
		.gpio_vbus_flag = 113,
		.gpio_id = 49,
	},
	.rev[5] = {
		.gpio_vbus_detect = 48,
		.gpio_vbus_flag = 113,
		.gpio_id = 49,
	},
};

static struct archos_camera_config camera_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.pwr_down = UNUSED_GPIO,
		.reset = 62,
	},
	.rev[1] = {
		.pwr_down = UNUSED_GPIO,
		.reset = 62,
	},
	.rev[2] = {
		.pwr_down = UNUSED_GPIO,
		.reset = 62,
	},
	.rev[3] = {
		.pwr_down = UNUSED_GPIO,
		.reset = 62,
	},
	.rev[4] = {
		.pwr_down = UNUSED_GPIO,
		.reset = 62,
	},
	.rev[5] = {
		.pwr_down = UNUSED_GPIO,
		.reset = 62,
	},
};

static struct gpio_keys_button gpio_volume_buttons[] = {
	{
		.code			= KEY_VOLUMEUP,
		.desc			= "volume up",
		.wakeup			= 1,
		.active_low		= 1,
	},
	{
		.code			= KEY_VOLUMEDOWN,
		.desc			= "volume down",
		.wakeup			= 1,
		.active_low		= 1,
	},
};

static struct gpio_keys_platform_data gpio_volume_keys_info = {
	.buttons	= gpio_volume_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_volume_buttons),
};

static struct platform_device volume_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_volume_keys_info,
	},
};

static struct platform_device vibrator_device = {
	.name	= TWL6030_VIBRATOR_NAME,
	.id	= -1,
};

static struct archos_usb_gadget_config gadget_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.product_name = "A80H",
		.product_id = 0x1510,
		.ums_luns = 1,
	},
	.rev[1] = {
		.product_name = "A80H",
		.product_id = 0x1510,
		.ums_luns = 1,
	},
	.rev[2] = {
		.product_name = "A80H",
		.product_id = 0x1510,
		.ums_luns = 1,
	},
	.rev[3] = {
		.product_name = "A80H",
		.product_id = 0x1510,
		.ums_luns = 1,
	},
	.rev[4] = {
		.product_name = "A80H",
		.product_id = 0x1510,
		.ums_luns = 1,
	},
	.rev[5] = {
		.product_name = "A80H",
		.product_id = 0x1510,
		.ums_luns = 1,
	},
};

static struct led_pwm pwm_leds[] = {
	{
		.name = "power",
		.default_trigger = "default-on",
		.pwm_id = 1,
		.max_brightness = 255,
		.pwm_period_ns = 7812500,
	},
#ifdef ADD_VCOM_LED_CTRL_FOR_VCOM_TWEAKING
	{
		.name = "vcom",
		.pwm_id = 2,
		.max_brightness = 254,
		// meaningless : pmic's pwms speed not configurable
		.pwm_period_ns = 64,
	},
#endif
};

static struct led_pwm_platform_data archos_pwm_led_data = {
	.num_leds = ARRAY_SIZE(pwm_leds),
	.leds = pwm_leds,
};

static struct platform_device archos_pwm_leds = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev	= {
		.platform_data = &archos_pwm_led_data,
	},
};

static struct omap_board_config_kernel board_config[] __initdata = {
	{ ARCHOS_TAG_AUDIO_TWL6040,	&audio_config},
	{ ARCHOS_TAG_ACCEL,		&accel_config},
	{ ARCHOS_TAG_COMPASS,		&compass_config},
	{ ARCHOS_TAG_CHARGE,		&charge_config},
	{ ARCHOS_TAG_DISPLAY,		&display_config},
	{ ARCHOS_TAG_LEDS,		&leds_config},
	{ ARCHOS_TAG_WIFI_BT,		&board_wifi_bt_config},
	{ ARCHOS_TAG_MUSB,		&musb_config},
	{ ARCHOS_TAG_GPS,		&gps_config},
	{ ARCHOS_TAG_CAMERA,		&camera_config},
	{ ARCHOS_TAG_USB_GADGET,	&gadget_config},
	{ ARCHOS_TAG_USB,		&usb_config},
	{ ARCHOS_TAG_SATA,		&sata_config},
	{ ARCHOS_TAG_I2C_TSP,		&i2c_tsp_config},
};

#ifdef CONFIG_OMAP2_DSS_HDMI

static int hdmi_pwron = -1;

static int archos_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	if (!gpio_is_valid(hdmi_pwron))
		return -EINVAL;
	
	gpio_set_value(hdmi_pwron, 1);
	return 0;
}

static void archos_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(hdmi_pwron))
		gpio_set_value(hdmi_pwron, 0);
}

static __init int archos_hdmi_init(void)
{
	const struct archos_disp_conf *conf;
	int ret;
	void __iomem *phymux_base = NULL;
	unsigned int dsimux = 0xFFFFFFFF;
	
	phymux_base = ioremap(0x4A100000, 0x1000);
	/* Turning on DSI PHY Mux*/
	__raw_writel(dsimux, phymux_base+0x618);
	dsimux = __raw_readl(phymux_base+0x618);

	conf = hwrev_ptr(&display_config, hardware_rev);
	if (IS_ERR(conf))
		return -ENODEV;

	ret = gpio_request(conf->hdmi_pwr , "hdmi_pwron");
	if (IS_ERR_VALUE(ret))
		return -EINVAL;
	
	hdmi_pwron = conf->hdmi_pwr;
	gpio_export(hdmi_pwron, false);
	gpio_direction_output(hdmi_pwron, 0);
	omap_mux_init_gpio(hdmi_pwron, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
	omap_mux_init_signal("hdmi_hpd", OMAP_PIN_INPUT );
	omap_mux_init_signal("hdmi_cec", OMAP_PIN_INPUT_PULLUP );
	omap_mux_init_signal("hdmi_ddc_scl", OMAP_PIN_INPUT_PULLUP );
	omap_mux_init_signal("hdmi_ddc_sda", OMAP_PIN_INPUT_PULLUP );
	return 0;
}

static struct omap_dss_device archos_4430_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.phy.dpi.data_lines = 24,
	.platform_enable = archos_panel_enable_hdmi,
	.platform_disable = archos_panel_disable_hdmi,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

#endif /* CONFIG_OMAP2_DSS_HDMI */

static struct omap_dss_device board_lcd_device;

static struct omap_dss_device *board_dss_devices[] = {
	&board_lcd_device,
#ifdef CONFIG_OMAP2_DSS_HDMI
	&archos_4430_hdmi_device,
#endif /* CONFIG_OMAP2_DSS_HDMI */
};

static struct omap_dss_board_info board_dss_data = {
	.num_devices	=	ARRAY_SIZE(board_dss_devices),
	.devices	=	board_dss_devices,
#ifdef CONFIG_OMAP2_DSS_HDMI
	.default_device	=	&archos_4430_hdmi_device,
#else
	.default_device	=	NULL,
#endif /* CONFIG_OMAP2_DSS_HDMI */
};

static struct platform_device *a80h_devices[] __initdata = {
	&wl127x_device,
	&btwilink_device,
	&fixed_supply_5v,
	&fixed_supply_1v8,
	&fixed_supply_vcc,
	&fixed_supply_vbus_musb,
	&vibrator_device,
	&fixed_supply_vmmc,
	&fixed_supply_vmmc_aux,
	&archos_pwm_leds,
};

static int __init omap4_leds_init(void)
{
	struct feature_tag_screen *screen;

	if (( screen = get_feature_tag(FTAG_SCREEN, feature_tag_size(feature_tag_screen)))) {
		if (!strcmp(screen->vendor, "CMI")) {
			if (screen->backlight != 0) {
				leds_config.rev[hardware_rev].bkl_max = screen->backlight;
				printk(KERN_INFO "Set Backlight value from tag\n");
			}
		}
	}
	return 0;
}

static void __init board_init_irq(void)
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
	.power			= 500,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc            = 2,
		.name		= "internal",
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd        = -EINVAL,
		.gpio_wp        = -EINVAL,
		.ocr_mask       = MMC_VDD_165_195,
		.nonremovable	= true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving   = true,
#endif
	},
	{
		.mmc		= 4,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply board_vmmc_supply[] = {
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

static struct regulator_init_data board_vaux2 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= 1,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct regulator_consumer_supply board_vaux3_supply[] = {
	REGULATOR_SUPPLY("vaux3", "vibrator"),
};

static struct regulator_init_data board_vaux3 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3200000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= 0,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(board_vaux3_supply),
	.consumer_supplies = board_vaux3_supply,
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data board_vmmc = {
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
	.num_consumer_supplies  = ARRAY_SIZE(board_vmmc_supply),
	.consumer_supplies      = board_vmmc_supply,
};

static struct regulator_init_data board_vpp = {
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

static struct regulator_init_data board_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= 1,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct regulator_init_data board_vcxio = {
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
		.always_on	= true,
	},
};

static struct regulator_init_data board_vdac = {
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
		.always_on	= true,
	},
};

static struct regulator_init_data board_vusb = {
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

static struct twl4030_usb_data board_twl6040_usb_data = {
	.name = "archos_usb_xceiv",
	.platform 	= &archos_vbus_info,
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
};

static struct twl4030_codec_audio_data twl6040_audio = {
};

static struct twl4030_codec_data twl6040_codec = {
	.audio_mclk	= 19200000,
	.audio	= &twl6040_audio,
	.naudint_irq    = OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};

static struct twl4030_madc_platform_data board_gpadc_data = {
	.irq_line	= 1,
};

static int board_batt_table[] = {
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

static struct twl4030_bci_platform_data board_bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= board_batt_table,
	.tblsize			= ARRAY_SIZE(board_batt_table),
};

static struct twl4030_platform_data board_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &board_vmmc,
	.vpp		= &board_vpp,
	.vana		= &board_vana,
	.vcxio		= &board_vcxio,
	.vdac		= &board_vdac,
	.vusb		= &board_vusb,
	.vaux2		= &board_vaux2,
	.vaux3		= &board_vaux3,
	.madc           = &board_gpadc_data,
	.bci            = &board_bci_data,

	/* children */
	.codec          = &twl6040_codec,
	.usb            = &board_twl6040_usb_data,
};


static struct omap_i2c_bus_board_data __initdata board_i2c_bus_pdata;
static struct omap_i2c_bus_board_data __initdata board_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata board_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata board_i2c_4_bus_pdata;

// pmic and audio
static struct i2c_board_info __initdata board_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &board_twldata,
	},
};

/*
 * LPDDR2 Configuration Data
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	2 Gb
 *		CS1 -	
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -	
 *	--------------------
 *	TOTAL -		4 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices[] = {
	{
	.cs0_device = &micron_2G_S4,
	.cs1_device = NULL
	},
	{
	.cs0_device = &elpida_2G_S4,
	.cs1_device = NULL
	},
	{
	.cs0_device = &elpida_2G_S4,
	.cs1_device = NULL
	},
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

static int __init omap4_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &board_i2c_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &board_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &board_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &board_i2c_4_bus_pdata);

	omap_register_i2c_bus(1, 400, &board_i2c_bus_pdata,
		board_i2c_boardinfo, ARRAY_SIZE(board_i2c_boardinfo)); // FIXME speed
	omap_register_i2c_bus(2, 100, &board_i2c_2_bus_pdata,
		board_i2c_2_boardinfo, ARRAY_SIZE(board_i2c_2_boardinfo)); // FIXME speed
	omap_register_i2c_bus(3, 100, &board_i2c_3_bus_pdata,
		board_i2c_3_boardinfo, ARRAY_SIZE(board_i2c_3_boardinfo)); // FIXME speed
	omap_register_i2c_bus(4, 400, &board_i2c_4_bus_pdata,
		board_i2c_4_boardinfo, ARRAY_SIZE(board_i2c_4_boardinfo));
	return 0;
}

static void enable_board_wakeup_source(void)
{
	/*
	 * Enable IO daisy for sys_nirq1/2, to be able to
	 * wakeup from interrupts from PMIC/Audio IC.
	 * Needed only in Device OFF mode.
	 */
	omap_mux_enable_wakeup("sys_nirq1");
	omap_mux_enable_wakeup("sys_nirq2");
}

static struct omap_volt_pmic_info omap_pmic_VCORE3 = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
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
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x28,
};

static struct omap_volt_pmic_info omap4460_pmic_VCORE1 = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
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
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x28,
};

static struct omap_volt_pmic_info omap4430_pmic_VCORE1 = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
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
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x39,
};

static struct omap_volt_pmic_info omap_pmic_VCORE2 = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
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
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x2D,
};

static struct omap_volt_vc_data vc_config = {
	.vdd0_on = 1375000,        /* 1.375v */
	.vdd0_onlp = 1375000,      /* 1.375v */
	.vdd0_ret = 837500,       /* 0.8375v */
	.vdd0_off = 0,		/* 0 v */
	.vdd1_on = 1300000,        /* 1.3v */
	.vdd1_onlp = 1300000,      /* 1.3v */
	.vdd1_ret = 837500,       /* 0.8375v */
	.vdd1_off = 0,		/* 0 v */
	.vdd2_on = 1200000,        /* 1.2v */
	.vdd2_onlp = 1200000,      /* 1.2v */
	.vdd2_ret = 837500,       /* .8375v */
	.vdd2_off = 0,		/* 0 v */
};

static struct omap_volt_vc_data vc446x_config = {
	.vdd0_on = 1350000,	/* 1.35v */
	.vdd0_onlp = 1350000,	/* 1.35v */
	.vdd0_ret = 837500,	/* 0.8375v */
	.vdd0_off = 0,		/* 0 v */
	.vdd1_on = 1350000,	/* 1.35v */
	.vdd1_onlp = 1350000,	/* 1.35v */
	.vdd1_ret = 837500,	/* 0.8375v */
	.vdd1_off = 0,		/* 0 v */
	.vdd2_on = 1350000,	/* 1.35v */
	.vdd2_onlp = 1350000,	/* 1.35v */
	.vdd2_ret = 837500,	/* .8375v */
	.vdd2_off = 0,		/* 0 v */
};



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
		.rts_padconf	= 0,
		.rts_override	= 0,
		.padconf	= OMAP4_CTRL_MODULE_PAD_MCSPI1_CS1_OFFSET,
		.padconf_wake_ev = OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_3,
		.wk_mask	= OMAP4_MCSPI1_CS1_DUPLICATEWAKEUPEVENT_MASK /* |
				  OMAP4_UART3_CTS_RCTX_DUPLICATEWAKEUPEVENT_MASK */,
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
	/* ttyO2 */
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
	/* ttyO3 */
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
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

#define GPIO_MSECURE	30
static void __init enable_msecure_gpio(void)
{
	/* To access twl registers we enable gpio_wk30
	 * we need this so the RTC driver can work.
	 */
	gpio_request(GPIO_MSECURE, "MSECURE");
	gpio_direction_output(GPIO_MSECURE, 1);

	omap_mux_init_gpio(GPIO_MSECURE, \
		OMAP_PIN_OUTPUT | OMAP_PIN_OFF_NONE);
	return;
}

static void __init tps62361_board_init(void)
{
	int  error;

	omap_mux_init_signal(TPS62361_PINMUX, OMAP_PIN_OUTPUT);
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

static void __init board_init(void)
{
	struct feature_tag_gpio_volume_keys * volume_keys;
	int package = OMAP_PACKAGE_CBS;
	const struct emif_device_details *emif_config;
	struct feature_tag_sdram *sdram;

	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, package);

	if (( sdram = get_feature_tag(FTAG_SDRAM, feature_tag_size(feature_tag_sdram)))) {
		if (!strcmp(sdram->product, "MT42L64M64D2")) {			// MICRON
			emif_config = &emif_devices[0];
			printk(KERN_INFO "DDR type micron\n");
		} else	if (!strcmp(sdram->product, "EDB4064B2PB")) {		// ELPIDA
			emif_config = &emif_devices[1];
			printk(KERN_INFO "DDR type elpida\n");
		} else	if (!strcmp(sdram->product, "H9TKNNN4KDMPQR")) {	// HYNIX
			printk(KERN_INFO "DDR type hynix\n");
			emif_config = &emif_devices[2];
		} else {
			printk(KERN_INFO "DDR type default\n");
			emif_config = &emif_devices[0];
		}
	} else {
		printk(KERN_INFO "DDR type unknown\n");
		emif_config = &emif_devices[0];
	}

	omap_emif_setup_device_details(emif_config, emif_config);
	omap_init_emif_timings();

	archos_usb_musb_init(&archos_vbus_info);
	enable_msecure_gpio();
	archos_audio_twl6040_init(&twl6040_codec);
	archos_accel_mma8453q_init(&board_mma8453q_pdata);
	archos_compass_init(&board_akm8975_pdata);
	archos_battery_twl4030_bci_init(&board_bci_data);
	omap4_i2c_init();
#ifdef CONFIG_OMAP2_DSS_HDMI
	archos_hdmi_init();
#endif

	omap4_leds_init();
	wlan_1271_config();

	if (panel_cpt_xga_8_init(&board_lcd_device) == 0) {
		board_dss_data.default_device = &board_lcd_device;
#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL
		// using dsi pll not valid on first boards, force dispc clock
		if ( hardware_rev < 4 )
			board_lcd_device.phy.dpi.force_dispc_clk = 1;
#endif
	}

	if ((volume_keys = get_feature_tag(FTAG_HAS_GPIO_VOLUME_KEYS,
					feature_tag_size(feature_tag_gpio_volume_keys)))) {

		gpio_volume_buttons[0].gpio = volume_keys->gpio_vol_up;
		gpio_volume_buttons[1].gpio = volume_keys->gpio_vol_down;

		platform_device_register(&volume_keys_gpio);
	}

	platform_add_devices(a80h_devices, ARRAY_SIZE(a80h_devices));

	archos_touchscreen_tm340_init(&board_tma340_pdata);

	wake_lock_init(&uart_lock, WAKE_LOCK_SUSPEND, "uart_wake_lock");

	/* UART configuration */
	if (hardware_rev >= 3) {
		omap_mux_init_signal("uart3_tx_irtx.safe_mode", OMAP_PIN_INPUT_PULLDOWN);
		omap_serial_platform_data[2].padconf = 0;
		omap_serial_platform_data[2].padconf_wake_ev = 0;
		omap_serial_platform_data[2].wk_mask = 0;
	}
	omap_serial_init(omap_serial_platform_data);

	usb_musb_init(&musb_board_data);

	omap4_twl6030_hsmmc_init(mmc);

	archos_omap4_ehci_init();
	archos_camera_mt9m114_init();
	omap_display_init(&board_dss_data);
	enable_board_wakeup_source();
	
	if ((hardware_rev > 1) || cpu_is_omap446x()) {
		hardware_comp.tps62361 = 1;
		tps62361_board_init();
		omap4_tps62361_init();
		omap_voltage_register_pmic(&omap4460_pmic_VCORE1, "core");
		if (cpu_is_omap446x()) {
			int status = omap_tshut_init();
			if (status)
				pr_err("TSHUT gpio initialization failed\n");
		}
	} else {
		omap_voltage_register_pmic(&omap_pmic_VCORE3, "core");
		omap_voltage_register_pmic(&omap4430_pmic_VCORE1, "mpu");
	}

	regulator_has_full_constraints();

	omap_voltage_register_pmic(&omap_pmic_VCORE2, "iva");

	if (/*(hardware_rev > 1) ||*/ cpu_is_omap446x())
		omap_voltage_init_vc(&vc446x_config);
	else
		omap_voltage_init_vc(&vc_config);
}

static void __init board_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(ARCHOS_A80H, "ARCHOS A80H Board")
	.phys_io	= 0x48000000,
	.io_pg_offst    = ((0xfa000000) >> 18) & 0xfffc, 
	.boot_params	= 0x80000100,
	.map_io		= board_map_io,
	.fixup		= fixup_archos,
	.init_irq	= board_init_irq,
	.init_machine	= board_init,
	.timer		= &omap_timer,
MACHINE_END
