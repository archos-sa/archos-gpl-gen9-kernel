/*
 *    archos-touchscreen-goodix.c : 17/02/2011
 *    g.revaillot, revaillot@archos.com
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>

#include <asm/mach-types.h>
#include <mach/board-archos.h>
#include <mach/gpio.h>

#include <plat/archos-gpio.h>

#include <linux/input/goodix-gt80x.h>

static struct regulator_consumer_supply tsp_vcc_consumer[] = {
	REGULATOR_SUPPLY("tsp_vcc", "3-0055"),
};
static struct regulator_init_data fixed_reg_tsp_vcc_initdata = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},

	.supply_regulator = "VCC",
	.consumer_supplies = tsp_vcc_consumer,
	.num_consumer_supplies = ARRAY_SIZE(tsp_vcc_consumer),
};
static struct fixed_voltage_config fixed_reg_tsp_vcc = {
	.supply_name	= "TSP_VCC",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.enable_high	= 1,
	.enabled_at_boot= 0,
	.init_data	= &fixed_reg_tsp_vcc_initdata,
};
static struct platform_device fixed_supply_tsp_vcc = {
	.name 	= "reg-fixed-voltage",
	.id = 7,
	.dev.platform_data = &fixed_reg_tsp_vcc,
};

static int ts_shtdwn;

static void set_shutdown(int on_off)
{
	gpio_set_value(ts_shtdwn, on_off);
}

int __init archos_touchscreen_goodix_init(struct goodix_gt80x_platform_data *pdata)
{
	const struct archos_i2c_tsp_config *tsp_config;
	const struct archos_i2c_tsp_conf *conf;

	tsp_config = omap_get_config(ARCHOS_TAG_I2C_TSP,
			struct archos_i2c_tsp_config);

	if (!tsp_config) {
		pr_err("%s: no configuration tag exist\n", __FUNCTION__);
		return -ENODEV;
	}

	conf = hwrev_ptr(tsp_config, hardware_rev);

	if (IS_ERR(conf)) {
		pr_err("%s: hardware_rev (%i) >= nrev (%i)\n",
			__FUNCTION__, hardware_rev, tsp_config->nrev);
		return -ENODEV;
	}

	if (conf->pwr_gpio > 0) {
		omap_mux_init_gpio(conf->pwr_gpio, PIN_OUTPUT);
		fixed_reg_tsp_vcc.gpio = conf->pwr_gpio;
		platform_device_register(&fixed_supply_tsp_vcc);
	} else {
		pr_err("%s: ts pwron gpio is not valid.\n", __FUNCTION__);
		return -ENODEV;
	}

	if (conf->shtdwn_gpio > 0) {
		archos_gpio_init_output(conf->shtdwn_gpio, "goodix_ts_shtdwn");
		ts_shtdwn = conf->shtdwn_gpio;
	} else {
		pr_err("%s: ts shutdown gpio is not valid.\n", __FUNCTION__);
		return -ENODEV;
	}

	if (conf->irq_gpio > 0) {
		archos_gpio_init_input(conf->irq_gpio, "goodix_ts_irq");
		pdata->irq = gpio_to_irq(conf->irq_gpio);
	} else {
		pr_err("%s: irq shutdown gpio is not valid.\n", __FUNCTION__);
		return -ENODEV;
	}

	pdata->set_shutdown = &set_shutdown;

	pr_debug("%s: irq_gpio %d - irq %d, pwr_gpio %d, shtdwn_gpio %d\n",
			__FUNCTION__, pdata->irq,
			conf->irq_gpio, conf->pwr_gpio, conf->shtdwn_gpio);

	return 0;
}

