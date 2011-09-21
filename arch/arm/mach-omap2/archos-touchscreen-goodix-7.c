/*
 *    archos-touchscreen-goodix.c : 17/02/2011
 *    g.revaillot, revaillot@archos.com
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include <asm/mach-types.h>
#include <mach/board-archos.h>
#include <mach/gpio.h>

#include <plat/archos-gpio.h>

#include <linux/input/goodix-gt80x.h>

static int ts_pwron;
static int ts_shtdwn;

static void set_power(int on_off)
{
	gpio_set_value(ts_pwron, on_off);
}

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
		archos_gpio_init_output(conf->pwr_gpio, "goodix_ts_pwron");
		ts_pwron = conf->pwr_gpio;
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

	pdata->set_power = &set_power;
	pdata->set_shutdown = &set_shutdown;

	pr_debug("%s: irq_gpio %d - irq %d, pwr_gpio %d, shtdwn_gpio %d\n",
			__FUNCTION__, pdata->irq,
			conf->irq_gpio, conf->pwr_gpio, conf->shtdwn_gpio);

	return 0;
}

