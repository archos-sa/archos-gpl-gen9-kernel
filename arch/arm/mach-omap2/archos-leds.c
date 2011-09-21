/*
 * Led(s) Board configuration
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/stat.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/leds.h>

#include <plat/display.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>
#include <plat/archos-gpio.h>
#include <asm/mach-types.h>
#include <plat/board.h>
#include <mach/board-archos.h>

#include "mux.h"

static struct gpio_led gpio_leds[] = {
	{
		.name			= "power",
		.default_trigger	= "default-on",
		.gpio			= -1,
		.active_low		= 0,
	},
	{
		.name			= "lcd-backlight",
		.default_trigger	= "backlight",
		.gpio			= -1,
		.active_low		= 1,
	}
};

static struct omap_pwm_led_platform_data board_backlight_data = {
	.name			= "lcd-backlight",
	.default_trigger	= "backlight",
};

static int bkl_power_gpio;
struct archos_pwm_conf backlight_led_pwm;
static struct regulator *bkl_reg = NULL;

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

static void bkl_set_pad(struct omap_pwm_led_platform_data *self, int on_off)
{
	if (on_off == 0) {
		if (self->invert)
			omap_mux_init_signal(backlight_led_pwm.signal_off, 
					OMAP_PIN_INPUT_PULLUP);
		else
			omap_mux_init_signal(backlight_led_pwm.signal_off, 
					OMAP_PIN_INPUT_PULLDOWN);
			
	} else
		omap_mux_init_signal(backlight_led_pwm.signal, 
				OMAP_PIN_OUTPUT);	
}

static void bkl_set_power(struct omap_pwm_led_platform_data *self, int on_off)
{
	pr_debug("%s: on_off:%d\n", __func__, on_off);
	
	if (bkl_reg != NULL) {
		if (on_off)
			regulator_enable(bkl_reg);
		else
			regulator_disable(bkl_reg);
	}

	if (gpio_is_valid(bkl_power_gpio))
		gpio_set_value( bkl_power_gpio, on_off );
	
}

static void bkl_set_power_initial(struct omap_pwm_led_platform_data *self)
{
	if (bkl_reg != NULL) {
		regulator_force_disable(bkl_reg);
	}

	if (gpio_is_valid(bkl_power_gpio))
		gpio_set_value( bkl_power_gpio, false );
	
}

static int __init archos_leds_init(void)
{
	const struct archos_leds_config *leds_cfg;
	const struct archos_leds_conf *cfg;
	int power_led;
	int ret;
	
	pr_debug("archos_leds_init\n");
	
	leds_cfg = omap_get_config( ARCHOS_TAG_LEDS, struct archos_leds_config );
	cfg = hwrev_ptr(leds_cfg, hardware_rev);
	if (IS_ERR(cfg)) {
		pr_err("archos_leds_init: no configuration for hwrev %i\n",
				hardware_rev);
		return -ENODEV;
	}

	/* Power Led (GPIO) */

	power_led = cfg->power_led;
	pr_debug("%s: power led on gpio %i\n", __func__, power_led);
	
	if (gpio_is_valid(power_led)) {
		gpio_leds[0].gpio = power_led;
		gpio_leds[0].active_low = cfg->pwr_invert;

		ret = platform_device_register(&board_led_device);
		if (IS_ERR_VALUE(ret))
			pr_err("unable to register power LED\n");
	} else
		pr_debug("%s: no power led configured\n", __func__);

	/* Backlight Led */
	backlight_led_pwm = cfg->backlight_led;
	pr_debug("%s: backlight led on pwm type %d timer %i\n",
			__func__, backlight_led_pwm.src, backlight_led_pwm.timer);

	if (backlight_led_pwm.signal)
		omap_mux_init_signal(backlight_led_pwm.signal, OMAP_PIN_OUTPUT);

	bkl_power_gpio = cfg->backlight_power;
	if (gpio_is_valid(bkl_power_gpio))
		archos_gpio_init_output(bkl_power_gpio, "bkl_power");

	switch (backlight_led_pwm.src) {
		case OMAP_DM_PWM:
			board_backlight_data.intensity_timer = backlight_led_pwm.timer;
			board_backlight_data.bkl_max = cfg->bkl_max;
			board_backlight_data.bkl_freq = cfg->bkl_freq;
			board_backlight_data.invert = cfg->bkl_invert;
			board_backlight_data.set_power = &bkl_set_power;
			if (backlight_led_pwm.signal_off != NULL) {
				board_backlight_data.set_pad = &bkl_set_pad;
				bkl_set_pad(&board_backlight_data, 0);
			}
			break;

		default:
			pr_err("%s: no support for backlight pwm source %d",
					__FUNCTION__, backlight_led_pwm.src);
			return -ENODEV;
	}

	ret = platform_device_register(&board_backlight_device);
	if (ret < 0) {
		pr_err("unable to register backlight LED\n");
		return ret;
	}
	
	if (cfg->bkl_regulator_name) {
		bkl_reg = regulator_get( &board_backlight_device.dev, 
				cfg->bkl_regulator_name);
		if (IS_ERR(bkl_reg)) {
			pr_err("Unable to get LED regulator\n");
			bkl_reg = NULL;
		}
	}

	// set backlight power to 0
	bkl_set_power_initial(&board_backlight_data);

	return 0;
}

device_initcall(archos_leds_init);
