/*
 * MMA8453Q Accelerometer board configuration
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mma8453q.h>

#include <mach/gpio.h>
#include <plat/archos-gpio.h>
#include <asm/mach-types.h>
#include <plat/board.h>
#include <mach/board-archos.h>


int __init archos_accel_mma8453q_init(struct mma8453q_pdata *pdata)
{
	struct archos_accel_conf accel_gpio;
	const struct archos_accel_config *accel_cfg;
	
	accel_cfg = omap_get_config( ARCHOS_TAG_ACCEL, struct archos_accel_config );
	if (accel_cfg == NULL) {
		printk(KERN_DEBUG "archos_accel_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= accel_cfg->nrev ) {
		printk(KERN_DEBUG "archos_accel_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, accel_cfg->nrev);
		return -ENODEV;
	}

	accel_gpio = accel_cfg->rev[hardware_rev];

	/* irq needed by the driver */
	if (accel_gpio.accel_int1 != -1)
		pdata->irq1 = gpio_to_irq(accel_gpio.accel_int1);
	else
		pdata->irq1 = -1;
	if (accel_gpio.accel_int2 != -1)
		pdata->irq2 = gpio_to_irq(accel_gpio.accel_int2);
	else
		pdata->irq2 = -1;
	printk("archos_accel_init: irq1 %d, irq2 %d\n", pdata->irq1, pdata->irq2);

	archos_gpio_init_input(accel_gpio.accel_int1, "accel_int1");
	archos_gpio_init_input(accel_gpio.accel_int2, "accel_int2");
	
	return 0;
}
