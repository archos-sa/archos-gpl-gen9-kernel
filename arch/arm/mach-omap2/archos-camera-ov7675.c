/*
 * A32 camera (OV7675) Board configuration
 *
 */

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <linux/module.h>

#include <mach/gpio.h>
#include <plat/archos-gpio.h>
#include <mach/board-archos.h>

#include "mux.h"

static int reset_gpio;
static int pwdn_gpio;

static int gpio_nb[2];

static struct platform_device camera_device = {
	.name		= "omap-ov7675-isp",
	.id		= -1,
	.dev            = {
		.platform_data = gpio_nb,
	},
};

int __init archos_camera_ov7675_init(void)
{
	const struct archos_camera_config *camera_cfg;

	pr_debug("OV7675 Camera Init\n");

	camera_cfg = omap_get_config( ARCHOS_TAG_CAMERA, struct archos_camera_config );
	if (camera_cfg == NULL) {
		printk(KERN_DEBUG "archos_camera_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= camera_cfg->nrev ) {
		pr_debug("archos_camera_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, camera_cfg->nrev);
		return -ENODEV;
	}

	reset_gpio = camera_cfg->rev[hardware_rev].reset;
	pwdn_gpio = camera_cfg->rev[hardware_rev].pwr_down;

	archos_gpio_init_output(reset_gpio, "cam_reset");
	if (camera_cfg->rev[hardware_rev].reset_mux)
		omap_mux_init_signal(camera_cfg->rev[hardware_rev].reset_mux, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
	gpio_nb[0] = reset_gpio;

	archos_gpio_init_output(pwdn_gpio, "cam_pwdn");
	if (camera_cfg->rev[hardware_rev].pwr_down_mux)
		omap_mux_init_signal(camera_cfg->rev[hardware_rev].pwr_down_mux, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
	gpio_nb[1] = pwdn_gpio;

#if 0
	omap_cfg_reg(A24_34XX_CAM_HS);
	omap_cfg_reg(A23_34XX_CAM_VS);
	omap_cfg_reg(C25_34XX_CAM_XCLKA);
	omap_cfg_reg(C27_34XX_CAM_PCLK);
	omap_cfg_reg(C23_34XX_CAM_FLD);
	omap_cfg_reg(AG17_34XX_CAM_D0);
	omap_cfg_reg(AH17_34XX_CAM_D1);
	omap_cfg_reg(B24_34XX_CAM_D2);
	omap_cfg_reg(C24_34XX_CAM_D3);
	omap_cfg_reg(D24_34XX_CAM_D4);
	omap_cfg_reg(A25_34XX_CAM_D5);
	omap_cfg_reg(K28_34XX_CAM_D6);
	omap_cfg_reg(L28_34XX_CAM_D7);
	omap_cfg_reg(B23_34XX_CAM_WEN);
#endif

	omap_mux_init_signal("cam_d0.cam_d0", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_d1.cam_d1", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_d2.cam_d2", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_d3.cam_d3", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_d4.cam_d4", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_d5.cam_d5", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_d6.cam_d6", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_d7.cam_d7", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_hs.cam_hs", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_vs.cam_vs", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_xclka.cam_xclka", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("cam_pclk.cam_pclk", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_wen.cam_wen", OMAP_PIN_OUTPUT);

	return platform_device_register(&camera_device);
}
