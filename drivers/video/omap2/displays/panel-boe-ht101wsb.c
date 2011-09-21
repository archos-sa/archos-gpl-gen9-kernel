/*
 * panel-boe-ht101wsb.c
 *
 *  Created on: Apri 13, 2010
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/display.h>
#include <linux/leds.h>

#include <mach/gpio.h>
#include <linux/fb.h>
#include <plat/display.h>
#include <plat/omap-pm.h>
#include <linux/delay.h>

static void panel_remove(struct omap_dss_device *disp)
{
}

static int panel_power_on(struct omap_dss_device *dssdev)
{
	int r;

	/* 
	 * FIXME: keep bus clock high, otherwise we see GFX_UNDERFLOW errors
	 * when L3 clock scaling happens
	 */
	omap_pm_set_min_bus_tput(&dssdev->dev, 
			OCP_INITIATOR_AGENT, 800000);
	
	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	omap_pm_set_min_bus_tput(&dssdev->dev, 
			OCP_INITIATOR_AGENT, 0);
	return r;
}

static void panel_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omapdss_dpi_display_disable(dssdev);

	omap_pm_set_min_bus_tput(&dssdev->dev, 
			OCP_INITIATOR_AGENT, 0);
}

static int panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void panel_disable(struct omap_dss_device *dssdev)
{
	panel_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int panel_suspend(struct omap_dss_device *dssdev)
{
	panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static int panel_get_recommended_bpp(struct omap_dss_device *dssdev)
{
	return 24;
}

// timings references to define, waiting for visual  !!!
#define BOE_LCD_PIXCLOCK	57700		/* 57,7MHz */

static struct omap_video_timings boe_panel_timings = {
		.x_res		= 1024,
		.y_res		= 600,

		.pixel_clock	= BOE_LCD_PIXCLOCK,	/* Mhz */
// 60hz
		.hsw		= 2,		/* horizontal sync pulse width */
		.hfp		= 272,		/* horizontal front porch */
		.hbp		= 200,		/* horizontal back porch */
		.vsw		= 4,		/* vertical sync pulse width */
		.vfp		= 30,		/* vertical front porch */
		.vbp		= 8,		/* vertical back porch */
};

static void panel_get_dimension(struct omap_dss_device *dssdev, u32 *width, u32 *height)
{
	*width = 222;			/* display width in mm  */
	*height = 125;			/* display height in mm */
}

static int panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF;
	dssdev->panel.timings = boe_panel_timings;

	return 0;
}

static struct omap_dss_driver boe_panel = {

	.driver		= {
		.name	= "boe_wsvga_10",
		.owner 	= THIS_MODULE,
	},
	.probe		= panel_probe,
	.remove		= panel_remove,
	.enable		= panel_enable,
	.disable	= panel_disable,
	.suspend	= panel_suspend,
	.resume		= panel_resume,
	.get_dimension	= panel_get_dimension,
	
	.set_timings	= panel_set_timings,
	.get_timings	= panel_get_timings,
	.check_timings	= panel_check_timings,

	.get_recommended_bpp = panel_get_recommended_bpp,

};

static int __init panel_drv_init(void)
{
	printk( "panel boe wsvga 10 init\n");

	omap_dss_register_driver(&boe_panel);
	return 0;
}

static void __exit panel_drv_exit(void)
{
	omap_dss_unregister_driver(&boe_panel);
}

module_init(panel_drv_init);
module_exit(panel_drv_exit);

MODULE_DESCRIPTION("BOE ht101wsb driver");
MODULE_LICENSE("GPL");

