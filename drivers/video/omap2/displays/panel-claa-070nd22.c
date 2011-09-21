/*
 * panel-claa-070nd22.c
 *
 *  Created on: Feb 18, 2011
 *      Author: Yvon Robic <robic@archos.com>
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
#include <linux/delay.h>

static void panel_remove(struct omap_dss_device *disp)
{
}

static int panel_enable(struct omap_dss_device *dssdev)
{
	int ret = 0;

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	ret = omapdss_dpi_display_enable(dssdev);
	if (ret == 0) {
		if (dssdev->platform_enable)
			ret = dssdev->platform_enable(dssdev);
		if (ret == 0)
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	}
	return ret;
}

static void panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	omapdss_dpi_display_disable(dssdev);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int panel_suspend(struct omap_dss_device *disp)
{
	panel_disable(disp);

	return 0;
}

static int panel_resume(struct omap_dss_device *disp)
{
	panel_enable(disp);

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
#define CLAA_LCD_PIXCLOCK	51200		/* 51,2MHz */

static struct omap_video_timings claa_panel_timings = {
		.x_res		= 1024,
		.y_res		= 600,

		.pixel_clock	= CLAA_LCD_PIXCLOCK,	/* Mhz */
// 60hz
		.hsw		= 2,		/* horizontal sync pulse width */
		.hfp		= 118,		/* horizontal front porch */
		.hbp		= 200,		/* horizontal back porch */
		.vsw		= 4,		/* vertical sync pulse width */
		.vfp		= 26,		/* vertical front porch */
		.vbp		= 5,		/* vertical back porch */
};

static void panel_get_dimension(struct omap_dss_device *dssdev, u32 *width, u32 *height)
{
	*width = 166;			/* display width in mm  */
	*height = 105;			/* display height in mm */
}

static int panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF;
	dssdev->panel.timings = claa_panel_timings;

	return 0;
}

static struct omap_dss_driver claa_panel = {

	.driver		= {
		.name	= "claa_wsvga_7",
		.owner 	= THIS_MODULE,
	},
	.probe		= panel_probe,
	.remove		= panel_remove,
	.enable		= panel_enable,
	.disable	= panel_disable,
	.suspend	= panel_suspend,
	.resume		= panel_resume,
	.get_recommended_bpp = panel_get_recommended_bpp,
	.get_dimension	= panel_get_dimension,
	.set_timings	= panel_set_timings,
	.get_timings	= panel_get_timings,
	.check_timings	= panel_check_timings,

};

static int __init panel_drv_init(void)
{
	printk( "panel claa wsvga 7 init\n");

	omap_dss_register_driver(&claa_panel);
	return 0;
}

static void __exit panel_drv_exit(void)
{
	omap_dss_unregister_driver(&claa_panel);
}

module_init(panel_drv_init);
module_exit(panel_drv_exit);
