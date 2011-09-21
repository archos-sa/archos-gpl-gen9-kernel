/*
 * panel-auo-a101ewe01.c
 *
 *  Created on: Feb 13, 2011
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

extern void dispc_enable_sidle(void);
extern void dispc_disable_sidle(void);

static void panel_remove(struct omap_dss_device *disp)
{
}

static int panel_display_on(struct omap_dss_device *dssdev)
{
	int ret = 0;

	dev_dbg(&dssdev->dev, "A10 disp enable\n");
	ret = omapdss_dpi_display_enable(dssdev);
	if (ret == 0) {
		if (dssdev->platform_enable)
			ret = dssdev->platform_enable(dssdev);
		if (ret == 0) {
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
			/* 
			 * FIXME: otherwise we see GFX_UNDERFLOW errors
			 */
			dispc_disable_sidle();
		}
	}
	return ret;
}

static void panel_display_off(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "A10 disp disable\n");

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* 
	 * FIXME: remove when root cause of GFX_UNDERFLOW is fixed
	 */
	dispc_enable_sidle();

	omapdss_dpi_display_disable(dssdev);

}

static int panel_enable(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;
	
	return panel_display_on(dssdev);

}

static void panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		panel_display_off(dssdev);
	
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int panel_suspend(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	panel_display_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int panel_resume(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	panel_display_on(dssdev);

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
#define AUO_LCD_PIXCLOCK	68930		/* 68,93MHz */
#define AUO_LCD_PIXCLOCK_PRCM	88000		/* request 88MHz to set 84M in prcm mode*/

static struct omap_video_timings auo_panel_timings = {
	.x_res		= 1280,
	.y_res		= 800,

	.pixel_clock	= AUO_LCD_PIXCLOCK,	/* Mhz */
// 60hz
	.hsw		= 2,		/* horizontal sync pulse width */
	.hfp		= 26,		/* horizontal front porch */
	.hbp		= 100,		/* horizontal back porch */
	.vsw		= 4,		/* vertical sync pulse width */
	.vfp		= 8,		/* vertical front porch */
	.vbp		= 4,		/* vertical back porch */
};

static void panel_get_dimension(struct omap_dss_device *dssdev, u32 *width, u32 *height)
{
	*width = 217;            /* display width in mm  */
	*height = 136;           /* display height in mm */
}

static int panel_probe(struct omap_dss_device *dssdev)
{
	// data on rising edge of pixclk
	dssdev->panel.config = OMAP_DSS_LCD_TFT /*| OMAP_DSS_LCD_IPC*/ | OMAP_DSS_LCD_IVS 
				| OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF;

	if (dssdev->phy.dpi.force_dispc_clk == 1)
		auo_panel_timings.pixel_clock = AUO_LCD_PIXCLOCK_PRCM;

	dssdev->panel.timings = auo_panel_timings;
	return 0;
}

static struct omap_dss_driver auo_panel = {

	.driver		= {
		.name	= "auo_wxga_10",
		.owner 	= THIS_MODULE,
	},
	.probe		= panel_probe,
	.remove		= panel_remove,
	.enable		= panel_enable,
	.disable	= panel_disable,
	.suspend	= panel_suspend,
	.resume		= panel_resume,
	.get_recommended_bpp = panel_get_recommended_bpp,
	.get_dimension  = panel_get_dimension,
	.set_timings	= panel_set_timings,
	.get_timings	= panel_get_timings,
	.check_timings	= panel_check_timings,

};

static int __init panel_drv_init(void)
{
	printk( "panel auo wxga10 init\n");

	omap_dss_register_driver(&auo_panel);
	return 0;
}

static void __exit panel_drv_exit(void)
{
	omap_dss_unregister_driver(&auo_panel);
}

module_init(panel_drv_init);
module_exit(panel_drv_exit);

MODULE_DESCRIPTION("AUO a101ewe driver");
MODULE_LICENSE("GPL");

