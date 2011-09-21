/*
 * panel-cpt-80xa01.c
 *
 *  Created on: Dec 23, 2010
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>

#include <plat/display.h>

static void panel_remove(struct omap_dss_device *dssdev)
{
}

static int panel_display_on(struct omap_dss_device *dssdev)
{
	int ret = 0;

	printk(KERN_INFO "A8 disp enable\n");
	ret = omapdss_dpi_display_enable(dssdev);

	if (dssdev->platform_enable && (ret == 0))
		ret = dssdev->platform_enable(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	
	return ret;
}

static void panel_display_off(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO "A8 disp disable\n");

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

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

static int panel_get_recommended_bpp(struct omap_dss_device *dssdev)
{
	return 24;
}

// timings references  for CPT CLAA080 1024x768 for A8
#define CPT_LCD_PIXCLOCK	65000		/* 65MHz */

static struct omap_video_timings cpt_panel_timings = {
		.x_res		= 1024,
		.y_res		= 768,

		.pixel_clock	= CPT_LCD_PIXCLOCK,	/* Mhz */
// 60hz
		.hsw		= 2,		/* horizontal sync pulse width */
		.hfp		= 218,		/* horizontal front porch */
		.hbp		= 100,		/* horizontal back porch */
		.vsw		= 4,		/* vertical sync pulse width */
		.vfp		= 26,		/* vertical front porch */
		.vbp		= 8,		/* vertical back porch */
};

static void panel_get_dimension(struct omap_dss_device *dssdev, u32 *width, u32 *height)
{
   *width = 162;            /* display width in mm  */
   *height = 122;           /* display height in mm */
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


static int panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF;

	dssdev->panel.timings = cpt_panel_timings;

	return 0;
}

static struct omap_dss_driver cpt_panel = {

	.driver		= {
		.name	= "cpt_xga_8",
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
	printk( KERN_INFO "panel cpt xga 8 init\n");

	omap_dss_register_driver(&cpt_panel);
	return 0;
}

static void __exit panel_drv_exit(void)
{
	omap_dss_unregister_driver(&cpt_panel);
}

module_init(panel_drv_init);
module_exit(panel_drv_exit);

MODULE_DESCRIPTION("CPT claa 80x01 driver");
MODULE_LICENSE("GPL");

