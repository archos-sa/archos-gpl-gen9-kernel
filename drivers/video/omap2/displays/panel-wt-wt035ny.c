/*
 * panel-wt-wt035ny.c
 *
 *  Created on: Sept 24, 2010
 *      Author: Niklas Schroeter <schroeter@archos.com>
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

/* ILI9325 control registers */
#define ILI9325_DRV_CODE			0x00  /* Driver code read, RO.*/
#define ILI9325_DRV_OUTPUT_CTRL_1		0x01  /* Driver output control 1, W.*/
#define ILI9325_LCD_DRV_CTRL			0x02  /* LCD driving control, W.*/
#define ILI9325_ENTRY_MOD			0x03  /* Entry mode, W.*/
#define ILI9325_RESIZE_CTRL			0x04  /* Resize Control, W.*/

#define ILI9325_DIS_CTRL_1			0x07  /* Display Control 1, W.*/
#define ILI9325_DIS_CTRL_2			0x08  /* Display Control 2, W.*/
#define ILI9325_DIS_CTRL_3			0x09  /* Display Control 3, W.*/
#define ILI9325_DIS_CTRL_4			0x0A  /* Display Control 4, W.*/

#define ILI9325_RGB_CTRL_1			0x0C  /* RGB display interface control 1, W.*/
#define ILI9325_FRAME_MARKER_POS		0x0D  /* Frame Marker Position, W.*/
#define ILI9325_RGB_CTRL_2			0x0F  /* RGB display interface control 2, W.*/

#define ILI9325_POW_CTRL_1			0x10  /* Power control 1, W.*/
#define ILI9325_POW_CTRL_2			0x11  /* Power control 2, W.*/
#define ILI9325_POW_CTRL_3			0x12  /* Power control 3, W.*/
#define ILI9325_POW_CTRL_4			0x13  /* Power control 4, W.*/

#define ILI9325_GRAM_HADDR			0x20  /* Horizontal GRAM address set, W.*/
#define ILI9325_GRAM_VADDR			0x21  /* Vertical GRAM address set, W.*/
#define ILI9325_GRAM_DATA			0x22  /* Write Data to GRAM, W.*/

#define ILI9325_POW_CTRL_7			0x29  /* Power control 7, W.*/

#define ILI9325_FRM_RATE_COLOR			0x2B  /* Frame Rate and Color Control, W. */

#define ILI9325_GAMMA_CTRL_1			0x30  /* Gamma Control 1, W. */
#define ILI9325_GAMMA_CTRL_2			0x31  /* Gamma Control 2, W. */
#define ILI9325_GAMMA_CTRL_3			0x32  /* Gamma Control 3, W. */
#define ILI9325_GAMMA_CTRL_4			0x35  /* Gamma Control 4, W. */
#define ILI9325_GAMMA_CTRL_5			0x36  /* Gamma Control 5, W. */
#define ILI9325_GAMMA_CTRL_6			0x37  /* Gamma Control 6, W. */
#define ILI9325_GAMMA_CTRL_7			0x38  /* Gamma Control 7, W. */
#define ILI9325_GAMMA_CTRL_8			0x39  /* Gamma Control 8, W. */
#define ILI9325_GAMMA_CTRL_9			0x3C  /* Gamma Control 9, W. */
#define ILI9325_GAMMA_CTRL_10			0x3D  /* Gamma Control l0, W. */

#define ILI9325_HOR_ADDR_START			0x50  /* Horizontal Address Start, W. */
#define ILI9325_HOR_ADDR_END			0x51  /* Horizontal Address End Position, W. */
#define ILI9325_VET_ADDR_START			0x52  /* Vertical Address Start, W. */
#define ILI9325_VET_ADDR_END			0x53  /* Vertical Address Start, W. */

#define ILI9325_DRV_OUTPUT_CTRL_2		0x60  /* Driver output control 2, W.*/

#define ILI9325_BASE_IMG_CTRL			0x61  /* Base Image Display Control, W.*/
#define ILI9325_VSCROLL_CTRL			0x6A  /* Vertical Scroll Control, W.*/

#define ILI9325_PAR_IMG1_POS			0x80  /* Partial Image 1 Display Position, W.*/
#define ILI9325_PAR_IMG1_START			0x81  /* Partial Image 1 Area (Start Line), W.*/
#define ILI9325_PAR_IMG1_END			0x82  /* Partial Image 1 Area (End Line), W.*/
#define ILI9325_PAR_IMG2_POS			0x83  /* Partial Image 2 Display Position, W.*/
#define ILI9325_PAR_IMG2_START			0x84  /* Partial Image 2 Area (Start Line), W.*/
#define ILI9325_PAR_IMG2_END			0x85  /* Partial Image 2 Area (End Line), W.*/

#define ILI9325_PAN_CTRL_1			0x90  /* Panel Interface Control 1, W.*/
#define ILI9325_PAN_CTRL_2			0x92  /* Panel Interface Control 2, W.*/
#define ILI9325_PAN_CTRL_3			0x93  /* Panel Interface Control 3, W.*/
#define ILI9325_PAN_CTRL_4			0x95  /* Panel Interface Control 4, W.*/


#define ILI9325_OTP_PROG_CTRL			0xA1  /* OTP VCM Programming Control, W.*/
#define ILI9325_OTP_STATUS			0xA2  /* OTP VCM Status and Enable, W.*/
#define ILI9325_OTP_ID_KEY			0xA5  /* OTP Programming ID Key, W.*/

/* Registers not depicted on the datasheet :-( */
#define ILI9325_TIMING_CTRL_1			0xE3  /* Timing control. */
#define ILI9325_TIMING_CTRL_2			0xE7  /* Timing control. */
#define ILI9325_TIMING_CTRL_3			0xEF  /* Timing control. */

#define ILI9325_PAN_CTRL_5			0x97  /* Panel Interface Control 5, W.*/
#define ILI9325_PAN_CTRL_6			0x98  /* Panel Interface Control 6, W.*/


// disp ctrl1 bits
#define D0 1<<0
#define D1 1<<1
#define CL 1<<2
#define DTE 1<<4
#define GON 1<<5
#define BASEE 1<<8
#define PTDE0 1<<12
#define PTDE1 1<<13

// pw ctrl1 bits
#define STB 1<<0
#define SLP 1<<1
#define DSTB 1<<2
#define AP0 1<<4
#define AP1 1<<5
#define AP2 1<<6
#define APE 1<<7
#define BT0 1<<8
#define BT1 1<<9
#define BT2 1<<10
#define SAP 1<<12


// pw ctrl2 bits
#define VC0 1<<0
#define VC1 1<<1
#define VC2 1<<2
#define DC00 1<<4
#define DC01 1<<5
#define DC02 1<<6
#define DC10 1<<8
#define DC11 1<<9
#define DC12 1<<10

// pw ctrl3 bits
#define VRH0 1<<0
#define VRH1 1<<1
#define VRH2 1<<2
#define VRH3 1<<3
#define PON 1<<4
#define VCIRE 1<<7

struct wt035ny_data {
	struct mutex lock;
	struct omap_dss_device *dssdev;
};

static inline void WriteCommand_Addr(u16 index)
{
        omap_rfbi_write_command(&index, 2);
}

static inline void WriteCommand_Data(u16 val)
{
        omap_rfbi_write_data(&val, 2);
}

/* Adjust the Gamma Curve  */
static void ili9481_gamma_adjust(void)
{
	WriteCommand_Addr(0xC8);	// Gamma Setting
	WriteCommand_Data(0x00);	// KP1[2:0],KP0[2:0]
	WriteCommand_Data(0x45);	// KP3[2:0],KP2[2:0]
	WriteCommand_Data(0x22);	// KP5[2:0],KP4[2:0]
	WriteCommand_Data(0x11);	// RP1[2:0],RP0[2:0]
	WriteCommand_Data(0x08);	// VRP0[3:0]
	WriteCommand_Data(0x00);	// VRP1[4:0]
	WriteCommand_Data(0x55);	// KN1[2:0],KN0[2:0]
	WriteCommand_Data(0x13);	// KN3[2:0],KN2[2:0]
	WriteCommand_Data(0x77);	// KN5[2:0],KN4[2:0]
	WriteCommand_Data(0x11);	// RN1[2:0],RN0[2:0]
	WriteCommand_Data(0x08);	// VRN0[3:0]
	WriteCommand_Data(0x00);	// VRN1[4:0]
}  

/* Start Initial Sequence */
static void ili9481_chip_init(struct omap_dss_device *dev)
{  
	WriteCommand_Addr(0x11);	// exit sleep mode
	msleep(20);
	WriteCommand_Addr(0xD0);	// power settings
	WriteCommand_Data(0x07);	// VC[2:0]
	WriteCommand_Data(0x41);	// PON,BT[2:0]
	WriteCommand_Data(0x05);	//11 VCIRE,VRH[3:0]

	if ( dev->ctrl.pixel_size == 16) {
		WriteCommand_Addr(0xD1);	// VCOM control
		WriteCommand_Data(0x00);	// SELVCM
		WriteCommand_Data(0x28);	// VCM[5:0]
		WriteCommand_Data(0x16);	// VDV[4:0]
	} else {
		WriteCommand_Addr(0xD1);	// VCOM control
		WriteCommand_Data(0x00);	// SELVCM
		WriteCommand_Data(0x24);	// VCM[5:0]
		WriteCommand_Data(0x11);	// VDV[4:0]
	}

	WriteCommand_Addr(0xD2);	// power for normal mode
	WriteCommand_Data(0x01);	// AP0[2:0]
	WriteCommand_Data(0x11);	// DC10[2:0],DC00[2:0]

	WriteCommand_Addr(0xC0);	// Panel settings
	WriteCommand_Data(0x10);	// REV & SM & GS
	WriteCommand_Data(0x3B);	// NL[5:0] 480 lignes
	WriteCommand_Data(0x00);	// SCN[6:0]
	WriteCommand_Data(0x00);	// SCN[6:0]
	WriteCommand_Data(0x02);	// NDL , PTS[2:0]
	WriteCommand_Data(0x11);	// PTG , ISC[3:0]

	if ( dev->ctrl.pixel_size == 16) {
		WriteCommand_Addr(0xC5);	// Frame rate
		WriteCommand_Data(0x04);	// 56 Hz
	} else {
		WriteCommand_Addr(0xC5);	// Frame rate
		WriteCommand_Data(0x02);	// 85 Hz
	}
	WriteCommand_Addr(0xC6);	// interface control
	WriteCommand_Data(0x13);	

	ili9481_gamma_adjust();

	WriteCommand_Addr(0xE4);
	WriteCommand_Data(0xA0);

	WriteCommand_Addr(0xF0);
	WriteCommand_Data(0x01);

	WriteCommand_Addr(0xF3);
	WriteCommand_Data(0x20);	//40
	WriteCommand_Data(0x0F);	//pre buffer

	WriteCommand_Addr(0xF7);
	WriteCommand_Data(0x80);

	WriteCommand_Addr(0x36);	// Set_address_mode
	WriteCommand_Data(0x0A);	// Horizontal Flip, Pixels in BGR

	msleep(120);

	if ( dev->ctrl.pixel_size == 16) {
		WriteCommand_Addr(0x3A);	// data format
		WriteCommand_Data(0x55);	// 16 bit
	} else {
		WriteCommand_Addr(0x3A);	// data format
		WriteCommand_Data(0x66);	// 18 bit
	}

	WriteCommand_Addr(0x29);	// set display on

	WriteCommand_Addr(0x2A);	// Page Address Set
	WriteCommand_Data(0x00);	// from 
	WriteCommand_Data(0x18);
	WriteCommand_Data(0x01);	// to
	WriteCommand_Data(0x27);

	WriteCommand_Addr(0x2B);	//Column Address Set
	WriteCommand_Data(0x00);	// from 0
	WriteCommand_Data(0x00);
	WriteCommand_Data(0x01);	// to 479
	WriteCommand_Data(0xDF);

	WriteCommand_Addr(0xb3);	// memory access
	WriteCommand_Data(0x00);	// 
	WriteCommand_Data(0x00);
	WriteCommand_Data(0x00);	//
	//WriteCommand_Data(0x01);	// dfm = 1
	WriteCommand_Data(0x00);	// dfm = 0

	WriteCommand_Addr(0x2c);	// start transfert image

}

static int wt035ny_panel_start(struct omap_dss_device *dssdev)
{
	int r = 0;
	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			return r;
	}

	r = omapdss_rfbi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DPI\n");
		return r;
	}

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	ili9481_chip_init(dssdev);

	return 0;
}

static int wt035ny_panel_enable(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO "wt035ny panel init is called ");
	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return wt035ny_panel_start(dssdev);
}
static void wt035ny_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static int wt035ny_panel_probe(struct omap_dss_device *dssdev)
{
	int r;
	struct wt035ny_data *wtd;
	
	wtd = kzalloc(sizeof(*wtd), GFP_KERNEL);
	if (!wtd) {
		r = -ENOMEM;
		goto err;
	}
	
	dev_set_drvdata(&dssdev->dev, wtd);
	
	mutex_init(&wtd->lock);

	wtd->dssdev = dssdev;

	dssdev->panel.config &= ~((OMAP_DSS_LCD_IPC) | (OMAP_DSS_LCD_IEO));
	dssdev->panel.config =  (OMAP_DSS_LCD_TFT) | (OMAP_DSS_LCD_ONOFF) |
						(OMAP_DSS_LCD_IHS)  |
						(OMAP_DSS_LCD_IVS) ;
	dssdev->panel.acb = 0x0;
//dssdev->panel.timings = wt035ny_ls_timings;

	return 0;
err:
	return r;
}

static void wt035ny_panel_remove(struct omap_dss_device *dssdev)
{
	struct wt035ny_data *wtd = dev_get_drvdata(&dssdev->dev);

	dev_set_drvdata(&dssdev->dev, NULL);
	kfree(wtd);
	return;
}

static void wt035ny_panel_stop(struct omap_dss_device *dssdev)
{
	omapdss_rfbi_display_disable(dssdev);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void wt035ny_panel_disable(struct omap_dss_device *dssdev)
{
	/* Turn of DLP Power */
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		wt035ny_panel_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int wt035ny_panel_suspend(struct omap_dss_device *dssdev)
{
	/* Turn of DLP Power */
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	wt035ny_panel_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int wt035ny_panel_resume(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO "wt wt035ny resume is called ");
	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	return wt035ny_panel_start(dssdev);
}

static void wt035ny_framedone_cb(void *data)
{
	struct omap_dss_device *dssdev = data;

	dev_dbg(&dssdev->dev, "framedone\n");
}

static int wt035ny_update_locked(struct omap_dss_device *dssdev,
				    u16 x, u16 y, u16 w, u16 h)
{
	struct wt035ny_data *wtd = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	WriteCommand_Addr(0x2c);	// start transfert image

	r = omap_rfbi_prepare_update(dssdev, &x, &y, &w, &h);
	if (r)
		goto err;

//printk("wt035ny_update_locked: %d, %d, %d, %d\n", x, y, w, h);

	r = omap_rfbi_update(dssdev, x, y, w, h, wt035ny_framedone_cb, dssdev);
	if (r)
		goto err;
	
	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&wtd->lock);
	return 0;
err:
	mutex_unlock(&wtd->lock);
	return r;
}

static int wt035ny_update(struct omap_dss_device *dssdev,
				    u16 x, u16 y, u16 w, u16 h)
{
	struct wt035ny_data *wtd = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "update %d, %d, %d x %d\n", x, y, w, h);

	mutex_lock(&wtd->lock);

	/* mark while waiting on bus so delayed update will not call update */
	dssdev->sched_update.waiting = true;
	dssdev->sched_update.waiting = false;

	return wt035ny_update_locked(dssdev, x, y, w, h);
}

static int wt035ny_sched_update(struct omap_dss_device *dssdev,
					u16 x, u16 y, u16 w, u16 h)
{
	struct wt035ny_data *wtd = dev_get_drvdata(&dssdev->dev);
	int r;
//printk("wt035ny_sched_update\n");

	mutex_lock(&wtd->lock);

#if 0
	r = omap_dsi_sched_update_lock(dssdev, x, y, w, h);

	if (!r)
		/* start the update now */
		return wt035ny_update_locked(dssdev, x, y, w, h);

	if (r == -EBUSY)
		r = 0;
#endif
	mutex_unlock(&wtd->lock);
	return r;
}
static int panel_enable_te(struct omap_dss_device *display, bool enable)
{
pr_debug("panel_enable_te %d \n", enable);
	if ( enable ) {
		WriteCommand_Addr(0x35);	// set tear on
		WriteCommand_Data(0x00);	// 00 mode 1 vsync only, 01 mode 2 vsync+hsync

		WriteCommand_Addr(0x44);	// te output line offset
		WriteCommand_Data(0x00);	// hsb 
		WriteCommand_Data(0x00);	// lsb 00 by default
	} else
		WriteCommand_Addr(0x34);	// set tear off

	return 0;
}

static int wt035ny_sync(struct omap_dss_device *dssdev)
{
	struct wt035ny_data *wtd = dev_get_drvdata(&dssdev->dev);

printk("wt035ny_sync\n");

	dev_dbg(&dssdev->dev, "sync\n");

	mutex_lock(&wtd->lock);
	mutex_unlock(&wtd->lock);

	dev_dbg(&dssdev->dev, "sync done\n");

	return 0;
}

static int wt035ny_set_update_mode(struct omap_dss_device *dssdev,
		enum omap_dss_update_mode mode)
{
	if (mode != OMAP_DSS_UPDATE_MANUAL)
			return -EINVAL;
	return 0;
}

static enum omap_dss_update_mode wt035ny_get_update_mode(
		struct omap_dss_device *dssdev)
{
	return OMAP_DSS_UPDATE_MANUAL;
}

static void wt035ny_setup_update(struct omap_dss_device *display,
				    u16 x, u16 y, u16 w, u16 h)
{
	WriteCommand_Addr(0x2c);	// start transfert image
}

static struct omap_dss_driver wt035ny_driver = {
	.probe		= wt035ny_panel_probe,
	.remove		= wt035ny_panel_remove,
	.enable		= wt035ny_panel_enable,
	.disable	= wt035ny_panel_disable,
	.get_resolution	= wt035ny_get_resolution,
	.suspend	= wt035ny_panel_suspend,
	.resume		= wt035ny_panel_resume,

	.set_update_mode = wt035ny_set_update_mode,
	.get_update_mode = wt035ny_get_update_mode,
	.enable_te	= panel_enable_te,
	.update		= wt035ny_update,
	.sched_update	= wt035ny_sched_update,
	.sync		= wt035ny_sync,

	.driver         = {
		.name   = "wt_lcd_35",
	.owner  = THIS_MODULE,
	},
};

static int __init panel_drv_init(void)
{
 	omap_dss_register_driver(&wt035ny_driver);
	return 0;
}

static void __exit panel_drv_exit(void)
{
	omap_dss_unregister_driver(&wt035ny_driver);
}

module_init(panel_drv_init);
module_exit(panel_drv_exit);

MODULE_DESCRIPTION("wt wt035ny driver");
MODULE_LICENSE("GPL");



