/*
 * include/video/dsscomp.h
 *
 * DSS Composition basic operation support
 *
 * Copyright (C) 2011 Texas Instruments, Inc
 * Author: Lajos Molnar <molnar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LINUX_DSSCOMP_H
#define _LINUX_DSSCOMP_H

#ifdef __KERNEL__
#include <plat/display.h>
#else

/* exporting enumerations from arch/arm/plat-omap/include/plat/display.h */
enum omap_plane {
	OMAP_DSS_GFX	= 0,
	OMAP_DSS_VIDEO1	= 1,
	OMAP_DSS_VIDEO2	= 2,
	OMAP_DSS_VIDEO3	= 3,
	OMAP_DSS_WB		= 4,
};

enum omap_channel {
	OMAP_DSS_CHANNEL_LCD	= 0,
	OMAP_DSS_CHANNEL_DIGIT	= 1,
	OMAP_DSS_CHANNEL_LCD2	= 2,
};

enum omap_color_mode {
	OMAP_DSS_COLOR_CLUT1		= 1 << 0,  /* BITMAP 1 */
	OMAP_DSS_COLOR_CLUT2		= 1 << 1,  /* BITMAP 2 */
	OMAP_DSS_COLOR_CLUT4		= 1 << 2,  /* BITMAP 4 */
	OMAP_DSS_COLOR_CLUT8		= 1 << 3,  /* BITMAP 8 */

	/* also referred to as RGB 12-BPP, 16-bit container  */
	OMAP_DSS_COLOR_RGB12U		= 1 << 4,  /* xRGB12-4444 */
	OMAP_DSS_COLOR_ARGB16		= 1 << 5,  /* ARGB16-4444 */
	OMAP_DSS_COLOR_RGB16		= 1 << 6,  /* RGB16-565 */

	/* also referred to as RGB 24-BPP, 32-bit container */
	OMAP_DSS_COLOR_RGB24U		= 1 << 7,  /* xRGB24-8888 */
	OMAP_DSS_COLOR_RGB24P		= 1 << 8,  /* RGB24-888 */
	OMAP_DSS_COLOR_YUV2		= 1 << 9,  /* YUV2 4:2:2 co-sited */
	OMAP_DSS_COLOR_UYVY		= 1 << 10, /* UYVY 4:2:2 co-sited */
	OMAP_DSS_COLOR_ARGB32		= 1 << 11, /* ARGB32-8888 */
	OMAP_DSS_COLOR_RGBA32		= 1 << 12, /* RGBA32-8888 */

	/* also referred to as RGBx 32 in TRM */
	OMAP_DSS_COLOR_RGBX24		= 1 << 13, /* RGBx24-8888 */
	OMAP_DSS_COLOR_NV12		= 1 << 14, /* NV12 format: YUV 4:2:0 */

	/* also referred to as RGBA12-4444 in TRM */
	OMAP_DSS_COLOR_RGBA16		= 1 << 15, /* RGBA16-4444 */

	OMAP_DSS_COLOR_RGBX12		= 1 << 16, /* RGBx12-4444 */
	OMAP_DSS_COLOR_ARGB16_1555	= 1 << 17, /* ARGB16-1555 */

	/* also referred to as xRGB16-555 in TRM */
	OMAP_DSS_COLOR_XRGB15		= 1 << 18, /* xRGB15-1555 */
};

enum omap_dss_trans_key_type {
	OMAP_DSS_COLOR_KEY_GFX_DST = 0,
	OMAP_DSS_COLOR_KEY_VID_SRC = 1,
};

enum omap_dss_display_state {
	OMAP_DSS_DISPLAY_DISABLED = 0,
	OMAP_DSS_DISPLAY_ACTIVE,
	OMAP_DSS_DISPLAY_SUSPENDED,
	OMAP_DSS_DISPLAY_TRANSITION,
};

/*
 * Stereoscopic Panel types
 * row, column, overunder, sidebyside options
 * are with respect to native scan order
 */
enum s3d_disp_type {
	S3D_DISP_NONE = 0,
	S3D_DISP_FRAME_SEQ,
	S3D_DISP_ROW_IL,
	S3D_DISP_COL_IL,
	S3D_DISP_PIX_IL,
	S3D_DISP_CHECKB,
	S3D_DISP_OVERUNDER,
	S3D_DISP_SIDEBYSIDE,
};

/* Subsampling direction is based on native panel scan order.*/
enum s3d_disp_sub_sampling {
	S3D_DISP_SUB_SAMPLE_NONE = 0,
	S3D_DISP_SUB_SAMPLE_V,
	S3D_DISP_SUB_SAMPLE_H,
};

/*
 * Indicates if display expects left view first followed by right or viceversa
 * For row interlaved displays, defines first row view
 * For column interleaved displays, defines first column view
 * For checkerboard, defines first pixel view
 * For overunder, defines top view
 * For sidebyside, defines west view
 */
enum s3d_disp_order {
	S3D_DISP_ORDER_L = 0,
	S3D_DISP_ORDER_R = 1,
};

/*
 * Indicates current view
 * Used mainly for displays that need to trigger a sync signal
 */
enum s3d_disp_view {
	S3D_DISP_VIEW_L = 0,
	S3D_DISP_VIEW_R,
};

struct s3d_disp_info {
	enum s3d_disp_type type;
	enum s3d_disp_sub_sampling sub_samp;
	enum s3d_disp_order order;
	/*
	 * Gap between left and right views
	 * For over/under units are lines
	 * For sidebyside units are pixels
	 * For other types ignored
	 */
	unsigned int gap;
};

struct omap_video_timings {
	/* Unit: pixels */
	__u16 x_res;
	/* Unit: pixels */
	__u16 y_res;
	/* Unit: KHz */
	__u32 pixel_clock;
	/* Unit: pixel clocks */
	__u16 hsw;	/* Horizontal synchronization pulse width */
	/* Unit: pixel clocks */
	__u16 hfp;	/* Horizontal front porch */
	/* Unit: pixel clocks */
	__u16 hbp;	/* Horizontal back porch */
	/* Unit: line clocks */
	__u16 vsw;	/* Vertical synchronization pulse width */
	/* Unit: line clocks */
	__u16 vfp;	/* Vertical front porch */
	/* Unit: line clocks */
	__u16 vbp;	/* Vertical back porch */
};

#endif

enum omap_dss_ilace_mode {
	OMAP_DSS_ILACE		= (1 << 0),	/* interlaced vs. progressive */
	OMAP_DSS_ILACE_SEQ	= (1 << 1),	/* sequential vs interleaved */
	OMAP_DSS_ILACE_SWAP	= (1 << 2),	/* swap fields, e.g. TB=>BT */

	OMAP_DSS_ILACE_NONE	= 0,
	OMAP_DSS_ILACE_IL_TB	= OMAP_DSS_ILACE,
	OMAP_DSS_ILACE_IL_BT	= OMAP_DSS_ILACE | OMAP_DSS_ILACE_SWAP,
	OMAP_DSS_ILACE_SEQ_TB	= OMAP_DSS_ILACE_IL_TB | OMAP_DSS_ILACE_SEQ,
	OMAP_DSS_ILACE_SEQ_BT	= OMAP_DSS_ILACE_IL_BT | OMAP_DSS_ILACE_SEQ,
};

/* YUV to RGB color conversion info */
struct dss2_color_conv_info {
	__s16 r_y, r_cr, r_cb;
	__s16 g_y, g_cr, g_cb;
	__s16 b_y, b_cr, b_cb;

	/* Y is 16..235, UV is 16..240 if not fullrange.  Otherwise 0..255 */
	__u16 fullrange;	/* bool */
} __attribute__ ((aligned(4)));

/* YUV VC1 range mapping info */
struct dss2_vc1_range_map_info {
	__u8 enable;	/* bool */

	__u8 range_y;	/* 0..7 */
	__u8 range_uv;	/* 0..7 */
} __attribute__ ((aligned(4)));

/* standard rectangle */
struct dss2_rect_t {
	__s32 x;	/* left */
	__s32 y;	/* top */
	__u32 w;	/* width */
	__u32 h;	/* height */
} __attribute__ ((aligned(4)));

/* decimation constraints */
struct dss2_decim {
	__u8 min_x;
	__u8 max_x;	/* 0 is same as 255 */
	__u8 min_y;
	__u8 max_y;	/* 0 is same as 255 */
} __attribute__ ((aligned(4)));

/*
 * A somewhat more user friendly interface to the DSS2.  This is a
 * direct interface to the DSS2 overlay and overlay_manager modules.
 * User-space APIs are provided for HW-specific control of DSS in
 * contrast with V4L2/FB that are more generic, but in this process
 * omit HW-specific features.
 *
 * For now managers are specified by display index as opposed to manager
 * type, so that display0 is always the default display (e.g. HDMI on
 * panda, and LCD blaze.)  For now you would need to query the displays
 * or use sysfs to find a specific display.
 *
 * Userspace operations are as follows:
 *
 * 1) check if DSS supports an overlay configuration, use DSSCOMP_CHECK_OVL
 * ioctl with the manager, overlay, and setup-mode information filled out.
 * All fields should be filled out as it may influence whether DSS can
 * display/render the overlay.
 *
 * If proper address information is not available, it may be possible to
 * use a type-of-address enumeration instead for luma/rgb and chroma (if
 * applicable) frames.
 *
 * Do this for each overlay before attempting to configure DSS.
 *
 * 2) configure DSS pipelines for display/manager using DSSCOMP_SETUP_MANAGER
 * ioctl.  You can delay applying the settings until an dss2_manager_apply()
 * with the same sync_id is called if the APPLY bit of setup mode is not
 * set.  However the CAPTURE/DISPLAY bits of the setup mode settings will
 * determine if at this time a capture will take place (in case of capture
 * only mode).  You may also set up additional pipelines with
 * dss2_overlay_setup() before this.
 *
 * 3) On OMAP4/5 you can use the DSS WB pipeline to copy (and convert) a buffer
 * using DSS.  Use the DSSCOMP_WB_COPY ioctl for this.  This is a blocking
 * call, and it may possibly fail if an ongoing WB capture mode has been
 * schedule (which is outside of the current scope of the DSS2 interface.)
 *
 */

/*
 * DSS2 overlay information.  This structure contains all information
 * needed to set up the overlay for a particular buffer to be displayed
 * at a particular orientation.
 *
 * The following information is deemed to be set globally, so it is not
 * included:
 *   - whether to enable zorder (always enabled)
 *   - whether to replicate/truncate color fields (it is decided per the
 *     whole manager/overlay settings, and is enabled unless overlay is
 *     directed to WB.)
 *
 * There is also no support for CLUT formats
 *
 * Requirements:
 *
 * 1) 0 <= crop.x <= crop.x + crop.w <= width
 * 2) 0 <= crop.y <= crop.y + crop.h <= height
 * 3) win.x <= win.x + win.w and win.w >= 0
 * 4) win.y <= win.y + win.h and win.h >= 0
 *
 * 5) color_mode is supported by overlay
 * 6) requested scaling is supported by overlay and functional clocks
 *
 * Notes:
 *
 * 1) Any portions of X:[pos_x, pos_x + out_width] and
 *    Y:[pos_y, pos_y + out_height] outside of the screen
 *    X:[0, screen.width], Y:[0, screen.height] will be cropped
 *    automatically without changing the scaling ratio.
 *
 * 2) Crop region will be adjusted to the pixel granularity:
 *    (2-by-1) for YUV422, (2-by-2) for YUV420.  This will
 *    not modify the output region.  Crop region is for the
 *    original (unrotated) buffer, so it does not change with
 *    rotation.
 *
 * 3) Rotation will not modify the output region, specifically
 *    its height and width.  Also the coordinate system of the
 *    display is always (0,0) = top left.
 *
 * 4) cconv and vc1 only needs to be filled for YUV color modes.
 *
 * 5) vc1.range_y and vc1.range_uv only needs to be filled if
 *    vc1.enable is true.
 */
struct dss2_ovl_cfg {
	__u16 width;	/* buffer width */
	__u16 height;	/* buffer height */
	__u32 stride;	/* buffer stride */

	enum omap_color_mode color_mode;
	__u8 pre_mult_alpha;	/* bool */
	__u8 global_alpha;	/* 0..255 */
	__u8 rotation;		/* 0..3 (*90 degrees clockwise) */
	__u8 mirror;	/* left-to-right: mirroring is applied after rotation */

	enum omap_dss_ilace_mode ilace;	/* interlace mode */

	struct dss2_rect_t win;		/* output window - on display */
	struct dss2_rect_t crop;	/* crop window - in source buffer */

	struct dss2_decim decim;	/* predecimation limits */

	struct dss2_color_conv_info cconv;
	struct dss2_vc1_range_map_info vc1;

	__u8 ix;		/* ovl index same as sysfs/overlay# */
	__u8 zorder;	/* 0..3 */
	__u8 enabled;	/* bool */
	__u8 zonly;	/* only set zorder and enabled bit */
} __attribute__ ((aligned(4)));

enum omapdss_buffer_type {
	OMAP_DSS_BUFTYPE_SDMA,
	OMAP_DSS_BUFTYPE_TILER_8BIT,
	OMAP_DSS_BUFTYPE_TILER_16BIT,
	OMAP_DSS_BUFTYPE_TILER_32BIT,
	OMAP_DSS_BUFTYPE_TILER_PAGE,
};

struct dss2_ovl_info {
	struct dss2_ovl_cfg cfg;

	union {
		/* user-space interfaces */
		struct {
			void *address;	/* main buffer address */

			/*
			 * For DSSCOMP_CHECK_OVL we allow specifying just the
			 * type of each buffer. This is used if we need to
			 * check whether DSS will be able to display a buffer
			 * if using a particular memory type before spending
			 * time to map/copy the buffer into that type of
			 * memory.  Default value of 0 uses the address to
			 * determine the type.
			 */
			__u16 ba_type;
			__u16 uv_type;
		};

		/* kernel-space interfaces */
		struct {
			__u32 ba;	/* base address */
			__u32 uv;	/* uv address */
		};
	};
};

/*
 * DSS2 manager information.
 *
 * The following information is deemed to be set globally, so it is not
 * included:
 *   - gamma correction
 *   - color phase correction
 *
 *   whether to enable zorder (always enabled)
 *   whether to replicate/truncate color fields (it is decided per the
 *   whole manager/overlay settings, and is enabled unless overlay is
 *   directed to WB.)
 * Notes:
 *
 * 1) trans_key_type and trans_enabled only need to be filled if
 *    trans_enabled is true, and alpha_blending is false.
 */
struct dss2_mgr_info {
	__u32 display_index;	/* display index same as sysfs/display# */

	__u32 default_color;

	enum omap_dss_trans_key_type trans_key_type;
	__u32 trans_key;
	__u8 trans_enabled;	/* bool */

	__u8 interlaced;		/* bool */
	__u8 alpha_blending;	/* bool - overrides trans_enabled */
} __attribute__ ((aligned(4)));

/*
 * ioctl: DSSCOMP_SETUP_MGR, struct dsscomp_setup_mgr_data
 *
 * 1. sets manager of each ovl in composition to the display
 * 2. calls set_dss_ovl_info() for each ovl to set up the
 *    overlay staging structures (this is a wrapper around ovl->set_info())
 * 3. calls set_dss_mgr_info() for mgr to set up the manager
 *    staging structures (this is a wrapper around mgr->set_info())
 * 4. if update is true:
 *      calls manager->apply()
 *      calls driver->update() in a non-blocking fashion
 *      this will program the DSS synchronously
 *
 * Notes:
 *
 * 1) x, y, w, h only needs to be set if update is true.
 *
 * All non-specified pipelines that currently are on the same display
 * will remain the same as on the previous frame.  You may want to
 * disable unused pipelines to avoid surprises.
 *
 * If get_sync_obj is false, it returns 0 on success, <0 error value
 * on failure.
 *
 * If get_sync_obj is true, it returns fd on success, or a negative value
 * on failure.  You can use the fd to wait on (using poll()).  It gets
 * ready when frame has been eclipsed by another frame.
 *
 * Note: frames do not get eclipsed when the display turns off.  Queue a
 * blank frame to eclipse old frames.  Blank frames get eclipsed when
 * programmed into DSS.
 *
 * All overlays to be used on the frame must be listed.  There is no way
 * to add another overlay to a defined frame.
 */
enum dsscomp_setup_mode {
	DSSCOMP_SETUP_MODE_APPLY = (1 << 0),	/* applies changes to cache */
	DSSCOMP_SETUP_MODE_DISPLAY = (1 << 1),	/* calls display update */
	DSSCOMP_SETUP_MODE_CAPTURE = (1 << 2),	/* capture to WB */

	/* just apply changes for next vsync/update */
	DSSCOMP_SETUP_APPLY = DSSCOMP_SETUP_MODE_APPLY,
	/* trigger an update (wait for vsync) */
	DSSCOMP_SETUP_DISPLAY =
			DSSCOMP_SETUP_MODE_APPLY | DSSCOMP_SETUP_MODE_DISPLAY,
	/* capture to WB - WB must be configured */
	DSSCOMP_SETUP_CAPTURE =
			DSSCOMP_SETUP_MODE_APPLY | DSSCOMP_SETUP_MODE_CAPTURE,
	/* display and capture to WB - WB must be configured */
	DSSCOMP_SETUP_DISPLAY_CAPTURE =
			DSSCOMP_SETUP_DISPLAY | DSSCOMP_SETUP_CAPTURE,
};

struct dsscomp_setup_mgr_data {
	__u32 sync_id;		/* synchronization ID */

	struct dss2_rect_t win; /* update region, set w/h to 0 for fullscreen */
	enum dsscomp_setup_mode mode;
	__u16 num_ovls;		/* # of overlays used in the composition */
	__u16 get_sync_obj;	/* ioctl should return a sync object */

	struct dss2_mgr_info mgr;
	struct dss2_ovl_info ovls[0]; /* up to 5 overlays to set up */
};

/*
 * ioctl: DSSCOMP_CHECK_OVL, struct dsscomp_check_ovl_data
 *
 * DISPLAY and/or CAPTURE bits must be filled for the mode field
 * correctly to be able to decide correctly if DSS can properly
 * render the overlay.
 *
 * ovl.ix is ignored.
 *
 * Returns a positive bitmask regarding which overlay of DSS can
 * render the overlay as it is configured for the display/display's
 * manager.  NOTE: that overlays that are assigned to other displays
 * may be returned.  If there is an invalid configuration (negative
 * sizes, etc.), a negative error value is returned.
 *
 * ovl->decim's min values will be modified to the smallest decimation that
 * DSS can use to support the overlay configuration.
 *
 * Assumptions:
 * - zorder will be distinct from other pipelines on that manager
 * - overlay will be enabled and routed to the display specified
 */
struct dsscomp_check_ovl_data {
	enum dsscomp_setup_mode mode;
	struct dss2_mgr_info mgr;
	struct dss2_ovl_info ovl;
};

/*
 * ioctl: DSSCOMP_WB_COPY, struct dsscomp_wb_copy_data
 *
 * Requirements:
 *	wb.ix must be OMAP_DSS_WB.
 *
 * Returns 0 on success (copy is completed), non-0 on failure.
 */
struct dsscomp_wb_copy_data {
	struct dss2_ovl_info ovl, wb;
};

/*
 * ioctl: DSSCOMP_QUERY_DISPLAY, struct dsscomp_display_info
 *
 * Gets informations about the display.  Fill in ix before calling
 * ioctl, and rest of the fields are filled in by ioctl.
 *
 * Returns: 0 on success, non-0 error value on failure.
 */
struct dsscomp_display_info {
	__u32 ix;			/* display index (sysfs/display#) */
	__u32 overlays_available;	/* bitmask of available overlays */
	__u32 overlays_owned;		/* bitmask of owned overlays */
	enum omap_channel channel;
	enum omap_dss_display_state state;
	__u8 enabled;			/* bool: resume-state if suspended */
	struct omap_video_timings timings;
	int mode;
	int code;
	struct s3d_disp_info s3d_info;	/* any S3D specific information */
	struct dss2_mgr_info mgr;	/* manager information */
};

/*
 * ioctl: DSSCOMP_WAIT, struct dsscomp_wait_data
 *
 * Use this ioctl to wait for one of the following events:
 *
 * A) the moment a composition is programmed into DSS
 * B) the moment a composition is first displayed (or captured)
 * C) the moment when a composition is no longer queued or displayed on a
 * display (it is released).  (A composition is assumed to be superceded
 * when another composition has been programmed into DSS, even if that
 * subsequent composition does not update/specify all overlays used by
 * the prior composition; moreover, even if it uses the same buffers.)
 *
 * Non-existing sync IDs are assumed to have been programmed, displayed and
 * released.  (They are assumed to be no-longer existing sync IDs.)
 *
 * Set timeout to desired timeout value in microseconds.  Set timeout
 * to 0 if you want to return a sync object (file descriptor) instead of
 * waiting for the event.  You can then poll on this sync object to
 * wait for the specified event.
 *
 * Returns: >=0 on success, <0 error value on failure (e.g. -ETIME).
 */
enum dsscomp_wait_phase {
	DSSCOMP_WAIT_RELEASED,
	DSSCOMP_WAIT_PROGRAMMED,
	DSSCOMP_WAIT_DISPLAYED,
};

struct dsscomp_wait_data {
	__u32 ix;		/* display index */
	__u32 sync_id;
	__u32 timeout_us;	/* timeout in microseconds */
	enum dsscomp_wait_phase phase;	/* phase to wait for */
};

/*
 * ioctl: DSSCOMP_LAST_RELEASED, struct dsscomp_wait_data
 *
 * Non-blocking sync.
 *
 * Fills in the sync_id of the last released frame on a
 * display specified by ix.
 *
 * Returns 0 on success, non-0 error value on failure.
 */

/*
 * ioctl: DSSCOMP_ISR_REFTIME, struct dsscomp_buffer
 * ioctl: DSSCOMP_ISR_PUT,     struct dsscomp_buffer
 * ioctl: DSSCOMP_ISR_GET,     struct dsscomp_buffer
 *
 * TODO...
 */
struct dsscomp_buffer {
	__u32		num_ovls;	/* number of overlays this should be applied to */
	__u32		ovl_ix[5];	/* ovl indices same as sysfs/overlay# */
	__u32		id;		/* opaque user ID */
	__u32		ts;		/* display time stamp in ms relative to the reference time */
	__u32		ref_ts;		/* current reference time */

	void 		*address;	/* user space buffer address */
	__u32		ba;
	__u32		uv;

	
	enum omap_color_mode color_mode;/* duplicated fields from the dss2_ovl_cfg */
	enum omap_dss_ilace_mode ilace;	/* duplicated fields from the dss2_ovl_cfg */

	__u32		height;		/* ... */
	__u32		stride;		/* ... */

	__u32		ofs_x;		/* left crop offset relative to address */
	__u32		ofs_y;		/* top  crop offset relative to address */
};

struct dsscomp_isr_cfg {
	__u32		num_ovls;
	__u32		ovl_ix[5];	/* ovl indices same as sysfs/overlay# */
	__u32		vsync_src;
};

/* IOCTLS */
#define DSSCOMP_SETUP_MGR	_IOW('O', 128, struct dsscomp_setup_mgr_data)
#define DSSCOMP_CHECK_OVL	_IOWR('O', 129, struct dsscomp_check_ovl_data)
#define DSSCOMP_WB_COPY		_IOW('O', 130, struct dsscomp_wb_copy_data)
#define DSSCOMP_QUERY_DISPLAY	_IOWR('O', 131, struct dsscomp_display_info)
#define DSSCOMP_WAIT		_IOW('O', 132, struct dsscomp_wait_data)
#define DSSCOMP_DROP		_IOW('O', 133, struct dsscomp_wait_data)

#define DSSCOMP_ISR_START	_IOWR('O', 140, struct dsscomp_isr_cfg )
#define DSSCOMP_ISR_STOP	_IO  ('O', 141 )
#define DSSCOMP_ISR_REFTIME	_IOWR('O', 142, struct dsscomp_buffer )
#define DSSCOMP_ISR_PUT	        _IOWR('O', 143, struct dsscomp_buffer )
#define DSSCOMP_ISR_GET		_IOWR('O', 144, struct dsscomp_buffer )
#define DSSCOMP_ISR_FLUSH	_IO  ('O', 145  )

#endif
