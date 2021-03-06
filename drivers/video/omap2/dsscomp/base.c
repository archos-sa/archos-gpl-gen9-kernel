/*
 * linux/drivers/video/omap2/dsscomp/base.c
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

#include <linux/kernel.h>

#include <linux/notifier.h>
#include <mach/tiler.h>

#include <plat/display.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>

#include <media/cma.h>
#include "dsscomp.h"

int debug;
module_param(debug, int, 0644);

/* color formats supported - bitfield info is used for truncation logic */
static const struct color_info {
	int a_ix, a_bt;	/* bitfields */
	int r_ix, r_bt;
	int g_ix, g_bt;
	int b_ix, b_bt;
	int x_bt;
	enum omap_color_mode mode;
	const char *name;
} fmts[2][16] = { {
	{ 0,  0, 0,  0, 0,  0, 0, 0, 1, OMAP_DSS_COLOR_CLUT1, "BITMAP1" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 2, OMAP_DSS_COLOR_CLUT2, "BITMAP2" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 4, OMAP_DSS_COLOR_CLUT4, "BITMAP4" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 8, OMAP_DSS_COLOR_CLUT8, "BITMAP8" },
	{ 0,  0, 8,  4, 4,  4, 0, 4, 4, OMAP_DSS_COLOR_RGB12U, "xRGB12-4444" },
	{ 12, 4, 8,  4, 4,  4, 0, 4, 0, OMAP_DSS_COLOR_ARGB16, "ARGB16-4444" },
	{ 0,  0, 11, 5, 5,  6, 0, 5, 0, OMAP_DSS_COLOR_RGB16, "RGB16-565" },
	{ 15, 1, 10, 5, 5,  5, 0, 5, 0, OMAP_DSS_COLOR_ARGB16_1555,
								"ARGB16-1555" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 8, OMAP_DSS_COLOR_RGB24U, "xRGB24-8888" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_RGB24P, "RGB24-888" },
	{ 0,  0, 12, 4, 8,  4, 4, 4, 4, OMAP_DSS_COLOR_RGBX12, "RGBx12-4444" },
	{ 0,  4, 12, 4, 8,  4, 4, 4, 0, OMAP_DSS_COLOR_RGBA16, "RGBA16-4444" },
	{ 24, 8, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_ARGB32, "ARGB32-8888" },
	{ 0,  8, 24, 8, 16, 8, 8, 8, 0, OMAP_DSS_COLOR_RGBA32, "RGBA32-8888" },
	{ 0,  0, 24, 8, 16, 8, 8, 8, 8, OMAP_DSS_COLOR_RGBX24, "RGBx24-8888" },
	{ 0,  0, 10, 5, 5,  5, 0, 5, 1, OMAP_DSS_COLOR_XRGB15, "xRGB15-1555" },
}, {
	{ 0,  0, 0,  0, 0,  0, 0, 0, 12, OMAP_DSS_COLOR_NV12, "NV12" },
	{ 0,  0, 12, 4, 8,  4, 4, 4, 4, OMAP_DSS_COLOR_RGBX12, "RGBx12-4444" },
	{ 0,  4, 12, 4, 8,  4, 4, 4, 0, OMAP_DSS_COLOR_RGBA16, "RGBA16-4444" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 0, 0, "invalid" },
	{ 0,  0, 8,  4, 4,  4, 0, 4, 4, OMAP_DSS_COLOR_RGB12U, "xRGB12-4444" },
	{ 12, 4, 8,  4, 4,  4, 0, 4, 0, OMAP_DSS_COLOR_ARGB16, "ARGB16-4444" },
	{ 0,  0, 11, 5, 5,  6, 0, 5, 0, OMAP_DSS_COLOR_RGB16, "RGB16-565" },
	{ 15, 1, 10, 5, 5,  5, 0, 5, 0, OMAP_DSS_COLOR_ARGB16_1555,
								"ARGB16-1555" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 8, OMAP_DSS_COLOR_RGB24U, "xRGB24-8888" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_RGB24P, "RGB24-888" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 16, OMAP_DSS_COLOR_YUV2, "YUYV" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 16, OMAP_DSS_COLOR_UYVY, "UYVY" },
	{ 24, 8, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_ARGB32, "ARGB32-8888" },
	{ 0,  8, 24, 8, 16, 8, 8, 8, 0, OMAP_DSS_COLOR_RGBA32, "RGBA32-8888" },
	{ 0,  0, 24, 8, 16, 8, 8, 8, 8, OMAP_DSS_COLOR_RGBX24, "RGBx24-8888" },
	{ 0,  0, 10, 5, 5,  5, 0, 5, 1, OMAP_DSS_COLOR_XRGB15, "xRGB15-1555" },
} };

static const struct color_info *get_color_info(enum omap_color_mode mode)
{
	int i;
	for (i = 0; i < sizeof(fmts) / sizeof(fmts[0][0]); i++)
		if (fmts[0][i].mode == mode)
			return fmts[0] + i;
	return NULL;
}

static int color_mode_to_bpp(enum omap_color_mode color_mode)
{
	const struct color_info *ci = get_color_info(color_mode);
	BUG_ON(!ci);

	return ci->a_bt + ci->r_bt + ci->g_bt + ci->b_bt + ci->x_bt;
}

union rect {
	struct {
		s32 x;
		s32 y;
		s32 w;
		s32 h;
	};
	struct {
		s32 xy[2];
		s32 wh[2];
	};
	struct dss2_rect_t r;
};

int crop_to_rect(union rect *crop, union rect *win, union rect *vis,
						int rotation, int mirror)
{
	int c, swap = rotation & 1;

	/* align crop window with display coordinates */
	if (swap)
		crop->y -= (crop->h = -crop->h);
	if (rotation & 2)
		crop->xy[!swap] -= (crop->wh[!swap] = -crop->wh[!swap]);
	if ((!mirror) ^ !(rotation & 2))
		crop->xy[swap] -= (crop->wh[swap] = -crop->wh[swap]);

	for (c = 0; c < 2; c++) {
		/* see if complete buffer is outside the vis or it is
		   fully cropped or scaled to 0 */
		if (win->wh[c] <= 0 || vis->wh[c] <= 0 ||
		    win->xy[c] + win->wh[c] <= vis->xy[c] ||
		    win->xy[c] >= vis->xy[c] + vis->wh[c] ||
		    !crop->wh[c ^ swap])
			return -ENOENT;

		/* crop left/top */
		if (win->xy[c] < vis->xy[c]) {
			/* correction term */
			int a = (vis->xy[c] - win->xy[c]) *
						crop->wh[c ^ swap] / win->wh[c];
			crop->xy[c ^ swap] += a;
			crop->wh[c ^ swap] -= a;
			win->wh[c] -= vis->xy[c] - win->xy[c];
			win->xy[c] = vis->xy[c];
		}
		/* crop right/bottom */
		if (win->xy[c] + win->wh[c] > vis->xy[c] + vis->wh[c]) {
			crop->wh[c ^ swap] = crop->wh[c ^ swap] *
				(vis->xy[c] + vis->wh[c] - win->xy[c]) /
								win->wh[c];
			win->wh[c] = vis->xy[c] + vis->wh[c] - win->xy[c];
		}

		if (!crop->wh[c ^ swap] || !win->wh[c])
			return -ENOENT;
	}

	/* realign crop window to buffer coordinates */
	if (rotation & 2)
		crop->xy[!swap] -= (crop->wh[!swap] = -crop->wh[!swap]);
	if ((!mirror) ^ !(rotation & 2))
		crop->xy[swap] -= (crop->wh[swap] = -crop->wh[swap]);
	if (swap)
		crop->y -= (crop->h = -crop->h);
	return 0;
}

#define ISR_STATE_UNUSED 0
#define ISR_STATE_CLOSED 1
#define ISR_STATE_OPENED 2

struct isr_ctx {
	atomic_t state;
	struct {
		struct dss2_ovl_cfg cfg;
		u8 valid;
	} info;
	struct {
		__u32 ba;
		__u32 uv;
		enum omap_dss_ilace_mode ilace;
	} addr;
};
static struct isr_ctx isr_ctx[5];
static DEFINE_SPINLOCK(isr_ctx_lock);

static inline int rectcmp(struct dss2_rect_t *r1, struct dss2_rect_t *r2)
{
	return !(r1->x == r2->x && r1->y == r2->y && r1->w == r2->w && r1->h == r2->h);
}

int set_dss_ovl_info(struct dss2_ovl_info *oi)
{
	struct omap_overlay_info info;
	struct omap_overlay *ovl;
	struct dss2_ovl_cfg *cfg;
	struct isr_ctx *ctx;
	union rect crop, win, vis;
	unsigned long flags = 0;
	int ret = -EINVAL, locked = 0;
	int c;
	int isr_state;

	/* check overlay number */
	if (!oi || oi->cfg.ix >= omap_dss_get_num_overlays())
		goto quit;
	cfg = &oi->cfg;
	ovl = omap_dss_get_overlay(cfg->ix);

	/* just in case there are new fields, we get the current info */
	ovl->get_overlay_info(ovl, &info);

	ctx = &isr_ctx[cfg->ix];
	isr_state = atomic_read(&ctx->state);

	info.enabled = cfg->enabled;
	if (!cfg->enabled) {
		if (isr_state == ISR_STATE_OPENED) {
			spin_lock_irqsave(&isr_ctx_lock, flags);
			locked = 1;
			ctx->info.cfg.enabled = 0;
		}
		goto done;
	}

	if (isr_state == ISR_STATE_OPENED) {
		/* if ovl crop/win changed, update ovl with addr set by avos.
		 * Otherwise, only update the info struct needed by avos */ 
		spin_lock_irqsave(&isr_ctx_lock, flags);
		locked = 1;
		if (ctx->addr.ba && ctx->info.valid &&
			    (rectcmp(&ctx->info.cfg.win, &cfg->win) ||
			    rectcmp(&ctx->info.cfg.crop, &cfg->crop))) {
			oi->ba = ctx->addr.ba;
			oi->uv = ctx->addr.uv;
			cfg->ilace = ctx->addr.ilace;
		}
		ctx->info.cfg = *cfg;
		ctx->info.valid = 1;
		if (!oi->ba) {
			ret = 0;
			goto quit;
		}		
	} else if (isr_state == ISR_STATE_CLOSED && !oi->ba) {
		/* avos don't put buffers anymore, disable overlay */ 
		info.enabled = 0;
		goto done;
	}


	/* copied params */
	info.zorder = cfg->zorder;

	if (cfg->zonly)
		goto done;

	info.global_alpha = cfg->global_alpha;
	info.pre_mult_alpha = cfg->pre_mult_alpha;
	info.rotation = cfg->rotation;
	info.mirror = cfg->mirror;
	info.color_mode = cfg->color_mode;

	/* crop to screen */
	crop.r = cfg->crop;
	win.r = cfg->win;
	vis.x = vis.y = 0;
	vis.w = ovl->manager->device->panel.timings.x_res;
	vis.h = ovl->manager->device->panel.timings.y_res;

	if (crop_to_rect(&crop, &win, &vis, cfg->rotation, cfg->mirror) ||
								vis.w < 2) {
		info.enabled = false;
		goto done;
	}

	/* adjust crop to UV pixel boundaries */
	for (c = 0; c < (cfg->color_mode == OMAP_DSS_COLOR_NV12 ? 2 :
		(cfg->color_mode &
		 (OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY)) ? 1 : 0); c++) {
		/* keep the output window to avoid trembling edges */
		crop.wh[c] += crop.xy[c] & 1;	/* round down start */
		crop.xy[c] &= ~1;
		crop.wh[c] += crop.wh[c] & 1;	/* round up end */

		/*
		 * Buffer is aligned on UV pixel boundaries, so no
		 * worries about extending crop region.
		 */
	}

	info.width  = crop.w;
	info.height = crop.h;
	if (cfg->rotation & 1)
		/* DISPC uses swapped height/width for 90/270 degrees */
		swap(info.width, info.height);
	info.pos_x = win.x;
	info.pos_y = win.y;
	info.out_width = win.w;
	info.out_height = win.h;

	/* calculate addresses and cropping */
	info.paddr = oi->ba;
	info.p_uv_addr = (info.color_mode == OMAP_DSS_COLOR_NV12) ? oi->uv : 0;
	info.vaddr = NULL;

#ifdef CONFIG_TILER_OMAP
	/* check for TILER 2D buffer */
	if (info.paddr >= 0x60000000 && info.paddr < 0x78000000) {
		int bpp = 1 << ((info.paddr >> 27) & 3);

		/* crop to top-left */

		/*
		 * DSS supports YUV422 on 32-bit mode, but its technically
		 * 2 bytes-per-pixel.
		 * Also RGB24-888 is 3 bytes-per-pixel even though no
		 * tiler pixel format matches this.
		 */
		if (cfg->color_mode &
				(OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY))
			bpp = 2;
		else if (cfg->color_mode == OMAP_DSS_COLOR_RGB24P)
			bpp = 3;

		info.paddr += crop.x * bpp +
			crop.y * tiler_stride(tiler_get_natural_addr(
							(void *) info.paddr));
		info.rotation_type = OMAP_DSS_ROT_TILER;
		info.screen_width = 0;

		/* for NV12 format also crop NV12 */
		if (info.color_mode == OMAP_DSS_COLOR_NV12)
			info.p_uv_addr += crop.x * bpp +
				(crop.y >> 1) * tiler_stride(
			tiler_get_natural_addr((void *) info.p_uv_addr));
	} else 
#endif
	{
		/* program tiler 1D as SDMA */

		int bpp = color_mode_to_bpp(cfg->color_mode);
		info.screen_width = cfg->stride * 8 / (bpp == 12 ? 8 : bpp);
		info.paddr += crop.x * (bpp / 8) + crop.y * cfg->stride;

		/* for NV12 format also crop NV12 */
		if (info.color_mode == OMAP_DSS_COLOR_NV12)
			info.p_uv_addr += crop.x * (bpp / 8) +
				(crop.y >> 1) * cfg->stride;

		if (cpu_is_omap44xx()) {
			/* no rotation on DMA buffer */
			if (cfg->rotation & 3 || cfg->mirror)
				goto quit;
			info.rotation_type = OMAP_DSS_ROT_DMA;
		} else {
#if !defined(CONFIG_TILER_OMAP) && defined(CONFIG_VIDEO_CMA)
			u32 new_paddr, old_paddr;
#endif
			info.mirror = cfg->mirror;
			info.rotation = cfg->rotation;
			info.rotation_type = OMAP_DSS_ROT_VRFB;
#if !defined(CONFIG_TILER_OMAP) && defined(CONFIG_VIDEO_CMA)
			old_paddr = info.paddr;
			ret = cma_set_output_buffer(info.paddr, &new_paddr, cfg->rotation, cfg->mirror, &info.width, &info.height, &info.screen_width);
			if (ret == 0){
				info.paddr = new_paddr;
			}
			ret = cma_is_buffer_ready(old_paddr, true);
			if (ret == -ETIME) goto quit;
#endif
		}
	}
	info.max_x_decim = cfg->decim.max_x ? : 255;
	info.max_y_decim = cfg->decim.max_y ? : 255;
	info.min_x_decim = cfg->decim.min_x ? : 1;
	info.min_y_decim = cfg->decim.min_y ? : 1;
	info.pic_height = cfg->height;

	info.field = 0;
	if (cfg->ilace & OMAP_DSS_ILACE_SEQ)
		info.field |= OMAP_FLAG_IBUF;
	if (cfg->ilace & OMAP_DSS_ILACE_SWAP)
		info.field |= OMAP_FLAG_ISWAP;
	/*
	 * Ignore OMAP_DSS_ILACE as there is no real support yet for
	 * interlaced interleaved vs progressive buffers
	 */

	info.out_wb = 0;

	/* :TODO: copy color conversion - this needs ovl support */

done:
	pr_debug("ovl%d: en=%d %x/%x (%dx%d|%d) => (%dx%d) @ (%d,%d) rot=%d "
		"mir=%d col=%x z=%d al=%02x prem=%d pich=%d ilace=%d\n",
		ovl->id, info.enabled, info.paddr, info.p_uv_addr, info.width,
		info.height, info.screen_width, info.out_width, info.out_height,
		info.pos_x, info.pos_y, info.rotation, info.mirror,
		info.color_mode, info.zorder, info.global_alpha,
		info.pre_mult_alpha, info.pic_height, info.field);

	/* set overlay info */
	ret = ovl->set_overlay_info(ovl, &info);
quit:
	if (locked)
		spin_unlock_irqrestore(&isr_ctx_lock, flags);
	return ret;
}

long convert_dss_ovl_addr(struct dsscomp_buffer *buf)
{
	__u32 addr;
	
	/* check overlay number */
	if (!buf)
		return -EINVAL;

	addr = (u32) buf->address;

	/* convert addresses to user space */
	if (buf->color_mode == OMAP_DSS_COLOR_NV12) {
		buf->uv = hwc_virt_to_phys(addr + buf->height * buf->stride +
					buf->ofs_x + buf->ofs_y / 2 * buf->stride);
	}
	buf->ba = hwc_virt_to_phys(addr + buf->ofs_x + buf->ofs_y * buf->stride);
	return 0;
}

int ovl_isr_start(struct dsscomp_dev *cdev)
{
	int i;

	for (i = 0; i < cdev->isr.num_ovls; i++)
		atomic_set(&isr_ctx[cdev->isr.ovl_ix[i]].state, ISR_STATE_OPENED);
	return 0;
}

int ovl_isr_stop(struct dsscomp_dev *cdev)
{
	int i;
	unsigned long flags;

	spin_lock_irqsave(&isr_ctx_lock, flags);
	for (i = 0; i < cdev->isr.num_ovls; i++) {
		struct isr_ctx *ctx = &isr_ctx[cdev->isr.ovl_ix[i]];

		atomic_set(&ctx->state, ISR_STATE_CLOSED);
		memset(&ctx->info.cfg, 0, sizeof(struct dss2_ovl_cfg));
		ctx->info.valid = 0;
		ctx->addr.ba = 0;
		ctx->addr.uv = 0;
		ctx->addr.ilace = 0;
	}
	spin_unlock_irqrestore(&isr_ctx_lock, flags);

	for (i = 0; i < cdev->isr.num_ovls; i++) {
		struct omap_overlay *ovl;
		struct omap_overlay_manager *mgr;
		struct omap_overlay_info info;

		ovl = omap_dss_get_overlay(cdev->isr.ovl_ix[i]);
		if (!ovl)
			continue;

		ovl->get_overlay_info(ovl, &info);
		info.enabled = 0;
		info.paddr = 0;
		info.p_uv_addr = 0;
		ovl->set_overlay_info(ovl, &info);
		mgr = ovl->manager;
		if (!mgr)
			continue;
		mgr->apply(mgr);
	}
	return 0;
}

/*
 * called from isr context
 */
int set_dss_ovl_addr(struct dsscomp_buffer *buf)
{
	struct omap_overlay_manager *mgr = NULL;
	int i;
	int r;
	/* check overlay number */
	if (!buf)
		return -EINVAL;

	for( i = 0; i < buf->num_ovls; i++ ) {
		struct omap_overlay_info info;
		struct omap_overlay *ovl;
		struct dss2_ovl_cfg *cfg;
		union rect crop, win, vis;
		int c;
		int ix = buf->ovl_ix[i];

		if( ix >= omap_dss_get_num_overlays())
			return -EINVAL;

		if (!isr_ctx[ix].info.valid) {
			continue;
		}

		ovl = omap_dss_get_overlay(ix);

		/* just in case there are new fields, we get the current info */
		ovl->get_overlay_info(ovl, &info);

		cfg = &isr_ctx[ix].info.cfg;
		isr_ctx[ix].addr.ba = buf->ba;
		isr_ctx[ix].addr.uv = buf->uv;
		isr_ctx[ix].addr.ilace = buf->ilace;

		info.enabled = cfg->enabled;
		if (!cfg->enabled) {
			continue;
		}

		/* copied params */
		info.zorder = cfg->zorder;

		if (cfg->zonly)
			continue;

		info.global_alpha = cfg->global_alpha;
		info.pre_mult_alpha = cfg->pre_mult_alpha;
		info.rotation = cfg->rotation;
		info.mirror = cfg->mirror;
		info.color_mode = cfg->color_mode;

		/* crop to screen */
		crop.r = cfg->crop;
		win.r = cfg->win;
		vis.x = vis.y = 0;
		vis.w = ovl->manager->device->panel.timings.x_res;
		vis.h = ovl->manager->device->panel.timings.y_res;

		if (crop_to_rect(&crop, &win, &vis, cfg->rotation, cfg->mirror) ||
									vis.w < 2) {
			info.enabled = false;
			goto done;
		}

		/* adjust crop to UV pixel boundaries */
		for (c = 0; c < (cfg->color_mode == OMAP_DSS_COLOR_NV12 ? 2 :
			(cfg->color_mode &
			 (OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY)) ? 1 : 0); c++) {
			/* keep the output window to avoid trembling edges */
			crop.wh[c] += crop.xy[c] & 1;	/* round down start */
			crop.xy[c] &= ~1;
			crop.wh[c] += crop.wh[c] & 1;	/* round up end */

			/*
			 * Buffer is aligned on UV pixel boundaries, so no
			 * worries about extending crop region.
			 */
		}

		info.width  = crop.w;
		info.height = crop.h;
		if (cfg->rotation & 1)
			/* DISPC uses swapped height/width for 90/270 degrees */
			swap(info.width, info.height);
		info.pos_x = win.x;
		info.pos_y = win.y;
		info.out_width = win.w;
		info.out_height = win.h;

		/* calculate addresses and cropping */
		info.paddr = buf->ba;
		info.p_uv_addr = (info.color_mode == OMAP_DSS_COLOR_NV12) ? buf->uv : 0;
		info.vaddr = NULL;

#ifdef CONFIG_TILER_OMAP
		/* check for TILER 2D buffer */
		if (info.paddr >= 0x60000000 && info.paddr < 0x78000000) {
			int bpp = 1 << ((info.paddr >> 27) & 3);

			/* crop to top-left */

			/*
			 * DSS supports YUV422 on 32-bit mode, but its technically
			 * 2 bytes-per-pixel.
			 * Also RGB24-888 is 3 bytes-per-pixel even though no
			 * tiler pixel format matches this.
			 */
			if (cfg->color_mode &
					(OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY))
				bpp = 2;
			else if (cfg->color_mode == OMAP_DSS_COLOR_RGB24P)
				bpp = 3;

			info.paddr += crop.x * bpp +
				crop.y * tiler_stride(tiler_get_natural_addr(
								(void *) info.paddr));
			info.rotation_type = OMAP_DSS_ROT_TILER;
			info.screen_width = 0;

			/* for NV12 format also crop NV12 */
			if (info.color_mode == OMAP_DSS_COLOR_NV12)
				info.p_uv_addr += crop.x * bpp +
					(crop.y >> 1) * tiler_stride(
				tiler_get_natural_addr((void *) info.p_uv_addr));
		} else 
#endif
		{
			/* program tiler 1D as SDMA */

			int bpp = color_mode_to_bpp(cfg->color_mode);
			info.screen_width = cfg->stride * 8 / (bpp == 12 ? 8 : bpp);
			info.paddr += crop.x * (bpp / 8) + crop.y * cfg->stride;

			/* for NV12 format also crop NV12 */
			if (info.color_mode == OMAP_DSS_COLOR_NV12)
				info.p_uv_addr += crop.x * (bpp / 8) +
					(crop.y >> 1) * cfg->stride;

			if (cpu_is_omap44xx()) {
				/* no rotation on DMA buffer */
				if (cfg->rotation & 3 || cfg->mirror)
					return -EINVAL;
				info.rotation_type = OMAP_DSS_ROT_DMA;
			} else {
#if !defined(CONFIG_TILER_OMAP) && defined(CONFIG_VIDEO_CMA)
				u32 new_paddr;
#endif
				info.mirror = cfg->mirror;
				info.rotation = cfg->rotation;
				info.rotation_type = OMAP_DSS_ROT_VRFB;
#if !defined(CONFIG_TILER_OMAP) && defined(CONFIG_VIDEO_CMA)
				r = cma_set_output_buffer(info.paddr, &new_paddr, cfg->rotation, cfg->mirror, &info.width, &info.height, &info.screen_width);
				if (r == 0)
				{
					info.paddr = new_paddr;
				} else {
					if (r == -ETIME)
						goto quit;
					r = 0;
				}
#endif
			}
		}
		info.max_x_decim = cfg->decim.max_x ? : 255;
		info.max_y_decim = cfg->decim.max_y ? : 255;
		info.min_x_decim = cfg->decim.min_x ? : 1;
		info.min_y_decim = cfg->decim.min_y ? : 1;
		info.pic_height = cfg->height;

		info.field = 0;
		if (buf->ilace & OMAP_DSS_ILACE_SEQ)
			info.field |= OMAP_FLAG_IBUF;
		if (buf->ilace & OMAP_DSS_ILACE_SWAP)
			info.field |= OMAP_FLAG_ISWAP;
#ifdef CONFIG_OMAP2_DSS_HDMI
		/*
		 * Ignore OMAP_DSS_ILACE as there is no real support yet for
		 * interlaced interleaved vs progressive buffers
		 */
		if (ovl->manager &&
		    ovl->manager->device &&
		    !strcmp(ovl->manager->device->name, "hdmi") &&
		    is_hdmi_interlaced())
			info.field |= OMAP_FLAG_IDEV;
#endif

		info.out_wb = 0;

		/* :TODO: copy color conversion - this needs ovl support */

done:
		pr_debug("ovl%d: en=%d %x/%x (%dx%d|%d) => (%dx%d) @ (%d,%d) rot=%d "
			"mir=%d col=%x z=%d al=%02x prem=%d pich=%d ilace=%d\n",
			ovl->id, info.enabled, info.paddr, info.p_uv_addr, info.width,
			info.height, info.screen_width, info.out_width, info.out_height,
			info.pos_x, info.pos_y, info.rotation, info.mirror,
			info.color_mode, info.zorder, info.global_alpha,
			info.pre_mult_alpha, info.pic_height, info.field);

		/* set overlay info */
		if( (r = ovl->set_overlay_info(ovl, &info)) ) {
			return r;
		}
	
		if( mgr != ovl->manager ) {
			if( mgr ) {	
				// there is an (old) manager that needs apply
				if( (r = mgr->apply(mgr)) ) {
					return r;
				}
			}
			mgr = ovl->manager;
		}
	}
	
	// there is a manager that needs apply?
	if( mgr ) {
		return mgr->apply(mgr);
	}
quit:
	return r;
}

struct omap_overlay_manager *find_dss_mgr(int display_ix)
{
	struct omap_overlay_manager *mgr;
	char name[32];
	int i;

	sprintf(name, "display%d", display_ix);

	for (i = 0; i < omap_dss_get_num_overlay_managers(); i++) {
		mgr = omap_dss_get_overlay_manager(i);
		if (mgr->device && !strcmp(name, dev_name(&mgr->device->dev)))
			return mgr;
	}
	return NULL;
}

int set_dss_mgr_info(struct dss2_mgr_info *mi)
{
	struct omap_overlay_manager_info info;
	struct omap_overlay_manager *mgr;

	if (!mi)
		return -EINVAL;
	mgr = find_dss_mgr(mi->display_index);
	if (!mgr)
		return -EINVAL;

	/* just in case there are new fields, we get the current info */
	mgr->get_manager_info(mgr, &info);

	info.alpha_enabled = mi->alpha_blending;
	info.default_color = mi->default_color;
	info.trans_enabled = mi->trans_enabled && !mi->alpha_blending;
	info.trans_key = mi->trans_key;
	info.trans_key_type = mi->trans_key_type;

	return mgr->set_manager_info(mgr, &info);
}

/*
 * ===========================================================================
 *				DEBUG METHODS
 * ===========================================================================
 */
void dump_ovl_info(struct dsscomp_dev *cdev, struct dss2_ovl_info *oi)
{
	struct dss2_ovl_cfg *c = &oi->cfg;
	const struct color_info *ci;

	if (!(debug & DEBUG_OVERLAYS) ||
	    !(debug & DEBUG_COMPOSITIONS))
		return;

	ci = get_color_info(c->color_mode);
	if (c->zonly) {
		dev_info(DEV(cdev), "ovl%d(%s z%d)\n",
			c->ix, c->enabled ? "ON" : "off", c->zorder);
		return;
	}
	dev_info(DEV(cdev), "ovl%d(%s z%d %s%s *%d%% %d*%d:%d,%d+%d,%d rot%d%s"
						" => %d,%d+%d,%d %p/%p|%d)\n",
		c->ix, c->enabled ? "ON" : "off", c->zorder,
		ci->name ? : "(none)",
		c->pre_mult_alpha ? " premult" : "",
		(c->global_alpha * 100 + 128) / 255,
		c->width, c->height, c->crop.x, c->crop.y,
		c->crop.w, c->crop.h,
		c->rotation, c->mirror ? "+mir" : "",
		c->win.x, c->win.y, c->win.w, c->win.h,
		(void *) oi->ba, (void *) oi->uv, c->stride);
}

void dump_comp_info(struct dsscomp_dev *cdev, struct dsscomp_setup_mgr_data *d,
			const char *phase)
{
	struct dss2_mgr_info *mi = &d->mgr;

	if (!(debug & DEBUG_COMPOSITIONS))
		return;

	dev_info(DEV(cdev), "[%08x] %s: %c%c%c"
		 "(dis%d(%s) alpha=%d col=%08x ilace=%d n=%d)\n",
		 d->sync_id, phase,
		 (d->mode & DSSCOMP_SETUP_MODE_APPLY) ? 'A' : '-',
		 (d->mode & DSSCOMP_SETUP_MODE_DISPLAY) ? 'D' : '-',
		 (d->mode & DSSCOMP_SETUP_MODE_CAPTURE) ? 'C' : '-',
		 mi->display_index,
		 (mi->display_index < cdev->num_displays && cdev->displays[mi->display_index]) ?
		 cdev->displays[mi->display_index]->name : "NONE",
		 mi->alpha_blending, mi->default_color,
		 mi->interlaced,
		 d->num_ovls);
}
