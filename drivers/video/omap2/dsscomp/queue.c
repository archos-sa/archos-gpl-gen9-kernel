/*
 * linux/drivers/video/omap2/dsscomp/queue.c
 *
 * DSS Composition queueing support
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
#include <linux/vmalloc.h>
#include <linux/sched.h>

#include <plat/display.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>
#include <mach/tiler.h>
#include <linux/slab.h>

#include <media/cma.h>

#include "dsscomp.h"

/* free overlay structs */
static LIST_HEAD(free_ois);

#define QUEUE_SIZE	3

#undef STRICT_CHECK

#define MAGIC_ACTIVE		0xAC54156E
#define MAGIC_APPLIED		0xA50504C1
#define MAGIC_PROGRAMMED	0x50520652
#define MAGIC_DISPLAYED		0xD15504CA

static struct dsscomp_data {
	u32 magic;
	struct dsscomp_setup_mgr_data frm;
	/*
	 * :TRICKY: before applying, overlays used in a composition are stored
	 * in ovl_mask and the other masks are empty.  Once composition is
	 * applied, blank is set to see if all overlays are to be disabled on
	 * this composition, any disabled overlays in the composition are set in
	 * ovl_dmask, and ovl_mask is updated to include ALL overlays that are
	 * actually on the display - even if they are not part of the
	 * composition. The reason: we use ovl_mask to see if an overlay is used
	 * or planned to be used on a manager.  We update ovl_mask when
	 * composition is programmed (removing the disabled overlays).
	 */
	bool blank;		/* true if all overlays are to be disabled */
	u32 ovl_mask;		/* overlays used on this frame */
	u32 ovl_dmask;		/* overlays disabled on this frame */
	u32 ix;			/* manager index that this frame is on */
	struct list_head q;
	struct list_head ois;
} *cis;

static struct dss2_overlay {
	struct dss2_ovl_info ovl;
	struct list_head q;
} *ois;

static struct {
	struct list_head q_ci;		/* compositions */
	struct list_head free_cis;	/* free composition structs */

	wait_queue_head_t wq;

	u32 ovl_mask;		/* overlays used on this display */
	u32 ovl_qmask;		/* overlays queued to this display */
	struct mutex mtx;
	wait_queue_head_t wq_free;
	bool free_slot;
} mgrq[MAX_MANAGERS];

static struct dsscomp_dev *cdev;


#define MUTEXED(exp, ix) ({ typeof(exp) __r;\
	mutex_lock(&mgrq[ix].mtx); __r = (exp); \
	mutex_unlock(&mgrq[ix].mtx); __r; })

/*
 * ===========================================================================
 *		EXIT
 * ===========================================================================
 */

/* Initialize queue structures, and set up state of the displays */
int dsscomp_queue_init(struct dsscomp_dev *cdev_)
{
	u32 i, j, ncis, nois;

	cdev = cdev_;
	ncis = QUEUE_SIZE * cdev->num_mgrs;
	nois = QUEUE_SIZE * cdev->num_ovls;

	INIT_LIST_HEAD(&free_ois);

	cis = vmalloc(sizeof(*cis) * ncis);
	ois = vmalloc(sizeof(*ois) * nois);

	if (cis && ois && ARRAY_SIZE(mgrq) >= cdev->num_mgrs) {
		ZERO(mgrq);
		ZEROn(cis, ncis);
		ZEROn(ois, nois);

		for (i = 0; i < nois; i++)
			list_add(&ois[i].q, &free_ois);
		for (i = 0; i < cdev->num_mgrs; i++) {
			struct omap_overlay_manager *mgr;
			INIT_LIST_HEAD(&mgrq[i].q_ci);
			INIT_LIST_HEAD(&mgrq[i].free_cis);
			init_waitqueue_head(&mgrq[i].wq);
			init_waitqueue_head(&mgrq[i].wq_free);

			for (j = 0; j < QUEUE_SIZE; j++)
				list_add(&cis[j + QUEUE_SIZE * i].q,
							&mgrq[i].free_cis);

			/* record overlays on this display */
			mgr = cdev->mgrs[i];
			for (j = 0; j < cdev->num_ovls; j++) {
				if (cdev->ovls[j]->info.enabled &&
				    mgr &&
				    cdev->ovls[j]->manager == mgr)
					mgrq[i].ovl_mask |= 1 << j;
			}
			mutex_init(&mgrq[i].mtx);
		}
		return 0;
	} else {
		vfree(cis);
		vfree(ois);
		return -ENOMEM;
	}
}

/* returns if composition is valid and active */
static inline struct dsscomp_data *validate(struct dsscomp_data *comp)
{
#ifdef STRICT_CHECK
	u32 ix, q_ix;
	struct dsscomp_data *c;
	struct omap_overlay_manager *mgr;

	if (!comp)
		return ERR_PTR(-EFAULT);

	ix = comp->frm.mgr.ix;
	if (ix >= cdev->num_displays || !cdev->displays[ix])
		return ERR_PTR(-EINVAL);
	mgr = cdev->displays[ix]->manager;
	if (!mgr || mgr->id >= cdev->num_mgrs)
		return ERR_PTR(-ENODEV);

	/* check if composition is active */
	list_for_each_entry(c, &mgrq[mgr->id].q_ci, q)
		if (c == comp)
			return c;
	return ERR_PTR(-ESRCH);
#else
	if (!comp)
		return ERR_PTR(-EFAULT);

	return (comp->magic == MAGIC_PROGRAMMED ||
		comp->magic == MAGIC_DISPLAYED ||
		comp->magic == MAGIC_APPLIED ||
		comp->magic == MAGIC_ACTIVE) ? comp : ERR_PTR(-ESRCH);
#endif
}


/* get display index from manager */
u32 dsscomp_get_display_index(struct omap_overlay_manager *mgr)
{
	u32 i;

	/* handle if manager is not attached to a display */
	if (!mgr || !mgr->device)
		return cdev->num_displays;

	/* find manager's display */
	for (i = 0; i < cdev->num_displays; i++)
		if (cdev->displays[i] == mgr->device)
			break;

	return i;
}

/*
 * ===========================================================================
 *		QUEUING SETUP OPERATIONS
 * ===========================================================================
 */

/* get composition by sync_id */
static dsscomp_t dsscomp_get(struct omap_overlay_manager *mgr, u32 sync_id)
{
	struct dsscomp_data *comp;

	/* get display index */
	u32 ix = mgr ? mgr->id : cdev->num_mgrs;
	if (ix >= cdev->num_mgrs)
		return NULL;

	/* find composition with sync id on manager */
	list_for_each_entry(comp, &mgrq[ix].q_ci, q)
		if (comp->frm.sync_id == sync_id)
			return comp;

	return NULL;
}

/* create a new composition for a display */
dsscomp_t dsscomp_new_sync_id(struct omap_overlay_manager *mgr, u32 sync_id,
	bool blankpost)
{
	struct dsscomp_data *comp = NULL;
	int r;
	u32 display_ix = dsscomp_get_display_index(mgr);

	/* check manager */
	u32 ix = mgr ? mgr->id : cdev->num_mgrs;
	if (ix >= cdev->num_mgrs || display_ix >= cdev->num_displays)
		return ERR_PTR(-EINVAL);

	/* see if sync_id exists */
	mutex_lock(&mgrq[ix].mtx);

	if (dsscomp_get(mgr, sync_id)) {
		r = -EEXIST;
		goto done;
	}

parse_again:
	/* check if there is space on the queue */
	if (list_empty(&mgrq[ix].free_cis)) {
		if (blankpost) {
			list_for_each_entry(comp, &mgrq[ix].q_ci, q) {
				if (comp->magic == MAGIC_ACTIVE)
					break;
			}

			if (&comp->q == &mgrq[ix].q_ci) {
				r = -EBUSY;
				goto done;
			}
			dsscomp_drop(comp);
		} else {
			mgrq[ix].free_slot = false;
			/* relese mutex and wait for free comp */
			mutex_unlock(&mgrq[ix].mtx);
			wait_event_interruptible_timeout(mgrq[ix].wq_free,
				mgrq[ix].free_slot, msecs_to_jiffies(60));

			if (!mgrq[ix].free_slot) {
				r = -EBUSY;
				dev_err(DEV(cdev), "[%d] QBUSY\n", mgr->id);
				return ERR_PTR(r);
			}
			mutex_lock(&mgrq[ix].mtx);
			goto parse_again;
		}
	}

	/* initialize new composition */
	comp = list_first_entry(&mgrq[ix].free_cis, typeof(*comp), q);
	list_move_tail(&comp->q, &mgrq[ix].q_ci);
	comp->ix = ix;	/* save where this composition came from */
	ZERO(comp->frm);
	INIT_LIST_HEAD(&comp->ois);
	comp->ovl_mask = comp->ovl_dmask = 0;
	comp->frm.sync_id = sync_id;
	comp->frm.mgr.display_index = display_ix;

	/* :TODO: retrieve last manager configuration */

	comp->magic = MAGIC_ACTIVE;
	r = 0;
 done:
	mutex_unlock(&mgrq[ix].mtx);
	return r ? ERR_PTR(r) : comp;
}
EXPORT_SYMBOL(dsscomp_new_sync_id);

dsscomp_t dsscomp_find(struct omap_overlay_manager *mgr, u32 sync_id)
{
	struct dsscomp_data *comp;

	/* check manager */
	if (!mgr || mgr->id >= cdev->num_mgrs)
		return ERR_PTR(-EINVAL);

	/* see if sync_id exists */
	mutex_lock(&mgrq[mgr->id].mtx);
	comp = dsscomp_get(mgr, sync_id);
	if (IS_ERR(comp))
		comp = NULL;
	mutex_unlock(&mgrq[mgr->id].mtx);
	return comp;
}
EXPORT_SYMBOL(dsscomp_find);

void dsscomp_release_active_comps()
{
	int ix;
	struct dsscomp_data *c, *c2;
	for (ix = 0; ix < omap_dss_get_num_overlay_managers(); ix++) {
		mutex_lock(&mgrq[ix].mtx);
		list_for_each_entry_safe(c, c2, &mgrq[ix].q_ci, q) {
			if (c) {
				if (c->magic == MAGIC_ACTIVE)
					dsscomp_drop(c);
			}
		}
		mutex_unlock(&mgrq[ix].mtx);
	}
}

void dsscomp_release_programmed_comps(int id)
{
	struct dsscomp_data *c, *c2;
	list_for_each_entry_safe(c, c2, &mgrq[id].q_ci, q) {
		if (c) {
			if (c->magic == MAGIC_PROGRAMMED)
				dsscomp_drop(c);
		}
	}
}

/* find first unapplied sync_id or 0 if none found */
u32 dsscomp_first_sync_id(struct omap_overlay_manager *mgr)
{
	struct dsscomp_data *comp;
	u32 sync_id = 0;

	/* get display index */
	u32 ix = mgr ? mgr->id : cdev->num_mgrs;
	if (ix >= cdev->num_mgrs)
		return 0;

	/* find first unapplied composition */
	mutex_lock(&mgrq[ix].mtx);
	list_for_each_entry(comp, &mgrq[ix].q_ci, q) {
		if (comp->magic == MAGIC_ACTIVE &&
		    (DSSCOMP_SETUP_MODE_APPLY & ~comp->frm.mode)) {
			sync_id = comp->frm.sync_id;
			break;
		}
	}
	mutex_unlock(&mgrq[ix].mtx);

	return sync_id;
}
EXPORT_SYMBOL(dsscomp_first_sync_id);

/* returns overlays used in a composition */
u32 dsscomp_get_ovls(dsscomp_t comp)
{
	if (!comp)
		return 0;

	BUG_ON(MUTEXED(IS_ERR(validate(comp)), comp->ix));

	return comp->ovl_mask;
}
EXPORT_SYMBOL(dsscomp_get_ovls);

/* set overlay info */
int dsscomp_set_ovl(dsscomp_t comp, struct dss2_ovl_info *ovl)
{
	int r = -EFAULT;

	if (comp && ovl) {
		u32 i, mask = 1 << ovl->cfg.ix;
		struct omap_overlay *o;
		struct dss2_overlay *oi;
		dsscomp_t chkcomp;
		u32 ix = comp->frm.mgr.display_index;
		if (ix < cdev->num_displays &&
		    cdev->displays[ix] &&
		    cdev->displays[ix]->manager)
			ix = cdev->displays[ix]->manager->id;
		else
			ix = cdev->num_mgrs;
		if (ix >= cdev->num_mgrs)
			return -ENODEV;

		mutex_lock(&mgrq[ix].mtx);

		/* check if composition is active */
		chkcomp = validate(comp);
		if (IS_ERR(chkcomp)) {
			r = PTR_ERR(chkcomp);
			goto done;
		}

		if (comp->magic != MAGIC_ACTIVE) {
			r = -EACCES;
			goto done;
		}

		if (ovl->cfg.ix >= cdev->num_ovls) {
			r = -EINVAL;
			goto done;
		}

		/* if overlay is already part of the composition */
		if (mask & comp->ovl_mask) {
			/* look up overlay */
			list_for_each_entry(oi, &comp->ois, q)
				if (oi->ovl.cfg.ix == ovl->cfg.ix)
					break;
			BUG_ON(&oi->q == &comp->ois);
		} else {
			/* check if ovl is free to use */
			r = -EBUSY;
			if (list_empty(&free_ois))
				goto done;

			/* not in any other displays queue */
			if (mask & ~mgrq[ix].ovl_qmask) {
				for (i = 0; i < cdev->num_mgrs; i++) {

					if (i == ix)
						continue;
					if (mgrq[i].ovl_qmask & mask)
						goto done;
				}
			}

			/* and disabled (unless forced) if on another manager */
			o = cdev->ovls[ovl->cfg.ix];
			if (o->info.enabled &&
					(!o->manager || o->manager->id != ix))
				goto done;

			/* add overlay to composition & display */
			comp->ovl_mask |= mask;
			comp->frm.num_ovls++;
			mgrq[ix].ovl_qmask |= mask;

			oi = list_first_entry(&free_ois, typeof(*oi), q);
			list_move_tail(&oi->q, &comp->ois);
		}

		oi->ovl = *ovl;
		r = 0;
 done:
		mutex_unlock(&mgrq[ix].mtx);
	}

	return r;
}
EXPORT_SYMBOL(dsscomp_set_ovl);

/* get overlay info */
int dsscomp_get_ovl(dsscomp_t comp, u32 ix, struct dss2_ovl_info *ovl)
{
	int r = -EFAULT;
	struct dss2_overlay *oi;

	if (comp && ovl) {
		dsscomp_t chkcomp;
		mutex_lock(&mgrq[comp->ix].mtx);

		/* check if composition is active */
		chkcomp = validate(comp);
		if (IS_ERR(chkcomp)) {
			r = PTR_ERR(chkcomp);
		} else if (comp->magic != MAGIC_ACTIVE) {
			r = -EACCES;
		} else {
			if (ix >= cdev->num_ovls) {
				r = -EINVAL;
			} else if (comp->ovl_mask & (1 << ix)) {
				r = 0;
				list_for_each_entry(oi, &comp->ois, q) {
					if (oi->ovl.cfg.ix == ix) {
						*ovl = oi->ovl;
						break;
					}
				}
				BUG_ON(&oi->q == &comp->ois);
			} else {
				/* :TODO: get past overlay info */
				r = -ENOENT;
			}
		}

		mutex_unlock(&mgrq[comp->ix].mtx);
	}

	return r;
}
EXPORT_SYMBOL(dsscomp_get_ovl);

/* get first overlay info in the composition*/
int dsscomp_get_first_ovl(dsscomp_t comp, struct dss2_ovl_info *ovl)
{
	int r = -EFAULT;
	struct dss2_overlay *oi;

	if (comp && ovl) {
		dsscomp_t chkcomp;
		mutex_lock(&mgrq[comp->ix].mtx);

		/* check if composition is active */
		chkcomp = validate(comp);
		if (IS_ERR(chkcomp))
			r = PTR_ERR(chkcomp);
		else if (comp->magic != MAGIC_ACTIVE)
			r = -EACCES;
		else {
			if (!list_empty(&comp->ois)) {
				r = 0;
				oi = list_first_entry(&comp->ois,
						typeof(*oi), q);
				*ovl = oi->ovl;
			}
		}
		mutex_unlock(&mgrq[comp->ix].mtx);
	}

	return r;
}
EXPORT_SYMBOL(dsscomp_get_first_ovl);


/* set manager info */
int dsscomp_set_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr)
{
	int r = -EFAULT;

	if (comp && mgr) {
		dsscomp_t chkcomp;
		mutex_lock(&mgrq[comp->ix].mtx);

		/* check if composition is active */
		chkcomp = validate(comp);
		if (IS_ERR(chkcomp)) {
			r = PTR_ERR(chkcomp);
			goto done;
		}

		if (comp->magic != MAGIC_ACTIVE) {
			r = -EACCES;
			goto done;
		}

		/* set display index in manager info */
		mgr->display_index = comp->frm.mgr.display_index;
		comp->frm.mgr = *mgr;
		r = 0;
 done:
		mutex_unlock(&mgrq[comp->ix].mtx);
	}

	return r;
}
EXPORT_SYMBOL(dsscomp_set_mgr);

/* get manager info */
int dsscomp_get_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr)
{
	int r = -EFAULT;

	if (comp && mgr) {
		dsscomp_t chkcomp;
		mutex_lock(&mgrq[comp->ix].mtx);

		/* check if composition is active */
		chkcomp = validate(comp);
		if (IS_ERR(chkcomp)) {
			r = PTR_ERR(chkcomp);
		} else if (comp->magic != MAGIC_ACTIVE) {
			r = -EACCES;
		} else {
			r = 0;
			*mgr = comp->frm.mgr;
		}

		mutex_unlock(&mgrq[comp->ix].mtx);
	}

	return r;
}
EXPORT_SYMBOL(dsscomp_get_mgr);

/* get manager info */
int dsscomp_setup(dsscomp_t comp, enum dsscomp_setup_mode mode,
			struct dss2_rect_t win)
{
	int r = -EFAULT;

	if (comp) {
		dsscomp_t chkcomp;
		mutex_lock(&mgrq[comp->ix].mtx);

		/* check if composition is active */
		chkcomp = validate(comp);
		if (IS_ERR(chkcomp)) {
			r = PTR_ERR(chkcomp);
		} else if (comp->magic != MAGIC_ACTIVE) {
			r = -EACCES;
		} else {
			r = 0;
			comp->frm.mode = mode;
			comp->frm.win = win;
		}

		mutex_unlock(&mgrq[comp->ix].mtx);
	}

	return r;
}
EXPORT_SYMBOL(dsscomp_setup);

/*
 * ===========================================================================
 *		QUEUING COMMITTING OPERATIONS
 * ===========================================================================
 */

static void refresh_masks(u32 ix)
{
	struct dsscomp_data *c;

	mgrq[ix].ovl_qmask = mgrq[ix].ovl_mask;
	list_for_each_entry(c, &mgrq[ix].q_ci, q) {
		if (c->magic != MAGIC_PROGRAMMED)
			mgrq[ix].ovl_qmask |= c->ovl_mask;
	}

	wake_up_interruptible_sync(&mgrq[ix].wq);
}

void dsscomp_drop(dsscomp_t c)
{
	struct dss2_overlay *o, *o2;

	if (debug & DEBUG_COMPOSITIONS)
		dev_info(DEV(cdev), "[%08x] released\n", c->frm.sync_id);

	list_for_each_entry_safe(o, o2, &c->ois, q) {
		if (o->ovl.cfg.enabled && o->ovl.ba) {
			if (o->ovl.cfg.color_mode == OMAP_DSS_COLOR_NV12) {
#ifdef CONFIG_TILER_OMAP
				tiler_set_buf_state(o->ovl.ba, TILBUF_FREE);
#endif
			} else if (o->ovl.cfg.color_mode == OMAP_DSS_COLOR_UYVY) {
#ifdef CONFIG_VIDEO_CMA
				cma_set_buf_state(o->ovl.ba, CMABUF_FREE, -1);
#endif
			}
		}
	}
	list_for_each_entry_safe(o, o2, &c->ois, q)
		list_move(&o->q, &free_ois);
	list_move(&c->q, &mgrq[c->ix].free_cis);
	c->magic = 0;

	mgrq[c->ix].free_slot = true;
	wake_up_interruptible_sync(&mgrq[c->ix].wq_free);
}
EXPORT_SYMBOL(dsscomp_drop);

/* get manager info */
int dsscomp_apply(dsscomp_t comp)
{
	int i, r = -EFAULT;
	u32 dmask, display_ix;
	struct omap_dss_device *dssdev;
	struct omap_dss_driver *drv;
	struct omap_overlay_manager *mgr;
	struct omap_overlay *ovl;
	struct dsscomp_setup_mgr_data *d;
	struct dsscomp_data *c, *c2;
	struct dss2_overlay *o, *o2;
	bool change = false;
	dsscomp_t chkcomp;

	if (!comp)
		return -EINVAL;

	mutex_lock(&mgrq[comp->ix].mtx);

	/* check if composition is active */
	chkcomp = validate(comp);
	if (IS_ERR(chkcomp)) {
		r = PTR_ERR(chkcomp);
		goto done;
	}

	if (comp->magic != MAGIC_ACTIVE) {
		r = -EACCES;
		goto done;
	}

	/* check if the display is valid and used */
	r = -ENODEV;
	d = &comp->frm;
	display_ix = d->mgr.display_index;
	if (display_ix >= cdev->num_displays)
		goto done;
	dssdev = cdev->displays[display_ix];
	if (!dssdev)
		goto done;

	drv = dssdev->driver;
	mgr = dssdev->manager;
	if (!mgr || !drv || mgr->id >= cdev->num_mgrs)
		goto done;

	/* skip all unapplied prior compositions */
	list_for_each_entry_safe(c, c2, &mgrq[mgr->id].q_ci, q) {
		if (c == comp)
			break;

		/* keep applied compositions, as callback has been scheduled */
		if (c->magic == MAGIC_ACTIVE) {
			dsscomp_drop(c);
			change = true;
		}
	}

	dump_comp_info(cdev, d, "apply");

	r = 0;
	dmask = 0;
	list_for_each_entry_safe(o, o2, &comp->ois, q) {
		struct dss2_ovl_info *oi = &o->ovl;

		/* keep track of disabled overlays */
		if (!oi->cfg.enabled)
			dmask |= 1 << oi->cfg.ix;

		if (r)
			goto done_ovl;

		dump_ovl_info(cdev, oi);

		if (oi->cfg.ix >= cdev->num_ovls) {
			r = -EINVAL;
			goto done_ovl;
		}
		ovl = cdev->ovls[oi->cfg.ix];

		/* set overlays' manager & info */
		if (ovl->info.enabled && ovl->manager != mgr) {
			r = -EBUSY;
			goto done_ovl;
		}
		if (ovl->manager != mgr) {
			/*
			 * Ideally, we should call ovl->unset_manager(ovl),
			 * but it may block on go even though the disabling
			 * of the overlay already went through.  So instead,
			 * we are just clearing the manager.
			 */
			ovl->manager = NULL;
			r = ovl->set_manager(ovl, mgr);
			if (r)
				goto done_ovl;
		}

		r = set_dss_ovl_info(oi);
	}

done_ovl:
	/*
	 * set manager's info - this also sets the completion callback,
	 * so if it succeeds, we will use the callback to complete the
	 * composition.  Otherwise, we can skip the composition now.
	 */
	r = r ? : set_dss_mgr_info(&d->mgr);
	if (r) {
		dev_err(DEV(cdev), "[%08x] [%d] set failed %d\n",
					d->sync_id, mgr->id, r);
		dsscomp_drop(comp);
		refresh_masks(mgr->id);
		change = true;
		goto done;
	} else {
		/* override manager's callback to avoid eclipsed cb */
		comp->blank = dmask == comp->ovl_mask;
		comp->ovl_dmask = dmask;
		/*
		 * Check other overlays that may also use this display.
		 * NOTE: This is only needed in case someone changes
		 * overlays via sysfs.
		 */
		for (i = 0; i < cdev->num_ovls; i++) {
			u32 mask = 1 << i;
			if ((~comp->ovl_mask & mask) &&
			    cdev->ovls[i]->info.enabled &&
			    cdev->ovls[i]->manager == mgr)
				comp->ovl_mask |= mask;
		}
	}

	/* apply changes and call update on manual panels */
	comp->magic = MAGIC_APPLIED;
	mutex_unlock(&mgrq[comp->ix].mtx);

	if (dssdev_manually_updated(dssdev)) {
		if (!d->win.w && !d->win.x)
			d->win.w = dssdev->panel.timings.x_res - d->win.x;
		if (!d->win.h && !d->win.y)
			d->win.h = dssdev->panel.timings.y_res - d->win.y;

		/* sync to prevent frame loss */
		r = drv->sync(dssdev) ? : mgr->apply(mgr);

		if (!r && (d->mode & DSSCOMP_SETUP_MODE_DISPLAY)) {
			if (drv->update)
				r = drv->update(dssdev, d->win.x,
					d->win.y, d->win.w, d->win.h);
		}
	} else {
		/* wait for sync to avoid tear */
		r = mgr->apply(mgr);
		if (!r)
			mgr->wait_for_vsync(mgr);
		else
			dev_err(DEV(cdev), "failed while applying %d", r);

		/* ignore this error if callback has already been registered */
		if (!mgr->info_dirty)
			r = 0;
	}
	mutex_lock(&mgrq[comp->ix].mtx);

done:
	if (change)
		refresh_masks(mgr->id);

	if (!r) {
		dsscomp_release_programmed_comps(mgr->id);
		comp->magic = MAGIC_PROGRAMMED;
		/* update used overlay mask */
		mgrq[mgr->id].ovl_mask = comp->ovl_mask & ~comp->ovl_dmask;
		refresh_masks(mgr->id);
	}

	mutex_unlock(&mgrq[comp->ix].mtx);

	return r;
}
EXPORT_SYMBOL(dsscomp_apply);

/*
 * ===========================================================================
 *		WAIT OPERATIONS
 * ===========================================================================
 */

/* return true iff composition phase has passed */
static bool is_wait_over(dsscomp_t comp, enum dsscomp_wait_phase phase)
{
	return IS_ERR(validate(comp)) ||
		(phase == DSSCOMP_WAIT_PROGRAMMED &&
			(comp->magic == MAGIC_PROGRAMMED ||
			 comp->magic == MAGIC_DISPLAYED)) ||
		(phase == DSSCOMP_WAIT_DISPLAYED &&
			comp->magic == MAGIC_DISPLAYED) ||
		(phase == DSSCOMP_WAIT_RELEASED &&
			comp->magic == 0);
}

/* wait for programming or release of a composition */
int dsscomp_wait(dsscomp_t comp, enum dsscomp_wait_phase phase, int timeout)
{
	u32 id;
	dsscomp_t chkcomp;

	if (!comp)
		return -EINVAL;

	mutex_lock(&mgrq[comp->ix].mtx);

	chkcomp = validate(comp);
	id = IS_ERR(chkcomp) ? 0 : comp->frm.sync_id;
	if (debug & DEBUG_WAITS)
		dev_info(DEV(cdev), "wait %s on [%08x]\n",
			phase == DSSCOMP_WAIT_DISPLAYED ? "display" :
			phase == DSSCOMP_WAIT_PROGRAMMED ? "program" :
			"release", id);

	if (!IS_ERR(comp) && !is_wait_over(comp, phase)) {
		u32 ix = comp->frm.mgr.display_index;

		mutex_unlock(&mgrq[comp->ix].mtx);

		if (ix >= cdev->num_displays ||
			!cdev->displays[ix] ||
			!cdev->displays[ix]->manager)
			return -EINVAL;

		ix = cdev->displays[ix]->manager->id;
		if (ix >= cdev->num_mgrs)
			return -EINVAL;

		/*
		 * we can check being active without mutex because we will
		 * also check it while holding the mutex before returning
		 */
		timeout = wait_event_interruptible_timeout(mgrq[ix].wq,
			is_wait_over(comp, phase), timeout);
		if (debug & DEBUG_WAITS)
			dev_info(DEV(cdev), "wait over [%08x]: %s %d\n", id,
				 timeout < 0 ? "signal" :
				 timeout > 0 ? "ok" : "timeout",
				 timeout);
		if (timeout <= 0)
			return timeout ? : -ETIME;
		mutex_lock(&mgrq[ix].mtx);
	}

	mutex_unlock(&mgrq[comp->ix].mtx);

	return 0;
}
EXPORT_SYMBOL(dsscomp_wait);

/*
 * ===========================================================================
 *		EXIT
 * ===========================================================================
 */
void dsscomp_queue_exit(void)
{
	struct dsscomp_data *c, *c2;
	if (cis && ois && cdev) {
		int i;
		for (i = 0; i < cdev->num_mgrs; i++)
			mutex_destroy(&mgrq[i].mtx);

		for (i = 0; i < cdev->num_mgrs; i++) {
			list_for_each_entry_safe(c, c2, &mgrq[i].q_ci, q)
				dsscomp_drop(c);
		}
		vfree(cis);
		vfree(ois);
		cis = NULL;
		ois = NULL;
		cdev = NULL;
	}
}
EXPORT_SYMBOL(dsscomp_queue_exit);
