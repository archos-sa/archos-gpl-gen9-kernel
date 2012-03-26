/*
 * cma.c
 *
 * Contiguous Memory Allocator
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>            /* struct cdev */
#include <linux/kdev_t.h>          /* MKDEV() */
#include <linux/fs.h>              /* register_chrdev_region() */
#include <linux/device.h>          /* struct class */
#include <linux/platform_device.h> /* platform_device() */
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <plat/vrfb.h>
#include <plat/dma.h>

#ifdef CONFIG_PM
#include <plat/omap-pm.h>
#endif

#define MODULE_NAME "cma"

#include <media/cma.h>

#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY
#include <mach/isp_user.h>
#include <linux/ispdss.h>
#include "../isp/ispresizer.h"
#include <linux/omap_resizer.h>
#endif

int debug_cma;
module_param(debug_cma, int, 0644);
int sync_cma = 1;	// for now force CMA in sync mode!
module_param(sync_cma, int, 0644);

#define NB_BUFFERS 	6
#define BPP	   	2
#define WIDTH		1280
#define HEIGHT		736
#define MAX_PIXELS_PER_LINE	2048

#define CMA_BUFFER_SIZE  PAGE_ALIGN(WIDTH*HEIGHT*BPP)

#define NB_OVERLAYS 3

struct cma_dev {
	struct cdev cdev;

	struct blocking_notifier_head notifier;
};

struct buf_info {
	wait_queue_head_t wq;
	bool locked;
	bool requested;
	enum cmabuf_state state;
	unsigned long phys_addr;
	unsigned long vrfb_paddr;
	int dma_ch;
	int width;
	int height;
	int ovl_ix;
	int out_width;
	int out_height;
	void* virt_addr;
	void* vrfb_vaddr;
	int use_vrfb;
	unsigned long max_size;
	struct cma_buffer info;
	struct vrfb vrfb_ctx;
#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY
	struct list_head node;
	int index;
#endif
};

#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY
struct cma_resizer_work {
	struct work_struct work;
	struct buf_info *info;
};
#endif

struct ovl_win {
	int width;
	int height;
	int rotation;
};

static struct cma_dev *cma_device;
static struct device *device;
static struct class *cmadev_class;
static s32 cma_major;
static s32 cma_minor;

static struct buf_info buffer_state[NB_BUFFERS];
static struct ovl_win ovl_win[NB_OVERLAYS];

static struct mutex mtx;

static struct platform_driver cma_driver_ldm = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "cma",
	},
};

#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY
struct isp_node pipe;
static LIST_HEAD(resizer_queue);
static struct workqueue_struct *resizer_workqueue;
#endif

static s32 cma_open(struct inode *ip, struct file *filp)
{
	int ret = 0;
	/* get the ISP resizer resource and configure it*/
	ispdss_put_resource();
	ret = ispdss_get_resource();
	if (ret) {
		printk(KERN_ERR "<%s>: <%s> failed to get ISP "
				"resizer resource = %d\n",
				__FILE__, __func__, ret);
	}
	return 0;
}

/*
 * Allocate buffers
 */
static int cma_alloc_buffers(void)
{
	int i, size;

	for (i = 0; i < NB_BUFFERS; i++) {
#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY
		buffer_state[i].index = i;
#endif
		buffer_state[i].locked    = false;
		buffer_state[i].requested = false;
		buffer_state[i].dma_ch    = -1;
		buffer_state[i].state     = CMABUF_READY;
		init_waitqueue_head(&buffer_state[i].wq);
		size = CMA_BUFFER_SIZE;
		buffer_state[i].virt_addr = alloc_pages_exact(size, GFP_KERNEL | GFP_DMA);
		if (buffer_state[i].virt_addr) {
			buffer_state[i].phys_addr   = virt_to_phys(buffer_state[i].virt_addr);
			buffer_state[i].max_size    = CMA_BUFFER_SIZE;
			buffer_state[i].info.offset = i << PAGE_SHIFT;
			buffer_state[i].info.index  = i;
			buffer_state[i].info.size   = 0;
		} else {
			printk(KERN_ERR "can not allocate buffers %d\n",i);
			return -1;
		}
		buffer_state[i].vrfb_vaddr = alloc_pages_exact(size, GFP_KERNEL | GFP_DMA);
		if (buffer_state[i].vrfb_vaddr) {
		      buffer_state[i].vrfb_paddr=virt_to_phys(buffer_state[i].vrfb_vaddr);
		} else {
			printk(KERN_ERR "can not allocate shadow vrfb buffers %d\n",i);
			return -1;
		}
		
		/* Setup associated vrfb context */
		omap_vrfb_request_ctx(&buffer_state[i].vrfb_ctx);
	}
	return 0;
}

static int cma_unlockbuf(int index)
{
	mutex_lock(&mtx);
	if (index < NB_BUFFERS) {
		if (buffer_state[index].locked == true) {
			buffer_state[index].locked = false;
			buffer_state[index].requested = false;
			wake_up_interruptible_sync(&buffer_state[index].wq);
		}
	}
	mutex_unlock(&mtx);
	return 0;
}

static int cma_waitbuf_unlocked(int index)
{
	u32 timeout = usecs_to_jiffies(66 * 1000); /*delay before timing out */

	if (buffer_state[index].locked == true) {
		mutex_unlock(&mtx);
		timeout = wait_event_interruptible_timeout(buffer_state[index].wq, buffer_state[index].locked == false, timeout);
		if (!timeout) {
			printk(KERN_ERR "CMA Buffer %d is still locked\n", index);
		}
		mutex_lock(&mtx);
		if (timeout <= 0)
			return timeout ? : -ETIME;
	}

	return 0;
}

static int cma_lockbuf(u32 index, bool force)
{
	int r;
	mutex_lock(&mtx);
	if (!force) r = cma_waitbuf_unlocked(index);
	else r =0;
	buffer_state[index].locked = true;
	mutex_unlock(&mtx);
	return r;
}

static int cma_waitbuf_ready(int index)
{
	u32 timeout = usecs_to_jiffies(33 * 1000); /*delay before timing out */

	if (buffer_state[index].state != CMABUF_READY) {
		timeout = wait_event_interruptible_timeout(buffer_state[index].wq, (buffer_state[index].state==CMABUF_READY), timeout);
		if (timeout == 0) {
			return -ETIME;
		}
	}

	return 0;
}

#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY

static int __enable_isp_rsz(int index, int line)
{
	int src_w, src_h, dst_w, dst_h, scale;
	int ovl_ix = buffer_state[index].ovl_ix;

	/* ISP resizer used for scaling 1/4x~1/8x and,
	 * for width > 1024 and scaling 1/2x~1/8x
	 */

        /* The input size is the same as the output size and not too big, no need to resize, just copy*/
	if (buffer_state[index].width == ovl_win[ovl_ix].width) return -1;

	if (buffer_state[index].width >= 1024)
		goto enable_and_exit;


	/* Check for vertical scale */
	src_h = buffer_state[index].height;
	dst_h = ovl_win[ovl_ix].height;
	scale = (1024 * src_h)/dst_h;
	if (scale > 4096)
		goto enable_and_exit;

	/* Check for horizontal scale */
	src_w = buffer_state[index].width;
	dst_w = ovl_win[ovl_ix].width;
	scale = (1024 * src_w)/dst_w;
	if (scale > 4096)
		goto enable_and_exit;

	return -1;

enable_and_exit:
	return 0;
}
#define enable_isp_rsz(x) __enable_isp_rsz(x, __LINE__)

static void __disable_isp_rsz(int index, int line)
{
}
#define disable_isp_rsz(x) __disable_isp_rsz(x, __LINE__)

static void cma_setup_pipe_from_buffer(int index, struct isp_node *pipe)
{
	/* setup source parameters */

	pipe->in.image.width = buffer_state[index].width;
	pipe->in.image.height = buffer_state[index].height;
	pipe->in.image.bytesperline = buffer_state[index].width * BPP;
	pipe->in.image.pixelformat = V4L2_PIX_FMT_UYVY;
	pipe->in.image.field = V4L2_FIELD_NONE;
	pipe->in.image.sizeimage = buffer_state[index].width * buffer_state[index].height * BPP;
	pipe->in.image.colorspace = V4L2_COLORSPACE_JPEG;

	pipe->in.crop.left = 0; pipe->in.crop.top = 0;
	pipe->in.crop.width = buffer_state[index].width;
	pipe->in.crop.height = buffer_state[index].height;

	pipe->out.image.width = buffer_state[index].out_width;
	pipe->out.image.height = buffer_state[index].out_height;

}

static void tput_get( void )
{
#ifdef CONFIG_PM
	if (!cpu_is_omap44xx()) {
		if (cpu_is_omap3630()) {
			omap_pm_set_min_bus_tput( device, OCP_INITIATOR_AGENT, 200 * 1000 * 4);
		} else {
			omap_pm_set_min_bus_tput( device, OCP_INITIATOR_AGENT, 166 * 1000 * 4);
		}
	}
#endif
}

static void tput_put( void )
{
#ifdef CONFIG_PM
	if (!cpu_is_omap44xx()) {
		omap_pm_set_min_bus_tput( device, OCP_INITIATOR_AGENT, 0);
	}
#endif
}

static u64 atime( void )
{
	struct timeval tv;
	do_gettimeofday(&tv);

	return (u64)1000000 * (u64)tv.tv_sec + (u64)tv.tv_usec;
}

static int init_isp_rsz(int index);

static void do_resize(struct buf_info *next) 
{
	int ret = 0;
	u64 start = atime();
	u64 p1, p2, p3, p4;

	tput_get();

	init_isp_rsz(next->index);
	p1 = atime() - start;

	ret = ispdss_begin(&pipe, next->index, next->index,
		MAX_PIXELS_PER_LINE * 4,
		buffer_state[next->index].vrfb_ctx.paddr[0],
		buffer_state[next->index].phys_addr,
		buffer_state[next->index].info.size);

	p2 = atime() - start;

	if (ret) {
		printk(KERN_ERR "<%s> ISP resizer Failed to resize "
				"the buffer = %d\n",
				__func__, ret);
	}

	buffer_state[next->index].dma_ch = -1;
	buffer_state[next->index].state = CMABUF_READY;
	disable_isp_rsz(next->index);

	tput_put();

	p3 = atime() - start;

	wake_up_interruptible_sync(&buffer_state[next->index].wq);
	
	p4 = atime() - start;

	if(debug_cma) {
		printk("isp post %6lld  %6lld  %6lld  %6lld  ", p1, p2, p3, p4 );
	}
}

/* This function configures and initializes the ISP resizer*/
static int init_isp_rsz(int index)
{
	int ret = 0;

	/* clear data */
	memset(&pipe, 0, sizeof(pipe));
	cma_setup_pipe_from_buffer(index, &pipe);

	ret = ispdss_configure(&pipe, NULL, NB_BUFFERS, NULL);
	if (ret) {
		printk(KERN_ERR "<%s> failed to configure "
				"ISP_resizer = %d\n",
				__func__, ret);
		return ret;
	}

	return ret;
}

static void cma_resizer_handler(struct work_struct *work) 
{
	struct cma_resizer_work *resizer_work =
		(struct cma_resizer_work *) work;
	do_resize( resizer_work->info );
	kfree(resizer_work);
}

static void start_resizer(int index) 
{
	if( sync_cma ) {
		do_resize( &buffer_state[index] );
		return;
	} else {
		struct cma_resizer_work *work = NULL;
		int queued_resizer_work = 0;

		work = kmalloc(sizeof(struct cma_resizer_work), GFP_KERNEL);
		if (!work) 
			return;
		INIT_WORK((struct work_struct *)work, cma_resizer_handler);
		work->info = &buffer_state[index];
		queued_resizer_work = queue_work(resizer_workqueue, (struct work_struct *)work);
		if (!queued_resizer_work) {
			printk(KERN_ERR "Failed to queue resizer work\n");
			kfree(work);
		}
	}
}
#endif

int get_buffer_index( u32 paddr )
{
	int index;
	for (index = 0; index < NB_BUFFERS; index++) {
		if (buffer_state[index].phys_addr == paddr)
			return index;
	}
	return -1;
}

int cma_is_buffer_ready(u32 paddr, bool wait)
{
	int index;

	if( (index = get_buffer_index( paddr )) < 0 ) {
		return -1;
	}

	if (wait) {
		return cma_waitbuf_ready(index);
	} else {
		return (buffer_state[index].state != CMABUF_READY) ? -ETIME : 0;
	}
}
EXPORT_SYMBOL(cma_is_buffer_ready);

u32 cma_set_output_buffer(u32 paddr, u32 *new_addr, int rotation, bool mirror, u16 *in_width, u16 *in_height, u16 *stride) 
{
	int index;
	int offset = 0;

	if( (index = get_buffer_index( paddr )) < 0 ) {
		return -1;
	}

	*in_width = buffer_state[index].out_width;
	*in_height = buffer_state[index].out_height;
	if (rotation%2)
		swap(*in_width, *in_height);

	if (buffer_state[index].use_vrfb == 0) {
	    //We can perhaps optimize bandwidth here
		*new_addr = buffer_state[index].phys_addr;
		buffer_state[index].state = CMABUF_READY;
		wake_up_interruptible_sync(&buffer_state[index].wq);
		return 0;
	}

	switch ((rotation + (mirror ? 2 : 0)) % 4) {
	case 0:
		offset = 0;
		break;
	case 1:
		offset = buffer_state[index].vrfb_ctx.yoffset * buffer_state[index].vrfb_ctx.bytespp;
		break;
	case 2:
		offset = (MAX_PIXELS_PER_LINE * buffer_state[index].vrfb_ctx.yoffset + buffer_state[index].vrfb_ctx.xoffset)* buffer_state[index].vrfb_ctx.bytespp;
		break;
	case 3:
		offset = (MAX_PIXELS_PER_LINE * buffer_state[index].vrfb_ctx.xoffset)* buffer_state[index].vrfb_ctx.bytespp;
		break;
	}

	*stride = 2048;
	*new_addr = buffer_state[index].vrfb_ctx.paddr[(rotation + (mirror?2:0))%4] + offset;
	if (mirror)
		*new_addr += MAX_PIXELS_PER_LINE *((rotation&1)?buffer_state[index].vrfb_ctx.xres:buffer_state[index].vrfb_ctx.yres - 1) *buffer_state[index].vrfb_ctx.bytespp;

	return 0;
}
EXPORT_SYMBOL(cma_set_output_buffer);

void callback_dma_tx(int lch, u16 ch_status, void* data) 
{
	int index = (int) data;
	omap_free_dma(lch);
	buffer_state[index].dma_ch = -1;
	buffer_state[index].state = CMABUF_READY;
	wake_up_interruptible_sync(&buffer_state[index].wq);
}

int startDmaTransfer(int index) 
{
	u32 dest_frame_index = 0, src_element_index = 0;
	u32 dest_element_index = 0, src_frame_index = 0;

	u32 elem_count = 0, frame_count = 0;

	if (omap_request_dma(OMAP_DMA_NO_DEVICE, "VRFB DMA TX", callback_dma_tx, (void*)index, &buffer_state[index].dma_ch) < 0) {
		printk(KERN_ERR "Cannot allocate dma channel for buffer %d\n", index);
	}

	/*
	 * DMA transfer in double index mode
	 */

	/* Frame index */
	dest_frame_index = ((MAX_PIXELS_PER_LINE * 4) - (buffer_state[index].out_width * 2)) + 1;

	/* Source and destination parameters */
	src_element_index = 0;
	src_frame_index = 0;
	dest_element_index = 1;
	/* Number of elements per frame */
	elem_count = buffer_state[index].out_width * 2;
	frame_count = buffer_state[index].out_height;

	omap_set_dma_transfer_params(buffer_state[index].dma_ch, OMAP_DMA_DATA_TYPE_S32, (elem_count / 4), frame_count, OMAP_DMA_SYNC_ELEMENT, OMAP_DMA_NO_DEVICE, 0x0);
	/* src_port required only for OMAP1 */
	omap_set_dma_src_params(buffer_state[index].dma_ch, 0, OMAP_DMA_AMODE_POST_INC, buffer_state[index].phys_addr, src_element_index, src_frame_index);
	/*set dma source burst mode for VRFB */
	omap_set_dma_src_burst_mode(buffer_state[index].dma_ch, OMAP_DMA_DATA_BURST_16);

	/* dest_port required only for OMAP1 */
	omap_set_dma_dest_params(buffer_state[index].dma_ch, 0, OMAP_DMA_AMODE_DOUBLE_IDX, buffer_state[index].vrfb_ctx.paddr[0], dest_element_index, dest_frame_index);
	/*set dma dest burst mode for VRFB */
	omap_set_dma_dest_burst_mode(buffer_state[index].dma_ch, OMAP_DMA_DATA_BURST_16);
	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0x20, 0);

	/*use posted write mode to increase bandwidth*/
	omap_set_dma_write_mode(buffer_state[index].dma_ch, OMAP_DMA_WRITE_POSTED);

	omap_start_dma(buffer_state[index].dma_ch);
	return 0;
}

int cma_set_buf_state(int paddr, enum cmabuf_state state, int ovl_ix)
{
	int index;
	int ret = 0;
	u64 start = atime();

	mutex_lock(&mtx);

	if( (index = get_buffer_index( paddr )) < 0 ) {
		ret = -1;
		goto quit;
	}

	buffer_state[index].state = state;
	if (state == CMABUF_READY) {
#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY
		int isp_enabled;
#endif
		if (ovl_ix < 0 || ovl_ix >= NB_OVERLAYS) {
			ret = -1;
			goto quit;
		}
		buffer_state[index].ovl_ix = ovl_ix;
		if (ovl_win[ovl_ix].width <= 0 || ovl_win[ovl_ix].height <= 0) {
			printk("cma_set_buf_state: ovl win not set\n");
			goto quit;
		}
#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY
		isp_enabled = !enable_isp_rsz(index);
		if ((!isp_enabled) && (buffer_state[index].width == ovl_win[ovl_ix].width) && (buffer_state[index].height == ovl_win[ovl_ix].height) && (buffer_state[index].width == 1280))
			buffer_state[index].height -= 2;
		if (isp_enabled) {
			buffer_state[index].out_width  = ((ovl_win[ovl_ix].width + 0x0F) & ~0x0F);
			buffer_state[index].out_height = ovl_win[ovl_ix].height * buffer_state[index].out_width / ovl_win[ovl_ix].width;

			if (buffer_state[index].out_width > 1264) {
				buffer_state[index].out_height = (buffer_state[index].out_height * 1264) /buffer_state[index].out_width;
				buffer_state[index].out_width = 1264;
			}
		} else
#endif
		{
			buffer_state[index].out_width  = buffer_state[index].width;
			buffer_state[index].out_height = buffer_state[index].height;
		}
		if ((ovl_win[ovl_ix].rotation != 0) || (isp_enabled)) {
			//Setup shadow vrfb context
			buffer_state[index].use_vrfb = 1;
			omap_vrfb_setup(&buffer_state[index].vrfb_ctx, buffer_state[index].vrfb_paddr, buffer_state[index].out_width, buffer_state[index].out_height, BPP, true, ovl_win[ovl_ix].rotation);
			if(debug_cma) printk("vrfb post %6lld  ", atime() - start);

			//change state to CMA_PREPROCESSING
			buffer_state[index].state = CMABUF_PREPROCESSING;

#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY
			if (isp_enabled) {
				start_resizer(index);
				if(debug_cma) printk("rsz post %6lld\n", atime() - start);
			} else
#endif
			{
				//initiate DMA copy
				startDmaTransfer(index);
				if(debug_cma) printk("dma post %6lld\n", atime() - start);
			}
		} else {
			if(debug_cma) printk("Optimized : no vrfb, dma or resize\n");
			buffer_state[index].use_vrfb = 0;
		}
	}
	wake_up_interruptible_sync(&buffer_state[index].wq);
quit:
	mutex_unlock(&mtx);

	return 0;
}
EXPORT_SYMBOL(cma_set_buf_state);

int cma_set_ovl_win(int ovl_ix, int width, int height, int rotation)
{
	if (ovl_ix < 0 || ovl_ix >= NB_OVERLAYS)
		return -1;

	mutex_lock(&mtx);
	ovl_win[ovl_ix].width    = width;
	ovl_win[ovl_ix].height   = height;
	ovl_win[ovl_ix].rotation = rotation;
	if (rotation%2) {
		swap(ovl_win[ovl_ix].width, ovl_win[ovl_ix].height);
	}
	mutex_unlock(&mtx);
	return 0;
}
EXPORT_SYMBOL(cma_set_ovl_win);

static int cma_reqbuf(struct cma_reqbuf *p)
{
	int r = -EINVAL;
	int i;
	mutex_lock(&mtx);
	for (i = 0; i<NB_BUFFERS; i++) {
		if ((buffer_state[i].max_size >= p->width * p->height * BPP) && (buffer_state[i].requested == false)) { //should be aligned
			buffer_state[i].info.size =  p->width * p->height * BPP;
			memcpy(&p->info, &buffer_state[i].info, sizeof(struct cma_buffer));
			mutex_unlock(&mtx);
			if (cma_waitbuf_ready(i) == -ETIME) {
				r= -EFAULT;
				return r;;
			}
			mutex_lock(&mtx);
			buffer_state[i].width      = p->width;
			buffer_state[i].height     = p->height;
			buffer_state[i].ovl_ix     = -1;
			buffer_state[i].out_width  = p->width;
			buffer_state[i].out_height = p->height;
			buffer_state[i].requested  = true;
			r = 0;
			goto done;
		}
	}
done:
	mutex_unlock(&mtx);
	return r;
}

static int cma_querybuf(struct cma_buffer *p)
{
	int r = -EINVAL;

	int index = p->index;

	mutex_lock(&mtx);

	if (index >= NB_BUFFERS)
		goto done;
	p->offset = buffer_state[index].info.offset;
	p->size = buffer_state[index].info.size;
	r = 0;
done:
	mutex_unlock(&mtx);
	return r;
}

static int cma_freebuf(u32 index)
{
	if (index >= NB_BUFFERS)
		return -EINVAL;
	mutex_lock(&mtx);
	buffer_state[index].requested = false;
	buffer_state[index].state = CMABUF_READY;
	mutex_unlock(&mtx);
	cma_unlockbuf(index);
	wake_up_interruptible_sync(&buffer_state[index].wq);
	return 0;
}

static s32 cma_release(struct inode *ip, struct file *filp)
{
	int i;
	for (i = 0; i < NB_BUFFERS; i++) {
		if (buffer_state[i].dma_ch != -1) omap_stop_dma(buffer_state[i].dma_ch);
		buffer_state[i].dma_ch = -1;
		memset(buffer_state[i].virt_addr, 0x0, CMA_BUFFER_SIZE);
		memset(buffer_state[i].vrfb_vaddr, 0x0, CMA_BUFFER_SIZE);
		cma_freebuf(i);
	}
#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY
	if (resizer_workqueue) 
		flush_workqueue(resizer_workqueue);
#endif
	ispdss_put_resource();
	return 0;
}

static s32 cma_ioctl(struct inode *ip, struct file *filp, u32 cmd, unsigned long arg)
{
	s32 ret = -EINVAL;
	switch (cmd) {
		case CMA_REQBUF:
		{
			struct cma_reqbuf p;

			if (copy_from_user(&p, (void __user *)arg, sizeof(struct cma_reqbuf)))
				return -EINVAL;
			if ((ret = cma_reqbuf(&p)))
				return ret;
			if (copy_to_user((void __user *)arg, &p, sizeof(struct cma_reqbuf)))
				return -EINVAL;
			break;
		}
		case CMA_QUERYBUF:
		{
			struct cma_buffer p;

			if (copy_from_user(&p, (void __user *)arg, sizeof(struct cma_buffer)))
				return -EINVAL;
			if ((ret = cma_querybuf(&p)))
				return ret;
			if (copy_to_user((void __user *)arg, &p, sizeof(struct cma_buffer)))
				return -EINVAL;
			break;
		}
		case CMA_FREEBUF:
		{
			u32 index;

			if (copy_from_user(&index, (void __user *)arg, sizeof(u32)))
				return -EINVAL;
			if ((ret = cma_freebuf(index)))
				return ret;
			break;
		}
		case CMA_LOCKBUF:
		{
			u32 index;

			if (copy_from_user(&index, (void __user *)arg, sizeof(u32)))
				return -EINVAL;
			if ((ret = cma_lockbuf(index, false)))
				return ret;
			break;
		}
		case CMA_UNLOCKBUF:
		{
			u32 index;

			if (copy_from_user(&index, (void __user *)arg, sizeof(u32)))
				return -EINVAL;
			if ((ret = cma_unlockbuf(index)))
				return ret;
			break;
		}
	}
	return ret;
}

static int cma_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long vma_size = PAGE_ALIGN(vma->vm_end - vma->vm_start);
	int ret = 0;
	int index;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	mutex_lock(&mtx);
	index = vma->vm_pgoff;

	if (index >= NB_BUFFERS) {
		ret = -1;
		goto done;
	}

	if (remap_pfn_range(vma, vma->vm_start,  buffer_state[index].phys_addr >> PAGE_SHIFT, vma_size, vma->vm_page_prot)) {
		ret=-EAGAIN;
		goto done;
	}

done:
	mutex_unlock(&mtx);
	return ret;
}

static const struct file_operations cma_fops = {
	.open    = cma_open,
 	.ioctl   = cma_ioctl,
	.release = cma_release,
 	.mmap    = cma_mmap,
};

static int __init cma_init(void)
{
	int r = -1;
	dev_t dev  = 0;

	mutex_init(&mtx);

	if (cma_alloc_buffers())
		goto error;
	cma_device = kmalloc(sizeof(*cma_device), GFP_KERNEL);
	if (!cma_device) {
		r = -ENOMEM;
		printk(KERN_ERR "can not allocate device structure\n");
		goto error;
	}

	memset(cma_device, 0x0, sizeof(*cma_device));
	if (cma_major) {
		dev = MKDEV(cma_major, cma_minor);
		r = register_chrdev_region(dev, 1, "cma");
	} else {
		r = alloc_chrdev_region(&dev, cma_minor, 1, "cma");
		cma_major = MAJOR(dev);
	}

	cdev_init(&cma_device->cdev, &cma_fops);
	cma_device->cdev.owner = THIS_MODULE;
	cma_device->cdev.ops   = &cma_fops;

	r = cdev_add(&cma_device->cdev, dev, 1);
	if (r)
		printk(KERN_ERR "cdev_add():failed\n");

	cmadev_class = class_create(THIS_MODULE, "cma");

	if (IS_ERR(cmadev_class)) {
		printk(KERN_ERR "class_create():failed\n");
		goto error;
	}

	device = device_create(cmadev_class, NULL, dev, NULL, "cma");
	if (device == NULL)
		printk(KERN_ERR "device_create() fail\n");

	r = platform_driver_register(&cma_driver_ldm);

#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY
	resizer_workqueue = create_singlethread_workqueue("cma_resizer_wq");
	if (!resizer_workqueue) {
		printk(KERN_ERR "Unable to create workqueue for resizer\n");
		r = -EBUSY;
	}
#endif

error:
	/* TODO: error handling for device registration */
	if (r) {
		kfree(cma_device);
	}
	return r;
}

static void __exit cma_exit(void)
{
	int i;
	for (i = 0; i < NB_BUFFERS; i++) {
		free_pages_exact(buffer_state[i].virt_addr, CMA_BUFFER_SIZE);
		/* Release associated vrfb context */
		omap_vrfb_release_ctx(&buffer_state[i].vrfb_ctx);
		free_pages_exact(buffer_state[i].vrfb_vaddr, CMA_BUFFER_SIZE);
		if (buffer_state[i].dma_ch != -1) omap_stop_dma(buffer_state[i].dma_ch);
		buffer_state[i].dma_ch = -1;
	}

	mutex_destroy(&mtx);
	platform_driver_unregister(&cma_driver_ldm);
	cdev_del(&cma_device->cdev);
	kfree(cma_device);
	device_destroy(cmadev_class, MKDEV(cma_major, cma_minor));
	class_destroy(cmadev_class);

#ifdef CONFIG_OMAP3_ISP_RESIZER_ON_OVERLAY
	if (resizer_workqueue) {
		flush_workqueue(resizer_workqueue);
		destroy_workqueue(resizer_workqueue);
	}
#endif
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Francois Caron <caron@archos.com>");
MODULE_DESCRIPTION("Contiguous Memory Allocator");
module_init(cma_init);
module_exit(cma_exit);
