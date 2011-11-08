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

#include <media/cma.h>

#define NB_BUFFERS 	6
#define BPP	   	2
#define WIDTH		1280
#define HEIGHT		736
#define MAX_PIXELS_PER_LINE	2048

#define CMA_BUFFER_SIZE  PAGE_ALIGN(WIDTH*HEIGHT*BPP)

struct cma_dev {
	struct cdev cdev;

	struct blocking_notifier_head notifier;
};

struct client_info{
	struct list_head list;
	pid_t pid;
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
	void* virt_addr;
	void* vrfb_vaddr;
	unsigned long max_size;
	struct cma_buffer info;
	struct client_info pid;
	struct vrfb vrfb_ctx;
};

static struct cma_dev *cma_device;
static struct class *cmadev_class;
static s32 cma_major;
static s32 cma_minor;

static struct buf_info buffer_state[NB_BUFFERS];

static struct mutex mtx;

static struct platform_driver cma_driver_ldm = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "cma",
	},
};

/* get process info, and increment refs for device tracking */
static s32 add_pid_locked(int index, pid_t pid)
{
	struct client_info *pi;
	struct client_info *new_pi;
	s32 r = 0;

	/* find process context */
	list_for_each_entry(pi, &buffer_state[index].pid.list, list) {
		if (pi->pid == pid)
			goto done;
	}

	new_pi = kzalloc(sizeof(*new_pi), GFP_KERNEL);
	if (new_pi == NULL) {
		r = -ENOMEM;
		goto done;
	}
	INIT_LIST_HEAD(&new_pi->list);
	new_pi->pid = pid;

	list_add(&new_pi->list, &buffer_state[index].pid.list);

done:
	return r;
}

/* get process info, and increment refs for device tracking */
static void remove_pid_locked(int index, pid_t pid)
{
	struct client_info *pi;
	struct list_head *pos, *q;

	/* find process context */
	list_for_each_safe(pos, q, &buffer_state[index].pid.list) {
		pi= list_entry(pos, struct client_info, list);
		if (pi->pid == pid)
			list_del(pos);
	}
}

static s32 cma_open(struct inode *ip, struct file *filp)
{
	return 0;
}

/*
 * Allocate buffers
 */
static int cma_alloc_buffers(void)
{
	int i, size;

	for (i = 0; i < NB_BUFFERS; i++) {
		buffer_state[i].locked = false;
		buffer_state[i].requested = false;
		buffer_state[i].dma_ch = -1;
		buffer_state[i].state = CMABUF_FREE;
		INIT_LIST_HEAD(&buffer_state[i].pid.list);
		init_waitqueue_head(&buffer_state[i].wq);
		size = CMA_BUFFER_SIZE;
		buffer_state[i].virt_addr = alloc_pages_exact(size, GFP_KERNEL | GFP_DMA);
		if (buffer_state[i].virt_addr) {
			buffer_state[i].phys_addr = virt_to_phys(buffer_state[i].virt_addr);
			buffer_state[i].max_size = CMA_BUFFER_SIZE;
			buffer_state[i].info.offset = i << PAGE_SHIFT;
			buffer_state[i].info.index = i;
			buffer_state[i].info.size = 0;
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
			wake_up_interruptible_sync(&buffer_state[index].wq);
		}
	}
	remove_pid_locked(index, current->tgid);
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
	add_pid_locked(index, current->tgid);
	mutex_unlock(&mtx);
	return r;
}


static int cma_waitbuf_free_or_busy(int index)
{
	u32 timeout = usecs_to_jiffies(66 * 1000); /*delay before timing out */

	if ((buffer_state[index].state != CMABUF_BUSY) && (buffer_state[index].state != CMABUF_FREE)) {
		timeout = wait_event_interruptible_timeout(buffer_state[index].wq, (buffer_state[index].state==CMABUF_FREE)||(buffer_state[index].state==CMABUF_BUSY), timeout);
		if (timeout <= 0)
			return timeout ? : -ETIME;
	}

	return 0;
}

u32 cma_set_vrfb_ctx(u32 paddr, u32 *new_addr, int width, int height, int rotation, bool mirror) {
	int index = 0;
	for (index = 0; index < NB_BUFFERS; index++) {
		if (buffer_state[index].phys_addr == paddr)
			break;
	}
	if (index >= NB_BUFFERS) {
		return -1;
	}

	cma_waitbuf_free_or_busy(index);

	*new_addr = buffer_state[index].vrfb_ctx.paddr[(rotation + (mirror?2:0))%4];
	if (mirror)
		  *new_addr += MAX_PIXELS_PER_LINE *((rotation&1)?width/2:height - 1) *4;
	return 0;
}
EXPORT_SYMBOL(cma_set_vrfb_ctx);

void callback_dma_tx(int lch, u16 ch_status, void* data) {
	int index = (int) data;
	omap_free_dma(lch);
	buffer_state[index].dma_ch = -1;
	buffer_state[index].state = CMABUF_BUSY;
	wake_up_interruptible_sync(&buffer_state[index].wq);
}

int startDmaTransfer(int index) {
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
	dest_frame_index = ((MAX_PIXELS_PER_LINE * 4) - (buffer_state[index].width * 2)) + 1;

	/* Source and destination parameters */
	src_element_index = 0;
	src_frame_index = 0;
	dest_element_index = 1;
	/* Number of elements per frame */
	elem_count = buffer_state[index].width * 2;
	frame_count = buffer_state[index].height;

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

	omap_start_dma(buffer_state[index].dma_ch);
	return 0;
}

int cma_set_buf_state(int paddr, enum cmabuf_state state) {
	int index = 0;
	for (index = 0; index < NB_BUFFERS; index++) {
	    if (buffer_state[index].phys_addr == paddr)
		break;
	}
	if (index >= NB_BUFFERS) {
	  return -1;
	}

	buffer_state[index].state = state;
	if (state == CMABUF_BUSY) {
		//Setup shadow vrfb context
		omap_vrfb_setup(&buffer_state[index].vrfb_ctx, buffer_state[index].vrfb_paddr, buffer_state[index].width, buffer_state[index].height, BPP, true, 0);
		//initiate DMA copy
		startDmaTransfer(index);
		//change state to CMA_DMACOPY
		state = CMABUF_DMACOPY;
	}
	wake_up_interruptible_sync(&buffer_state[index].wq);

	return 0;
}
EXPORT_SYMBOL(cma_set_buf_state);

static int cma_reqbuf(struct cma_reqbuf *p)
{
	int r = -EINVAL;
	int i;
	mutex_lock(&mtx);
	for (i = 0; i<NB_BUFFERS; i++) {
		if ((buffer_state[i].max_size >= p->width*p->height*BPP)&&(buffer_state[i].requested == false)) { //should be aligned
			buffer_state[i].info.size =  p->width*p->height*BPP;
			memcpy(&p->info, &buffer_state[i].info, sizeof(struct cma_buffer));
			mutex_unlock(&mtx);
			if (cma_waitbuf_free_or_busy(i) == -ETIME) {
				r= -EFAULT;
				return r;;
			}
			mutex_lock(&mtx);
			buffer_state[i].width = p->width;
			buffer_state[i].height = p->height;
			buffer_state[i].requested = true;
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
	buffer_state[index].state = CMABUF_FREE;
	memset(buffer_state[index].virt_addr, 0x0, CMA_BUFFER_SIZE);
	memset(buffer_state[index].vrfb_vaddr, 0x0, CMA_BUFFER_SIZE);
	if (buffer_state[index].dma_ch != -1) omap_stop_dma(buffer_state[index].dma_ch);
	buffer_state[index].dma_ch = -1;
	mutex_unlock(&mtx);
	cma_unlockbuf(index);
	wake_up_interruptible_sync(&buffer_state[index].wq);
	return 0;
}

static s32 cma_release(struct inode *ip, struct file *filp)
{
	int i;
	for (i = 0; i < NB_BUFFERS; i++) {
		cma_freebuf(i);
		remove_pid_locked(i, current->tgid);
	}
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
	struct device *device = NULL;

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
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Francois Caron <caron@archos.com>");
MODULE_DESCRIPTION("Contiguous Memory Allocator");
module_init(cma_init);
module_exit(cma_exit);
