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

#ifndef CMA_H
#define CMA_H

struct cma_buffer {
	/* set by userspace */
	__u32 index;
	/* set by kernel */
	__u32 size;
	__u32 offset;
};

struct cma_reqbuf {
	/* set by userspace */
	__u32 width;
	__u32 height;
	/* set by kernel */
	struct cma_buffer info;
};

#define CMA_REQBUF  _IOWR('z', 100, struct cma_reqbuf)
#define CMA_QUERYBUF  _IOWR('z', 101, struct cma_buffer)
#define CMA_FREEBUF  _IOWR('z', 102, __u32)
#define CMA_LOCKBUF   _IOWR('z', 103, __u32)
#define CMA_UNLOCKBUF   _IOWR('z', 104, __u32)

extern int cma_lock_buffer(int index);
extern int cma_unlock_buffer(int index);

enum cmabuf_state {
	CMABUF_FREE = 0,
	CMABUF_BUSY = 1,
	CMABUF_PREPROCESSING = 2
};

extern u32 cma_set_output_buffer(u32 paddr, u32* new_addr, int rotation,  bool mirror, u16 *in_width, u16 *in_height);
extern int cma_set_buf_state(int ba, enum cmabuf_state state, int ovl_ix);
extern int cma_set_ovl_win(int ovl_ix, int width, int height, int rotation);

#endif
