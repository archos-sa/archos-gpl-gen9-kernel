/*
 *    tr16c0-i2c.h : 23/06/2011
 *    g.revaillot, revaillot@archos.com
 */

#ifndef _LINUX_TR16C0_H
#define _LINUX_TR16C0_H

#define TR16C0_NAME	"tr16c0_i2c_tsp"
#define TR16C0_ADDR		(0x5c)

struct tr16c0_platform_data {
	int irq;
	int flags;
};

#endif

