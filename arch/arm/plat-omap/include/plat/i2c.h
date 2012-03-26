/*
 * Helper module for board specific I2C bus registration
 *
 * Copyright (C) 2009 Nokia Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#ifndef __ASM__ARCH_OMAP_I2C_H
#define __ASM__ARCH_OMAP_I2C_H

#include <linux/i2c.h>

struct omap_i2c_bus_board_data {
	struct hwspinlock *handle;
	int (*hwspinlock_lock) (struct hwspinlock *handle);
	int (*hwspinlock_unlock) (struct hwspinlock *handle);
};

#if defined(CONFIG_I2C_OMAP) || defined(CONFIG_I2C_OMAP_MODULE)
extern int omap_register_i2c_bus(int bus_id, u32 clkrate,
				 struct omap_i2c_bus_board_data *pdata,
				 struct i2c_board_info const *info,
				 unsigned len);
#else
static inline int omap_register_i2c_bus(int bus_id, u32 clkrate,
				 struct omap_i2c_bus_board_data *pdata,
				 struct i2c_board_info const *info,
				 unsigned len)
{
	return 0;
}
#endif

/**
 * i2c_dev_attr - OMAP I2C controller device attributes for omap_hwmod
 * @fifo_depth: total controller FIFO size (in bytes)
 * @flags: differences in hardware support capability
 *
 * @fifo_depth represents what exists on the hardware, not what is
 * actually configured at runtime by the device driver.
 */
struct omap_i2c_dev_attr {
	u8	fifo_depth;
	u8	flags;
};

enum {
	I2C_PULLUP_STD_4K5_FAST_1K66 = 0,
	I2C_PULLUP_STD_2K1_FAST_920OM,
	I2C_PULLUP_STD_860_OM_FAST_500_OM,
	I2C_PULLUP_STD_NA_FAST_300_OM
};

void __init omap1_i2c_mux_pins(int bus_id);
void __init omap2_i2c_mux_pins(int bus_id);

void omap2_i2c_pullup(int bus_id, unsigned int pullup);

#endif //__ASM__ARCH_OMAP_I2C_H_
