/*
 * linux/arch/arm/mach-omap2/archos-fixup.c
 *
 * Copyright (C) Archos S.A.,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/feature_list.h>

static char command_line[][COMMAND_LINE_SIZE] __initdata = {
	[0] = CONFIG_CMDLINE_DEFAULT,
#ifdef CONFIG_CMDLINE_AXYZ
	[1] = CONFIG_CMDLINE_AXYZ,
#else
	[1] = "",
#endif
#ifdef CONFIG_CMDLINE_PANDA
	[2] = CONFIG_CMDLINE_PANDA,
#else
	[2] = "",
#endif
#ifdef CONFIG_CMDLINE_HDD
	[3] = CONFIG_CMDLINE_HDD,
#else
	[3] = "",
#endif
#ifdef CONFIG_CMDLINE_G8
	[4] = CONFIG_CMDLINE_G8,
#else
	[4] = "",
#endif
	[5] = "",
	[6] = "",
};

void __init fixup_archos(struct machine_desc *desc,
		struct tag *tags, char **cmdline, struct meminfo *mi)
{
	if (machine_is_archos_a80s() || machine_is_archos_a101s() ||
	    machine_is_omap_4430sdp()) {
		*cmdline = command_line[0];
	} else if (machine_is_omap4_panda()) {
		*cmdline = command_line[2];
	} else if (machine_is_archos_a80h() || machine_is_archos_a101h() ) {
		*cmdline = command_line[3];
	} else if (machine_is_archos_a101it()) {
        	*cmdline = command_line[4];
	} else {
		printk("%s : NO COMMAND LINE FOUND!", __func__);
		return;
	}

	printk("fixup_archos: [%s]\n", *cmdline);
}

#if defined(CONFIG_SERIAL_OMAP_CONSOLE) && defined(CONFIG_FEATURE_LIST)
extern int add_preferred_console(char *name, int idx, char *options);
static int __init archos_console_init(void)
{
	struct feature_tag_serial_port *serial_port = get_feature_tag(FTAG_SERIAL_PORT,
				feature_tag_size(feature_tag_serial_port));
	if (serial_port) {
		static char options[32];
		snprintf(options, 32, "%dn8", serial_port->speed);
		printk("%s : serial port UART%d, %s\n",__FUNCTION__, serial_port->uart_id, options);
#ifdef CONFIG_ARCHOS_FORCE_CONSOLE_UART3
		serial_port->uart_id=3;
#endif
		return add_preferred_console("ttyO", serial_port->uart_id-1, options);
	}
	
	return 0;
}
console_initcall(archos_console_init);
#endif
