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
#include <linux/string.h>
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
#ifdef CONFIG_CMDLINE_A70S2
	[5] = CONFIG_CMDLINE_A70S2,
#else
	[5] = "",
#endif
#ifdef CONFIG_CMDLINE_A70H2
	[6] = CONFIG_CMDLINE_A70H2,
#else
	[6] = "",
#endif
#ifdef CONFIG_CMDLINE_LUDO
	[7] = CONFIG_CMDLINE_LUDO,
#else
	[7] = "",
#endif
};

void __init fixup_archos(struct machine_desc *desc,
		struct tag *tags, char **cmdline, struct meminfo *mi)
{
	struct tag *t = tags;

	if (machine_is_archos_a80s() || machine_is_archos_a101s() ||
	    machine_is_archos_a101xs() || machine_is_omap_4430sdp()) {
		*cmdline = command_line[0];
	} else if (machine_is_omap4_panda()) {
		*cmdline = command_line[2];
	} else if (machine_is_archos_a80h() || machine_is_archos_a101h() ||
		machine_is_archos_a120())  {
		*cmdline = command_line[3];
	} else if (machine_is_archos_a101it()) {
        	*cmdline = command_line[4];
	} else if (machine_is_archos_a70s2()) {
		*cmdline = command_line[5];
	} else if (machine_is_archos_a70h2()) {
		*cmdline = command_line[6];
	} else if (machine_is_archos_ludo()) {
		*cmdline = command_line[7];
	} else {
		printk("%s : NO COMMAND LINE FOUND!", __func__);
		return;
	}

	for (; t->hdr.size; t = tag_next(t))
		if (t->hdr.tag == ATAG_MEM) {
			unsigned int sz = t->u.mem.size;
			unsigned long st = t->u.mem.start;

			printk(KERN_INFO "%s: %dM@0x%lx from bootloader.\n",
					__func__, sz/(1024*1024), st);

			if (sz > 512 * 1024 * 1024) {
				char mem[64];
				snprintf(mem, sizeof(mem)," mem=%dM@0x%lx",
						sz/(1024*1024) - 512, st + 512*1024*1024);

				strlcat(*cmdline, mem, COMMAND_LINE_SIZE);
			}
			break;
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
