/* File: linux/arch/arm/mach-omap2/archos-usb-android.c
 *
 *  This file contains Archos platform-specific data for the Android USB
 * gadget driver.
 *
 * Copyright © 2009 Chidambar Zinnoury - Archos S.A.
 */

/*
#define DEBUG
*/

#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/ch9.h>
#include <linux/usb/f_accessory.h>

#include <mach/board-archos.h>
#include <mach/archos-dieid.h>


#define PRODUCT_ID_A80S		0x1500
#define PRODUCT_ID_A80H		0x1510
#define PRODUCT_ID_A101S	0x1520
#define PRODUCT_ID_A101H	0x1530
#define PRODUCT_ID_A101XS	0x1540
#define PRODUCT_ID_A120		0x1550

#define UMS_PRODUCT_ID		0x00
#define ADB_PRODUCT_ID		0x01
#define UMS_ADB_PRODUCT_ID	0x02
#define RNDIS_PRODUCT_ID	0x03
#define RNDIS_ADB_PRODUCT_ID	0x04
#define ACM_PRODUCT_ID		0x05
#define ACM_ADB_PRODUCT_ID	0x06
#define ACM_UMS_ADB_PRODUCT_ID	0x07
#define MTP_PRODUCT_ID		0x08
#define MTP_ADB_PRODUCT_ID 	0x09
#define MTP_UMS_ADB_PRODUCT_ID	0x0A

#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)

static char serial_number[] = "A5X-00000000-00000000-00000000-00000000";
static char product_name[32];

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_acm[] = {
	"acm",
};

static char *usb_functions_acm_adb[] = {
	"acm",
	"adb",
};

static char *usb_functions_acm_ums_adb[] = {
	"acm",
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};
static char *usb_functions_mtp_ums_adb[] = {
	"mtp",
	"usb_mass_storage",
	"adb",
};

#ifdef CONFIG_USB_ANDROID_ACCESSORY
static char *usb_functions_accessory[] = {
       "accessory",
};
static char *usb_functions_accessory_adb[] = {
       "accessory",
       "adb",
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	"accessory",
#endif
#ifdef CONFIG_USB_ANDROID_MTP
	"mtp",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id     = PRODUCT_ID_A80S + UMS_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_ums),
		.functions      = usb_functions_ums,
	},
	{
		.product_id     = PRODUCT_ID_A80S + ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_adb),
		.functions      = usb_functions_adb,
	},
	{
		.product_id     = PRODUCT_ID_A80S + UMS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_ums_adb),
		.functions      = usb_functions_ums_adb,
	},
	{
		.product_id     = PRODUCT_ID_A80S + RNDIS_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis),
		.functions      = usb_functions_rndis,
	},
	{
		.product_id     = PRODUCT_ID_A80S + RNDIS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis_adb),
		.functions      = usb_functions_rndis_adb,
	},
	{
		.product_id     = PRODUCT_ID_A80S + ACM_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_acm),
		.functions      = usb_functions_acm,
	},
	{
		.product_id     = PRODUCT_ID_A80S + ACM_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_acm_adb),
		.functions      = usb_functions_acm_adb,
	},
	{
		.product_id     = PRODUCT_ID_A80S + ACM_UMS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_acm_ums_adb),
		.functions      = usb_functions_acm_ums_adb,
	},
	{
		.product_id     = PRODUCT_ID_A80S + MTP_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp),
		.functions      = usb_functions_mtp,
	},
	{
		.product_id     = PRODUCT_ID_A80S + MTP_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_adb),
		.functions      = usb_functions_mtp_adb,
	},
	{
		.product_id     = PRODUCT_ID_A80S + MTP_UMS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_ums_adb),
		.functions      = usb_functions_mtp_ums_adb,
    },
#ifdef CONFIG_USB_ANDROID_ACCESSORY
    {
        .vendor_id      = USB_ACCESSORY_VENDOR_ID,
        .product_id     = USB_ACCESSORY_PRODUCT_ID,
        .num_functions  = ARRAY_SIZE(usb_functions_accessory),
        .functions      = usb_functions_accessory,
    },
    {
        .vendor_id      = USB_ACCESSORY_VENDOR_ID,
        .product_id     = USB_ACCESSORY_ADB_PRODUCT_ID,
        .num_functions  = ARRAY_SIZE(usb_functions_accessory_adb),
        .functions      = usb_functions_accessory_adb,
    },
#endif
};


static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id		= 0x0e79,
#ifdef CONFIG_USB_ANDROID_MTP	
	.product_id		= PRODUCT_ID_A80S + MTP_PRODUCT_ID,
#else
	.product_id		= PRODUCT_ID_A80S,
#endif
	.version		= 0x0100,
	.product_name		= product_name,
	.manufacturer_name	= "Archos",
	.serial_number		= serial_number,
	.num_products		= ARRAY_SIZE(usb_products),
	.products		= usb_products,
	.num_functions		= ARRAY_SIZE(usb_functions_all),
	.functions		= usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_ACCESSORY
static struct android_usb_platform_data accessory_pdata = {
    .vendor_id  = USB_ACCESSORY_VENDOR_ID,
};

static struct platform_device android_usb_accessory = {
    .name = "accessory",
    .id = -1,
    .dev = {
        .platform_data = &accessory_pdata,
    }
};
#endif

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.vendor			= "ARCHOS",
	.product		= product_name,
	.release		= 0,
	.nluns			= 2,
};

static struct platform_device android_usb_mass_storage= {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &usb_mass_storage_pdata,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x0e79,
	.vendorDescr	= "Archos",
	};

static struct platform_device rndis_device = {
	.name		= "rndis",
	.id		= -1,
	.dev		= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static u32 __init prod_id_hash( u32 prod_id[4] )
{
	/* we use the FNV-1A hash
	  see http://isthe.com/chongo/tech/comp/fnv
	 */
	u32 hval = 0x01000193F;
	int i, k;
	
	// compute the hash over the 128 bits (=4*8 bytes) of prod_id
	for (i=0; i<4; i++) {
		u32 tmp=prod_id[i];
		for (k=0; k<4; k++) {
			/* xor the bottom with the current octet */
			hval ^= (u32)(tmp&0xFF);
			tmp >>= 8;

			/* multiply by the 32 bit FNV magic prime mod 2^32 */
			hval += (hval<<1) + (hval<<4) + (hval<<7) + (hval<<8) + (hval<<24);
		}
	}
	return hval;
}

static int __init usb_android_init(void)
{
	const struct archos_usb_gadget_config *gadget_cfg;
	const struct archos_usb_gadget_conf *conf;
	u32 prod_id[4];
#ifdef CONFIG_USB_ANDROID_RNDIS
	int i;
	char *src;
#endif
	get_dieid(prod_id);

	gadget_cfg = omap_get_config(ARCHOS_TAG_USB_GADGET,
			struct archos_usb_gadget_config);
	if (NULL == gadget_cfg)
		return -ENODEV;
	
	conf = hwrev_ptr(gadget_cfg, hardware_rev);
	if (IS_ERR(conf)) {
		pr_err("%s: no device configuration for hardware_rev %i\n",
				__func__, hardware_rev);	
		return -ENODEV;
	}

	strlcpy(product_name, conf->product_name, sizeof(product_name));
	memset(serial_number, 0, sizeof(serial_number));
	snprintf(serial_number, sizeof(serial_number)-1,
			"%s-%08X",
			product_name,
			prod_id_hash( prod_id ));
#ifdef CONFIG_USB_ANDROID_MTP		
	android_usb_pdata.product_id = conf->product_id + MTP_PRODUCT_ID;
#else
	android_usb_pdata.product_id = conf->product_id;
#endif
	usb_products[UMS_PRODUCT_ID].product_id = conf->product_id + UMS_PRODUCT_ID;
	usb_products[ADB_PRODUCT_ID].product_id = conf->product_id + ADB_PRODUCT_ID;	
	usb_products[UMS_ADB_PRODUCT_ID].product_id = conf->product_id + UMS_ADB_PRODUCT_ID;
	usb_products[RNDIS_PRODUCT_ID].product_id = conf->product_id + RNDIS_PRODUCT_ID;
	usb_products[RNDIS_ADB_PRODUCT_ID].product_id = conf->product_id + RNDIS_ADB_PRODUCT_ID;
	usb_products[ACM_PRODUCT_ID].product_id = conf->product_id + ACM_PRODUCT_ID;
	usb_products[ACM_ADB_PRODUCT_ID].product_id = conf->product_id + ACM_ADB_PRODUCT_ID;
	usb_products[ACM_UMS_ADB_PRODUCT_ID].product_id = conf->product_id + ACM_UMS_ADB_PRODUCT_ID;
	usb_products[MTP_PRODUCT_ID].product_id = conf->product_id + MTP_PRODUCT_ID;
	usb_products[MTP_ADB_PRODUCT_ID].product_id = conf->product_id + MTP_ADB_PRODUCT_ID;
	usb_products[MTP_UMS_ADB_PRODUCT_ID].product_id = conf->product_id + MTP_UMS_ADB_PRODUCT_ID;
	
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	usb_mass_storage_pdata.nluns = conf->ums_luns;
#endif	
	
/* FIXME: to add to framework!
	if ( machine_charges_from_USB() ) {
		android_usb_pdata.bmAttributes = USB_CONFIG_ATT_ONE;
		android_usb_pdata.bMaxPower = 250;  500mA 
	} else {
		android_usb_pdata.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
		android_usb_pdata.bMaxPower	= CONFIG_USB_GADGET_VBUS_DRAW / 2;
	}
*/

#ifdef CONFIG_USB_ANDROID_RNDIS
	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	src = serial_number;
	for (i = 0; *src && i<ETH_ALEN; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	if (platform_device_register(&rndis_device) < 0) {
		pr_err("Unable to register Android RNDIS device\n");
		return -ENODEV;
	}
#endif

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE	
	pr_debug("registering Android USB mass storage device  (%s)\n", 
			android_usb_pdata.serial_number);
	if (platform_device_register(&android_usb_mass_storage) < 0) {
		pr_err("Unable to register Android USB mass storage device\n");
		return -ENODEV;
	}
#endif

#ifdef CONFIG_USB_ANDROID_ACCESSORY
	pr_debug("registering Android USB Accessory device  (%s)\n",
			android_usb_pdata.serial_number);
	if (platform_device_register(&android_usb_accessory) < 0) {
		pr_err("Unable to register Android USB Accessory device\n");
		return -ENODEV;
	}
#endif


	pr_debug("registering Android USB device (%s)\n", 
			android_usb_pdata.serial_number);
	if (platform_device_register(&android_usb_device) < 0) {
		pr_err("Unable to register Android USB device\n");
		return -ENODEV;
	}
	
	return 0;
}

late_initcall(usb_android_init);

#endif
