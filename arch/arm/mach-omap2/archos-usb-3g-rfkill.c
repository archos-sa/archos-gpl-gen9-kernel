#include <linux/init.h>
#include <linux/rfkill.h>
#include <mach/board-archos.h>
#include <linux/platform_device.h>

#include "mux.h"

#define DEVICE_NAME	"archos_usb_3g"

#define USB_OFF_WHEN_SUSPENDED

static struct archos_3g_rfkill_device {
	struct platform_device *pdev;
	struct rfkill *rfkill;
	int gpio_pwron;
	bool state;
} archos_3g_rfkill;

static inline void archos_usb_3g_rfkill_enable(struct archos_3g_rfkill_device *rfkill_dev, bool en)
{
	if (rfkill_dev->gpio_pwron > 0)
		gpio_set_value(rfkill_dev->gpio_pwron, en);

	rfkill_dev->state = en;
}

static int archos_usb_3g_rfkill_set_block(void *data, bool blocked)
{
	struct archos_3g_rfkill_device *rfkill_dev = data;
	bool enabled = !blocked;
	int ret = 0;

	dev_dbg(&rfkill_dev->pdev->dev, "%s, enabled: %d\n", __func__, enabled);

	archos_usb_3g_rfkill_enable(rfkill_dev, enabled);

	return ret;
}

static void archos_usb_3g_rfkill_query(struct rfkill *rfkill, void *data)
{
	struct archos_3g_rfkill_device *rfkill_dev = data;

	dev_dbg(&rfkill_dev->pdev->dev, "%s, enabled: %d\n", __func__, rfkill_dev->state);

	rfkill_set_sw_state(rfkill, !rfkill_dev->state);
}

static const struct rfkill_ops archos_usb_3g_rfkill_ops = {
	.set_block = archos_usb_3g_rfkill_set_block,
	.query = archos_usb_3g_rfkill_query,
};

#ifdef CONFIG_PM
static int archos_usb_3g_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct archos_3g_rfkill_device *rfkill_dev = &archos_3g_rfkill;

#ifdef USB_OFF_WHEN_SUSPENDED
	gpio_set_value(rfkill_dev->gpio_pwron, false);
#else
	if (rfkill_dev->state && rfkill_dev->gpio_pwron > 0)
		gpio_set_value(rfkill_dev->gpio_pwron, true);
#endif

	return 0;
}

/* implemented in usb-ehci.c */
extern int is_ehci_port0_connected(void);
static int archos_usb_3g_resume(struct platform_device *pdev)
{
	struct archos_3g_rfkill_device *rfkill_dev = &archos_3g_rfkill;
	bool dongle_connected = is_ehci_port0_connected();

#ifdef USB_OFF_WHEN_SUSPENDED
	if (rfkill_dev->state && rfkill_dev->gpio_pwron > 0)
		gpio_set_value(rfkill_dev->gpio_pwron, true);
#else
	if (rfkill_dev->state && !dongle_connected && rfkill_dev->gpio_pwron > 0)
		gpio_set_value(rfkill_dev->gpio_pwron, false);
#endif

	return 0;
}
#endif

static struct platform_driver archos_usb_3g_driver = {
#ifdef CONFIG_PM
	.suspend	= archos_usb_3g_suspend,
	.resume		= archos_usb_3g_resume,
#endif
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
	}
};

int __init archos_usb_3g_rfkill_init(void)
{
	struct archos_3g_rfkill_device *rfkill_dev = &archos_3g_rfkill;
	const struct archos_usb_config * usb_cfg;
	const struct archos_usb_conf * cfg;
	struct platform_device *pdev;
	int ret;
	
	usb_cfg = omap_get_config( ARCHOS_TAG_USB, struct archos_usb_config );
	if (!usb_cfg)
		return -ENODEV;
	
	cfg = hwrev_ptr(usb_cfg, hardware_rev);
	if (IS_ERR(cfg)) {
		pr_err("%s: no device configuration for hardware_rev %i\n",
				DEVICE_NAME, hardware_rev);	
		return -ENODEV;
	}

	ret = platform_driver_register(&archos_usb_3g_driver);
	if (ret)
		return ret;		
	pdev = platform_device_alloc(DEVICE_NAME, -1);
	if (!pdev) {
		ret = -ENOMEM;
		goto initfail1;
	}
	ret = platform_device_add(pdev);
	if (ret)
		goto initfail2;

	rfkill_dev->pdev = pdev;

	rfkill_dev->rfkill = rfkill_alloc("archos_usb_wwan",
					&pdev->dev,
					RFKILL_TYPE_WWAN,
					&archos_usb_3g_rfkill_ops, (void *)rfkill_dev);

	if (!rfkill_dev->rfkill) {
		dev_err(&pdev->dev, "%s - Out of memory\n", __func__);
		goto initfail3;
	}

	if ((rfkill_dev->gpio_pwron = cfg->enable_5v) > 0) {
		ret = gpio_request(rfkill_dev->gpio_pwron, "L3G_PWRON");
		if (ret) {
			dev_err(&pdev->dev, "Cannot request GPIO %d\n", rfkill_dev->gpio_pwron);
			goto initfail4;
		}
		omap_mux_init_gpio(rfkill_dev->gpio_pwron, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);

		gpio_export(rfkill_dev->gpio_pwron, false);
		gpio_direction_output(rfkill_dev->gpio_pwron, 0);
	}

	if (rfkill_register(rfkill_dev->rfkill) < 0) {
		dev_err(&pdev->dev, "Failed to register rfkill\n");
		goto initfail5;
	}

	archos_usb_3g_rfkill_enable(rfkill_dev, true);
	
	return 0;

//initfail6:
//	rfkill_unregister(rfkill_dev->rfkill);
initfail5:
	gpio_free(rfkill_dev->gpio_pwron);
initfail4:
	rfkill_destroy(rfkill_dev->rfkill);
initfail3:
	platform_device_del(pdev);
initfail2:
	platform_device_put(pdev);
initfail1:
	platform_driver_unregister(&archos_usb_3g_driver);
	return ret;
}

device_initcall(archos_usb_3g_rfkill_init);
