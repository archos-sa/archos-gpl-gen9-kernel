/*
 * twl6030-vib.c : 25/02/2011
 * g.revaillot, revaillot@archos.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Derived from: twl6030-vib.c
 * Derived from: vib-gpio.c
 * Additional derivation from: twl6040-vibra.c
 */

#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c/twl.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#include "../staging/android/timed_output.h"

#include <linux/twl6030-vib.h>
#define DEBUG

struct twl6030_vibrator_data {
	struct timed_output_dev dev;
	struct work_struct work;
	struct hrtimer timer;
	struct semaphore sem;
	int state;
	int request;
	struct regulator *reg;
};

static int twl6030_setup_vibrator(struct twl6030_vibrator_data *priv)
{
	int ret;
	//Set vibrator to 2V by default
	ret=regulator_set_voltage(priv->reg, 1200*1000, 1200*1000);

	return ret;
}

static void twl6030_start_vibrator(struct twl6030_vibrator_data *priv)
{	
	int ret;
	int i;
	int request;

	down(&priv->sem);
	request=priv->request;
	if(request == priv->state) {
		up(&priv->sem);
		return;
	}
	for(i=0;i<5;++i) {
		if(request) {
			ret=regulator_enable(priv->reg);
		} else {
			ret=regulator_disable(priv->reg);
		}
		if(!ret) {
			priv->state=request;
			up(&priv->sem);
			return;
		}
		pr_err("vib: state to %d failed\n", request);
	}
	up(&priv->sem);
	pr_err("PMIC failed too much\n");
	return;
}

static int twl6030_vibrator_get_time(struct timed_output_dev *dev)
{
	struct twl6030_vibrator_data *priv=
		container_of(dev, struct twl6030_vibrator_data, dev);

	if (hrtimer_active(&priv->timer)) {
		struct timeval t;
		t = ktime_to_timeval(hrtimer_get_remaining(&priv->timer));
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	}

	return 0;
}

static void twl6030_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct twl6030_vibrator_data *priv =
		container_of(dev, struct twl6030_vibrator_data, dev);

	if (value < 0)
		return;

	if(down_timeout(&priv->sem, msecs_to_jiffies(10))) {
		//What should we do ?
		pr_err("vib: failed enable because of taken lock\n");
		return;
	}

	if (value == 0) {
		priv->request=0;
		hrtimer_cancel(&priv->timer);
		schedule_work(&priv->work);
	} else {
		ktime_t delay = ktime_set(value / 1000, (value % 1000) * 1000000);

		if (!hrtimer_active(&priv->timer)) {
			priv->request=1;
		} else {
			ktime_t prev_delay = hrtimer_get_remaining(&priv->timer);
			if (delay.tv64 < prev_delay.tv64)
				delay = prev_delay;
		}

		hrtimer_start(&priv->timer, delay, HRTIMER_MODE_REL);
		schedule_work(&priv->work);
	}

	up(&priv->sem);
}

static enum hrtimer_restart twl6030_vibrator_timer_func(struct hrtimer *timer)
{
	struct twl6030_vibrator_data *priv =
		container_of(timer, struct twl6030_vibrator_data, timer);

	priv->request=0;
	schedule_work(&priv->work);

	return HRTIMER_NORESTART;
}

static void update_vibrator(struct work_struct *work) {
	struct twl6030_vibrator_data *priv =
		container_of(work, struct twl6030_vibrator_data, work);
	twl6030_start_vibrator(priv);
}

#ifdef DEBUG
static ssize_t show_voltage(struct device* dev, 
		    struct device_attribute *attr, char* buf)
{
	int stat;
	struct timed_output_dev *tdev=dev_get_drvdata(dev);
	struct twl6030_vibrator_data *priv =
		container_of(tdev, struct twl6030_vibrator_data, dev);

	stat=regulator_get_voltage(priv->reg);
	return snprintf(buf, PAGE_SIZE, "%i\n", stat/1000);
}
static ssize_t store_voltage(struct device* dev,
		    struct device_attribute *attr, const char* buf, size_t len)
{
	int new_mV = simple_strtol(buf, NULL, 10);
	int err;
	struct timed_output_dev *tdev=dev_get_drvdata(dev);
	struct twl6030_vibrator_data *priv =
		container_of(tdev, struct twl6030_vibrator_data, dev);

	err=regulator_set_voltage(priv->reg, new_mV*1000, new_mV*1000);
	if(err) {
		pr_err("Setting regulator voltage failed: %d\n", err);
		return err;
	}
	return len;
}
static DEVICE_ATTR(voltage, S_IRUGO|S_IWUSR, show_voltage, store_voltage);
#endif

static int twl6030_vibrator_probe(struct platform_device *pdev)
{
	struct twl6030_vibrator_data *priv;
	int ret = 0;

	dev_info(&pdev->dev, "%s\n", __FUNCTION__);

	priv = kzalloc(sizeof(struct twl6030_vibrator_data), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	sema_init(&priv->sem, 1);

	platform_set_drvdata(pdev, priv);

	priv->dev.name = "vibrator";
	priv->dev.get_time = twl6030_vibrator_get_time;
	priv->dev.enable = twl6030_vibrator_enable;

	ret = timed_output_dev_register(&priv->dev);
	if (ret < 0) {
		pr_err("vib: Couldn't register timed_output device !\n");
		goto err_timed_output_dev_register;
	}

	INIT_WORK(&priv->work, update_vibrator);

	priv->reg=regulator_get(priv->dev.dev, "vaux3");
	if(IS_ERR_OR_NULL(priv->reg)) {
		ret=PTR_ERR(priv->reg);
		pr_err("vib: Couldn't get regulator !:%d\n", ret);
		goto err_get_regulator;
	}

	ret = twl6030_setup_vibrator(priv);
	if (ret < 0) {
		pr_err("vib: Couldn't setup vibrator\n");
		goto err_setup_vibrator;
	}

	hrtimer_init(&priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	priv->timer.function = twl6030_vibrator_timer_func;

#ifdef DEBUG
	ret = device_create_file(&pdev->dev, &dev_attr_voltage);
	if (ret < 0)
		dev_err(&pdev->dev, "attr fail\n");
#endif

	return 0;

err_setup_vibrator:
	regulator_put(priv->reg);

err_get_regulator:
	timed_output_dev_unregister(&priv->dev);

err_timed_output_dev_register:
	kfree(priv);


	return ret;
}

static int twl6030_vibrator_remove(struct platform_device *pdev)
{
	struct twl6030_vibrator_data *priv = platform_get_drvdata(pdev);
#ifdef DEBUG
	device_remove_file(&pdev->dev, &dev_attr_voltage);
#endif

	priv->request=0;
	twl6030_start_vibrator(priv);
	timed_output_dev_unregister(&priv->dev);
	regulator_put(priv->reg);
	kfree(priv);

	return 0;
}


static struct platform_driver twl6030_vibrator_driver = {
	.probe = twl6030_vibrator_probe,
	.remove = twl6030_vibrator_remove,
	.driver = {
		.name = TWL6030_VIBRATOR_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init twl6030_vibrator_init(void)
{
	return platform_driver_register(&twl6030_vibrator_driver);
}

static void __exit twl6030_vibrator_exit(void)
{
	platform_driver_unregister(&twl6030_vibrator_driver);
}

module_init(twl6030_vibrator_init);
module_exit(twl6030_vibrator_exit);

MODULE_AUTHOR("Guillaume Revaillot <revaillot@archos.com>");
MODULE_DESCRIPTION("TWL6030 Vibrator Driver");
MODULE_LICENSE("GPL");
