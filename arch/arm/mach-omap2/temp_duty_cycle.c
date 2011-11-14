/*
 * Module to control max opp based on temperature
 *
 * based on archos_omap4_duty_cycle.c from Eduardo Valentin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/cpufreq.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <plat/omap_device.h>
#include <plat/temperature_sensor.h>

#include <mach/board-archos.h>

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Module to control max opp duty cycle based on on-die sensor temperature");

enum archos_omap4_duty_state {
	OMAP4_DUTY_NORMAL = 0,
	OMAP4_DUTY_HEATING,
	OMAP4_DUTY_COOLING,
	OMAP4_DUTY_ENTER_COOLING,
};

/* state struct */
struct archos_omap4_duty_cycle {
	struct platform_device *pdev;
	struct device *dev;
	
	/* configuration data */
	unsigned int nitro_interval;		/* max. time we spend at OPP_NITRO */
	unsigned int cooling_interval;		/* time we spend at cooling OPP */
	unsigned int nitro_rate;		/* clock rate for OPP_NITRO */
	unsigned int cooling_rate;		/* clock rate used for cooling */
	unsigned int extra_cooling_rate;	/* clock rate used for extra cooling */
	bool	inhibit_charging;		/* during extra cooling turn off charging */
	
	struct duty_cycle_temp_table		/* table of parameters per temperature range */
		duty_cycle_temp_table[10];
	
	/* heating/cooling control */
	bool cooling_needed;			/* do we need to cool or can we stay at OPP_NITRO for longer */
	bool extra_cooling_needed;		/* do we need to cool even more */
	int heating_budget;			/* time in ms we can stay at OPP_NITRO during this interval */
	struct delayed_work work;
	struct delayed_work charge_inhibit_work;

	enum archos_omap4_duty_state state;
	struct mutex mutex_duty;
	
	struct notifier_block cpufreq_nb;
};

static int archos_omap4_compute_heat_budget( struct archos_omap4_duty_cycle *odc )
{
	int temperature = omap4430_current_on_die_temperature();
	int i;

	int heating_budget = 0;
	odc->cooling_needed = false;
	odc->extra_cooling_needed = false;
	
	for(i=0; i<10; i++) {
		if ( !odc->duty_cycle_temp_table[i].temperature ) {
			odc->cooling_needed = true;
			odc->extra_cooling_needed = true;
			break;
		}
		if (temperature < odc->duty_cycle_temp_table[i].temperature) {
			heating_budget = odc->duty_cycle_temp_table[i].heating_budget;
			odc->cooling_needed = odc->duty_cycle_temp_table[i].cooling_needed;
			break;
		}
	}

	pr_debug("temperature %d heating_budget %d cooling needed %d extra_cooling needed %d\n", 
			 temperature, heating_budget, odc->cooling_needed, odc->extra_cooling_needed);
	return heating_budget;
}

static int omap4_check_cpufreq_high( struct archos_omap4_duty_cycle *odc, int cur_freq )
{
	return ( (!odc->extra_cooling_needed && cur_freq >= odc->nitro_rate) ||
	         ( odc->extra_cooling_needed && cur_freq >= odc->cooling_rate) );
}

static void archos_omap4_duty_enter_normal(struct archos_omap4_duty_cycle *odc)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);

	pr_debug("%s enter at (%u)\n", __func__, policy->cur);
	odc->state = OMAP4_DUTY_NORMAL;

	odc->heating_budget = 0;
	if (odc->extra_cooling_needed) {
		policy->max = odc->cooling_rate;
		policy->user_policy.max = odc->cooling_rate;
	} else {
		policy->max = odc->nitro_rate;
		policy->user_policy.max = odc->nitro_rate;
	}
	cpufreq_update_policy(policy->cpu);
	schedule_delayed_work(&odc->work, msecs_to_jiffies(30000)); /* keep alive */
}

static void archos_omap4_duty_enter_cooling(struct archos_omap4_duty_cycle *odc, unsigned int next_max,
					enum archos_omap4_duty_state next_state)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);

	odc->state = next_state;
	pr_debug("%s enter at (%u) state %d next state %d\n", __func__, policy->cur, odc->state, next_state);

	policy->max = next_max;
	policy->user_policy.max = next_max;
	cpufreq_update_policy(policy->cpu);

	cancel_delayed_work(&odc->work);
	schedule_delayed_work(&odc->work, msecs_to_jiffies(odc->cooling_interval));
	if ( odc->extra_cooling_needed )
		schedule_delayed_work(&odc->charge_inhibit_work, 0);
}

static void archos_omap4_duty_enter_heating(struct archos_omap4_duty_cycle *odc)
{
	pr_debug("%s enter at ()\n", __func__);
	odc->state = OMAP4_DUTY_HEATING;

	if (!odc->cooling_needed || !odc->heating_budget) {
		odc->heating_budget = archos_omap4_compute_heat_budget(odc);
	}
	
	cancel_delayed_work(&odc->work);
	schedule_delayed_work(&odc->work, msecs_to_jiffies(odc->heating_budget));
}

static void archos_omap4_duty_wg(struct work_struct *work)
{
	struct archos_omap4_duty_cycle *odc = container_of(work, struct archos_omap4_duty_cycle, work.work);
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	
	pr_debug("archos_omap4_duty_wg %d %d\n", odc->state, policy->cur);
	mutex_lock(&odc->mutex_duty);

	switch(odc->state) {
		case OMAP4_DUTY_NORMAL:
			/* make sure we stay at sane temperature, check here... */
			if ( !archos_omap4_compute_heat_budget(odc) && 
			      odc->extra_cooling_needed )
				archos_omap4_duty_enter_cooling(odc, odc->extra_cooling_rate, OMAP4_DUTY_COOLING);
			else
				schedule_delayed_work(&odc->work, msecs_to_jiffies(30000)); /* keep alive... */
			break;
		case OMAP4_DUTY_ENTER_COOLING:
		case OMAP4_DUTY_HEATING:
			if (odc->extra_cooling_needed)
				archos_omap4_duty_enter_cooling(odc, odc->extra_cooling_rate, OMAP4_DUTY_COOLING);
			else if (odc->cooling_needed)
					archos_omap4_duty_enter_cooling(odc, odc->cooling_rate, OMAP4_DUTY_COOLING);
			else {
				struct cpufreq_policy *policy = cpufreq_cpu_get(0);
				
				if ( omap4_check_cpufreq_high(odc, policy->cur) )
					archos_omap4_duty_enter_heating(odc);
				else {
					archos_omap4_duty_enter_normal(odc);
				}
			}
			break;
		case OMAP4_DUTY_COOLING:
			if ( !odc->heating_budget ) {
				/* check if we have to keep reduced speed */
				if ( !archos_omap4_compute_heat_budget(odc)) {
					if (odc->extra_cooling_needed)
						archos_omap4_duty_enter_cooling(odc, odc->extra_cooling_rate, OMAP4_DUTY_COOLING);
					else if (odc->cooling_needed)
						archos_omap4_duty_enter_cooling(odc, odc->cooling_rate, OMAP4_DUTY_COOLING);
					break;
				}
			}
			archos_omap4_duty_enter_normal(odc);
			break;
	}
	mutex_unlock(&odc->mutex_duty);
}

extern void twl6030_inhibit_usb_charging( int stop_resume );

static void archos_omap4_charge_inhibit_wg(struct work_struct *work)
{
	struct archos_omap4_duty_cycle *odc = container_of(work, struct archos_omap4_duty_cycle, charge_inhibit_work.work);

	if (!odc->inhibit_charging)
		return;
	pr_debug("archos_omap4_charge_inhibit_wg %d extra_cooling_needed %d\n", odc->state, odc->extra_cooling_needed);

	if ( odc->extra_cooling_needed ) {
		pr_debug("inhibit charging\n");
		twl6030_inhibit_usb_charging(1);
		cancel_delayed_work(&odc->charge_inhibit_work);
		/* check again in 60s */
		schedule_delayed_work(&odc->charge_inhibit_work, msecs_to_jiffies(60000));
	} else {
		pr_debug("allow charging\n");
		twl6030_inhibit_usb_charging(0);
	}
}

static int archos_omap4_duty_frequency_change(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct cpufreq_freqs *freqs = data;
	struct archos_omap4_duty_cycle *odc = container_of(nb, struct archos_omap4_duty_cycle, cpufreq_nb);

	/* We are interested only in POSTCHANGE transactions */
	if (event != CPUFREQ_POSTCHANGE)
		goto done;

	pr_debug("%s: cpu %d freqs %u->%u state: %d\n", __func__, freqs->cpu, freqs->old,
							freqs->new, odc->state);
	switch (odc->state) {
		case OMAP4_DUTY_NORMAL:
			if (omap4_check_cpufreq_high(odc, freqs->new)) {
				mutex_lock(&odc->mutex_duty);
				archos_omap4_duty_enter_heating(odc);
				mutex_unlock(&odc->mutex_duty);
			}
			break;
		case OMAP4_DUTY_HEATING:
			if (!omap4_check_cpufreq_high(odc, freqs->new)) {
				mutex_lock(&odc->mutex_duty);
				odc->state = OMAP4_DUTY_ENTER_COOLING;
				cancel_delayed_work(&odc->work);
				schedule_delayed_work(&odc->work, 0);
				mutex_unlock(&odc->mutex_duty);
			}
			break;
		case OMAP4_DUTY_ENTER_COOLING:
		case OMAP4_DUTY_COOLING:
			break;
	}

done:
	return 0;
}

static int __init archos_omap4_duty_probe(struct platform_device *pdev)
{
	struct archos_omap4_duty_cycle	*odc;
	const struct archos_temp_duty_cycle_config *temp_dc_cfg;
	int err = 0;

	odc = kzalloc(sizeof *odc, GFP_KERNEL);
	if (!odc)
		return -ENOMEM;

	/* Data initialization */
	odc->dev = &pdev->dev;
	odc->cooling_needed = false;
	odc->state = OMAP4_DUTY_NORMAL;

	temp_dc_cfg = omap_get_config(ARCHOS_TAG_TEMP_DUTY_CYCLE,
			struct archos_temp_duty_cycle_config);
	if (NULL != temp_dc_cfg) {
		pr_debug("using values from board file\n");
		odc->nitro_interval     = temp_dc_cfg->nitro_interval;
		odc->cooling_interval   = temp_dc_cfg->cooling_interval;
		odc->nitro_rate         = temp_dc_cfg->nitro_rate;
		odc->cooling_rate       = temp_dc_cfg->cooling_rate;
		odc->extra_cooling_rate = temp_dc_cfg->extra_cooling_rate;
		odc->inhibit_charging	= temp_dc_cfg->inhibit_charging;
		memcpy( odc->duty_cycle_temp_table, temp_dc_cfg->duty_cycle_temp_table, 
				sizeof(odc->duty_cycle_temp_table));
	} else {	
		pr_warn("No archos_temp_duty_cycle_config found in board file! Using defaults\n");
	
		odc->nitro_interval = 6000;
		odc->cooling_interval = 6000; /* 2*tau */
		odc->nitro_rate = 1200000;
		odc->cooling_rate = 1008000;
		odc->extra_cooling_rate = 800000;
		odc->inhibit_charging = false;
		
		/*
		* we have the following from TI:
		*    time constant is around 3s between on-die-sensor and hot-spot or PCB,
		*      thus after about 6s, heating/cooling should be finished
		*
		* below 75 deg. we allow OPP_TNT
		* between 75 deg. and 80 deg. we allow 50%-16% of nitro_interval followed by 6s (cooling interva) at 1GHz
		* between 80 deg. and 85 deg. we disallow OPP_NITRO
		* above 85 deg. we even do not allow 1GHz
		*/

		memcpy( odc->duty_cycle_temp_table, (struct duty_cycle_temp_table[]){
			{ .temperature=75000, .heating_budget=6000,   .cooling_needed=false, },
			{ .temperature=77000, .heating_budget=6000,   .cooling_needed=true,  },
			{ .temperature=78000, .heating_budget=6000/2, .cooling_needed=true,  },
			{ .temperature=79000, .heating_budget=6000/3, .cooling_needed=true,  },
			{ .temperature=80000, .heating_budget=6000/6, .cooling_needed=true,  },
			{ .temperature=85000, .heating_budget=0,      .cooling_needed=true,  },
			{ 0 }
		}, sizeof(odc->duty_cycle_temp_table));
	}
	
	INIT_DELAYED_WORK(&odc->work, archos_omap4_duty_wg);
	INIT_DELAYED_WORK(&odc->charge_inhibit_work, archos_omap4_charge_inhibit_wg);
	
	mutex_init(&odc->mutex_duty);
	
	odc->heating_budget = 0;

	odc->cpufreq_nb.notifier_call = archos_omap4_duty_frequency_change;
	
	/* Register the cpufreq notification */
	if (cpufreq_register_notifier(&odc->cpufreq_nb,
						CPUFREQ_TRANSITION_NOTIFIER)) {
		pr_err("%s: failed to setup cpufreq_notifier\n", __func__);
		err = -EINVAL;
		goto exit;
	}
	
	/* start work queue initially */
	schedule_delayed_work(&odc->work, msecs_to_jiffies(30000));
	
	return 0;
exit:
	kfree(odc);
	return err;
}

static int __exit archos_omap4_duty_remove(struct platform_device *pdev)
{
	struct archos_omap4_duty_cycle *odc = platform_get_drvdata(pdev);

	cpufreq_unregister_notifier(&odc->cpufreq_nb,
					CPUFREQ_TRANSITION_NOTIFIER);

	cancel_delayed_work_sync(&odc->work);
	cancel_delayed_work_sync(&odc->charge_inhibit_work);

	kfree(odc);

	return 0;
}

static struct platform_device *archos_omap4_duty_device;

static struct platform_driver archos_omap4_duty_driver = {
	.remove = __exit_p(archos_omap4_duty_remove),
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "archos_omap4_duty_cycle",
	},
};

/* Module Interface */
static int __init archos_omap4_duty_module_init(void)
{
	int err = 0;

	if (!cpu_is_omap443x())
		return 0;

	archos_omap4_duty_device = platform_device_register_simple("archos_omap4_duty_cycle",
								-1, NULL, 0);
	if (IS_ERR(archos_omap4_duty_device)) {
		err = PTR_ERR(archos_omap4_duty_device);
		pr_err("Unable to register omap4 duty cycle device\n");
		goto exit;
	}

	err = platform_driver_probe(&archos_omap4_duty_driver, archos_omap4_duty_probe);
	if (err)
		goto exit_pdevice;

	return 0;

exit_pdevice:
	platform_device_unregister(archos_omap4_duty_device);
exit:
	return err;
}

static void __exit archos_omap4_duty_module_exit(void)
{
	platform_device_unregister(archos_omap4_duty_device);
	platform_driver_unregister(&archos_omap4_duty_driver);
}

module_init(archos_omap4_duty_module_init);
module_exit(archos_omap4_duty_module_exit);

