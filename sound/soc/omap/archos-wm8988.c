/*
 * archos-wm8988.c  --  SoC audio for Gen8/9 Archos Board based on WM8988
 *
 * Author: Jean-Christophe Rona <rona@archos.com>
 *
 *    Based on archos.c from Gen8
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

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/switch.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/wm8988.h"

#include <plat/archos-audio-wm8988.h>
#include <plat/clock.h>

/* Define this if you want to enable DAPM (Dynamic Power Management) */
#define DYNAMIC_POWER

static struct audio_wm8988_device_config *pt_audio_device_io = NULL;

static int mclk;

static int wm8988_suspended = 0;

static int archos_spk_func = 0;
static int archos_recv_func = 0;
static int archos_hp_func = 0;
static int archos_mic_func = 0;

static void _enable_master_clock(void) {
	if (pt_audio_device_io->set_codec_master_clk_state)
		pt_audio_device_io->set_codec_master_clk_state(1);
}

static void _disable_master_clock(void) {
	if (pt_audio_device_io->set_codec_master_clk_state)
		pt_audio_device_io->set_codec_master_clk_state(0);
}

static int _get_master_clock_rate(void) {
	if (pt_audio_device_io->get_master_clock_rate)
		return pt_audio_device_io->get_master_clock_rate();
	return 0;
}

static inline void _enable_speaker_ampli(void) {
	if (pt_audio_device_io->set_speaker_state)
		pt_audio_device_io->set_speaker_state(1);
}

static inline void _disable_speaker_ampli(void) {
	if (pt_audio_device_io->set_speaker_state)
		pt_audio_device_io->set_speaker_state(0);
}

static int _get_headphone_state(void) {
	if (pt_audio_device_io->get_headphone_plugged)
		return pt_audio_device_io->get_headphone_plugged();
	return 0;
}

static int _get_headphone_irq(void) {
	if (pt_audio_device_io->get_headphone_irq)
		return pt_audio_device_io->get_headphone_irq();
	return -1;
}

static int _suspend_io(void) 
{
	if (pt_audio_device_io->suspend)
		pt_audio_device_io->suspend();
	return 0;
}

static int _resume_io(void)
{
	if (pt_audio_device_io->resume)
		pt_audio_device_io->resume();
	return 0;
}

/* Headphone plug detection stuff ------------------------------------------ */

#define HEAPHONE_SW_NAME	"headphone_switch"

struct work_struct headphone_work;

static ssize_t print_headphone_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", HEAPHONE_SW_NAME);
}

static ssize_t print_headphone_state(struct switch_dev *sdev, char *buf)
{
	ssize_t buflen=0;

	switch(sdev->state)
	{
		case 0:
			buflen = sprintf(buf, "Not plugged\n");
			break;
		case 1:
			buflen = sprintf(buf, "Plugged\n");
			break;
		default:
			buflen = sprintf(buf, "Unknown\n");
			break;
	}
	return buflen;
}

static struct switch_dev headphone_sw_dev = {
	.name = HEAPHONE_SW_NAME,
	.print_name  = print_headphone_name,
	.print_state = print_headphone_state,
};

static irqreturn_t headphone_plug_isr(int irq, void *dev_id)
{
	schedule_work(&headphone_work);

	return IRQ_HANDLED;
}

static void headphone_plug_work_func(struct work_struct *work)
{
	int headphone_current_state;

	headphone_current_state = _get_headphone_state();
	//printk("HP IRQ Handler : %d\n", headphone_current_state);
	switch_set_state(&headphone_sw_dev, headphone_current_state);
}

static int headphone_plug_register(void)
{
	int irq, err;
	int headphone_current_state;

	if ( (irq = _get_headphone_irq() ) != -1) {
		err = request_irq(irq, headphone_plug_isr, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
					"audio-headphone-int", &pt_audio_device_io);
		if(err < 0)
			return err;

		INIT_WORK(&headphone_work, headphone_plug_work_func);

		err = switch_dev_register(&headphone_sw_dev);
		if(err < 0)
			return err;

		headphone_current_state = _get_headphone_state();
		switch_set_state(&headphone_sw_dev, headphone_current_state);
	}
	return 0;
}

static int headphone_plug_unregister(void)
{
	int irq;

	if ( (irq = _get_headphone_irq() ) != -1) {
		free_irq(irq, &pt_audio_device_io);
		switch_dev_unregister(&headphone_sw_dev);
	}
	return 0;
}

/* ------------------------------------------------------------------------- */

#ifdef DYNAMIC_POWER
/* Blocs will be powered dynamically, input routes could be set
	using WM8988 DAPM controls but we will activate them here to to be sure */
static void archos_ext_control(struct snd_soc_codec *codec)
{
	/* First Mic */
	if (archos_mic_func)
		snd_soc_dapm_enable_pin(codec->dapm, "Mic");
	else
		snd_soc_dapm_disable_pin(codec->dapm, "Mic");

	/* Finally, Speaker, Receiver and Headphone*/
	if (archos_spk_func)
		snd_soc_dapm_enable_pin(codec->dapm, "Speaker");
	else
		snd_soc_dapm_disable_pin(codec->dapm, "Speaker");

	if (archos_recv_func)
		snd_soc_dapm_enable_pin(codec->dapm, "Receiver");
	else
		snd_soc_dapm_disable_pin(codec->dapm, "Receiver");

	if (archos_hp_func)
		snd_soc_dapm_enable_pin(codec->dapm, "Headphone");
	else
		snd_soc_dapm_disable_pin(codec->dapm, "Headphone");

	snd_soc_dapm_sync(codec->dapm);
}
#else /* DYNAMIC_POWER */
/* Blocs have to be powered manually, intput routes have to be set too */
static void archos_ext_control(struct snd_soc_codec *codec)
{
	/* First Mic */
	if (archos_mic_func)
		wm8988_power_up_inputs(codec, INP_BLOC_PGA, ON);
	else
		wm8988_power_up_inputs(codec, INP_BLOC_PGA, OFF);

	/* Finally, Speaker, Receiver and Headphone*/
	if (archos_spk_func) {
		wm8988_select_outputs(codec, OUTPUT_SPK, ON);
		_enable_speaker_ampli();
	} else if (archos_recv_func) 
		wm8988_select_outputs(codec, OUTPUT_SPK, ON);
	} else {
		_disable_speaker_ampli();
		wm8988_select_outputs(codec, OUTPUT_SPK, OFF);
	}

	if (archos_hp_func)
		wm8988_select_outputs(codec, OUTPUT_HP, ON);
	else
		wm8988_select_outputs(codec, OUTPUT_HP, OFF);
}
#endif /* DYNAMIC_POWER */

/* Archos board <--> WM8988 ops -------------------------------------------- */

static int archos_wm8988_startup(struct snd_pcm_substream *substream)
{
//	struct snd_pcm_runtime *runtime = substream->runtime;
//	struct snd_soc_pcm_runtime *rtd = substream->private_data;
//	struct snd_soc_codec *codec = rtd->socdev->codec;


//	return clk_enable(sys_clkout2);
	return 0;
}

static void archos_wm8988_shutdown(struct snd_pcm_substream *substream)
{
//	clk_disable(sys_clkout2);
}

static int archos_wm8988_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/*
	 * Userspace can try to set HW params too early (resume work not completed),
	 * leading to "External abort" if we do not check the resume status.
	 */
	if (wm8988_suspended)
		return -EBUSY;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_DSP_A |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI fmt configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_DSP_A_1PHASE |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);

	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set McBSP clock to external */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_EXT, 0,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops archos_wm8988_ops = {
	.startup = archos_wm8988_startup,
	.hw_params = archos_wm8988_hw_params,
	.shutdown = archos_wm8988_shutdown,
};

/* ------------------------------------------------------------------------- */

/* Archos board <--> FM/BT ops --------------------------------------------- */

static int archos_wl1271_startup(struct snd_pcm_substream *substream)
{
//	struct snd_pcm_runtime *runtime = substream->runtime;
//	struct snd_soc_pcm_runtime *rtd = substream->private_data;
//	struct snd_soc_codec *codec = rtd->socdev->codec;

	return 0;
}

static void archos_wl1271_shutdown(struct snd_pcm_substream *substream)
{
}

static int archos_wl1271_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	//printk(KERN_INFO "archos_wl1271_hw_params : freq = %d, channels = %d\n", params_rate(params), params_channels(params));
	
	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_IB_IF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI fmt configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_IB_IF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}
#if 0	/* Needed if the OMAP is Master */
	/* Set MCBSP sysclock to functional (96 Mhz) */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK, 0,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err(KERN_ERR "can't set cpu system clock\n");
		return ret;
	}

	/* Set Clock Divisor to 40 to output 2,4 Mhz */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_MCBSP_CLKGDV, 40);
	if (ret < 0) {
		pr_err(KERN_ERR "can't set SRG clock divider\n");
		return ret;
	}
#endif
	return 0;
}

static struct snd_soc_ops archos_wl1271_ops = {
	.startup = archos_wl1271_startup,
	.hw_params = archos_wl1271_hw_params,
	.shutdown = archos_wl1271_shutdown,
};

/* ------------------------------------------------------------------------- */

static int archos_get_spk(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = archos_spk_func;

	return 0;
}

static int archos_set_spk(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (archos_spk_func == ucontrol->value.integer.value[0])
		return 0;

	archos_spk_func = ucontrol->value.integer.value[0];
	archos_ext_control(codec);

	return 1;
}

static int archos_get_recv(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = archos_recv_func;

	return 0;
}

static int archos_set_recv(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (archos_recv_func == ucontrol->value.integer.value[0])
		return 0;

	archos_recv_func = ucontrol->value.integer.value[0];
	archos_ext_control(codec);

	return 1;
}

static int archos_get_hp(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = archos_hp_func;

	return 0;
}

static int archos_set_hp(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (archos_hp_func == ucontrol->value.integer.value[0])
		return 0;

	archos_hp_func = ucontrol->value.integer.value[0];
	archos_ext_control(codec);

	return 1;
}

static int archos_get_mic(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = archos_mic_func;

	return 0;
}

static int archos_set_mic(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (archos_mic_func == ucontrol->value.integer.value[0])
		return 0;

	archos_mic_func = ucontrol->value.integer.value[0];
	archos_ext_control(codec);

	return 1;
}

#ifdef DYNAMIC_POWER
static int speaker_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		_enable_speaker_ampli();
	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		_disable_speaker_ampli();
	}

	return 0;
}

static const struct snd_soc_dapm_widget board_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Speaker", speaker_event),
	SND_SOC_DAPM_SPK("Receiver", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"Headphone", NULL, "LOUT1"},
	{"Headphone", NULL, "ROUT1"},

	{"Speaker", NULL, "LOUT2"},
	{"Speaker", NULL, "ROUT2"},

	{"Receiver", NULL, "LOUT2"},
	{"Receiver", NULL, "ROUT2"},

	{"LINPUT1", NULL, "Mic"},
	{"RINPUT1", NULL, "Mic"},
};
#endif /* DYNAMIC_POWER */

static const char *spk_function[] = {"Off", "On"};
static const char *recv_function[] = {"Off", "On"};
static const char *hp_function[] = {"Off", "On"};
static const char *mic_function[] = {"Off", "On"};
static const struct soc_enum archos_controls_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spk_function), spk_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(recv_function), recv_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hp_function), hp_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mic_function), mic_function),
};

static const struct snd_kcontrol_new board_controls[] = {
	SOC_ENUM_EXT("Speaker Function Playback Switch", archos_controls_enum[0],
		     archos_get_spk, archos_set_spk),
	SOC_ENUM_EXT("Receiver Function Playback Switch", archos_controls_enum[1],
		     archos_get_recv, archos_set_recv),
	SOC_ENUM_EXT("Headphone Function Playback Switch", archos_controls_enum[2],
		     archos_get_hp, archos_set_hp),
	SOC_ENUM_EXT("Mic Function Capture Switch",  archos_controls_enum[3],
		     archos_get_mic, archos_set_mic),
};

static int archos_wm8988_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int i, err;

	/* The codec needs to know the MCLK (Sample Rate for DAC and ADC are set by the codec) */
	err = snd_soc_dai_set_sysclk(rtd->codec_dai, WM8988_SYSCLK, mclk,
				     SND_SOC_CLOCK_IN);
	if (err < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return err;
	}

	/* Add archos G8 specific controls */
	for (i = 0; i < ARRAY_SIZE(board_controls); i++) {
		err = snd_ctl_add(codec->snd_card,
			snd_soc_cnew(&board_controls[i], codec, NULL));
		if (err < 0)
			return err;
	}

#ifdef DYNAMIC_POWER
	/* Not used (so it won't trigger any power up) */
	snd_soc_dapm_nc_pin(codec->dapm, "LINPUT2");
	snd_soc_dapm_nc_pin(codec->dapm, "RINPUT2");

	/* Add archos G7 specific widgets */
	snd_soc_dapm_new_controls(codec->dapm, board_dapm_widgets,
				  ARRAY_SIZE(board_dapm_widgets));

	/* Set up archos specific audio path audio_map */
	snd_soc_dapm_add_routes(codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec->dapm);
#endif /* DYNAMIC_POWER */

	/* Sync stuff with the external controls config */
	archos_ext_control(codec);

	return 0;
}

static int archos_wl1271_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

#ifdef CONFIG_PM
int archos_wm8988_suspend_pre(struct platform_device *pdev, pm_message_t state)
{
	wm8988_suspended = 1;
#ifndef DYNAMIC_POWER
	if (archos_spk_func) {
		msleep(100);
		_disable_speaker_ampli();
	}
#endif /* DYNAMIC_POWER */
	return 0;
}

int archos_wm8988_suspend_post(struct platform_device *pdev, pm_message_t state)
{
	_disable_master_clock();
	_suspend_io();
	return 0;
}

int archos_wm8988_resume_pre(struct platform_device *pdev)
{
	_resume_io();
	_enable_master_clock();
	return 0;
}

int archos_wm8988_resume_post(struct platform_device *pdev)
{
	int headphone_current_state;

#ifndef DYNAMIC_POWER
	if (archos_spk_func) {
		msleep(100);
		_enable_speaker_ampli();
	}
#endif /* DYNAMIC_POWER */

	if (_get_headphone_irq() != -1) {
		headphone_current_state = _get_headphone_state();
		switch_set_state(&headphone_sw_dev, headphone_current_state);
	}

	wm8988_suspended = 0;

	return 0;
}

int archos_wl1271_suspend_pre(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

int archos_wl1271_suspend_post(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

int archos_wl1271_resume_pre(struct platform_device *pdev)
{
	return 0;
}

int archos_wl1271_resume_post(struct platform_device *pdev)
{
	return 0;
}

#else

#define archos_wm8988_suspend_pre NULL
#define archos_wm8988_suspend_post NULL
#define archos_wm8988_resume_pre NULL
#define archos_wm8988_resume_post NULL
#define archos_wl1271_suspend_pre NULL
#define archos_wl1271_suspend_post NULL
#define archos_wl1271_resume_pre NULL
#define archos_wl1271_resume_post NULL

#endif

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link archos_dai[] = {
	{
	.name = "Wolfson",
	.stream_name = "WM8988",
	.cpu_dai_name = "omap-mcbsp-dai.1",	// McBSP2
	.platform_name = "omap-pcm-audio",
	.codec_dai_name = "wm8988-hifi",
	.codec_name = "wm8988-codec.2-001a",
	.init = archos_wm8988_init,
	.ops = &archos_wm8988_ops,
	},{
	.name = "FM/BT",
	.stream_name = "WL1271",
	.cpu_dai_name = "omap-mcbsp-dai.2",	// McBSP3
	.platform_name = "omap-pcm-audio",
	.codec_dai_name = "wl1271-hifi",
	.no_codec = 1,
	.init = archos_wl1271_init,
	.ops = &archos_wl1271_ops,
	}
};

/* Audio machine driver (WM8988) */
static struct snd_soc_card snd_soc_archos_wm8988 = {
	.name = "Wolfson",
	.long_name = "Wolfson",
	.owner = THIS_MODULE,
	.dai_link = &archos_dai[0],
	.num_links = 1,
	.suspend_pre = &archos_wm8988_suspend_pre,
	.suspend_post = &archos_wm8988_suspend_post,
	.resume_pre = &archos_wm8988_resume_pre,
	.resume_post = &archos_wm8988_resume_post,
};

/* Audio machine driver (FM/BT) */
static struct snd_soc_card snd_soc_archos_wl1271 = {
	.name = "FM/BT",
	.long_name = "FM/BT",
	.owner = THIS_MODULE,
	.dai_link = &archos_dai[1],
	.num_links = 1,
	.suspend_pre = &archos_wl1271_suspend_pre,
	.suspend_post = &archos_wl1271_suspend_post,
	.resume_pre = &archos_wl1271_resume_pre,
	.resume_post = &archos_wl1271_resume_post,
};

static struct platform_device *archos_snd_device_wm8988;
static struct platform_device *archos_snd_device_wl1271;

static int __devinit archos_soc_probe(struct platform_device *pdev)
{
	int ret;

	pr_info("Archos Device SoC init\n");

	/* Master clock has to be enable to feed the WM8988 */
	pt_audio_device_io = archos_audio_wm8988_get_io();
	_enable_master_clock();

	mclk = _get_master_clock_rate();

	printk(KERN_DEBUG "Archos ASoC: master clk is: %d\n", mclk);

	if(headphone_plug_register() < 0) {
		printk(KERN_ERR "archos_soc_init: Error registering Headphone device\n");	
		ret = -ENODEV;
		goto err2;
	}

	archos_snd_device_wm8988 = platform_device_alloc("soc-audio", -1);
	if (!archos_snd_device_wm8988) {
		printk(KERN_ERR "archos_soc_init: Platform device allocation failed for WM8988\n");
		ret = -ENOMEM;
		goto err3;
	}
	archos_snd_device_wl1271 = platform_device_alloc("soc-audio", -2);
	if (!archos_snd_device_wl1271) {
		printk(KERN_ERR "archos_soc_init: Platform device allocation failed for FM/BT\n");
		ret = -ENOMEM;
		goto err4;
	}

	platform_set_drvdata(archos_snd_device_wm8988, &snd_soc_archos_wm8988);
	platform_set_drvdata(archos_snd_device_wl1271, &snd_soc_archos_wl1271);

	ret = platform_device_add(archos_snd_device_wm8988);
	if (ret)
		goto err6;
	ret = platform_device_add(archos_snd_device_wl1271);
	if (ret)
		goto err7;

	msleep(2);

	return 0;

err7:
	platform_device_unregister(archos_snd_device_wm8988);
err6:
	platform_device_put(archos_snd_device_wl1271);
err4:
	platform_device_put(archos_snd_device_wm8988);
err3:
	headphone_plug_unregister();
err2:
	_disable_master_clock();

	return ret;
}

static int __devexit archos_soc_remove(struct platform_device *pdev)
{
	headphone_plug_unregister();
	platform_device_unregister(archos_snd_device_wm8988);
	platform_device_unregister(archos_snd_device_wl1271);
	_disable_master_clock();
	return 0;
}

static struct platform_driver wm8988_audio_soc_driver = {
	.driver = {
		.name = "archos-wm8988-asoc",
		.owner = THIS_MODULE,
	},
	.probe = archos_soc_probe,
	.remove = __devexit_p(archos_soc_remove),
};

static int __init archos_soc_init(void)
{
	return platform_driver_register(&wm8988_audio_soc_driver);
}
module_init(archos_soc_init);

static void __exit archos_soc_exit(void)
{
	platform_driver_unregister(&wm8988_audio_soc_driver);
}
module_exit(archos_soc_exit);

MODULE_AUTHOR("Jean-Christophe Rona <rona@archos.com>");
MODULE_DESCRIPTION("ALSA SoC Archos (WM8988) Device");
MODULE_LICENSE("GPL");
