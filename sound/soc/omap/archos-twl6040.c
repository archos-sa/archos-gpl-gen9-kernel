/*
 * archos-twl6040.c  --  SoC audio for Gen9 Archos Board based on TWL6040
 *
 * Author: Misael Lopez Cruz <x0052729@ti.com>
 *         Jean-Christophe Rona <rona@archos.com>
 *
 *    Based on sdp4430.c
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
#include <linux/i2c.h>
#include <linux/mfd/twl6040-codec.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>

#include <asm/mach-types.h>
#include <plat/mcbsp.h>
#include <plat/hardware.h>
#include <plat/mux.h>
#include <plat/archos-audio-twl6040.h>

#include "omap-mcpdm.h"
#include "omap-abe.h"
#include "omap-pcm.h"
#include "omap-mcbsp.h"
#include "omap-dmic.h"
#include "../codecs/twl6040.h"

#ifdef CONFIG_SND_OMAP_SOC_HDMI
#include "omap-hdmi.h"
#endif

static struct audio_twl6040_device_config *pt_audio_device_io = NULL;

static int twl6040_power_mode;
static int mcbsp_cfg;
static int mclock_en = 0;

static void _enable_master_clock(void) {
	if (pt_audio_device_io->set_codec_master_clk_state && !mclock_en)
		pt_audio_device_io->set_codec_master_clk_state(1);
}

static void _disable_master_clock(void) {
	if (pt_audio_device_io->set_codec_master_clk_state&& mclock_en)
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

static int archos_omap4_modem_mcbsp_configure(struct snd_pcm_substream *substream,
	int flag)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_substream *modem_substream[2];
	struct snd_soc_pcm_runtime *modem_rtd;

	if (flag) {
		if (!mcbsp_cfg) {
			modem_substream[substream->stream] =
				snd_soc_get_dai_substream(rtd->card,
							OMAP_ABE_BE_MM_EXT1,
							substream->stream);
			if (unlikely(modem_substream[substream->stream] == NULL))
				return -ENODEV;

			modem_rtd =
				modem_substream[substream->stream]->private_data;

			/* Set cpu DAI configuration */
			ret = snd_soc_dai_set_fmt(modem_rtd->cpu_dai,
					  SND_SOC_DAIFMT_I2S |
					  SND_SOC_DAIFMT_NB_NF |
					  SND_SOC_DAIFMT_CBM_CFM);

			if (unlikely(ret < 0)) {
				printk(KERN_ERR "can't set Modem cpu DAI configuration\n");
				goto exit;
			} else {
				mcbsp_cfg = 1;
			}
		}
	} else {
		mcbsp_cfg = 0;
	}

exit:
	return ret;
}

static int archos_omap4_mcpdm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int clk_id, freq;
	int ret = 0;

	switch (twl6040_power_mode) {
		case 1:
			clk_id = TWL6040_SYSCLK_SEL_HPPLL;
			freq = 19200000;
			_enable_master_clock();
			/* set the codec mclk */
			ret = snd_soc_dai_set_sysclk(codec_dai, clk_id, freq,
						SND_SOC_CLOCK_IN);
			break;
		case 2:
			/* 
			 * The clock used will be the 32KHz one, but
			 * the DAC will work in High-Performance mode.
			 */
			clk_id = TWL6040_SYSCLK_SEL_HPDAC_LPPLL;
			freq = 32768;
			/* set the codec mclk */
			ret = snd_soc_dai_set_sysclk(codec_dai, clk_id, freq,
						SND_SOC_CLOCK_IN);
			_disable_master_clock();
			break;
		case 0:
			clk_id = TWL6040_SYSCLK_SEL_LPPLL;
			freq = 32768;
			/* set the codec mclk */
			ret = snd_soc_dai_set_sysclk(codec_dai, clk_id, freq,
						SND_SOC_CLOCK_IN);
			_disable_master_clock();
			break;
	}

	if (ret) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	if (rtd->current_fe == ABE_FRONTEND_DAI_MODEM) {
		/* set Modem McBSP configuration  */
		ret = archos_omap4_modem_mcbsp_configure(substream, 1);
	}

	return ret;
}

static int archos_omap4_mcpdm_hw_free(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	if (rtd->current_fe == ABE_FRONTEND_DAI_MODEM) {
		/* freed Modem McBSP configuration */
		ret = archos_omap4_modem_mcbsp_configure(substream, 0);
	}

	return ret;
}

static struct snd_soc_ops archos_omap4_mcpdm_ops = {
	.hw_params = archos_omap4_mcpdm_hw_params,
	.hw_free = archos_omap4_mcpdm_hw_free,
};

static int archos_omap4_mcbsp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int be_id = rtd->dai_link->be_id;
	int ret = 0;

	if (be_id == OMAP_ABE_DAI_BT_VX) {
	        ret = snd_soc_dai_set_fmt(cpu_dai,
                                  SND_SOC_DAIFMT_DSP_B |
                                  SND_SOC_DAIFMT_NB_IF |
                                  SND_SOC_DAIFMT_CBM_CFM);
	} else {
		ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	}

	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/*
	 * TODO: where does this clock come from (external source??) -
	 * do we need to enable it.
	 */
	/* Set McBSP clock to external */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK,
				     64 * params_rate(params),
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu system clock\n");
		return ret;
	}
	return 0;
}

static struct snd_soc_ops archos_omap4_mcbsp_ops = {
	.hw_params = archos_omap4_mcbsp_hw_params,
};

static int archos_omap4_dmic_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;

	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_DMIC_SYSCLK_PAD_CLKS,
				     19200000, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set DMIC cpu system clock\n");
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_DMIC_CLKDIV, 8);
	if (ret < 0) {
		printk(KERN_ERR "can't set DMIC cpu clock divider\n");
		return ret;
	}

	if (rtd->current_fe == ABE_FRONTEND_DAI_MODEM) {
		/* set Modem McBSP configuration  */
		ret = archos_omap4_modem_mcbsp_configure(substream, 1);
	}

	return ret;
}
 
static int archos_omap4_dmic_hw_free(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	if (rtd->current_fe == ABE_FRONTEND_DAI_MODEM) {
		/* freed Modem McBSP configuration */
		ret = archos_omap4_modem_mcbsp_configure(substream, 0);
	}

	return ret;
}

static struct snd_soc_ops archos_omap4_dmic_ops = {
	.hw_params = archos_omap4_dmic_hw_params,
	.hw_free = archos_omap4_dmic_hw_free,
};

static int mcbsp_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_interval *channels = hw_param_interval(params,
                                       SNDRV_PCM_HW_PARAM_CHANNELS);
	unsigned int be_id = rtd->dai_link->be_id;
	unsigned int threshold;
	int val;

	switch (be_id) {
	case OMAP_ABE_DAI_MM_FM:
		val = SNDRV_PCM_FORMAT_S32_LE;
		channels->min = 2;
		threshold = 2;
		break;
	case OMAP_ABE_DAI_BT_VX:
		val = SNDRV_PCM_FORMAT_S16_LE;
		channels->min = 1;
		threshold = 1;
		break;
	default:
		val = SNDRV_PCM_FORMAT_S16_LE;
		threshold = 1;
		break;
	}

	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
		     val);

	omap_mcbsp_set_tx_threshold(cpu_dai->id, threshold);
	omap_mcbsp_set_rx_threshold(cpu_dai->id, threshold);

	return 0;
}

static int dmic_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
	                            SNDRV_PCM_HW_PARAM_FIRST_MASK],
	                            SNDRV_PCM_FORMAT_S32_LE);
	return 0;
}

/* Headset jack */
static struct snd_soc_jack hs_jack;

/*Headset jack detection DAPM pins */
static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headset Stereophone",
		.mask = SND_JACK_HEADPHONE,
	},
};

static int archos_omap4_get_power_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = twl6040_power_mode;
	return 0;
}

static int archos_omap4_set_power_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	if (twl6040_power_mode == ucontrol->value.integer.value[0])
		return 0;

	twl6040_power_mode = ucontrol->value.integer.value[0];

	return 1;
}

static const char *power_texts[] = {"Low-Power", "High-Performance", "Medium-Power"};

static const struct soc_enum archos_omap4_enum[] = {
	SOC_ENUM_SINGLE_EXT(3, power_texts),
};

static const struct snd_kcontrol_new archos_omap4_controls[] = {
	SOC_ENUM_EXT("TWL6040 Power Mode", archos_omap4_enum[0],
		archos_omap4_get_power_mode, archos_omap4_set_power_mode),
};

/* Archos machine DAPM */
static const struct snd_soc_dapm_widget archos_omap4_twl6040_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Ext Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_HP("Headset Stereophone", NULL),
	SND_SOC_DAPM_SPK("Earphone Spk", NULL),
	SND_SOC_DAPM_INPUT("Aux/FM Stereo In"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* External Mics: MAINMIC, SUBMIC with bias*/
	{"MAINMIC", NULL, "Main Mic Bias"},
	{"SUBMIC", NULL, "Main Mic Bias"},
	{"Main Mic Bias", NULL, "Ext Mic"},

	/* External Speakers: HFL, HFR */
	{"Ext Spk", NULL, "HFL"},
	{"Ext Spk", NULL, "HFR"},

	/* Headset Mic: HSMIC with bias */
	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headset Mic"},

	/* Headset Stereophone (Headphone): HSOL, HSOR */
	{"Headset Stereophone", NULL, "HSOL"},
	{"Headset Stereophone", NULL, "HSOR"},

	/* Earphone speaker */
	{"Earphone Spk", NULL, "EP"},

	/* Aux/FM Stereo In: AFML, AFMR */
	{"AFML", NULL, "Aux/FM Stereo In"},
	{"AFMR", NULL, "Aux/FM Stereo In"},
};

static int archos_omap4_twl6040_init_hs(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret;

	/* Add specific controls */
	ret = snd_soc_add_controls(codec, archos_omap4_controls,
				ARRAY_SIZE(archos_omap4_controls));
	if (ret)
		return ret;

	/* Add specific widgets */
	ret = snd_soc_dapm_new_controls(codec->dapm, archos_omap4_twl6040_dapm_widgets,
				ARRAY_SIZE(archos_omap4_twl6040_dapm_widgets));
	if (ret)
		return ret;

	/* Set up specific audio path audio_map */
	snd_soc_dapm_add_routes(codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	/* Connected pins */
	snd_soc_dapm_enable_pin(codec->dapm, "Ext Mic");
	snd_soc_dapm_enable_pin(codec->dapm, "Ext Spk");
	snd_soc_dapm_enable_pin(codec->dapm, "AFML");
	snd_soc_dapm_enable_pin(codec->dapm, "AFMR");
	snd_soc_dapm_disable_pin(codec->dapm, "Headset Mic");
	snd_soc_dapm_disable_pin(codec->dapm, "Headset Stereophone");

	/* allow audio paths from the audio modem to run during suspend */
	snd_soc_dapm_ignore_suspend(codec, "Ext Mic");
	snd_soc_dapm_ignore_suspend(codec, "Ext Spk");
	snd_soc_dapm_ignore_suspend(codec, "AFML");
	snd_soc_dapm_ignore_suspend(codec, "AFMR");
	snd_soc_dapm_ignore_suspend(codec, "Headset Mic");
	snd_soc_dapm_ignore_suspend(codec, "Headset Stereophone");

	ret = snd_soc_dapm_sync(codec->dapm);
	if (ret)
		return ret;

	/* Headset jack detection */
	ret = snd_soc_jack_new(codec, "Headset Jack",
				SND_JACK_HEADSET, &hs_jack);
	if (ret)
		return ret;

	ret = snd_soc_jack_add_pins(&hs_jack, ARRAY_SIZE(hs_jack_pins),
				hs_jack_pins);

	twl6040_hs_jack_detect(codec, &hs_jack, SND_JACK_HEADSET);

	/* don't wait before switching of HS power */
	rtd->pmdown_time = 0;

	return ret;
}

static int archos_omap4_twl6040_init_hf(struct snd_soc_pcm_runtime *rtd)
{
	/* don't wait before switching of HF power */
	rtd->pmdown_time = 0;
	return 0;
}

/* TODO: make this a separate BT CODEC driver or DUMMY */
static struct snd_soc_dai_driver dai[] = {
{
	.name = "Bluetooth",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
/* TODO: make this a separate FM CODEC driver or DUMMY */
{
	.name = "FM Digital",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "HDMI",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
};

static const char *mm1_be[] = {
		OMAP_ABE_BE_PDM_DL1,
		OMAP_ABE_BE_PDM_UL1,
		OMAP_ABE_BE_PDM_DL2,
		OMAP_ABE_BE_BT_VX,
		OMAP_ABE_BE_MM_EXT0,
		OMAP_ABE_BE_DMIC0,
		OMAP_ABE_BE_DMIC1,
		OMAP_ABE_BE_DMIC2,
		OMAP_ABE_BE_VXREC,
};

static const char *mm2_be[] = {
		OMAP_ABE_BE_PDM_UL1,
		OMAP_ABE_BE_BT_VX,
		OMAP_ABE_BE_MM_EXT0,
		OMAP_ABE_BE_DMIC0,
		OMAP_ABE_BE_DMIC1,
		OMAP_ABE_BE_DMIC2,
		OMAP_ABE_BE_VXREC,
};

static const char *tones_be[] = {
		OMAP_ABE_BE_PDM_DL1,
		OMAP_ABE_BE_PDM_DL2,
		OMAP_ABE_BE_BT_VX,
		OMAP_ABE_BE_MM_EXT0,
};

static const char *vib_be[] = {
		OMAP_ABE_BE_PDM_VIB,
};

static const char *modem_be[] = {
		OMAP_ABE_BE_PDM_DL1,
		OMAP_ABE_BE_PDM_UL1,
		OMAP_ABE_BE_PDM_DL2,
		OMAP_ABE_BE_BT_VX,
		OMAP_ABE_BE_DMIC0,
		OMAP_ABE_BE_DMIC1,
		OMAP_ABE_BE_DMIC2,
};

static const char *mm_lp_be[] = {
		OMAP_ABE_BE_PDM_DL1,
		OMAP_ABE_BE_PDM_DL2,
		OMAP_ABE_BE_BT_VX,
		OMAP_ABE_BE_MM_EXT0,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link archos_omap4_dai[] = {

/*
 * Frontend DAIs - i.e. userspace visible interfaces (ALSA PCMs)
 */

	{
		.name = "Archos Media",
		.stream_name = "Multimedia",

		/* ABE components - MM-UL & MM_DL */
		.cpu_dai_name = "MultiMedia1",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = mm1_be,
		.num_be = ARRAY_SIZE(mm1_be),
		.fe_playback_channels = 2,
		.fe_capture_channels = 8,
		.no_host_mode = SND_SOC_DAI_LINK_OPT_HOST,
	},
	{
		.name = "Archos Media Capture",
		.stream_name = "Multimedia Capture",

		/* ABE components - MM-UL2 */
		.cpu_dai_name = "MultiMedia2",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = mm2_be,
		.num_be = ARRAY_SIZE(mm2_be),
		.fe_capture_channels = 2,
	},
	{
		.name = "Archos Voice",
		.stream_name = "Voice",

		/* ABE components - VX-UL & VX-DL */
		.cpu_dai_name = "Voice",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = mm1_be,
		.num_be = ARRAY_SIZE(mm1_be),
		.fe_playback_channels = 2,
		.fe_capture_channels = 2,
		.no_host_mode = SND_SOC_DAI_LINK_OPT_HOST,
	},
	{
		.name = "Archos Tones Playback",
		.stream_name = "Tone Playback",

		/* ABE components - TONES_DL */
		.cpu_dai_name = "Tones",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = tones_be,
		.num_be = ARRAY_SIZE(tones_be),
		.fe_playback_channels = 2,
	},
	{
		.name = "Archos Vibra Playback",
		.stream_name = "VIB-DL",

		.cpu_dai_name = "Vibra",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = vib_be,
		.num_be = ARRAY_SIZE(vib_be),
		.fe_playback_channels = 2,
	},
	{
		.name = "Archos MODEM",
		.stream_name = "MODEM",

		/* ABE components - MODEM <-> McBSP2 */
		.cpu_dai_name = "MODEM",
		.platform_name = "omap-aess-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = modem_be,
		.num_be = ARRAY_SIZE(modem_be),
		.fe_playback_channels = 2,
		.fe_capture_channels = 2,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
	},
	{
		.name = "Archos Media LP",
		.stream_name = "Multimedia",

		/* ABE components - MM-DL (mmap) */
		.cpu_dai_name = "MultiMedia1 LP",
		.platform_name = "omap-aess-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = mm_lp_be,
		.num_be = ARRAY_SIZE(mm_lp_be),
		.fe_playback_channels = 2,
		.no_host_mode = SND_SOC_DAI_LINK_OPT_HOST,
	},
#ifdef CONFIG_SND_OMAP_SOC_HDMI
	{
		.name = "hdmi",
		.stream_name = "HDMI",

		.cpu_dai_name = "hdmi-dai",
		.platform_name = "omap-pcm-audio",

		/* HDMI*/
		.codec_dai_name = "HDMI",

		.no_codec = 1,
	},
#endif
	{
		.name = "Legacy McBSP",
		.stream_name = "Multimedia",

		/* ABE components - MCBSP2 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.platform_name = "omap-pcm-audio",

		/* FM */
		.codec_dai_name = "FM Digital",

		.no_codec = 1, /* TODO: have a dummy CODEC */
		.ops = &archos_omap4_mcbsp_ops,
	},
	{
		.name = "Legacy McPDM",
		.stream_name = "Headset Playback",

		/* ABE components - DL1 */
		.cpu_dai_name = "mcpdm-dl",
		.platform_name = "omap-pcm-audio",

		/* Phoenix - DL1 DAC */
		.codec_dai_name =  "twl6040-dl1",
		.codec_name = "twl6040-codec",

		.ops = &archos_omap4_mcpdm_ops,
		.ignore_suspend = 1,
	},
	{
		.name = "Legacy DMIC",
		.stream_name = "DMIC Capture",

		/* ABE components - DMIC0 */
		.cpu_dai_name = "omap-dmic-dai-0",
		.platform_name = "omap-pcm-audio",

		/* DMIC codec */
		.codec_dai_name = "dmic-hifi",
		.codec_name = "dmic-codec.0",

		.ops = &archos_omap4_dmic_ops,
		.ignore_suspend = 1,
	},

/*
 * Backend DAIs - i.e. dynamically matched interfaces, invisible to userspace.
 * Matched to above interfaces at runtime, based upon use case.
 */

	{
		.name = OMAP_ABE_BE_PDM_DL1,
		.stream_name = "HS Playback",

		/* ABE components - DL1 */
		.cpu_dai_name = "mcpdm-dl1",
		.platform_name = "omap-aess-audio",

		/* Phoenix - DL1 DAC */
		.codec_dai_name =  "twl6040-dl1",
		.codec_name = "twl6040-codec",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.init = archos_omap4_twl6040_init_hs,
		.ops = &archos_omap4_mcpdm_ops,
		.be_id = OMAP_ABE_DAI_PDM_DL1,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_PDM_UL1,
		.stream_name = "Analog Capture",

		/* ABE components - UL1 */
		.cpu_dai_name = "mcpdm-ul1",
		.platform_name = "omap-aess-audio",

		/* Phoenix - UL ADC */
		.codec_dai_name =  "twl6040-ul",
		.codec_name = "twl6040-codec",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.ops = &archos_omap4_mcpdm_ops,
		.be_id = OMAP_ABE_DAI_PDM_UL,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_PDM_DL2,
		.stream_name = "HF Playback",

		/* ABE components - DL2 */
		.cpu_dai_name = "mcpdm-dl2",
		.platform_name = "omap-aess-audio",

		/* Phoenix - DL2 DAC */
		.codec_dai_name =  "twl6040-dl2",
		.codec_name = "twl6040-codec",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.init = archos_omap4_twl6040_init_hf,
		.ops = &archos_omap4_mcpdm_ops,
		.be_id = OMAP_ABE_DAI_PDM_DL2,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_PDM_VIB,
		.stream_name = "Vibra",

		/* ABE components - VIB1 DL */
		.cpu_dai_name = "mcpdm-vib",
		.platform_name = "omap-aess-audio",

		/* Phoenix - PDM to PWM */
		.codec_dai_name =  "twl6040-vib",
		.codec_name = "twl6040-codec",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.ops = &archos_omap4_mcpdm_ops,
		.be_id = OMAP_ABE_DAI_PDM_VIB,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_BT_VX,
		.stream_name = "BT",

		/* ABE components - MCBSP1 - BT-VX */
		.cpu_dai_name = "omap-mcbsp-dai.0",
		.platform_name = "omap-aess-audio",

		/* Bluetooth */
		.codec_dai_name = "Bluetooth",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.no_codec = 1, /* TODO: have a dummy CODEC */
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &archos_omap4_mcbsp_ops,
		.be_id = OMAP_ABE_DAI_BT_VX,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_MM_EXT0,
		.stream_name = "FM",

		/* ABE components - MCBSP2 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.platform_name = "omap-aess-audio",

		/* FM */
		.codec_dai_name = "FM Digital",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.no_codec = 1, /* TODO: have a dummy CODEC */
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &archos_omap4_mcbsp_ops,
		.be_id = OMAP_ABE_DAI_MM_FM,
	},
	{
		.name = OMAP_ABE_BE_MM_EXT1,
		.stream_name = "MODEM",

		/* ABE components - MCBSP2 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.platform_name = "omap-aess-audio",

		/* MODEM */
		.codec_dai_name = "MODEM",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.no_codec = 1, /* TODO: have a dummy CODEC */
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &archos_omap4_mcbsp_ops,
		.be_id = OMAP_ABE_DAI_MODEM,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_DMIC0,
		.stream_name = "DMIC0",

		/* ABE components - DMIC UL 1 */
		.cpu_dai_name = "omap-dmic-abe-dai-0",
		.platform_name = "omap-aess-audio",

		/* DMIC 0 */
		.codec_dai_name = "dmic-hifi",
		.codec_name = "dmic-codec.0",
		.ops = &archos_omap4_dmic_ops,

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = dmic_be_hw_params_fixup,
		.be_id = OMAP_ABE_DAI_DMIC0,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_DMIC1,
		.stream_name = "DMIC1",

		/* ABE components - DMIC UL 1 */
		.cpu_dai_name = "omap-dmic-abe-dai-1",
		.platform_name = "omap-aess-audio",

		/* DMIC 1 */
		.codec_dai_name = "dmic-hifi",
		.codec_name = "dmic-codec.1",
		.ops = &archos_omap4_dmic_ops,

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = dmic_be_hw_params_fixup,
		.be_id = OMAP_ABE_DAI_DMIC1,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_DMIC2,
		.stream_name = "DMIC2",

		/* ABE components - DMIC UL 2 */
		.cpu_dai_name = "omap-dmic-abe-dai-2",
		.platform_name = "omap-aess-audio",

		/* DMIC 2 */
		.codec_dai_name = "dmic-hifi",
		.codec_name = "dmic-codec.2",
		.ops = &archos_omap4_dmic_ops,

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = dmic_be_hw_params_fixup,
		.be_id = OMAP_ABE_DAI_DMIC2,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_VXREC,
		.stream_name = "VXREC",

		/* ABE components - VxREC */
		.cpu_dai_name = "omap-abe-vxrec-dai",
		.platform_name = "omap-aess-audio",

		/* no codec needed */
		.codec_dai_name = "null-codec-dai",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.no_codec = 1, /* TODO: have a dummy CODEC */
		.be_id = OMAP_ABE_DAI_VXREC,
		.ignore_suspend = 1,
	},
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_archos_omap4 = {
	.name = "ARCHOS-TWL6040",
	.long_name = "Archos Board (TWL6040)",
	.dai_link = archos_omap4_dai,
	.num_links = ARRAY_SIZE(archos_omap4_dai),
};

static struct platform_device *archos_omap4_snd_device;

static int __devinit archos_omap4_soc_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("%s: Archos Device SoC init\n", __func__);

	/* Master clock has to be enable to feed the TWL6040 */
	pt_audio_device_io = archos_audio_twl6040_get_io();
//	_enable_master_clock();

	archos_omap4_snd_device = platform_device_alloc("soc-audio", -1);
	if (!archos_omap4_snd_device) {
		pr_err("Platform device allocation failed\n");
		ret = -ENOMEM;
		goto err1;
	}

	ret = snd_soc_register_dais(&archos_omap4_snd_device->dev, dai, ARRAY_SIZE(dai));
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s: unable to register SOC DAI, ret=%i\n",
				__func__, ret);
		goto err2;
	}

	platform_set_drvdata(archos_omap4_snd_device, &snd_soc_archos_omap4);

	ret = platform_device_add(archos_omap4_snd_device);
	if (ret) {
		printk(KERN_ERR "Unable to add platform device\n");
		goto err2;
	}

	return 0;

err2:
	platform_device_put(archos_omap4_snd_device);
err1:
//	_disable_master_clock();

	return ret;
}

static int __devexit archos_omap4_soc_remove(struct platform_device *pdev)
{
	platform_device_unregister(archos_omap4_snd_device);
//	_disable_master_clock();
	return 0;
}

static struct platform_driver twl6040_audio_soc_driver = {
	.driver = {
		.name = "archos-twl6040-asoc",
		.owner = THIS_MODULE,
	},
	.probe = archos_omap4_soc_probe,
	.remove = __devexit_p(archos_omap4_soc_remove),
};

static int __init archos_omap4_soc_init(void)
{
	return platform_driver_register(&twl6040_audio_soc_driver);
}
module_init(archos_omap4_soc_init);

static void __exit archos_omap4_soc_exit(void)
{
	platform_driver_unregister(&twl6040_audio_soc_driver);
}
module_exit(archos_omap4_soc_exit);

MODULE_AUTHOR("Misael Lopez Cruz <x0052729@ti.com>");
MODULE_DESCRIPTION("ALSA SoC Archos (TWL6040) Device");
MODULE_LICENSE("GPL");

