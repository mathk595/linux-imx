/*
 * Copyright (C) 2019 Keith & Koep GmbH
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/syscon.h>

struct imx_priv {
	struct platform_device *pdev;
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	struct clk *codec_clk;
	unsigned int clk_frequency;
};

static const struct snd_soc_dapm_widget imx_wm8983_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Speaker", NULL),
	SND_SOC_DAPM_LINE("LineIn Jack", NULL),
	SND_SOC_DAPM_LINE("Microphone", NULL),
};

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime 	*rtd 		= substream->private_data;
	struct snd_soc_dai 		*cpu_dai 	= asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card 		*card 		= rtd->card;
	struct device 			*dev 		= card->dev;
	unsigned int fmt;
	int ret = 0;

	fmt = 	SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF |		/* Normal Bit-Clock + Frame */
		SND_SOC_DAIFMT_CBS_CFS;		/* Clock + Frame Slave */

	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 
			0, /*active slot tx_mask*/ 
			0, /*active slot rx_mask*/
			2, /*Number of slots*/
			params_physical_width(params) /*Bit-width for each slot*/);
	if (ret) {
		dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops imx_hifi_ops = {
// .startup
// .shutdown
	.hw_params = imx_hifi_hw_params,
// .hw_free
// .prepare
// .trigger
};

static int imx_wm8983_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime 	*rtd 		= list_first_entry(&card->rtd_list, struct snd_soc_pcm_runtime, list);
	struct snd_soc_dai 		*codec_dai 	= asoc_rtd_to_cpu(rtd, 0);
	struct imx_priv 		*priv 		= snd_soc_card_get_drvdata(card);
	int ret;

	priv->clk_frequency = clk_get_rate(priv->codec_clk);

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, priv->clk_frequency,
							SND_SOC_CLOCK_IN);

	return 0;
}

static int imx_wm8983_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np = NULL;
	struct platform_device *cpu_pdev;
	struct imx_priv *priv;
	struct i2c_client *codec_pdev = NULL;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdev = pdev;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_pdev = of_find_i2c_device_by_node(codec_np);
	if (!codec_pdev || !codec_pdev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec i2c device \n");
		ret = -EINVAL;
		goto fail;
	}

	priv->codec_clk = devm_clk_get(&codec_pdev->dev, "mclk");
	if (IS_ERR(priv->codec_clk)) {
		ret = PTR_ERR(priv->codec_clk);
		dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
		goto fail;
	}

	priv->dai.name = "HiFi";
	priv->dai.stream_name = "HiFi";
	// If more then ohne wm8983, need to set cpu_name or cpu_of_node!
	// priv->dai.cpu_name
	// priv->dai.cpu_of_node
	priv->dai.cpus->dai_name = dev_name(&cpu_pdev->dev);
	// priv->dai.codec_name
	priv->dai.codecs->of_node = codec_np;
	priv->dai.codecs->dai_name = "wm8983";
	// priv->dai.codecs
	// priv->dai.num_codecs	
	// priv->dai.platform_name
	priv->dai.platforms->of_node = cpu_np;
	// priv->dai.id
	// priv->dai.params
	// priv->dai.num_params
	// priv->dai.dai_fmt
	// priv->dai.trigger[]
	// priv->dai.init
	// priv->dai.be_hw_params_fixup
	priv->dai.ops = &imx_hifi_ops;
	// priv->dai.compr_ops
	// priv->dai.nonatomic
	// priv->dai.playback_only
	// priv->dai.capture_only
	// priv->dai.ignore_suspend
	// priv->dai.symmetric_rates
	// priv->dai.symmetric_channels
	// priv->dai.symmetric_samplebits
	// priv->dai.nopcm
	// priv->dai.dynamic
	// priv->dai.dpcm_capture
	// priv->dai.dpcm_playback
	// priv->dai.dpcm_merged_format
	// priv->dai.dpcm_merged_chan
	// priv->dai.ignore_pmdown_time
	// priv->dai.list
	// priv->dai.dobj

	
	// priv->card.name:
	// priv->card.long_name
	// priv->card.dmi_longname
	priv->card.dev = &pdev->dev;
	// priv->card.snd_card
	priv->card.owner = THIS_MODULE;
	// priv->card.mutex
	// priv->card.dapm_mutex
	// priv->card.instantiated
	// priv->card.probe
	priv->card.late_probe = imx_wm8983_late_probe;
	// priv->card.remove
	// priv->card.suspend_pre
	// priv->card.suspend_post
	// priv->card.resume_pre
	// priv->card.resume_post
	// priv->card.set_bias_level
	// priv->card.set_bias_level_post
	// priv->card.add_dai_link
	// priv->card.remove_dai_link
	// priv->card.pmdown_time
	priv->card.dai_link = &(priv->dai);
	priv->card.num_links = 1;
	// priv->card.dai_link_list
	// priv->card.num_dai_links
	// priv->card.rtd_list
	// priv->card.num_rtd
	// priv->card.codec_conf
	// priv->card.num_configs
	// priv->card.aux_dev
	// priv->card.num_aux_devs
	// priv->card.aux_comp_list
	// priv->card.controls
	// priv->card.num_controls		
	priv->card.dapm_widgets = imx_wm8983_dapm_widgets;
	priv->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8983_dapm_widgets);
	// priv->card.dapm_routes
	// priv->card.num_dapm_routes
	// priv->card.of_dapm_widgets
	// priv->card.num_of_dapm_widgets

	// priv->card.of_dapm_routes:
	// priv->card.num_of_dapm_routes:
	ret = snd_soc_of_parse_card_name(&priv->card, "model");
	if (ret)
		goto fail;

	ret = snd_soc_of_parse_audio_routing(&priv->card, "audio-routing");
	if (ret)
		goto fail;
	// priv->card.fully_routed
	// priv->card.deferred_resume_work
	// priv->card.component_dev_list
	// priv->card.widgets
	// priv->card.paths
	// priv->card.dapm_list
	// priv->card.dapm_dirty
	// priv->card.dobj_list
	// priv->card.dapm
	// priv->card.dapm_stats
	// priv->card.update
	// priv->card.pop_time
	
	// priv->card.drvdata:
	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = devm_snd_soc_register_card(&pdev->dev, &priv->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	ret = 0;
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static const struct of_device_id imx_wm8983_dt_ids[] = {
	{ .compatible = "kuk,imx-audio-wm8983", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8983_dt_ids);

static struct platform_driver imx_wm8983_driver = {
	.driver = {
		.name = "imx-wm8983",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm8983_dt_ids,
	},
	.probe = imx_wm8983_probe,
};
module_platform_driver(imx_wm8983_driver);

MODULE_AUTHOR("Keith & Koep GmbH");
MODULE_DESCRIPTION("Keith & Koep i.MX WM8983 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm8983");
