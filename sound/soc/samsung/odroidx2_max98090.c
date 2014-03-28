/*
 *  Copyright (C) 2014 Samsung Electronics Co., Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/of.h>
#include <linux/module.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <linux/clk.h>
#include "i2s.h"

static int set_epll_rate(unsigned long rate)
{
	struct clk *fout_epll;

	fout_epll = clk_get(NULL, "fout_epll");
	if (IS_ERR(fout_epll)) {
		pr_err(KERN_ERR "%s: failed to get fout_epll\n", __func__);
		return PTR_ERR(fout_epll);
	}

	if (rate == clk_get_rate(fout_epll))
		goto out;

	clk_set_rate(fout_epll, rate);

out:
	clk_put(fout_epll);

	return 0;
}

static int odroidx2_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int bfs, psr, rfs, ret;
	unsigned long rclk;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_U24:
	case SNDRV_PCM_FORMAT_S24:
		bfs = 48;
		break;
	case SNDRV_PCM_FORMAT_U16_LE:
	case SNDRV_PCM_FORMAT_S16_LE:
		bfs = 32;
		break;
	default:
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 16000:
	case 22050:
	case 24000:
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
		if (bfs == 48)
			rfs = 384;
		else
			rfs = 256;
		break;
	case 64000:
		rfs = 384;
		break;
	case 8000:
	case 11025:
	case 12000:
		if (bfs == 48)
			rfs = 768;
		else
			rfs = 512;
		break;
	default:
		return -EINVAL;
	}

	rclk = params_rate(params) * rfs;

	switch (rclk) {
	case 4096000:
	case 5644800:
	case 6144000:
	case 8467200:
	case 9216000:
		psr = 8;
		break;
	case 8192000:
	case 11289600:
	case 12288000:
	case 16934400:
	case 18432000:
		psr = 4;
		break;
	case 22579200:
	case 24576000:
	case 33868800:
	case 36864000:
		psr = 2;
		break;
	case 67737600:
	case 73728000:
		psr = 1;
		break;
	default:
		pr_err("Not yet supported!\n");
		return -EINVAL;
	}

	set_epll_rate(rclk * psr);

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
					| SND_SOC_DAIFMT_NB_NF
					| SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
					| SND_SOC_DAIFMT_NB_NF
					| SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, rclk, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK,
					0, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, SAMSUNG_I2S_DIV_BCLK, bfs);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops odroidx2_ops = {
	.hw_params = odroidx2_hw_params,
};

static struct snd_soc_dai_link odroidx2_dai[] = {
	{
		.name = "MAX98090",
		.stream_name = "Max98090 PCM",
		.codec_dai_name = "HiFi",
		.ops = &odroidx2_ops,
	},
};

static struct snd_soc_card odroidx2 = {
	.name = "odroidx2",
	.owner = THIS_MODULE,
	.dai_link = odroidx2_dai,
	.num_links = ARRAY_SIZE(odroidx2_dai),
};

static int odroidx2_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &odroidx2;

	if (!pdev->dev.of_node)
		return -ENODEV;

	card->dev = &pdev->dev;

	odroidx2_dai[0].codec_name = NULL;
	odroidx2_dai[0].codec_of_node = of_parse_phandle(np, "audio-codec", 0);
	if (!odroidx2_dai[0].codec_of_node) {
		dev_err(&pdev->dev,
			"Property 'Tizen,audio-codec' missing or invalid\n");
		return -EINVAL;
	}

	odroidx2_dai[0].cpu_name = NULL;
	odroidx2_dai[0].cpu_of_node = of_parse_phandle(np, "i2s-controller", 0);
	if (!odroidx2_dai[0].cpu_of_node) {
		dev_err(&pdev->dev,
			"Property 'Tizen,i2s-controller' missing or invalid\n");
		return -EINVAL;
	}

	odroidx2_dai[0].platform_of_node = odroidx2_dai[0].cpu_of_node;

	ret = snd_soc_register_card(card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed:%d\n", ret);

	return ret;
}

static int odroidx2_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&odroidx2);

	return 0;
}

static const struct of_device_id odroidx2_audio_of_match[] = {
	{ .compatible = "samsung,odroidx2-audio", },
	{ },
};
MODULE_DEVICE_TABLE(of, odroid_audio_of_match);

static struct platform_driver odroidx2_audio_driver = {
	.driver = {
		.name = "odroidx2-audio",
		.owner = THIS_MODULE,
		.of_match_table = odroidx2_audio_of_match,
	},
	.probe = odroidx2_audio_probe,
	.remove = odroidx2_audio_remove,
};

module_platform_driver(odroidx2_audio_driver);

MODULE_AUTHOR("zhen1.chen@samsung.com");
MODULE_DESCRIPTION("ALSA SoC Odroidx2 Audio support");
MODULE_LICENSE("GPL");
