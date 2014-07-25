/*
 * pwm-fan.c - Hwmon driver for fans connected to PWM lines.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *
 * Author: Kamil Debski <k.debski@samsung.com>
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
 */

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>

#define MAX_PWM 255

static const unsigned char pwm_fan_cooling_states[] = {
	0, (MAX_PWM * 2) / 5, (MAX_PWM * 2) / 3, MAX_PWM
};

struct pwm_fan_ctx {
	struct mutex lock;
	struct pwm_device *pwm;
	unsigned char pwm_value;
	unsigned char pwm_fan_state;
	unsigned char pwm_fan_max_state;
};

static int  __set_pwm(struct pwm_fan_ctx *ctx, unsigned long pwm)
{
	unsigned long duty;
	int ret;

	if (ctx->pwm_value == pwm)
		return 0;

	mutex_lock(&ctx->lock);
	if (pwm == 0) {
		pwm_disable(ctx->pwm);
		goto exit_set_pwm;
	}

	duty = DIV_ROUND_UP(pwm * (ctx->pwm->period - 1), MAX_PWM);
	ret = pwm_config(ctx->pwm, duty, ctx->pwm->period);
	if (ret)
		goto exit_set_pwm_err;

	if (ctx->pwm_value == 0) {
		ret = pwm_enable(ctx->pwm);
		if (ret)
			goto exit_set_pwm_err;
	}

exit_set_pwm:
	ctx->pwm_value = pwm;
exit_set_pwm_err:
	mutex_unlock(&ctx->lock);
	return ret;
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	unsigned long pwm;
	int ret;

	if (kstrtoul(buf, 10, &pwm) || pwm > MAX_PWM)
		return -EINVAL;

	ret = __set_pwm(ctx, pwm);
	if (ret)
		return ret;

	return count;
}

static ssize_t show_pwm(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ctx->pwm_value);
}


static SENSOR_DEVICE_ATTR(pwm1, S_IRUGO | S_IWUSR, show_pwm, set_pwm, 0);

static struct attribute *pwm_fan_attributes[] = {
	&sensor_dev_attr_pwm1.dev_attr.attr,
	NULL,
};

static const struct attribute_group pwm_fan_group = {
	.attrs = pwm_fan_attributes,
};

/* thermal cooling device callbacks */
static int pwm_fan_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;

	*state = ctx->pwm_fan_max_state;

	return 0;
}

static int pwm_fan_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;
	if (!ctx)
		return -EINVAL;

	*state = ctx->pwm_fan_state;

	return 0;
}

static int
pwm_fan_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;
	int ret;

	if (!ctx || (state > ctx->pwm_fan_max_state))
		return -EINVAL;

	if (state == ctx->pwm_fan_state)
		return 0;

	ret = __set_pwm(ctx, pwm_fan_cooling_states[state]);
	if (ret) {
		pr_err("%s: Cannot set pwm!\n", __func__);
		return ret;
	}

	ctx->pwm_fan_state = state;

	return ret;
}

static const struct thermal_cooling_device_ops pwm_fan_cooling_ops = {
	.get_max_state = pwm_fan_get_max_state,
	.get_cur_state = pwm_fan_get_cur_state,
	.set_cur_state = pwm_fan_set_cur_state,
};

static int pwm_fan_probe(struct platform_device *pdev)
{
	struct thermal_cooling_device *cdev;
	struct pwm_fan_ctx *ctx;
	struct device *hwmon;
	int ret;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_init(&ctx->lock);

	ctx->pwm = devm_of_pwm_get(&pdev->dev, pdev->dev.of_node, 0);
	if (IS_ERR(ctx->pwm)) {
		dev_err(&pdev->dev, "Could not get PWM\n");
		return PTR_ERR(ctx->pwm);
	}

	dev_set_drvdata(&pdev->dev, ctx);
	platform_set_drvdata(pdev, ctx);

	ret = sysfs_create_group(&pdev->dev.kobj, &pwm_fan_group);
	if (ret)
		return ret;

	hwmon = hwmon_device_register(&pdev->dev);

	if (IS_ERR(hwmon)) {
		dev_err(&pdev->dev, "Failed to register hwmon device");
		pwm_disable(ctx->pwm);
		return PTR_ERR(hwmon);
	}

	cdev = thermal_cooling_device_register("pwm-fan", ctx,
					       &pwm_fan_cooling_ops);
	if (IS_ERR(cdev)) {
		dev_err(&pdev->dev, "Failed to register pwm-fan as thermal coolong device");
		pwm_disable(ctx->pwm);
		return PTR_ERR(cdev);
	}
	ctx->pwm_fan_max_state = ARRAY_SIZE(pwm_fan_cooling_states) - 1;

	return 0;
}

static int pwm_fan_remove(struct platform_device *pdev)
{
	struct pwm_fan_ctx *ctx = platform_get_drvdata(pdev);

	if (ctx->pwm_value)
		pwm_disable(ctx->pwm);
	mutex_destroy(&ctx->lock);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_fan_suspend(struct device *dev)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	if (ctx->pwm_value)
		pwm_disable(ctx->pwm);
	return 0;
}

static int pwm_fan_resume(struct device *dev)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	if (ctx->pwm_value)
		return pwm_enable(ctx->pwm);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_fan_pm, pwm_fan_suspend, pwm_fan_resume);

static struct of_device_id of_pwm_fan_match[] = {
	{ .compatible = "pwm-fan", },
	{},
};

static struct platform_driver pwm_fan_driver = {
	.probe		= pwm_fan_probe,
	.remove		= pwm_fan_remove,
	.driver	= {
		.name		= "pwm-fan",
		.pm		= &pwm_fan_pm,
		.of_match_table	= of_pwm_fan_match,
	},
};

module_platform_driver(pwm_fan_driver);

MODULE_AUTHOR("Kamil Debski <k.debski@samsung.com>");
MODULE_ALIAS("platform:pwm-fan");
MODULE_DESCRIPTION("PWM FAN driver");
MODULE_LICENSE("GPL");
