/*
 *	LED Flash class driver for the AAT1290
 *	1.5A Step-Up Current Regulator for Flash LEDs
 *
 *	Copyright (C) 2015, Samsung Electronics Co., Ltd.
 *	Author: Jacek Anaszewski <j.anaszewski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/led-class-flash.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <media/v4l2-flash.h>
#include <linux/workqueue.h>

#define AAT1290_MOVIE_MODE_CURRENT_ADDR	17
#define AAT1290_MAX_MM_CURR_PERCENT_0	16
#define AAT1290_MAX_MM_CURR_PERCENT_100	1

#define AAT1290_FLASH_SAFETY_TIMER_ADDR	18

#define AAT1290_MOVIE_MODE_CONFIG_ADDR	19
#define AAT1290_MOVIE_MODE_OFF		1
#define AAT1290_MOVIE_MODE_ON		3

#define AAT1290_MM_CURRENT_RATIO_ADDR	20
#define AAT1290_MM_TO_FL_1_92		1
#define AAT1290_MM_TO_FL_3_7		2
#define AAT1290_MM_TO_FL_5_5		3
#define AAT1290_MM_TO_FL_7_3		4
#define AAT1290_MM_TO_FL_9		5
#define AAT1290_MM_TO_FL_10_7		6
#define AAT1290_MM_TO_FL_12_4		7
#define AAT1290_MM_TO_FL_14		8
#define AAT1290_MM_TO_FL_15_9		9
#define AAT1290_MM_TO_FL_17_5		10
#define AAT1290_MM_TO_FL_19_1		11
#define AAT1290_MM_TO_FL_20_8		12
#define AAT1290_MM_TO_FL_22_4		13
#define AAT1290_MM_TO_FL_24		14
#define AAT1290_MM_TO_FL_25_6		15
#define AAT1290_MM_TO_FL_OFF		16

#define AAT1290_LATCH_TIME_MIN_US	500
#define AAT1290_LATCH_TIME_MAX_US	1000
#define AAT1290_EN_SET_TICK_TIME_US	1
#define AAT1290_FLEN_OFF_DELAY_TIME_US	10
#define AAT1290_FLASH_TM_NUM_LEVELS	16

struct aat1290_led_settings {
	struct led_flash_setting torch_brightness;
	struct led_flash_setting flash_brightness;
	struct led_flash_setting flash_timeout;
};

struct aat1290_led {
	struct platform_device *pdev;
	struct mutex lock;

	struct led_classdev_flash fled_cdev;
	struct v4l2_flash *v4l2_flash;

	int flen_gpio;
	int en_set_gpio;
#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
	int ext_strobe_gpio;
#endif

	u32 max_flash_tm;
	bool movie_mode;

	char *label;
	unsigned int torch_brightness;
	unsigned int flash_timeout;
	struct work_struct work_brightness_set;
};

static struct aat1290_led *fled_cdev_to_led(
				struct led_classdev_flash *fled_cdev)
{
	return container_of(fled_cdev, struct aat1290_led, fled_cdev);
}

static void aat1290_as2cwire_write(struct aat1290_led *led, int addr, int value)
{
	int i;

	gpio_set_value(led->flen_gpio, 0);
	gpio_set_value(led->en_set_gpio, 0);

	udelay(AAT1290_FLEN_OFF_DELAY_TIME_US);

	/* write address */
	for (i = 0; i < addr; ++i) {
		udelay(AAT1290_EN_SET_TICK_TIME_US);
		gpio_set_value(led->en_set_gpio, 0);
		udelay(AAT1290_EN_SET_TICK_TIME_US);
		gpio_set_value(led->en_set_gpio, 1);
	}

	usleep_range(AAT1290_LATCH_TIME_MIN_US, AAT1290_LATCH_TIME_MAX_US);

	/* write data */
	for (i = 0; i < value; ++i) {
		udelay(AAT1290_EN_SET_TICK_TIME_US);
		gpio_set_value(led->en_set_gpio, 0);
		udelay(AAT1290_EN_SET_TICK_TIME_US);
		gpio_set_value(led->en_set_gpio, 1);
	}

	usleep_range(AAT1290_LATCH_TIME_MIN_US, AAT1290_LATCH_TIME_MAX_US);
}

static void aat1290_set_flash_safety_timer(struct aat1290_led *led,
					unsigned int micro_sec)
{
	struct led_classdev_flash *fled_cdev = &led->fled_cdev;
	struct led_flash_setting *flash_tm = &fled_cdev->timeout;
	int flash_tm_reg = AAT1290_FLASH_TM_NUM_LEVELS -
				(micro_sec / flash_tm->step) + 1;

	aat1290_as2cwire_write(led, AAT1290_FLASH_SAFETY_TIMER_ADDR,
							flash_tm_reg);
}

static void aat1290_brightness_set(struct aat1290_led *led,
					enum led_brightness brightness)
{
	mutex_lock(&led->lock);

	if (brightness == 0) {
		gpio_set_value(led->flen_gpio, 0);
		gpio_set_value(led->en_set_gpio, 0);
		goto unlock;
	}

	if (!led->movie_mode) {
		aat1290_as2cwire_write(led, AAT1290_MM_CURRENT_RATIO_ADDR,
					AAT1290_MM_TO_FL_1_92);
		led->movie_mode = true;
	}

	aat1290_as2cwire_write(led, AAT1290_MOVIE_MODE_CURRENT_ADDR,
				AAT1290_MAX_MM_CURR_PERCENT_0 - brightness);
	aat1290_as2cwire_write(led, AAT1290_MOVIE_MODE_CONFIG_ADDR,
				AAT1290_MOVIE_MODE_ON);
unlock:
	mutex_unlock(&led->lock);
}

/* LED subsystem callbacks */

static void aat1290_brightness_set_work(struct work_struct *work)
{
	struct aat1290_led *led =
		container_of(work, struct aat1290_led, work_brightness_set);

	aat1290_brightness_set(led, led->torch_brightness);
}

static void aat1290_led_brightness_set(struct led_classdev *led_cdev,
					enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(led_cdev);
	struct aat1290_led *led = fled_cdev_to_led(fled_cdev);

	led->torch_brightness = brightness;
	schedule_work(&led->work_brightness_set);
}

static int aat1290_led_brightness_set_sync(struct led_classdev *led_cdev,
					enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(led_cdev);
	struct aat1290_led *led = fled_cdev_to_led(fled_cdev);

	aat1290_brightness_set(led, brightness);

	return 0;
}

static int aat1290_led_flash_strobe_set(struct led_classdev_flash *fled_cdev,
					 bool state)

{
	struct aat1290_led *led = fled_cdev_to_led(fled_cdev);
	struct led_classdev *led_cdev = &fled_cdev->led_cdev;
	struct led_flash_setting *timeout = &fled_cdev->timeout;

	mutex_lock(&led->lock);

	if (state == 0) {
		gpio_set_value(led->flen_gpio, 0);
		gpio_set_value(led->en_set_gpio, 0);
		goto unlock;
	}

	aat1290_set_flash_safety_timer(led, timeout->val);

	/*
	 * To reenter movie mode after a flash event the part
	 * must be cycled off and back on to reset the movie
	 * mode and reprogrammed via the AS2Cwire. Therefore
	 * the brightness value needs to be updated here to
	 * reflect the actual state.
	 */
	led_cdev->brightness = 0;
	led->movie_mode = false;

	gpio_set_value(led->flen_gpio, 1);

unlock:
	mutex_unlock(&led->lock);

	return 0;
}

static int aat1290_led_flash_timeout_set(struct led_classdev_flash *fled_cdev,
						u32 timeout)
{
	/*
	 * Don't do anything - flash timeout is cached in the led-class-flash
	 * core and will be applied in the strobe_set op, as writing the
	 * safety timer register spuriously turns the torch mode on.
	 */

	return 0;
}

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
static int aat1290_led_external_strobe_set(struct v4l2_flash *v4l2_flash,
						bool enable)
{
	struct aat1290_led *led = fled_cdev_to_led(v4l2_flash->fled_cdev);

	gpio_set_value(led->ext_strobe_gpio, enable);

	return 0;
}
#endif

static int aat1290_led_parse_dt(struct aat1290_led *led,
				struct device *dev)
{
	struct device_node *node = dev->of_node, *child_node;
	int cnt_leds = 0, ret;

	for_each_available_child_of_node(node, child_node) {
		ret = of_property_read_string(child_node, "label",
					(const char **) &led->label);
		if (ret < 0) {
			dev_err(dev,
				"Error reading \"label\" DT property.\n");
			return ret;
		}

		ret = of_property_read_u32(child_node, "flash-timeout-us",
					&led->max_flash_tm);
		if (ret < 0)
			dev_err(dev,
				"Error reading \"flash-timeout-us\" DT property\n");

		++cnt_leds;
	}

	if (cnt_leds != 1) {
		dev_err(dev,
			"Expected one DT child node for the connected LED.\n");
		return -EINVAL;
	}


	return ret;
}

static void aat1290_init_flash_settings(struct aat1290_led *led,
					 struct aat1290_led_settings *s)
{
	struct led_flash_setting *setting;

	/* Init flash intensity setting */
	setting = &s->torch_brightness;
	/*
	 * Torch current is adjustable in logarithmic fashion and thus
	 * it is not possible to define fixed step in microamperes.
	 * Instead led brightness levels are used to make possible
	 * setting all the supported levels from V4L2 Flash sub-device.
	 */
	setting->min = 1;
	setting->max = AAT1290_MAX_MM_CURR_PERCENT_0 -
		       AAT1290_MAX_MM_CURR_PERCENT_100;
	setting->step = 1;
	setting->val = setting->max;

	/* Init flash timeout setting */
	setting = &s->flash_timeout;
	setting->min = led->max_flash_tm / AAT1290_FLASH_TM_NUM_LEVELS;
	setting->max = setting->min * AAT1290_FLASH_TM_NUM_LEVELS;
	setting->step = setting->min;
	setting->val = setting->max;
}

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
static void aat1290_init_v4l2_ctrl_config(struct aat1290_led *led,
					struct aat1290_led_settings *s,
					struct v4l2_flash_ctrl_config *config)
{
	struct led_flash_setting *setting;
	struct v4l2_ctrl_config *c;

	c = &config->intensity;
	setting = &s->torch_brightness;
	c->min = setting->min;
	c->max = setting->max;
	c->step = setting->step;
	c->def = setting->val;

	c = &config->flash_timeout;
	setting = &s->flash_timeout;
	c->min = setting->min;
	c->max = setting->max;
	c->step = setting->step;
	c->def = setting->val;

	config->has_external_strobe = gpio_is_valid(led->ext_strobe_gpio);
}
#else
#define aat1290_init_v4l2_ctrl_config(s, config)
#endif

static const struct led_flash_ops flash_ops = {
	.strobe_set = aat1290_led_flash_strobe_set,
	.timeout_set = aat1290_led_flash_timeout_set,
};

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
static const struct v4l2_flash_ops v4l2_flash_ops = {
	.external_strobe_set = aat1290_led_external_strobe_set,
};
#endif

static int aat1290_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dev_node = pdev->dev.of_node;
	struct aat1290_led *led;
	struct led_classdev *led_cdev;
	struct led_classdev_flash *fled_cdev;
#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
	struct v4l2_flash_ctrl_config v4l2_flash_config;
#endif
	struct aat1290_led_settings settings;
	int flen_gpio, enset_gpio, ext_strobe_gpio, ret;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->pdev = pdev;
	platform_set_drvdata(pdev, led);

	if (!dev_node)
		return -ENXIO;

	flen_gpio = of_get_gpio(dev_node, 0);
	if (gpio_is_valid(flen_gpio)) {
		ret = devm_gpio_request_one(dev, flen_gpio, GPIOF_DIR_OUT,
						"aat1290_flen");
		if (ret < 0) {
			dev_err(dev,
				"failed to request GPIO %d, error %d\n",
							flen_gpio, ret);
			return ret;
		}
	}
	led->flen_gpio = flen_gpio;

	enset_gpio = of_get_gpio(dev_node, 1);
	if (gpio_is_valid(enset_gpio)) {
		ret = devm_gpio_request_one(dev, enset_gpio, GPIOF_DIR_OUT,
						"aat1290_en_set");
		if (ret < 0) {
			dev_err(dev,
				"failed to request GPIO %d, error %d\n",
							enset_gpio, ret);
			return ret;
		}
	}
	led->en_set_gpio = enset_gpio;

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
	ext_strobe_gpio = of_get_gpio(dev_node, 2);
	if (gpio_is_valid(ext_strobe_gpio)) {
		ret = devm_gpio_request_one(dev, ext_strobe_gpio, GPIOF_DIR_OUT,
						"aat1290_en_hw_strobe");
		if (ret < 0) {
			dev_err(dev,
				"failed to request GPIO %d, error %d\n",
							ext_strobe_gpio, ret);
			return ret;
		}
	}
	led->ext_strobe_gpio = ext_strobe_gpio;
#endif

	ret = aat1290_led_parse_dt(led, &pdev->dev);
	if (ret < 0)
		return ret;

	fled_cdev = &led->fled_cdev;

	/* Init flash settings */
	aat1290_init_flash_settings(led, &settings);

	fled_cdev->timeout = settings.flash_timeout;

	/* Init V4L2 Flash controls basing on initialized settings */
	aat1290_init_v4l2_ctrl_config(led, &settings, &v4l2_flash_config);

	/* Init led class */
	led_cdev = &fled_cdev->led_cdev;
	led_cdev->name = led->label;
	led_cdev->brightness_set = aat1290_led_brightness_set;
	led_cdev->brightness_set_sync = aat1290_led_brightness_set_sync;
	led_cdev->max_brightness = settings.torch_brightness.max;
	led_cdev->flags |= LED_DEV_CAP_FLASH;

	INIT_WORK(&led->work_brightness_set, aat1290_brightness_set_work);

	fled_cdev->ops = &flash_ops;

	/* Register in the LED subsystem. */
	ret = led_classdev_flash_register(&pdev->dev, fled_cdev);
	if (ret < 0)
		return ret;

	mutex_init(&led->lock);

	of_node_get(dev_node);
	led_cdev->dev->of_node = dev_node;

	/* Create V4L2 Flash subdev. */
	led->v4l2_flash = v4l2_flash_init(fled_cdev,
					  &v4l2_flash_ops,
					  &v4l2_flash_config);
	if (IS_ERR(led->v4l2_flash)) {
		ret = PTR_ERR(led->v4l2_flash);
		goto error_v4l2_flash_init;
	}

	return 0;

error_v4l2_flash_init:
	of_node_put(dev_node);
	led_classdev_flash_unregister(fled_cdev);
	mutex_destroy(&led->lock);

	return ret;
}

static int aat1290_led_remove(struct platform_device *pdev)
{
	struct aat1290_led *led = platform_get_drvdata(pdev);

	v4l2_flash_release(led->v4l2_flash);
	of_node_put(led->fled_cdev.led_cdev.dev->of_node);
	led_classdev_flash_unregister(&led->fled_cdev);
	cancel_work_sync(&led->work_brightness_set);

	mutex_destroy(&led->lock);

	return 0;
}

static struct of_device_id aat1290_led_dt_match[] = {
	{.compatible = "skyworks,aat1290"},
	{},
};

static struct platform_driver aat1290_led_driver = {
	.probe		= aat1290_led_probe,
	.remove		= aat1290_led_remove,
	.driver		= {
		.name	= "aat1290-led",
		.owner	= THIS_MODULE,
		.of_match_table = aat1290_led_dt_match,
	},
};

module_platform_driver(aat1290_led_driver);

MODULE_AUTHOR("Jacek Anaszewski <j.anaszewski@samsung.com>");
MODULE_DESCRIPTION("Skyworks Current Regulator for Flash LEDs");
MODULE_LICENSE("GPL v2");
