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
#include <linux/led-class-flash.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <media/v4l2-flash.h>

#define AAT1290_MOVIE_MODE_CURRENT_ADDR	17
#define AAT1290_MAX_MM_CURR_PERCENT_0	16
#define AAT1290_MAX_MM_CURR_PERCENT_100	1

#define AAT1290_FLASH_SAFETY_TIMER_ADDR	18

#define AAT1290_MOVIE_MODE_CONFIG_ADDR	19
#define AAT1290_MOVIE_MODE_OFF		1
#define AAT1290_MOVIE_MODE_ON		3

#define AAT1290_MM_CURRENT_RATIO_ADDR	20
#define AAT1290_MM_TO_FL_1_92		1

#define AAT1290_MM_TO_FL_RATIO		1000 / 1920
#define AAT1290_MAX_MM_CURRENT(fl_max)	(fl_max * AAT1290_MM_TO_FL_RATIO)

#define AAT1290_LATCH_TIME_MIN_US	500
#define AAT1290_LATCH_TIME_MAX_US	1000
#define AAT1290_EN_SET_TICK_TIME_US	1
#define AAT1290_FLEN_OFF_DELAY_TIME_US	10
#define AAT1290_FLASH_TM_NUM_LEVELS	16
#define AAT1290_MM_CURRENT_SCALE_SIZE	15

#define AAT1290_NAME			"aat1290"

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
/* Number of AAT1290 devices in the system. */
static unsigned int dev_count;
#endif


struct aat1290_led_settings {
	struct led_flash_setting torch_brightness;
	struct led_flash_setting flash_timeout;
};

struct aat1290_led {
	/* platform device data */
	struct platform_device *pdev;
	/* secures access to the device */
	struct mutex lock;

	/* related LED Flash class device */
	struct led_classdev_flash fled_cdev;
	/* V4L2 Flash device */
	struct v4l2_flash *v4l2_flash;

	/* FLEN pin */
	int flen_gpio;
	/* EN|SET pin  */
	int en_set_gpio;
#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
	/* movie mode current scale */
	int *mm_current_scale;
#endif

	/* maximum flash timeout */
	u32 max_flash_tm;
	/* maximum LED current in flash mode */
	u32 max_flash_current;
	/* device mode */
	bool movie_mode;

	/* brightness cache */
	unsigned int torch_brightness;
	/* assures led-triggers compatibility */
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
		led->movie_mode = false;
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
		goto done;
	}

	aat1290_set_flash_safety_timer(led, timeout->val);

	gpio_set_value(led->flen_gpio, 1);

done:
	/*
	 * To reenter movie mode after a flash event the part must be cycled
	 * off and back on to reset the movie mode and reprogrammed via the
	 * AS2Cwire. Therefore the brightness and movie_mode properties needs
	 * to be updated here to reflect the actual state.
	 */
	led_cdev->brightness = 0;
	led->movie_mode = false;

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

static int aat1290_led_parse_dt(struct aat1290_led *led,
			struct device_node **sub_node,
			struct v4l2_flash_ctrl_config *v4l2_flash_config)
{
	struct led_classdev *led_cdev = &led->fled_cdev.led_cdev;
	struct device *dev = &led->pdev->dev;
	struct device_node *dev_node = dev->of_node, *child_node;
	int flen_gpio, enset_gpio;
#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
	struct pinctrl *pinctrl;
#endif
	int ret = 0;

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
	} else {
		return -EINVAL;
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
	} else {
		return -EINVAL;
	}
	led->en_set_gpio = enset_gpio;

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
	pinctrl = devm_pinctrl_get_select_default(&led->pdev->dev);
	if (IS_ERR(pinctrl)) {
		v4l2_flash_config->has_external_strobe = false;
		dev_info(dev,
			 "No support for external strobe detected.\n");
	} else {
		v4l2_flash_config->has_external_strobe = true;
	}
#endif

	child_node = of_get_next_available_child(dev->of_node, NULL);
	if (!child_node) {
		dev_err(dev, "No DT child node found for connected LED.\n");
		return -EINVAL;
	}

	of_property_read_string(child_node, "label",
				(const char **) &led_cdev->name);

	ret = of_property_read_u32(child_node, "flash-max-microamp",
				&led->max_flash_current);
	if (ret < 0) {
		dev_err(dev,
			"Error reading \"flash-max-microamp\" DT property\n");
		return ret;
	}

	ret = of_property_read_u32(child_node, "flash-timeout-us",
				&led->max_flash_tm);
	if (ret < 0) {
		dev_err(dev,
			"Error reading \"flash-timeout-us\" DT property\n");
		return ret;
	}

	*sub_node = child_node;

	return ret;
}

static void aat1290_init_flash_settings(struct aat1290_led *led,
					 struct aat1290_led_settings *s)
{
	struct led_flash_setting *setting;

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
	/* Init flash intensity setting */
	setting = &s->torch_brightness;
	setting->min = led->mm_current_scale[0];
	setting->max = led->mm_current_scale[AAT1290_MM_CURRENT_SCALE_SIZE - 1];
	setting->step = 1;
	setting->val = setting->max;
#endif

	/* Init flash timeout setting */
	setting = &s->flash_timeout;
	setting->min = led->max_flash_tm / AAT1290_FLASH_TM_NUM_LEVELS;
	setting->max = setting->min * AAT1290_FLASH_TM_NUM_LEVELS;
	setting->step = setting->min;
	setting->val = setting->max;
}

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
enum led_brightness aat1290_intensity_to_brightness(
					struct v4l2_flash *v4l2_flash,
					s32 intensity)
{
	struct led_classdev_flash *fled_cdev = v4l2_flash->fled_cdev;
	struct aat1290_led *led = fled_cdev_to_led(fled_cdev);
	int i;

	for (i = AAT1290_MM_CURRENT_SCALE_SIZE - 1; i >= 0; --i)
		if (intensity >= led->mm_current_scale[i])
			return i + 1;

	return 1;
}

s32 aat1290_brightness_to_intensity(struct v4l2_flash *v4l2_flash,
					enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = v4l2_flash->fled_cdev;
	struct aat1290_led *led = fled_cdev_to_led(fled_cdev);

	return led->mm_current_scale[brightness - 1];
}

static int aat1290_led_external_strobe_set(struct v4l2_flash *v4l2_flash,
						bool enable)
{
	struct aat1290_led *led = fled_cdev_to_led(v4l2_flash->fled_cdev);
	struct led_classdev_flash *fled_cdev = v4l2_flash->fled_cdev;
	struct led_classdev *led_cdev = &fled_cdev->led_cdev;
	struct pinctrl *pinctrl;

	gpio_set_value(led->flen_gpio, 0);
	gpio_set_value(led->en_set_gpio, 0);

	led->movie_mode = false;
	led_cdev->brightness = 0;

	pinctrl = devm_pinctrl_get_select(&led->pdev->dev,
						enable ? "isp" : "host");
	if (IS_ERR(pinctrl)) {
		dev_warn(&led->pdev->dev, "Unable to switch strobe source.\n");
		return PTR_ERR(pinctrl);
	}

	return 0;
}

int init_mm_current_scale(struct aat1290_led *led)
{
	int max_mm_current_percent[] = { 20, 22, 25, 28, 32, 36, 40, 45, 50, 56,
						63, 71, 79, 89, 100 };
	int i, max_mm_current = AAT1290_MAX_MM_CURRENT(led->max_flash_current);

	led->mm_current_scale = devm_kzalloc(&led->pdev->dev,
						sizeof(max_mm_current_percent),
						GFP_KERNEL);
	if (!led->mm_current_scale)
		return -ENOMEM;

	for (i = 0; i < AAT1290_MM_CURRENT_SCALE_SIZE; ++i)
		led->mm_current_scale[i] = max_mm_current *
					  max_mm_current_percent[i] / 100;

	return 0;
}

static void aat1290_init_v4l2_ctrl_config(struct aat1290_led *led,
					struct aat1290_led_settings *s,
					struct v4l2_flash_ctrl_config *config)
{
	struct led_flash_setting *setting;
	struct v4l2_ctrl_config *c;
	char suffix[10];

	if (++dev_count > 1)
		snprintf(suffix, sizeof(suffix), "_%d", dev_count);

	snprintf(config->dev_name, sizeof(config->dev_name), "%s%s",
		 AAT1290_NAME, dev_count > 1 ? suffix : "");

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
}

static const struct v4l2_flash_ops v4l2_flash_ops = {
	.external_strobe_set = aat1290_led_external_strobe_set,
	.intensity_to_led_brightness = aat1290_intensity_to_brightness,
	.led_brightness_to_intensity = aat1290_brightness_to_intensity,
};
#else
#define aat1290_init_v4l2_ctrl_config(led, s, config)
#define init_mm_current_scale(led) (0)
#endif

static const struct led_flash_ops flash_ops = {
	.strobe_set = aat1290_led_flash_strobe_set,
	.timeout_set = aat1290_led_flash_timeout_set,
};

static int aat1290_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aat1290_led *led;
	struct led_classdev *led_cdev;
	struct device_node *sub_node = NULL;
	struct led_classdev_flash *fled_cdev;
	struct aat1290_led_settings settings;
	struct v4l2_flash_ctrl_config v4l2_flash_config = {};
	int ret;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->pdev = pdev;
	platform_set_drvdata(pdev, led);

	fled_cdev = &led->fled_cdev;
	led_cdev = &fled_cdev->led_cdev;

	ret = aat1290_led_parse_dt(led, &sub_node, &v4l2_flash_config);
	if (ret < 0)
		return ret;

	if (!led_cdev->name)
		led_cdev->name = AAT1290_NAME;

	/*
	 * Init non-linear movie mode current scale basing
	 * on the max flash current from Device Tree binding.
	 */
	ret = init_mm_current_scale(led);
	if (ret < 0)
		return ret;

	/* Init flash settings */
	aat1290_init_flash_settings(led, &settings);

	/* Init V4L2 Flash controls basing on initialized settings */
	aat1290_init_v4l2_ctrl_config(led, &settings, &v4l2_flash_config);

	fled_cdev->timeout = settings.flash_timeout;
	fled_cdev->ops = &flash_ops;

	/* Init LED class */
	led_cdev->brightness_set = aat1290_led_brightness_set;
	led_cdev->brightness_set_sync = aat1290_led_brightness_set_sync;
	led_cdev->max_brightness = AAT1290_MM_CURRENT_SCALE_SIZE;
	led_cdev->flags |= LED_DEV_CAP_FLASH;

	INIT_WORK(&led->work_brightness_set, aat1290_brightness_set_work);

	/* Register in the LED subsystem. */
	ret = led_classdev_flash_register(&pdev->dev, fled_cdev);
	if (ret < 0)
		return ret;

	mutex_init(&led->lock);

	led_cdev->dev->of_node = sub_node;

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
	led_classdev_flash_unregister(fled_cdev);
	mutex_destroy(&led->lock);

	return ret;
}

static int aat1290_led_remove(struct platform_device *pdev)
{
	struct aat1290_led *led = platform_get_drvdata(pdev);

	v4l2_flash_release(led->v4l2_flash);
	led_classdev_flash_unregister(&led->fled_cdev);
	cancel_work_sync(&led->work_brightness_set);

	mutex_destroy(&led->lock);

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
	--dev_count;
#endif

	return 0;
}

static const struct of_device_id aat1290_led_dt_match[] = {
	{.compatible = "skyworks,aat1290"},
	{},
};

static struct platform_driver aat1290_led_driver = {
	.probe		= aat1290_led_probe,
	.remove		= aat1290_led_remove,
	.driver		= {
		.name	= "aat1290",
		.owner	= THIS_MODULE,
		.of_match_table = aat1290_led_dt_match,
	},
};

module_platform_driver(aat1290_led_driver);

MODULE_AUTHOR("Jacek Anaszewski <j.anaszewski@samsung.com>");
MODULE_DESCRIPTION("Skyworks Current Regulator for Flash LEDs");
MODULE_LICENSE("GPL v2");
