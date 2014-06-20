/*
 *  drivers/extcon/extcon_gpio.c
 *
 *  Single-state GPIO extcon driver based on extcon class
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * Modified by MyungJoo Ham <myungjoo.ham@samsung.com> to support extcon
 * (originally switch class is supported)
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
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/extcon.h>
#include <linux/extcon/extcon-gpio.h>

struct gpio_extcon_data {
	struct extcon_dev edev;
	unsigned gpio;
	const char *state_on;
	const char *state_off;
	int irq;
	struct delayed_work work;
	unsigned long debounce_jiffies;
};

static void gpio_extcon_work(struct work_struct *work)
{
	int state;
	struct gpio_extcon_data	*data =
		container_of(to_delayed_work(work), struct gpio_extcon_data,
			     work);

	state = gpio_get_value(data->gpio);

	dev_dbg(data->edev.dev, "gpio_extcon_work state: %d", state);

	extcon_set_state(&data->edev, state);
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_extcon_data *extcon_data = dev_id;

	dev_dbg(extcon_data->edev.dev, "***** gpio_irq_handler *****");

	schedule_delayed_work(&extcon_data->work,
			      extcon_data->debounce_jiffies);
	return IRQ_HANDLED;
}

static ssize_t extcon_gpio_print_state(struct extcon_dev *edev, char *buf)
{
	struct gpio_extcon_data	*extcon_data =
		container_of(edev, struct gpio_extcon_data, edev);
	const char *state;
	if (extcon_get_state(edev))
		state = extcon_data->state_on;
	else
		state = extcon_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -EINVAL;
}

static struct gpio_extcon_platform_data *
gpio_extcon_parse_dt(struct platform_device *pdev)
{
	struct gpio_extcon_platform_data *pdata;
	struct device_node *np = pdev->dev.of_node;
	u32 val;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata),
			     GFP_KERNEL);
	if (pdata == NULL)
		return -ENOMEM;

	pdata->gpio = of_get_gpio(np, 0);
	if (pdata->gpio == -EPROBE_DEFER)
		return ERR_PTR(-EPROBE_DEFER);

	if (of_property_read_u32(np, "irq_flags", &val) == 0)
		pdata->irq_flags = val;

	if (of_property_read_u32(np, "debounce", &val) == 0)
		pdata->debounce = val;

	pdata->name = np->name;
	return pdata;
}

static int gpio_extcon_probe(struct platform_device *pdev)
{
	struct gpio_extcon_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_extcon_data *extcon_data;
	int ret = 0;

	if (!pdata && pdev->dev.of_node) {
		pdata = gpio_extcon_parse_dt(pdev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}
	if (!pdata)
		return -EBUSY;
	if (!pdata->irq_flags) {
		dev_err(&pdev->dev, "IRQ flag is not specified.\n");
		return -EINVAL;
	}

	extcon_data = devm_kzalloc(&pdev->dev, sizeof(struct gpio_extcon_data),
				   GFP_KERNEL);
	if (!extcon_data)
		return -ENOMEM;

	extcon_data->edev.name = pdata->name;
	extcon_data->gpio = pdata->gpio;
	extcon_data->state_on = pdata->state_on;
	extcon_data->state_off = pdata->state_off;
	if (pdata->state_on && pdata->state_off)
		extcon_data->edev.print_state = extcon_gpio_print_state;
	extcon_data->debounce_jiffies = msecs_to_jiffies(pdata->debounce);

	ret = extcon_dev_register(&extcon_data->edev, &pdev->dev);
	if (ret < 0)
		return ret;

	ret = devm_gpio_request_one(&pdev->dev, extcon_data->gpio, GPIOF_DIR_IN,
				    pdev->name);
	if (ret < 0)
		goto err;

	INIT_DELAYED_WORK(&extcon_data->work, gpio_extcon_work);

	extcon_data->irq = gpio_to_irq(extcon_data->gpio);
	dev_dbg(extcon_data->edev.dev, "name: %s irq: %d", extcon_data->edev.name, extcon_data->irq);

	if (extcon_data->irq < 0) {
		ret = extcon_data->irq;
		goto err;
	}

	ret = request_any_context_irq(extcon_data->irq, gpio_irq_handler,
				      pdata->irq_flags, pdev->name,
				      extcon_data);
	if (ret < 0)
		goto err;

	platform_set_drvdata(pdev, extcon_data);
	/* Perform initial detection */
	gpio_extcon_work(&extcon_data->work.work);

	return 0;

err:
	extcon_dev_unregister(&extcon_data->edev);

	return ret;
}

static int gpio_extcon_remove(struct platform_device *pdev)
{
	struct gpio_extcon_data *extcon_data = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&extcon_data->work);
	free_irq(extcon_data->irq, extcon_data);
	extcon_dev_unregister(&extcon_data->edev);

	return 0;
}

static const struct of_device_id extcon_gpio_of_match[] = {
	{ .compatible = "extcon-gpio" },
	{ },
};
MODULE_DEVICE_TABLE(of, extcon_gpio_of_match);

static struct platform_driver gpio_extcon_driver = {
	.probe		= gpio_extcon_probe,
	.remove		= gpio_extcon_remove,
	.driver		= {
		.name	= "extcon-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(extcon_gpio_of_match),
	},
};

module_platform_driver(gpio_extcon_driver);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO extcon driver");
MODULE_LICENSE("GPL");
