/*
 * sec-system-rev.c - Read Hardware revision of board from gpios pin
 *
 * Copyright (C) 2014 Samsung Electronics
 * Chanwoo Choi <cw00.choi@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/of_gpio.h>

#include <asm/system_info.h>

#define HW_REV_GPIOS	4

static int sec_system_rev_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int i, ret, hw_rev[HW_REV_GPIOS];
	enum of_gpio_flags flags;
	char gpio_name[10];

	if (!of_find_property(np, "gpios", NULL)) {
		dev_warn(&pdev->dev, "Failed to get system revision\n");
		return -EINVAL;
	}

	system_rev = 0;

	for (i = 0; i < HW_REV_GPIOS; i++) {
		hw_rev[i] = of_get_gpio_flags(np, i, &flags);
		if (hw_rev[i] < 0) {
			dev_err(&pdev->dev,
				"Failed to get gpio flags, err: %d\n",
				hw_rev[i]);
			return hw_rev[i];
		}
		snprintf(gpio_name, sizeof(gpio_name), "HW_REV%d", i);

		ret = devm_gpio_request(&pdev->dev, hw_rev[i], gpio_name);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to request gpio\n");
			return ret;
		}
		system_rev |= (gpio_get_value(hw_rev[i]) << i);
	}

	dev_dbg(&pdev->dev, "H/W Revision : 0x%x\n", system_rev);

	return 0;
}

static struct of_device_id sec_system_rev_of_match[] = {
	{ .compatible = "samsung,sec-system-rev", },
	{ /* sentinel */ },
};

static struct platform_driver sec_system_rev_device_driver = {
	.probe		= sec_system_rev_probe,
	.driver		= {
		.name	= "sec-system-rev",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(sec_system_rev_of_match),
	}
};

static int __init sec_system_rev_init(void)
{
	return platform_driver_register(&sec_system_rev_device_driver);
}
subsys_initcall(sec_system_rev_init);

static void __exit sec_system_rev_exit(void)
{
	platform_driver_unregister(&sec_system_rev_device_driver);
}
module_exit(sec_system_rev_exit);
