/*
 * V4L2 Flash LED sub-device registration helpers.
 *
 *	Copyright (C) 2015 Samsung Electronics Co., Ltd
 *	Author: Jacek Anaszewski <j.anaszewski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation."
 */

#ifndef _V4L2_FLASH_H
#define _V4L2_FLASH_H

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>

struct led_classdev_flash;
struct led_classdev;
struct v4l2_flash;
enum led_brightness;

enum ctrl_init_data_id {
	LED_MODE,
	TORCH_INTENSITY,
	INDICATOR_INTENSITY,
	STROBE_SOURCE,
	FLASH_STROBE,
	STROBE_STOP,
	STROBE_STATUS,
	FLASH_TIMEOUT,
	FLASH_INTENSITY,
	FLASH_FAULT,
	SYNC_STROBE,
	NUM_FLASH_CTRLS,
};

/*
 * struct v4l2_flash_ctrl_data - flash control initialization data -
 *				 filled basing on the features declared
 *				 by the LED Flash class driver
 * @config:	initialization data for a control
 * @supported:	indicates whether a control is supported
 *		by the LED Flash class driver
 */
struct v4l2_flash_ctrl_data {
	struct v4l2_ctrl_config config;
	bool supported;
};

struct v4l2_flash_ops {
	/* setup strobing the flash by hardware pin state assertion */
	int (*external_strobe_set)(struct v4l2_flash *v4l2_flash,
					bool enable);
};

/**
 * struct v4l2_flash_ctrl_config - V4L2 Flash controls initialization data
 * @intensity:			constraints for the led in a non-flash mode
 * @flash_intensity:		V4L2_CID_FLASH_INTENSITY settings constraints
 * @flash_timeout:		V4L2_CID_FLASH_TIMEOUT constraints
 * @flash_faults:		possible flash faults
 * @has_external_strobe:	external strobe capability
 * @indicator_led:		signifies that a led is of indicator type
 */
struct v4l2_flash_ctrl_config {
	struct v4l2_ctrl_config intensity;
	struct v4l2_ctrl_config flash_intensity;
	struct v4l2_ctrl_config flash_timeout;
	u32 flash_faults;
	bool has_external_strobe:1;
	bool indicator_led:1;
};

/**
 * struct v4l2_flash - Flash sub-device context
 * @fled_cdev:		LED Flash Class device controlled by this sub-device
 * @ops:		V4L2 specific flash ops
 * @sd:			V4L2 sub-device
 * @hdl:		flash controls handler
 * @ctrls:		array of pointers to controls, whose values define
 *			the sub-device state
 * @sync_strobe_menu	leds available for flash strobe synchronization
 */
struct v4l2_flash {
	struct led_classdev_flash *fled_cdev;
	const struct v4l2_flash_ops *ops;

	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *ctrls[STROBE_SOURCE + 1];
	char **sync_strobe_menu;
};

static inline struct v4l2_flash *v4l2_subdev_to_v4l2_flash(
							struct v4l2_subdev *sd)
{
	return container_of(sd, struct v4l2_flash, sd);
}

static inline struct v4l2_flash *v4l2_ctrl_to_v4l2_flash(struct v4l2_ctrl *c)
{
	return container_of(c->handler, struct v4l2_flash, hdl);
}

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
/**
 * v4l2_flash_init - initialize V4L2 flash led sub-device
 * @fled_cdev:	the LED Flash Class device to wrap
 * @flash_ops:	V4L2 Flash device ops
 * @config:	initialization data for V4L2 Flash controls
 *
 * Create V4L2 subdev wrapping given LED subsystem device.
 *
 * Returns: A valid pointer, or, when an error occurs, the return
 * value is encoded using ERR_PTR(). Use IS_ERR() to check and
 * PTR_ERR() to obtain the numeric return value.
 */
struct v4l2_flash *v4l2_flash_init(struct led_classdev_flash *fled_cdev,
				   const struct v4l2_flash_ops *ops,
				   struct v4l2_flash_ctrl_config *config);

/**
 * v4l2_flash_release - release V4L2 Flash sub-device
 * @flash: the V4L2 Flash device to release
 *
 * Release V4L2 flash led subdev.
 */
void v4l2_flash_release(struct v4l2_flash *v4l2_flash);

#else
#define v4l2_flash_init(fled_cdev, ops, config) (NULL)
#define v4l2_flash_release(v4l2_flash)
#endif /* CONFIG_V4L2_FLASH_LED_CLASS */

#endif /* _V4L2_FLASH_H */
