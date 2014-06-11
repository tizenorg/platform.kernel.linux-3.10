/*
 * S3C RTC Tick counter driver
 *
 * Jonghwa Lee <jonghwa3.lee@samsung.com>
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>

static int 
