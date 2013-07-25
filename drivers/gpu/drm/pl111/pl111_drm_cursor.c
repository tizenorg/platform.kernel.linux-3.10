/*
 * (C) COPYRIGHT 2012-2013 ARM Limited. All rights reserved.
 *
 * Parts of this file were based on sources as follows:
 *
 * Copyright (c) 2006-2008 Intel Corporation
 * Copyright (c) 2007 Dave Airlie <airlied@linux.ie>
 * Copyright (C) 2011 Texas Instruments
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms of
 * such GNU licence.
 *
 */

/**
 * pl111_drm_cursor.c
 * Implementation of cursor functions for PL111 DRM
 */
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <linux/module.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include "pl111_drm.h"

#define PL111_MAX_CURSOR_WIDTH (64)
#define PL111_MAX_CURSOR_HEIGHT (64)

static int pl111_drm_cursor_plane_disable(struct drm_plane *plane)
{
	pl111_cursor_disable();
	return 0;
}

static int pl111_drm_cursor_plane_update(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		int crtc_x, int crtc_y,
		unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y,
		uint32_t src_w, uint32_t src_h)
{
	struct pl111_gem_bo *bo = PL111_BO_FROM_FRAMEBUFFER(fb);

	/*
	 * TODO Find out if there is a way to know if the image needs changing.
	 * The cursor API might be better for us than planes as that has
	 * distinct set cursor image and set cursor position call backs.
	 */

	pl111_set_cursor_image(bo->backing_data.dma.fb_cpu_addr);

	pl111_cursor_enable();
	pl111_set_cursor_position(crtc_x, crtc_y);

	return 0;
}

void pl111_drm_cursor_plane_destroy(struct drm_plane *plane)
{
	pl111_drm_cursor_plane_disable(plane);
	drm_plane_cleanup(plane);
}

static const struct drm_plane_funcs pl111_drm_cursor_plane_funcs = {
	.update_plane = pl111_drm_cursor_plane_update,
	.disable_plane = pl111_drm_cursor_plane_disable,
	.destroy = pl111_drm_cursor_plane_destroy,
};

/*
 * We don't actually support ARGB8888 for the cursor only PL111 LBBP, the
 * rasult of setting this is that it creates a buffer larger than we actually
 * need. But there are no compatible formats defined in fourcc.h, so we will
 * only read 256 32 bits words from the buffer to set the cursor image.
 * We expect user space to have formatted the buffer correctly to LBBP.
 */
static uint32_t pl111_cursor_formats[] = { DRM_FORMAT_ARGB8888 };

int pl111_cursor_plane_init(struct drm_device *dev,
				struct pl111_drm_cursor_plane *cursor,
				unsigned long possible_crtcs)
{
	cursor->formats = pl111_cursor_formats;
	cursor->num_formats_supported = ARRAY_SIZE(pl111_cursor_formats);

	return drm_plane_init(dev, &cursor->base, possible_crtcs,
			&pl111_drm_cursor_plane_funcs, cursor->formats,
			cursor->num_formats_supported, false);
}


