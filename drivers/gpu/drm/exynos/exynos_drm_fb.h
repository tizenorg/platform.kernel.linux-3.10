/*
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _EXYNOS_DRM_FB_H_
#define _EXYNOS_DRM_FB_H

#define to_exynos_fb(x)	container_of(x, struct exynos_drm_fb, fb)

struct exynos_drm_partial_pos {
	unsigned int x;
	unsigned int y;
	unsigned int w;
	unsigned int h;
};

/*
 * exynos specific framebuffer structure.
 *
 * @fb: drm framebuffer obejct.
 * @buf_cnt: a buffer count to drm framebuffer.
 * @exynos_gem_obj: array of exynos specific gem object containing a gem object.
 * @part_pos: a position to partial update region.
 */
struct exynos_drm_fb {
	struct drm_framebuffer		fb;
	unsigned int			buf_cnt;
	struct exynos_drm_gem_obj	*exynos_gem_obj[MAX_FB_BUFFER];
	struct exynos_drm_partial_pos	part_pos;
	atomic_t			partial_mode;
};

struct drm_framebuffer *
exynos_drm_framebuffer_init(struct drm_device *dev,
			    struct drm_mode_fb_cmd2 *mode_cmd,
			    struct drm_gem_object *obj);

/* get memory information of a drm framebuffer */
struct exynos_drm_gem_buf *exynos_drm_fb_buffer(struct drm_framebuffer *fb,
						 int index);

void exynos_drm_mode_config_init(struct drm_device *dev);

/* set a buffer count to drm framebuffer. */
void exynos_drm_fb_set_buf_cnt(struct drm_framebuffer *fb,
						unsigned int cnt);

/* get a buffer count to drm framebuffer. */
unsigned int exynos_drm_fb_get_buf_cnt(struct drm_framebuffer *fb);

#endif
