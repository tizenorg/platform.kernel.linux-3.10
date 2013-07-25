/*
 * (C) COPYRIGHT 2012-2013 ARM Limited. All rights reserved.
 *
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
 * pl111_drm_fb.c
 * Implementation of the framebuffer functions for PL111 DRM
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

static void pl111_fb_destroy(struct drm_framebuffer *framebuffer)
{
	struct pl111_drm_framebuffer *pl111_fb;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct drm_crtc *crtc;
#endif
	DRM_DEBUG_KMS("Destroying framebuffer 0x%p...\n", framebuffer);

	pl111_fb = PL111_FB_FROM_FRAMEBUFFER(framebuffer);

	/*
	 * Because flips are deferred, wait for all previous flips to complete
	 */
	wait_event(priv.wait_for_flips,
			atomic_read(&priv.nr_flips_in_flight) == 0);
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	/*
	 * Release KDS resources if it's currently being displayed. Only occurs
	 * when the last framebuffer is destroyed.
	 */
	list_for_each_entry(crtc, &framebuffer->dev->mode_config.crtc_list,
				head) {
		struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(crtc);
		spin_lock(&pl111_crtc->current_displaying_lock);
		if (pl111_crtc->displaying_fb == framebuffer) {
			/* Release the current buffers */
			if (pl111_crtc->old_kds_res_set != NULL) {
				DRM_DEBUG_KMS("Releasing KDS resources for ");
				DRM_DEBUG_KMS("displayed 0x%p\n", framebuffer);
				kds_resource_set_release(
					&pl111_crtc->old_kds_res_set);
			}
			pl111_crtc->old_kds_res_set = NULL;
		}
		spin_unlock(&pl111_crtc->current_displaying_lock);
	}
#endif
	drm_framebuffer_cleanup(framebuffer);

	if ((pl111_fb->bo != NULL) && (&pl111_fb->bo->gem_object != NULL))
		drm_gem_object_unreference_unlocked(&pl111_fb->bo->gem_object);

	kfree(pl111_fb);

	DRM_DEBUG_KMS("Destroyed framebuffer 0x%p\n", framebuffer);
}

static int pl111_fb_create_handle(struct drm_framebuffer *fb,
				struct drm_file *file_priv,
				unsigned int *handle)
{
	struct pl111_gem_bo *bo = PL111_BO_FROM_FRAMEBUFFER(fb);
	DRM_DEBUG_KMS("DRM %s on fb=%p\n", __func__, fb);

	if (bo == NULL)
		return -EINVAL;

	return drm_gem_handle_create(file_priv, &bo->gem_object, handle);
}

const struct drm_framebuffer_funcs fb_funcs = {
	.destroy = pl111_fb_destroy,
	.create_handle = pl111_fb_create_handle,
};

struct drm_framebuffer *pl111_fb_create(struct drm_device *dev,
					struct drm_file *file_priv,
					struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct pl111_drm_framebuffer *pl111_fb = NULL;
	struct drm_framebuffer *fb = NULL;
	struct drm_gem_object *gem_obj;
	struct pl111_gem_bo *bo;

	pr_info("DRM %s\n", __func__);
	gem_obj = drm_gem_object_lookup(dev, file_priv, mode_cmd->handles[0]);
	if (gem_obj == NULL) {
		DRM_ERROR("Could not get gem obj from handle to create fb\n");
		goto out;
	}

	bo = PL111_BO_FROM_GEM(gem_obj);
	/* Don't even attempt PL111_BOT_SHM, it's not contiguous */
	BUG_ON(bo->type != PL111_BOT_DMA);

	switch ((char)(mode_cmd->pixel_format & 0xFF)) {
	case 'Y':
	case 'U':
	case 'V':
	case 'N':
	case 'T':
		DRM_ERROR("YUV formats not supported\n");
		goto out;
	}

	pl111_fb = kzalloc(sizeof(struct pl111_drm_framebuffer), GFP_KERNEL);
	if (pl111_fb == NULL) {
		DRM_ERROR("Could not allocate pl111_drm_framebuffer\n");
		goto out;
	}
	fb = &pl111_fb->fb;

	if (drm_framebuffer_init(dev, fb, &fb_funcs)) {
		DRM_ERROR("drm_framebuffer_init failed\n");
		kfree(fb);
		fb = NULL;
		goto out;
	}

	drm_helper_mode_fill_fb_struct(fb, mode_cmd);

	PL111_BO_TO_FRAMEBUFFER(fb, bo);

	DRM_DEBUG_KMS("Created fb 0x%p for gem_obj 0x%p physaddr=0x%.8x\n",
			fb, gem_obj, bo->backing_data.dma.fb_dev_addr);

out:
	return fb;
}
