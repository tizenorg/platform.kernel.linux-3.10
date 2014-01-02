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
 * pl111_drm_fbdev.c
 * Implementation of the framebuffer device for PL111 DRM
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>

#include "pl111_drm.h"

#define MAX_CONNECTOR		4
#define PREFERRED_BPP		16

#define to_pl111_fbdev(x)	container_of(x, struct pl111_drm_fbdev,\
				drm_fb_helper)

struct pl111_drm_fbdev {
	struct drm_fb_helper		drm_fb_helper;
	struct pl111_gem_bo		*gem_bo;
};

static struct fb_ops pl111_drm_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_check_var	= drm_fb_helper_check_var,
	.fb_set_par	= drm_fb_helper_set_par,
	.fb_blank	= drm_fb_helper_blank,
	.fb_pan_display	= drm_fb_helper_pan_display,
	.fb_setcmap	= drm_fb_helper_setcmap,
};

static int pl111_drm_fbdev_update(struct drm_fb_helper *helper,
				  struct drm_framebuffer *fb)
{
	struct fb_info *fbi = helper->fbdev;
	struct drm_device *dev = helper->dev;
	struct pl111_gem_bo *bo = PL111_BO_FROM_FRAMEBUFFER(fb);
	struct pl111_gem_bo_dma *bo_dma = &bo->backing_data.dma;
	unsigned int size = fb->width * fb->height * (fb->bits_per_pixel >> 3);
	unsigned long offset;

	drm_fb_helper_fill_fix(fbi, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(fbi, helper, fb->width, fb->height);

	/* map pages with kernel virtual space. */
	if (!bo_dma->fb_cpu_addr) {
		phys_addr_t dma_addr = bo_dma->fb_dev_addr;
		if (dma_addr)
			bo_dma->fb_cpu_addr = phys_to_virt(dma_addr);
		else
			bo_dma->fb_cpu_addr = (void __iomem *)NULL;

		if (!bo_dma->fb_cpu_addr) {
			DRM_ERROR("failed to map pages to kernel space.\n");
			return -EIO;
		}
	}

	offset = fbi->var.xoffset * (fb->bits_per_pixel >> 3);
	offset += fbi->var.yoffset * fb->pitches[0];

	dev->mode_config.fb_base = (resource_size_t)bo_dma->fb_dev_addr;
	fbi->screen_base = bo_dma->fb_cpu_addr + offset;
	fbi->fix.smem_start = (unsigned long)bo_dma->fb_dev_addr;

	fbi->screen_size = size;
	fbi->fix.smem_len = size;

	return 0;
}

static int pl111_drm_fbdev_create(struct drm_fb_helper *helper,
				    struct drm_fb_helper_surface_size *sizes)
{
	struct pl111_drm_fbdev *pl111_fbdev = to_pl111_fbdev(helper);
	struct pl111_gem_bo *gem_bo;
	struct drm_device *dev = helper->dev;
	struct fb_info *fbi;
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct platform_device *pdev = dev->platformdev;
	unsigned long size;
	int ret;

	DRM_DEBUG_KMS("surface width(%d), height(%d) and bpp(%d\n",
			sizes->surface_width, sizes->surface_height,
			sizes->surface_bpp);

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pitches[0] = sizes->surface_width * (sizes->surface_bpp >> 3);
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
							  sizes->surface_depth);

	mutex_lock(&dev->struct_mutex);

	fbi = framebuffer_alloc(0, &pdev->dev);
	if (!fbi) {
		DRM_ERROR("failed to allocate fb info.\n");
		ret = -ENOMEM;
		goto out;
	}

	size = mode_cmd.pitches[0] * mode_cmd.height;

	/* 0 means to allocate physically continuous memory */
	gem_bo = pl111_drm_gem_create(dev, 0, size);
	if (IS_ERR(gem_bo)) {
		ret = PTR_ERR(gem_bo);
		goto err_release_framebuffer;
	}
	pl111_fbdev->gem_bo = gem_bo;

	helper->fb = pl111_drm_fb_init(dev, &mode_cmd,
			&gem_bo->gem_object);
	if (IS_ERR(helper->fb)) {
		DRM_ERROR("failed to create drm framebuffer.\n");
		ret = PTR_ERR(helper->fb);
		goto err_destroy_gem;
	}

	helper->fbdev = fbi;

	fbi->par = helper;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->fbops = &pl111_drm_fb_ops;

	ret = fb_alloc_cmap(&fbi->cmap, 256, 0);
	if (ret) {
		DRM_ERROR("failed to allocate cmap.\n");
		goto err_destroy_framebuffer;
	}

	ret = pl111_drm_fbdev_update(helper, helper->fb);
	if (ret < 0)
		goto err_dealloc_cmap;

	mutex_unlock(&dev->struct_mutex);
	return ret;

err_dealloc_cmap:
	fb_dealloc_cmap(&fbi->cmap);
err_destroy_framebuffer:
	drm_framebuffer_cleanup(helper->fb);
err_destroy_gem:
	pl111_gem_free_object(&gem_bo->gem_object);
err_release_framebuffer:
	framebuffer_release(fbi);

/*
 * if failed, all resources allocated above would be released by
 * drm_mode_config_cleanup() when drm_load() had been called prior
 * to any specific driver such as fimd or hdmi driver.
 */
out:
	mutex_unlock(&dev->struct_mutex);
	return ret;
}

static struct drm_fb_helper_funcs pl111_drm_fb_helper_funcs = {
	.fb_probe =	pl111_drm_fbdev_create,
};

int pl111_drm_fbdev_init(struct drm_device *dev)
{
	struct pl111_drm_fbdev *fbdev;
	struct pl111_drm_dev_private *private = dev->dev_private;
	struct drm_fb_helper *helper;
	unsigned int num_crtc;
	int ret;

	if (!dev->mode_config.num_crtc || !dev->mode_config.num_connector)
		return 0;

	fbdev = kzalloc(sizeof(*fbdev), GFP_KERNEL);
	if (!fbdev)
		return -ENOMEM;

	private->fb_helper = helper = &fbdev->drm_fb_helper;
	helper->funcs = &pl111_drm_fb_helper_funcs;

	num_crtc = dev->mode_config.num_crtc;

	ret = drm_fb_helper_init(dev, helper, num_crtc, MAX_CONNECTOR);
	if (ret < 0) {
		DRM_ERROR("failed to initialize drm fb helper.\n");
		goto err_init;
	}

	ret = drm_fb_helper_single_add_all_connectors(helper);
	if (ret < 0) {
		DRM_ERROR("failed to register drm_fb_helper_connector.\n");
		goto err_setup;

	}

	/* disable all the possible outputs/crtcs before entering KMS mode */
	drm_helper_disable_unused_functions(dev);

	ret = drm_fb_helper_initial_config(helper, PREFERRED_BPP);
	if (ret < 0) {
		DRM_ERROR("failed to set up hw configuration.\n");
		goto err_setup;
	}

	return 0;

err_setup:
	drm_fb_helper_fini(helper);

err_init:
	private->fb_helper = NULL;
	kfree(fbdev);

	return ret;
}

static void pl111_drm_fbdev_destroy(struct drm_device *dev,
				      struct drm_fb_helper *fb_helper)
{
	struct pl111_drm_fbdev *pl111_fbd = to_pl111_fbdev(fb_helper);
	struct pl111_gem_bo *gem_bo = pl111_fbd->gem_bo;
	struct drm_framebuffer *fb;

	/* release drm framebuffer and real buffer */
	if (fb_helper->fb && fb_helper->fb->funcs) {
		fb = fb_helper->fb;
		if (fb) {
			drm_framebuffer_unregister_private(fb);
			drm_framebuffer_remove(fb);
		}
	}

	/* release linux framebuffer */
	if (fb_helper->fbdev) {
		struct fb_info *info;
		int ret;

		info = fb_helper->fbdev;
		ret = unregister_framebuffer(info);
		if (ret < 0)
			DRM_DEBUG_KMS("failed unregister_framebuffer()\n");

		if (info->cmap.len)
			fb_dealloc_cmap(&info->cmap);

		framebuffer_release(info);
	}

	drm_fb_helper_fini(fb_helper);
}

void pl111_drm_fbdev_fini(struct drm_device *dev)
{
	struct pl111_drm_dev_private *private = dev->dev_private;
	struct pl111_drm_fbdev *fbdev;

	if (!private || !private->fb_helper)
		return;

	fbdev = to_pl111_fbdev(private->fb_helper);

	if (fbdev->gem_bo)
		pl111_gem_free_object(&fbdev->gem_bo->gem_object);

	pl111_drm_fbdev_destroy(dev, private->fb_helper);
	kfree(fbdev);
	private->fb_helper = NULL;
}

void pl111_drm_fbdev_restore_mode(struct drm_device *dev)
{
	struct pl111_drm_dev_private *private = dev->dev_private;

	if (!private || !private->fb_helper)
		return;

	drm_modeset_lock_all(dev);
	drm_fb_helper_restore_fbdev_mode(private->fb_helper);
	drm_modeset_unlock_all(dev);
}
