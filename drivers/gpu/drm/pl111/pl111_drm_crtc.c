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
 * pl111_drm_crtc.c
 * Implementation of the CRTC functions for PL111 DRM
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

static int pl111_crtc_num;

static void vsync_worker(struct work_struct *work)
{
	struct pl111_drm_flip_resource *flip_res;
	struct pl111_gem_bo *bo;
	struct pl111_drm_crtc *pl111_crtc;
	struct drm_device *dev;
	int flips_in_flight;
	flip_res =
		container_of(work, struct pl111_drm_flip_resource, vsync_work);

	pl111_crtc = to_pl111_crtc(flip_res->crtc);
	dev = pl111_crtc->crtc.dev;

	DRM_DEBUG_KMS("DRM Finalizing flip_res=%p\n", flip_res);

	bo = PL111_BO_FROM_FRAMEBUFFER(flip_res->fb);
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	if (flip_res->worker_release_kds == true) {
		spin_lock(&pl111_crtc->current_displaying_lock);
		release_kds_resource_and_display(flip_res);
		spin_unlock(&pl111_crtc->current_displaying_lock);
	}
#endif
	/* Release DMA buffer on this flip */
	if (bo->gem_object.export_dma_buf != NULL)
		dma_buf_put(bo->gem_object.export_dma_buf);

	drm_handle_vblank(dev, pl111_crtc->crtc_index);

	/* Wake up any processes waiting for page flip event */
	if (flip_res->event) {
		spin_lock_bh(&dev->event_lock);
		drm_send_vblank_event(dev, pl111_crtc->crtc_index,
					flip_res->event);
		spin_unlock_bh(&dev->event_lock);
	}

	drm_vblank_put(dev, pl111_crtc->crtc_index);

	/*
	 * workqueue.c:process_one_work():
	 * "It is permissible to free the struct work_struct from
	 *  inside the function that is called from it"
	 */
	kmem_cache_free(priv.page_flip_slab, flip_res);

	flips_in_flight = atomic_dec_return(&priv.nr_flips_in_flight);
	if (flips_in_flight == 0 ||
			flips_in_flight == (NR_FLIPS_IN_FLIGHT_THRESHOLD - 1))
		wake_up(&priv.wait_for_flips);

	DRM_DEBUG_KMS("DRM release flip_res=%p\n", flip_res);
}

void pl111_common_irq(struct pl111_drm_crtc *pl111_crtc)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pl111_crtc->base_update_lock, irq_flags);

	if (pl111_crtc->current_update_res != NULL) {
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
		/*
		 * If the lock is not acquired defer completion of the
		 * resource that caused the buffer update
		 */
		pl111_crtc->current_update_res->worker_release_kds =
								true;
		if (0 != spin_trylock(
			&pl111_crtc->current_displaying_lock)) {
			/* release the resource immediately */
			release_kds_resource_and_display(
					pl111_crtc->current_update_res);
			/*
			 * prevent worker from attempting to release
			 * resource again
			 */
			pl111_crtc->current_update_res->
					worker_release_kds = false;
			spin_unlock(&pl111_crtc->
					current_displaying_lock);
		}
#endif
		/*
		 * Release dma_buf and resource
		 * (if not already released)
		 */
		queue_work(pl111_crtc->vsync_wq,
			&pl111_crtc->current_update_res->vsync_work);
		pl111_crtc->current_update_res = NULL;
	}

	if (!list_empty(&pl111_crtc->update_queue)) {
		struct pl111_drm_flip_resource *flip_res;
		/* Remove the head of the list */
		flip_res = list_first_entry(&pl111_crtc->update_queue,
			struct pl111_drm_flip_resource, link);
		list_del(&flip_res->link);
		do_flip_to_res(flip_res);
		/*
		 * current_update_res will be set, so guarentees that
		 * another flip_res coming in gets queued instead of
		 * handled immediately
		 */
	}

	spin_unlock_irqrestore(&pl111_crtc->base_update_lock, irq_flags);
}

void show_framebuffer_on_crtc_cb(void *cb1, void *cb2)
{
	struct pl111_drm_flip_resource *flip_res = cb1;
	struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(flip_res->crtc);

	pl111_crtc->show_framebuffer_cb(cb1, cb2);
}

int show_framebuffer_on_crtc(struct drm_crtc *crtc,
				struct drm_framebuffer *fb, bool page_flip,
				struct drm_pending_vblank_event *event)
{
	struct pl111_gem_bo *bo;
	struct pl111_drm_flip_resource *flip_res;
	int flips_in_flight;
	int old_flips_in_flight;

	crtc->fb = fb;

	bo = PL111_BO_FROM_FRAMEBUFFER(fb);
	if (bo == NULL) {
		DRM_DEBUG_KMS("Failed to get pl111_gem_bo object\n");
		return -EINVAL;
	}

	/* If this is a full modeset, wait for all outstanding flips to complete
	 * before continuing. This avoids unnecessary complication from being
	 * able to queue up multiple modesets and queues of mixed modesets and
	 * page flips.
	 *
	 * Modesets should be uncommon and will not be performant anyway, so
	 * making them synchronous should have negligible performance impact.
	 */
	if (!page_flip) {
		int ret = wait_event_killable(priv.wait_for_flips,
				atomic_read(&priv.nr_flips_in_flight) == 0);
		if (ret)
			return ret;
	}

	/*
	 * There can be more 'early display' flips in flight than there are
	 * buffers, and there is (currently) no explicit bound on the number of
	 * flips. Hence, we need a new allocation for each one.
	 *
	 * Note: this could be optimized down if we knew a bound on the flips,
	 * since an application can only have so many buffers in flight to be
	 * useful/not hog all the memory
	 */
	flip_res = kmem_cache_alloc(priv.page_flip_slab, GFP_KERNEL);
	if (flip_res == NULL) {
		pr_err("kmem_cache_alloc failed to alloc - flip ignored\n");
		return -ENOMEM;
	}

	/*
	 * increment flips in flight, whilst blocking when we reach
	 * NR_FLIPS_IN_FLIGHT_THRESHOLD
	 */
	do {
		/*
		 * Note: use of assign-and-then-compare in the condition to set
		 * flips_in_flight
		 */
		int ret = wait_event_killable(priv.wait_for_flips,
				(flips_in_flight =
					atomic_read(&priv.nr_flips_in_flight))
				< NR_FLIPS_IN_FLIGHT_THRESHOLD);
		if (ret != 0) {
			kmem_cache_free(priv.page_flip_slab, flip_res);
			return ret;
		}

		old_flips_in_flight = atomic_cmpxchg(&priv.nr_flips_in_flight,
					flips_in_flight, flips_in_flight + 1);
	} while (old_flips_in_flight != flips_in_flight);

	flip_res->fb = fb;
	flip_res->crtc = crtc;
	flip_res->page_flip = page_flip;
	flip_res->event = event;
	INIT_WORK(&flip_res->vsync_work, vsync_worker);
	INIT_LIST_HEAD(&flip_res->link);
	DRM_DEBUG_KMS("DRM alloc flip_res=%p\n", flip_res);
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	if (bo->gem_object.export_dma_buf != NULL) {
		struct dma_buf *buf = bo->gem_object.export_dma_buf;
		unsigned long shared[1] = { 0 };
		struct kds_resource *resource_list[1] = {
				get_dma_buf_kds_resource(buf) };
		int err;

		get_dma_buf(buf);
		DRM_DEBUG_KMS("Got dma_buf %p\n", buf);

		/* Wait for the KDS resource associated with this buffer */
		err = kds_async_waitall(&flip_res->kds_res_set,
					&priv.kds_cb, flip_res, fb, 1, shared,
					resource_list);
		BUG_ON(err);
	} else {
		struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(crtc);

		DRM_DEBUG_KMS("No dma_buf for this flip\n");

		/* No dma-buf attached so just call the callback directly */
		flip_res->kds_res_set = NULL;
		pl111_crtc->show_framebuffer_cb(flip_res, fb);
	}
#else
	if (bo->gem_object.export_dma_buf != NULL) {
		struct dma_buf *buf = bo->gem_object.export_dma_buf;

		get_dma_buf(buf);
		DRM_DEBUG_KMS("Got dma_buf %p\n", buf);
	} else {
		DRM_DEBUG_KMS("No dma_buf for this flip\n");
	}

	/* No dma-buf attached to this so just call the callback directly */
	{
		struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(crtc);
		pl111_crtc->show_framebuffer_cb(flip_res, fb);
	}
#endif

	/* For the same reasons as the wait at the start of this function,
	 * wait for the modeset to complete before continuing.
	 */
	if (!page_flip) {
		int ret = wait_event_killable(priv.wait_for_flips,
				flips_in_flight == 0);
		if (ret)
			return ret;
	}

	return 0;
}

int pl111_crtc_page_flip(struct drm_crtc *crtc, struct drm_framebuffer *fb,
			struct drm_pending_vblank_event *event)
{
	DRM_DEBUG_KMS("%s: crtc=%p, fb=%p, event=%p\n",
			__func__, crtc, fb, event);
	return show_framebuffer_on_crtc(crtc, fb, true, event);
}

int pl111_crtc_helper_mode_set(struct drm_crtc *crtc,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode,
				int x, int y, struct drm_framebuffer *old_fb)
{
	int ret;
	struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(crtc);
	struct drm_display_mode *duplicated_mode;

	DRM_DEBUG_KMS("DRM crtc_helper_mode_set, x=%d y=%d bpp=%d\n",
			adjusted_mode->hdisplay, adjusted_mode->vdisplay,
			crtc->fb->bits_per_pixel);

	duplicated_mode = drm_mode_duplicate(crtc->dev, adjusted_mode);
	if (!duplicated_mode)
		return -ENOMEM;

	pl111_crtc->new_mode = duplicated_mode;
	ret = show_framebuffer_on_crtc(crtc, crtc->fb, false, NULL);
	if (ret != 0) {
		pl111_crtc->new_mode = pl111_crtc->current_mode;
		drm_mode_destroy(crtc->dev, duplicated_mode);
	}

	return ret;
}

void pl111_crtc_helper_prepare(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);
}

void pl111_crtc_helper_commit(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);
}

bool pl111_crtc_helper_mode_fixup(struct drm_crtc *crtc,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);

#ifdef CONFIG_ARCH_VEXPRESS
	/*
	 * 1024x768 with more than 16 bits per pixel does not work correctly
	 * on Versatile Express
	 */
	if (mode->hdisplay == 1024 && mode->vdisplay == 768 &&
			crtc->fb->bits_per_pixel > 16) {
		return false;
	}
#endif

	return true;
}

void pl111_crtc_helper_disable(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);
	clcd_disable(crtc);
}

void pl111_crtc_destroy(struct drm_crtc *crtc)
{
	struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(crtc);

	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);

	destroy_workqueue(pl111_crtc->vsync_wq);
	drm_crtc_cleanup(crtc);
	kfree(pl111_crtc);
}

const struct drm_crtc_funcs crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.page_flip = pl111_crtc_page_flip,
	.destroy = pl111_crtc_destroy
};

const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.mode_set = pl111_crtc_helper_mode_set,
	.prepare = pl111_crtc_helper_prepare,
	.commit = pl111_crtc_helper_commit,
	.mode_fixup = pl111_crtc_helper_mode_fixup,
	.disable = pl111_crtc_helper_disable,
};

bool pl111_crtc_is_fb_currently_displayed(struct drm_device *dev,
					struct drm_framebuffer *fb)
{
	struct drm_crtc *crtc;

	if (fb == NULL)
		return false;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(crtc);
		if (pl111_crtc->displaying_fb == fb)
			return true;
	}
	return false;
}

struct pl111_drm_crtc *pl111_crtc_create(struct drm_device *dev)
{
	struct pl111_drm_crtc *pl111_crtc;

	pl111_crtc = kzalloc(sizeof(struct pl111_drm_crtc), GFP_KERNEL);
	if (pl111_crtc == NULL) {
		pr_err("Failed to allocated pl111_drm_crtc\n");
		return NULL;
	}

	drm_crtc_init(dev, &pl111_crtc->crtc, &crtc_funcs);
	drm_crtc_helper_add(&pl111_crtc->crtc, &crtc_helper_funcs);

	pl111_crtc->crtc_index = pl111_crtc_num;
	pl111_crtc_num++;
	pl111_crtc->vsync_wq = alloc_ordered_workqueue("pl111_drm_vsync_%d",
					WQ_HIGHPRI, pl111_crtc->crtc_index);
	if (!pl111_crtc->vsync_wq) {
		pr_err("Failed to allocate vsync workqueue\n");
		drm_crtc_cleanup(&pl111_crtc->crtc);
		return NULL;
	}

	pl111_crtc->crtc.enabled = 0;
	pl111_crtc->displaying_fb = NULL;
	pl111_crtc->last_bpp = 0;
	pl111_crtc->current_update_res = NULL;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	pl111_crtc->old_kds_res_set = NULL;
#endif
	pl111_crtc->show_framebuffer_cb = show_framebuffer_on_crtc_cb_internal;
	INIT_LIST_HEAD(&pl111_crtc->update_queue);
	spin_lock_init(&pl111_crtc->current_displaying_lock);
	spin_lock_init(&pl111_crtc->base_update_lock);

	return pl111_crtc;
}

