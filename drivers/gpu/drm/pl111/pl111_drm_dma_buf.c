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
 * pl111_drm_dma_buf.c
 * Implementation of the dma_buf functions for PL111 DRM
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

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
static void obtain_kds_if_currently_displayed(struct drm_device *dev,
						struct drm_framebuffer *fb,
						struct dma_buf *dma_buf)
{
	unsigned long shared[1] = { 0 };
	struct kds_resource *resource_list[1];
	struct kds_resource_set *kds_res_set;
	struct drm_crtc *crtc;
	bool cb_has_called = false;
	int err;
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wake);

	/*
	 * Not all pl111_gem_bo structures have a framebuffer attached - early
	 * out in those cases
	 */
	if (fb == NULL)
		return;

	DRM_DEBUG_KMS("Obtaining initial KDS res for fb:%p bo:%p dma_buf:%p\n",
			fb, PL111_BO_FROM_FRAMEBUFFER(fb), dma_buf);

	resource_list[0] = get_dma_buf_kds_resource(dma_buf);
	get_dma_buf(dma_buf);

	/*
	 * Can't use kds_waitall(), because kbase will be let through due to
	 * locked ignore'
	 */
	err = kds_async_waitall(&kds_res_set,
				&priv.kds_obtain_current_cb, &wake,
				&cb_has_called, 1, shared, resource_list);
	BUG_ON(err);
	wait_event(wake, cb_has_called == true);

	list_for_each_entry(crtc, &fb->dev->mode_config.crtc_list, head) {
		struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(crtc);
		spin_lock(&pl111_crtc->current_displaying_lock);
		if (pl111_crtc->displaying_fb == fb) {
			DRM_DEBUG_KMS("Initial KDS resource for fb %p", fb);
			DRM_DEBUG_KMS(" is being displayed, keeping\n");
			/* There shouldn't be a previous buffer to release */
			BUG_ON(pl111_crtc->old_kds_res_set);

			if (kds_res_set == NULL) {
				err = kds_async_waitall(&kds_res_set,
						&priv.kds_obtain_current_cb,
						&wake, &cb_has_called,
						1, shared, resource_list);
				BUG_ON(err);
				wait_event(wake, cb_has_called == true);
			}

			/* Current buffer will need releasing on next flip */
			pl111_crtc->old_kds_res_set = kds_res_set;

			/*
			 * Clear kds_res_set, so a new kds_res_set is allocated
			 * for additional CRTCs
			 */
			kds_res_set = NULL;
		}
		spin_unlock(&pl111_crtc->current_displaying_lock);
	}

	/* kds_res_set will be NULL here if any CRTCs are displaying fb */
	if (kds_res_set != NULL) {
		DRM_DEBUG_KMS("Initial KDS resource for fb %p", fb);
		DRM_DEBUG_KMS(" not being displayed, discarding\n");
		/* They're not being displayed, release them */
		kds_resource_set_release(&kds_res_set);
	}

	dma_buf_put(dma_buf);
}
#endif

static int pl111_dma_buf_mmap(struct dma_buf *buffer,
			struct vm_area_struct *vma)
{
	struct drm_gem_object *obj = buffer->priv;
	struct pl111_gem_bo *bo = PL111_BO_FROM_GEM(obj);
	DRM_DEBUG_KMS("DRM %s on dma_buf=%p\n", __func__, buffer);

	return pl111_bo_mmap(obj, bo, vma, buffer->size);
}

static void pl111_dma_buf_release(struct dma_buf *buf)
{
	/*
	 * Need to release the dma_buf's reference on the gem object it was
	 * exported from, and also clear the gem object's export_dma_buf
	 * pointer to this dma_buf as it no longer exists
	 */
	struct drm_gem_object *obj = (struct drm_gem_object *)buf->priv;
	struct pl111_gem_bo *bo;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct drm_crtc *crtc;
#endif
	bo = PL111_BO_FROM_GEM(obj);

	DRM_DEBUG_KMS("Releasing dma_buf %p, drm_gem_obj=%p\n", buf, obj);

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	list_for_each_entry(crtc, &bo->gem_object.dev->mode_config.crtc_list,
				head) {
		struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(crtc);
		spin_lock(&pl111_crtc->current_displaying_lock);
		if (pl111_crtc->displaying_fb == bo->fb) {
			kds_resource_set_release(&pl111_crtc->old_kds_res_set);
			pl111_crtc->old_kds_res_set = NULL;
		}
		spin_unlock(&pl111_crtc->current_displaying_lock);
	}
#endif
	mutex_lock(&priv.export_dma_buf_lock);

	obj->export_dma_buf = NULL;
	drm_gem_object_unreference_unlocked(obj);

	mutex_unlock(&priv.export_dma_buf_lock);
}

static int pl111_dma_buf_attach(struct dma_buf *buf, struct device *dev,
				struct dma_buf_attachment *attach)
{
	DRM_DEBUG_KMS("Attaching dma_buf %p to device %p attach=%p\n", buf,
			dev, attach);

	attach->priv = dev;

	return 0;
}

static void pl111_dma_buf_detach(struct dma_buf *buf,
				struct dma_buf_attachment *attach)
{
	DRM_DEBUG_KMS("Detaching dma_buf %p attach=%p\n", attach->dmabuf,
			attach);
}

/* Heavily from exynos_drm_dmabuf.c */
static struct sg_table *pl111_dma_buf_map_dma_buf(struct dma_buf_attachment
						*attach,
						enum dma_data_direction
						direction)
{
	struct drm_gem_object *obj = attach->dmabuf->priv;
	struct pl111_gem_bo *bo = PL111_BO_FROM_GEM(obj);
	struct sg_table *sgt;
	int ret;
	int size, n_pages, nents;

	DRM_DEBUG_KMS("Mapping dma_buf %p from attach=%p\n", attach->dmabuf,
		      attach);

	size = obj->size;
	n_pages = PAGE_ALIGN(size) >> PAGE_SHIFT;

	if (bo->type == PL111_BOT_DMA) {
		sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
		if (!sgt) {
			DRM_ERROR("Failed to allocate sg_table\n");
			return ERR_PTR(-ENOMEM);
		}

		ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
		if (ret < 0) {
			DRM_ERROR("Failed to allocate page table\n");
			return ERR_PTR(-ENOMEM);
		}
		sg_dma_len(sgt->sgl) = size;
		sg_set_page(sgt->sgl,
				pfn_to_page(PFN_DOWN
					(bo->backing_data.dma.fb_dev_addr)),
				size, 0);
		sg_dma_address(sgt->sgl) = bo->backing_data.dma.fb_dev_addr;

	} else {
		struct page **pages;

		pages = get_pages(obj);
		if (IS_ERR(pages)) {
			dev_err(obj->dev->dev, "could not get pages: %ld\n",
				PTR_ERR(pages));
			return ERR_CAST(pages);
		}
		sgt = drm_prime_pages_to_sg(pages, n_pages);
		if (sgt == NULL)
			return ERR_PTR(-ENOMEM);
		nents = dma_map_sg(attach->dev, sgt->sgl, sgt->nents,
				direction);
		if (!nents) {
			DRM_ERROR("Failed to map dma buffer\n");
			sg_free_table(sgt);
			kfree(sgt);
			return ERR_PTR(-ENOMEM);
		}
		if (nents < sgt->nents) {
			/* dma_map_sg() may merge sglist entries (e.g. if
			 * they are contiguous) so nents may be less than
			 * sgt->nents. If this happens we need to fix
			 * sgt->nents as it is used by the caller */
			DRM_DEBUG_KMS(
				"sg list entries merged during mapping\n");
			sgt->nents = nents;
		}
	}
	return sgt;
}

static void pl111_dma_buf_unmap_dma_buf(struct dma_buf_attachment *attach,
					struct sg_table *sgt,
					enum dma_data_direction direction)
{
	struct drm_gem_object *obj = attach->dmabuf->priv;
	struct pl111_gem_bo *bo = PL111_BO_FROM_GEM(obj);

	DRM_DEBUG_KMS("Unmapping dma_buf %p from attach=%p\n", attach->dmabuf,
			attach);

	if (PL111_BOT_SHM == bo->type) {
		/* use orig_nents here as nents may have been
		 * modified in pl111_dma_buf_map_dma_buf() */
		dma_unmap_sg(attach->dev, sgt->sgl, sgt->orig_nents,
						direction);
	}
	sg_free_table(sgt);
	kfree(sgt);
	sgt = NULL;
}

static void *pl111_dma_buf_kmap_atomic(struct dma_buf *dma_buf,
					unsigned long what)
{
	DRM_ERROR("pl111_dma_buf_kmap_atomic not implemented, dma_buf=%p\n",
			dma_buf);
	return NULL;
}

static void *pl111_dma_buf_kmap(struct dma_buf *dma_buf, unsigned long what)
{
	DRM_ERROR("pl111_dma_buf_kmap not implemented, dma_buf=%p\n", dma_buf);
	return NULL;
}

static struct dma_buf_ops pl111_dma_buf_ops = {
	.release = &pl111_dma_buf_release,
	.attach = &pl111_dma_buf_attach,
	.detach = &pl111_dma_buf_detach,
	.map_dma_buf = &pl111_dma_buf_map_dma_buf,
	.unmap_dma_buf = &pl111_dma_buf_unmap_dma_buf,
	.kmap_atomic = &pl111_dma_buf_kmap_atomic,
	.kmap = &pl111_dma_buf_kmap,
	.mmap = &pl111_dma_buf_mmap,
};

struct dma_buf *pl111_gem_prime_export(struct drm_device *dev,
				      struct drm_gem_object *obj, int flags)
{
	struct dma_buf *new_buf;
	struct pl111_gem_bo *bo;
	size_t size;

	DRM_DEBUG_KMS("DRM %s on dev=%p drm_gem_obj=%p\n", __func__, dev, obj);
	size = obj->size;

	new_buf = dma_buf_export(obj /*priv */ , &pl111_dma_buf_ops, size,
					flags | O_RDWR);
	bo = PL111_BO_FROM_GEM(new_buf->priv);

	/*
	 * bo->gem_object.export_dma_buf not setup until after gem_prime_export
	 * finishes
	 */

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	/*
	 * Ensure that we hold the kds resource if it's the currently
	 * displayed buffer.
	 */
	obtain_kds_if_currently_displayed(dev, bo->fb, new_buf);
#endif

	DRM_DEBUG_KMS("Created dma_buf %p\n", new_buf);
	return new_buf;
}

int pl111_prime_handle_to_fd(struct drm_device *dev, struct drm_file *file_priv,
				uint32_t handle, uint32_t flags, int *prime_fd)
{
	int result;
	/*
	 * This will re-use any existing exports, and calls
	 * driver->gem_prime_export to do the first export when needed
	 */
	DRM_DEBUG_KMS("DRM %s on file_priv=%p, handle=0x%.8x\n", __func__,
			file_priv, handle);

	mutex_lock(&priv.export_dma_buf_lock);
	result = drm_gem_prime_handle_to_fd(dev, file_priv, handle, flags,
						prime_fd);
	mutex_unlock(&priv.export_dma_buf_lock);

	return result;
}
