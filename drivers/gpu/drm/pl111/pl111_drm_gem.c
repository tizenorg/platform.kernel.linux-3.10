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
 * pl111_drm_gem.c
 * Implementation of the GEM functions for PL111 DRM
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

void pl111_gem_free_object(struct drm_gem_object *obj)
{
	struct pl111_gem_bo *bo;
	struct drm_device *dev = obj->dev;
	DRM_DEBUG_KMS("DRM %s on drm_gem_object=%p\n", __func__, obj);

	bo = PL111_BO_FROM_GEM(obj);

	if (obj->map_list.map != NULL)
		drm_gem_free_mmap_offset(obj);

	if (bo->type == PL111_BOT_DMA) {
		dma_free_writecombine(dev->dev, obj->size,
					bo->backing_data.dma.fb_cpu_addr,
					bo->backing_data.dma.fb_dev_addr);
	} else if (bo->backing_data.shm.pages != NULL) {
		put_pages(obj, bo->backing_data.shm.pages);
	}
	drm_gem_object_release(obj);

	kfree(bo);

	DRM_DEBUG_KMS("Destroyed dumb_bo handle 0x%p\n", bo);
}

int pl111_dumb_create(struct drm_file *file_priv,
		struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	int ret = 0;
	struct pl111_gem_bo *bo = NULL;
	uint32_t bytes_pp;
	bool create_contig_buffer;

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (bo == NULL) {
		ret = -ENOMEM;
		goto finish;
	}

	/* Round bpp up, to allow for case where bpp<8 */
	bytes_pp = args->bpp >> 3;
	if (args->bpp & ((1 << 3) - 1))
		bytes_pp++;

	args->pitch = ALIGN(args->width * bytes_pp, 64);
	args->size = PAGE_ALIGN(args->pitch * args->height);

	DRM_DEBUG_KMS("dumb_create w=%d h=%d p=%d bpp=%d b=%d s=%llu f=0x%x\n",
			args->width, args->height, args->pitch, args->bpp,
			bytes_pp, args->size, args->flags);

	create_contig_buffer = args->flags & PL111_BO_SCANOUT;
#ifndef ARCH_HAS_SG_CHAIN
	/*
	 * If the ARCH can't chain we can't have non-contiguous allocs larger
	 * than a single sg can hold.
	 * In this case we fall back to using contiguous memory
	 */
	if (!create_contig_buffer) {
		long unsigned int n_pages =
				PAGE_ALIGN(args->size) >> PAGE_SHIFT;
		if (n_pages > SG_MAX_SINGLE_ALLOC) {
			create_contig_buffer = true;
			/*
			 * Non-contiguous allocation request changed to
			 * contigous
			 */
			DRM_INFO("non-contig alloc to contig %lu > %lu pages.",
					n_pages, SG_MAX_SINGLE_ALLOC);
		}
	}
#endif
	if (!create_contig_buffer) {
		/* not scanout compatible - use non-contiguous buffer */
		bo->type = PL111_BOT_SHM;
		ret = drm_gem_object_init(dev, &bo->gem_object, args->size);
		if (ret != 0) {
			DRM_ERROR("DRM could not init SHM backed GEM obj\n");
			kfree(bo);
			ret = -ENOMEM;
			goto finish;
		}
		DRM_DEBUG_KMS("Num bytes: %d\n", bo->gem_object.size);
	} else {
		/* scanout compatible - use contiguous buffer */
		bo->type = PL111_BOT_DMA;

		bo->backing_data.dma.fb_cpu_addr =
			dma_alloc_writecombine(dev->dev, args->size,
					&bo->backing_data.dma.fb_dev_addr,
					GFP_KERNEL);
		if (bo->backing_data.dma.fb_cpu_addr == NULL) {
			DRM_ERROR("dma_alloc_writecombine failed\n");
			kfree(bo);
			ret = -ENOMEM;
			goto finish;
		}

		ret = drm_gem_private_object_init(dev, &bo->gem_object,
							args->size);
		if (ret != 0) {
			DRM_ERROR("DRM could not initialise GEM object\n");
			dma_free_writecombine(dev->dev, args->size,
					bo->backing_data.dma.fb_cpu_addr,
					bo->backing_data.dma.fb_dev_addr);
			kfree(bo);
			ret = -ENOMEM;
			goto finish;
		}
	}

	DRM_DEBUG_KMS("dumb_create: 0x%p with w=%d, h=%d, p=%d, bpp=%d,",
		bo, args->width, args->height, args->pitch, args->bpp);
	DRM_DEBUG_KMS("bytes_pp=%d, s=%llu, flags=0x%x, %s 0x%.8lx, type=%d\n",
		bytes_pp, args->size, args->flags,
		(bo->type == PL111_BOT_DMA) ? "physaddr" : "shared page array",
		(bo->type == PL111_BOT_DMA)
			? (unsigned long)bo->backing_data.dma.fb_dev_addr
			: (unsigned long)bo->backing_data.shm.pages, bo->type);

	/* omap_gem_new_handle() */
	ret = drm_gem_handle_create(file_priv, &bo->gem_object, &args->handle);
	if (ret != 0) {
		DRM_ERROR("DRM failed to create GEM handle\n");
		drm_gem_object_release(&bo->gem_object);
		if (bo->type == PL111_BOT_DMA) {
			dma_free_writecombine(dev->dev, args->size,
					bo->backing_data.dma.fb_cpu_addr,
					bo->backing_data.dma.fb_dev_addr);
		}
		kfree(bo);
		return ret;
	}
	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference_unlocked(&bo->gem_object);
	DRM_DEBUG_KMS("dumb_create completed: fp=%p h=0x%.8x gem_object=%p",
			file_priv, args->handle, &bo->gem_object);

finish:
	return ret;
}

int pl111_dumb_destroy(struct drm_file *file_priv, struct drm_device *dev,
		uint32_t handle)
{
	DRM_DEBUG_KMS("DRM %s on file_priv=%p handle=0x%.8x\n", __func__,
			file_priv, handle);
	return drm_gem_handle_delete(file_priv, handle);
}

int pl111_dumb_map_offset(struct drm_file *file_priv,
			struct drm_device *dev, uint32_t handle,
			uint64_t *offset)
{
	/* omap_gem_dump_map_offset */
	struct drm_gem_object *obj;
	int ret = 0;
	DRM_DEBUG_KMS("DRM %s on file_priv=%p handle=0x%.8x\n", __func__,
			file_priv, handle);

	/* GEM does all our handle to object mapping */
	obj = drm_gem_object_lookup(dev, file_priv, handle);
	if (obj == NULL) {
		ret = -ENOENT;
		goto fail;
	}

	if (obj->map_list.map == NULL) {
		ret = drm_gem_create_mmap_offset(obj);
		if (ret != 0)
			goto fail;
	}

	*offset = (uint64_t) obj->map_list.hash.key << PAGE_SHIFT;

	drm_gem_object_unreference_unlocked(obj);
fail:
	return ret;
}

/* Based on drm_vm.c and omapdrm driver */
int pl111_bo_mmap(struct drm_gem_object *obj, struct pl111_gem_bo *bo,
		 struct vm_area_struct *vma, size_t size)
{
	int ret = 0;

	DRM_DEBUG_KMS("DRM %s on drm_gem_object=%p, pl111_gem_bo=%p\n",
			__func__, obj, bo);

	if (obj->size < vma->vm_end - vma->vm_start) {
		ret = -EINVAL;
		goto done;
	}

	if (bo->type == PL111_BOT_DMA) {
		vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
		vma->vm_page_prot =
			pgprot_noncached(vm_get_page_prot(vma->vm_flags));

		if (remap_pfn_range(vma, vma->vm_start,
			(bo->backing_data.dma.fb_dev_addr) >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start, vma->vm_page_prot))
			ret = -EAGAIN;
	} else {
		unsigned long uaddr = vma->vm_start;
		long usize = obj->size;
		int i = 0;
		struct page **pages;
		vma->vm_flags &= ~VM_PFNMAP;
		vma->vm_flags |= VM_MIXEDMAP;
		vma->vm_page_prot =
			pgprot_noncached(vm_get_page_prot(vma->vm_flags));
		pages = get_pages(obj);
		if (IS_ERR(pages)) {
			dev_err(obj->dev->dev, "could not get pages: %ld\n",
				PTR_ERR(pages));
			return PTR_ERR(pages);
		}
		do {
			ret = vm_insert_page(vma, uaddr, pages[i]);
			if (ret != 0) {
				DRM_ERROR("failed to remap user space.\n");
				return ret;
			}
			uaddr += PAGE_SIZE;
			usize -= PAGE_SIZE;
			i++;
		} while (usize > 0);
	}

done:
	return ret;
}

int pl111_gem_mmap(struct file *file_priv, struct vm_area_struct *vma)
{
	int ret;
	struct drm_file *priv = file_priv->private_data;
	struct drm_device *dev = priv->minor->dev;
	struct drm_gem_mm *mm = dev->mm_private;
	struct drm_local_map *map = NULL;
	struct drm_hash_item *hash;
	struct drm_gem_object *obj;
	struct pl111_gem_bo *bo;
	DRM_DEBUG_KMS("DRM %s\n", __func__);

	ret = drm_gem_mmap(file_priv, vma);

	drm_ht_find_item(&mm->offset_hash, vma->vm_pgoff, &hash);
	map = drm_hash_entry(hash, struct drm_map_list, hash)->map;
	obj = map->handle;
	bo = PL111_BO_FROM_GEM(obj);

	DRM_DEBUG_KMS("DRM %s on pl111_gem_bo %p\n", __func__, bo);

	return pl111_bo_mmap(obj, bo, vma, vma->vm_end - vma->vm_start);
}
