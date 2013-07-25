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
 * pl111_drm_vma.c
 * Implementation of the VM functions for PL111 DRM
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

/* BEGIN drivers/staging/omapdrm/omap_gem_helpers.c */
/**
 * drm_gem_put_pages - helper to free backing pages for a GEM object
 * @obj: obj in question
 * @pages: pages to free
 */
static void _drm_gem_put_pages(struct drm_gem_object *obj, struct page **pages,
				bool dirty, bool accessed)
{
	int i, npages;
	struct pl111_gem_bo *bo;
	npages = obj->size >> PAGE_SHIFT;
	bo = PL111_BO_FROM_GEM(obj);
	for (i = 0; i < npages; i++) {
		if (dirty)
			set_page_dirty(pages[i]);
		if (accessed)
			mark_page_accessed(pages[i]);
		/* Undo the reference we took when populating the table */
		page_cache_release(pages[i]);
	}
	drm_free_large(pages);
}

void put_pages(struct drm_gem_object *obj, struct page **pages)
{
	int i, npages;
	struct pl111_gem_bo *bo;
	npages = obj->size >> PAGE_SHIFT;
	bo = PL111_BO_FROM_GEM(obj);
	_drm_gem_put_pages(obj, pages, true, true);
	if (bo->backing_data.shm.dma_addrs) {
		for (i = 0; i < npages; i++) {
			if (!dma_mapping_error(obj->dev->dev,
					bo->backing_data.shm.dma_addrs[i])) {
				dma_unmap_page(obj->dev->dev,
					bo->backing_data.shm.dma_addrs[i],
					PAGE_SIZE,
					DMA_BIDIRECTIONAL);
			}
		}
		kfree(bo->backing_data.shm.dma_addrs);
		bo->backing_data.shm.dma_addrs = NULL;
	}
}

/**
 * drm_gem_get_pages - helper to allocate backing pages for a GEM object
 * @obj: obj in question
 * @gfpmask: gfp mask of requested pages
 */
static struct page **_drm_gem_get_pages(struct drm_gem_object *obj,
					gfp_t gfpmask)
{
	struct inode *inode;
	struct address_space *mapping;
	struct page *p, **pages;
	int i, npages;

	/* This is the shared memory object that backs the GEM resource */
	inode = obj->filp->f_path.dentry->d_inode;
	mapping = inode->i_mapping;

	npages = obj->size >> PAGE_SHIFT;

	pages = drm_malloc_ab(npages, sizeof(struct page *));
	if (pages == NULL)
		return ERR_PTR(-ENOMEM);

	gfpmask |= mapping_gfp_mask(mapping);

	for (i = 0; i < npages; i++) {
		p = shmem_read_mapping_page_gfp(mapping, i, gfpmask);
		if (IS_ERR(p))
			goto fail;
		pages[i] = p;

		/*
		 * There is a hypothetical issue w/ drivers that require
		 * buffer memory in the low 4GB.. if the pages are un-
		 * pinned, and swapped out, they can end up swapped back
		 * in above 4GB.  If pages are already in memory, then
		 * shmem_read_mapping_page_gfp will ignore the gfpmask,
		 * even if the already in-memory page disobeys the mask.
		 *
		 * It is only a theoretical issue today, because none of
		 * the devices with this limitation can be populated with
		 * enough memory to trigger the issue.  But this BUG_ON()
		 * is here as a reminder in case the problem with
		 * shmem_read_mapping_page_gfp() isn't solved by the time
		 * it does become a real issue.
		 *
		 * See this thread: http://lkml.org/lkml/2011/7/11/238
		 */
		BUG_ON((gfpmask & __GFP_DMA32) &&
			(page_to_pfn(p) >= 0x00100000UL));
	}

	return pages;

fail:
	while (i--)
		page_cache_release(pages[i]);

	drm_free_large(pages);
	return ERR_PTR(PTR_ERR(p));
}

struct page **get_pages(struct drm_gem_object *obj)
{
	struct pl111_gem_bo *bo;
	bo = PL111_BO_FROM_GEM(obj);

	if (bo->backing_data.shm.pages == NULL) {
		struct page **p;
		int npages = obj->size >> PAGE_SHIFT;
		int i;

		p = _drm_gem_get_pages(obj, GFP_KERNEL);
		if (IS_ERR(p))
			return ERR_PTR(-ENOMEM);

		bo->backing_data.shm.pages = p;

		if (bo->backing_data.shm.dma_addrs == NULL) {
			bo->backing_data.shm.dma_addrs =
				kzalloc(npages * sizeof(dma_addr_t),
					GFP_KERNEL);
			if (bo->backing_data.shm.dma_addrs == NULL)
				goto error_out;
		}

		for (i = 0; i < npages; ++i) {
			bo->backing_data.shm.dma_addrs[i] =
				dma_map_page(obj->dev->dev, p[i], 0, PAGE_SIZE,
					DMA_BIDIRECTIONAL);
			if (dma_mapping_error(obj->dev->dev,
					bo->backing_data.shm.dma_addrs[i]))
				goto error_out;
		}
	}

	return bo->backing_data.shm.pages;

error_out:
	put_pages(obj, bo->backing_data.shm.pages);
	bo->backing_data.shm.pages = NULL;
	return ERR_PTR(-ENOMEM);
}

/* END drivers/staging/omapdrm/omap_gem_helpers.c */

int pl111_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct page **pages;
	pgoff_t pgoff;
	unsigned long pfn;
	struct drm_gem_object *obj = vma->vm_private_data;
	struct pl111_gem_bo *bo = PL111_BO_FROM_GEM(obj);
	DRM_DEBUG_KMS("DRM %s on pl111_gem_bo %p\n", __func__, bo);

	/* We don't use vmf->pgoff since that has the fake offset: */
	pgoff = ((unsigned long)vmf->virtual_address -
		 vma->vm_start) >> PAGE_SHIFT;
	if (bo->type == PL111_BOT_SHM) {
		pages = get_pages(obj);
		if (IS_ERR(pages)) {
			dev_err(obj->dev->dev,
				"could not get pages: %ld\n", PTR_ERR(pages));
			return PTR_ERR(pages);
		}
		pfn = page_to_pfn(pages[pgoff]);
		DRM_DEBUG_KMS("physaddr 0x%.8x for offset 0x%x\n",
				PFN_PHYS(pfn), PFN_PHYS(pgoff));
		vm_insert_mixed(vma, (unsigned long)vmf->virtual_address, pfn);
		return VM_FAULT_NOPAGE;
	} else {
		DRM_DEBUG_KMS("Fault on non-shared memory %p\n",
				vmf->virtual_address);
		return VM_FAULT_SIGBUS;
	}
}
