/*
 * Copyright (C) 2014 Samsung Electronics Co.Ltd
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Chanho Park <chanho61.park@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <linux/seqno-fence.h>
#include <linux/reservation.h>
#include <linux/dmabuf-sync.h>

#define DEFAULT_SYNC_TIMEOUT	500	/* in millisecond. */
#define BEGIN_CPU_ACCESS(old, new_type)	\
			((old->accessed_type & DMA_BUF_ACCESS_DMA_W) == \
			  DMA_BUF_ACCESS_DMA_W && new_type == DMA_BUF_ACCESS_R)

#define END_CPU_ACCESS(old, new_type)	\
			(((old->accessed_type == DMA_BUF_ACCESS_W) || \
			 (old->accessed_type == DMA_BUF_ACCESS_RW)) && \
			 new_type & DMA_BUF_ACCESS_DMA)

static int dmabuf_sync_enabled = 1;
static unsigned long seqno;
static LIST_HEAD(sync_obj_list_head);
static DEFINE_SPINLOCK(sync_obj_list_lock);

MODULE_PARM_DESC(enabled, "Check if dmabuf sync is supported or not");
module_param_named(enabled, dmabuf_sync_enabled, int, 0444);

static const char *dmabuf_sync_get_driver_name(struct fence *fence)
{
	return NULL;
}

static const char *dmabuf_sync_get_timeline_name(struct fence *fence)
{
	return NULL;
}

static bool dmabuf_sync_enable_sw_signaling(struct fence *fence)
{
	fence_enable_sw_signaling(fence);

	return true;
}

static const struct fence_ops fence_default_ops = {
	.get_driver_name = dmabuf_sync_get_driver_name,
	.get_timeline_name = dmabuf_sync_get_timeline_name,
	.enable_signaling = dmabuf_sync_enable_sw_signaling,
	.signaled = fence_is_signaled,
	.wait = fence_default_wait,
};

/**
 * dmabuf_sync_is_supported - Check if dmabuf sync is supported or not.
 */
bool dmabuf_sync_is_supported(void)
{
	return dmabuf_sync_enabled == 1;
}
EXPORT_SYMBOL_GPL(dmabuf_sync_is_supported);

/*
 * Perform cache operation according to access type.
 *
 * This function is called by dmabuf_sync_wait or dmabuf_sync_wait_all function
 * to change the ownership of a buffer - for cache coherence between CPU and
 * DMA -, CPU -> DMA or DMA -> CPU after signaled.
 */
static void dmabuf_sync_cache_ops(struct dmabuf_sync_object *sobj)
{
	struct seqno_fence *sf= &sobj->base;
	struct dma_buf *dmabuf = sf->sync_buf;

	/* It doesn't need cache operation if access first time. */
	if (!dmabuf->resv->accessed_type)
		goto out;

	if (END_CPU_ACCESS(dmabuf->resv, sobj->access_type))
		/* cache clean */
		dma_buf_end_cpu_access(dmabuf, 0, dmabuf->size,
						DMA_TO_DEVICE);
	else if (BEGIN_CPU_ACCESS(dmabuf->resv, sobj->access_type))
		/* cache invalidate */
		dma_buf_begin_cpu_access(dmabuf, 0, dmabuf->size,
						DMA_FROM_DEVICE);

out:
	/* Update access type to new one. */
	dmabuf->resv->accessed_type = sobj->access_type;
}

/**
 * dmabuf_sync_check - check whether a given buffer is required sync operation.
 *
 * @dmabuf: An object to dma_buf structure.
 *
 * This function should be called by dmabuf_sync_wait or dmabuf_sync_wait_all
 * function before it makes current thread to be blocked. It returnes true if
 * buffer sync is needed else false.
 */
bool dmabuf_sync_check(struct dma_buf *dmabuf)
{
	struct reservation_object_list *rol;
	struct reservation_object *ro;
	struct fence *fence = NULL;
	unsigned int i;

	rcu_read_lock();
	ro = rcu_dereference(dmabuf->resv);
	if (!ro || !ro->fence)
		goto unlock_rcu;

	rol = rcu_dereference(ro->fence);

	/* Check if there is any fence requested for a read access. */
	for (i = 0; i < rol->shared_count; i++) {
		fence = rcu_dereference(rol->shared[i]);
		if (!fence)
			continue;

		break;
	}

	if (i != rol->shared_count) {
		rcu_read_unlock();
		return true;
	}

	/* And then check if there is a fence requested for a write access. */
	fence = rcu_dereference(ro->fence_excl);
unlock_rcu:
	rcu_read_unlock();
	return fence ? true : false;
}

/**
 * dmabuf_sync_init - Allocate and initialize a dmabuf sync.
 *
 * @priv: A device private data.
 * @name: A sync object name.
 *
 * This function should be called by dma device driver when a device context
 * or an event context is created. The created dmabuf sync should be set
 * to the context. Each dma device driver has one sync object per a thread.
 * The caller can get a new dmabuf sync for buffer synchronization
 * through this function. It returns dmabuf_sync object if true else error.
 */
struct dmabuf_sync *dmabuf_sync_init(const char *name,
					struct dmabuf_sync_priv_ops *ops,
					void *priv)
{
	struct dmabuf_sync *sync;

	sync = kzalloc(sizeof(*sync), GFP_KERNEL);
	if (!sync)
		return ERR_PTR(-ENOMEM);

	strncpy(sync->name, name, DMABUF_SYNC_NAME_SIZE);

	sync->ops = ops;
	sync->priv = priv;
	INIT_LIST_HEAD(&sync->syncs);
	spin_lock_init(&sync->lock);

	return sync;
}
EXPORT_SYMBOL_GPL(dmabuf_sync_init);

/*
 * dmabuf_sync_get_obj - Add a given object to sync's list.
 *
 * @sync: An object to dmabuf_sync structure.
 * @dmabuf: An object to dma_buf structure.
 * @type: A access type to a dma buf.
 *	The DMA_BUF_ACCESS_R means that this dmabuf could be accessed by
 *	others for read access. On the other hand, the DMA_BUF_ACCESS_W
 *	means that this dmabuf couldn't be accessed by others but would be
 *	accessed by caller's dma exclusively. And the DMA_BUF_ACCESS_DMA can be
 *	combined.
 *
 * This function creates and initializes a new dmabuf sync object and adds
 * the object to a given syncs list. dmabuf_sync has one dmabuf_sync_object
 * per a dmabuf.
 */
static int dmabuf_sync_get_obj(struct dmabuf_sync *sync, struct dma_buf *dmabuf,
					unsigned int type)
{
	struct dmabuf_sync_object *sobj;
	unsigned long flags;
	int ret;

	if (!sync)
		return -EFAULT;

	if (!IS_VALID_DMA_BUF_ACCESS_TYPE(type))
		return -EINVAL;

	if ((type & DMA_BUF_ACCESS_RW) == DMA_BUF_ACCESS_RW)
		type &= ~DMA_BUF_ACCESS_R;

	sobj = kzalloc(sizeof(*sobj), GFP_KERNEL);
	if (!sobj)
		return -ENOMEM;

	spin_lock_init(&sobj->lock);
	get_dma_buf(dmabuf);
	sobj->access_type = type;

	if (sobj->access_type & DMA_BUF_ACCESS_R) {
		ret = reservation_object_reserve_shared(dmabuf->resv);
		if (ret) {
			dma_buf_put(dmabuf);
			kfree(sobj);
			return ret;
		}
	}

	seqno_fence_init(&sobj->base, &sobj->lock, dmabuf, 0, 0, 0, ++seqno,
				&fence_default_ops);

	spin_lock_irqsave(&sync->lock, flags);
	list_add_tail(&sobj->l_head, &sync->syncs);
	spin_unlock_irqrestore(&sync->lock, flags);

	return 0;
}

/*
 * dmabuf_sync_put_objs - Release all sync objects of dmabuf_sync.
 *
 * @sync: An object to dmabuf_sync structure.
 *
 * This function should be called if some operation failed after
 * dmabuf_sync_get_obj call to release all sync objects.
 */
static void dmabuf_sync_put_objs(struct dmabuf_sync *sync)
{
	struct dmabuf_sync_object *sobj, *next;
	unsigned long flags;

	spin_lock_irqsave(&sync->lock, flags);

	list_for_each_entry_safe(sobj, next, &sync->syncs, l_head) {
		struct seqno_fence *sf = &sobj->base;

		dma_buf_put(sf->sync_buf);
		list_del_init(&sobj->l_head);
		kfree(sobj);
	}

	spin_unlock_irqrestore(&sync->lock, flags);
}

/*
 * dmabuf_sync_put_obj - Release a given sync object.
 *
 * @sync: An object to dmabuf_sync structure.
 * @dmabuf: A dmabuf object to be released.
 *
 * This function should be called if some operation failed after
 * dmabuf_sync_get_obj call to release a given sync object.
 */
static void dmabuf_sync_put_obj(struct dmabuf_sync *sync,
					struct dma_buf *dmabuf)
{
	struct dmabuf_sync_object *sobj, *next;
	unsigned long flags;

	spin_lock_irqsave(&sync->lock, flags);

	list_for_each_entry_safe(sobj, next, &sync->syncs, l_head) {
		struct seqno_fence *sf = &sobj->base;

		if (sf->sync_buf != dmabuf)
			continue;

		dma_buf_put(dmabuf);
		list_del_init(&sobj->l_head);
		kfree(sobj);

		break;
	}

	spin_unlock_irqrestore(&sync->lock, flags);
}

/**
 * dmabuf_sync_get - Get dmabuf sync object.
 *
 * @sync: An object to dmabuf_sync structure.
 * @sync_buf: A dmabuf object to be synchronized with others.
 * @type: A access type to a dma buf.
 *	The DMA_BUF_ACCESS_R means that this dmabuf could be accessed by
 *	others for read access. On the other hand, the DMA_BUF_ACCESS_W
 *	means that this dmabuf couldn't be accessed by others but would be
 *	accessed by caller's dma exclusively. And the DMA_BUF_ACCESS_DMA can
 *	be combined with other.
 *
 * This function should be called after dmabuf_sync_init function is called.
 * The caller can tie up multiple dmabufs into one sync object by calling this
 * function several times.
 */
int dmabuf_sync_get(struct dmabuf_sync *sync, void *sync_buf, unsigned int type)
{
	int ret;

	if (!sync || !sync_buf)
		return -EFAULT;

	ret = dmabuf_sync_get_obj(sync, sync_buf, type);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(dmabuf_sync_get);

/**
 * dmabuf_sync_put - Put dmabuf sync object to a given dmabuf.
 *
 * @sync: An object to dmabuf_sync structure.
 * @dmabuf: An dmabuf object.
 *
 * This function should be called if some operation failed after
 * dmabuf_sync_get function is called to release the dmabuf, or
 * dmabuf_sync_signal function is called.
 */
void dmabuf_sync_put(struct dmabuf_sync *sync, struct dma_buf *dmabuf)
{
	if (!sync || !dmabuf) {
		WARN_ON(1);
		return;
	}

	if (list_empty(&sync->syncs))
		return;

	dmabuf_sync_put_obj(sync, dmabuf);
}
EXPORT_SYMBOL_GPL(dmabuf_sync_put);

/**
 * dmabuf_sync_put_all - Put dmabuf sync object to dmabufs.
 *
 * @sync: An object to dmabuf_sync structure.
 *
 * This function should be called if some operation is failed after
 * dmabuf_sync_get function is called to release all sync objects, or
 * dmabuf_sync_unlock function is called.
 */
void dmabuf_sync_put_all(struct dmabuf_sync *sync)
{
	if (!sync) {
		WARN_ON(1);
		return;
	}

	if (list_empty(&sync->syncs))
		return;

	dmabuf_sync_put_objs(sync);
}
EXPORT_SYMBOL_GPL(dmabuf_sync_put_all);


/*
 * dmabuf_sync_update - register a fence object to reservation_object.
 *
 * @sobj: An object o dmabuf_sync_object.
 *
 * This function is called by dmabuf_sync_wait and dmabuf_sync_wait_all
 * functions when there is no any thread accessing the same dmabuf or
 * after current thread is waked up for that current thread has a ownership
 * of the dmabuf.
 */
static void dmabuf_sync_update(struct dmabuf_sync_object *sobj)
{
	struct seqno_fence *sf = &sobj->base;
	struct dma_buf *dmabuf = sf->sync_buf;

	if (sobj->access_type & DMA_BUF_ACCESS_W)
		reservation_object_add_excl_fence(dmabuf->resv, &sf->base);
	else
		reservation_object_add_shared_fence(dmabuf->resv, &sf->base);
}

/*
 * make_sure_the_req_ordering- check if a given sobj was requested the first
 *				for buffer sync.
 *
 * @csobj: An object to dmabuf_sync_object for checking if buffer sync was
 *	requested the first for buffer sync.
 *
 * This function is called by dmabuf_sync_wait and dmabuf_sync_wait_all
 * after current thread is waked up to check if there is any object that
 * buffer sync was earlier requested than current thread.
 * If there is a object, the current thread doesn't have the ownership of
 * a dmabuf so will try to block again to yield the ownership to other
 * thread that requested earlier buffer sync than current thread.
 */
static int make_sure_the_req_ordering(struct dmabuf_sync_object *csobj)
{
	struct dmabuf_sync_object *sobj;
	unsigned long flags;
	bool found = false;

	spin_lock_irqsave(&sync_obj_list_lock, flags);

	if (list_empty(&sync_obj_list_head)) {
		spin_unlock_irqrestore(&sync_obj_list_lock, flags);
		/* There should be no such case. */
		WARN_ON(1);
		return -EPERM;
	}

	list_for_each_entry(sobj, &sync_obj_list_head, g_head) {
		struct seqno_fence *sf = &sobj->base;
		/*
		 * If there is a requested sobj earlier than a given sobj
		 * then the given sobj should be blocked again to yield
		 * the buffer ownership to the earier one.
		 */
		if (sobj != csobj) {
			long timeout;

			timeout = fence_wait_timeout(&sf->base, true,
					msecs_to_jiffies(DEFAULT_SYNC_TIMEOUT));
			if (!timeout) {
				spin_unlock_irqrestore(&sync_obj_list_lock,
							flags);
				pr_warning("signal wait has been timed out.\n");
				return -EAGAIN;
			}
		} else {
			found = true;
			break;
		}
	}

	/*
	 * Delete a given sobj from sync_obj_list_head if there is no any
	 * sobj of other thread requested earlier than a owner thread of
	 * a given sobj.
	 */
	if (found)
		list_del_init(&sobj->g_head);

	spin_unlock_irqrestore(&sync_obj_list_lock, flags);
	return 0;
}

/**
 * dmabuf_sync_wait_all - wait for the completion of DMA or CPU access to
 *				all dmabufs.
 *
 * @sync: An object to dmabuf_sync structure.
 *
 * The caller should call this function prior to CPU or DMA access to
 * dmabufs so that another thread can not access the dmabufs.
 */
long dmabuf_sync_wait_all(struct dmabuf_sync *sync)
{
	struct dmabuf_sync_object *sobj;
	unsigned long timeout = 0;
	unsigned long flags;

	spin_lock_irqsave(&sync->lock, flags);

	list_for_each_entry(sobj, &sync->syncs, l_head) {
		struct dma_buf *dmabuf;
		struct seqno_fence *sf;
		bool all_wait;
		int ret;

		spin_unlock_irqrestore(&sync->lock, flags);

		spin_lock_irqsave(&sync_obj_list_lock, flags);
		/*
		 * sync_obj_list_head is used to check if there is any sobj
		 * of other thread that requested earlier buffer sync than
		 * current thread.
		 */
		list_add_tail(&sobj->g_head, &sync_obj_list_head);
		spin_unlock_irqrestore(&sync_obj_list_lock, flags);

		sf = &sobj->base;
		dmabuf = sf->sync_buf;

		if (!dmabuf_sync_check(dmabuf)) {
			fence_enable_sw_signaling(&sf->base);
			dmabuf_sync_update(sobj);
			spin_lock_irqsave(&sync->lock, flags);
			continue;
		}

		/*
		 * Need to wait for all buffers for a read or a write
		 * if it should access a buffer for a write.
		 * Otherwise, just wait for only the completion of a buffer
		 * access for a write.
		 */
		all_wait = sobj->access_type & DMA_BUF_ACCESS_W;
		timeout = reservation_object_wait_timeout_rcu(dmabuf->resv,
				all_wait, true,
				msecs_to_jiffies(DEFAULT_SYNC_TIMEOUT));
		if (!timeout)
			pr_warning("signal wait has been timed out.\n");

		/*
		 * Check if there is any sobj of other thread that requested
		 * earlier than current thread. If other sobj, then it should
		 * yield the ownership of a buffer to other thread for buffer
		 * access ordering.
		 */
		ret = make_sure_the_req_ordering(sobj);
		if (ret) {
			spin_lock_irqsave(&sync_obj_list_lock, flags);
			list_del_init(&sobj->g_head);
			spin_unlock_irqrestore(&sync_obj_list_lock, flags);

			dma_buf_put(dmabuf);
			kfree(sobj);

			return ret;
		}

		fence_enable_sw_signaling(&sobj->base.base);
		dmabuf_sync_update(sobj);
		dmabuf_sync_cache_ops(sobj);

		spin_lock_irqsave(&sync->lock, flags);
	}

	spin_unlock_irqrestore(&sync->lock, flags);

	return timeout;
}
EXPORT_SYMBOL_GPL(dmabuf_sync_wait_all);

/**
 * dmabuf_sync_wait - wait for the completion of DMA or CPU access to a dmabuf.
 *
 * @sync: An object to dmabuf_sync structure.
 *
 * The caller should call this function prior to CPU or DMA access to
 * a dmabuf so that another thread can not access the dmabuf.
 */
long dmabuf_sync_wait(struct dma_buf *dmabuf, unsigned int access_type)
{
	struct dmabuf_sync_object *sobj;
	unsigned long timeout = 0;
	bool all_wait;
	unsigned long flags;
	int ret;

	sobj = kzalloc(sizeof(*sobj), GFP_KERNEL);
	if (!sobj)
		return -ENOMEM;

	spin_lock_init(&sobj->lock);
	get_dma_buf(dmabuf);
	sobj->access_type = access_type;

	if (sobj->access_type & DMA_BUF_ACCESS_R) {
		ret = reservation_object_reserve_shared(dmabuf->resv);
		if (ret) {
			dma_buf_put(dmabuf);
			kfree(sobj);
			return ret;
		}
	}

	seqno_fence_init(&sobj->base, &sobj->lock, dmabuf, 0, 0, 0, ++seqno,
				&fence_default_ops);

	spin_lock_irqsave(&sync_obj_list_lock, flags);
	list_add_tail(&sobj->g_head, &sync_obj_list_head);
	spin_unlock_irqrestore(&sync_obj_list_lock, flags);

	if (!dmabuf_sync_check(dmabuf)) {
		/* If there is no need to wait, just signal the fence */
		fence_enable_sw_signaling(&sobj->base.base);
		dmabuf_sync_update(sobj);
		return timeout;
	}

	all_wait = access_type & DMA_BUF_ACCESS_W;

	timeout = reservation_object_wait_timeout_rcu(dmabuf->resv,
			all_wait, true,
			msecs_to_jiffies(DEFAULT_SYNC_TIMEOUT));
	if (!timeout) {
		pr_warning("signal wait has been timed out.\n");
		return timeout;
	}

	/*
	 * Check if there is any sobj of other thread that requested
	 * earlier than current thread. If other sobj, then it should
	 * yield the ownership of a buffer to other thread for buffer
	 * access ordering.
	 */
	ret = make_sure_the_req_ordering(sobj);
	if (ret) {
		spin_lock_irqsave(&sync_obj_list_lock, flags);
		list_del_init(&sobj->g_head);
		spin_unlock_irqrestore(&sync_obj_list_lock, flags);

		dma_buf_put(dmabuf);
		kfree(sobj);

		return ret;
	}

	fence_enable_sw_signaling(&sobj->base.base);

	dmabuf_sync_update(sobj);
	dmabuf_sync_cache_ops(sobj);

	return timeout;
}
EXPORT_SYMBOL_GPL(dmabuf_sync_wait);


/**
 * dmabuf_sync_signal_all - wake up all threads blocked when tried to access
 *				any buffer of sync object.
 *
 * @sync: An object to dmabuf_sync structure.
 *
 * The caller should call this function after CPU or DMA access to
 * the dmabufs is completed so that others can access the dmabufs.
 */
int dmabuf_sync_signal_all(struct dmabuf_sync *sync)
{
	struct dmabuf_sync_object *sobj;
	unsigned long flags;
	int ret = -EAGAIN;

	spin_lock_irqsave(&sync->lock, flags);

	list_for_each_entry(sobj, &sync->syncs, l_head) {
		struct seqno_fence *sf = &sobj->base;

		ret = fence_signal(&sf->base);
		if (ret) {
			pr_warning("signal request has been failed.\n");
			dma_buf_put(sf->sync_buf);
			break;
		}

		dma_buf_put(sf->sync_buf);
	}

	spin_unlock_irqrestore(&sync->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(dmabuf_sync_signal_all);

/**
 * dmabuf_sync_signal - wake up all threads blocked when tried to access a
 *			given buffer.
 *
 * @dmabuf: An object to dma_buf structure.
 *
 * The caller should call this function after the completion of a dmabuf
 * access by CPU or DMA so that another thread can access the dmabuf.
 */
int dmabuf_sync_signal(struct dma_buf *dmabuf)
{
	struct reservation_object_list *rol;
	struct dmabuf_sync_object *sobj;
	struct reservation_object *ro;
	struct seqno_fence *sf;
	struct fence *fence;
	int ret = -EINVAL;
	int i;

	rcu_read_lock();

	ro = rcu_dereference(dmabuf->resv);
	rol = rcu_dereference(ro->fence);

	/* Was it a fence for a write? */
	fence = rcu_dereference(ro->fence_excl);
	if (fence)
		goto found;

	/* Was it a fence for a read. */
	for (i = 0; i < rol->shared_count; i++) {
		fence = rcu_dereference(rol->shared[i]);
		if (!fence)
			continue;

		break;
	}

	if (!fence) {
		rcu_read_unlock();
		dma_buf_put(dmabuf);

		/* There should be no such case. */
		WARN_ON(1);
		return -EPERM;
	}

found:
	ret = fence_signal(fence);
	if (ret)
		pr_warning("signal request has been failed.\n");

	rcu_read_unlock();
	dma_buf_put(dmabuf);

	sf = to_seqno_fence(fence);
	sobj = container_of(sf, struct dmabuf_sync_object, base);

	kfree(sobj);

	return ret;
}
EXPORT_SYMBOL_GPL(dmabuf_sync_signal);

/**
 * dmabuf_sync_fini - Release a given dmabuf sync.
 *
 * @sync: An object to dmabuf_sync structure.
 *
 * This function should be called if some operation failed after
 * dmabuf_sync_init call to release relevant resources, and after
 * dmabuf_sync_signal or dmabuf_sync_signal_all function is called.
 */
void dmabuf_sync_fini(struct dmabuf_sync *sync)
{
	unsigned long flags;

	if (WARN_ON(!sync))
		return;

	spin_lock_irqsave(&sync->lock, flags);

	if (list_empty(&sync->syncs))
		goto free_sync;

	dmabuf_sync_signal_all(sync);
	dmabuf_sync_put_all(sync);

free_sync:
	spin_unlock_irqrestore(&sync->lock, flags);

	if (sync->ops && sync->ops->free)
		sync->ops->free(sync->priv);

	kfree(sync);
}
EXPORT_SYMBOL_GPL(dmabuf_sync_fini);
