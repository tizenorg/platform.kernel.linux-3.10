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
#include <linux/delay.h>

#include <linux/seqno-fence.h>
#include <linux/reservation.h>
#include <linux/dmabuf-sync.h>

#define DEFAULT_SYNC_TIMEOUT	2000	/* in millisecond. */
#define BEGIN_CPU_ACCESS(old, new_type)	\
			((old->accessed_type & DMA_BUF_ACCESS_DMA_W) == \
			  DMA_BUF_ACCESS_DMA_W && new_type == DMA_BUF_ACCESS_R)

#define END_CPU_ACCESS(old, new_type)	\
			(((old->accessed_type == DMA_BUF_ACCESS_W) || \
			 (old->accessed_type == DMA_BUF_ACCESS_RW)) && \
			 new_type & DMA_BUF_ACCESS_DMA)

static int dmabuf_sync_enabled = 1;
static unsigned long seqno;
static LIST_HEAD(orders);
static DEFINE_SPINLOCK(orders_lock);

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
	return true;
}

static void dmabuf_sync_object_release(struct fence *fence)
{
	struct seqno_fence *sf = to_seqno_fence(fence);

	kfree(sf);
}

static const struct fence_ops fence_default_ops = {
	.get_driver_name = dmabuf_sync_get_driver_name,
	.get_timeline_name = dmabuf_sync_get_timeline_name,
	.enable_signaling = dmabuf_sync_enable_sw_signaling,
	.wait = fence_default_wait,
	.release = dmabuf_sync_object_release,
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
	struct dma_buf *dmabuf = sobj->dmabuf;

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
struct fence *dmabuf_sync_check(struct dma_buf *dmabuf)
{
	struct reservation_object_list *rol;
	struct reservation_object *ro;
	struct fence *fence = NULL;
	unsigned int i;

	rcu_read_lock();
	ro = rcu_dereference(dmabuf->resv);
	if (!ro || !ro->fence)
		goto check_excl_fence;

	rol = rcu_dereference(ro->fence);

	/* Check if there is any fence requested for a read access. */
	for (i = 0; i < rol->shared_count; i++) {
		fence = rcu_dereference(rol->shared[i]);
		if (!fence)
			continue;

		break;
	}

	if (fence)
		goto unlock_rcu;

check_excl_fence:
	/* And then check if there is a fence requested for a write access. */
	fence = rcu_dereference(ro->fence_excl);

unlock_rcu:
	rcu_read_unlock();
	return fence;
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
	struct seqno_fence *sfence;

	sync = kzalloc(sizeof(*sync), GFP_KERNEL);
	if (!sync)
		return ERR_PTR(-ENOMEM);

	sfence = kzalloc(sizeof(*sfence), GFP_KERNEL);
	if (!sfence) {
		kfree(sync);
		return ERR_PTR(-ENOMEM);
	}

	strncpy(sync->name, name, DMABUF_SYNC_NAME_SIZE);

	sync->ops = ops;
	sync->sfence = sfence;
	sync->priv = priv;
	INIT_LIST_HEAD(&sync->syncs);
	spin_lock_init(&sync->lock);
	spin_lock_init(&sync->flock);

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
					unsigned int ctx, unsigned int type)
{
	struct dmabuf_sync_object *sobj;
	unsigned long s_flags;
	unsigned int i;

	if (!sync || (sync && !sync->sfence))
		return -EFAULT;

	if (!IS_VALID_DMA_BUF_ACCESS_TYPE(type))
		return -EINVAL;

	if ((type & DMA_BUF_ACCESS_RW) == DMA_BUF_ACCESS_RW)
		type &= ~DMA_BUF_ACCESS_R;

	sobj = kzalloc(sizeof(*sobj), GFP_KERNEL);
	if (!sobj)
		return -ENOMEM;

	spin_lock_init(&sobj->lock);
	sobj->access_type = type;
	sobj->sfence = sync->sfence;
	sobj->dmabuf = dmabuf;
	seqno_fence_init(sobj->sfence, &sync->flock, dmabuf, ctx,
				0, 0, ++seqno, &fence_default_ops);

	for (i = 0; i < sync->obj_cnt; i++)
		fence_get(&sync->sfence->base);

	sync->obj_cnt++;

	spin_lock_irqsave(&sync->lock, s_flags);
	list_add_tail(&sobj->l_head, &sync->syncs);
	spin_unlock_irqrestore(&sync->lock, s_flags);

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
	unsigned long s_flags;

	spin_lock_irqsave(&sync->lock, s_flags);
	list_for_each_entry_safe(sobj, next, &sync->syncs, l_head) {

		spin_unlock_irqrestore(&sync->lock, s_flags);

		list_del_init(&sobj->l_head);
		fence_put(&sobj->sfence->base);

		kfree(sobj);

		spin_lock_irqsave(&sync->lock, s_flags);
	}
	spin_unlock_irqrestore(&sync->lock, s_flags);
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
	unsigned long s_flags;

	spin_lock_irqsave(&sync->lock, s_flags);
	list_for_each_entry_safe(sobj, next, &sync->syncs, l_head) {
		spin_unlock_irqrestore(&sync->lock, s_flags);

		if (sobj->dmabuf != dmabuf)
			continue;

		list_del_init(&sobj->l_head);
		fence_put(&sobj->sfence->base);
		kfree(sobj);

		spin_lock_irqsave(&sync->lock, s_flags);
		break;
	}
	spin_unlock_irqrestore(&sync->lock, s_flags);
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
int dmabuf_sync_get(struct dmabuf_sync *sync, void *sync_buf,
			unsigned int ctx, unsigned int type)
{
	if (!sync || !sync_buf)
		return -EFAULT;

	return dmabuf_sync_get_obj(sync, sync_buf, ctx, type);
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
	unsigned long s_flags;

	if (!sync || !dmabuf) {
		WARN_ON(1);
		return;
	}

	spin_lock_irqsave(&sync->lock, s_flags);
	if (list_empty(&sync->syncs)) {
		spin_unlock_irqrestore(&sync->lock, s_flags);
		return;
	}
	spin_unlock_irqrestore(&sync->lock, s_flags);

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
	unsigned long s_flags;

	if (!sync) {
		WARN_ON(1);
		return;
	}

	spin_lock_irqsave(&sync->lock, s_flags);
	if (list_empty(&sync->syncs)) {
		spin_unlock_irqrestore(&sync->lock, s_flags);
		return;
	}
	spin_unlock_irqrestore(&sync->lock, s_flags);

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
	struct seqno_fence *sf = sobj->sfence;
	struct dma_buf *dmabuf = sobj->dmabuf;

	if (sobj->access_type & DMA_BUF_ACCESS_R) {
		struct reservation_object_list *fobj;

		fobj = reservation_object_get_list(dmabuf->resv);

		/*
		 * Reserve spaces for shared fences if this is the first call
		 * or there is no space for a new fence.
		 */
		if (!fobj || (fobj && fobj->shared_count == fobj->shared_max))
			reservation_object_reserve_shared(dmabuf->resv);

		reservation_object_add_shared_fence(dmabuf->resv, &sf->base);
	} else {
		reservation_object_add_excl_fence(dmabuf->resv, &sf->base);
	}
}

static struct dmabuf_sync_object *get_obj_from_req_queue(struct seqno_fence *sf)
{
	struct dmabuf_sync_object *sobj, *next, *out = NULL;
	unsigned long o_flags;

	spin_lock_irqsave(&orders_lock, o_flags);
	if (list_empty(&orders)) {
		/* There should be no such case. */
		WARN_ON(1);
		goto err;
	}

	list_for_each_entry_safe(sobj, next, &orders, g_head) {
		if (sobj->sfence == sf) {
			out = sobj;
			break;
		}
	}

err:
	spin_unlock_irqrestore(&orders_lock, o_flags);
	return out;
}


static void remove_obj_from_req_queue(struct dmabuf_sync_object *csobj)
{
	struct dmabuf_sync_object *sobj, *next;
	unsigned long o_flags;

	spin_lock_irqsave(&orders_lock, o_flags);
	if (list_empty(&orders)) {
		spin_unlock_irqrestore(&orders_lock, o_flags);
		/* There should be no such case. */
		WARN_ON(1);
		return;
	}

	list_for_each_entry_safe(sobj, next, &orders, g_head) {
		if (sobj == csobj) {
			list_del_init(&sobj->g_head);
			break;
		}

	}
	spin_unlock_irqrestore(&orders_lock, o_flags);
}

/*
 * make_sure_the_req_ordering- check if a given sobj was requested the first
 *				for buffer sync.
 *
 * @csobj: An object to dmabuf_sync_object for checking if buffer sync was
 *	requested the first for buffer sync.
 *
 * This function can be called by dmabuf_sync_wait and dmabuf_sync_wait_all
 * to check if there is any object that buffer sync was requested earlier
 * than current thread.
 * If there is a object, the current thread doesn't have the ownership of
 * a dmabuf so will try to check the priority again to yield the ownership
 * to other threads that requested buffer sync earlier than current thread.
 */
static bool is_higher_priority_than_current(struct dma_buf *dmabuf,
					struct dmabuf_sync_object *csobj)
{
	struct dmabuf_sync_object *sobj;
	unsigned long o_flags;
	bool ret = false;

	spin_lock_irqsave(&orders_lock, o_flags);
	if (list_empty(&orders)) {
		/* There should be no such case. */
		WARN_ON(1);
		goto out;
	}

	list_for_each_entry(sobj, &orders, g_head) {
		if (sobj != csobj)
			ret = true;
		break;
	}

out:
	spin_unlock_irqrestore(&orders_lock, o_flags);
	return ret;
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
	unsigned long s_flags;

	spin_lock_irqsave(&sync->lock, s_flags);
	list_for_each_entry(sobj, &sync->syncs, l_head) {
		struct dma_buf *dmabuf;
		struct seqno_fence *sf;
		unsigned long o_flags;
		bool all_wait;

		spin_unlock_irqrestore(&sync->lock, s_flags);

		/*
		 * orders is used to check if there is any sobj
		 * of other thread that requested earlier buffer sync than
		 * current thread.
		 */
		spin_lock_irqsave(&orders_lock, o_flags);
		list_add_tail(&sobj->g_head, &orders);
		spin_unlock_irqrestore(&orders_lock, o_flags);

		sf = sobj->sfence;
		dmabuf = sobj->dmabuf;

		/*
		 * It doesn't need to wait for other thread or threads
		 * if there is no any sync object which has higher priority
		 * than this one, sobj so go update a fence.
		 */
		if (!is_higher_priority_than_current(dmabuf, sobj))
			goto out_enable_signal;

go_back_to_wait:
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
			pr_warning("[DMA] signal wait has been timed out.\n");

		/*
		 * Check if there is any sobj of other thread that requested
		 * earlier than current thread. If other sobj, then it should
		 * yield the ownership of a buffer to other thread for buffer
		 * access ordering so go wait for the thread.
		 */
		if (is_higher_priority_than_current(dmabuf, sobj)) {
			/* TODO */
			goto go_back_to_wait;
		}

out_enable_signal:
		fence_enable_sw_signaling(&sf->base);
		dmabuf_sync_update(sobj);
		dmabuf_sync_cache_ops(sobj);

		spin_lock_irqsave(&sync->lock, s_flags);
	}
	spin_unlock_irqrestore(&sync->lock, s_flags);

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
long dmabuf_sync_wait(struct dma_buf *dmabuf, unsigned int ctx,
			unsigned int access_type)
{
	struct dmabuf_sync_object *sobj;
	unsigned long timeout = 0;
	bool all_wait;
	unsigned long o_flags;

	sobj = kzalloc(sizeof(*sobj), GFP_KERNEL);
	if (!sobj)
		return -ENOMEM;

	sobj->sfence = kzalloc(sizeof(struct seqno_fence), GFP_KERNEL);
	if (!sobj->sfence) {
		kfree(sobj);
		return -ENOMEM;
	}

	spin_lock_init(&sobj->lock);

	sobj->access_type = access_type;
	sobj->dmabuf = dmabuf;
	seqno_fence_init(sobj->sfence, &sobj->lock, dmabuf, ctx, 0, 0, ++seqno,
				&fence_default_ops);

	spin_lock_irqsave(&orders_lock, o_flags);
	list_add_tail(&sobj->g_head, &orders);
	spin_unlock_irqrestore(&orders_lock, o_flags);

	/*
	 * It doesn't need to wait for other thread or threads
	 * if there is no any sync object which has higher priority
	 * than this one, sobj so go update a fence.
	 */
	if (!is_higher_priority_than_current(dmabuf, sobj))
		goto out_enable_signal;

	all_wait = access_type & DMA_BUF_ACCESS_W;

	timeout = reservation_object_wait_timeout_rcu(dmabuf->resv,
			all_wait, true,
			msecs_to_jiffies(DEFAULT_SYNC_TIMEOUT));
	if (!timeout)
		pr_warning("[CPU] signal wait has been timed out.\n");

go_back_to_wait:
	/*
	 * Check if there is any sobj of other thread that requested
	 * earlier than current thread. If other sobj, then it should
	 * yield the ownership of a buffer to other thread for buffer
	 * access ordering so go wait for the thread.
	 */
	if (is_higher_priority_than_current(dmabuf, sobj)) {
		/* TODO */
		goto go_back_to_wait;
	}

out_enable_signal:
	fence_enable_sw_signaling(&sobj->sfence->base);
	dmabuf_sync_update(sobj);
	fence_put(&sobj->sfence->base);
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
	unsigned long s_flags;
	int ret = -EAGAIN;

	spin_lock_irqsave(&sync->lock, s_flags);
	list_for_each_entry(sobj, &sync->syncs, l_head) {
		spin_unlock_irqrestore(&sync->lock, s_flags);

		remove_obj_from_req_queue(sobj);

		ret = fence_signal(&sobj->sfence->base);
		if (ret) {
			pr_warning("signal request has been failed.\n");
			spin_lock_irqsave(&sync->lock, s_flags);
			break;
		}

		spin_lock_irqsave(&sync->lock, s_flags);
	}
	spin_unlock_irqrestore(&sync->lock, s_flags);

	return ret;
}
EXPORT_SYMBOL_GPL(dmabuf_sync_signal_all);

static int dmabuf_sync_signal_fence(struct fence *fence)
{
	struct dmabuf_sync_object *sobj;
	struct seqno_fence *sf;
	int ret;

	sf = to_seqno_fence(fence);
	sobj = get_obj_from_req_queue(sf);

	remove_obj_from_req_queue(sobj);

	ret = fence_signal(fence);
	if (ret)
		pr_warning("signal request has been failed.\n");

	kfree(sobj);

	return ret;
}

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
	struct reservation_object *ro;
	struct fence *fence;
	int i;

	rcu_read_lock();

	ro = rcu_dereference(dmabuf->resv);

	/* Was it a fence for a write? */
	fence = rcu_dereference(ro->fence_excl);
	if (fence) {
		if (fence->context == (unsigned int)current) {
			dmabuf_sync_signal_fence(fence);
			goto out;
		}
	}

	rol = rcu_dereference(ro->fence);
	if (!rol)
		goto out;

	/* Was it a fence for a read. */
	for (i = 0; i < rol->shared_count; i++) {
		fence = rcu_dereference(rol->shared[i]);
		if (!fence)
			continue;

		if (fence && fence->context != (unsigned int)current)
			continue;

		break;
	}

	if (fence)
		dmabuf_sync_signal_fence(fence);

out:
	rcu_read_unlock();

	return 0;
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
	unsigned long s_flags;

	if (WARN_ON(!sync))
		return;

	spin_lock_irqsave(&sync->lock, s_flags);
	if (list_empty(&sync->syncs))
		goto free_sync;

	/*
	 * It considers a case that a caller never call dmabuf_sync_put_all().
	 */
	dmabuf_sync_put_all(sync);

free_sync:
	spin_unlock_irqrestore(&sync->lock, s_flags);

	if (sync->ops && sync->ops->free)
		sync->ops->free(sync->priv);

	kfree(sync);
}
EXPORT_SYMBOL_GPL(dmabuf_sync_fini);
