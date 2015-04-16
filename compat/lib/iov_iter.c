#include <linux/export.h>
#include <linux/uio.h>
#include <linux/pagemap.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <net/checksum.h>

void iov_iter_init(struct iov_iter *i, int direction,
			const struct iovec *iov, unsigned long nr_segs,
			size_t count)
{
	/* It will get better.  Eventually... */
	if (segment_eq(get_fs(), KERNEL_DS)) {
		direction |= ITER_KVEC;
		i->type = direction;
		i->kvec = (struct kvec *)iov;
	} else {
		i->type = direction;
		i->iov = iov;
	}
	i->nr_segs = nr_segs;
	i->iov_offset = 0;
	i->count = count;
}
EXPORT_SYMBOL(iov_iter_init);

void iov_iter_kvec(struct iov_iter *i, int direction,
			const struct kvec *kvec, unsigned long nr_segs,
			size_t count)
{
	BUG_ON(!(direction & ITER_KVEC));
	i->type = direction;
	i->kvec = kvec;
	i->nr_segs = nr_segs;
	i->iov_offset = 0;
	i->count = count;
}
EXPORT_SYMBOL(iov_iter_kvec);
