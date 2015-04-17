#ifndef __COMPAT_LINUX_UIO_H
#define __COMPAT_LINUX_UIO_H

#include_next <linux/uio.h>

/* other compatibility headers */
#include <linux/time64.h>
#include <linux/fs.h>

enum {
	ITER_IOVEC = 0,
	ITER_KVEC = 2,
	ITER_BVEC = 4,
};

void iov_iter_init(struct iov_iter *i, int direction, const struct iovec *iov,
			unsigned long nr_segs, size_t count);

void iov_iter_kvec(struct iov_iter *i, int direction, const struct kvec *kvec,
			unsigned long nr_segs, size_t count);

#endif /* _COMPAT_LINUX_UIO_H */
