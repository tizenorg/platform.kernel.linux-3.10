#ifndef _COMPAT_LINUX_FS_H
#define _COMPAT_LINUX_FS_H

#include_next <linux/fs.h>

ssize_t vfs_iter_read(struct file *file, struct iov_iter *iter, loff_t *ppos);
ssize_t vfs_iter_write(struct file *file, struct iov_iter *iter, loff_t *ppos);

#endif /* _COMPAT_LINUX_FS_H */
