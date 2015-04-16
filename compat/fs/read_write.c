#include <linux/aio.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/uio.h>
#include <linux/fsnotify.h>
#include <linux/security.h>
#include <linux/export.h>
#include <linux/syscalls.h>
#include <linux/pagemap.h>
#include <linux/splice.h>
#include <linux/compat.h>

#include <asm/uaccess.h>
#include <asm/unistd.h>

ssize_t vfs_iter_read(struct file *file, struct iov_iter *iter, loff_t *ppos)
{
	struct kiocb kiocb;
	ssize_t ret;

	if (!file->f_op->read_iter)
		return -EINVAL;

	init_sync_kiocb(&kiocb, file);
	kiocb.ki_pos = *ppos;

	iter->type |= READ;
	ret = file->f_op->read_iter(&kiocb, iter);
	BUG_ON(ret == -EIOCBQUEUED);
	if (ret > 0)
		*ppos = kiocb.ki_pos;
	return ret;
}
EXPORT_SYMBOL(vfs_iter_read);

ssize_t vfs_iter_write(struct file *file, struct iov_iter *iter, loff_t *ppos)
{
	struct kiocb kiocb;
	ssize_t ret;

	if (!file->f_op->write_iter)
		return -EINVAL;

	init_sync_kiocb(&kiocb, file);
	kiocb.ki_pos = *ppos;

	iter->type |= WRITE;
	ret = file->f_op->write_iter(&kiocb, iter);
	BUG_ON(ret == -EIOCBQUEUED);
	if (ret > 0)
		*ppos = kiocb.ki_pos;
	return ret;
}
EXPORT_SYMBOL(vfs_iter_write);
