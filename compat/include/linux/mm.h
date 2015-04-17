#ifndef __COMPAT_LINUX_MM_H
#define __COMPAT_LINUX_MM_H

#include_next <linux/mm.h>

extern void truncate_inode_pages_final(struct address_space *);

#endif /* _COMPAT_LINUX_MM_H */
