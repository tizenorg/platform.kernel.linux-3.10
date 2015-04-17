#ifndef __COMPAT_LINUX_PAGEMAP_H
#define __COMPAT_LINUX_PAGEMAP_H

#include_next <linux/pagemap.h>

#define AS_EXITING (__GFP_BITS_SHIFT + 5) /* final truncate in progress */

static inline void mapping_set_exiting(struct address_space *mapping)
{
	set_bit(AS_EXITING, &mapping->flags);
}

static inline int mapping_exiting(struct address_space *mapping)
{
	return test_bit(AS_EXITING, &mapping->flags);
}

#endif /* _COMPAT_LINUX_PAGEMAP_H */
