#ifndef __COMPAT_LINUX_ERR_H
#define __COMPAT_LINUX_ERR_H

#include_next <linux/err.h>

#ifndef __ASSEMBLY__

static inline int __must_check PTR_ERR_OR_ZERO(__force const void *ptr)
{
	if (IS_ERR(ptr))
		return PTR_ERR(ptr);
	else
		return 0;
}

/* Deprecated */
#define PTR_RET(p) PTR_ERR_OR_ZERO(p)

#endif

#endif /* _COMPAT_LINUX_ERR_H */
