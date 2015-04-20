#ifndef __COMPAT_LINUX_CGROUP_H
#define __COMPAT_LINUX_CGROUP_H

#include_next <linux/cgroup.h>

extern char *__task_cgroup_path(struct task_struct *task, char *buf, size_t buflen);

#define task_cgroup_path(task, buf, buflen) __task_cgroup_path(task, buf, buflen)

#endif /* _COMPAT_LINUX_CGROUP_H */
