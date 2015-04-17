#ifndef _LINUX_TIME64_H
#define _LINUX_TIME64_H

#include <linux/time.h>
#include <linux/ktime.h>

/* copied from struct timespec */
struct timespec64 {
	__kernel_time_t tv_sec;                 /* seconds */
	long            tv_nsec;                /* nanoseconds */
};

static inline void ktime_get_ts64(struct timespec64 *ts)
{
	ktime_get_ts((struct timespec *)ts);
}

static inline s64 timespec64_to_ns(const struct timespec64 *ts)
{
	return (s64) timespec_to_ns((struct timespec *)ts);
}

#endif /* _LINUX_TIME64_H */
