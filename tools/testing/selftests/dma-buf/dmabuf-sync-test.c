#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>

#include "../../../include/uapi/linux/dma-buf-test.h"

#define DMA_BUF_DEV "/dev/dmabuf"

/* Global variables */
int thread_count = 1;

struct dmabuf_sync_thread_data {
	pthread_t thread;
	int thread_no;
	int fd;
}

void *dmabuf_sync_thread(void *data)
{
	struct dmabuf_sync_thread_data *sync_data = data;

}

static void usage(char *name)
{
	fprintf(stderr, "usage: %s [-s]\n", name);
	fprintf(stderr, "-t N : the number of threads\n");
	exit(0);
}

extern char *optarg;
static const char optstr[] = "t:";

int main(int argc, char **argv)
{
	int fd, c, err = 0;
	int i;
	struct dmabuf_create buf;
	struct dmabuf_sync_thread_data *data;

	while ((c = getopt(argc, argv, optstr)) != -1) {
		switch (c) {
		case 't':
			if (sscanf(optarg, "%d", &thread_count) != 1)
				usage(argv[0]);
			break;
		}
	}

	fd = open(DMA_BUF_DEV, O_RDWR);
	if (fd < 0) {
		perror("cannot open %s\n", DMA_BUF_DEV);
		return -1;
	}

	buf.flags = 0;
	buf.size = 1 * 1024 * 1024;

	err = ioctl(fd, DMABUF_IOCTL_CREATE, &buf);
	if (err < 0) {
		perror("ioctl DMABUF_IOCTL_CREATE error\n");
		goto err_ioctl;
	}

	data = malloc(sizeof(struct dmabuf_sync_thread_data) * thread_count);
	if (!data) {
		perror("cannot allocate memory\n");
		close(fd);
		return -1;
	}

	for (i = 0; i < thread_count; i++) {
		data[i].thread_no = i;
		pthread_create(&data[i].thread, NULL, dmabuf_sync_thread,
			       &data[i]);
	}

	for (i = 0; i < thread_count; i++) {
		void *val;
		pthread_join(data[i].thread, &val);
	}

err_alloc:
	ioctl(fd, DMABUF_IOCTL_DELETE, &buf);

err_ioctl:
	close(fd);

	return err;
}
