CFLAGS=
LDFLAGS=-lpthread

all:
	gcc $(CFLAGS) $(LDFLAGS) dmabuf-sync-test.c -o dmabuf-sync-test

run_tests: all
	@./dmabuf-sync-test || echo "dmabuf-sync-test: [FAIL]"

clean:
	rm -fr ./dmabuf-sync-test
