ifdef CONFIG_KDBUS
override LINUXINCLUDE := $(patsubst -Iinclude, -Icompat/include -Iinclude , $(LINUXINCLUDE))
obj-y += fs/
obj-y += lib/
obj-y += mm/
endif
