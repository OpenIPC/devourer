# Out-of-tree build of the RTL8821CE TSF PHC prototype.
#   make            # build against the running kernel
#   make KDIR=/path # build against a specific kernel tree
obj-m += rtl8821ce_phc.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
