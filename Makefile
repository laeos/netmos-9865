KDIR:=/lib/modules/$(shell  uname -r)/build/

DEBIAN_VERSION_FILE:=/etc/debian_version
DEBIAN_DISTRO:=$(wildcard $(DEBIAN_VERSION_FILE))

obj-m +=mcs9865.o
obj-m +=mcs9865-isa.o

default:
	$(RM) *.mod.c *.o *.ko .*.cmd *.symvers
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
load:
	insmod mcs9865.ko
	insmod mcs9865-isa.ko

unload:
	rmmod mcs9865-isa
	rmmod mcs9865

DEST=/lib/modules/$(shell uname -r)/kernel/drivers/tty/serial
install:
	mkdir -p $(DEST)
	cp mcs9865.ko mcs9865-isa.ko $(DEST)
	depmod -A

uninstall:
	modprobe -r mcs9865
	modprobe -r mcs9865-isa
	rm $(DEST)/mcs9865*
	depmod -A

clean:
	$(RM) *.mod.c *.o *.ko .*.cmd *.symvers *.order *.markers
	$(RM) -r .tmp_versions
