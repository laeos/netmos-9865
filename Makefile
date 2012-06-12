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
unload:
	rmmod mcs9865

install:
	cp mcs9865.ko mcs9865-isa.ko /lib/modules/$(shell uname -r)/kernel/drivers/serial/
	depmod -A
	chmod +x mcs9865
	cp mcs9865 /etc/init.d/
ifeq ($(DEBIAN_DISTRO), $(DEBIAN_VERSION_FILE))
	ln -s /etc/init.d/mcs9865 /etc/rcS.d/S99mcs9865 || true
else
	ln -s /etc/init.d/mcs9865 /etc/rc3.d/S99mcs9865 || true  	
	ln -s /etc/init.d/mcs9865 /etc/rc5.d/S99mcs9865 || true
endif
	modprobe mcs9865
	modprobe mcs9865-isa	

uninstall:
	modprobe -r mcs9865
	modprobe -r mcs9865-isa
	rm /lib/modules/$(shell uname -r)/kernel/drivers/serial/mcs9865*
	depmod -A
	rm -f /etc/init.d/mcs9865
ifeq ($(DEBIAN_DISTRO), $(DEBIAN_VERSION_FILE))
	rm -f /etc/rcS.d/S99mcs9865
else
	rm -f /etc/rc3.d/S99mcs9865
	rm -f /etc/rc5.d/S99mcs9865
endif

clean:
	$(RM) *.mod.c *.o *.ko .*.cmd *.symvers *.order *.markers
	$(RM) -r .tmp_versions
