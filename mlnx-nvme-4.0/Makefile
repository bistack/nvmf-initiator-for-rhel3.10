ifneq ($(KERNELRELEASE),)
obj-y		+= host/
else
K_SRC := /lib/modules/`uname -r`/build

all:
	make -C $(K_SRC) M=$(PWD) modules \
	CONFIG_NVME_CORE=m	\
	CONFIG_BLK_DEV_NVME=m	\
	CONFIG_NVME_FABRICS=m	\
	CONFIG_NVME_RDMA=m
clean:
	rm -rf ./host*.ko ./host/*.o ./host/*.mod.c Module.symvers modules.order
endif
