ifneq ($(KERNELRELEASE),)
obj-y		+= host/
else
K_DIR := /lib/modules/`uname -r`
K_SRC := $(K_DIR)/build

all:
	make -C $(K_SRC) M=$(PWD) modules \
	CONFIG_NVME_CORE=m	\
	CONFIG_BLK_DEV_NVME=m	\
	CONFIG_NVME_FABRICS=m	\
	CONFIG_NVME_RDMA=m
install:
	cp ./host/*.ko $(K_DIR)/kernel/drivers/nvme/host/
clean:
	rm -rf ./host*.ko ./host/*.o ./host/*.mod.c Module.symvers modules.order
endif
