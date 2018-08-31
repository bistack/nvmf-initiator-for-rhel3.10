ifneq ($(KERNELRELEASE),)
obj-y		+= host/
else
K_VER := `uname -r`
K_DIR := /lib/modules/$(K_VER)
K_SRC := $(K_DIR)/build

all:
	make -C $(K_SRC) M=$(PWD) modules \
	CONFIG_NVME_CORE=m	\
	CONFIG_BLK_DEV_NVME=m	\
	CONFIG_NVME_FABRICS=m	\
	CONFIG_NVME_RDMA=m
install:
	- cp $(K_DIR)/kernel/drivers/block/nvme.ko ./
	- rm -rf $(K_DIR)/kernel/drivers/block/nvme.ko
	- rm -rf $(K_DIR)/extra/mlnx-nvme
	- rm -rf $(K_DIR)/extra/mlnx-ofa_kernel/drivers/nvme
	- mkdir -p $(K_DIR)/kernel/drivers/nvme/host/
	- cp ./host/*.ko $(K_DIR)/kernel/drivers/nvme/host/
	depmod $(K_VER)
clean:
	rm -rf ./host*.ko ./host/*.o ./host/*.mod.c Module.symvers modules.order
endif
