K_VER := `uname -r`
K_DIR := /lib/modules/$(K_VER)

install:
	- rmmod nvme-rdma
	- rmmod nvme-fabrics
	insmod nvme-fabrics.ko
	insmod nvme-rdma.ko
	- cp $(K_DIR)/kernel/drivers/block/nvme.ko ./
	- rm -rf $(K_DIR)/kernel/drivers/block/nvme.ko
	- rm -rf $(K_DIR)/extra/mlnx-nvme
	- rm -rf $(K_DIR)/extra/mlnx-ofa_kernel/drivers/nvme
	- mkdir -p $(K_DIR)/kernel/drivers/nvme/host/
	- cp *.ko $(K_DIR)/kernel/drivers/nvme/host/
	depmod $(K_VER)
