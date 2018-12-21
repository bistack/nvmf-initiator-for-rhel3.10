K_VER := `uname -r`
K_DIR := /lib/modules/$(K_VER)

install:
	- rmmod nvme-rdma
	- rmmod nvme-fabrics
	insmod nvme-fabrics.ko
	- insmod nvme-rdma.ko
	- rm -rf $(K_DIR)/extra/mlnx-nvme
	- rm -rf $(K_DIR)/extra/mlnx-ofa_kernel/drivers/nvme
	- rm -rf $(K_DIR)/kernel/drivers/nvmf/host/
	- mkdir -p $(K_DIR)/kernel/drivers/nvmf/host/
	cp -f *.ko $(K_DIR)/kernel/drivers/nvmf/host/
	depmod $(K_VER)
