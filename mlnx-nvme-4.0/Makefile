OFA := ../../mlnx-ofa_kernel-4.3
K_VER := `uname -r`

ifneq ($(KERNELRELEASE),)
obj-y		+= host/
else

OFA_RDMA_H := $(OFA)/include/rdma
OFA_COMPAT_H := $(OFA)/compat

HDIR := $(PWD)/host
RDMA_H := $(HDIR)/include/rdma
COMPAT_H := $(HDIR)/compat

K_DIR := /lib/modules/$(K_VER)
K_SRC := $(K_DIR)/build

all:
	make -C $(K_SRC) M=$(PWD) modules \
	CONFIG_NVME_CORE=m	\
	CONFIG_BLK_DEV_NVME=m	\
	CONFIG_NVME_FABRICS=m	\
	CONFIG_NVME_RDMA=m	\
	DEBUG=	\
	HAVE_BLK_MQ_ALLOC_REQUEST_HCTX= \
	LINUXINCLUDE=' \
		-include $(COMPAT_H)/config.h \
		-I$(HDIR)/include \
		-I$$(srctree)/arch/$$(SRCARCH)/include \
		-Iarch/$$(SRCARCH)/include/generated \
		$$(if $$(KBUILD_SRC),-Iinclude2 -I$$(srctree)/include) \
		-I$$(srctree)/include \
		-I$$(srctree)/arch/$$(SRCARCH)/include/uapi \
		-Iarch/$$(SRCARCH)/include/generated/uapi \
		-I$$(srctree)/include/uapi \
		-Iinclude/generated/uapi \
		-include $$(srctree)/include/linux/kconfig.h \
		'
	mv ./host/*.ko ./nvmf_out/

headers:
	- mkdir -p $(RDMA_H)
	- mkdir -p $(COMPAT_H)
	- mkdir -p $(HDIR)/include/linux/sched
	cp $(OFA_RDMA_H)/ib_verbs.h $(RDMA_H)/
	cp $(OFA_RDMA_H)/ib_verbs_exp_def.h $(RDMA_H)/
	cp $(OFA_RDMA_H)/ib_verbs_exp.h $(RDMA_H)/
	- cp $(OFA_RDMA_H)/restrack.h $(RDMA_H)/
	cp $(OFA_RDMA_H)/rdma_cm.h $(RDMA_H)/ 
	cp $(OFA_RDMA_H)/mr_pool.h $(RDMA_H)/ 
	cp $(OFA)/include/linux/cgroup_rdma.h $(HDIR)/include/linux/
	cp $(OFA)/include/linux/sched/task.h $(HDIR)/include/linux/sched/
	cp $(OFA_COMPAT_H)/config.h $(COMPAT_H)

config:
	cd $(MLNX_OFED_SRC) && ./configure --with-nvmf-host-without-fc --without-nvmf_target-mod

clean:
	rm -rf ./nvmf_out/* ./host*.ko ./host/*.o ./host/*.mod.c Module.symvers modules.order
endif
