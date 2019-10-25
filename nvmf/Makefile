ifneq ($(KERNELRELEASE),)
obj-y		+= host/
else

OFA := ../../mlnx-ofa_kernel-4.2
K_VER := `uname -r`
K_SRC := /lib/modules/$(K_VER)/build

OFA_RDMA_H := $(OFA)/include/rdma
OFA_UAPI_RDMA_H := $(OFA)/include/uapi/rdma
OFA_COMPAT_H := $(OFA)/compat

HDIR := $(PWD)/host
RDMA_H := $(HDIR)/include/rdma
UAPI_RDMA_H := $(HDIR)/include/uapi/rdma
COMPAT_H := $(HDIR)/compat

#	CONFIG_NVME_CORE=m	\
#	CONFIG_BLK_DEV_NVME=m	\

all:
	make -C $(K_SRC) M=$(PWD) modules \
	CONFIG_NVME_FABRICS=m	\
	CONFIG_NVME_RDMA=m	\
	DEBUG=	\
	HAVE_BLK_MQ_ALLOC_REQUEST_HCTX= \
	LINUXINCLUDE=' \
		-include $(COMPAT_H)/config.h \
		-I$(HDIR)/include \
		-I$(HDIR)/include/uapi \
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
	mv ./host/*.ko ./nvmf-mod/

headers:
	- rm -rf $(RDMA_H)/*.h
	- rm -rf $(UAPI_RDMA_H)/*.h
	- mkdir -p $(RDMA_H)
	- mkdir -p $(UAPI_RDMA_H)
	- mkdir -p $(UAPI_RDMA_H)/hfi
	- mkdir -p $(COMPAT_H)
	- mkdir -p $(HDIR)/include/linux/sched
	cp $(OFA_RDMA_H)/ib_verbs.h $(RDMA_H)/
	cp $(OFA_RDMA_H)/ib_verbs_exp_def.h $(RDMA_H)/
	cp $(OFA_RDMA_H)/ib_verbs_exp.h $(RDMA_H)/
	- cp $(OFA_RDMA_H)/restrack.h $(RDMA_H)/
	cp $(OFA_RDMA_H)/rdma_cm.h $(RDMA_H)/
	cp $(OFA_RDMA_H)/mr_pool.h $(RDMA_H)/
	cp $(OFA_UAPI_RDMA_H)/hfi/hfi1_ioctl.h $(UAPI_RDMA_H)/hfi/hfi1_ioctl.h
	- cp $(OFA_UAPI_RDMA_H)/ib_user_ioctl_verbs.h $(UAPI_RDMA_H)/ib_user_ioctl_verbs.h
	cp $(OFA_UAPI_RDMA_H)/rdma_user_cm.h $(UAPI_RDMA_H)/rdma_user_cm.h
	cp $(OFA_UAPI_RDMA_H)/rdma_user_ioctl.h $(UAPI_RDMA_H)/rdma_user_ioctl.h
	- cp $(OFA_UAPI_RDMA_H)/rdma_user_ioctl_cmds.h $(UAPI_RDMA_H)/rdma_user_ioctl_cmds.h
	cp $(OFA_UAPI_RDMA_H)/ib_user_verbs.h $(UAPI_RDMA_H)/ib_user_verbs.h
	cp $(OFA_UAPI_RDMA_H)/ib_user_mad.h $(UAPI_RDMA_H)/ib_user_mad.h
	cp $(OFA)/include/linux/cgroup_rdma.h $(HDIR)/include/linux/
	cp $(OFA)/include/linux/sched/task.h $(HDIR)/include/linux/sched/
	cp $(OFA_COMPAT_H)/config.h $(COMPAT_H)

config:
	cd $(OFA) && ./configure --with-nvmf-host-without-fc --without-nvmf_target-mod --kernel-sources=$(K_SRC)

clean:
	rm -rf ./nvmf_out/* ./host*.ko ./host/*.o ./host/*.mod.c Module.symvers modules.order
endif
