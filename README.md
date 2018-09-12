# nvmf-initiator-for-rhel3.10

NVMe over Fabrics host side modules for RHEL3.10 linux kernel.  
It is based on backport work of MLNX-ofed-4.3

# prepare befor compile

1. download the linux ofed tgz from mellanox's website. (tested on ofed 4.2, 4.3 and 4.4)  
2. un-tar the src/MLNX_OFED_SRC-$ver.tgz. \#tar zxf MLNX_OFED_SRC-$ver.tgz  
3. find the file in the SRPMS dir, whci named mlnx-ofa_kernel-$ver.src.rpm.  
4. \#rpm2cpio mlnx-ofa_kernel-$ver.src.rpm | cpio -id.  
5. get the mlnx-ofa_kernel-$ver.tgz and un-tar the tgz package.  
6. note the mlx-ofa_kernel-$ver dir as OFA.

# compile and install

1. git clone this project.  
2. eidt the Makefile to update $OFA path and $KVER.  
3. \#make config  
4. \#make headers  
5. \#make  
6. \#make install  
7. \#modprobe nvme-rdma  
8. \#modinfo nvme-rdma  
