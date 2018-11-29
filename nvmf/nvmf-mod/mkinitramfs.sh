is6u3=0
grep 6.3 /etc/redhat-release > /dev/null
if [ $? -eq 0 ]; then
    is6u3=1
fi

mkdir ./tmp
t=`date +"%Y%m%d%H%M%S"`

if [ $is6u3 -eq 1 ]; then
    installkernel_conf=/usr/share/dracut/modules.d/90kernel-modules/installkernel
    cp $installkernel_conf ./tmp/installkernel_$t
    sed /scsi_wait_scan/d ./tmp/installkernel_$t > ./tmp/installkernel.new
    cp ./tmp/installkernel.new $installkernel_conf

    init_sh=/usr/share/dracut/modules.d/99base/init
    cp $init_sh ./tmp/init_$t
    sed /scsi_wait_scan/d ./tmp/init_$t > ./tmp/init.new
    cp ./tmp/init.new $init_sh
fi

if [ ! -e /boot/vmlinuz-${K_VER}.bak ]; then
    mv /boot/vmlinuz-${K_VER} /boot/vmlinuz-${K_VER}.bak
fi

K_VER=`uname -r`
/sbin/dracut --force --add-drivers "nvme" --omit-drivers "mlx5_core mlx5_ib mlx_compat" /boot/initrd-${K_VER}.img ${K_VER}
rm -rf /boot/initrd-${K_VER}kdump.img

sync
echo 3 > /proc/sys/vm/drop_caches
sync

cp -f /boot/initrd-${K_VER}.img ./tmp/i.gz
cd ./tmp/
gunzip i.gz
cpio -id < i
find -name "nvme*.ko"
