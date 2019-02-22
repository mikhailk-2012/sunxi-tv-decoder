#!/bin/bash

export ARCH=arm
export CROSS_COMPILE=/opt/linaro/gcc-linaro-7.2.1-2017.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
export KDIR="/mnt/a20work/MarsBoard-A20-Linux-SDK-V1.1/linux-sunxi/build-mba20"
export INSTALL_MOD_PATH="/mnt/a20work/MarsBoard-A20-Linux-SDK-V1.1/sunxi-mali/mod_"

rm -f /tftpboot/sunxi_tvd.ko
make clean
make
cp sunxi_tvd.ko /tftpboot
