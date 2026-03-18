#!/bin/bash
# Script ran on Raspberry PI3 kernel 4.9.35-v7+

# some GPÏOs are meant to be used by the kernel driver we're building
echo 12 > /sys/class/gpio/unexport
echo 16 > /sys/class/gpio/unexport
echo 19 > /sys/class/gpio/unexport

# update DRIVER_DIR according to the folder you installed the delivery
export DRIVER_DIR=${PWD}/vl53Lx/
export PHIO_DIR=${PWD}/../android/hardware/vl53lx_test/

cd ${DRIVER_DIR}
make clean
make

# Device tree overlay installation & loading
sudo dtoverlay -R
sudo cp ${DRIVER_DIR}/stmvl53lx.dtbo /boot/overlays/.
sudo dtoverlay stmvl53lx

# Driver installation
sudo insmod stmvl53lx.ko
sudo chmod 777 /dev/stmvl53lx_ranging

# phio application build
cd ${PHIO_DIR}
make clean
make
# Launch a ranging test
sudo ./phio -s -Z=10 -S
