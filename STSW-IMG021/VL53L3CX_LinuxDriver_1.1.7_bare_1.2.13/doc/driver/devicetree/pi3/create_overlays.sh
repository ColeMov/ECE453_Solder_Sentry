#! /usr/bin/env bash
  MAJOR_VERSION=$(uname -r | awk -F '.' '{print $1}')
  MINOR_VERSION=$(uname -r | awk -F '.' '{print $2}')
  if [ $MAJOR_VERSION -ge 6 ] && [ $MINOR_VERSION -gt 1 ] ; then
	dtc -I dts -o stmvl53l1.dtbo -O dtb stmvl53l1_over_6_1.dts
	dtc -I dts -o stmvl53lx.dtbo -O dtb stmvl53lx_over_6_1.dts
  else
	dtc -I dts -o stmvl53l1.dtbo -O dtb stmvl53l1.dts
	dtc -I dts -o stmvl53lx.dtbo -O dtb stmvl53lx.dts
  fi

sudo cp stmvl53l1.dtbo /boot/overlays/stmvl53l1.dtbo
sudo cp stmvl53lx.dtbo /boot/overlays/stmvl53lx.dtbo
