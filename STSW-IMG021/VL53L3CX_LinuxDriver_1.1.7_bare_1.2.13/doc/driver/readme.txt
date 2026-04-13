# vl53l1 Linux driver documentation

# driver compilation options
- define VL53L1_LOG_ENABLE to compile with low level traces

# driver parameters

## forcing device insertion

For debug purpose it's sometimes useful to force device insertion when inserting module. For that you can use following module parameters:

- force_device: set it to one to force device insertion when inserting module.
- adapter_nb: i2c adapter to use.
- xsdn_gpio_nb: gpio number to which vl53l1 xsdn pin is connected.
- intr_gpio_nb: gpio number to which vl53l1 interrupt pin is connected. This parameter is optional.
- pwren_gpio_nb: gpio number use to control vl53l1 power. This parameter is optional.

For example to force device insertion on a pi3 platform with a vl53l1 module and a photonics plug-in use following command line:
```
insmod stmvl53l1.ko force_device=1 adapter_nb=1 xsdn_gpio_nb=19 intr_gpio_nb=16 pwren_gpio_nb=12
```

## control vl53l1 reset state when device is not ranging

Use optional force_device_on_en_default to control device reset state when in stop mode. Default value is 'yes' and so by default device will not be under reset when stopped. Select value 'no' to change behavior.

## control low level traces

If driver is compile with VL53L1_LOG_ENABLE option then you can control either at load time or at run time which low level traces are redirected to kernel log. For that you can use following module parameters:

- trace_function is a boolean that control tracing of function entry/exit. Default value is 'no'. Can be control at runtime using '/sys/module/stmvl53l1/parameters/trace_function' path.
- trace_module is a bit field integer that allow to enable/disable tracing per module. See vl53l1_platform_log.h file for possible values. Can be control at run time using '/sys/module/stmvl53l1/parameters/trace_module' path.
- trace_level is an integer that allow to control level of tracing. See vl53l1_platform_log.h file for possible values. Can be control at run time using '/sys/module/stmvl53l1/parameters/trace_level' path.

# user interfaces

## ioctl interface

This is the main interface to control and get data from vl53l1 device. You find list of supported ioctls with their description in stmvl53l1_if.h file.

## sysfs interface

This is an alternative interface to control vl53l1 device. When inserted an vl53l1 device will have an associated input directory in '/sys/class/input/inputxx/' where you can find file to control device through sysfs. Below is the list of those file that doesn't have a one to one relation with ioctls and ioctls parameters.

- name: will return 'STM VL53L1 proximity sensor'.
- enable_ps_sensor: allow to start/stop device. This is equivalent to VL53L1_IOCTL_START/VL53L1_IOCTL_STOP ioctls.
- set_delay_ms: control polling delay. This is equivalent to VL53L1_POLLDELAY_PAR.
- enable_debug: dynamically enable/disable vl53l1 debug traces. It will only output traces if driver was compile with DEBUG flag. Default value is 'Y'.
- do_flush: A write in this file will triggered insertion of an ABS_GAS event. Associated value is the number of such flushes requested.
- tuning_status: This a read-only file that allow to display all tuning parameters key/value.

## input sub-system interface

vl53l1 driver expose an event file where a set of events will be pushed for each data measure. This file will be name '/dev/input/eventxx'. You can check it is vl53l1 file with EVIOCGNAME ioctl call and check return string has value 'STM VL53L1 proximity sensor'.
At each data measure a set of event object will be inserted in input sub-system. Below is the list of events with meaning of the associated value:

- ABS_DISTANCE: distance of object in cm.
- ABS_HAT0X: second part of the time-stamp.
- ABS_HAT0Y: microsecond part of the time-stamp.
- ABS_HAT1X: distance of object in mm.
- ABS_HAT1Y: range status code.
- ABS_HAT2X: return signal rate. It's 16.16 fix point value.
- ABS_HAT2Y: return ambient rate. It's 16.16 fix point value.
- ABS_HAT3X: sigma value of measure in mm.
- ABS_HAT3Y: Dmax distance in mm.
- ABS_WHEEL: sigma limit check use for measure in mm.
- ABS_PRESSURE: Effective SPAD count for the return signal. Divide by 256 to obtain real value.
- ABS_BRAKE: pack four different sub-fields of 8 bits each. From msb to lsb:
  - RoiStatus: status value of region of interest.
  - roi: index of region of interest.
  - obj_number: number of object for the current roi.
  - obj: index of object.
- ABS_TILT_X: pack two different sub-fields of 16 bits each. From msb to lsb:
  - RangeMaxMilliMeter: max object distance value.
  - RangeMinMilliMeter: min object distance value.
- ABS_TOOL_WIDTH: quality level in percentage from 0 to 100.

 Each set of such events will be separated by an input sync event (EV_SYN/SYN_REPORT combo). Be aware that Linux input subsystem will not send an event again if data has not be changed.

# device state

vl53l1 device can be in two different states. Either it's in stop state or in start (aka ranging) state.
When in stop state you can configure device using various iotcls (or sysfs parameters). Then once your device is configured you can go in start state using VL53L1_IOCTL_START. 
When in start state vl53l1 device is continuously ranging. In that mode you can obtain ranging data using one of the four data ioctls (VL53L1_IOCTL_GETDATAS, VL53L1_IOCTL_MZ_DATA, VL53L1_IOCTL_GETDATAS_BLOCKING and VL53L1_IOCTL_MZ_DATA_BLOCKING). When in start mode you can't change device configuration on the fly with the exception of timing budget value parameter. If you want to change device parameters you need to set device in stop mode, change parameters and start device again.

# calibration 

vl53l1 device driver offer two ioctls to fulfill factory calibration purpose.
- VL53L1_IOCTL_PERFORM_CALIBRATION: will perform one of the three possible calibration.
  - VL53L1_CALIBRATION_REF_SPAD: will trigger a reference spad calibration
  - VL53L1_CALIBRATION_CROSSTALK: will trigger a crosstalk calibration and is only need when using a cover glass.
  - VL53L1_CALIBRATION_OFFSET: will trigger zero offset calibration.
  - VL53L1_CALIBRATION_OFFSET_PER_ZONE: will trigger zero offset for a given zone of interest configuration.
- VL53L1_IOCTL_CALIBRATION_DATA: will allow to get/set calibration data for calibrations VL53L1_CALIBRATION_REF_SPAD, VL53L1_CALIBRATION_CROSSTALK and VL53L1_CALIBRATION_OFFSET.
- VL53L1_IOCTL_ZONE_CALIBRATION_DATA: will allow to get/set zone calibration data for calibration VL53L1_CALIBRATION_OFFSET_PER_ZONE.

Typical calibration step sequence is the following:
1. restore previous calibration data step using VL53L1_IOCTL_CALIBRATION_DATA in set mode.
2. perform step calibration step using VL53L1_IOCTL_PERFORM_CALIBRATION
3. retrieve new calibration data using VL53L1_IOCTL_CALIBRATION_DATA in get mode.

Then when device is in normal usage before ranging you should restore factory calibration using VL53L1_IOCTL_CALIBRATION_DATA in set mode.

# tuning parameters

It's possible to fine tune some settings using tuning interface. Once you are ok with a set of tuning parameters you can generate stm31vl53l1_tunings.h file so they are applied at device insertion.


# driver device tree documentation

you can find stmvl53l1 binding ABI in devicetree/st,stmvl53l1.txt

## device tree example for raspbian

### insert device only once

raspbian support dynamic overlay. This means you can insert your device
dynamically. For that you need a dts overlay. You can find one for pi3
board in devicetree/pi3/stmvl53l1.dts.
Copy it on your board and compile it by running following command
```
dtc -I dts -O dtb -o stmvl53l1.dtbo stmvl53l1.dts
```
Then you have to insert it using dtoverlay command
```
dtoverlay stmvl53l1.dtbo
```
And you check your device has been inserted using
```
dtc -I fs /proc/device-tree
```

### permanent insertion

But the next time you reboot your device will not be inserted unless you call
again dtoverlay command. You can make the change persistent to reboot by
executing following instructions
* First copy stmvl53l1.dtbo into /boot/overlays directory
* Add following line to /boot/config.txt file
```
dtoverlay=stmvl53l1
```
On next reboot you can verify that your device is present using
```
dtc -I fs /proc/device-tree
```
