#!/bin/sh
sudo dtc -@ -I dts -O dtb -o /boot/overlays/it8951.dtbo rpi-overlays/it8951-overlay.dts
dtoverlay it8951
modprobe tinydrm
insmod it8951.ko
