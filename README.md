tinydrm kernel module for Waveshare it8951 based eink kits

Tested on raspberry pi zero w with kernel 4.19, only 6" panel is supported

Setup (on raspian):
```
$ cd tinydrm_it8951
$ make
```

you can test it with 
```
$ sudo sh utils/load.sh
```

To enable the screen at boot:

Raspberry Pi
- copy it8951.ko in your /lib/modules directory (eg /lib/modules/4.19.66+/kernel/drivers/gpu/drm/tinydrm/)
- generate overlay : sudo make rpi_overlay
- add modules tinydrm and it8951 in /etc/modules
- add a line "dtoverlay=it8951" in /boot/config.txt
- add "fbcon=map:1 vt.color=0xf0" at the end of /boot/cmdline.txt
- to disable cursor blinking, add a line "echo 0 > /sys/class/graphics/fbcon/cursor_blink" in /etc/rc.local

Beaglebone
   - generate overlay: sudo make bb_spi0_overlay
   - sudo cp IT8951-00A0.dtbo /lib/firmware/
   - edit /boot/uEnv.txt to add dtb : dtb_overlay=/lib/firmware/IT8951-00A0.dtbo
    - add "vt.color=0xf0" at the end of cmdline= in /boot/uEnv.txt

You can now reboot, the eink panel will be the screen by default
