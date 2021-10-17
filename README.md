tinydrm kernel module for Waveshare it8951 based eink kits.

Tested on raspberry pi zero w with kernel 4.19, only 6" panel is supported.

## Compile

```bash
cd tinydrm_it8951
make
```

you can test it with 
```bash
sudo sh utils/load.sh
```

## Enable display at boot

### Raspberry Pi

- copy it8951.ko in your /lib/modules directory (eg /lib/modules/4.19.66+/kernel/drivers/gpu/drm/tinydrm/)
- generate overlay : sudo make rpi_overlay
- add modules tinydrm and it8951 in /etc/modules
- add a line "dtoverlay=it8951" in /boot/config.txt
- add "fbcon=map:1 vt.color=0xf0" at the end of /boot/cmdline.txt
- to disable cursor blinking, add a line "echo 0 > /sys/class/graphics/fbcon/cursor_blink" in /etc/rc.local

### Beaglebone

- generate overlay: sudo make bb_spi0_overlay
- edit /boot/uEnv.txt to add dtb : dtb_overlay=/lib/firmware/BB-IT8951-SPI0-00A0.dtbo
- add "vt.color=0xf0" at the end of cmdline= in /boot/uEnv.txt

You can now reboot, the eink panel will be the screen by default

## Kernel module options

You should change the VCOM value to match your panel, eg for -2.04V:
```bash
modprobe it8951 vcom=-2040
```

I successfully got 24Mhz SPI clock on beaglebone, you can change default frequency:
```bash
modprobe it8951 spi_freq=24000000
```

Finally, the driver include an automatic waveform selector, you can disable it at module load and set a predefined mode:
```bash
modprobe it8951 update_mode=2
```

Where update_mode can be one of:
```c
#define IT8951_MODE_INIT  0
#define IT8951_MODE_DU    1
#define IT8951_MODE_GC16  2
#define IT8951_MODE_GL16  3
#define IT8951_MODE_GLR16 4
#define IT8951_MODE_GLD16 5
```

You can change the waveform at any moment with a /sys entry (eg on BBB):
```bash
cat /sys/devices/platform/ocp/48030000.spi/spi_master/spi0/spi0.0/update_mode
echo 2 > /sys/devices/platform/ocp/48030000.spi/spi_master/spi0/spi0.0/update_mode
```

You can also set the refresh rate for ghosting (number of refresh before full refresh):
```bash
cat /sys/devices/platform/ocp/48030000.spi/spi_master/spi0/spi0.0/ghosting_refresh
echo 50 > /sys/devices/platform/ocp/48030000.spi/spi_master/spi0/spi0.0/ghosting_refresh
```
