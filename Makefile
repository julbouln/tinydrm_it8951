KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

rpi_overlay:
	dtc -I dts -O dtb -o boot/overlays/it8951.dtb rpi-overlays/it8951-overlay.dts
bb_spi0_overlay:
	cpp -x assembler-with-cpp -nostdinc -I /opt/source/dtb-4.19-ti/include -undef -D__DTS__ bbb-overlays/BB-IT8951-SPI0-00A0.dts | dtc -i /opt/source/dtb-4.19-ti/include -I dts -O dtb -b 0 -o /lib/firmware/BB-IT8951-SPI0-00A0.dtbo -
bb_spi1_overlay:
	cpp -x assembler-with-cpp -nostdinc -I /opt/source/dtb-4.19-ti/include -undef -D__DTS__ bbb-overlays/BB-IT8951-SPI1-00A0.dts | dtc -i /opt/source/dtb-4.19-ti/include -I dts -O dtb -b 0 -o /lib/firmware/BB-IT8951-SPI1-00A0.dtbo -
pb_spi0_overlay:
	cpp -x assembler-with-cpp -nostdinc -I /opt/source/dtb-4.19-ti/include -undef -D__DTS__ bbb-overlays/PB-IT8951-SPI0-00A0.dts | dtc -i /opt/source/dtb-4.19-ti/include -I dts -O dtb -b 0 -o /lib/firmware/PB-IT8951-SPI0-00A0.dtbo -
pb_spi0_overlay_ups:
	cpp -x assembler-with-cpp -nostdinc -I /opt/source/dtb-4.19-ti/include -undef -D__DTS__ bbb-overlays/PB-IT8951-SPI0-UPS-00A0.dts | dtc -i /opt/source/dtb-4.19-ti/include -I dts -O dtb -b 0 -o /lib/firmware/PB-IT8951-SPI0-UPS-00A0.dtbo -
