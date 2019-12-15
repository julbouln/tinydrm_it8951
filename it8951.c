/*
 * Copyright 2019 Julien Boulnois
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/sched/clock.h>
#include <linux/spi/spi.h>
#include <linux/pm.h>

#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/tinydrm/tinydrm.h>
#include <drm/tinydrm/tinydrm-helpers.h>

//Built in I80 Command Code
#define IT8951_TCON_SYS_RUN      0x0001
#define IT8951_TCON_STANDBY      0x0002
#define IT8951_TCON_SLEEP        0x0003
#define IT8951_TCON_REG_RD       0x0010
#define IT8951_TCON_REG_WR       0x0011
#define IT8951_TCON_MEM_BST_RD_T 0x0012
#define IT8951_TCON_MEM_BST_RD_S 0x0013
#define IT8951_TCON_MEM_BST_WR   0x0014
#define IT8951_TCON_MEM_BST_END  0x0015
#define IT8951_TCON_LD_IMG       0x0020
#define IT8951_TCON_LD_IMG_AREA  0x0021
#define IT8951_TCON_LD_IMG_END   0x0022

//I80 User defined command code
#define USDEF_I80_CMD_DPY_AREA     0x0034
#define USDEF_I80_CMD_DPY_BUF_AREA 0x0037
#define USDEF_I80_CMD_PWR_SEQ 	   0x0038
#define USDEF_I80_CMD_VCOM		   0x0039

#define USDEF_I80_CMD_GET_DEV_INFO 0x0302

//Rotate mode
#define IT8951_ROTATE_0     0
#define IT8951_ROTATE_90    1
#define IT8951_ROTATE_180   2
#define IT8951_ROTATE_270   3

//Pixel mode , BPP - Bit per Pixel
#define IT8951_2BPP   0
#define IT8951_3BPP   1
#define IT8951_4BPP   2
#define IT8951_8BPP   3

//Waveform Mode
#define IT8951_MODE_0   0
#define IT8951_MODE_1   1
#define IT8951_MODE_2   2
#define IT8951_MODE_3   3
#define IT8951_MODE_4   4

#define IT8951_MODE_INIT  0
#define IT8951_MODE_DU    1
#define IT8951_MODE_GC16  2
#define IT8951_MODE_GL16  3
#define IT8951_MODE_GLR16 4
#define IT8951_MODE_GLD16 5
// the following mode does not seems to work ...
#define IT8951_MODE_A2    6
#define IT8951_MODE_DU4   7

//Endian Type
#define IT8951_LDIMG_L_ENDIAN   0
#define IT8951_LDIMG_B_ENDIAN   1
//Auto LUT
#define IT8951_DIS_AUTO_LUT   0
#define IT8951_EN_AUTO_LUT    1
//LUT Engine Status
#define IT8951_ALL_LUTE_BUSY 0xFFFF

//-----------------------------------------------------------------------
// IT8951 TCon Registers defines
//-----------------------------------------------------------------------
//Register Base Address
#define DISPLAY_REG_BASE 0x1000               //Register RW access for I80 only
//Base Address of Basic LUT Registers
#define LUT0EWHR  (DISPLAY_REG_BASE + 0x00)   //LUT0 Engine Width Height Reg
#define LUT0XYR   (DISPLAY_REG_BASE + 0x40)   //LUT0 XY Reg
#define LUT0BADDR (DISPLAY_REG_BASE + 0x80)   //LUT0 Base Address Reg
#define LUT0MFN   (DISPLAY_REG_BASE + 0xC0)   //LUT0 Mode and Frame number Reg
#define LUT01AF   (DISPLAY_REG_BASE + 0x114)  //LUT0 and LUT1 Active Flag Reg
//Update Parameter Setting Register
#define UP0SR (DISPLAY_REG_BASE + 0x134)      //Update Parameter0 Setting Reg

#define UP1SR     (DISPLAY_REG_BASE + 0x138)  //Update Parameter1 Setting Reg
#define LUT0ABFRV (DISPLAY_REG_BASE + 0x13C)  //LUT0 Alpha blend and Fill rectangle Value
#define UPBBADDR  (DISPLAY_REG_BASE + 0x17C)  //Update Buffer Base Address
#define LUT0IMXY  (DISPLAY_REG_BASE + 0x180)  //LUT0 Image buffer X/Y offset Reg
#define LUTAFSR   (DISPLAY_REG_BASE + 0x224)  //LUT Status Reg (status of All LUT Engines)

#define BGVR      (DISPLAY_REG_BASE + 0x250)  //Bitmap (1bpp) image color table

//-------System Registers----------------
#define SYS_REG_BASE 0x0000

//Address of System Registers
#define I80CPCR (SYS_REG_BASE + 0x04)
//-------Memory Converter Registers----------------
#define MCSR_BASE_ADDR 0x0200
#define MCSR (MCSR_BASE_ADDR + 0x0000)
#define LISAR (MCSR_BASE_ADDR + 0x0008)

//#define IT8951_DEBUG

struct it8951_load_img_info
{
	uint16_t endian_type; //little or Big Endian
	uint16_t pixel_format; //bpp
	uint16_t rotate; //Rotate mode
	uint32_t start_fb_addr; //Start address of source Frame buffer
	uint32_t img_buf_base_addr;//Base address of target image buffer

};

//structure prototype 2
struct it8951_area_img_info
{
	uint16_t x;
	uint16_t y;
	uint16_t width;
	uint16_t height;
};

struct it8951_dev_info
{
	uint16_t panel_w;
	uint16_t panel_h;
	uint16_t img_buf_addr_l;
	uint16_t img_buf_addr_h;
	uint16_t fw_version[8]; 	//16 Bytes String
	uint16_t lut_version[8]; 	//16 Bytes String
};

struct it8951_epd {
	struct tinydrm_device tinydrm;
	struct spi_device *spi;

	struct gpio_desc *reset;
	struct gpio_desc *hrdy;

	struct it8951_dev_info dev_info;

	uint32_t img_buf_addr;

	uint32_t last_full_refresh;

	int update_mode;

	bool little_endian;
	bool enabled;
	bool running;

	uint8_t *buf;
	struct drm_gem_cma_object *bo;
};

static void it8951_wait_for_ready(struct it8951_epd *epd, int us)
{
	int waited = 0;
	while (!gpiod_get_value_cansleep(epd->hrdy) && waited < 1000000) {
		usleep_range(us, us * 2);
		waited += us;
	}
#ifdef IT8951_DEBUG
		printk(KERN_INFO "it8951: wait_for_ready %d\n",waited);
#endif
}

/* SPI data transfer */

static inline void it8951_memcpy_swab16(struct it8951_epd *epd, u16 *dst, u16 *src, size_t len)
{
	if (epd->little_endian) {
		int i;
		for (i = 0; i < len; i++) {
			*dst++ = swab16(*src++);
		}
	} else {
		memcpy(dst, src, len);
	}
}

static inline u16 it8951_swab16(struct it8951_epd *epd, u16 data) {
	if (epd->little_endian) {
		return swab16(data);
	} else {
		return data;
	}
}

static int it8951_spi_transfer(struct it8951_epd *epd, uint16_t preamble, bool dummy, const void *tx, void *rx, uint32_t len) {
	int speed_hz = 12000000; // can't get it works at > 12Mhz
	int ret;
	u8 *txbuf = NULL, *rxbuf = NULL;
	uint16_t spreamble = it8951_swab16(epd, preamble);

#ifdef IT8951_DEBUG
	if(tx)
		printk(KERN_INFO "it8951: it8951_spi_transfer preamble:%x len:%d tx:%x\n",preamble, len, ((uint16_t *)tx)[0]);
	else
		printk(KERN_INFO "it8951: it8951_spi_transfer preamble:%x len:%d\n",preamble, len);
#endif

	if (tx) {
		txbuf = kmalloc(len, GFP_KERNEL);
		if (!txbuf) {
			ret = -ENOMEM;
			goto out_free;
		}
		it8951_memcpy_swab16(epd, (uint16_t *)txbuf, (uint16_t *)tx, len / 2);
	}

	if (rx) {
		rxbuf = kmalloc(len, GFP_KERNEL);
		if (!rxbuf) {
			ret = -ENOMEM;
			goto out_free;
		}
	}

	it8951_wait_for_ready(epd, 100);

	if (dummy) {
		uint16_t dummy = 0;
		struct spi_transfer tr[3] = {};

		tr[0].tx_buf = &spreamble;
		tr[0].len = 2;
		tr[0].speed_hz = speed_hz;

		tr[1].rx_buf = &dummy;
		tr[1].len = 2;
		tr[1].speed_hz = speed_hz;

		tr[2].tx_buf = txbuf;
		tr[2].rx_buf = rxbuf;
		tr[2].len = len;
		tr[2].speed_hz = speed_hz;

		ret = spi_sync_transfer(epd->spi, tr, 3);
	} else {
		struct spi_transfer tr[2] = {};

		tr[0].tx_buf = &spreamble;
		tr[0].len = 2;
		tr[0].speed_hz = speed_hz;

		tr[1].tx_buf = txbuf;
		tr[1].rx_buf = rxbuf;
		tr[1].len = len;
		tr[1].speed_hz = speed_hz;

		ret = spi_sync_transfer(epd->spi, tr, 2);
	}

	if (rx && !ret) {
		it8951_memcpy_swab16(epd, (uint16_t *)rx, (uint16_t *)rxbuf, len / 2);

#ifdef IT8951_DEBUG
//		int i;
//		for(i=0;i<len;i+=4) {
//			printk(KERN_INFO "it8951: it8951_spi_transfer preamble:%x len:%d %d:%x %d:%x %d:%x %d:%x\n",preamble, len, i,rxbuf[i],i+1,rxbuf[i+1],i+2,rxbuf[i+2],i+3,rxbuf[i+3]);			
//		}
#endif
	}

out_free:
	kfree(rxbuf);
	kfree(txbuf);
	return ret;
}

static void it8951_write_cmd_code(struct it8951_epd *epd, uint16_t cmd_code) {
	it8951_spi_transfer(epd, 0x6000, false, &cmd_code, NULL, 2);
}

static void it8951_write_data(struct it8951_epd *epd, uint16_t data) {
	it8951_spi_transfer(epd, 0x0000, false, &data, NULL, 2);
}

static void it8951_write_n_data(struct it8951_epd *epd, uint8_t *data, uint32_t len)
{
	it8951_spi_transfer(epd, 0x0000, false, data, NULL, len);
}

static uint16_t it8951_read_data(struct it8951_epd *epd) {
	uint16_t data = 0;
	it8951_spi_transfer(epd, 0x1000, true, NULL, &data, 2);
	return data;
}

static void it8951_read_n_data(struct it8951_epd *epd, uint8_t* buf, uint32_t len) {
	it8951_spi_transfer(epd, 0x1000, true, NULL, buf, len);
}

/* Power management */

void it8951_system_run(struct it8951_epd *epd)
{
	it8951_write_cmd_code(epd, IT8951_TCON_SYS_RUN);
}

void it8951_standby(struct it8951_epd *epd)
{
	it8951_write_cmd_code(epd, IT8951_TCON_STANDBY);
}

void it8951_sleep(struct it8951_epd *epd)
{
	it8951_write_cmd_code(epd, IT8951_TCON_SLEEP);
}

/* registers and commands */

static uint16_t it8951_read_reg(struct it8951_epd *epd, uint16_t reg_addr)
{
	uint16_t data;

	it8951_write_cmd_code(epd, IT8951_TCON_REG_RD);
	it8951_write_data(epd, reg_addr);
	data = it8951_read_data(epd);
	return data;
}

static void it8951_write_reg(struct it8951_epd *epd, uint16_t reg_addr, uint16_t value)
{
	it8951_write_cmd_code(epd, IT8951_TCON_REG_WR);
	it8951_write_data(epd, reg_addr);
	it8951_write_data(epd, value);
}

static void it8951_send_cmd_arg(struct it8951_epd *epd, uint16_t cmd_code, uint16_t* arg, uint16_t num_arg)
{
	uint16_t i;
	it8951_write_cmd_code(epd, cmd_code);
	for (i = 0; i < num_arg; i++)
	{
		it8951_write_data(epd, arg[i]);
	}
}

static void it8951_load_img_start(struct it8951_epd *epd, struct it8951_load_img_info* load_img_info)
{
	uint16_t arg;
	arg = (load_img_info->endian_type << 8 )
	      | (load_img_info->pixel_format << 4)
	      | (load_img_info->rotate);
	it8951_write_cmd_code(epd, IT8951_TCON_LD_IMG);
	it8951_write_data(epd, arg);
}

static void it8951_load_img_area_start(struct it8951_epd *epd, struct it8951_load_img_info* load_img_info, struct it8951_area_img_info* area_img_info)
{
	uint16_t arg[5];
	arg[0] = (load_img_info->endian_type << 8 )
	         | (load_img_info->pixel_format << 4)
	         | (load_img_info->rotate);
	arg[1] = area_img_info->x;
	arg[2] = area_img_info->y;
	arg[3] = area_img_info->width;
	arg[4] = area_img_info->height;
	it8951_send_cmd_arg(epd, IT8951_TCON_LD_IMG_AREA, arg, 5);
}

static void it8951_load_img_end(struct it8951_epd *epd)
{
	it8951_write_cmd_code(epd, IT8951_TCON_LD_IMG_END);
}

static void it8951_get_system_info(struct it8951_epd *epd)
{
	struct it8951_dev_info* dev_info = &epd->dev_info;

	memset(dev_info, 0, sizeof(struct it8951_dev_info));

	it8951_write_cmd_code(epd, USDEF_I80_CMD_GET_DEV_INFO);

	it8951_read_n_data(epd, (uint8_t *)dev_info, sizeof(struct it8951_dev_info));

	printk(KERN_INFO "it8951: panel %dx%d\n",
	       dev_info->panel_w, dev_info->panel_h );
	printk(KERN_INFO "it8951: FW version = %s\n", (uint8_t*)dev_info->fw_version);
	printk(KERN_INFO "it8951: LUT version = %s\n", (uint8_t*)dev_info->lut_version);
}

static void it8951_set_img_buf_base_addr(struct it8951_epd *epd, uint32_t base_addr)
{
	uint16_t h = (uint16_t)((base_addr >> 16) & 0x0000FFFF);
	uint16_t l = (uint16_t)( base_addr & 0x0000FFFF);
	it8951_write_reg(epd, LISAR + 2, h);
	it8951_write_reg(epd, LISAR, l);
}

static void it8951_wait_for_display_ready(struct it8951_epd *epd)
{
	//Check IT8951 Register LUTAFSR => NonZero Busy, 0 - Free
	while (it8951_read_reg(epd, LUTAFSR)) {
		//printk(KERN_INFO "it8951: wait_for_display_ready\n");
		usleep_range(1000, 2000);
	}
}

static void it8951_packed_pixel_write(struct it8951_epd *epd, struct it8951_load_img_info* load_img_info)
{
	uint32_t j = 0;
	//Source buffer address of Host
	uint8_t* frame_buf = (uint8_t*)load_img_info->start_fb_addr;

	//Set Image buffer(IT8951) Base address
	it8951_set_img_buf_base_addr(epd, load_img_info->img_buf_base_addr);
	//Send Load Image start Cmd
	it8951_load_img_start(epd, load_img_info);
	//Host Write Data
	for (j = 0; j < epd->dev_info.panel_h / 2; j++) // 4bits
	{
		it8951_write_n_data(epd, frame_buf, epd->dev_info.panel_w);
		frame_buf += epd->dev_info.panel_w;
	}

	//Send Load Img End Command
	it8951_load_img_end(epd);
}

#define MAX_SPI_TRANSFER 32768

static void it8951_packed_pixel_write_area(struct it8951_epd *epd, struct it8951_load_img_info* load_img_info, struct it8951_area_img_info* area_img_info)
{
	uint32_t j = 0;
	//Source buffer address of Host
	uint8_t* frame_buf = (uint8_t*)load_img_info->start_fb_addr;

	//Set Image buffer(IT8951) Base address
	it8951_set_img_buf_base_addr(epd, load_img_info->img_buf_base_addr);
	//Send Load Image start Cmd
	it8951_load_img_area_start(epd, load_img_info, area_img_info);
	//Host Write Data

	if (area_img_info->width * area_img_info->height / 2 < MAX_SPI_TRANSFER) {
		it8951_write_n_data(epd, frame_buf, area_img_info->width * area_img_info->height / 2);
	} else {
		for (j = 0; j < area_img_info->height / 2; j++) // 4bits
		{
			it8951_write_n_data(epd, frame_buf, area_img_info->width);
			frame_buf += area_img_info->width;
		}
	}

#if 0
	it8951_write_n_data(epd, frame_buf, area_img_info->height * area_img_info->width);
#endif

	//Send Load Img End Command
	it8951_load_img_end(epd);
}

static void it8951_display_area(struct it8951_epd *epd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t dpy_mode)
{
	it8951_write_cmd_code(epd, USDEF_I80_CMD_DPY_AREA);
	it8951_write_data(epd, x);
	it8951_write_data(epd, y);
	it8951_write_data(epd, w);
	it8951_write_data(epd, h);
	it8951_write_data(epd, dpy_mode);


	it8951_wait_for_ready(epd, w * h / 8); // wait longer for data
}

static inline uint8_t _it8951_rgb_to_4bits(uint32_t rgb) {
	u8 r = (rgb & 0x00ff0000) >> 16;
	u8 g = (rgb & 0x0000ff00) >> 8;
	u8 b =  rgb & 0x000000ff;

	/* ITU BT.601: Y = 0.299 R + 0.587 G + 0.114 B */
	return ((3 * r + 6 * g + b) / 10) >> 4;
}

static void it8951_memcpy_gray4(uint8_t *dst, uint8_t *src, int src_width, struct drm_clip_rect *clip) {
	int clip_w, clip_h, y;
	clip_w = clip->x2 - clip->x1;
	clip_h = clip->y2 - clip->y1;

	if (clip_w == src_width) { // optimize if same pitch
		memcpy(dst, src + (clip->y1 * clip_w + clip->x1) / 2, clip_w * clip_h / 2);
	} else {
		for (y = 0; y < clip_h; y++)
			memcpy(dst + (y * clip_w) / 2, src + ((y + clip->y1)*src_width + clip->x1) / 2, clip_w / 2);
	}
}

static void it8951_xrgb8888_to_gray4(uint8_t *dst_vaddr, void *src_vaddr, struct drm_framebuffer *fb,
                                     struct drm_clip_rect *clip)
{
	unsigned int x, y;
	u32 *src;
	u8 *dst;

	for (y = clip->y1; y < clip->y2; y++) {
		src = src_vaddr + (y * fb->pitches[0]);
		src += clip->x1;
		dst = dst_vaddr + (y * fb->width + clip->x1) / 2;

		for (x = clip->x1; x < clip->x2; x += 2) {
			*dst++ = _it8951_rgb_to_4bits(*src) + (_it8951_rgb_to_4bits(*(src + 1)) << 4);
			src += 2;
		}
	}
}

/* Auto waveform */
struct gray4_comparator {
	uint32_t from_grays[16];
	uint32_t to_grays[16];

	uint32_t x1;
	uint32_t y1;
	uint32_t x2;
	uint32_t y2;

	uint32_t *transitions;
};

// compare grays in bufs
static int gray4_compare(uint8_t *from_img, uint8_t *to_img, uint32_t w, uint32_t h, struct gray4_comparator *comp) {
	int diff = 0;
	int x, y, i;

	comp->x1 = w;
	comp->y1 = h;
	comp->x2 = 0;
	comp->y2 = 0;

	for (y = 0; y < h; y++) {
		for (x = 0; x < w; x += 2) {
			i = (y * w + x) / 2;
			// 4bits x
			if ((from_img[i] >> 4) != (to_img[i] >> 4)) {
				comp->transitions[(from_img[i] >> 4) * 16 + (to_img[i] >> 4)]++;
				diff++;
				if (x < comp->x1)
					comp->x1 = x;
				if (x > comp->x2)
					comp->x2 = x;
				if (y < comp->y1)
					comp->y1 = y;
				if (y > comp->y2)
					comp->y2 = y;
			}
			comp->from_grays[from_img[i] >> 4]++;
			comp->to_grays[to_img[i] >> 4]++;

			// 4bits x+1
			if ((from_img[i] & 0xf) != (to_img[i] & 0xf)) {
				comp->transitions[(from_img[i] & 0xf) * 16 + (to_img[i] & 0xf)]++;
				diff++;
				if (x + 1 < comp->x1)
					comp->x1 = x + 1;
				if (x + 1 > comp->x2)
					comp->x2 = x + 1;
				if (y < comp->y1)
					comp->y1 = y;
				if (y > comp->y2)
					comp->y2 = y;
			}
			comp->from_grays[from_img[i] & 0xf]++;
			comp->to_grays[to_img[i] & 0xf]++;

		}
	}
	comp->x2++;
	comp->y2++;
	return diff;
}

// diff == 0 for any to black or white
static int wf_du_match(struct gray4_comparator *comp) {
	int diff = 0;
	int f, t;
	for (f = 0; f < 16; f++) {
		for (t = 1; t < 15; t++) { // except 0x0 black and 0xf white
			diff += comp->transitions[f * 16 + t];
		}
	}
	return diff;
}

static int gray4_auto_wf(uint8_t *from_img, uint8_t *to_img, uint32_t w, uint32_t h, struct gray4_comparator *comp) {
	int diff, du_diff, len;

	memset(comp, 0, sizeof(struct gray4_comparator));
	comp->transitions = kzalloc(256 * sizeof(uint32_t), GFP_KERNEL);

	diff = gray4_compare(from_img, to_img, w, h, comp);

	du_diff = wf_du_match(comp);

	kfree(comp->transitions);
	comp->transitions = NULL;

	len = w * h / 2;

#ifdef IT8951_DEBUG
	printk(KERN_INFO "it8951: auto waveform len:%d diff:%d du_diff:%d x1:%d,y1:%d,x2:%d,y2:%d w:%d,h:%d\n", len, diff, du_diff, comp->x1, comp->y1, comp->x2, comp->y2, w, h);
#endif
	if (diff > 0) {
		if (du_diff == 0 ) { // DU for any gray to black or white
			return IT8951_MODE_DU;
		} else {
			if (comp->from_grays[0xf] > len / 2 && comp->to_grays[0xf] > len / 2) // GL16 for > 50% white pixel
			{
				if (diff < len / 32) { // reduced flashing GLD16 for < 3% diff
					return IT8951_MODE_GLD16;
				} else {
					return IT8951_MODE_GL16;
				}
			} else {
				return IT8951_MODE_GC16;
			}
		}
	} else {
		return -1;
	}
}


/* Linux driver part */

static struct fb_deferred_io it8951_defio = {
	.delay = HZ / 4, // 4 fps at best
	.deferred_io = drm_fb_helper_deferred_io,
};

static inline struct it8951_epd *
epd_from_tinydrm(struct tinydrm_device *tdev)
{
	return container_of(tdev, struct it8951_epd, tinydrm);
}

DEFINE_DRM_GEM_CMA_FOPS(it8951_fops);

static struct drm_driver it8951_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME |
	DRIVER_ATOMIC,
	.fops			= &it8951_fops,
	TINYDRM_GEM_DRIVER_OPS,
	.name			= "it8951",
	.desc			= "it8951 e-ink",
	.date			= "20190913",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id it8951_of_match[] = {
	{ .compatible = "ite,it8951" },
	{},
};
MODULE_DEVICE_TABLE(of, it8951_of_match);

static const struct spi_device_id it8951_id[] = {
	{ "it8951", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, it8951_id);

#define CLIP_PADDING 4

static int it8951_fb_dirty(struct drm_framebuffer *fb,
                           struct drm_file *file_priv,
                           unsigned int flags, unsigned int color,
                           struct drm_clip_rect *clips,
                           unsigned int num_clips)
{

	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	struct dma_buf_attachment *import_attach = cma_obj->base.import_attach;
	struct tinydrm_device *tdev = fb->dev->dev_private;
	struct it8951_epd *epd = epd_from_tinydrm(tdev);
	struct drm_clip_rect clip;
	int clip_w, clip_h, ret, wf;
	bool full_screen, full_refresh;
	struct drm_gem_cma_object *tmp_bo = NULL, *tmp_prev_bo = NULL;
	u8 *buf = NULL;

	struct it8951_load_img_info load_img_info;

	if(epd->dev_info.panel_w == 0 || epd->dev_info.panel_h == 0) {
		//printk(KERN_INFO "it8951: initialization failed\n");
		return 0;
	}

	if (!epd->enabled) {
		printk(KERN_INFO "it8951: not enabled yet\n");
		return 0;
	}

	// create local 4bits buffer
	if (!epd->buf) {
		epd->bo = drm_gem_cma_create(fb->dev, epd->dev_info.panel_w * epd->dev_info.panel_h / 2);
		epd->buf = epd->bo->vaddr;
		//epd->buf = kmalloc(epd->dev_info.panel_w * epd->dev_info.panel_h / 2, GFP_KERNEL);
		memset(epd->buf, 0xf, epd->dev_info.panel_w * epd->dev_info.panel_h / 2); // set pixels to white
	}

	// full refresh for ghosting
	if (epd->update_mode == -1 && epd->last_full_refresh == 32) {
		clip.x1 = 0;
		clip.x2 = epd->dev_info.panel_w;
		clip.y1 = 0;
		clip.y2 = epd->dev_info.panel_h;

		full_screen = true;
		full_refresh = true;
	} else {
		full_screen = tinydrm_merge_clips(&clip, clips, num_clips, flags,
		                                  epd->dev_info.panel_w, epd->dev_info.panel_h);
		full_refresh = false;

		// CLIP_PADDING bytes padding
		if (clip.x1 % CLIP_PADDING != 0)
			clip.x1 -= clip.x1 % CLIP_PADDING;

		if (clip.y1 % CLIP_PADDING != 0)
			clip.y1 -= clip.y1 % CLIP_PADDING;

		if (clip.x2 % CLIP_PADDING != 0)
			clip.x2 += CLIP_PADDING - (clip.x2 % CLIP_PADDING);

		if (clip.y2 % CLIP_PADDING != 0)
			clip.y2 += CLIP_PADDING - (clip.y2 % CLIP_PADDING);

		if (clip.x2 > epd->dev_info.panel_w)
			clip.x2 = epd->dev_info.panel_w;

		if (clip.y2 > epd->dev_info.panel_h)
			clip.y2 = epd->dev_info.panel_h;
	}

	clip_w = clip.x2 - clip.x1;
	clip_h = clip.y2 - clip.y1;

#ifdef IT8951_DEBUG
	printk(KERN_INFO "it8951: dirty panel:%dx%d flags:%d color:%d pitch:%d clips:%d, full_screen:%d, x1:%d, y1:%d, x2:%d, y2:%d\n",
	       epd->dev_info.panel_w, epd->dev_info.panel_h, flags, color, fb->pitches[0], num_clips, full_screen, clip.x1, clip.y1, clip.x2, clip.y2);
#endif

	// create tmp buffer
	tmp_prev_bo = drm_gem_cma_create(fb->dev, clip_w * clip_h / 2);

	// copy previous buffer
	if (!full_refresh && epd->update_mode == -1)
		it8951_memcpy_gray4(tmp_prev_bo->vaddr, epd->buf, epd->dev_info.panel_w, &clip);

	if (import_attach) {
		ret = dma_buf_begin_cpu_access(import_attach->dmabuf,
		                               DMA_FROM_DEVICE);
		if (ret)
			goto out_free;
	}

	it8951_xrgb8888_to_gray4(epd->buf, cma_obj->vaddr, fb, &clip);

	if (import_attach) {
		ret = dma_buf_end_cpu_access(import_attach->dmabuf,
		                             DMA_FROM_DEVICE);
		if (ret)
			goto out_free;
	}

	if (full_screen || full_refresh) {
		buf = epd->buf;
	} else {
		// create tmp buffer
		tmp_bo = drm_gem_cma_create(fb->dev, clip_w * clip_h / 2);
		it8951_memcpy_gray4(tmp_bo->vaddr, epd->buf, epd->dev_info.panel_w, &clip);
		buf = tmp_bo->vaddr;
	}

	if (full_refresh) {
		wf = IT8951_MODE_GC16;
	} else {
		if (epd->update_mode == -1) {
			struct gray4_comparator comp;
			wf = gray4_auto_wf(tmp_prev_bo->vaddr, buf, clip_w, clip_h, &comp);
#ifdef IT8951_DEBUG
			printk(KERN_INFO "it8951: auto waveform use %d\n", wf);
#endif
		} else {
			wf = epd->update_mode;
		}
	}

	load_img_info.start_fb_addr    = (uint32_t)buf;
	load_img_info.endian_type     = IT8951_LDIMG_L_ENDIAN;
	load_img_info.pixel_format =    IT8951_4BPP;
	load_img_info.rotate         = IT8951_ROTATE_0;
	load_img_info.img_buf_base_addr = epd->img_buf_addr;

	if (wf != -1) {
		if (!epd->running)
			it8951_system_run(epd);
		epd->running = true;

		if (full_screen) {
			it8951_wait_for_display_ready(epd);
			it8951_packed_pixel_write(epd, &load_img_info);
		} else {
			struct it8951_area_img_info area_img_info;
			//Set Load Area
			area_img_info.x      = clip.x1;
			area_img_info.y      = clip.y1;
			area_img_info.width  = clip.x2 - clip.x1;
			area_img_info.height = clip.y2 - clip.y1;

			it8951_wait_for_display_ready(epd);
			it8951_packed_pixel_write_area(epd, &load_img_info, &area_img_info);
		}

		it8951_display_area(epd, clip.x1, clip.y1, clip_w, clip_h, wf);

		if (epd->running)
			it8951_standby(epd);
		epd->running = false;

		if (epd->update_mode == -1) {
			if (wf == IT8951_MODE_GC16 && full_screen)
				epd->last_full_refresh = 0;
			else
				epd->last_full_refresh++;
		}
	} else {
		// no refresh needed
	}

out_free:
	if (tmp_bo)
		drm_gem_cma_free_object((struct drm_gem_object*)tmp_bo);
	if (tmp_prev_bo)
		drm_gem_cma_free_object((struct drm_gem_object*)tmp_prev_bo);

	return ret;
}

static const struct drm_framebuffer_funcs it8951_fb_funcs = {
	.destroy	= drm_gem_fb_destroy,
	.create_handle	= drm_gem_fb_create_handle,
	.dirty = tinydrm_fb_dirty,
};

static void it8951_pipe_enable(struct drm_simple_display_pipe *pipe,
                               struct drm_crtc_state *crtc_state,
                               struct drm_plane_state *plane_state)
{
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct it8951_epd *epd = epd_from_tinydrm(tdev);
	//struct drm_framebuffer *fb = pipe->plane.fb;

	printk(KERN_INFO "it8951: pipe enable\n");

	gpiod_set_value_cansleep(epd->reset, 0);
	msleep(100);
	gpiod_set_value_cansleep(epd->reset, 1);

	msleep(100);

	it8951_get_system_info(epd);

	epd->img_buf_addr = epd->dev_info.img_buf_addr_l | (epd->dev_info.img_buf_addr_h << 16);

	//Set to Enable I80 Packed mode
	it8951_write_reg(epd, I80CPCR, 0x0001);

	epd->enabled = true;
	epd->last_full_refresh = 0;

	epd->bo = NULL;
	epd->buf = NULL;

	it8951_standby(epd);
	epd->running = false;
}

static void it8951_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct it8951_epd *epd = epd_from_tinydrm(tdev);

	printk(KERN_INFO "it8951: pipe disable\n");

	mutex_lock(&tdev->dirty_lock);
	epd->enabled = false;
	mutex_unlock(&tdev->dirty_lock);

	if (epd->bo)
		drm_gem_cma_free_object((struct drm_gem_object*)epd->bo);
}

static const struct drm_simple_display_pipe_funcs it8951_pipe_funcs = {
	.enable = it8951_pipe_enable,
	.disable = it8951_pipe_disable,
	.update = tinydrm_display_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const uint32_t it8951_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const struct drm_display_mode it8951_mode = {
	TINYDRM_MODE(800, 600, 0, 0),
};


static ssize_t it8951_update_mode_show(struct device *device,
                                       struct device_attribute *attr,
                                       char *buf)
{
	struct it8951_epd *epd = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", (epd->update_mode));
}

static ssize_t it8951_update_mode_store(struct device *device,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
	struct it8951_epd *epd = dev_get_drvdata(device);

	sscanf(buf, "%du", &epd->update_mode);
	return count;
}

static struct device_attribute it8951_update_mode_attr = __ATTR(update_mode, S_IRUGO | S_IWUSR, it8951_update_mode_show, it8951_update_mode_store);


static int it8951_probe(struct spi_device *spi)
{
	const struct drm_display_mode *mode;
	const struct of_device_id *match;
	struct device *dev = &spi->dev;
	struct tinydrm_device *tdev;
	struct it8951_epd *epd;
	struct fb_info *info;

	int ret;

	match = of_match_device(it8951_of_match, dev);

	/* The SPI device is used to allocate dma memory */
	if (!dev->coherent_dma_mask) {
		ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_warn(dev, "Failed to set dma mask %d\n", ret);
			return ret;
		}
	}

	epd = devm_kzalloc(dev, sizeof(*epd), GFP_KERNEL);
	if (!epd)
		return -ENOMEM;

	epd->spi = spi;

	epd->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(epd->reset)) {
		ret = PTR_ERR(epd->reset);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
		return ret;
	}

	epd->hrdy = devm_gpiod_get(dev, "hrdy", GPIOD_IN);
	if (IS_ERR(epd->hrdy)) {
		ret = PTR_ERR(epd->hrdy);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "Failed to get gpio 'hrdy'\n");
		return ret;
	}

	epd->little_endian = tinydrm_machine_little_endian();
	epd->update_mode = -1; // auto

	tdev = &epd->tinydrm;

	ret = devm_tinydrm_init(dev, tdev, &it8951_fb_funcs, &it8951_driver);
	if (ret)
		return ret;

	mode = &it8951_mode;

	tdev->fb_dirty = it8951_fb_dirty;

	ret = tinydrm_display_pipe_init(tdev, &it8951_pipe_funcs,
	                                DRM_MODE_CONNECTOR_VIRTUAL,
	                                it8951_formats,
	                                ARRAY_SIZE(it8951_formats), mode, 0);
	if (ret)
		return ret;

	drm_mode_config_reset(tdev->drm);

	spi_set_drvdata(spi, epd);

	printk(KERN_INFO "it8951: LE %d\n", epd->little_endian);
	printk(KERN_INFO "it8951: SPI speed: %uMHz\n", spi->max_speed_hz / 1000000);

	ret = devm_tinydrm_register(tdev);
	if (ret)
		return ret;

	// set deferedio
	info = tdev->drm->fb_helper->fbdev;
	info->fbdefio = &it8951_defio;
	fb_deferred_io_init(info);

	ret = device_create_file(&spi->dev, &it8951_update_mode_attr);
	if (ret) {
		dev_err(dev, "Failed to create sysfs update_mode\n");
	}

	return ret;
}

static int it8951_remove(struct spi_device *spi)
{
	device_remove_file(&spi->dev, &it8951_update_mode_attr);

	printk(KERN_INFO "it8951: removed\n");
	return 0;
}

static void it8951_shutdown(struct spi_device *spi)
{
	struct it8951_epd *epd = spi_get_drvdata(spi);
	struct tinydrm_device *tdev = &epd->tinydrm;

	tinydrm_shutdown(tdev);

	printk(KERN_INFO "it8951: shutted down\n");
}

static struct spi_driver it8951_spi_driver = {
	.driver = {
		.name = "it8951",
		.owner = THIS_MODULE,
		.of_match_table = it8951_of_match,
	},
	.id_table = it8951_id,
	.probe = it8951_probe,
	.remove = it8951_remove,
	.shutdown = it8951_shutdown,
};
module_spi_driver(it8951_spi_driver);

MODULE_DESCRIPTION("DRM driver for the it8951 based Waveshare eInk panels");
MODULE_AUTHOR("Julien Boulnois");
MODULE_LICENSE("GPL");
