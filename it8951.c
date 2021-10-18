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

#define WIP_FIX_REFRESH
//#define IT8951_DEBUG

//Built in I80 Command Code
#define IT8951_TCON_SYS_RUN 0x0001
#define IT8951_TCON_STANDBY 0x0002
#define IT8951_TCON_SLEEP 0x0003
#define IT8951_TCON_REG_RD 0x0010
#define IT8951_TCON_REG_WR 0x0011
#define IT8951_TCON_MEM_BST_RD_T 0x0012
#define IT8951_TCON_MEM_BST_RD_S 0x0013
#define IT8951_TCON_MEM_BST_WR 0x0014
#define IT8951_TCON_MEM_BST_END 0x0015
#define IT8951_TCON_LD_IMG 0x0020
#define IT8951_TCON_LD_IMG_AREA 0x0021
#define IT8951_TCON_LD_IMG_END 0x0022

//I80 User defined command code
#define USDEF_I80_CMD_DPY_AREA 0x0034
#define USDEF_I80_CMD_DPY_BUF_AREA 0x0037
#define USDEF_I80_CMD_PWR_SEQ 0x0038
#define USDEF_I80_CMD_VCOM 0x0039

#define USDEF_I80_CMD_GET_DEV_INFO 0x0302

//Rotate mode
#define IT8951_ROTATE_0 0
#define IT8951_ROTATE_90 1
#define IT8951_ROTATE_180 2
#define IT8951_ROTATE_270 3

//Pixel mode , BPP - Bit per Pixel
#define IT8951_2BPP 0
#define IT8951_3BPP 1
#define IT8951_4BPP 2
#define IT8951_8BPP 3

//Waveform Mode
#define IT8951_MODE_0 0
#define IT8951_MODE_1 1
#define IT8951_MODE_2 2
#define IT8951_MODE_3 3
#define IT8951_MODE_4 4

#define IT8951_MODE_INIT 0
#define IT8951_MODE_DU 1
#define IT8951_MODE_GC16 2
#define IT8951_MODE_GL16 3
#define IT8951_MODE_GLR16 4
#define IT8951_MODE_GLD16 5
// the following mode does not seems to work ...
#define IT8951_MODE_A2 6
#define IT8951_MODE_DU4 7

//Endian Type
#define IT8951_LDIMG_L_ENDIAN 0
#define IT8951_LDIMG_B_ENDIAN 1
//Auto LUT
#define IT8951_DIS_AUTO_LUT 0
#define IT8951_EN_AUTO_LUT 1
//LUT Engine Status
#define IT8951_ALL_LUTE_BUSY 0xFFFF

//-----------------------------------------------------------------------
// IT8951 TCon Registers defines
//-----------------------------------------------------------------------
//Register Base Address
#define DISPLAY_REG_BASE 0x1000 //Register RW access for I80 only
//Base Address of Basic LUT Registers
#define LUT0EWHR (DISPLAY_REG_BASE + 0x00)	//LUT0 Engine Width Height Reg
#define LUT0XYR (DISPLAY_REG_BASE + 0x40)	//LUT0 XY Reg
#define LUT0BADDR (DISPLAY_REG_BASE + 0x80) //LUT0 Base Address Reg
#define LUT0MFN (DISPLAY_REG_BASE + 0xC0)	//LUT0 Mode and Frame number Reg
#define LUT01AF (DISPLAY_REG_BASE + 0x114)	//LUT0 and LUT1 Active Flag Reg
//Update Parameter Setting Register
#define UP0SR (DISPLAY_REG_BASE + 0x134) //Update Parameter0 Setting Reg

#define UP1SR (DISPLAY_REG_BASE + 0x138)	 //Update Parameter1 Setting Reg
#define LUT0ABFRV (DISPLAY_REG_BASE + 0x13C) //LUT0 Alpha blend and Fill rectangle Value
#define UPBBADDR (DISPLAY_REG_BASE + 0x17C)	 //Update Buffer Base Address
#define LUT0IMXY (DISPLAY_REG_BASE + 0x180)	 //LUT0 Image buffer X/Y offset Reg
#define LUTAFSR (DISPLAY_REG_BASE + 0x224)	 //LUT Status Reg (status of All LUT Engines)

#define BGVR (DISPLAY_REG_BASE + 0x250) //Bitmap (1bpp) image color table

//-------System Registers----------------
#define SYS_REG_BASE 0x0000

//Address of System Registers
#define I80CPCR (SYS_REG_BASE + 0x04)
//-------Memory Converter Registers----------------
#define MCSR_BASE_ADDR 0x0200
#define MCSR (MCSR_BASE_ADDR + 0x0000)
#define LISAR (MCSR_BASE_ADDR + 0x0008)

struct it8951_load_img_info
{
	uint16_t endian_type;		//little or Big Endian
	uint16_t pixel_format;		//bpp
	uint16_t rotate;			//Rotate mode
	uint32_t start_fb_addr;		//Start address of source Frame buffer
	uint32_t img_buf_base_addr; //Base address of target image buffer
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
	uint16_t fw_version[8];	 //16 Bytes String
	uint16_t lut_version[8]; //16 Bytes String
};

struct it8951_epd
{
	struct tinydrm_device tinydrm;
	struct spi_device *spi;

	struct gpio_desc *reset;
	struct gpio_desc *hrdy;

	struct it8951_dev_info dev_info;

	uint32_t w;
	uint32_t h;

	uint32_t img_buf_addr;

	uint32_t last_full_refresh;

	int update_mode;
	int ghosting_refresh;

	bool enabled;
	bool running;

	uint32_t rotation;

	uint8_t *buf;
};

static int vcom = 0;
module_param(vcom, int, 0660);
MODULE_PARM_DESC(vcom, "panel VCOM voltage in mV");

static int spi_freq = 12000000; // can't get it works at > 12Mhz at least on RPI
module_param(spi_freq, int, 0660);
MODULE_PARM_DESC(spi_freq, "SPI frequency in Hz, default 12000000");

static int update_mode = -1; // auto
module_param(update_mode, int, 0660);
MODULE_PARM_DESC(update_mode, "Waveform update mode, default -1 (auto)");

static int ghosting_refresh = 32; // auto
module_param(ghosting_refresh, int, 0660);
MODULE_PARM_DESC(ghosting_refresh, "Full refresh frequency for ghosting, default every 32 refresh");

static void it8951_spi_wait_for_ready(struct it8951_epd *epd, int us)
{
	int waited = 0;
	while (!gpiod_get_value_cansleep(epd->hrdy) && waited < 1000000)
	{
		usleep_range(us, us * 2);
		waited += us;
	}
#ifdef IT8951_DEBUG
//	printk(KERN_INFO "it8951: wait_for_ready %d\n", waited);
#endif
}

/* SPI data transfer */

static inline void it8951_spi_memcpy_swab16(struct it8951_epd *epd, u16 *dst, u16 *src, size_t len)
{
#if defined(__LITTLE_ENDIAN)
	int i;
	for (i = 0; i < len; i++)
	{
		*dst++ = swab16(*src++);
	}
#else
	memcpy(dst, src, len);
#endif
}

static inline u16 it8951_spi_swab16(struct it8951_epd *epd, u16 data)
{
#if defined(__LITTLE_ENDIAN)
	return swab16(data);
#else
	return data;
#endif
}

static int it8951_spi_transfer(struct it8951_epd *epd, uint16_t preamble, bool dummy, const void *tx, void *rx, uint32_t len)
{
	int speed_hz = spi_freq;
	int ret;
	u8 *txbuf = NULL, *rxbuf = NULL;
	uint16_t spreamble = it8951_spi_swab16(epd, preamble);

#ifdef IT8951_DEBUG
//	if (tx)
//		printk(KERN_INFO "it8951: it8951_spi_transfer preamble:%x len:%d tx:%x\n", preamble, len, ((uint16_t *)tx)[0]);
//	else
//		printk(KERN_INFO "it8951: it8951_spi_transfer preamble:%x len:%d\n", preamble, len);
#endif

	if (tx)
	{
		txbuf = kmalloc(len, GFP_KERNEL);
		if (!txbuf)
		{
			ret = -ENOMEM;
			goto out_free;
		}
		it8951_spi_memcpy_swab16(epd, (uint16_t *)txbuf, (uint16_t *)tx, len / 2);
	}

	if (rx)
	{
		rxbuf = kmalloc(len, GFP_KERNEL);
		if (!rxbuf)
		{
			ret = -ENOMEM;
			goto out_free;
		}
	}

	it8951_spi_wait_for_ready(epd, 100);

	if (dummy)
	{
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
	}
	else
	{
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

	if (rx && !ret)
	{
		it8951_spi_memcpy_swab16(epd, (uint16_t *)rx, (uint16_t *)rxbuf, len / 2);

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

static void it8951_spi_write_cmd_code(struct it8951_epd *epd, uint16_t cmd_code)
{
	it8951_spi_transfer(epd, 0x6000, false, &cmd_code, NULL, 2);
}

static void it8951_spi_write_data(struct it8951_epd *epd, uint16_t data)
{
	it8951_spi_transfer(epd, 0x0000, false, &data, NULL, 2);
}

static void it8951_spi_write_n_data(struct it8951_epd *epd, uint8_t *data, uint32_t len)
{
	it8951_spi_transfer(epd, 0x0000, false, data, NULL, len);
}

static uint16_t it8951_spi_read_data(struct it8951_epd *epd)
{
	uint16_t data = 0;
	it8951_spi_transfer(epd, 0x1000, true, NULL, &data, 2);
	return data;
}

static void it8951_spi_read_n_data(struct it8951_epd *epd, uint8_t *buf, uint32_t len)
{
	it8951_spi_transfer(epd, 0x1000, true, NULL, buf, len);
}

/* Power management */

static void it8951_spi_system_run(struct it8951_epd *epd)
{
	it8951_spi_write_cmd_code(epd, IT8951_TCON_SYS_RUN);
}

static void it8951_spi_standby(struct it8951_epd *epd)
{
	it8951_spi_write_cmd_code(epd, IT8951_TCON_STANDBY);
}

static void it8951_spi_sleep(struct it8951_epd *epd)
{
	it8951_spi_write_cmd_code(epd, IT8951_TCON_SLEEP);
}

/* registers and commands */

static uint16_t it8951_spi_read_reg(struct it8951_epd *epd, uint16_t reg_addr)
{
	uint16_t data;

	it8951_spi_write_cmd_code(epd, IT8951_TCON_REG_RD);
	it8951_spi_write_data(epd, reg_addr);
	data = it8951_spi_read_data(epd);
	return data;
}

static void it8951_spi_write_reg(struct it8951_epd *epd, uint16_t reg_addr, uint16_t value)
{
	it8951_spi_write_cmd_code(epd, IT8951_TCON_REG_WR);
	it8951_spi_write_data(epd, reg_addr);
	it8951_spi_write_data(epd, value);
}

static void it8951_spi_send_cmd_arg(struct it8951_epd *epd, uint16_t cmd_code, uint16_t *arg, uint16_t num_arg)
{
	uint16_t i;
	it8951_spi_write_cmd_code(epd, cmd_code);
	for (i = 0; i < num_arg; i++)
	{
		it8951_spi_write_data(epd, arg[i]);
	}
}

static void it8951_spi_load_img_start(struct it8951_epd *epd, struct it8951_load_img_info *load_img_info)
{
	uint16_t arg;
	arg = (load_img_info->endian_type << 8) | (load_img_info->pixel_format << 4) | (load_img_info->rotate);
	it8951_spi_write_cmd_code(epd, IT8951_TCON_LD_IMG);
	it8951_spi_write_data(epd, arg);
}

static void it8951_spi_load_img_area_start(struct it8951_epd *epd, struct it8951_load_img_info *load_img_info, struct it8951_area_img_info *area_img_info)
{
	uint16_t arg[5];
	arg[0] = (load_img_info->endian_type << 8) | (load_img_info->pixel_format << 4) | (load_img_info->rotate);
	arg[1] = area_img_info->x;
	arg[2] = area_img_info->y;
	arg[3] = area_img_info->width;
	arg[4] = area_img_info->height;
	it8951_spi_send_cmd_arg(epd, IT8951_TCON_LD_IMG_AREA, arg, 5);
}

static void it8951_spi_load_img_end(struct it8951_epd *epd)
{
	it8951_spi_write_cmd_code(epd, IT8951_TCON_LD_IMG_END);
}

static void it8951_spi_get_system_info(struct it8951_epd *epd)
{
	struct it8951_dev_info *dev_info = &epd->dev_info;

	memset(dev_info, 0, sizeof(struct it8951_dev_info));

	it8951_spi_write_cmd_code(epd, USDEF_I80_CMD_GET_DEV_INFO);

	it8951_spi_read_n_data(epd, (uint8_t *)dev_info, sizeof(struct it8951_dev_info));

	printk(KERN_INFO "it8951: panel %dx%d\n",
		   dev_info->panel_w, dev_info->panel_h);
	printk(KERN_INFO "it8951: FW version = %s\n", (uint8_t *)dev_info->fw_version);
	printk(KERN_INFO "it8951: LUT version = %s\n", (uint8_t *)dev_info->lut_version);
}

static void it8951_spi_set_img_buf_base_addr(struct it8951_epd *epd, uint32_t base_addr)
{
	uint16_t h = (uint16_t)((base_addr >> 16) & 0x0000FFFF);
	uint16_t l = (uint16_t)(base_addr & 0x0000FFFF);
	it8951_spi_write_reg(epd, LISAR + 2, h);
	it8951_spi_write_reg(epd, LISAR, l);
}

static void it8951_spi_wait_for_display_ready(struct it8951_epd *epd)
{
	//Check IT8951 Register LUTAFSR => NonZero Busy, 0 - Free
	while (it8951_spi_read_reg(epd, LUTAFSR))
	{
		//printk(KERN_INFO "it8951: wait_for_display_ready\n");
		usleep_range(1000, 2000);
	}
}

static void it8951_spi_packed_pixel_write(struct it8951_epd *epd, struct it8951_load_img_info *load_img_info)
{
	uint32_t j = 0;
	//Source buffer address of Host
	uint8_t *frame_buf = (uint8_t *)load_img_info->start_fb_addr;

	//Set Image buffer(IT8951) Base address
	it8951_spi_set_img_buf_base_addr(epd, load_img_info->img_buf_base_addr);
	//Send Load Image start Cmd
	it8951_spi_load_img_start(epd, load_img_info);
	//Host Write Data
	for (j = 0; j < epd->dev_info.panel_h / 2; j++) // 4bits
	{
		it8951_spi_write_n_data(epd, frame_buf, epd->dev_info.panel_w);
		frame_buf += epd->dev_info.panel_w;
	}

	//Send Load Img End Command
	it8951_spi_load_img_end(epd);
}

#define MAX_SPI_TRANSFER 32768

static void it8951_spi_packed_pixel_write_area(struct it8951_epd *epd, struct it8951_load_img_info *load_img_info, struct it8951_area_img_info *area_img_info)
{
	uint32_t j = 0;
	//Source buffer address of Host
	uint8_t *frame_buf = (uint8_t *)load_img_info->start_fb_addr;

	//Set Image buffer(IT8951) Base address
	it8951_spi_set_img_buf_base_addr(epd, load_img_info->img_buf_base_addr);
	//Send Load Image start Cmd
	it8951_spi_load_img_area_start(epd, load_img_info, area_img_info);
	//Host Write Data

	if (area_img_info->width * area_img_info->height / 2 < MAX_SPI_TRANSFER)
	{
		it8951_spi_write_n_data(epd, frame_buf, area_img_info->width * area_img_info->height / 2);
	}
	else
	{
		for (j = 0; j < area_img_info->height / 2; j++) // 4bits
		{
			it8951_spi_write_n_data(epd, frame_buf, area_img_info->width);
			frame_buf += area_img_info->width;
		}
	}

	//Send Load Img End Command
	it8951_spi_load_img_end(epd);
}

static void it8951_spi_display_area(struct it8951_epd *epd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t dpy_mode, uint8_t rotate)
{
	uint16_t rot_x, rot_y, rot_w, rot_h;

	switch (rotate)
	{
	case IT8951_ROTATE_90:
		rot_x = y;
		rot_y = epd->w - (x + w);
		rot_w = h;
		rot_h = w;
		break;
	case IT8951_ROTATE_180:
		rot_x = epd->w - (x + w);
		rot_y = epd->h - (y + h);
		rot_w = w;
		rot_h = h;
		break;
	case IT8951_ROTATE_270:
		rot_x = epd->h - (y + h);
		rot_y = x;
		rot_w = h;
		rot_h = w;
		break;
	default:
		rot_x = x;
		rot_y = y;
		rot_w = w;
		rot_h = h;
		break;
	}

	it8951_spi_write_cmd_code(epd, USDEF_I80_CMD_DPY_AREA);
	it8951_spi_write_data(epd, rot_x);
	it8951_spi_write_data(epd, rot_y);
	it8951_spi_write_data(epd, rot_w);
	it8951_spi_write_data(epd, rot_h);
	it8951_spi_write_data(epd, dpy_mode);

	it8951_spi_wait_for_ready(epd, w * h / 8); // wait longer for data
}

/* 4-bits gray data processing */

static void it8951_gray4_memcpy(uint8_t *dst, uint8_t *src, int src_width, struct drm_clip_rect *clip)
{
	int clip_w, clip_h, y;
	clip_w = clip->x2 - clip->x1;
	clip_h = clip->y2 - clip->y1;

	if (clip_w == src_width)
	{ // optimize if same pitch
		memcpy(dst, src + (clip->y1 * clip_w + clip->x1) / 2, clip_w * clip_h / 2);
	}
	else
	{
		for (y = 0; y < clip_h; y++)
			memcpy(dst + (y * clip_w) / 2, src + ((y + clip->y1) * src_width + clip->x1) / 2, clip_w / 2);
	}
}

static inline uint8_t it8951_rgba_to_gray4_pixel(uint32_t rgba)
{
	u32 y;
	u8 r = (rgba & 0x00ff0000) >> 16;
	u8 g = (rgba & 0x0000ff00) >> 8;
	u8 b = rgba & 0x000000ff;

	/* ITU BT.601: Y = 0.299 R + 0.587 G + 0.114 B */
	//return ((3 * r + 6 * g + b) / 10) >> 4;
	y = (r * 77) + (g * 151) + (b * 28);
	return y >> 12;
}

// from tinydrm-helpers.c
static void it8951_xrgb8888_to_gray4(uint8_t *dst_vaddr, void *src_vaddr, struct drm_framebuffer *fb,
									 struct drm_clip_rect *clip)
{
	unsigned int len = (clip->x2 - clip->x1) * sizeof(u32);
	unsigned int x, y;
	void *buf;
	u32 *src;
	u8 *dst;

	/*
	 * The cma memory is write-combined so reads are uncached.
	 * Speed up by fetching one line at a time.
	 */
	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return;

	for (y = clip->y1; y < clip->y2; y++)
	{
		src = src_vaddr + (y * fb->pitches[0]);
		src += clip->x1;
		memcpy(buf, src, len);
		src = buf;
		dst = dst_vaddr + (y * fb->width + clip->x1) / 2;

		for (x = clip->x1; x < clip->x2; x += 2)
		{
			*dst++ = it8951_rgba_to_gray4_pixel(*src) + (it8951_rgba_to_gray4_pixel(*(src + 1)) << 4);
			src += 2;
		}
	}
	kfree(buf);
}

/* Auto waveform */
struct it8951_gray4_comparator
{
	uint32_t from_grays[16];
	uint32_t to_grays[16];

	uint32_t x1;
	uint32_t y1;
	uint32_t x2;
	uint32_t y2;

	// transitions count (from_pix * 16 + to_pix)
	uint32_t *transitions;
};

// compare grays in bufs and keep some stats
static int it8951_gray4_compare(uint8_t *from_img, uint8_t *to_img, uint32_t w, uint32_t h, struct it8951_gray4_comparator *comp)
{
	int diff = 0;
	int x, y, i;

	comp->x1 = w;
	comp->y1 = h;
	comp->x2 = 0;
	comp->y2 = 0;

	for (y = 0; y < h; y++)
	{
		for (x = 0; x < w; x += 2)
		{
			i = (y * w + x) / 2;
			// 4bits x
			if ((from_img[i] >> 4) != (to_img[i] >> 4))
			{
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
			if ((from_img[i] & 0xf) != (to_img[i] & 0xf))
			{
				comp->transitions[(from_img[i] & 0xf) * 16 + (to_img[i] & 0xf)]++;
				diff++;
				if ((x + 1) < comp->x1)
					comp->x1 = x + 1;
				if ((x + 1) > comp->x2)
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

// diff == 0 if onlye black to white or white to black
static int it8951_gray4_wf_a2_match(struct it8951_gray4_comparator *comp)
{
	int diff = 0;
	int i;
	for (i = 0; i < 256; i++)
	{
		if ((i != 0x0f) && (i != 0xf0))
			diff += comp->transitions[i];
	}
	return diff;
}

// diff == 0 if only any to black or white
static int it8951_gray4_wf_du_match(struct it8951_gray4_comparator *comp)
{
	int diff = 0;
	int f, t;
	for (f = 0; f < 16; f++)
	{
		for (t = 1; t < 15; t++)
		{ // except 0x0 black and 0xf white
			diff += comp->transitions[f * 16 + t];
		}
	}
	return diff;
}

static int it8951_gray4_auto_wf(uint8_t *from_img, uint8_t *to_img, uint32_t w, uint32_t h, struct it8951_gray4_comparator *comp)
{
	int wf = -1, diff = 0, du_diff = 0, a2_diff = 0, len;

	memset(comp, 0, sizeof(struct it8951_gray4_comparator));
	comp->transitions = kzalloc(256 * sizeof(uint32_t), GFP_KERNEL);

	diff = it8951_gray4_compare(from_img, to_img, w, h, comp);

	len = w * h / 2;

	if (diff > 0)
	{
		a2_diff = it8951_gray4_wf_a2_match(comp);

		if (a2_diff == 0)
		{
			wf = IT8951_MODE_GLR16; // FIXME actually A2 for M641 LUT but GLR16 for others ?
		}
		else
		{
			du_diff = it8951_gray4_wf_du_match(comp);

			if (du_diff == 0)
			{ // DU for any gray to black or white
				wf = IT8951_MODE_DU;
			}
			else
			{
				if (comp->from_grays[0xf] > len / 2 && comp->to_grays[0xf] > len / 2) // GL16 for > 50% white pixel
				{
#ifdef WIP_FIX_REFRESH
					wf = IT8951_MODE_GL16;
#else
					// FIXME: GLD16 does not seems to work
					if (diff < len / 32)
					{ // reduced flashing GLD16 for < 3% diff
						wf = IT8951_MODE_GLD16;
					}
					else
					{
						wf = IT8951_MODE_GL16;
					}
#endif
				}
				else
				{
					wf = IT8951_MODE_GC16;
				}
			}
		}
	}

#ifdef IT8951_DEBUG
	printk(KERN_INFO "it8951: auto waveform wf:%d len:%d diff:%d a2_diff:%d du_diff:%d x1:%d,y1:%d,x2:%d,y2:%d w:%d,h:%d\n", wf, len, diff, a2_diff, du_diff, comp->x1, comp->y1, comp->x2, comp->y2, w, h);
#endif

	kfree(comp->transitions);
	comp->transitions = NULL;

	return wf;
}

/* Linux driver part */

static inline struct it8951_epd *
epd_from_tinydrm(struct tinydrm_device *tdev)
{
	return container_of(tdev, struct it8951_epd, tinydrm);
}

DEFINE_DRM_GEM_CMA_FOPS(it8951_fops);

static struct drm_driver it8951_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME |
					   DRIVER_ATOMIC,
	.fops = &it8951_fops,
	TINYDRM_GEM_DRIVER_OPS,
	.name = "it8951",
	.desc = "it8951 e-ink",
	.date = "20190913",
	.major = 1,
	.minor = 0,
};

static const struct of_device_id it8951_of_match[] = {
	{.compatible = "ite,it8951"},
	{},
};
MODULE_DEVICE_TABLE(of, it8951_of_match);

static const struct spi_device_id it8951_id[] = {
	{"it8951", 0},
	{},
};
MODULE_DEVICE_TABLE(spi, it8951_id);

#define CLIP_PADDING 4

// CLIP_PADDING bytes padding
static void it8951_fb_clip_padding(struct drm_clip_rect *clip, int w, int h)
{
	if (clip->x1 % CLIP_PADDING != 0)
		clip->x1 -= clip->x1 % CLIP_PADDING;

	if (clip->y1 % CLIP_PADDING != 0)
		clip->y1 -= clip->y1 % CLIP_PADDING;

	if (clip->x2 % CLIP_PADDING != 0)
		clip->x2 += CLIP_PADDING - (clip->x2 % CLIP_PADDING);

	if (clip->y2 % CLIP_PADDING != 0)
		clip->y2 += CLIP_PADDING - (clip->y2 % CLIP_PADDING);

	if (clip->x2 > w)
		clip->x2 = w;

	if (clip->y2 > h)
		clip->y2 = h;
}

static void it8951_run(struct it8951_epd *epd)
{
	if (!epd->running)
		it8951_spi_system_run(epd);
	epd->running = true;
}

static void it8951_standby(struct it8951_epd *epd)
{
	if (epd->running)
		it8951_spi_standby(epd);
	epd->running = false;
}

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
	bool full_screen, full_refresh;
	void *tmp_buf = NULL, *tmp_prev_buf = NULL;
	u8 *buf = NULL;
	int ret, wf, clip_w, clip_h;

	struct it8951_load_img_info load_img_info;

	if (!epd->enabled)
	{
		// printk(KERN_WARNING "it8951: not enabled yet\n");
		return 0;
	}

	// full refresh for ghosting
	if (epd->update_mode == -1 && epd->last_full_refresh >= epd->ghosting_refresh)
	{
		clip.x1 = 0;
		clip.x2 = epd->w;
		clip.y1 = 0;
		clip.y2 = epd->h;

		full_screen = true;
		full_refresh = true;
	}
	else
	{
		full_screen = tinydrm_merge_clips(&clip, clips, num_clips, flags, epd->w, epd->h);
		full_refresh = false;

		it8951_fb_clip_padding(&clip, epd->w, epd->h);
	}

	clip_w = clip.x2 - clip.x1;
	clip_h = clip.y2 - clip.y1;

	load_img_info.endian_type = IT8951_LDIMG_L_ENDIAN;
	load_img_info.pixel_format = IT8951_4BPP;
	load_img_info.rotate = epd->rotation / 90;
	load_img_info.img_buf_base_addr = epd->img_buf_addr;

#ifdef IT8951_DEBUG
	printk(KERN_INFO "it8951: dirty panel:%dx%d flags:%d color:%d pitch:%d clips:%d, full_screen:%d, x1:%d, y1:%d, x2:%d, y2:%d\n",
		   epd->w, epd->h, flags, color, fb->pitches[0], num_clips, full_screen, clip.x1, clip.y1, clip.x2, clip.y2);
#endif

	// create tmp buffer
	tmp_prev_buf = kmalloc(clip_w * clip_h / 2, GFP_KERNEL);

	// copy previous buffer
	if (!full_refresh && epd->update_mode == -1)
	{
		it8951_gray4_memcpy(tmp_prev_buf, epd->buf, epd->w, &clip);
	}

	if (import_attach)
	{
		ret = dma_buf_begin_cpu_access(import_attach->dmabuf,
									   DMA_FROM_DEVICE);
		if (ret)
			goto out_free;
	}

	it8951_xrgb8888_to_gray4(epd->buf, cma_obj->vaddr, fb, &clip);

	if (import_attach)
	{
		ret = dma_buf_end_cpu_access(import_attach->dmabuf,
									 DMA_FROM_DEVICE);
		if (ret)
			goto out_free;
	}

	if (full_screen || full_refresh)
	{
		buf = epd->buf;
	}
	else
	{
		// create tmp buffer
		tmp_buf = kmalloc(clip_w * clip_h / 2, GFP_KERNEL);
		it8951_gray4_memcpy(tmp_buf, epd->buf, epd->w, &clip);
		buf = tmp_buf;
	}

	if (full_refresh)
	{
		wf = IT8951_MODE_GC16;
	}
	else
	{
		if (epd->update_mode == -1)
		{
			struct it8951_gray4_comparator comp;
			wf = it8951_gray4_auto_wf(tmp_prev_buf, buf, clip_w, clip_h, &comp);
#ifdef WIP_FIX_REFRESH
			// we use our own clip from detected changes
			// no change
			if ((comp.x1 > comp.x2) || (comp.y1 > comp.y2))
			{
				wf = -1;
			}
			else
			{
				int nx1, ny1, nx2, ny2;
				nx1 = clip.x1 + comp.x1;
				ny1 = clip.y1 + comp.y1;
				nx2 = clip.x1 + comp.x2;
				ny2 = clip.y1 + comp.y2;

				if (full_screen || full_refresh)
				{
				}
				else
				{
					clip.x1 = nx1;
					clip.y1 = ny1;
					clip.x2 = nx2;
					clip.y2 = ny2;

					it8951_fb_clip_padding(&clip, epd->w, epd->h);

					clip_w = clip.x2 - clip.x1;
					clip_h = clip.y2 - clip.y1;

					kfree(tmp_buf);
					tmp_buf = kmalloc(clip_w * clip_h / 2, GFP_KERNEL);
					it8951_gray4_memcpy(tmp_buf, epd->buf, epd->w, &clip);
					buf = tmp_buf;
				}

#ifdef IT8951_DEBUG
				printk(KERN_INFO "it8951: new clip x1:%d, y1:%d, x2:%d, y2:%d\n", clip.x1, clip.y1, clip.x2, clip.y2);
#endif
			}
#endif
		}
		else
		{
			wf = epd->update_mode;
		}
	}

	if (wf != -1)
	{
		it8951_run(epd);

		load_img_info.start_fb_addr = (uint32_t)buf;

		if (full_screen)
		{
			it8951_spi_wait_for_display_ready(epd);
			it8951_spi_packed_pixel_write(epd, &load_img_info);
		}
		else
		{
			struct it8951_area_img_info area_img_info;
			//Set Load Area
			area_img_info.x = clip.x1;
			area_img_info.y = clip.y1;
			area_img_info.width = clip_w;
			area_img_info.height = clip_h;

			it8951_spi_wait_for_display_ready(epd);
			it8951_spi_packed_pixel_write_area(epd, &load_img_info, &area_img_info);
		}

		it8951_spi_display_area(epd, clip.x1, clip.y1, clip_w, clip_h, wf, load_img_info.rotate);

		it8951_standby(epd);

		if (epd->update_mode == -1)
		{
			if (wf == IT8951_MODE_GC16 && full_screen)
				epd->last_full_refresh = 0;
			else
				epd->last_full_refresh++;
		}
	}
	else
	{
		// no refresh needed
	}

out_free:
	if (tmp_buf)
		kfree(tmp_buf);
	if (tmp_prev_buf)
		kfree(tmp_prev_buf);

	return ret;
}

static const struct drm_framebuffer_funcs it8951_fb_funcs = {
	.destroy = drm_gem_fb_destroy,
	.create_handle = drm_gem_fb_create_handle,
	.dirty = tinydrm_fb_dirty,
};

static void it8951_clear_dpy(struct it8951_epd *epd)
{
	struct it8951_load_img_info load_img_info;

	if(!epd->enabled)
		return;

	it8951_run(epd);
	// first refresh
	load_img_info.endian_type = IT8951_LDIMG_L_ENDIAN;
	load_img_info.pixel_format = IT8951_4BPP;
	load_img_info.rotate = epd->rotation / 90;
	load_img_info.img_buf_base_addr = epd->img_buf_addr;

	// init waveform
	load_img_info.start_fb_addr = (uint32_t)epd->buf;

	it8951_spi_wait_for_display_ready(epd);
	it8951_spi_packed_pixel_write(epd, &load_img_info);

	it8951_spi_display_area(epd, 0, 0, epd->w, epd->h, IT8951_MODE_INIT, load_img_info.rotate);

	it8951_standby(epd);
}

static void it8951_pipe_enable(struct drm_simple_display_pipe *pipe,
							   struct drm_crtc_state *crtc_state,
							   struct drm_plane_state *plane_state)
{
	int cur_vcom;
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct it8951_epd *epd = epd_from_tinydrm(tdev);

	printk(KERN_INFO "it8951: pipe enable\n");

	gpiod_set_value_cansleep(epd->reset, 0);
	msleep(100);
	gpiod_set_value_cansleep(epd->reset, 1);
	msleep(100);

	epd->running = true;

	it8951_spi_get_system_info(epd);

	if (epd->w != epd->dev_info.panel_w || epd->h != epd->dev_info.panel_h)
	{
		printk(KERN_ERR "it8951: panel size does not match device tree\n");
		return;
	}

	if (epd->rotation == 90 || epd->rotation == 270)
	{
		epd->w = epd->h;
		epd->h = epd->w;
	}

	epd->img_buf_addr = epd->dev_info.img_buf_addr_l | (epd->dev_info.img_buf_addr_h << 16);

	//Set to Enable I80 Packed mode
	it8951_spi_write_reg(epd, I80CPCR, 0x0001);

	// Get current vcom value
	it8951_spi_write_cmd_code(epd, USDEF_I80_CMD_VCOM);
	it8951_spi_write_data(epd, 0);
	cur_vcom = it8951_spi_read_data(epd);

	printk(KERN_INFO "it8951: VCOM -%dmV\n", cur_vcom);
	if (vcom)
	{
		if (vcom != -cur_vcom)
		{
			if (vcom < -200 && vcom > -2700)
			{
				printk(KERN_INFO "it8951: change VCOM to %dmV\n", vcom);
				it8951_spi_write_cmd_code(epd, USDEF_I80_CMD_VCOM);
				it8951_spi_write_data(epd, 1);
				it8951_spi_write_data(epd, -vcom);

				it8951_spi_write_cmd_code(epd, USDEF_I80_CMD_VCOM);
				it8951_spi_write_data(epd, 0);
				cur_vcom = it8951_spi_read_data(epd);
				printk(KERN_INFO "it8951: VCOM -%dmV\n", cur_vcom);
			}
			else
			{
				printk(KERN_WARNING "it8951: invalid VCOM %d, must be between -200 and -2700\n", vcom);
			}
		}
	}

	epd->enabled = true;
	epd->last_full_refresh = 0;

	epd->buf = NULL;

	// create local 4-bits buffer
	epd->buf = kmalloc(epd->w * epd->h / 2, GFP_KERNEL);
	memset(epd->buf, 0xff, epd->w * epd->h / 2); // set pixels to white

	it8951_clear_dpy(epd);
}

static void it8951_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct it8951_epd *epd = epd_from_tinydrm(tdev);

	printk(KERN_INFO "it8951: pipe disable\n");

	mutex_lock(&tdev->dirty_lock);
	// clear screen before leaving
	if (epd->buf)
	{
		it8951_clear_dpy(epd);
		kfree(epd->buf);
	}

	epd->enabled = false;
	mutex_unlock(&tdev->dirty_lock);
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

static struct drm_display_mode it8951_mode = {
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

static ssize_t it8951_ghosting_refresh_show(struct device *device,
											struct device_attribute *attr,
											char *buf)
{
	struct it8951_epd *epd = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", (epd->ghosting_refresh));
}

static ssize_t it8951_ghosting_refresh_store(struct device *device,
											 struct device_attribute *attr,
											 const char *buf, size_t count)
{
	struct it8951_epd *epd = dev_get_drvdata(device);

	sscanf(buf, "%du", &epd->ghosting_refresh);
	return count;
}

static struct device_attribute it8951_update_mode_attr = __ATTR(update_mode, S_IRUGO | S_IWUSR, it8951_update_mode_show, it8951_update_mode_store);
static struct device_attribute it8951_ghosting_refresh_attr = __ATTR(ghosting_refresh, S_IRUGO | S_IWUSR, it8951_ghosting_refresh_show, it8951_ghosting_refresh_store);

static int it8951_probe(struct spi_device *spi)
{
	const struct drm_display_mode *mode;
	const struct of_device_id *match;
	struct device *dev = &spi->dev;
	struct tinydrm_device *tdev;
	struct it8951_epd *epd;
	int ret;

	match = of_match_device(it8951_of_match, dev);

	/* The SPI device is used to allocate dma memory */
	if (!dev->coherent_dma_mask)
	{
		ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret)
		{
			dev_warn(dev, "Failed to set dma mask %d\n", ret);
			return ret;
		}
	}

	epd = devm_kzalloc(dev, sizeof(*epd), GFP_KERNEL);
	if (!epd)
		return -ENOMEM;

	epd->spi = spi;

	epd->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(epd->reset))
	{
		ret = PTR_ERR(epd->reset);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
		return ret;
	}

	epd->hrdy = devm_gpiod_get(dev, "hrdy", GPIOD_IN);
	if (IS_ERR(epd->hrdy))
	{
		ret = PTR_ERR(epd->hrdy);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "Failed to get gpio 'hrdy'\n");
		return ret;
	}

	epd->update_mode = update_mode;
	epd->ghosting_refresh = ghosting_refresh;

	tdev = &epd->tinydrm;

	ret = devm_tinydrm_init(dev, tdev, &it8951_fb_funcs, &it8951_driver);
	if (ret)
		return ret;

	ret = device_property_read_u32(dev, "rotation", &epd->rotation);
	if (ret)
		epd->rotation = 0;

	epd->w = 800;
	epd->h = 600;

	ret = device_property_read_u32(dev, "xres", &epd->w);
	if (!ret)
	{
		ret = device_property_read_u32(dev, "yres", &epd->h);
		if (!ret)
		{
			it8951_mode.hdisplay = epd->w;
			it8951_mode.hsync_start = epd->w;
			it8951_mode.hsync_end = epd->w;
			it8951_mode.htotal = epd->w;
			it8951_mode.vdisplay = epd->h;
			it8951_mode.vsync_start = epd->h;
			it8951_mode.vsync_end = epd->h;
			it8951_mode.vtotal = epd->h;
		}
	}

	mode = &it8951_mode;

	tdev->fb_dirty = it8951_fb_dirty;

	ret = tinydrm_display_pipe_init(tdev, &it8951_pipe_funcs,
									DRM_MODE_CONNECTOR_VIRTUAL,
									it8951_formats,
									ARRAY_SIZE(it8951_formats), mode, epd->rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(tdev->drm);

	spi_set_drvdata(spi, epd);

	printk(KERN_INFO "it8951: SPI speed: %uMHz\n", spi->max_speed_hz / 1000000);

	ret = devm_tinydrm_register(tdev);
	if (ret)
		return ret;

	ret = device_create_file(&spi->dev, &it8951_update_mode_attr);
	if (ret)
		dev_err(dev, "Failed to create sysfs update_mode\n");

	ret = device_create_file(&spi->dev, &it8951_ghosting_refresh_attr);
	if (ret)
		dev_err(dev, "Failed to create sysfs ghosting_refresh\n");

	return ret;
}

static int it8951_remove(struct spi_device *spi)
{
	device_remove_file(&spi->dev, &it8951_update_mode_attr);
	device_remove_file(&spi->dev, &it8951_ghosting_refresh_attr);

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
