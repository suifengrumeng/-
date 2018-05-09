/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <binman_sym.h>
#include <dm.h>
#include <spl.h>
#include <asm/u-boot.h>
#include <nand.h>
#include <fat.h>
#include <version.h>
#include <image.h>
#include <malloc.h>
#include <dm/root.h>
#include <linux/compiler.h>
#include <fdt_support.h>

#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/display.h>
#include <videomodes.h>
#include <asm/arch/lcdc.h>
#include <video_fb.h>
#include <asm/gpio.h>
#include <asm/arch/gpio.h>
#include <fdtdec.h>
#include <malloc.h>
#include <linux/ctype.h>

#include "booting.h"

DECLARE_GLOBAL_DATA_PTR;

#ifndef CONFIG_SYS_UBOOT_START
#define CONFIG_SYS_UBOOT_START	CONFIG_SYS_TEXT_BASE
#endif
#ifndef CONFIG_SYS_MONITOR_LEN
/* Unknown U-Boot size, let's assume it will not be more than 200 KB */
#define CONFIG_SYS_MONITOR_LEN	(200 * 1024)
#endif

u32 *boot_params_ptr = NULL;

/* See spl.h for information about this */
binman_sym_declare(ulong, u_boot_any, pos);

/* Define board data structure */
static bd_t bdata __attribute__ ((section(".data")));

/*
 * Board-specific Platform code can reimplement show_boot_progress () if needed
 */
__weak void show_boot_progress(int val) {}

/*
 * Default function to determine if u-boot or the OS should
 * be started. This implementation always returns 1.
 *
 * Please implement your own board specific funcion to do this.
 *
 * RETURN
 * 0 to not start u-boot
 * positive if u-boot should start
 */
#ifdef CONFIG_SPL_OS_BOOT
__weak int spl_start_uboot(void)
{
	puts("SPL: Please implement spl_start_uboot() for your board\n");
	puts("SPL: Direct Linux boot not active!\n");
	return 1;
}

/* weak default platform specific function to initialize
 * dram banks
 */
__weak int dram_init_banksize(void)
{
	return 0;
}

/*
 * Weak default function for arch specific zImage check. Return zero
 * and fill start and end address if image is recognized.
 */
int __weak bootz_setup(ulong image, ulong *start, ulong *end)
{
	 return 1;
}
#endif

void spl_fixup_fdt(void)
{
#if defined(CONFIG_SPL_OF_LIBFDT) && defined(CONFIG_SYS_SPL_ARGS_ADDR)
	void *fdt_blob = (void *)CONFIG_SYS_SPL_ARGS_ADDR;
	int err;

	err = fdt_check_header(fdt_blob);
	if (err < 0) {
		printf("fdt_root: %s\n", fdt_strerror(err));
		return;
	}

	/* fixup the memory dt node */
	err = fdt_shrink_to_minimum(fdt_blob, 0);
	if (err == 0) {
		printf("spl: fdt_shrink_to_minimum err - %d\n", err);
		return;
	}

	err = arch_fixup_fdt(fdt_blob);
	if (err) {
		printf("spl: arch_fixup_fdt err - %d\n", err);
		return;
	}
#endif
}

/*
 * Weak default function for board specific cleanup/preparation before
 * Linux boot. Some boards/platforms might not need it, so just provide
 * an empty stub here.
 */
__weak void spl_board_prepare_for_linux(void)
{
	/* Nothing to do! */
}

__weak void spl_board_prepare_for_boot(void)
{
	/* Nothing to do! */
}

void spl_set_header_raw_uboot(struct spl_image_info *spl_image)
{
	ulong u_boot_pos = binman_sym(ulong, u_boot_any, pos);

	spl_image->size = CONFIG_SYS_MONITOR_LEN;
	if (u_boot_pos != BINMAN_SYM_MISSING) {
		/* biman does not support separate entry addresses at present */
		spl_image->entry_point = u_boot_pos;
		spl_image->load_addr = u_boot_pos;
	} else {
		spl_image->entry_point = CONFIG_SYS_UBOOT_START;
		spl_image->load_addr = CONFIG_SYS_TEXT_BASE;
	}
	spl_image->os = IH_OS_U_BOOT;
	spl_image->name = "U-Boot";
}

int spl_parse_image_header(struct spl_image_info *spl_image,
			   const struct image_header *header)
{
	if (image_get_magic(header) == IH_MAGIC) {
#ifdef CONFIG_SPL_LEGACY_IMAGE_SUPPORT
		u32 header_size = sizeof(struct image_header);

		if (spl_image->flags & SPL_COPY_PAYLOAD_ONLY) {
			/*
			 * On some system (e.g. powerpc), the load-address and
			 * entry-point is located at address 0. We can't load
			 * to 0-0x40. So skip header in this case.
			 */
			spl_image->load_addr = image_get_load(header);
			spl_image->entry_point = image_get_ep(header);
			spl_image->size = image_get_data_size(header);
		} else {
			spl_image->entry_point = image_get_load(header);
			/* Load including the header */
			spl_image->load_addr = spl_image->entry_point -
				header_size;
			spl_image->size = image_get_data_size(header) +
				header_size;
		}
		spl_image->os = image_get_os(header);
		spl_image->name = image_get_name(header);
		debug("spl: payload image: %.*s load addr: 0x%lx size: %d\n",
			IH_NMLEN, spl_image->name,
			spl_image->load_addr, spl_image->size);
#else
		/* LEGACY image not supported */
		debug("Legacy boot image support not enabled, proceeding to other boot methods\n");
		return -EINVAL;
#endif
	} else {
#ifdef CONFIG_SPL_PANIC_ON_RAW_IMAGE
		/*
		 * CONFIG_SPL_PANIC_ON_RAW_IMAGE is defined when the
		 * code which loads images in SPL cannot guarantee that
		 * absolutely all read errors will be reported.
		 * An example is the LPC32XX MLC NAND driver, which
		 * will consider that a completely unreadable NAND block
		 * is bad, and thus should be skipped silently.
		 */
		panic("** no mkimage signature but raw image not supported");
#endif

#ifdef CONFIG_SPL_OS_BOOT
		ulong start, end;

		if (!bootz_setup((ulong)header, &start, &end)) {
			spl_image->name = "Linux";
			spl_image->os = IH_OS_LINUX;
			spl_image->load_addr = CONFIG_SYS_LOAD_ADDR;
			spl_image->entry_point = CONFIG_SYS_LOAD_ADDR;
			spl_image->size = end - start;
			debug("spl: payload zImage, load addr: 0x%lx size: %d\n",
			      spl_image->load_addr, spl_image->size);
			return 0;
		}
#endif

#ifdef CONFIG_SPL_RAW_IMAGE_SUPPORT
		/* Signature not found - assume u-boot.bin */
		debug("mkimage signature not found - ih_magic = %x\n",
			header->ih_magic);
		spl_set_header_raw_uboot(spl_image);
#else
		/* RAW image not supported, proceed to other boot methods. */
		debug("Raw boot image support not enabled, proceeding to other boot methods\n");
		return -EINVAL;
#endif
	}

	return 0;
}

__weak void __noreturn jump_to_image_no_args(struct spl_image_info *spl_image)
{
	typedef void __noreturn (*image_entry_noargs_t)(void);

	image_entry_noargs_t image_entry =
		(image_entry_noargs_t)spl_image->entry_point;

	debug("image entry point: 0x%lX\n", spl_image->entry_point);
	image_entry();
}

static int spl_common_init(bool setup_malloc)
{
	int ret;

	debug("spl_early_init()\n");

#if CONFIG_VAL(SYS_MALLOC_F_LEN)
	if (setup_malloc) {
#ifdef CONFIG_MALLOC_F_ADDR
		gd->malloc_base = CONFIG_MALLOC_F_ADDR;
#endif
		gd->malloc_limit = CONFIG_VAL(SYS_MALLOC_F_LEN);
		gd->malloc_ptr = 0;
	}
#endif
	ret = bootstage_init(true);
	if (ret) {
		debug("%s: Failed to set up bootstage: ret=%d\n", __func__,
		      ret);
		return ret;
	}
	bootstage_mark_name(BOOTSTAGE_ID_START_SPL, "spl");
	if (CONFIG_IS_ENABLED(OF_CONTROL) && !CONFIG_IS_ENABLED(OF_PLATDATA)) {
		ret = fdtdec_setup();
		if (ret) {
			debug("fdtdec_setup() returned error %d\n", ret);
			return ret;
		}
	}
	if (CONFIG_IS_ENABLED(DM)) {
		bootstage_start(BOOTSTATE_ID_ACCUM_DM_SPL, "dm_spl");
		/* With CONFIG_SPL_OF_PLATDATA, bring in all devices */
		ret = dm_init_and_scan(!CONFIG_IS_ENABLED(OF_PLATDATA));
		bootstage_accum(BOOTSTATE_ID_ACCUM_DM_SPL);
		if (ret) {
			debug("dm_init_and_scan() returned error %d\n", ret);
			return ret;
		}
	}

	return 0;
}

void spl_set_bd(void)
{
	if (!gd->bd)
		gd->bd = &bdata;
}

int spl_early_init(void)
{
	int ret;

	ret = spl_common_init(true);
	if (ret)
		return ret;
	gd->flags |= GD_FLG_SPL_EARLY_INIT;

	return 0;
}

int spl_init(void)
{
	int ret;
	bool setup_malloc = !(IS_ENABLED(CONFIG_SPL_STACK_R) &&
			IS_ENABLED(CONFIG_SPL_SYS_MALLOC_SIMPLE));

	if (!(gd->flags & GD_FLG_SPL_EARLY_INIT)) {
		ret = spl_common_init(setup_malloc);
		if (ret)
			return ret;
	}
	gd->flags |= GD_FLG_SPL_INIT;

	return 0;
}

#ifndef BOOT_DEVICE_NONE
#define BOOT_DEVICE_NONE 0xdeadbeef
#endif

__weak void board_boot_order(u32 *spl_boot_list)
{
	spl_boot_list[0] = spl_boot_device();
}

static struct spl_image_loader *spl_ll_find_loader(uint boot_device)
{
	struct spl_image_loader *drv =
		ll_entry_start(struct spl_image_loader, spl_image_loader);
	const int n_ents =
		ll_entry_count(struct spl_image_loader, spl_image_loader);
	struct spl_image_loader *entry;

	for (entry = drv; entry != drv + n_ents; entry++) {
		if (boot_device == entry->boot_device)
			return entry;
	}

	/* Not found */
	return NULL;
}

static int spl_load_image(struct spl_image_info *spl_image,
			  struct spl_image_loader *loader)
{
	struct spl_boot_device bootdev;

	bootdev.boot_device = loader->boot_device;
	bootdev.boot_device_name = NULL;

	return loader->load_image(spl_image, &bootdev);
}

/**
 * boot_from_devices() - Try loading an booting U-Boot from a list of devices
 *
 * @spl_image: Place to put the image details if successful
 * @spl_boot_list: List of boot devices to try
 * @count: Number of elements in spl_boot_list
 * @return 0 if OK, -ve on error
 */
static int boot_from_devices(struct spl_image_info *spl_image,
			     u32 spl_boot_list[], int count)
{
	int i;

	for (i = 0; i < count && spl_boot_list[i] != BOOT_DEVICE_NONE; i++) {
		struct spl_image_loader *loader;

		loader = spl_ll_find_loader(spl_boot_list[i]);
#if defined(CONFIG_SPL_SERIAL_SUPPORT) && defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
		if (loader)
			printf("Trying to boot from %s\n", loader->name);
		else
			puts("SPL: Unsupported Boot Device!\n");
#endif
		if (loader && !spl_load_image(spl_image, loader))
			return 0;
	}

	return -ENODEV;
}


/****************************************/

#if 1
//#define CONFIG_SPL_DISPLAY_PRINT 


/*
#define AHB_RESET_OFFSET_DE_BE0		12
#define AHB_GATE_OFFSET_DE_BE0		12
#define CCM_DRAM_GATE_OFFSET_DE_BE0	26
#define SUNXI_DE_BE_MODE_ENABLE			(1 << 0)

#define AHB_RESET_OFFSET_LCD1		5
#define AHB_RESET_OFFSET_LCD0		4

#define AHB_GATE_OFFSET_LCD0		4

#define AHB_RESET_OFFSET_DRC0		25

*/
#define CONFIG_VIDEO_LCD_MODE "x:800,y:600,depth:18,pclk_khz:33000,le:87,ri:40,up:31,lo:13,hs:10,vs:10,sync:3,vmode:0"

static void sunxi_frontend_init(void) {}
static void sunxi_frontend_mode_set(const struct ctfb_res_modes *mode, unsigned int address) {}
static void sunxi_frontend_enable(void) {}


static void sunxi_composer_init(void)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	struct sunxi_de_be_reg * const de_be =
		(struct sunxi_de_be_reg *)SUNXI_DE_BE0_BASE;
	int i;

	sunxi_frontend_init();				//**********************

	/* Reset off */
	setbits_le32(&ccm->ahb_reset1_cfg, 1 << AHB_RESET_OFFSET_DE_BE0);

	/* Clocks on */
	setbits_le32(&ccm->ahb_gate1, 1 << AHB_GATE_OFFSET_DE_BE0);

	setbits_le32(&ccm->dram_clk_gate, 1 << CCM_DRAM_GATE_OFFSET_DE_BE0);

	clock_set_de_mod_clock(&ccm->be0_clk_cfg, 300000000);

	/* Engine bug, clear registers after reset */
	for (i = 0x0800; i < 0x1000; i += 4)
		writel(0, SUNXI_DE_BE0_BASE + i);

	setbits_le32(&de_be->mode, SUNXI_DE_BE_MODE_ENABLE);
}

void lcdc_init(struct sunxi_lcdc_reg * const lcdc)
{
	/* Init lcdc */
	writel(0, &lcdc->ctrl); /* Disable tcon */
	writel(0, &lcdc->int0); /* Disable all interrupts */

	/* Disable tcon0 dot clock */
	clrbits_le32(&lcdc->tcon0_dclk, SUNXI_LCDC_TCON0_DCLK_ENABLE);

	/* Set all io lines to tristate */
	writel(0xffffffff, &lcdc->tcon0_io_tristate);
	writel(0xffffffff, &lcdc->tcon1_io_tristate);
}

static void sunxi_lcdc_init(void)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	struct sunxi_lcdc_reg * const lcdc =
		(struct sunxi_lcdc_reg *)SUNXI_LCD0_BASE;

	setbits_le32(&ccm->ahb_reset1_cfg, 1 << AHB_RESET_OFFSET_LCD0);


	/* Clock on */
	setbits_le32(&ccm->ahb_gate1, 1 << AHB_GATE_OFFSET_LCD0);
	
	lcdc_init(lcdc);		//**********************
}


static void sunxi_drc_init(void)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	/* On sun6i the drc must be clocked even when in pass-through mode */
	setbits_le32(&ccm->ahb_reset1_cfg, 1 << AHB_RESET_OFFSET_DRC0);
	clock_set_de_mod_clock(&ccm->iep_drc0_clk_cfg, 300000000);
}

static void sunxi_engines_init(void)
{
	sunxi_composer_init();
	sunxi_lcdc_init();
	sunxi_drc_init();
}

enum sunxi_monitor {
	sunxi_monitor_none,
	sunxi_monitor_dvi,
	sunxi_monitor_hdmi,
	sunxi_monitor_lcd,
	sunxi_monitor_vga,
	sunxi_monitor_composite_pal,
	sunxi_monitor_composite_ntsc,
	sunxi_monitor_composite_pal_m,
	sunxi_monitor_composite_pal_nc,
};

struct sunxi_display {
	GraphicDevice graphic_device;
	enum sunxi_monitor monitor;
	unsigned int depth;
	unsigned int fb_addr;
	unsigned int fb_size;
} sunxi_display;

/*
static u32 sunxi_rgb2yuv_coef[12] = {
	0x00000107, 0x00000204, 0x00000064, 0x00000108,
	0x00003f69, 0x00003ed6, 0x000001c1, 0x00000808,
	0x000001c1, 0x00003e88, 0x00003fb8, 0x00000808
};
*/

static void sunxi_composer_mode_set(struct ctfb_res_modes *mode,
				    unsigned int address)
{
	struct sunxi_de_be_reg * const de_be =
		(struct sunxi_de_be_reg *)SUNXI_DE_BE0_BASE;
//	int i;

	sunxi_frontend_mode_set(mode, address);

	writel(SUNXI_DE_BE_HEIGHT(mode->yres) | SUNXI_DE_BE_WIDTH(mode->xres),
	       &de_be->disp_size);
	writel(SUNXI_DE_BE_HEIGHT(mode->yres) | SUNXI_DE_BE_WIDTH(mode->xres),
	       &de_be->layer0_size);

	writel(SUNXI_DE_BE_LAYER_STRIDE(mode->xres), &de_be->layer0_stride);
	writel(address << 3, &de_be->layer0_addr_low32b);
	writel(address >> 29, &de_be->layer0_addr_high4b);

	writel(SUNXI_DE_BE_LAYER_ATTR1_FMT_XRGB8888, &de_be->layer0_attr1_ctrl);

	setbits_le32(&de_be->mode, SUNXI_DE_BE_MODE_LAYER0_ENABLE);
	if (mode->vmode == FB_VMODE_INTERLACED)
		setbits_le32(&de_be->mode, SUNXI_DE_BE_MODE_DEFLICKER_ENABLE | SUNXI_DE_BE_MODE_INTERLACE_ENABLE);
	
	printf("SUNXI_DE_BE0_BASE = 0x%x, address = 0x%x, mode->xres = %d, mode->yres = %d, mode->vmode = %d\n", SUNXI_DE_BE0_BASE, address, mode->xres, mode->yres, mode->vmode);

}


static bool sunxi_is_composite(void)
{
	switch (sunxi_display.monitor) {
	case sunxi_monitor_none:
	case sunxi_monitor_dvi:
	case sunxi_monitor_hdmi:
	case sunxi_monitor_lcd:
	case sunxi_monitor_vga:
		return false;
	case sunxi_monitor_composite_pal:
	case sunxi_monitor_composite_ntsc:
	case sunxi_monitor_composite_pal_m:
	case sunxi_monitor_composite_pal_nc:
		return true;
	}

	return false; /* Never reached */
}

void lcdc_pll_set(struct sunxi_ccm_reg *ccm, int tcon, int dotclock,
		  int *clk_div, int *clk_double, bool is_composite)
{
	int value, n, m, min_m, max_m, diff;
	int best_n = 0, best_m = 0, best_diff = 0x0FFFFFFF;
	int best_double = 0;
	bool use_mipi_pll = 0;

	if (tcon == 0) {
		min_m = 6;
		max_m = 127;
	} else {
		min_m = 1;
		max_m = 15;
	}

	/*
	 * Find the lowest divider resulting in a matching clock, if there
	 * is no match, pick the closest lower clock, as monitors tend to
	 * not sync to higher frequencies.
	 */
	for (m = min_m; m <= max_m; m++) {
		n = (m * dotclock) / 3000;

		if ((n >= 9) && (n <= 127)) {
			value = (3000 * n) / m;
			diff = dotclock - value;
			if (diff < best_diff) {
				best_diff = diff;
				best_m = m;
				best_n = n;
				best_double = 0;
			}
		}

		/* These are just duplicates */
		if (!(m & 1))
			continue;

		/* No double clock on DE2 */
		n = (m * dotclock) / 6000;
		if ((n >= 9) && (n <= 127)) {
			value = (6000 * n) / m;
			diff = dotclock - value;
			if (diff < best_diff) {
				best_diff = diff;
				best_m = m;
				best_n = n;
				best_double = 1;
			}
		}
	}

	{
		clock_set_pll3(best_n * 3000000);
		debug("dotclock: %dkHz = %dkHz: (%d * 3MHz * %d) / %d\n",
		      dotclock,
		      (best_double + 1) * clock_get_pll3() / best_m / 1000,
		      best_double + 1, best_n, best_m);
	}

	if (tcon == 0) {
		u32 pll;

		if (use_mipi_pll)
			pll = CCM_LCD_CH0_CTRL_MIPI_PLL;
		else if (best_double)
			pll = CCM_LCD_CH0_CTRL_PLL3_2X;
		else
			pll = CCM_LCD_CH0_CTRL_PLL3;

		writel(CCM_LCD_CH0_CTRL_GATE | CCM_LCD_CH0_CTRL_RST | pll,
		       &ccm->lcd0_ch0_clk_cfg);

	}
	else {
		writel(CCM_LCD_CH1_CTRL_GATE |
		       (best_double ? CCM_LCD_CH1_CTRL_PLL3_2X :
				      CCM_LCD_CH1_CTRL_PLL3) |
		       CCM_LCD_CH1_CTRL_M(best_m), &ccm->lcd0_ch1_clk_cfg);
		if (is_composite)
			setbits_le32(&ccm->lcd0_ch1_clk_cfg,
				     CCM_LCD_CH1_CTRL_HALF_SCLK1);
	}

	*clk_div = best_m;
	*clk_double = best_double;
}


static void sunxi_ctfb_mode_to_display_timing(const struct ctfb_res_modes *mode,
					      struct display_timing *timing)
{
	timing->pixelclock.typ = mode->pixclock_khz * 1000;

	timing->hactive.typ = mode->xres;
	timing->hfront_porch.typ = mode->right_margin;
	timing->hback_porch.typ = mode->left_margin;
	timing->hsync_len.typ = mode->hsync_len;

	timing->vactive.typ = mode->yres;
	timing->vfront_porch.typ = mode->lower_margin;
	timing->vback_porch.typ = mode->upper_margin;
	timing->vsync_len.typ = mode->vsync_len;

	if (mode->sync & FB_SYNC_HOR_HIGH_ACT)
		timing->flags |= DISPLAY_FLAGS_HSYNC_HIGH;
	else
		timing->flags |= DISPLAY_FLAGS_HSYNC_LOW;
	if (mode->sync & FB_SYNC_VERT_HIGH_ACT)
		timing->flags |= DISPLAY_FLAGS_VSYNC_HIGH;
	else
		timing->flags |= DISPLAY_FLAGS_VSYNC_LOW;
	if (mode->vmode == FB_VMODE_INTERLACED)
		timing->flags |= DISPLAY_FLAGS_INTERLACED;
}


static int lcdc_get_clk_delay(const struct display_timing *mode, int tcon)
{
	int delay;

	delay = mode->vfront_porch.typ + mode->vsync_len.typ +
		mode->vback_porch.typ;
	if (mode->flags & DISPLAY_FLAGS_INTERLACED)
		delay /= 2;
	if (tcon == 1)
		delay -= 2;

	return (delay > 30) ? 30 : delay;
}


void lcdc_tcon0_mode_set(struct sunxi_lcdc_reg * const lcdc,
			 const struct display_timing *mode,
			 int clk_div, bool for_ext_vga_dac,
			 int depth, int dclk_phase)
{
	int bp, clk_delay, total, val;

	/* Use tcon0 */
	clrsetbits_le32(&lcdc->ctrl, SUNXI_LCDC_CTRL_IO_MAP_MASK,
			SUNXI_LCDC_CTRL_IO_MAP_TCON0);

	clk_delay = lcdc_get_clk_delay(mode, 0);			//********************
	writel(SUNXI_LCDC_TCON0_CTRL_ENABLE |
	       SUNXI_LCDC_TCON0_CTRL_CLK_DELAY(clk_delay), &lcdc->tcon0_ctrl);

	writel(SUNXI_LCDC_TCON0_DCLK_ENABLE |
	       SUNXI_LCDC_TCON0_DCLK_DIV(clk_div), &lcdc->tcon0_dclk);

	writel(SUNXI_LCDC_X(mode->hactive.typ) |
	       SUNXI_LCDC_Y(mode->vactive.typ), &lcdc->tcon0_timing_active);

	bp = mode->hsync_len.typ + mode->hback_porch.typ;
	total = mode->hactive.typ + mode->hfront_porch.typ + bp;
	writel(SUNXI_LCDC_TCON0_TIMING_H_TOTAL(total) |
	       SUNXI_LCDC_TCON0_TIMING_H_BP(bp), &lcdc->tcon0_timing_h);

	bp = mode->vsync_len.typ + mode->vback_porch.typ;
	total = mode->vactive.typ + mode->vfront_porch.typ + bp;
	writel(SUNXI_LCDC_TCON0_TIMING_V_TOTAL(total) |
	       SUNXI_LCDC_TCON0_TIMING_V_BP(bp), &lcdc->tcon0_timing_v);


	writel(SUNXI_LCDC_X(mode->hsync_len.typ) |
	       SUNXI_LCDC_Y(mode->vsync_len.typ), &lcdc->tcon0_timing_sync);

	writel(0, &lcdc->tcon0_hv_intf);
	writel(0, &lcdc->tcon0_cpu_intf);


	if (depth == 18 || depth == 16) {
		writel(SUNXI_LCDC_TCON0_FRM_SEED, &lcdc->tcon0_frm_seed[0]);
		writel(SUNXI_LCDC_TCON0_FRM_SEED, &lcdc->tcon0_frm_seed[1]);
		writel(SUNXI_LCDC_TCON0_FRM_SEED, &lcdc->tcon0_frm_seed[2]);
		writel(SUNXI_LCDC_TCON0_FRM_SEED, &lcdc->tcon0_frm_seed[3]);
		writel(SUNXI_LCDC_TCON0_FRM_SEED, &lcdc->tcon0_frm_seed[4]);
		writel(SUNXI_LCDC_TCON0_FRM_SEED, &lcdc->tcon0_frm_seed[5]);
		writel(SUNXI_LCDC_TCON0_FRM_TAB0, &lcdc->tcon0_frm_table[0]);
		writel(SUNXI_LCDC_TCON0_FRM_TAB1, &lcdc->tcon0_frm_table[1]);
		writel(SUNXI_LCDC_TCON0_FRM_TAB2, &lcdc->tcon0_frm_table[2]);
		writel(SUNXI_LCDC_TCON0_FRM_TAB3, &lcdc->tcon0_frm_table[3]);
		writel(((depth == 18) ?
			SUNXI_LCDC_TCON0_FRM_CTRL_RGB666 :
			SUNXI_LCDC_TCON0_FRM_CTRL_RGB565),
		       &lcdc->tcon0_frm_ctrl);
	}

	val = SUNXI_LCDC_TCON0_IO_POL_DCLK_PHASE(dclk_phase);
	if (mode->flags & DISPLAY_FLAGS_HSYNC_LOW)
		val |= SUNXI_LCDC_TCON_HSYNC_MASK;
	if (mode->flags & DISPLAY_FLAGS_VSYNC_LOW)
		val |= SUNXI_LCDC_TCON_VSYNC_MASK;

	writel(val, &lcdc->tcon0_io_polarity);

	writel(0, &lcdc->tcon0_io_tristate);
}


static void sunxi_lcdc_tcon0_mode_set(struct ctfb_res_modes *mode,
				      bool for_ext_vga_dac)
{
	struct sunxi_lcdc_reg * const lcdc =
		(struct sunxi_lcdc_reg *)SUNXI_LCD0_BASE;
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	int clk_div, clk_double, pin;
	struct display_timing timing;


	for (pin = SUNXI_GPD(0); pin <= SUNXI_GPD(27); pin++) {
		sunxi_gpio_set_cfgpin(pin, SUNXI_GPD_LCD0);				//??????????????????

	}

	lcdc_pll_set(ccm, 0, mode->pixclock_khz, &clk_div, &clk_double,	//**************
		     sunxi_is_composite());

	sunxi_ctfb_mode_to_display_timing(mode, &timing);				//*************
	lcdc_tcon0_mode_set(lcdc, &timing, clk_div, 1,	//************
			    sunxi_display.depth, 1);					
}

static void sunxi_composer_enable(void)
{
	struct sunxi_de_be_reg * const de_be =
		(struct sunxi_de_be_reg *)SUNXI_DE_BE0_BASE;

	sunxi_frontend_enable();

	setbits_le32(&de_be->reg_ctrl, SUNXI_DE_BE_REG_CTRL_LOAD_REGS);
	setbits_le32(&de_be->mode, SUNXI_DE_BE_MODE_START);
}

void lcdc_enable(struct sunxi_lcdc_reg * const lcdc, int depth)
{
	setbits_le32(&lcdc->ctrl, SUNXI_LCDC_CTRL_TCON_ENABLE);
}


static void sunxi_mode_set(struct ctfb_res_modes *mode, unsigned int address){
	struct sunxi_lcdc_reg * const lcdc =
		(struct sunxi_lcdc_reg *)SUNXI_LCD0_BASE;

		sunxi_composer_mode_set(mode, address);		//***********
		sunxi_lcdc_tcon0_mode_set(mode, 0);			//**********
		sunxi_composer_enable();					//*************
		lcdc_enable(lcdc, sunxi_display.depth);		//***********
}

static void memsetll(unsigned int *p, int c, int v)
{
	while (c--)
		*(p++) = v;
}


static void *video_fb_address;
static unsigned char *lcd_base;



int show_draw_logo(unsigned int * pd)
{
	
	unsigned short *src; 
	unsigned char *p_lcd;
	int i, j;

 	src =  (unsigned short *)gImage_booting;

	lcd_base = (unsigned char *)pd;

	p_lcd = lcd_base + 500 * 800 * 4;		//buff的形状应该是600 * 800 *32bit


    for(i=540;i<585;i++)
    {
        for(j=0;j<800;j++)
        {
            if(j>=200)
            {
             *p_lcd = 50;
			 p_lcd++;
             *p_lcd = 50;
			 p_lcd++;
             *p_lcd = 50;
			 p_lcd++;
			 *p_lcd = 50;
			 p_lcd++;

			}
            else
            {
             *p_lcd = (*src & 0x001f)<<3;
			 p_lcd++;
             *p_lcd = (((*src&0x07e0)>>5)&0x3f)<<2;
			 p_lcd++;
             *p_lcd = ((*src>>11) & 0x001f)<<3;
			 p_lcd++;
			 *p_lcd = 50;
			 p_lcd++;
			 src++;
            }
		  }

		}

	return 0;
}



int video_get_video_mode(unsigned int *xres, unsigned int *yres,
	unsigned int *depth, unsigned int *freq, const char **options)
{
#if 0 
	char *p = env_get("video-mode");
	if (!p)
		return 0;

	/* Skip over the driver name, which we don't care about. */
	p = strchr(p, ':');
	if (!p)
		return 0;

	/* Get the X-resolution*/
	while (*p && !isdigit(*p))
		p++;
	*xres = simple_strtoul(p, &p, 10);
	if (!*xres)
		return 0;

	/* Get the Y-resolution */
	while (*p && !isdigit(*p))
		p++;
	*yres = simple_strtoul(p, &p, 10);
	if (!*yres)
		return 0;

	/* Get the depth */
	while (*p && !isdigit(*p))
		p++;
	*depth = simple_strtoul(p, &p, 10);
	if (!*depth)
		return 0;

	/* Get the frequency */
	while (*p && !isdigit(*p))
		p++;
	*freq = simple_strtoul(p, &p, 10);
	if (!*freq)
		return 0;

	/* Find the extra options, if any */
	p = strchr(p, ',');
	*options = p ? p + 1 : NULL;
	#endif

	return 0;
}

struct ctfb_res_modes resc_mode_init[10] = {
	{ 640,  480, 60, 39683,  25200,  48,  16, 33, 10,  96, 2, 0, FB_VMODE_NONINTERLACED},
	{ 800,  600, 60, 25000,  40000,  88,  40, 23,  1, 128, 4, FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT, FB_VMODE_NONINTERLACED},
	{1024,  768, 60, 15384,  65000, 160,  24, 29,  3, 136, 6, 0, FB_VMODE_NONINTERLACED},
	{ 960,  720, 75, 13468,  74250, 176,  72, 27,  1, 112, 2, 0, FB_VMODE_NONINTERLACED},
	{1152,  864, 75,  9259, 108000, 256,  64, 32,  1, 128, 3, FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT, FB_VMODE_NONINTERLACED},
	{1280, 1024, 60,  9259, 108000, 248,  48, 38,  1, 112, 3, FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT, FB_VMODE_NONINTERLACED},
	{1280,  720, 60, 13468,  74250, 220, 110, 20,  5,  40, 5, FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT, FB_VMODE_NONINTERLACED},
	{1360,  768, 60, 11696,  85500, 256,  64, 17,  3, 112, 7, 0, FB_VMODE_NONINTERLACED},
	{1920, 1080, 60,  6734, 148500, 148,  88, 36,  4,  44, 5, FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT, FB_VMODE_NONINTERLACED},
	{1920, 1200, 60,  6494, 154000,  80,  48, 26,  3,  32, 6, FB_SYNC_HOR_HIGH_ACT, FB_VMODE_NONINTERLACED},

};


 void video_get_ctfb_res_res(int default_mode, unsigned int default_depth,
			      struct ctfb_res_modes **mode_ret,
			      unsigned int *depth_ret,
			      const char **options)
{
	unsigned int i, xres, yres, depth, refresh;

	*mode_ret = &resc_mode_init[default_mode];
	*depth_ret = default_depth;
	*options = NULL;

	if (!video_get_video_mode(&xres, &yres, &depth, &refresh, options))
		return;

	printf("xres = %d, yres = %d, depth = %d, refresh = %d ----------->\n", xres, yres, depth, refresh);

	for (i = 0; i < RES_MODES_COUNT; i++) {
		if (resc_mode_init[i].xres == xres &&
		    resc_mode_init[i].yres == yres &&
		    resc_mode_init[i].refresh == refresh) {
			*mode_ret = &resc_mode_init[i];
			*depth_ret = depth;
			return;
		}
	}

	printf("video-mode %dx%d-%d@%d not available, falling back to %dx%d-%d@%d\n",
	       xres, yres, depth, refresh, (*mode_ret)->xres,
	       (*mode_ret)->yres, *depth_ret, (*mode_ret)->refresh);
}


int drv_video_init(void){
//	static GraphicDevice *graphic_device = &sunxi_display.graphic_device;
	struct ctfb_res_modes  * mode = NULL;
//	struct ctfb_res_modes custom;
	const char *options;

	int overscan_offset, overscan_x, overscan_y;
	unsigned int fb_dma_addr;
//	char mon[16];
//	char *lcd_mode = CONFIG_VIDEO_LCD_MODE;


	memset(&sunxi_display, 0, sizeof(struct sunxi_display));

//	video_get_ctfb_res_modes(RES_MODE_1024x768, 24, &mode, &sunxi_display.depth, &options);
	video_get_ctfb_res_res(RES_MODE_1024x768, 24, &mode, &sunxi_display.depth, &options);

	sunxi_display.monitor = sunxi_monitor_lcd;
	sunxi_display.depth   = 18;
	overscan_x = 0;
	overscan_y = 0;

	mode->xres = 800;
	mode->yres = 600;
	mode->hsync_len= 10;
	mode->left_margin= 87;
	mode->lower_margin= 13;
	mode->pixclock = 39683;				//39683
	mode->pixclock_khz= 33000;			//33000
	mode->refresh= 60;					//60
	mode->right_margin= 40;
	mode->sync= 3;						// 3
	mode->upper_margin = 31;
	mode->vmode= 0;
	mode->vsync_len= 10;

	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
	gd->bd->bi_dram[0].size = get_effective_memsize();

	sunxi_display.fb_size = (mode->xres * mode->yres * 4 + 0xfff) & ~0xfff;
	overscan_offset = (overscan_y * mode->xres + overscan_x) * 4;
	if (overscan_offset)
		sunxi_display.fb_size += 0x1000;

	gd->fb_base = gd->bd->bi_dram[0].start +
		      gd->bd->bi_dram[0].size - sunxi_display.fb_size;	

	sunxi_engines_init();




	fb_dma_addr = gd->fb_base - CONFIG_SYS_SDRAM_BASE;			//************************
	sunxi_display.fb_addr = gd->fb_base;

//	video_fb_address = (void *) sunxi_display.fb_addr;

	sunxi_mode_set(mode, fb_dma_addr);

	memsetll(gd->fb_base, 800 * 600, 50);

//	printf("gd->bd->bi_dram[0].start =  0x%lx, gd->bd->bi_dram[0].size =  0x%lx, gd->fb_base = 0x%lx, fb_dma_addr =0x%x\n", gd->bd->bi_dram[0].start, gd->bd->bi_dram[0].size, gd->fb_base, fb_dma_addr);


/*
	graphic_device->frameAdrs = sunxi_display.fb_addr;
	graphic_device->gdfIndex = GDF_32BIT_X888RGB;
	graphic_device->gdfBytesPP = 4;
	graphic_device->winSizeX = mode->xres - 2 * overscan_x;
	graphic_device->winSizeY = mode->yres - 2 * overscan_y;
	graphic_device->plnSizeX = mode->xres * graphic_device->gdfBytesPP;
*/


	

//	printf("2. draw ---------->");
	show_draw_logo(gd->fb_base);
	mdelay(5000);

	return 0;
}


void spl_display_print(void){
	drv_video_init();
}

#endif


/*****************************************/



void board_init_r(gd_t *dummy1, ulong dummy2)
{
	u32 spl_boot_list[] = {
		BOOT_DEVICE_NONE,
		BOOT_DEVICE_NONE,
		BOOT_DEVICE_NONE,
		BOOT_DEVICE_NONE,
		BOOT_DEVICE_NONE,
	};
	struct spl_image_info spl_image;

	debug(">>spl:board_init_r()\n");

	spl_set_bd();

#ifdef CONFIG_SPL_OS_BOOT
	dram_init_banksize();
#endif

#if defined(CONFIG_SYS_SPL_MALLOC_START)
	mem_malloc_init(CONFIG_SYS_SPL_MALLOC_START,
			CONFIG_SYS_SPL_MALLOC_SIZE);
	gd->flags |= GD_FLG_FULL_MALLOC_INIT;
#endif
	if (!(gd->flags & GD_FLG_SPL_INIT)) {
		if (spl_init())
			hang();
	}
#if !defined(CONFIG_PPC) && !defined(CONFIG_ARCH_MX6)
	/*
	 * timer_init() does not exist on PPC systems. The timer is initialized
	 * and enabled (decrementer) in interrupt_init() here.
	 */
	timer_init();
#endif

#ifdef CONFIG_SPL_BOARD_INIT
	spl_board_init();
#endif

	spl_display_print();

//	printf("1111111111\n");
	memset(&spl_image, '\0', sizeof(spl_image));
#ifdef CONFIG_SYS_SPL_ARGS_ADDR
	spl_image.arg = (void *)CONFIG_SYS_SPL_ARGS_ADDR;
#endif
	board_boot_order(spl_boot_list);

	if (boot_from_devices(&spl_image, spl_boot_list,
			      ARRAY_SIZE(spl_boot_list))) {
		puts("SPL: failed to boot from all boot devices\n");
		hang();
	}

//	spl_display_print();
//	printf("22222222222222\n");

#ifdef CONFIG_CPU_V7M
	spl_image.entry_point |= 0x1;
#endif
	switch (spl_image.os) {
	case IH_OS_U_BOOT:
		debug("Jumping to U-Boot\n");
		break;
#if CONFIG_IS_ENABLED(ATF)
	case IH_OS_ARM_TRUSTED_FIRMWARE:
		debug("Jumping to U-Boot via ARM Trusted Firmware\n");
		spl_invoke_atf(&spl_image);
		break;
#endif
#ifdef CONFIG_SPL_OS_BOOT
	case IH_OS_LINUX:
		debug("Jumping to Linux\n");
		spl_fixup_fdt();
		spl_board_prepare_for_linux();
		jump_to_image_linux(&spl_image);
#endif
	default:
		debug("Unsupported OS image.. Jumping nevertheless..\n");
	}
#if CONFIG_VAL(SYS_MALLOC_F_LEN) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
	debug("SPL malloc() used %#lx bytes (%ld KB)\n", gd->malloc_ptr,
	      gd->malloc_ptr / 1024);
#endif
#ifdef CONFIG_BOOTSTAGE_STASH
	int ret;

	bootstage_mark_name(BOOTSTAGE_ID_END_SPL, "end_spl");
	ret = bootstage_stash((void *)CONFIG_BOOTSTAGE_STASH_ADDR,
			      CONFIG_BOOTSTAGE_STASH_SIZE);
	if (ret)
		debug("Failed to stash bootstage: err=%d\n", ret);
#endif

	debug("loaded - jumping to U-Boot...\n");
	spl_board_prepare_for_boot();
	jump_to_image_no_args(&spl_image);
}

/*
 * This requires UART clocks to be enabled.  In order for this to work the
 * caller must ensure that the gd pointer is valid.
 */


void preloader_console_init(void)
{
	gd->baudrate = CONFIG_BAUDRATE;

	serial_init();		/* serial communications setup */

	gd->have_console = 1;

	puts("\nU-Boot SPL " PLAIN_VERSION " (" U_BOOT_DATE " - " \
			U_BOOT_TIME ")\n");
#ifdef CONFIG_SPL_DISPLAY_PRINT
	spl_display_print();
#endif
}

/**
 * spl_relocate_stack_gd() - Relocate stack ready for board_init_r() execution
 *
 * Sometimes board_init_f() runs with a stack in SRAM but we want to use SDRAM
 * for the main board_init_r() execution. This is typically because we need
 * more stack space for things like the MMC sub-system.
 *
 * This function calculates the stack position, copies the global_data into
 * place, sets the new gd (except for ARM, for which setting GD within a C
 * function may not always work) and returns the new stack position. The
 * caller is responsible for setting up the sp register and, in the case
 * of ARM, setting up gd.
 *
 * All of this is done using the same layout and alignments as done in
 * board_init_f_init_reserve() / board_init_f_alloc_reserve().
 *
 * @return new stack location, or 0 to use the same stack
 */
ulong spl_relocate_stack_gd(void)
{
#ifdef CONFIG_SPL_STACK_R
	gd_t *new_gd;
	ulong ptr = CONFIG_SPL_STACK_R_ADDR;

#if defined(CONFIG_SPL_SYS_MALLOC_SIMPLE) && CONFIG_VAL(SYS_MALLOC_F_LEN)
	if (CONFIG_SPL_STACK_R_MALLOC_SIMPLE_LEN) {
		ptr -= CONFIG_SPL_STACK_R_MALLOC_SIMPLE_LEN;
		gd->malloc_base = ptr;
		gd->malloc_limit = CONFIG_SPL_STACK_R_MALLOC_SIMPLE_LEN;
		gd->malloc_ptr = 0;
	}
#endif
	/* Get stack position: use 8-byte alignment for ABI compliance */
	ptr = CONFIG_SPL_STACK_R_ADDR - roundup(sizeof(gd_t),16);
	new_gd = (gd_t *)ptr;
	memcpy(new_gd, (void *)gd, sizeof(gd_t));
#if CONFIG_IS_ENABLED(DM)
	dm_fixup_for_gd_move(new_gd);
#endif
#if !defined(CONFIG_ARM)
	gd = new_gd;
#endif
	return ptr;
#else
	return 0;
#endif
}
