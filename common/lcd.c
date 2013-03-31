/*
 * Common LCD routines for supported CPUs
 *
 */

/************************************************************************/
/* ** HEADER FILES							*/
/************************************************************************/

/* #define DEBUG */

#include <config.h>
#include <common.h>
#include <command.h>
#include <version.h>
#include <stdarg.h>
#include <linux/types.h>
#include <stdio_dev.h>
#if defined(CONFIG_POST)
#include <post.h>
#endif
#include <lcd.h>
#include <watchdog.h>
#include <asm/io.h>
#ifdef CONFIG_LCD

/************************************************************************/
/* ** FONT DATA								*/
/************************************************************************/
#include <video_font.h>		/* Get font data, width and height	*/
#include <video_font_data.h>
/************************************************************************/
/* ** LOGO DATA								*/
/************************************************************************/
#ifdef CONFIG_LCD_LOGO
# include <bmp_logo.h>		/* Get logo data, width and height	*/
# include <bmp_logo_data.h>
# if (CONSOLE_COLOR_WHITE >= BMP_LOGO_OFFSET) && (LCD_BPP != LCD_COLOR16)
#  error Default Color Map overlaps with Logo Color Map
# endif
#endif


ulong lcd_setmem (ulong addr);

DECLARE_GLOBAL_DATA_PTR;

vidinfo_t panel_info = {
  S3CFB_HRES,  //行点数 320
  S3CFB_VRES,  //行数   240
           0,  //显示宽度
           0,  //显示高度
   4,//PIXELBITS, //每像素点使用位数 16bpp
};

char lcd_is_enabled = 0;
int lcd_line_length;
int lcd_color_fg;
int lcd_color_bg;

void *lcd_base;		/* Start of framebuffer memory	*/
void *lcd_console_address;	/* Start of console buffer	*/

/*
 * Frame buffer memory information
 */


static int lcd_init (void *lcdbase);

extern ulong get_HCLK(void);
static void lcd_hline(int x,int y,int width,uint16_t color);
static void lcd_vline(int x,int y,int height,uint16_t color);

static void lcd_drawchars (ushort x, ushort y, uchar *str, int count);
static inline void lcd_puts_xy (ushort x, ushort y, uchar *s);
static inline void lcd_putc_xy (ushort x, ushort y, uchar  c);
static int lcd_getbgcolor (void);
static void lcd_setfgcolor (int color);
static void lcd_setbgcolor (int color);
static void *lcd_logo (void);

short console_col;
short console_row;

static void console_scrollup (void)
{
	/* Copy up rows ignoring the first one */
	memcpy (CONSOLE_ROW_FIRST, CONSOLE_ROW_SECOND, CONSOLE_SCROLL_SIZE);

	/* Clear the last one */
	memset (CONSOLE_ROW_LAST, COLOR_MASK(lcd_color_bg), CONSOLE_ROW_SIZE);
}

/*----------------------------------------------------------------------*/

static inline void console_back (void)
{
	if (--console_col < 0) {
		console_col = CONSOLE_COLS-1 ;
		if (--console_row < 0) {
			console_row = 0;
		}
	}

	lcd_putc_xy (console_col * VIDEO_FONT_WIDTH,
		     console_row * VIDEO_FONT_HEIGHT,
		     ' ');
}

/*----------------------------------------------------------------------*/

static inline void console_newline (void)
{
	++console_row;
	console_col = 0;

	/* Check if we need to scroll the terminal */
	if (console_row >= CONSOLE_ROWS) {
		/* Scroll everything up */
		console_scrollup () ;
		--console_row;
	}
}


void lcd_putc (const char c)
{
	if (!lcd_is_enabled) {
		serial_putc(c);
		return;
	}

	switch (c) {
	case '\r':	console_col = 0;
			return;

	case '\n':	console_newline();
			return;

	case '\t':	/* Tab (8 chars alignment) */
			console_col +=  8;
			console_col &= ~7;

			if (console_col >= CONSOLE_COLS) {
				console_newline();
			}
			return;

	case '\b':	console_back();
			return;

	default:	lcd_putc_xy (console_col * VIDEO_FONT_WIDTH,
				     console_row * VIDEO_FONT_HEIGHT,
				     c);
			if (++console_col >= CONSOLE_COLS) {
				console_newline();
			}
			return;
	}
	/* NOTREACHED */
}

void lcd_puts (const char *s)
{
	/*if (!lcd_is_enabled) {
		serial_puts (s);
		return;
	}*/
    serial_puts (s);
	while (*s) {
		lcd_putc (*s++);
	}
}

void lcd_printf(const char *fmt, ...)
{
	va_list args;
	char buf[CONFIG_SYS_PBSIZE];

	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);

	lcd_puts(buf);
}

/************************************************************************/
/* ** Low-Level Graphics Routines					*/
/************************************************************************/
static void lcd_hline(int x,int y,int width,uint16_t color)
{
  uint16_t *pp;
  int i;

  pp = (uint16_t *)((uint32_t)lcd_base + y * lcd_line_length + x * panel_info.vl_bpix / 8 );  
  for(i=0;i<width;i++)
  {
    *pp = color;
    pp++;
  } 
}

static void lcd_vline(int x,int y,int height,uint16_t color)
{
  uint16_t *pp;
  int i;

  for(i=0;i<height;i++)
  {
    pp = (uint16_t *)((uint32_t)lcd_base + (y + i) * lcd_line_length + x * NBITS(panel_info.vl_bpix)/ 8 );  
    *pp = color;
  } 
}

static void lcd_drawchars (ushort x, ushort y, uchar *str, int count)
{
	uchar *dest;
	ushort row;

#if LCD_BPP == LCD_MONOCHROME
	ushort off  = x * (1 << LCD_BPP) % 8;
#endif

	dest = (uchar *)(lcd_base + y * lcd_line_length + x * (1<<LCD_BPP)/ 8);

	for (row=0;  row < VIDEO_FONT_HEIGHT;  ++row, dest += lcd_line_length)  {
		uchar *s = str;
		int i;
#if LCD_BPP == LCD_COLOR16
		ushort *d = (ushort *)dest;
#else
		uchar *d = dest;
#endif

#if LCD_BPP == LCD_MONOCHROME
		uchar rest = *d & -(1 << (8-off));
		uchar sym;
#endif
		for (i=0; i<count; ++i) {
			uchar c, bits;

			c = *s++;
			bits = video_fontdata[c * VIDEO_FONT_HEIGHT + row];

#if LCD_BPP == LCD_MONOCHROME
			sym  = (COLOR_MASK(lcd_color_fg) & bits) |
			       (COLOR_MASK(lcd_color_bg) & ~bits);

			*d++ = rest | (sym >> off);
			rest = sym << (8-off);
#elif LCD_BPP == LCD_COLOR8
			for (c=0; c<8; ++c) {
				*d++ = (bits & 0x80) ?
						lcd_color_fg : lcd_color_bg;
				bits <<= 1;
			}
#elif LCD_BPP == LCD_COLOR16
			for (c=0; c<8; ++c) {
				*d++ = (bits & 0x80) ?
						lcd_color_fg : lcd_color_bg;
				bits <<= 1;
			}
#endif
		}
#if LCD_BPP == LCD_MONOCHROME
		*d  = rest | (*d & ((1 << (8-off)) - 1));
#endif
	}
}

/*----------------------------------------------------------------------*/

static inline void lcd_puts_xy (ushort x, ushort y, uchar *s)
{
#if defined(CONFIG_LCD_LOGO) && !defined(CONFIG_LCD_INFO_BELOW_LOGO)
	lcd_drawchars (x, y+BMP_LOGO_HEIGHT, s, strlen ((char *)s));
#else
	lcd_drawchars (x, y, s, strlen ((char *)s));
#endif
}

/*----------------------------------------------------------------------*/

static inline void lcd_putc_xy (ushort x, ushort y, uchar c)
{
#if defined(CONFIG_LCD_LOGO) && !defined(CONFIG_LCD_INFO_BELOW_LOGO)
	lcd_drawchars (x, y+BMP_LOGO_HEIGHT, &c, 1);
#else
	lcd_drawchars (x, y, &c, 1);
#endif
}


ulong calc_fbsize (void)
{
  ulong size;
  
  int line_length = (panel_info.vl_col * NBITS(panel_info.vl_bpix)) / 8;
  size = line_length * panel_info.vl_row;
//#ifdef LCD_FRAMEBUFFER
//  size = 4096;
//#endif
  
  return size;
}
/************************************************************************/
/*   根据给定的显示缓冲，初始化lcd                                           */
/*   uboot使用内存有限，只能开辟一个基本窗口                                  */
void lcd_ctrl_init(void *lcdbase)
{
  ulong freq_lcdclk;
  ulong freq_Hclk;
  ulong fb_size;
  unsigned char nn;
  unsigned short *pp;
  int i;

  printf("********************************************************\n");
  printf("initial lcd controller\n");
  //配置管脚
  GPICON_REG = 0xaaaaaaaa;
  GPIPUD_REG = 0xaaaaaaaa;
  GPJCON_REG = 0xaaaaaaaa;
  GPJPUD_REG = 0xaaaaaaaa;
  
  lcd_disable();
  S3C_WINCON0 &= (~(S3C_WINCONx_ENWIN_F_ENABLE));
  

  // (1)MOFPCON:SEL_BYPASS[3] value@0x7410800C 必须设置为’0’.
  MIFPCON_REG &= (~SEL_BYPASS_MASK);
  //(2)SPCON:LCD_SEL[1:0]value@0x74F0081A0 必须设置为’00’,使用主机 I/F 类型,或者设置为‘01’ 使用 RGB I/F 类型。
  SPCON_REG &= (~LCD_SEL_MASK);
  SPCON_REG |= (RGB_IF_STYLE_MASK);
  //(3)VIDCON0:配置视频输出格式和显示使能/禁止。
  
  freq_lcdclk = S3CFB_PIXEL_CLOCK;
  freq_Hclk = get_HCLK();
  nn = (unsigned char)(freq_Hclk / freq_lcdclk) - 1;
  
  if(freq_lcdclk < freq_Hclk/2)
  {
    
    S3C_VIDCON0 = S3C_VIDCON0_INTERLACE_F_PROGRESSIVE + S3C_VIDCON0_VIDOUT_RGB_IF + \
            S3C_VIDCON0_PNRMODE_RGB_P + S3C_VIDCON0_CLKVALUP_ST_FRM + S3C_VIDCON0_CLKVAL_F(nn) + \
            S3C_VIDCON0_CLKDIR_DIVIDED + S3C_VIDCON0_CLKSEL_F_HCLK; 
  }else
  {
    S3C_VIDCON0 = S3C_VIDCON0_INTERLACE_F_PROGRESSIVE + S3C_VIDCON0_VIDOUT_RGB_IF + \
            S3C_VIDCON0_PNRMODE_RGB_P + S3C_VIDCON0_CLKVALUP_ST_FRM + S3C_VIDCON0_CLKVAL_F(0) + \
            S3C_VIDCON0_CLKDIR_DIRECTED  + S3C_VIDCON0_CLKSEL_F_HCLK; 
  }
  printf("  clk_freq:%d MHz,  div_freq:%d ,rea_freq:%d MHz \n",freq_lcdclk/1000000,nn,freq_Hclk/(nn+1)/1000000);
  //(4)VIDCON1:RGB I/F 控制信号。
  nn = 0;

  if(S3CFB_IVCLK)
  {
    nn += S3C_VIDCON1_IVCLK_RISE_EDGE;
  }
  if(S3CFB_IHSYNC)
  {
    nn += S3C_VIDCON1_IHSYNC_INVERT;
  }
  if(S3CFB_IVSYNC)
  {
    nn += S3C_VIDCON1_IVSYNC_INVERT;
  }
  if(S3CFB_IVDEN)
  {
    nn += S3C_VIDCON1_IVDEN_INVERT;
  }
  S3C_VIDCON1 = (unsigned int)nn;
  S3C_VIDCON2 = 0;
  //(5)I80IFCONx:i80 系统 I/F 控制信号。
  //(6)ITUIFCON0:ITU(BT.601)接口控制
  //(7)VIDTCONx:配置视频输出时序和显示尺寸。
  S3C_VIDTCON0 = S3C_VIDTCON0_VBPD(S3CFB_VBP - 1) | S3C_VIDTCON0_VFPD(S3CFB_VFP - 1) | S3C_VIDTCON0_VSPW(S3CFB_VSW - 1);
  S3C_VIDTCON1 = S3C_VIDTCON1_HBPD(S3CFB_HBP - 1) | S3C_VIDTCON1_HFPD(S3CFB_HFP -1) | S3C_VIDTCON1_HSPW(S3CFB_HSW - 1);
  S3C_VIDTCON2 = S3C_VIDTCON2_LINEVAL(S3CFB_VRES - 1) | S3C_VIDTCON2_HOZVAL(S3CFB_HRES - 1);

  printf("\n HBP = %d HFP = %d HSW = %d,Hpixs:%d",S3CFB_HBP,S3CFB_HFP,S3CFB_HSW,S3CFB_HRES);
  printf("\n VBP = %d VFP = %d VSW = %d,Vpixs:%d",S3CFB_VBP,S3CFB_VFP,S3CFB_VSW,S3CFB_VRES);

  //(8)WINCONx:窗口格式设置
  S3C_WINCON0 = S3C_WINCONx_BPPMODE_F_16BPP_565|S3C_WINCONx_BYTSWP_ENABLE;
  S3C_WINCON1 = 0;
  S3C_WINCON2 = 0;
  S3C_WINCON3 = 0;
  S3C_WINCON4 = 0;
  //(9)VIDOSDxA ,VIDOSDxB:窗口位置设置
  
  S3C_VIDOSD0A = S3C_VIDOSDxA_OSD_LTX_F(0) + S3C_VIDOSDxA_OSD_LTY_F(0);
  S3C_VIDOSD0B = S3C_VIDOSDxB_OSD_RBX_F(S3CFB_HRES) | S3C_VIDOSDxB_OSD_RBY_F(S3CFB_VRES);
  S3C_VIDOSD0C = S3C_VIDOSD0C_OSDSIZE(S3CFB_HRES*S3CFB_VRES);
  
  S3C_VIDOSD1A = 0;
  S3C_VIDOSD1B = 0;
  S3C_VIDOSD1C = 0;
  S3C_VIDOSD1D = 0;
  S3C_VIDOSD2A = 0;
  S3C_VIDOSD2B = 0;
  S3C_VIDOSD2C = 0;
  S3C_VIDOSD2D = 0;
  S3C_VIDOSD3A = 0;
  S3C_VIDOSD3B = 0;
  S3C_VIDOSD3C = 0;
  S3C_VIDOSD4A = 0;
  S3C_VIDOSD4B = 0;
  S3C_VIDOSD4C = 0;

  //(10)VIDOSDxC:alpha 值设置

  //(11)VIDWxxADDx:源图像地址设置

  fb_size = calc_fbsize();
//#ifdef LCD_FRAMEBUFFER
 // fb_size =   (panel_info.vl_col * panel_info.vl_bpix) / 8 * panel_info.vl_row;
//#endif
 // S3C_VIDW00ADD0B0 = (unsigned int)(lcdbase) | S3C_VIDWxxADD0_VBANK_F((unsigned int)lcdbase);
  S3C_VIDW00ADD0B0 = virt_to_phys(lcdbase);
  S3C_VIDW00ADD0B1 = 0;
  S3C_VIDW01ADD0B0 = 0;
  S3C_VIDW01ADD0B1 = 0;
  S3C_VIDW02ADD0 = 0;
  S3C_VIDW03ADD0 = 0;
  S3C_VIDW04ADD0 = 0;

 // S3C_VIDW00ADD1B0 = S3C_VIDWxxADD1_VBASEL_F((unsigned int)(lcdbase) + fb_size);
  S3C_VIDW00ADD1B0 = virt_to_phys((unsigned int)(lcdbase) + fb_size);
  S3C_VIDW00ADD1B1 = 0;
  S3C_VIDW01ADD1B0 = 0;
  S3C_VIDW01ADD1B1 = 0;
  S3C_VIDW02ADD1 = 0;
  S3C_VIDW03ADD1 = 0;
  S3C_VIDW04ADD1 = 0;
  
  S3C_VIDW00ADD2 = S3C_VIDWxxADD2_OFFSIZE_F(0) | (S3C_VIDWxxADD2_PAGEWIDTH_F(panel_info.vl_col*panel_info.vl_bpix/8));
  S3C_VIDW01ADD2 = 0;
  S3C_VIDW02ADD2 = 0;
  S3C_VIDW03ADD2 = 0;
  S3C_VIDW04ADD2 = 0;
  //(12)WxKEYCONx:色键值寄存器
  //(13)WINxMAP:窗口颜色控制
  //(14)WPALCON:调色板控制寄存器
  //(15)WxPDATAxx:索引窗口调色板数据
  printf("\nFrameBuff:%08x",(unsigned int)lcdbase);
#if 1
  memset(lcdbase,0x55,fb_size);
#else
  pp = lcdbase;
  for(i=0;i< S3CFB_HRES * S3CFB_VRES;i++)
  {
    *pp = 0xf100;
    pp++; 
  }
#endif
  lcd_enable();
  S3C_WINCON0 |= S3C_WINCONx_ENWIN_F_ENABLE;
  printf("\n  LCD initialization Finished. \n");


}

/************************************************************************/
/* ** GENERIC Initialization Routines					*/
/************************************************************************/

int drv_lcd_init (void)
{
	struct stdio_dev lcddev;
	int rc;

	lcd_base = (void *)(gd->fb_base);

	lcd_line_length = (panel_info.vl_col * NBITS (panel_info.vl_bpix)) / 8;

	lcd_init (lcd_base);		/* LCD initialization */

	/* Device initialization */
	memset (&lcddev, 0, sizeof (lcddev));

	strcpy (lcddev.name, "lcd");
	lcddev.ext   = 0;			/* No extensions */
	lcddev.flags = DEV_FLAGS_OUTPUT;	/* Output only */
	lcddev.putc  = lcd_putc;		/* 'putc' function */
	lcddev.puts  = lcd_puts;		/* 'puts' function */

	rc = stdio_register (&lcddev);

	return (rc == 0) ? 1 : rc;
}

/*----------------------------------------------------------------------*/
static int do_lcd_clear(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	lcd_clear();
	return 0;
}

void lcd_clear(void)
{
#if LCD_BPP == LCD_MONOCHROME
	/* Setting the palette */
	lcd_initcolregs();

#elif LCD_BPP == LCD_COLOR8
	/* Setting the palette */
	/*lcd_setcolreg  (CONSOLE_COLOR_BLACK,       0,    0,    0);
	lcd_setcolreg  (CONSOLE_COLOR_RED,	0xFF,    0,    0);
	lcd_setcolreg  (CONSOLE_COLOR_GREEN,       0, 0xFF,    0);
	lcd_setcolreg  (CONSOLE_COLOR_YELLOW,	0xFF, 0xFF,    0);
	lcd_setcolreg  (CONSOLE_COLOR_BLUE,        0,    0, 0xFF);
	lcd_setcolreg  (CONSOLE_COLOR_MAGENTA,	0xFF,    0, 0xFF);
	lcd_setcolreg  (CONSOLE_COLOR_CYAN,	   0, 0xFF, 0xFF);
	lcd_setcolreg  (CONSOLE_COLOR_GREY,	0xAA, 0xAA, 0xAA);
	lcd_setcolreg  (CONSOLE_COLOR_WHITE,	0xFF, 0xFF, 0xFF);*/
#endif

#ifndef CONFIG_SYS_WHITE_ON_BLACK
	lcd_setfgcolor (CONSOLE_COLOR_WHITE);
	lcd_setbgcolor (CONSOLE_COLOR_BLACK);
#else
	lcd_setfgcolor (CONSOLE_COLOR_BLACK);
	lcd_setbgcolor (CONSOLE_COLOR_WHITE);
#endif	/* CONFIG_SYS_WHITE_ON_BLACK */

#ifdef	LCD_TEST_PATTERN
	test_pattern();
#else
	/* set framebuffer to background color */
	memset ((char *)lcd_base,
		COLOR_MASK(lcd_getbgcolor()),
		lcd_line_length*panel_info.vl_row);
#endif
	/* Paint the logo and retrieve LCD base address */
	//debug ("[LCD] Drawing the logo...\n");
	lcd_console_address = lcd_logo ();
	//lcd_console_address = (void *)lcd_base;

	console_col = 0;
	console_row = 0;
}

U_BOOT_CMD(
	cls,	1,	1,	do_lcd_clear,
	"clear screen",
	""
);

/*----------------------------------------------------------------------*/

static int lcd_init (void *lcdbase)
{
	/* Initialize the lcd controller */
	debug ("[LCD] Initializing LCD frambuffer at %p\n", lcdbase);

	lcd_ctrl_init (lcdbase);
	lcd_is_enabled = 1;
	lcd_clear();
	lcd_enable ();

	/* Initialize the console */
	console_col = 0;
#ifdef CONFIG_LCD_INFO_BELOW_LOGO
	console_row = 7 + BMP_LOGO_HEIGHT / VIDEO_FONT_HEIGHT;
#else
	console_row = 1;	/* leave 1 blank line below logo */
#endif

	return 0;
}


/************************************************************************/
/* ** ROM capable initialization part - needed to reserve FB memory	*/
/************************************************************************/
/*
 * This is called early in the system initialization to grab memory
 * for the LCD controller.
 * Returns new address for monitor, after reserving LCD buffer memory
 *
 * Note that this is running from ROM, so no write access to global data.
 */
ulong lcd_setmem (ulong addr)
{
	ulong size;
	int line_length = (panel_info.vl_col * NBITS (panel_info.vl_bpix)) / 8;

	debug ("LCD panel info: %d x %d, %d bit/pix\n",
		panel_info.vl_col, panel_info.vl_row, NBITS (panel_info.vl_bpix) );

	size = line_length * panel_info.vl_row;

	/* Round up to nearest full page */
	size = (size + (PAGE_SIZE - 1)) & ~(PAGE_SIZE - 1);

	/* Allocate pages for the frame buffer. */
	addr -= size;

	debug ("Reserving %ldk for LCD Framebuffer at: %08lx\n", size>>10, addr);

	return (addr);
}

/*----------------------------------------------------------------------*/

static void lcd_setfgcolor (int color)
{
	lcd_color_fg = color;
}

/*----------------------------------------------------------------------*/

static void lcd_setbgcolor (int color)
{
	lcd_color_bg = color;
}

/*----------------------------------------------------------------------*/

#ifdef	NOT_USED_SO_FAR
static int lcd_getfgcolor (void)
{
	return lcd_color_fg;
}
#endif	/* NOT_USED_SO_FAR */

/*----------------------------------------------------------------------*/

static int lcd_getbgcolor (void)
{
	return lcd_color_bg;
}

/*----------------------------------------------------------------------*/

void lcd_disable (void)
{
  S3C_VIDCON0 &= (~(S3C_VIDCON0_ENVID_ENABLE | S3C_VIDCON0_ENVID_F_ENABLE));
}

void lcd_enable (void)
{
  S3C_VIDCON0 |= (S3C_VIDCON0_ENVID_ENABLE | S3C_VIDCON0_ENVID_F_ENABLE);
}


void lcd_panel_disable(void)
{
  MIFPCON_REG |= SEL_BYPASS_MASK;
}
/************************************************************************/
/************************************************************************/
/* ** Chipset depending Bitmap / Logo stuff...                          */
/************************************************************************/
#ifdef CONFIG_LCD_LOGO
void bitmap_plot (int x, int y)
{
#ifdef CONFIG_ATMEL_LCD
	uint *cmap;
#else
	ushort *cmap;
#endif
	ushort i, j;
	uchar *bmap;
	uchar *fb;
	ushort *fb16;
#if defined(CONFIG_CPU_PXA)
	struct pxafb_info *fbi = &panel_info.pxa;
#elif defined(CONFIG_MPC823)
	volatile immap_t *immr = (immap_t *) CONFIG_SYS_IMMR;
	volatile cpm8xx_t *cp = &(immr->im_cpm);
#endif

	debug ("Logo: width %d  height %d  colors %d  cmap %d\n",
		BMP_LOGO_WIDTH, BMP_LOGO_HEIGHT, BMP_LOGO_COLORS,
		(int)(sizeof(bmp_logo_palette)/(sizeof(ushort))));

	bmap = &bmp_logo_bitmap[0];
	fb   = (uchar *)(lcd_base + y * lcd_line_length + x);

	if (NBITS(panel_info.vl_bpix) < 12) {
		/* Leave room for default color map */
#if defined(CONFIG_CPU_PXA)
		cmap = (ushort *)fbi->palette;
#elif defined(CONFIG_MPC823)
		cmap = (ushort *)&(cp->lcd_cmap[BMP_LOGO_OFFSET*sizeof(ushort)]);
#elif defined(CONFIG_ATMEL_LCD)
		cmap = (uint *) (panel_info.mmio + ATMEL_LCDC_LUT(0));
#else
		/*
		 * default case: generic system with no cmap (most likely 16bpp)
		 * We set cmap to the source palette, so no change is done.
		 * This avoids even more ifdef in the next stanza
		 */
		cmap = bmp_logo_palette;
#endif

		WATCHDOG_RESET();

		/* Set color map */
		for (i=0; i<(sizeof(bmp_logo_palette)/(sizeof(ushort))); ++i) {
			ushort colreg = bmp_logo_palette[i];
#ifdef CONFIG_ATMEL_LCD
			uint lut_entry;
#ifdef CONFIG_ATMEL_LCD_BGR555
			lut_entry = ((colreg & 0x000F) << 11) |
				    ((colreg & 0x00F0) <<  2) |
				    ((colreg & 0x0F00) >>  7);
#else /* CONFIG_ATMEL_LCD_RGB565 */
			lut_entry = ((colreg & 0x000F) << 1) |
				    ((colreg & 0x00F0) << 3) |
				    ((colreg & 0x0F00) << 4);
#endif
			*(cmap + BMP_LOGO_OFFSET) = lut_entry;
			cmap++;
#else /* !CONFIG_ATMEL_LCD */
#ifdef  CONFIG_SYS_INVERT_COLORS
			*cmap++ = 0xffff - colreg;
#else
			*cmap++ = colreg;
#endif
#endif /* CONFIG_ATMEL_LCD */
		}

		WATCHDOG_RESET();

		for (i=0; i<BMP_LOGO_HEIGHT; ++i) {
			memcpy (fb, bmap, BMP_LOGO_WIDTH);
			bmap += BMP_LOGO_WIDTH;
			fb   += panel_info.vl_col;
		}
	}
	else { /* true color mode */
		u16 col16;
		fb16 = (ushort *)(lcd_base + y * lcd_line_length + x);
		for (i=0; i<BMP_LOGO_HEIGHT; ++i) {
			for (j=0; j<BMP_LOGO_WIDTH; j++) {
				col16 = bmp_logo_palette[(bmap[j]-16)];
				fb16[j] =
					((col16 & 0x000F) << 1) |
					((col16 & 0x00F0) << 3) |
					((col16 & 0x0F00) << 4);
				}
			bmap += BMP_LOGO_WIDTH;
			fb16 += panel_info.vl_col;
		}
	}

	WATCHDOG_RESET();
}
#endif /* CONFIG_LCD_LOGO */

/*----------------------------------------------------------------------*/
#if defined(CONFIG_CMD_BMP) || defined(CONFIG_SPLASH_SCREEN)
/*
 * Display the BMP file located at address bmp_image.
 * Only uncompressed.
 */

#ifdef CONFIG_SPLASH_SCREEN_ALIGN
#define BMP_ALIGN_CENTER	0x7FFF
#endif

int lcd_display_bitmap(ulong bmp_image, int x, int y)
{
#if !defined(CONFIG_MCC200)
	ushort *cmap = NULL;
#endif
	ushort *cmap_base = NULL;
	ushort i, j;
	uchar *fb;
	bmp_image_t *bmp=(bmp_image_t *)bmp_image;
	uchar *bmap;
	ushort padded_line;
	unsigned long width, height, byte_width;
	unsigned long pwidth = panel_info.vl_col;
	unsigned colors, bpix, bmp_bpix;
#if defined(CONFIG_CPU_PXA)
	struct pxafb_info *fbi = &panel_info.pxa;
#elif defined(CONFIG_MPC823)
	volatile immap_t *immr = (immap_t *) CONFIG_SYS_IMMR;
	volatile cpm8xx_t *cp = &(immr->im_cpm);
#endif

	if (!((bmp->header.signature[0]=='B') &&
		(bmp->header.signature[1]=='M'))) {
		printf ("Error: no valid bmp image at %lx\n", bmp_image);
		return 1;
	}

	width = le32_to_cpu (bmp->header.width);
	height = le32_to_cpu (bmp->header.height);
	bmp_bpix = le16_to_cpu(bmp->header.bit_count);
	colors = 1 << bmp_bpix;

	bpix = NBITS(panel_info.vl_bpix);

	if ((bpix != 1) && (bpix != 8) && (bpix != 16)) {
		printf ("Error: %d bit/pixel mode, but BMP has %d bit/pixel\n",
			bpix, bmp_bpix);
		return 1;
	}

	/* We support displaying 8bpp BMPs on 16bpp LCDs */
	if (bpix != bmp_bpix && (bmp_bpix != 8 || bpix != 16)) {
		printf ("Error: %d bit/pixel mode, but BMP has %d bit/pixel\n",
			bpix,
			le16_to_cpu(bmp->header.bit_count));
		return 1;
	}

	debug ("Display-bmp: %d x %d  with %d colors\n",
		(int)width, (int)height, (int)colors);

#if !defined(CONFIG_MCC200)
	/* MCC200 LCD doesn't need CMAP, supports 1bpp b&w only */
	if (bmp_bpix == 8) {
#if defined(CONFIG_CPU_PXA)
		cmap = (ushort *)fbi->palette;
#elif defined(CONFIG_MPC823)
		cmap = (ushort *)&(cp->lcd_cmap[255*sizeof(ushort)]);
#elif !defined(CONFIG_ATMEL_LCD)
		cmap = cmap;//panel_info.cmap;
#endif

		cmap_base = cmap;

		/* Set color map */
		for (i=0; i<colors; ++i) {
			bmp_color_table_entry_t cte = bmp->color_table[i];
#if !defined(CONFIG_ATMEL_LCD)
			ushort colreg =
				( ((cte.red)   << 8) & 0xf800) |
				( ((cte.green) << 3) & 0x07e0) |
				( ((cte.blue)  >> 3) & 0x001f) ;
#ifdef CONFIG_SYS_INVERT_COLORS
			*cmap = 0xffff - colreg;
#else
			*cmap = colreg;
#endif
#if defined(CONFIG_MPC823)
			cmap--;
#else
			cmap++;
#endif
#else /* CONFIG_ATMEL_LCD */
			lcd_setcolreg(i, cte.red, cte.green, cte.blue);
#endif
		}
	}
#endif

	/*
	 *  BMP format for Monochrome assumes that the state of a
	 * pixel is described on a per Bit basis, not per Byte.
	 *  So, in case of Monochrome BMP we should align widths
	 * on a byte boundary and convert them from Bit to Byte
	 * units.
	 *  Probably, PXA250 and MPC823 process 1bpp BMP images in
	 * their own ways, so make the converting to be MCC200
	 * specific.
	 */
#if defined(CONFIG_MCC200)
	if (bpix==1)
	{
		width = ((width + 7) & ~7) >> 3;
		x     = ((x + 7) & ~7) >> 3;
		pwidth= ((pwidth + 7) & ~7) >> 3;
	}
#endif

	padded_line = (width&0x3) ? ((width&~0x3)+4) : (width);

#ifdef CONFIG_SPLASH_SCREEN_ALIGN
	if (x == BMP_ALIGN_CENTER)
		x = max(0, (pwidth - width) / 2);
	else if (x < 0)
		x = max(0, pwidth - width + x + 1);

	if (y == BMP_ALIGN_CENTER)
		y = max(0, (panel_info.vl_row - height) / 2);
	else if (y < 0)
		y = max(0, panel_info.vl_row - height + y + 1);
#endif /* CONFIG_SPLASH_SCREEN_ALIGN */

	if ((x + width)>pwidth)
		width = pwidth - x;
	if ((y + height)>panel_info.vl_row)
		height = panel_info.vl_row - y;

	bmap = (uchar *)bmp + le32_to_cpu (bmp->header.data_offset);
	fb   = (uchar *) (lcd_base +
		(y + height - 1) * lcd_line_length + x * bpix / 8);

	switch (bmp_bpix) {
	case 1: /* pass through */
	case 8:
		if (bpix != 16)
			byte_width = width;
		else
			byte_width = width * 2;

		for (i = 0; i < height; ++i) {
			WATCHDOG_RESET();
			for (j = 0; j < width; j++) {
				if (bpix != 16) {
#if defined(CONFIG_CPU_PXA) || defined(CONFIG_ATMEL_LCD)
					*(fb++) = *(bmap++);
#elif defined(CONFIG_MPC823) || defined(CONFIG_MCC200)
					*(fb++) = 255 - *(bmap++);
#endif
				} else {
					*(uint16_t *)fb = cmap_base[*(bmap++)];
					fb += sizeof(uint16_t) / sizeof(*fb);
				}
			}
			bmap += (width - padded_line);
			fb   -= (byte_width + lcd_line_length);
		}
		break;

#if defined(CONFIG_BMP_16BPP)
	case 16:
		for (i = 0; i < height; ++i) {
			WATCHDOG_RESET();
			for (j = 0; j < width; j++) {
#if defined(CONFIG_ATMEL_LCD_BGR555)
				*(fb++) = ((bmap[0] & 0x1f) << 2) |
					(bmap[1] & 0x03);
				*(fb++) = (bmap[0] & 0xe0) |
					((bmap[1] & 0x7c) >> 2);
				bmap += 2;
#else
				*(fb++) = *(bmap++);
				*(fb++) = *(bmap++);
#endif
			}
			bmap += (padded_line - width) * 2;
			fb   -= (width * 2 + lcd_line_length);
		}
		break;
#endif /* CONFIG_BMP_16BPP */

	default:
		break;
	};

	return (0);
}
#endif

static void *lcd_logo (void)
{
#ifdef CONFIG_SPLASH_SCREEN
	char *s;
	ulong addr;
	static int do_splash = 1;

	if (do_splash && (s = getenv("splashimage")) != NULL) {
		int x = 0, y = 0;
		do_splash = 0;

		addr = simple_strtoul (s, NULL, 16);
#ifdef CONFIG_SPLASH_SCREEN_ALIGN
		if ((s = getenv ("splashpos")) != NULL) {
			if (s[0] == 'm')
				x = BMP_ALIGN_CENTER;
			else
				x = simple_strtol (s, NULL, 0);

			if ((s = strchr (s + 1, ',')) != NULL) {
				if (s[1] == 'm')
					y = BMP_ALIGN_CENTER;
				else
					y = simple_strtol (s + 1, NULL, 0);
			}
		}
#endif /* CONFIG_SPLASH_SCREEN_ALIGN */

#ifdef CONFIG_VIDEO_BMP_GZIP
		bmp_image_t *bmp = (bmp_image_t *)addr;
		unsigned long len;

		if (!((bmp->header.signature[0]=='B') &&
		      (bmp->header.signature[1]=='M'))) {
			addr = (ulong)gunzip_bmp(addr, &len);
		}
#endif

		if (lcd_display_bitmap (addr, x, y) == 0) {
			return ((void *)lcd_base);
		}
	}
#endif /* CONFIG_SPLASH_SCREEN */

#ifdef CONFIG_LCD_LOGO
	bitmap_plot (0, 0);
#endif /* CONFIG_LCD_LOGO */

#ifdef CONFIG_LCD_INFO
	console_col = LCD_INFO_X / VIDEO_FONT_WIDTH;
	console_row = LCD_INFO_Y / VIDEO_FONT_HEIGHT;
	lcd_show_board_info();
#endif /* CONFIG_LCD_INFO */

#if defined(CONFIG_LCD_LOGO) && !defined(CONFIG_LCD_INFO_BELOW_LOGO)
	return ((void *)((ulong)lcd_base + BMP_LOGO_HEIGHT * lcd_line_length));
#else
	return ((void *)lcd_base);
#endif /* CONFIG_LCD_LOGO && !CONFIG_LCD_INFO_BELOW_LOGO */
}

/************************************************************************/
/************************************************************************/

#endif /* CONFIG_LCD */
