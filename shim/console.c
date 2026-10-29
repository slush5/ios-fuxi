/*
 * Minimal text console on the ABL splash framebuffer. With the MMU off
 * every access is Device memory, so stay aligned (-mstrict-align).
 */
#include "console.h"
#include "font.h"

#define SCALE		3
#define CH_W		(8 * SCALE)
#define CH_H		(16 * SCALE)
#define MARGIN		24
#define LABEL_COLS	8

#if defined(FUXI_SALAMI) && defined(FUXI_SALAMI_KICK) && \
    (!defined(FUXI_SALAMI_MDSS_STAGE) || FUXI_SALAMI_MDSS_STAGE < 9)
#error "FUXI_SALAMI_KICK reads INTF1 and writes CTL0 START: build it only with FUXI_SALAMI_MDSS_STAGE=9"
#endif

volatile uint32_t *con_fb = (volatile uint32_t *)FB_BASE;
static uint32_t con_bg;
static int row;
/* Salami (OnePlus 11): no kick. A CTL0 START write is unproven there
 * (display_salami_plan.md P-1, stage S9), and the loader console runs inside
 * loader_main with no USB report to attribute a hang. INTF1 +0x2b4 (and
 * +0x2b0/+0x298) was read safely by probe stage S5 on 2026-10-28 and by the
 * XNU tegate calibration (1169 samples, 431 s session) on 2026-10-29.
 * -DFUXI_SALAMI_KICK restores the Fuxi kick once S9 has passed. */
static void kick(void)
{
#if defined(FUXI) && (!defined(FUXI_SALAMI) || defined(FUXI_SALAMI_KICK))
 if (!con_fb) return;
 /* ABL's INTF1 autorefresh already sends every frame; a START on top of it
  * corrupts the DSC stream (ar-01). Kick only when autorefresh is off. */
 if (*(volatile uint32_t *)0xae362b4 & 0x80000000u) return;
 volatile uint32_t *start=(volatile uint32_t *)0xae1601c;
 __asm__ volatile("dsb sy" ::: "memory");
 if (!(*start & 1)) *start=1;
 __asm__ volatile("dsb sy" ::: "memory");
#endif
}

void con_clear(uint32_t bg)
{
 if (!con_fb) return;
	for (int y = 0; y < FB_H; y++) {
		volatile uint32_t *line = con_fb + y * (FB_STRIDE / 4);
		for (int x = 0; x < FB_W; x++)
			line[x] = bg;
	}
	con_bg = bg;
	row = 0;
}

static void put_char(int col, char ch, uint32_t fg)
{
 if (!con_fb) return;
	if (ch < 0x20 || ch > 0x7e)
		ch = '?';
	const unsigned char *g = font8x16[ch - 0x20];
	int x0 = MARGIN + col * CH_W, y0 = MARGIN + row * CH_H;
	if (x0 + CH_W > FB_W || y0 + CH_H > FB_H)
		return;
	for (int gy = 0; gy < 16; gy++)
		for (int gx = 0; gx < 8; gx++) {
			uint32_t c = (g[gy] & (0x80 >> gx)) ? fg : con_bg;
			for (int sy = 0; sy < SCALE; sy++) {
				volatile uint32_t *p = con_fb + (y0 + gy * SCALE + sy) * (FB_STRIDE / 4)
						       + x0 + gx * SCALE;
				for (int sx = 0; sx < SCALE; sx++)
					p[sx] = c;
			}
		}
}

static int put_str(int col, const char *s, uint32_t fg)
{
	while (*s)
		put_char(col++, *s++, fg);
	return col;
}

static void fmt_hex(char *buf, uint64_t v, int digits)
{
	static const char hex[] = "0123456789ABCDEF";
	for (int i = 0; i < digits; i++)
		buf[i] = hex[(v >> (4 * (digits - 1 - i))) & 0xf];
	buf[digits] = 0;
}

void con_skip(void)
{
	row++;
	kick();
}

void con_str(const char *label, const char *s)
{
	put_str(0, label, YELLOW);
	put_str(LABEL_COLS, s, WHITE);
	row++;
	kick();
}

void con_hex(const char *label, uint64_t v)
{
	char buf[17];
	fmt_hex(buf, v, 16);
	con_str(label, buf);
}

void con_hex2(const char *label, const char *a, uint32_t va, const char *b, uint32_t vb)
{
	char buf[9];
	int col = put_str(0, label, YELLOW);
	col = col < LABEL_COLS ? LABEL_COLS : col + 1;
	col = put_str(col, a, YELLOW);
	fmt_hex(buf, va, 8);
	col = put_str(col, buf, WHITE);
	if (b) {
		col = put_str(col + 1, b, YELLOW);
		fmt_hex(buf, vb, 8);
		put_str(col, buf, WHITE);
	}
	row++;
	kick();
}
