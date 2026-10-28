/* Salami boot logo hand-off text (FUXI_BOOT_LOGO loader builds, loader.c
 * boot_logo_args). Plain C with no loader dependency, so a host test can run
 * the same code. Every access is a byte store or load: with the MMU off the
 * command line is Device memory. */
#pragma once
#include <stdint.h>

enum {
 BOOT_LOGO_ARGS_OK,
 BOOT_LOGO_ARGS_UNTERMINATED, /* no NUL in the cap bytes: left alone */
 BOOT_LOGO_ARGS_PRESENT,      /* already names sweetlogo_: XNU takes the first match */
 BOOT_LOGO_ARGS_FULL,         /* the text would not fit in cap - 1 characters */
};

/* Puts s at out[n...] (counts only when out is 0); returns the new position. */
static inline unsigned boot_logo_put_str(volatile char *out, unsigned n, const char *s)
{
 for (; *s; s++, n++)
  if (out) out[n] = *s;
 return n;
}

/* Puts v in radix 10 or 16 (lower case, no leading zeros) at out[n...]. */
static inline unsigned boot_logo_put_num(volatile char *out, unsigned n, uint64_t v, unsigned radix)
{
 unsigned digits = 1;
 for (uint64_t t = v; t >= radix; t /= radix) digits++;
 for (unsigned i = digits; i-- > 0; v /= radix)
  if (out) out[n + i] = "0123456789abcdef"[v % radix];
 return n + digits;
}

/* " sweetlogo_pa=0x<pa> sweetlogo_w=<w> sweetlogo_h=<h>" at out (counts only
 * when out is 0); returns its length. */
static inline unsigned boot_logo_args_text(volatile char *out, uint64_t pa, unsigned w, unsigned h)
{
 unsigned n = boot_logo_put_str(out, 0, " sweetlogo_pa=0x");
 n = boot_logo_put_num(out, n, pa, 16);
 n = boot_logo_put_str(out, n, " sweetlogo_w=");
 n = boot_logo_put_num(out, n, w, 10);
 n = boot_logo_put_str(out, n, " sweetlogo_h=");
 return boot_logo_put_num(out, n, h, 10);
}

/* Appends that text to the NUL-terminated command line cl of cap bytes (at
 * most cap - 1 characters, XNU's BOOT_LINE_LENGTH rule). Returns
 * BOOT_LOGO_ARGS_OK, or another BOOT_LOGO_ARGS_* with cl unchanged. */
static inline int boot_logo_args_append(volatile char *cl, unsigned cap, uint64_t pa, unsigned w, unsigned h)
{
 static const char key[] = "sweetlogo_";
 unsigned len = 0;
 while (len < cap && cl[len]) len++;
 if (len >= cap) return BOOT_LOGO_ARGS_UNTERMINATED;
 for (unsigned i = 0; i + sizeof(key) - 1 <= len; i++) {
  unsigned k = 0;
  while (key[k] && cl[i + k] == key[k]) k++;
  if (!key[k]) return BOOT_LOGO_ARGS_PRESENT;
 }
 const unsigned n = boot_logo_args_text(0, pa, w, h);
 if (n > cap - 1 - len) return BOOT_LOGO_ARGS_FULL;
 boot_logo_args_text(cl + len, pa, w, h);
 cl[len + n] = 0;
 return BOOT_LOGO_ARGS_OK;
}
