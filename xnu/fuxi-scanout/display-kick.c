extern int fuxi_gpu_scanout_owned(void);
#if defined(FUXI_SALAMI)
/*
 * fuxikick (salami only, default 0): 0 never touch MDSS (dsb only); 1 read
 * INTF1 autorefresh once (first kick) and START CTL0 when it is off; 2 Fuxi
 * behaviour (autorefresh read every kick); 3 never read INTF1, CTL0 START only.
 * Any other value is 0. Parsed at the first kick, so fbcon_probe stays as is;
 * a race between CPUs on the first kick parses (or reads) the same value twice.
 */
static int fuxi_kick_mode = -1, fuxi_kick_ar = -1;
#endif
void fuxi_display_kick(void)
{
    /* GPU-SCANOUT owns manual START until reset; console must stay muted. */
    if (fuxi_gpu_scanout_owned()) return;
#if defined(FUXI_SALAMI)
    if (fuxi_kick_mode < 0) {
        uint32_t fuxikick = 0;
        PE_parse_boot_argn("fuxikick", &fuxikick, sizeof(fuxikick));
        fuxi_kick_mode = fuxikick <= 3 ? (int)fuxikick : 0;
    }
    if (fuxi_kick_mode == 1 && fuxi_kick_ar < 0)
        fuxi_kick_ar = fuxi_intf1 == NULL || (fuxi_intf1[0x2b4 / 4] & 0x80000000u);
    if (fuxi_kick_mode == 0 || (fuxi_kick_mode == 1 && fuxi_kick_ar) ||
        (fuxi_kick_mode == 2 && (fuxi_intf1 == NULL || (fuxi_intf1[0x2b4 / 4] & 0x80000000u)))) {
        __asm__ volatile("dsb sy" ::: "memory");
        return;
    }
#else
    if (fuxi_intf1 == NULL || (fuxi_intf1[0x2b4 / 4] & 0x80000000u)) {
        __asm__ volatile("dsb sy" ::: "memory");
        return;
    }
#endif
    if (fuxi_ctl0 != NULL && !(fuxi_ctl0[0x1c / 4] & 1)) {
        __asm__ volatile("dsb sy" ::: "memory");
        fuxi_ctl0[0x1c / 4] = 1;
        __asm__ volatile("dsb sy" ::: "memory");
    }
}
