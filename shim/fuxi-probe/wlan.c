/* Lane-wlan (milestone 1): read-only WCN7850 / PCIe0 / BT-UART snapshot on
 * request 0x5c, value 0x574c ('WL'), index 1. Included from probe.c after
 * gpu.c (uses probe_safe_rd32). No register writes of any kind.
 *
 * Sources (see docs-lanes/wlan-bt.md):
 *  GCC offsets      research/gcc-kalama.c (base 0x100000)
 *  TCSR clkref      tcsrcc-sm8550.c tcsr_pcie_0_clkref_en 0x15100 (base 0x1fc0000)
 *  PCIe0 regs       live DT qcom,pcie@1c00000 (parf 0x1c00000, phy 0x1c06000,
 *                   dm_core 0x60000000), mainline pcie-qcom.c PARF offsets
 *  GDSCs            live DT qcom,gdsc@16b004 / @16c000, collapse vote 0x152020
 *  BT UART          live DT qcom,qup_uart@898000 (QUP2 SE6), wrapper 0x8c0000
 *  GPIOs            cnss: wlan-en 80, bt-en 81, sw-ctrl 82, wlan-sw-ctrl 83,
 *                   uart 76-79; pcie0 perst 94, clkreq 95, wake 96
 *  cmd-db           reserved aop_cmd_db_region 0x81c60000 + 0x20000
 *
 * Gating (touch-lane lesson: an unclocked peripheral read hangs the NoC and
 * resets the SoC; probe_safe_rd32 only recovers synchronous aborts):
 *  PARF   read only when pcie_0 GDSC reports on and cfg_ahb CBCR is running
 *  PHY    additionally phy GDSC on and aux CBCR running
 *  DBI    additionally slv_axi CBCR running
 *  SE6    read only when wrap2 m_ahb CBCR is running
 */
#define WL_GCC    0x00100000UL
#define WL_TCSR   0x01fc0000UL
#define WL_TLMM   TS_TLMM
#define WL_PARF   0x01c00000ULL
#define WL_PHY    0x01c06000ULL
#define WL_DBI    0x60000000ULL
#define WL_QUP2   0x008c0000ULL
#define WL_SE6    0x00898000ULL
#define WL_CMDDB  0x81c60000UL
#define WL_CMDDB_BYTES 0x20000u
#define WL_MAGIC  0x574c0001u

static uint32_t wlan_report[128];

static uint32_t wl_srd(uint64_t pa, unsigned *faults)
{
 uint32_t f = 0, v = probe_safe_rd32(pa, &f);
 if (f) (*faults)++;
 return v;
}
static int wl_on(uint32_t cbcr) { return !(cbcr & 0x80000000u); }
static uint8_t wl_cdb8(uint32_t off) { return (uint8_t)(usb_rd(WL_CMDDB, off & ~3u) >> ((off & 3u) * 8)); }
static uint32_t wl_cdb16(uint32_t off) { return wl_cdb8(off) | (uint32_t)wl_cdb8(off + 1) << 8; }

/* cmd-db resource address by 8-byte name; 0 when absent or malformed. */
static uint32_t wl_cmddb_addr(const char *name)
{
 for (unsigned s = 0; s < 8; s++) {
  uint32_t row = 8 + s * 16;
  if (!wl_cdb16(row)) break;
  uint32_t at = 144 + wl_cdb16(row + 2), n = wl_cdb16(row + 6);
  if (at > WL_CMDDB_BYTES || n > (WL_CMDDB_BYTES - at) / 24) return 0;
  for (uint32_t i = 0; i < n; i++) {
   uint32_t e = at + i * 24; unsigned j;
   for (j = 0; j < 8 && name[j]; j++) if (wl_cdb8(e + j) != (uint8_t)name[j]) break;
   if (j < 8 && (name[j] || wl_cdb8(e + j))) continue;
   return wl_cdb16(e + 16) | wl_cdb16(e + 18) << 16;
  }
 }
 return 0;
}

static void wlan_snapshot(void)
{
 uint32_t *r = wlan_report;
 unsigned faults = 0, n;
 memset(r, 0, sizeof(wlan_report));
 r[0] = WL_MAGIC; r[2] = 1;
 /* GCC branch votes and GDSC collapse vote (always-on GCC block). */
 r[4] = usb_rd(WL_GCC, 0x52000); r[5] = usb_rd(WL_GCC, 0x52008);
 r[6] = usb_rd(WL_GCC, 0x52010); r[7] = usb_rd(WL_GCC, 0x52020);
 static const uint32_t pcie_cbcr[] = {0x6b020, 0x6b02c, 0x6b038, 0x6b03c, 0x6b048,
                                      0x6b01c, 0x6b054, 0x1003c, 0x1004c, 0x10030};
 for (n = 0; n < 10; n++) r[8 + n] = usb_rd(WL_GCC, pcie_cbcr[n]);
 r[19] = usb_rd(WL_GCC, 0x6b004); r[20] = usb_rd(WL_GCC, 0x6b008);
 r[21] = usb_rd(WL_GCC, 0x6c000); r[22] = usb_rd(WL_GCC, 0x6c004);
 static const uint32_t bcr[] = {0x6b000, 0x6c01c, 0x6c014, 0x6c020, 0x6c028};
 for (n = 0; n < 5; n++) r[23 + n] = usb_rd(WL_GCC, bcr[n]);
 r[28] = usb_rd(WL_GCC, 0x6b070);
 r[29] = usb_rd(WL_GCC, 0x6b074); r[30] = usb_rd(WL_GCC, 0x6b078);
 r[31] = usb_rd(WL_GCC, 0x6b058); r[32] = usb_rd(WL_GCC, 0x6b05c);
 r[33] = usb_rd(WL_TCSR, 0x15100);
 r[34] = usb_rd(WL_GCC, 0x233cc); r[35] = usb_rd(WL_GCC, 0x233d0);
 r[36] = usb_rd(WL_GCC, 0x233d4); r[37] = usb_rd(WL_GCC, 0x23004);
 r[38] = usb_rd(WL_GCC, 0x1e754);
 /* SE6 RCG: CMD, CFG, DFSR, 8 DFS perf levels with M and N. */
 r[39] = usb_rd(WL_GCC, 0x1e760); r[40] = usb_rd(WL_GCC, 0x1e764);
 r[41] = usb_rd(WL_GCC, 0x1e774);
 for (n = 0; n < 8; n++) {
  r[42 + n] = usb_rd(WL_GCC, 0x1e77c + 4 * n);
  r[50 + n] = usb_rd(WL_GCC, 0x1e7bc + 4 * n);
  r[58 + n] = usb_rd(WL_GCC, 0x1e7fc + 4 * n);
 }
 /* TLMM CFG and IN_OUT: gpio76..83, then 94..96. */
 for (n = 0; n < 8; n++) {
  r[66 + n] = usb_rd(WL_TLMM + 0x1000u * (76 + n), 0);
  r[74 + n] = usb_rd(WL_TLMM + 0x1000u * (76 + n), 4);
 }
 for (n = 0; n < 3; n++) {
  r[82 + n] = usb_rd(WL_TLMM + 0x1000u * (94 + n), 0);
  r[85 + n] = usb_rd(WL_TLMM + 0x1000u * (94 + n), 4);
 }
 r[88] = usb_rd(RH_DRV_PA, 0); r[89] = usb_rd(RH_DRV_PA, 0xc);
 /* cmd-db: header magic, then the RPMh addresses the later stages would vote. */
 r[90] = usb_rd(WL_CMDDB, 4);
 if (r[90] == 0x0c0330db) {
  static const char names[10][8] = {"ldob15", "smpe4", "smpg2", "smpg4", "smpg5",
                                    "smpg6", "ldoe1", "ldoe3", "clka1", "vrm.wlan"};
  for (n = 0; n < 10; n++) {
   char nm[9]; memcpy(nm, names[n], 8); nm[8] = 0;
   r[91 + n] = wl_cmddb_addr(nm);
  }
 }
 /* Gated PCIe0 reads. */
 int core_on = (r[19] & 0x80000000u) && wl_on(r[10]);
 if (core_on) {
  r[101] |= 1;
  r[102] = wl_srd(WL_PARF + 0x1b0, &faults);  /* LTSSM */
  r[103] = wl_srd(WL_PARF + 0x40, &faults);   /* PHY_CTRL */
  r[104] = wl_srd(WL_PARF + 0x1000, &faults); /* DEVICE_TYPE */
  r[105] = wl_srd(WL_PARF + 0x224, &faults);  /* INT_ALL_STATUS */
  r[106] = wl_srd(WL_PARF + 0x350, &faults);  /* DBI_BASE_ADDR_V2 */
  r[107] = wl_srd(WL_PARF + 0x634, &faults);  /* ATU_BASE_ADDR */
  if ((r[21] & 0x80000000u) && wl_on(r[11])) {
   r[101] |= 2;
   r[108] = wl_srd(WL_PHY + 0x214, &faults);  /* PCS status (bit6 PHYSTATUS) */
   r[109] = wl_srd(WL_PHY + 0x240, &faults);  /* power down control */
  }
  if (wl_on(r[8])) {
   r[101] |= 4;
   r[110] = wl_srd(WL_DBI + 0x0, &faults);    /* RC vendor/device */
   r[111] = wl_srd(WL_DBI + 0x18, &faults);   /* bus numbers */
   r[112] = wl_srd(WL_DBI + 0x80, &faults);   /* PCIe cap LNKCTL/LNKSTA */
  }
 }
 /* Gated QUP2 SE6 reads. */
 if (wl_on(r[34])) {
  r[101] |= 8;
  r[113] = wl_srd(WL_QUP2 + 0x4, &faults);    /* QUP HW version */
  r[114] = wl_srd(WL_SE6 + 0x68, &faults);    /* FW_REVISION_RO (proto 15:8, UART 2) */
  r[115] = wl_srd(WL_SE6 + 0x40, &faults);    /* GENI_STATUS */
  r[116] = wl_srd(WL_SE6 + 0x48, &faults);    /* M_CLK_CFG */
  r[117] = wl_srd(WL_SE6 + 0x7c, &faults);    /* CLK_SEL */
  r[118] = wl_srd(WL_SE6 + 0x908, &faults);   /* IOS (bit1 low = CTS asserted) */
  r[119] = wl_srd(WL_SE6 + 0xe24, &faults);   /* HW_PARAM_0 */
  r[120] = wl_srd(WL_SE6 + 0x258, &faults);   /* DMA_MODE_EN */
 }
 r[124] = faults;
 r[125] = probe_fault_esr;
 r[3] = 1;   /* snapshot complete */
}

/* ===================================================================
 * Milestone 3: gated WCN7850 power-up + PCIe0 link + endpoint config read.
 * Request 0x5c 'WL', index 0x300 + limit (limit 1..13 = last step to run);
 * index 0x3ff re-reads the M3 report. Steps already completed in this boot
 * are skipped; after any failure every later request only returns the
 * report (reboot to retry). Each step stores its number in r[2] before it
 * touches hardware and paints "WLAN M3 STEP nn" on the scanout when the
 * probe-03 geometry is present, so a hang or reset can be attributed (or
 * bisected by running increasing limits).
 *
 * Write sources (docs-lanes/wlan-bt.md section 7 has the full table):
 *  2  RPMh VRM votes, rail order = pwrseq-qcom-wcn.c wcn7850 (vdd, vddio,
 *     vddaon, vdddig, vddrfa1p2, vddrfa1p8); voltages = live DT
 *     chip_cfg@1 qcom,vdd-wlan-*-config inside each regulator's live-DT
 *     min/max; addresses = cmd-db, checked against the OrangeFox dump first.
 *  3  WLAN_EN gpio80 = live DT cnss_wlan_en_active (gpio, pull-up, 16 mA,
 *     output high); 50 ms = pwrseq wcn7850 pwup_delay_ms.
 *  4  PHY rails ldoe1 880 mV / ldoe3 1200 mV = live DT vreg-0p9/vreg-1p2,
 *     order = qmp_phy_vreg_l (vdda-phy, vdda-pll).
 *  5  PERST# gpio94 low (pcie0_perst_default, qcom_ep_reset_assert),
 *     CLKREQ# gpio95 func 1 pull-up, WAKE# gpio96 input pull-up, 2 mA.
 *  6  GDSCs: gcc-sm8550.c pcie_0_gdsc / pcie_0_phy_gdsc (VOTABLE,
 *     POLL_CFG_GDSCR, RETAIN_FF_ENABLE) in gdsc.c gdsc_enable order.
 *  7  Clocks: sm8550.dtsi pcie0 + pcie0_phy clock lists, rchng 100 MHz
 *     (assigned-clock-rates), aux 19.2 MHz (live DT clock-frequency).
 *  8  pcie-qcom.c qcom_pcie_init_2_7_0 (BCR pulse, PARF programming).
 *  9  phy-qcom-qmp-pcie.c qmp_pcie_init + qmp_pcie_power_on with the
 *     sm8550_qmp_gen3x2 tables (identical to the live-DT qcom,phy-sequence
 *     body plus RXEQEVAL_TIME 0x6f4).
 * 10  qcom_ep_reset_deassert: 100 ms, PERST# high, 1 ms.
 * 11  qcom_pcie_2_3_2_ltssm_enable; dw_pcie_wait_for_link (DLLLA), 1 s bound.
 * 12  dw_pcie_setup_rc bus numbers; dw_pcie_prog_outbound_atu CFG0 region 0;
 *     endpoint 01:00.0 dword 0 (expect 0x110717cb), CRS/all-ones retry 1 s.
 * 13  Standard BAR sizing on the endpoint (all-ones write, read, restore);
 *     the command register is untouched, so decode stays disabled.
 */
#define WL3_MAGIC 0x574c0003u
#define WL3_STEPS 13u
#if defined(FUXI_SALAMI)
/* K-W1, OnePlus 11 (salami): step 1 expects these cmd-db addresses, looked
 * up by name in wlan_snapshot names[] order (ldob15 smpe4 smpg2 smpg4 smpg5
 * smpg6 ldoe1 ldoe3) and measured on salami (salami-port measure/cmd_db.txt).
 * ldob15 is 0x40b00 here; the Fuxi ldob15 value 0x40a00 is salami ldob12, the
 * panel vddio, and is refused outright. Step 2 order and mV are salami live
 * DT chip_cfg@1 (S4E 950 mV). The salami loader STALLs request 0x5c, so
 * only the XNU copy (bsd/dev/wlan.c, patch 0099) runs these steps on salami. */
#define WL_SALAMI_LDOB12 0x40a00u
static const uint32_t wl_salami_rail[8] = {0x40b00, 0x40200, 0x40400, 0x40500, 0x44000, 0x40600, 0x43900, 0x43a00};
#endif
static uint32_t wl3_report[128];
static unsigned wl3_done, wl3_failed;
static unsigned wl3_faults;

static void wl_wr(uintptr_t base, uint32_t off, uint32_t v) { ts_wr(base, off, v); }
static void wl_set(uintptr_t base, uint32_t off, uint32_t bits) { wl_wr(base, off, usb_rd(base, off) | bits); }
static void wl_clr(uintptr_t base, uint32_t off, uint32_t bits) { wl_wr(base, off, usb_rd(base, off) & ~bits); }
static uint32_t wl_prd(uint64_t pa) { return wl_srd(pa, &wl3_faults); }
static void wl_pwr(uint64_t pa, uint32_t v) { if (probe_safe_wr32(pa, v)) wl3_faults++; }
static uint64_t wl_now_us(void) { return rsr("cntpct_el0") * 1000000ull / rsr("cntfrq_el0"); }

static void wl_mark(unsigned step)
{
 wl3_report[2] = step;
 __asm__ volatile("dsb sy" ::: "memory");
#ifdef FUXI_WLAN_XNU
 /* XNU copy (bsd/dev/wlan.c via xnu/patches/0099): log, no scanout paint. */
 printf("FUXI WLAN: step %u\n", step);
#else
 if (!probe_paint_ok()) return;
 volatile uint32_t *fb = (volatile uint32_t *)0xb8000000UL;
 char text[] = "WLAN M3 STEP 00";
 text[13] = (char)('0' + step / 10); text[14] = (char)('0' + step % 10);
 for (unsigned y = 1700; y < 1780; y++)
  for (unsigned x = 0; x < 1080; x++) fb[y * 1080 + x] = 0xff402010;
 for (unsigned c = 0; text[c]; c++) {
  const unsigned char *glyph = font8x16[(unsigned char)text[c] - 32];
  for (unsigned gy = 0; gy < 16; gy++)
   for (unsigned gx = 0; gx < 8; gx++)
    if (glyph[gy] & (0x80u >> gx))
     for (unsigned sy = 0; sy < 4; sy++)
      for (unsigned sx = 0; sx < 4; sx++)
       fb[(1708 + gy * 4 + sy) * 1080 + 48 + c * 32 + gx * 4 + sx] = 0xffffffff;
 }
 __asm__ volatile("dsb sy" ::: "memory");
#endif
}

/* RPMh VRM: +0 voltage (mV), +4 enable. Linux rpmh-regulator sends the
 * voltage before the enable. 0 when both commands completed. */
static uint32_t wl_vrm(uint32_t addr, uint32_t mv)
{
 uint32_t e = rh_send(0, addr, mv);
 if (!e) e = rh_send(0, addr + 4, 1) << 8;
 return e;
}

static int wl_poll_clr(uintptr_t base, uint32_t off, uint32_t mask, unsigned us)
{
 for (unsigned i = 0; i < us; i++) { if (!(usb_rd(base, off) & mask)) return 1; delay(1); }
 return !(usb_rd(base, off) & mask);
}

/* GDSC enable per gdsc.c (VOTABLE + POLL_CFG_GDSCR + RETAIN_FF_ENABLE);
 * 1500 us = live DT qcom,gds-timeout. */
static uint32_t wl_gdsc(uint32_t gdscr, uint32_t vote_bit, uint32_t *polls)
{
 wl_clr(WL_GCC, 0x52020, vote_bit);
 unsigned i;
 for (i = 0; i < 1500; i++) { if (usb_rd(WL_GCC, gdscr + 4) & (1u << 16)) break; delay(1); }
 *polls = i;
 if (!(usb_rd(WL_GCC, gdscr + 4) & (1u << 16))) return 1;
 delay(1);
 wl_set(WL_GCC, gdscr, 1u << 11);
 return 0;
}

/* clk-rcg2: CFG (hid 4:0, src 10:8, mode 13:12) then CMD_UPDATE, 500 us. */
static int wl_rcg(uint32_t rcgr, uint32_t cfg)
{
 if ((usb_rd(WL_GCC, rcgr + 4) & 0x371fu) == cfg) return 1;
 wl_wr(WL_GCC, rcgr + 4, (usb_rd(WL_GCC, rcgr + 4) & ~0x371fu) | cfg);
 wl_set(WL_GCC, rcgr, 1);
 return wl_poll_clr(WL_GCC, rcgr, 1, 500);
}

/* Branch is running if CLK_OFF is clear; a branch under hardware clock
 * gating (CBCR bit 1) may legitimately read OFF at idle. */
static int wl_branch_ok(uint32_t cbcr_off)
{
 for (unsigned i = 0; i < 500; i++) {
  uint32_t v = usb_rd(WL_GCC, cbcr_off);
  if (!(v & 0x80000000u) || (v & 2u)) return 1;
  delay(1);
 }
 return 0;
}

/* sm8550_qmp_gen3x2 serdes/tx/rx/pcs/pcs_misc tables as absolute offsets
 * from the PHY base (qmp_pcie_offsets_v5). Equal to the live-DT
 * qcom,phy-sequence body; 0x6f4 = RXEQEVAL_TIME is mainline-only. */
static const uint16_t wl_phy_tbl[][2] = {
 {0xc0,0x1},{0xcc,0x62},{0xd0,0x2},{0x60,0xf8},{0x64,0x1},{0x0,0x93},{0x4,0x1},{0xe0,0x90},
 {0xe4,0x82},{0xf4,0x7},{0x70,0x2},{0x10,0x2},{0x74,0x16},{0x14,0x16},{0x78,0x36},{0x18,0x36},
 {0x110,0x8},{0xbc,0xa},{0x120,0x42},{0x80,0x4},{0x84,0xd},{0x20,0xa},{0x24,0x1a},{0x88,0x41},
 {0x28,0x34},{0x90,0xab},{0x94,0xaa},{0x98,0x1},{0x30,0x55},{0x34,0x55},{0x38,0x1},{0x140,0x14},
 {0x164,0x34},{0x3c,0x1},{0x1c,0x4},{0x174,0x16},{0x1bc,0xf},{0x170,0xa0},
 {0x11a4,0x38},{0x10dc,0x11},{0x1160,0xbf},{0x1164,0xbf},{0x1168,0xb7},{0x116c,0xea},{0x115c,0x3f},
 {0x1174,0x5c},{0x1178,0x9c},{0x117c,0x1a},{0x1180,0x89},{0x1170,0xdc},{0x1188,0x94},{0x118c,0x5b},
 {0x1190,0x1a},{0x1194,0x89},{0x10cc,0x0},{0x1008,0x9},{0x1014,0x5},{0x104c,0x8},{0x1050,0x8},
 {0x10d8,0xf},{0x1118,0x1c},{0x10f8,0x7},{0x11f8,0x8},
 {0xe84,0x15},{0xe90,0x3f},{0xee4,0x2},{0xe40,0x6},{0xe3c,0x18},
 {0x19a4,0x38},{0x18dc,0x11},{0x1960,0xbf},{0x1964,0xbf},{0x1968,0xb7},{0x196c,0xea},{0x195c,0x3f},
 {0x1974,0x5c},{0x1978,0x9c},{0x197c,0x1a},{0x1980,0x89},{0x1970,0xdc},{0x1988,0x94},{0x198c,0x5b},
 {0x1990,0x1a},{0x1994,0x89},{0x18cc,0x0},{0x1808,0x9},{0x1814,0x5},{0x184c,0x8},{0x1850,0x8},
 {0x18d8,0xf},{0x1918,0x1c},{0x18f8,0x7},{0x19f8,0x8},
 {0x1684,0x15},{0x1690,0x3f},{0x16e4,0x2},{0x1640,0x6},{0x163c,0x18},
 {0x2dc,0x5},{0x388,0x77},{0x398,0xb},{0x3e0,0xf},{0x3d0,0x8c},
 {0x6a4,0x1e},{0x6f4,0x27},{0x60c,0x1d},{0x614,0x7},{0x620,0xc1},{0x694,0x0},
};

/* PCI Express capability offset in the RC's DBI; 0 if absent. */
static uint32_t wl_pcie_cap(void)
{
 uint32_t p = wl_prd(WL_DBI + 0x34) & 0xfc;
 for (unsigned i = 0; i < 48 && p >= 0x40; i++) {
  uint32_t v = wl_prd(WL_DBI + p);
  if ((v & 0xff) == 0x10) return p;
  p = (v >> 8) & 0xfc;
 }
 return 0;
}

/* Shared initial/resume PHY sequence. Resume holds PHY BCR for the
 * downstream reset-control 1 ms; initial bring-up retains its 250 us hold. */
static int wl3_phy_init(unsigned reset_us)
{
  uint32_t *r = wl3_report;
  unsigned i;
  wl_set(WL_GCC, 0x6c01c, 1); delay(reset_us);
  if (!(usb_rd(WL_GCC, 0x6c01c) & 1u)) return 1;
  wl_clr(WL_GCC, 0x6c01c, 1); delay(10);
  if (usb_rd(WL_GCC, 0x6c01c) & 1u) return 1;
  wl_pwr(WL_PHY + 0x240, wl_prd(WL_PHY + 0x240) | 3u);       /* SW_PWRDN|REFCLK_DRV_DSBL */
  for (i = 0; i < sizeof(wl_phy_tbl) / sizeof(wl_phy_tbl[0]); i++)
   wl_pwr(WL_PHY + wl_phy_tbl[i][0], wl_phy_tbl[i][1]);
  wl_wr(WL_GCC, 0x6b070, usb_rd(WL_GCC, 0x6b070) & ~3u);     /* pipe mux: PHY pipe */
  wl_set(WL_GCC, 0x52008, 1u << 4);                          /* pipe branch vote */
  wl_pwr(WL_PHY + 0x200, wl_prd(WL_PHY + 0x200) & ~1u);      /* SW_RESET release */
  wl_pwr(WL_PHY + 0x244, wl_prd(WL_PHY + 0x244) | 3u);       /* SERDES|PCS start */
  delay(1000);
  for (i = 0; i < 10000; i += 10) { if (!(wl_prd(WL_PHY + 0x214) & (1u << 6))) break; delay(10); }
  r[62] = i; r[63] = wl_prd(WL_PHY + 0x214);
  r[64] = usb_rd(WL_GCC, 0x6b048); r[65] = usb_rd(WL_GCC, 0x6b070);
  if (wl3_faults) return 1;
  return (r[63] & (1u << 6)) ? 2 : 0;
}

/* Returns 0 on success or a nonzero reason; the caller encodes step<<8. */
static int wl3_step(unsigned step)
{
 uint32_t *r = wl3_report;
 uint32_t polls = 0;
 switch (step) {
 case 1: { /* precheck, read-only */
#if defined(FUXI_SALAMI)
  const uint32_t *expect = wl_salami_rail;
#else
  static const uint32_t expect[8] = {0x40a00, 0x40200, 0x40400, 0x40500, 0x44000, 0x40600, 0x43900, 0x43a00};
#endif
  wlan_snapshot();
  if (wlan_report[90] != 0x0c0330db) return 1;
  for (unsigned i = 0; i < 8; i++) {
   r[20 + i] = wlan_report[91 + i];
#if defined(FUXI_SALAMI)
   if (wlan_report[91 + i] == WL_SALAMI_LDOB12) return 5;   /* salami ldob12, panel vddio */
#endif
   if (wlan_report[91 + i] != expect[i]) return 2;
  }
  uint32_t id = usb_rd(RH_DRV_PA, 0), cfg = usb_rd(RH_DRV_PA, 0xc);
  if (((id >> 16) & 0xff) != 3 || ((cfg >> 12) & 0x3f) < 3) return 3;
  r[28] = usb_rd(WL_GCC, 0x0);           /* GPLL0 mode: bit 31 lock */
  return (r[28] & 0x80000000u) ? 0 : 4;
 }
 case 2: { /* WCN rails in pwrseq order: s5g l15b s2g s4e s4g s6g */
#if defined(FUXI_SALAMI)
  static const uint16_t mv[6] = {1000, 1800, 980, 950, 1350, 1900};   /* salami DT: S4E 950 mV */
#else
  static const uint16_t mv[6] = {1000, 1800, 980, 952, 1350, 1900};
#endif
  static const uint8_t idx[6] = {4, 0, 2, 1, 3, 5};
  for (unsigned i = 0; i < 6; i++) {
   r[29 + i] = wl_vrm(r[20 + idx[i]], mv[i]);
   if (r[29 + i]) return 1 + (int)i;
   delay(1000);
  }
  return 0;
 }
 case 3: { /* WLAN_EN high */
  uintptr_t sw = WL_TLMM + 0x1000u * 82, wsw = WL_TLMM + 0x1000u * 83;
  r[36] = usb_rd(sw, 4); r[37] = usb_rd(wsw, 4);
  ts_gpio(80, 0, 3, 16, 1);
  delay(50000);
  r[38] = usb_rd(WL_TLMM + 0x1000u * 80, 0); r[39] = usb_rd(WL_TLMM + 0x1000u * 80, 4);
  r[40] = usb_rd(sw, 4); r[41] = usb_rd(wsw, 4);
  return (r[39] & 2) ? 0 : 1;
 }
 case 4: /* PCIe PHY rails: vdda-phy l1e, vdda-pll l3e */
  r[35] = wl_vrm(r[26], 880);
  if (r[35]) return 1;
  r[42] = wl_vrm(r[27], 1200);
  if (r[42]) return 2;
  delay(1000);
  return 0;
 case 5: /* PERST# asserted, CLKREQ#, WAKE# */
  ts_gpio(94, 0, 1, 2, 0);
  ts_gpio(95, 1, 3, 2, -1);
  ts_gpio(96, 0, 3, 2, -1);
  delay(1000);
  r[43] = usb_rd(WL_TLMM + 0x1000u * 94, 4); r[44] = usb_rd(WL_TLMM + 0x1000u * 96, 4);
  return (r[43] & 2) ? 1 : 0;
 case 6:
  if (wl_gdsc(0x6b004, 1u << 0, &polls)) { r[45] = polls; return 1; }
  r[45] = polls;
  if (wl_gdsc(0x6c000, 1u << 3, &polls)) { r[46] = polls; return 2; }
  r[46] = polls;
  r[47] = usb_rd(WL_GCC, 0x6b004); r[48] = usb_rd(WL_GCC, 0x6c000);
  return 0;
 case 7: { /* clocks */
  static const uint32_t cb[8] = {0x6b038, 0x6b020, 0x6b02c, 0x6b03c, 0x6b01c, 0x1003c, 0x1004c, 0x6b054};
  if (!wl_rcg(0x6b074, 0x000)) return 1;           /* aux: XO 19.2 MHz */
  if (!wl_rcg(0x6b058, 0x605)) return 2;           /* rchng: GPLL0 even (300 MHz) / 3 */
  wl_set(WL_GCC, 0x52008, (1u << 0) | (1u << 1) | (1u << 2) | (1u << 3) | (1u << 5));
  wl_set(WL_GCC, 0x52000, (1u << 12) | (1u << 19) | (1u << 22));
  wl_set(WL_TCSR, 0x15100, 1);
  uint32_t bad = 0;
  for (unsigned i = 0; i < 8; i++) {
   if (!wl_branch_ok(cb[i])) bad |= 1u << i;
   r[49 + i] = usb_rd(WL_GCC, cb[i]);
  }
  r[57] = bad;
  return bad ? 3 : 0;
 }
 case 8: /* core reset + PARF (qcom_pcie_init_2_7_0) */
  wl_set(WL_GCC, 0x6b000, 1); delay(1000);
  if (!(usb_rd(WL_GCC, 0x6b000) & 1u)) return 1;
  wl_clr(WL_GCC, 0x6b000, 1); delay(1000);
  if (usb_rd(WL_GCC, 0x6b000) & 1u) return 1;
  wl_pwr(WL_PARF + 0x1000, 4);
  wl_pwr(WL_PARF + 0x40, wl_prd(WL_PARF + 0x40) & ~1u);
#ifdef FUXI_WLAN_XNU
  if (!wl_m6o_parf()) {
#endif
  wl_pwr(WL_PARF + 0x350, 0x60000000u); wl_pwr(WL_PARF + 0x354, 0);
  wl_pwr(WL_PARF + 0x634, 0x60001000u); wl_pwr(WL_PARF + 0x638, 0);
  wl_pwr(WL_PARF + 0x358, 0); wl_pwr(WL_PARF + 0x35c, 0x80000000u);
  wl_pwr(WL_PARF + 0x0, wl_prd(WL_PARF + 0x0) & ~(1u << 29));
#ifdef FUXI_WLAN_XNU
  }
#endif
  wl_pwr(WL_PARF + 0x174, wl_prd(WL_PARF + 0x174) | (1u << 4));
  wl_pwr(WL_PARF + 0x20, wl_prd(WL_PARF + 0x20) & ~(1u << 5));
  wl_pwr(WL_PARF + 0x1a8, wl_prd(WL_PARF + 0x1a8) | (1u << 31));
#ifdef FUXI_WLAN_XNU
  if (wl_m6o_host(8)) return 0x6f;
#endif
  r[58] = wl_prd(WL_PARF + 0x1000); r[59] = wl_prd(WL_PARF + 0x40);
  r[60] = wl_prd(WL_PARF + 0x350); r[61] = wl_prd(WL_PARF + 0x634);
  return (wl3_faults || (r[58] & 0xf) != 4) ? 1 : 0;
 case 9: return wl3_phy_init(250); /* QMP PHY, initial reset hold */
 case 10: /* PERST# deassert */
#ifdef FUXI_WLAN_XNU
  if (wl_m6o_host(10)) return 0x6f;
#endif
  delay(100000);
  ts_gpio(94, 0, 1, 2, 1);
  delay(1000);
  r[66] = usb_rd(WL_TLMM + 0x1000u * 94, 4);
  return (r[66] & 2) ? 0 : 1;
 case 11: { /* LTSSM + link */
  uint32_t cap = wl_pcie_cap(), lnk = 0, lt = 0;
  uint64_t t0, seen0 = 0, seen1 = 0;
  r[67] = cap;
  if (!cap || wl3_faults) return 1;
#ifdef FUXI_WLAN_XNU
  if (wl_m6o_host(11)) return 0x6f;
#endif
  wl_pwr(WL_PARF + 0x1b0, wl_prd(WL_PARF + 0x1b0) | (1u << 8));
  t0 = wl_now_us();
  for (;;) {
   lt = wl_prd(WL_PARF + 0x1b0);
   unsigned s = lt & 0x3f;
   if (s < 32) seen0 |= 1ull << s; else seen1 |= 1ull << (s - 32);
   lnk = wl_prd(WL_DBI + cap + 0x10);
   if (lnk & (1u << 29)) break;
   if (wl_now_us() - t0 > 1000000 || wl3_faults) break;
   delay(100);
  }
  r[68] = (uint32_t)((wl_now_us() - t0) / 1000);
  r[69] = lt; r[70] = lnk; r[71] = (uint32_t)seen0; r[72] = (uint32_t)seen1;
  r[73] = wl_prd(WL_PARF + 0x224);
  if (wl3_faults) return 2;
  return (lnk & (1u << 29)) ? 0 : 3;
 }
 case 12: { /* endpoint config space through iATU CFG0 */
  const uint64_t a = 0x60001000ull;  /* unrolled outbound region 0 */
  uint64_t t0;
  uint32_t id;
  unsigned i, tries = 0;
  delay(100000);
  wl_pwr(WL_DBI + 0x18, (wl_prd(WL_DBI + 0x18) & 0xff000000u) | 0x00ff0100u);
  wl_pwr(a + 0x08, 0x60100000u); wl_pwr(a + 0x0c, 0);
  wl_pwr(a + 0x10, 0x601fffffu); wl_pwr(a + 0x20, 0);
  wl_pwr(a + 0x14, 1u << 24);    wl_pwr(a + 0x18, 0);
  wl_pwr(a + 0x00, 4);           /* type CFG0 */
  wl_pwr(a + 0x04, 1u << 31);    /* enable */
  for (i = 0; i < 5; i++) { if (wl_prd(a + 0x04) & (1u << 31)) break; delay(9000); }
  r[74] = wl_prd(a + 0x04);
  if (!(r[74] & (1u << 31)) || wl3_faults) return 1;
  t0 = wl_now_us();
  do {
   id = wl_prd(0x60100000ull);
   tries++;
   if (id != 0xffffffffu && id != 0xffff0001u && id != 0) break;
   delay(10000);
  } while (wl_now_us() - t0 < 1000000 && !wl3_faults);
  r[75] = id; r[76] = tries;
  r[77] = wl_prd(0x60100008ull); r[78] = wl_prd(0x6010000cull);
  r[79] = wl_prd(0x6010002cull); r[80] = wl_prd(0x60100004ull);
  if (wl3_faults) return 2;
#ifdef FUXI_WLAN_XNU
  if (id == 0x110717cbu && wl_m6o_host(12)) return 0x6f;
#endif
  return id == 0x110717cbu ? 0 : 3;
 }
 case 13: /* BAR sizing: r[81+b] original, r[87+b] all-ones readback */
  for (unsigned b = 0; b < 6; b++) {
   uint64_t at = 0x60100010ull + 4 * b;
   uint32_t orig = wl_prd(at);
   wl_pwr(at, 0xffffffffu);
   r[87 + b] = wl_prd(at);
   wl_pwr(at, orig);
   r[81 + b] = orig;
   if (!(orig & 1) && ((orig >> 1) & 3) == 2 && b < 5) {   /* 64-bit memory BAR */
    uint32_t ohi = wl_prd(at + 4);
    wl_pwr(at + 4, 0xffffffffu);
    r[88 + b] = wl_prd(at + 4);
    wl_pwr(at + 4, ohi);
    r[82 + b] = ohi;
    b++;
   }
  }
  return wl3_faults ? 1 : 0;
 }
 return 0xff;
}

static void wlan_m3(unsigned limit)
{
 uint32_t *r = wl3_report;
 r[0] = WL3_MAGIC; r[1] = limit;
 if (wl3_failed || limit < 1 || limit > WL3_STEPS) return;
 for (unsigned s = 1; s <= limit; s++) {
  if (wl3_done & (1u << s)) continue;
  wl_mark(s);
  wl3_faults = 0;
  uint64_t t0 = wl_now_us();
  int e = wl3_step(s);
  r[100 + s] = (uint32_t)((wl_now_us() - t0) / 1000);   /* ms per step */
  r[3 + s] = e ? (s << 8 | (unsigned)e) : 0x1000u | s;
  r[124] += wl3_faults;
  r[125] = probe_fault_esr;
  r[126] = (uint32_t)probe_fault_far;
  if (e) { wl3_failed = 1; r[3] = s << 8 | (unsigned)e; return; }
  wl3_done |= 1u << s;
 }
 r[3] = 0;
}
