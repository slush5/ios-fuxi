# Kiwi RX study: how the vendor host driver brings up RX on WCN7850, against our lane and L19

Study only, plus one read-only line (C2h, at the end). Question: on the phone the firmware takes all 4095 buffers from
refill ring 160 and copies them into the per-MAC rings 161/169 (entries 0..2046 and 0..2015 valid), but the HP words we
gave for 161/169 stay 0, monitor status rings 185/193 stay at tp 0, RXDMA writes nothing, nothing reaches REO, release
or WMI MGMT_RX, and the passive scan stalls after a few channels with the firmware alive. The firmware is Qualcomm's
Android build (WLAN.HMT.1.0-03639-QCAHMTSWPL_V1.0_V2.0_SILICONZ), made for the vendor host driver, so this reads that
driver for kiwi (WCN7850).

## Sources and licences

Vendor tree: the OnePlus 11 (SM8550) vendor modules as mirrored at github.com/LineageOS/android_kernel_oneplus_sm8550-modules,
commit 9c16bd242fa00a4508c98908160602a314f20f0d. Abbreviations for the paths below:

- `CMN` = `qcom/opensource/wlan/qca-wifi-host-cmn`
- `CLD` = `qcom/opensource/wlan/qcacld-3.0`
- `FWAPI` = `qcom/opensource/wlan/fw-api`

Every vendor source file cited here carries the ISC permission notice ("Permission to use, copy, modify, and/or
distribute this software for any purpose with or without fee is hereby granted..."), checked file by file:
`CMN/dp/wifi3.0/{dp_main.c, dp_htt.c, dp_rx.c, dp_internal.h, dp_types.h}`, `CMN/dp/wifi3.0/be/{dp_be.c, dp_be.h,
dp_be_rx.c}`, `CMN/dp/wifi3.0/monitor/{dp_mon.c, 1.0/dp_mon_1.0.c, 1.0/dp_rx_mon_dest_1.0.c, 1.0/dp_mon_filter_1.0.c}`,
`CMN/hal/wifi3.0/{hal_srng.c, hal_api.h, hal_internal.h, hal_rx.h}`, `CMN/hal/wifi3.0/kiwi/hal_kiwi.c`,
`CMN/hal/wifi3.0/be/{hal_be_generic_api.h, hal_be_rx_tlv.h}`, `CMN/wlan_cfg/{wlan_cfg.c, cfg_dp.h, wlan_init_cfg.h}`,
`CMN/hif/src/pcie/if_pci.c`, `CMN/wmi/src/wmi_unified_tlv.c`, `FWAPI/fw/{htt.h, wmi_unified.h}`,
`FWAPI/hw/kiwi/v2/wcss_seq_hwioreg_umac.h`, `CLD/core/cds/src/cds_api.c`, `CLD/core/wma/src/wma_main.c`,
`CLD/core/hdd/src/wlan_hdd_main.c`, `CLD/components/dp/core/src/wlan_dp_rx_fst.c`. `CLD/Kbuild` and `CLD/configs/*` have no
licence header (make fragments); they are cited only for which build flags are set, nothing is taken from them. No GPL
file was opened for this study (no MHI, QRTR or QMI core, no GPL-only vendor file). No code is copied: behaviour is
described and lines are cited; the C2h code is our own.

L19 = Linux ath12k v6.19 (BSD-3-Clause-Clear), `drivers/net/wireless/ath/ath12k`, tag v6.19 (05f7e89). The host test
re-derives our lane's L19 facts from that tree (`ATH12K_SRC`), and it passed for this change.

Confidence marks: [READ] read in the code; [INF] inferred from code; [GUESS] plausible, not shown.

## Build configuration of the phone's vendor driver (kiwi_v2 on Kalama)

- `kiwi_v2_defconfig` includes `default_defconfig` (`CLD/configs/kiwi_v2_defconfig:1-8`). [READ]
- `QCA_HOST2FW_RXBUF_RING`, `DP_CON_MON` and `DP_INTR_POLL_BOTH` are defined unconditionally (`CLD/Kbuild:4247, 4251,
  4254`). [READ]
- Monitor is the 1.0 implementation (`CLD/Kbuild:2205-2211`): no HOST2MON/MON2HOST rings. [READ]
- On for kiwi: Beryllium DP, HW cookie conversion, `NO_RX_PKT_HDR_TLV`, RX FISA (`default_defconfig:1486-1491`,
  `CLD/Kbuild:4644`), shadow v3 and delayed register writes (`default_defconfig:669-687`), force wake, IPA on Kalama
  (`kiwi_v2_defconfig:13-22`). [READ]
- Not set for kiwi: PPEDS, direct link, word-based TLV, UMAC reset, VDEV stats HW offload. [INF from absence]

## a. How the vendor driver sets up RX for kiwi

### Order relative to HTC, WMI INIT and HTT

`hdd_wlan_start_modules` (`CLD/core/hdd/src/wlan_hdd_main.c:4498-4717`) runs `cds_open`, `cds_dp_open`, `cds_pre_enable`,
then `cds_enable`. [READ]

1. `cds_open` (`CLD/core/cds/src/cds_api.c:783, 821-838`): HTC target wait, then for TARGET_TYPE_KIWI `cdp_soc_attach` and
   `cdp_soc_init`.
   - `dp_soc_attach` reads the CMEM base and size from HIF, which got them from QMI `dev_mem[0]`
     (`CMN/dp/wifi3.0/dp_main.c:15667-15670`, `CMN/hif/src/pcie/if_pci.c:3695-3697`); `dp_soc_attach_be` carves CMEM for
     cookie conversion, then for the FISA flow search table, and allocates the SPT pages
     (`CMN/dp/wifi3.0/be/dp_be.c:786-850`).
   - `dp_soc_init` (`dp_main.c:15791-15990`), in order: `dp_soc_init_be` writes the PPT entries into CMEM and programs the
     REO/WBM cookie registers (`dp_be.c:875-910`); HTT soc attach (15813-15821); `dp_soc_cfg_init` with the kiwi overrides
     (15823); interrupt mode and masks (15833-15852); the WBM idle link ring and its list (15854-15859); every UMAC ring
     (WBM release, TCL, reinject, WBM2SW release, REO exception, REO command and its init, REO status, TX completion, REO
     destination, `dp_main.c:16842-16960`); then the HTT HTC service connect (15866, `dp_htt.c:4043-4118`).
2. `cds_dp_open` (`cds_api.c:957-972`): pdev attach allocates `rx_mac_buf_ring[]` (`dp_main.c:5236-5256`); `dp_pdev_init`
   (`dp_main.c:17397-17560`) initialises the refill ring and the RXDMA error destination ring (`16468-16515`), the per-MAC
   buffer rings (`5265-5281`), the monitor status rings (`1.0/dp_mon_1.0.c:261-290`), then
   `dp_rx_pdev_buffers_alloc` fills only the refill ring with entries - 1 buffers (`CMN/dp/wifi3.0/dp_rx.c:3385-3407`).
   With the default 4096 that is 4095, as we post.
3. `cds_pre_enable` (`cds_api.c:1103-1160`): WMA pre-start, HTC start, wait for WMI READY (1150). WMI_INIT is sent from
   the service-ready-ext handler (`CLD/core/wma/src/wma_main.c:7294`).
4. `cds_enable` (`cds_api.c:1193-1250`): `wma_start` (1216), MAC and SME start, then `cdp_soc_attach_target` (1239) =
   `dp_soc_attach_target_wifi3` (`dp_main.c:6898-6991`), after WMI READY:
   1. HTT VERSION_REQ (`dp_htt.c:2352-2357`);
   2. `dp_rxdma_ring_config`: every SRING_SETUP (below);
   3. `dp_rxdma_ring_sel_cfg_be`: RX_RING_SELECTION_CFG (6918);
   4. `dp_rx_target_fst_config`: HTT RX_FSE_SETUP_CFG (6933, `6726-6750`), non-fatal;
   5. `dp_rx_fisa_config`: HTT RX_FISA_CFG, pdev 0, timeout 0xffffffff (6940-6945, `6755-6764`);
   6. `hal_reo_setup` and the REO error destination remap, last (6985, 6987).

So the vendor programs cookie conversion, the idle list and every UMAC ring before HTC/WMI INIT, fills the refill ring
before WMI INIT, and announces the LMAC rings by HTT only after WMI READY, followed by FSE/FISA and REO setup. L19 does the
same split: the rings in `ath12k_dp_alloc` before core start, the cc registers at service ready before INIT, the
SRING_SETUP after the HTT version exchange. [READ]

### The SRING_SETUP messages kiwi gets (`dp_rxdma_ring_config`, `dp_main.c:6574-6646`)

| Ring | HTT ring id | HTT type | pdev_id | Count on kiwi |
|---|---|---|---|---|
| refill (`rx_refill_buf_ring[lmac 0]`) | HOST1_TO_FW_RXBUF | SW_TO_SW | mac_id = 0 | 1 |
| IPA refill 2 (only if IPA is on at run time) | HOST2_TO_FW_RXBUF | SW_TO_SW | 0 | 0 or 1 |
| per-MAC buffer (`rx_mac_buf_ring[mac]`) | RXDMA_HOST_BUF | SW_TO_HW | mac + 1 | 1, 2 if DBS capable |
| RXDMA error destination | RXDMA_NON_MONITOR_DEST | HW_TO_SW | 1 | 1 (mac 0 only) |
| monitor status (`rxdma_mon_status_ring[mac]`) | RXDMA_MONITOR_STATUS | SW_TO_HW | mac + 1 | per MAC |

- Ring type selection: `dp_htt.c:668-747`; pdev_id is `mac_id` for SW_TO_SW and `DP_SW2HW_MACID(mac)` for the other two
  (`dp_htt.c:779-787`). [READ]
- `max_mac_rings` is 2, cut to 1 unless the platform says DBS capable (`dp_main.c:16032-16044`). [READ]
- `num_rxdma_dst_rings_per_pdev` = 1 and `rxdma1_enable` = 0 for kiwi (`dp_main.c:17191-17208, 17310-17312`): one error
  ring, no monitor buf/dst/desc rings (`1.0/dp_rx_mon_dest_1.0.c:2268-2305`). [READ]
- No REO, WBM or TCL ring is announced by HTT: the host programs those registers itself. [READ]

### SRING_SETUP words as the vendor fills them (`dp_htt.c:635-922`, field masks `FWAPI/fw/htt.h:5474-5812`)

- w0: msg type 0xb, pdev_id, ring_id (bits 16-23), ring_type (bits 24-31).
- w1/w2: ring base address lo/hi.
- w3: entry size in dwords, ring size in dwords; LOOPCOUNT_DISABLE (bit 25) only for SW_TO_HW rings (818-819); the MSI,
  TLV and HOST_FW swap bits come from HAL flags that exist only on big-endian hosts (`CMN/hal/wifi3.0/hal_srng.c:1696-1701`),
  so they are 0; RING_ID_DISABLE and SECURITY never set.
- w4/w5: remote HP address; w6/w7: remote TP address (`hal_api.h:2668-2723`, `hal_srng.c:1707-1782`). For an LMAC
  source ring the HP is a host DMA word in the shadow write-pointer area (index ring_id - LMAC start) and the TP a word of
  the shadow read-pointer area (index ring_id); a destination ring the reverse. The same split L19 makes with wrp/rdp.
- w8/w9/w10: MSI address lo/hi and data; 0 in the vendor's poll mode or for a ring not in an interrupt group
  (`dp_main.c:2425-2433, 1683-1697`).
- w11: batch counter threshold (bits 0-14, in words) and timer threshold (us >> 3, bits 16-31); SW_INTR_MODE never set
  (870-876). Kiwi uses the per-ring-type table (`dp_main.c:1909-1955`, `CMN/wlan_cfg/wlan_cfg.c:2325-2410`).
- w12: low threshold only when the low-threshold interrupt is enabled (881-887). PREFETCH_TIMER and RESPONSE_REQUIRED
  never set; the vendor does not wait for SRING_SETUP_DONE (902-910).

These are L19's words (docs-lanes/wlan-w20.md C2c): same ring ids and types, same pdev rule, same LOOP_CNT_DIS rule, same
remote-pointer scheme, MSI 0 as in the vendor's poll mode. Only sizes and thresholds differ (the vendor's per-MAC ring
default is 1024 entries, `cfg_dp.h:335-341`, ours and L19's 2048).

### RX_RING_SELECTION_CFG (`dp_rxdma_ring_sel_cfg_be`, `CMN/dp/wifi3.0/be/dp_be.c:1143-1238`)

- Sent per MAC to the per-MAC buffer ring (`dp_get_rxdma_ring`, `dp_internal.h:1395-1407`), never to the refill ring:
  ring_id RXDMA_HOST_BUF, type SW_TO_HW, pdev_id mac + 1, buffer size 2048 (`hal_rx.h:53-56`; `dp_htt.c:1083-1860`).
- w0: RX_OFFSETS_VALID set; RX_MON_GLOBAL_EN, PACKET_TYPE_ENABLE_DATA and the drop threshold not set (monitor 1.0
  registers no setters).
- Packet filters: FP only; `fp_mgmt` = 0, `fp_ctrl` = BAR, `fp_data` = UCAST | MCAST | DATA (`dp_be.c:1164-1179`). No
  management frame is routed to the host rings: management reaches the host by WMI MGMT_RX.
- TLVs: mpdu_start, msdu_end, packet; offsets packet = RX TLV size, mpdu_start = its offset in `rx_pkt_tlvs`,
  msdu_end 0, the rest 0 (`dp_be.c:1191-1203`, `hal_be_rx_tlv.h:401-424, 2251-2254`). No packet header TLV
  (NO_RX_PKT_HDR_TLV), so the packet offset is about 272 by the header's size comments [INF]; L19 keeps the header TLV
  (packet 400, header 272, mpdu_start 144, msdu_end 0). No word mask (`dp_be.c:1020-1043`).
- The monitor status rings get no ring selection at bring-up, only for monitor mode, pktlog, enhanced stats or CFR
  (`monitor/dp_mon.c:312, 523, 1204, 1233, 1279, 1965`). L19 sends none in STA mode either.

### Messages the vendor sends in this path that L19 does not

| Message | Where | Content |
|---|---|---|
| HTT RX_FSE_SETUP_CFG | `dp_main.c:6726-6750`, `wlan_dp_rx_fst.c:439-490`, `dp_htt.c:4667-4790` | pdev 0, flow table in CMEM at base + 0x1000 (address hi ORed with 0x10), max entries and search depth from wlan_cfg, IPv4-compatible-IPv6 prefix, Toeplitz key |
| HTT RX_FISA_CFG | `dp_main.c:6755-6764`, `dp_htt.c:4958-5045` | pdev 0, aggregation timeout 0xffffffff |
| SRING_SETUP HOST2_TO_FW_RXBUF | `dp_main.c:6574-6646` | the IPA refill ring, only with IPA on |
| WMI INIT flag1 bit 19 HTT_H2T_NO_HTC_HDR_LEN_IN_MSG_LEN | `wma_main.c:7300-7307`, `wmi_unified_tlv.c:9151-9154`, `wmi_unified.h:4458-4469` | set when the firmware advertises service 176; changes only the length rule of HTT TX_FRM, which a receive-only host never sends |
| WMI INIT flag1 bit 30 IPA_DISABLE | `wma_main.c:238-248`, `wmi_unified_tlv.c:9178-9179`, `wmi_unified.h:4556-4560` | set when IPA is off at run time |
| WMI LRO_CONFIG | `dp_main.c:7331-7336, 5114-5172` | once, at the first vdev attach (L19 sends it at mac start; we found it stalls our polled host) |

L19's flag1 is `atf_config | BSS_CHANNEL_INFO_64 | ACK_RSSI` (L19 `wmi.c:3999-4001`); ours is a copy. Not sent for this
kiwi build: PPE config, UMAC hang recovery, vdev stats config, full monitor mode. [READ]

## b. Who fills the per-MAC RXDMA buffer rings, and where their HP lives

- The firmware fills them from the host refill ring. The vendor says so at `dp_rxdma_ring_config`
  (`dp_main.c:6533-6544`): on MCL the host provides buffers in the Host2FW ring, the firmware copies them into the MAC
  ring and "updates ring_idx in register". The host never replenishes `rx_mac_buf_ring[]`; its replenish targets only
  the refill ring (`dp_rx.c:3393-3406`), and its other uses of `rx_mac_buf_ring` are allocation, SRING_SETUP, ring
  selection and stats (`dp_main.c:5249-5313, 6622-6624`, `dp_be.c:1215-1229`). [READ]
- The HP of the per-MAC rings: the vendor hands the firmware a host-memory word (SRING_SETUP w4/w5) as for every LMAC
  ring, but never reads it, and its own comment puts the index the firmware updates "in register". The kiwi HAL table
  gives RXDMA_BUF no register at all ("LMAC rings are not accessed from host", `CMN/hal/wifi3.0/kiwi/hal_kiwi.c:2645-2662`)
  and LMAC rings have no shadow register (`hal_srng.c:310-333`). So the producer index the firmware drives is RXDMA's own
  hardware register, inside the target, and the host-memory word may stay 0 forever. [READ for the comment and tables;
  INF that the word is never written]
- The selector is compile-time, not negotiated: `QCA_HOST2FW_RXBUF_RING` (always on for qcacld) gives "host refill ring
  (HOST1_TO_FW, SW_TO_SW) plus firmware-fed per-MAC SW_TO_HW rings" (`dp_htt.c:670-701`). Without it (WIN builds) the
  BUF0 ring itself is announced as RXDMA_HOST_BUF SW_TO_HW and the host fills it (`dp_htt.c:690-695`). The "host fills a
  DMAC common ring" mode exists only for QCN9224/QCA5332 (`hal_soc->dmac_cmn_src_rxbuf_ring`). No WMI service bit,
  resource-config flag, HTT version check or wlan_cfg item switches this for kiwi; `per_pdev_lmac_ring` (default 0,
  `cfg_dp.h:103-105, 721-726`) only changes the pdev/lmac id mapping. [READ; absence by search]
- One refill ring per pdev (lmac 0), not one per MAC (`dp_main.c:16476-16483, 6587-6591`). All bands map to MAC0 on kiwi
  (`dp_main.c:10745-10750`). [READ]
- The vendor's host ring numbers (refill 184, per-MAC 185 in `hal_internal.h:268-313`) differ from L19's 160/161/169, but
  ring numbers never go on the wire: the firmware sees HTT ring id, type, pdev id and addresses only. [READ]

So on kiwi the firmware fills 161/169 (which the phone shows), and their HP is not published to the host in any way the
vendor relies on. "161/169 valid but wrp[1]/wrp[9] = 0" is the vendor's normal state, not a fault. This confirms C2g's
L19 reading with the vendor's own words.

## c. CMEM and cookie conversion on kiwi

- CMEM layout (`CMN/dp/wifi3.0/be/dp_be.h:38-147`, `dp_be.c:185-205, 275-294, 796-835`): the PPT at CMEM + 0, 4 KiB
  (1024 entries of 4 bytes, page address >> 12), then the FISA flow search table at CMEM + 0x1000, 16 KiB. 4 KiB +
  16 KiB = 0x5000: exactly the region QMI gives us (0x113000, size 0x5000), so 0x113000-0x113fff is the PPT and
  0x114000-0x117fff the flow table. [READ; INF for our addresses]
- PPT carving: TX descriptor pools first (one 512-byte slice, 128 entries, per pool, `WLAN_CFG_NUM_TX_DESC_MAX` 0x10000 /
  512 x 4), then RX pools (128 bytes each) from offset `MAX_TXDESC_POOLS` x 512 (`dp_be.h:127-147`, `dp_types.h:77-85`,
  `cfg_dp.h:185, 409`). Only the entries for pages actually allocated are written (`dp_be.c:421-457`). So our 280-entry
  pre-read (indices 0..279) sees the vendor's TX pool pages, in runs starting at 0 and 128 (and 256), and none of its RX
  pages. [INF]
- The vendor programs cookie conversion itself, before WMI INIT, with plain writes (not read-modify-write), unless FTM or
  the `ghw_cc_enable` INI (default 1) is off (`dp_be.c:222-250`, `CMN/hal/wifi3.0/be/hal_be_generic_api.h:3314-3434`):
  REO SW_COOKIE_CFG0 = CMEM base; CFG1 = global enable, convert enable, 4 KiB align, offset MSB 8, page MSB 19
  (`DP_CC_DESC_ID_PPT_PAGE_OS_MSB`), address bits 39:32 = 0x10, which is 0x1d1310 with the kiwi v2 field layout
  (`FWAPI/hw/kiwi/v2/wcss_seq_hwioreg_umac.h:177-214`); WBM CFG0/1 the same without the enables (0x51310); WBM
  convert config global plus per-WBM2SW enables except WBM2SW1 (the firmware's ring, `dp_be.c:129-181`); WBM CFG_2 =
  0x4b; with `DP_HW_COOKIE_CONVERT_EXCEPTION` a read-modify-write of WBM MISC_CONTROL bit 15. [READ; the totals are
  arithmetic, INF]
- PPT_MSB: the vendor uses 19 (0x13), the same as L19. It never writes 0x17. The phone's REO CFG1 0x111700 (page MSB 0x17,
  offset MSB 8, GLOBAL_ENABLE, no convert enable, no base) is not a vendor value; WBM CFG_2 0x40 is its documented reset
  value (`wcss_seq_hwioreg_umac.h:192-214`). Reading: REO CFG1 0x111700 is most likely a reset or firmware default, and
  nothing has programmed cookie conversion on this boot when we start. [GUESS for 0x111700's origin]
- Does the firmware expect it? Cookie conversion is host-owned: the firmware's own WBM2SW1 is excluded, the host's cookies
  are `ppt << 9 | spt` (`dp_be.h:654-667`, `be/dp_be_rx.c:955-1003`), and REO/WBM translate them on the way out. RXDMA's
  buffer consumption does not depend on it; a wrong PPT corrupts what REO/WBM report, it does not stop RXDMA. [INF]
- A stale PPT: the single-pdev deinit (`dp_be.c:460-475`) clears only host bookkeeping and leaves CMEM as it is; only the
  multi-pdev variant zeroes the PPT (477-497). Init writes the entries it uses and never clears the rest. So the vendor
  expects stale PPT words from an earlier session, overwrites what it needs and ignores the rest: neither reset nor
  refuse. The non-zero words the phone shows are the Android driver's PPT (it loads at boot whether Wi-Fi is on or not,
  which matches "toggling Wi-Fi changes nothing"). [READ for deinit/init; INF for the phone]
- Our lane (C2f/C2g) treats any non-zero PPT word as an owner and drops cookie conversion for the boot, so on the phone cc
  is never programmed. That is conservative and harmless for RXDMA (above), but it differs from the vendor, which would
  simply overwrite.

## d. Interrupts and doorbells at RX start, and their polled equivalents

- MSI: optional. The vendor's poll mode (NAPI off, `dp_main.c:3161-3180`) sends every SRING_SETUP with MSI address and
  data 0 (`dp_main.c:2425-2433`) and services the rings from a timer. The firmware takes HP/TP locations from w4-w7, not
  from the MSI. Our MSI 0 is the vendor's poll configuration. [READ]
- Refill ring: no doorbell. Its HP is a host-memory store the firmware reads (`hal_api.h:2384-2393`, the LMAC branch of
  access end); ours (wrp[0]) is the same. [READ]
- UMAC rings (REO, WBM, TCL): HP/TP through shadow registers (v3), optionally deferred to a worker (delayed register
  write, `hal_api.h:450-458`, `hal_srng.c:1042-1090`); the polled equivalent is the plain register or shadow write we make.
  [READ]
- Wake: register accesses beyond the first 4 KiB of BAR0 outside the init phase, and CMEM writes
  (`hal_kiwi.c:1823-1830`), are bracketed by force-wake request/release (`hal_api.h:272-325, 525-569`). Our lane holds
  the W11 device wake for every W20 access, which covers this. [READ; INF for equivalence]
- No HTT T2H in the RX bring-up needs an answer: SRING_SETUP asks none, VERSION_CONF is consumed, and the FSE CMEM-base
  wait applies only when the base is unknown, which it is not on kiwi (`wlan_dp_rx_fst.c:469-481`). [READ]

Nothing at RX start needs an interrupt or a doorbell that we do not already give by polling.

## e. Comparison and ranking

What the vendor shows about the symptom itself, before any ranking:

1. "161/169 filled, their HP words 0": vendor-normal (b). The firmware keeps that index in RXDMA's register.
2. "Mon status 185/193 never taken": vendor-normal without monitor, pktlog, stats or CFR filters (a).
3. "RXDMA writes nothing" during a passive scan before association: expected with the vendor's filter, and with L19's,
   which is the same (FP data and BAR only, no management: L19 `dp_rx.c:4567-4603`,
   `ath12k_dp_rxdma_ring_sel_config_wcn7850`, re-derived by our test in C2d). Beacons and probe responses go to the firmware and up as WMI MGMT_RX. With no association there is no
   unicast data for us, and a STA's RXPCU passes no foreign BSS data, so an empty buffer set is what the vendor would see
   too. [INF]

So the three RX-ring observations do not, on the vendor's evidence, show a broken RX ring setup. The failure that
matters is the other half of the symptom: no WMI MGMT_RX and a stalled scan, which C2 (no RX rings at all) already showed
on r1c2. The table ranks the differences by how likely each explains the full symptom.

| Rank | Difference | Vendor | L19 | Our lane (C2c..C2g) | Likelihood |
|---|---|---|---|---|---|
| 1 | RX flow search table and FISA (FSE_SETUP_CFG with the table in CMEM + 0x1000, FISA_CFG) | sent after ring selection | not sent | not sent | medium: the Android firmware's RX offload engine is built around a host-given flow table; without it the RX classifier may hold or drop frames before RXDMA; non-fatal on the host side, unknown on the firmware side [GUESS] |
| 2 | Cookie conversion over a stale PPT | overwrites the PPT entries it uses, programs REO CFG1 0x1d1310 etc. before INIT | same values, at service ready, no pre-read | dropped every boot (a PPT word non-zero), REO CFG1 left 0x111700 with GLOBAL_ENABLE | medium-low for RXDMA, but it leaves REO in a state no driver configured: GLOBAL_ENABLE with no base and no convert enable [INF] |
| 3 | WMI INIT flag1 IPA_DISABLE (bit 30) | set unless IPA is on | not set | not set (L19 copy) | low-medium: without it an IPA-built firmware may route RX through IPA rings (REO2SW4, HOST2_TO_FW refill 2) the host never set up [GUESS] |
| 4 | RX_RING_SELECTION_CFG layout | no pkt-hdr TLV, packet offset about 272 | pkt-hdr TLV, packet offset 400 | L19's | low: a layout mismatch mis-parses, it does not stop RXDMA |
| 5 | Per-MAC ring size | 1024 (INI 1024..4096) | 2048 | 2048 | low |
| 6 | LRO_CONFIG timing | at first vdev attach | at mac start | off by default (it stalled us) | low for RX; explains the earlier stall only |
| 7 | flag1 HTT_H2T_NO_HTC_HDR_LEN_IN_MSG_LEN | set if service 176 | not set | not set | very low: only HTT TX_FRM lengths change |
| 8 | MSI words, doorbells, wake | 0 in poll mode, no doorbell, force wake | MSI | 0, no doorbell, wake held | very low |
| 9 | Order of HTT ring announcement | after WMI READY | after HTT version | after VERSION_CONF, before the channel list | very low |

### Next rungs, behind a new fuxiwlanw20 bit, default off (spec only; nothing implemented here)

Proposed bit: w20 0x8000 `VENDORRX`, qrtr 7 only, needing 0x4000 (the C2f order) and 0x100. Sub-pieces selected by a
new `fuxiwlanvrx` mask in the style of `fuxiwlanw29` (absent = 0x03), so the boot can bisect them:

- vrx 0x01, cookie conversion over a stale PPT (rank 2). In `wl29_cc_init`, with the REO path and the common rings ours,
  the region valid and `wlcm_cc_owned` free, a non-zero PPT pre-read no longer refuses: print the C2h line, then write the
  280 entries we use (indices 0..279, page IOVA >> 12) exactly as now, which is what the vendor's init does over a stale
  table. Then C2f's six cc_config writes at service ready, unchanged: REO CFG0 = CMEM base (0x113000), REO CFG1 =
  0x1d1310, WBM CFG0 = 0x113000, WBM CFG1 = 0x51310, WBM CFG2 |= 0xb (0x40 -> 0x4b), convert config 0x13a. Still refused:
  any cfg0 base, the WBM CFG1 enables, convert GLOBAL_EN, any ~0. New line: `W29 ppt overwritten: N stale entries of
  280 replaced (fuxiwlanvrx 0x01)`.
- vrx 0x02, the flow search table and FISA (rank 1). After the two RX_RING_SELECTION_CFG, on the HTT endpoint, no reply:
  RX_FSE_SETUP_CFG (HTT H2T type per `FWAPI/fw/htt.h`, words per `dp_htt.c:4667-4790`): pdev 0, base lo = CMEM base +
  0x1000 (0x114000), base hi = 0x10, max entries and search depth from the vendor defaults, IPv4-compatible-IPv6 prefix,
  a fixed Toeplitz key (our own constant, not a secret); the 16 KiB at 0x114000..0x117fff pre-read first through a new
  read allowlist and left as found (whether the vendor host ever writes table entries in CMEM mode was not settled by
  this study [GUESS that it does not]; the rung writes none). Then RX_FISA_CFG: pdev 0, timeout 0xffffffff. Lines: `W29 fse setup sent base 0x114000
  entries E depth D`, `W29 fisa cfg sent`.
- vrx 0x04, IPA_DISABLE (rank 3), changes C1: WMI INIT flag1 |= 0x40000000. Separate bit because it touches INIT; new
  line `W21 init flag1 X (IPA_DISABLE)`.

Readouts that decide the next step: the C2h line (whose pages, how many), whether MGMT_RX appears and the scan completes
with 0x02, and the W27/W28 watches. If 0x02 alone completes the scan, the flow table was the stall; if nothing changes,
the next place to look is the WMI side of MGMT_RX (scan filters and service bits the Android firmware wants), not the RX
rings.

## C2h: the read-only PPT summary line (w20 0x4000)

When `wl29_cc_init` pre-reads the 280 PPT entries (w20 0x4000 with fuxiwlanw29 0x02, the REO path and the common rings
ours, the region valid, the six cc registers not an owner) and any is non-zero, one line before the existing refusal:

```
FUXI WLAN: W29 ppt before: base B non-zero N of 280 (all-ones U) first I last L steps S; samples [i]=V ...; nothing written
```

N counts non-zero entries among the 280 read, U those that read ~0 (unreadable), I and L the first and last non-zero
index, S the non-zero entries equal to their non-zero predecessor + 1 (4 KiB pages laid out one after another), and up to
four samples: the non-zero entries of ordinal 0, N/3, 2N/3 and N-1, no repeat. It is built from the values already read
through the CMEM allowlist (`wlcm_ppt_summary` in `fuxi_wlan_ce.h`, `wl29_ppt_before` in `fuxi_wlan.c`): no further CMEM,
register or memory access, nothing written. The widest form is 191 bytes. All-zero PPT: no line, cc programmed as before.
Model: `s_c2h_ppt` (contiguous 280 entries, two sparse entries with one ~0, a single entry, a gap, all ~0; exactly the 280
pre-reads, no write; the all-zero case prints nothing); five mutants in `C2F_MUTANTS`. On the phone the line should show
the vendor's TX pool pages: runs from index 0 and 128 (section c).
