# Management RX study: MGMT RX REO, and what else the vendor sends around a scan (C2h, `fuxiwlanvrx`)

Study plus code behind a new default-off knob. Question: on the phone (a) no WMI MGMT_RX (beacon) ever arrives, and
(b) once host RX rings exist (w20 0x100) the RX engine runs and the passive scan stalls after 3 to 15 channels with the
firmware alive (M0, no SYSERR) and no further WMI event. Study K (`docs-lanes/wlan-kiwi-rx-study.md` on branch
`study/kiwi-rx`, 954d5ec) showed the RX-ring readings themselves are what the vendor would see. The leading hypothesis
going in: our QMI HOST_CAP MLO TLVs (this firmware fails an allocation without them) make the firmware run MLO
management-RX reordering (MGMT RX REO) and hold management frames until the host takes part. This study tests that
hypothesis against the vendor's own driver and against L19, and lists every other vendor command or flag around a scan
that bears on management-frame delivery. Its four candidate pieces are implemented behind `fuxiwlanvrx` (C2h, W30 lines;
the code and tests are described in `docs-lanes/wlan-w20.md`, section C2h).

## Sources and licences

Vendor tree: the OnePlus 11 (SM8550) vendor modules, github.com/LineageOS/android_kernel_oneplus_sm8550-modules,
commit 9c16bd242fa00a4508c98908160602a314f20f0d (codelinaro and googlesource were not used; the GitHub mirror is the
one Study K read). Path abbreviations:

- `CMN` = `qcom/opensource/wlan/qca-wifi-host-cmn`
- `CLD` = `qcom/opensource/wlan/qcacld-3.0`
- `FWAPI` = `qcom/opensource/wlan/fw-api`

Every vendor source file cited carries the ISC permission notice ("Permission to use, copy, modify, and/or distribute
this software for any purpose with or without fee is hereby granted..."), checked file by file; the host test
re-checks the notice of the 14 files it re-derives constants from (`vendor_c2h` in `tools/test-fuxi-wlan-ce.py`).
`CLD/Kbuild` and `CLD/configs/*` are make fragments with no licence header; they are cited only for which objects and
flags are built. No GPL file was opened (no cnss2, MHI, QRTR or QMI core). Nothing is copied: behaviour is described
with file:line cites; the code is our own. The Toeplitz key the vendor sends is data (it is also the well-known RSS
verification key), written down as ten words.

L19 = Linux ath12k v6.19 (BSD-3-Clause-Clear), `drivers/net/wireless/ath/ath12k`, tag v6.19 (05f7e89).

Marks: [READ] read in the code; [INF] inferred from code; [GUESS] plausible, not shown.

## 1. MGMT RX REO in the vendor driver

### What it is

- A host-side reordering of management frames across MLO links. The firmware forwards each management frame up as it
  always does, tagged with a per-link packet counter and a global (cross-chip) timestamp; the host holds frames in a
  reorder list until it can prove no earlier frame is still in flight on another link, and only then delivers them.
  [READ]
  - The tag is a `wmi_mgmt_rx_reo_params` TLV after the frame in MGMT_RX: global timestamp, link id, counter valid,
    packet counter, PPDU duration, MPDU end time (`FWAPI/fw/wmi_unified.h:7030-7059`, its place in the event:
    `7166-7176`; TLV tag 0x3d2).
  - Frames the firmware keeps or drops itself are reported with `WMI_MGMT_RX_FW_CONSUMED_EVENTID` (event 0x700d, group
    MGMT event 13; `wmi_unified.h:2105-2106`, struct `wmi_mgmt_rx_fw_consumed_hdr` 6985-7028, TLV tag 0x3d3: TSF,
    pdev, ML peer id / link id / subtype in `peer_info_subtype`, frequency, global timestamp, counter), so the host
    can account for the hole in the counter sequence. [READ]
  - Three snapshots per link live in shared memory, written by firmware and hardware and read by the host: FW consumed,
    FW forwarded and MAC HW (`CMN/umac/cmn_services/mgmt_txrx/dispatcher/inc/wlan_mgmt_txrx_rx_reo_public_structs.h:24-73`;
    their bit layouts `FWAPI/fw/wlan_defs.h:1404-1540`). The memory is the MLO global host shared-memory arena
    (`CMN/target_if/init_deinit/src/mlo_global_h_shmem_arena.c:782-811`, `mlo_global_h_shmem_arena_api.c:133-176`,
    both under `WLAN_MLO_GLOBAL_SHMEM_SUPPORT`). [READ]
  - The host algorithm: per frame a wait count per link from the snapshots
    (`CMN/umac/cmn_services/mgmt_txrx/core/src/wlan_mgmt_txrx_rx_reo.c:965-1000`,
    `wlan_mgmt_rx_reo_algo_calculate_wait_count`), the entry point `wlan_mgmt_rx_reo_algo_entry` (3329-3454) queuing
    or releasing the frame. The holding is in the host, never in the firmware. [READ]
- Host to firmware there is exactly one command: `WMI_MGMT_RX_REO_FILTER_CONFIGURATION_CMDID`
  (`wmi_unified.h:890`; fixed param pdev id plus a 64-bit type/subtype bitmap, 7178-7193), encoded by
  `send_mgmt_rx_reo_filter_config_cmd_tlv` (`CMN/wmi/src/wmi_unified_tlv.c:13147-13195`). Its only caller chain is
  `target_if_mgmt_rx_reo_filter_config` (`CMN/target_if/mgmt_txrx/src/target_if_mgmt_txrx_rx_reo.c:519-542`) and
  `tgt_mgmt_rx_reo_filter_config` (`.../dispatcher/src/wlan_mgmt_txrx_rx_reo_tgt_api.c:180-196`), which nothing in this
  tree calls. [READ; no caller by search]

### When it is enabled on kiwi

- Firmware side: `WMI_SERVICE_MGMT_RX_REO_SUPPORTED` = 299 (`FWAPI/fw/wmi_services.h:545`), an advertisement. The vendor
  maps it (`wmi_unified_tlv.c:21500-21501`) and turns it into the soc flag `WLAN_SOC_F_MGMT_RX_REO_CAPABLE`
  (`CMN/target_if/init_deinit/src/init_event_handler.c:333-336`); the target_if MGMT_RX path extracts REO params only
  when that flag is set (`target_if_mgmt_txrx_rx_reo.c:567-569`). [READ]
- No WMI INIT resource-config bit, no flags2 bit and no host_service_flags bit enables or disables management
  reordering: none of the `WMI_RSRC_CFG_*` setters names it (`wmi_unified.h:5206-5500`), and the vendor's
  `wmi_copy_resource_config` sets nothing for it (`wmi_unified_tlv.c:9034-9314`). [READ; by search]
- The QMI HOST_CAP MLO TLVs are filled by cnss2, a GPL platform driver not opened here. What the vendor host does
  with MLO on kiwi is visible in its build instead: `CONFIG_WLAN_FEATURE_11BE_MLO := y` (`CLD/configs/kiwi_v2_defconfig:33-34`)
  builds the MLO manager (`CLD/Kbuild:1417-1435`) but not its multi-chip setup (`wlan_mlo_mgr_setup.c` is not in the
  object list; `WLAN_MLO_MULTI_CHIP` is never defined, so `target_if_mlo_setup_req` and `target_if_mlo_ready`,
  `CMN/target_if/core/src/target_if_main.c:1001-1123`, and the WMI MLO_SETUP / MLO_READY encoders,
  `CMN/wmi/src/wmi_unified_11be_tlv.c:1462-1651`, are compiled out). [READ]

### Is it built for the phone at all? No.

- `CLD/Kbuild:1382-1391`: the management TX/RX objects are `wlan_mgmt_txrx_main.o`, `wlan_mgmt_txrx_utils_api.o` and
  `wlan_mgmt_txrx_tgt_api.o` only; `wlan_mgmt_txrx_rx_reo*.o`, `target_if_mgmt_txrx_rx_reo.o` and the shared-memory
  arena objects are not built, and `WLAN_MGMT_RX_REO_SUPPORT` / `WLAN_MLO_GLOBAL_SHMEM_SUPPORT` are defined nowhere in
  `CLD/Kbuild` or `CLD/configs`. Every REO function above sits under `#ifdef WLAN_MGMT_RX_REO_SUPPORT`
  (e.g. `wmi_unified_tlv.c:13046`, `20472-20477`; `target_if_main.c:547-558`). [READ]
- So the Android driver that this very firmware was built for takes no part in MGMT RX REO: it sends no filter,
  allocates no snapshot memory, reads no snapshot, ignores the REO TLV in MGMT_RX and never handles FW_CONSUMED. And
  Android receives beacons with it. [READ for the build; INF for the conclusion]

### What the firmware does when the host does not take part

- The design puts all holding in the host (above), and the firmware still forwards every management frame it does not
  consume; FW_CONSUMED only reports frames it kept anyway (for example ones it answers itself). Nothing in the vendor
  interface makes delivery wait for a host acknowledgement: there is no host-to-firmware "consumed" or "release"
  message, only the optional filter. [READ for the interface; INF that the firmware never waits]
- The only alternative management path is opt-in: flags2 bit 21 `IS_MGMT_SRNG_ENABLED`, "enable new MGMT SRNG for
  beacons and probe responses" (`wmi_unified.h:4856-4860`, `5446-5450`), with `WMI_MGMT_SRNG_REAP_EVENTID` (0x700e,
  2108-2109). The vendor never sets it (its setter has no caller in `CMN` or `CLD`), nor L19, nor we (our flags2 is
  `CALC_NEXT_DTIM_COUNT_SET` only). So beacons are meant to arrive as WMI MGMT_RX on all three hosts. [READ; by search]

### L19

- L19 implements none of it for any chip: no REO filter, no snapshot, no REO TLV parse (its MGMT_RX TLV walk keeps only
  the header and the first byte array, `wmi.c:6160-6180`, from 6182, `7113-7135`), and it drops FW_CONSUMED silently
  ("debug might flood", `wmi.c:9943-9946`; id `wmi.h:796`). [READ]
- L19 sends the same HOST_CAP MLO TLVs we send when MLO is on (`qmi.c:2045-2150`, `ath12k_host_cap_parse_mlo`); on
  WCN7850 MLO is on when the PHY_CAP reply says `single_chip_mlo_support` (`qmi.c:2271-2272`, `core.c:2130-2155`). It
  allocates any `MLO_GLOBAL_MEM_REGION_TYPE` the firmware asks for (`qmi.c:2616-2650`, `qmi.h:179`), and it sends WMI
  MLO_SETUP and MLO_READY per radio (`core.c:1083-1104`, 1123; `mac.c:14595-14681`; `wmi.c:10837-10900`). Beacons reach
  mac80211 on WCN7850 all the same. [READ; INF that L19's success shows the firmware needs no REO host part]
- Why L19 works without it: the reordering is a host-side service for multi-link delivery order; a host that ignores
  the REO TLV simply receives frames in firmware order. [INF]

### Conclusion for the hypothesis

The vendor evidence contradicts it: management reordering cannot be holding our beacons, because (1) its holding is
done by the host, (2) the phone's own vendor driver does not compile it and still receives beacons from this firmware,
(3) nothing in WMI INIT or QMI turns it on or off, and (4) L19 ignores it too. There is no clean host opt-out because
there is no host opt-in. Our QMI MLO TLVs equal L19's, and the vendor sends no MLO_SETUP / MLO_READY either, so the MLO
side gives no other candidate. [INF from the READ items]

Piece 0x01 (MGMTREO) therefore sends and writes nothing. It is a read-only readout of what the firmware does with
management frames during our scan: service 299 at service ready, then counts of MGMT_RX events, of those with a REO
params TLV (and a valid counter), of FW_CONSUMED events (with beacon and probe-response subtypes) and of SRNG_REAP
events. Many FW_CONSUMED beacons with zero MGMT_RX would say the firmware consumes the beacons itself (a filter or
scan-config question, not an MLO one); zero of everything says the frames never reach the firmware's management path.

## 2. The other candidates from Study K, read again for this question

### 0x02 CCSTALE: cookie conversion over a stale PPT

- The vendor's PPT is 1024 entries, 4 KiB, at the CMEM base (`CMN/dp/wifi3.0/be/dp_be.h:43-51`); its single-pdev
  deinit clears only host bookkeeping and its init writes the entries it uses over whatever is there, never clearing
  the rest (Study K, `dp_be.c:421-457, 460-497`). [READ]
- Ours: with `fuxiwlanvrx` 0x02, `wl29_cc_init` reads our 280 entries as before; a non-zero one no longer refuses
  cookie conversion. The 280 are written exactly as C2f writes them (`wlcm_ppt_val`, through `wlcm_cmem_ok`), the other
  744 are read through a separate read-only allowlist (`wlvrx_ppt_rest_ok`: inside the 4 KiB table and inside the QMI
  region) and counted, never written; then C2f's six cc_config writes at service ready. Unchanged: the REO and
  common-ring ownership gates, the CMEM region bounds, the six cc register pre-reads (`wlcm_cc_owned`), and an
  unreadable (~0) entry among the 280 still refuses with nothing written.

### 0x04 IPADIS: WMI INIT flag1 bit 30

- `WMI_RSRC_CFG_FLAG_IPA_DISABLE_S` 30, mask 0x40000000, "the target should disable IPA" (`wmi_unified.h:4556-4560`).
  The vendor sets the field every boot to `!ucfg_ipa_is_enabled()` (`CLD/core/wma/src/wma_main.c:237-248`, called at
  671; `wmi_unified_tlv.c:9178-9179`); `WMI_RSRC_CFG_FLAG_SET` clears then sets (`wmi_unified.h:5206-5211`).
  `ucfg_ipa_is_enabled` is the IPA config's enable bit, false without an IPA config or with the IPA component compiled
  out (`CMN/ipa/dispatcher/src/wlan_ipa_ucfg_api.c:57-60`, `.../inc/wlan_ipa_ucfg_api.h:496-499`,
  `CMN/ipa/core/src/wlan_ipa_main.c:90-93`). On the phone IPA is on (`kiwi_v2_defconfig:16-22`); on our host there is no
  IPA, so the vendor rule gives 1. [READ]
- Ours: with 0x04, `wl_w22_run` sets flag1 = `wlvrx_flag1_ipa(flag1, false)` after the encode and before the send
  (0x40020 becomes 0x40040020). L19 never sets it (`wmi.c:3999-4001`).

### 0x08 FSE: HTT RX_FSE_SETUP_CFG and RX_FISA_CFG

- When: `dp_soc_attach_target_wifi3` after the ring selection: `dp_rx_target_fst_config` then `dp_rx_fisa_config`
  (`CMN/dp/wifi3.0/dp_main.c:6918-6945`), FISA only if the FSE setup succeeded. The FST attach needs the
  `dp_rx_fisa_enable` INI, default 1 (`CMN/wlan_cfg/cfg_dp.h:1321-1338`; `CLD/components/dp/core/src/wlan_dp_rx_fst.c:284-297`). [READ]
- Where: the flow table is in CMEM, after the PPT: `DP_FST_MEM_OFFSET_IN_CMEM` = 4 KiB, 16 KiB carved
  (`dp_be.h:53-58`; `dp_be.c:184-205`, allocated after cookie conversion at 832-834). With a CMEM base the host table is
  not allocated or cleared at all (`CMN/hal/wifi3.0/hal_rx_flow.c:399-455`: the DDR branch allocates and zeroes, the
  CMEM branch only records the base), and the setup message carries `base_addr_lo` = the CMEM address, `base_addr_hi`
  = (base >> 32) | 0x10 ("Always use 0x10", `wlan_dp_rx_fst.c:454-460`). The host writes table entries only later, per
  flow, from the data path. [READ; INF that nothing is written before the first data flow]
- RX_FSE_SETUP_CFG (`CMN/dp/wifi3.0/dp_htt.c:4667-4790`; `FWAPI/fw/htt.h:905, 8809-8900, 9241-9363`): 14 words. w0 type
  0x12, pdev 0 ("0 indicates msg is for all LMAC rings"); w1 records 128 (INI default, `cfg_dp.h:438-441`, 1277-1283),
  max search 16 (Beryllium, `CMN/wlan_cfg/wlan_cfg.h:42-46`; set at `wlan_cfg.c:3011`), IP DA/SA prefix 2
  (`HTT_RX_IPV4_COMPATIBLE_IPV6`, `htt.h:8895-8900`; `wlan_dp_rx_fst.c:468`); w2/w3 the address; w4..w13 the 40-byte
  Toeplitz key read as ten native words (`wlan_cfg.c:2414-2420`, `dp_htt.c:4728-4760`). The last word is ORed in whole:
  its 27-bit field check is only `A_ASSERT` (`htt.h:281-282`, 9355-9363), which logs and continues in the vendor build
  (`CLD/uapi/linux/osapi_linux.h:126-142`, `WLAN_DEBUG` from `CLD/Kbuild:3514`), so the phone's firmware receives
  0xfa01acbe there. [READ]
- RX_FISA_CFG (`dp_htt.c:4958-5045`; `htt.h:908, 8902-8990, 9218-9238`): 3 words. w0 type 0x15, pdev 0 (read from the
  zeroed message); w1 FISA_V2 enable 1 and aggregation limit 0xf, = 0x1f; w2 the timeout 0xffffffff
  (`FISA_MAX_TIMEOUT`, `dp_main.c:6752-6764`). Neither asks for an answer; the CMEM-response wait applies only when the
  CMEM base is unknown (`wlan_dp_rx_fst.c:474-488`). [READ]
- The flow search entry is 16 dwords with VALID at offset 0x24 bit 8 (`FWAPI/hw/kiwi/v2/rx_flow_search_entry.h:27,
  146-149`).
- Ours: with 0x08 (needs w20 0x800), `wl30_fse` right after the two RX_RING_SELECTION_CFG: the 128 entries' VALID words
  read through a read-only allowlist (`wlvrx_fst_rd_ok`), nothing written to CMEM, then the two messages with the
  vendor's words. Refused (one line, not fatal, as the vendor's own failure there is not) without a QMI CMEM region or
  with one too small for the 4 KiB PPT plus the 16 KiB table.

## 3. Everything else the vendor sends before or around a scan that bears on management delivery

Compared with our INIT words (`wlwmi_encode_init`, `fuxi_wlan_ce.h`) and our C2 scan sequence (`wl_w23_run`), with L19.

| What | Vendor (kiwi) | L19 | Ours | Bears on mgmt delivery? |
|---|---|---|---|---|
| MGMT RX REO filter, snapshots | not built (section 1) | none | none | no |
| flags2 bit 21 MGMT SRNG | never set (`wmi_unified.h:4856-4860`) | not set | not set | no: WMI MGMT_RX is the path on all three |
| WMI MLO_SETUP / MLO_READY | not built (`WLAN_MLO_MULTI_CHIP` off) | sent if single-chip MLO (`core.c:1083-1123`) | not sent | no: the vendor does not send them |
| flag1 IPA_DISABLE (bit 30) | `!ipa-enabled` (section 2) | 0 | 0 (vrx 0x04: 1) | possibly: an IPA-built firmware may route RX to IPA rings [GUESS] |
| flag1 MGMT_COMP_EVT_BUNDLE (bit 9) | 1 (`wma_main.c:643`) | 0 | 0 | no: management TX completions only |
| flag1 TX_MSDU_ID_NEW_PARTITION | 1 (`wma_main.c:644`) | 0 | 0 | no: TX descriptor ids |
| flag1 HTT_H2T_NO_HTC_HDR_LEN (bit 19) | if service 176 (Study K) | 0 | 0 | no: HTT TX_FRM lengths |
| host_service_flags bit 1 MULTI_RADIO_EVTS | 1 (`wmi_unified_tlv.c:9260-9261`) | 0 | 0 | no: RADIO_LINK_STATS splitting (`wmi_unified.h:4888-4897`) |
| WMI SET_FEATURE_SET | after READY when the service is set (`wma_main.c:7429-7430`) | not sent | not sent | no by name; capability report [INF] |
| DFS phyerr offload enable | after READY if offloaded (`wma_main.c:7442-7444`) | at mac start | w29 0x10 | no |
| START_SCAN control flags | PASSIVE, OFDM, 2/5 GHz, chan-stat by INI (`CMN/umac/scan/dispatcher/src/wlan_scan_ucfg_api.c:798-805`), and for a normal scan CCK, BCAST_PROBE, FILTER_PROBE_REQ, DS and TPC IE (`CMN/umac/scan/core/src/wlan_scan_manager.c:1180-1186`); encoded at `wmi_unified_tlv.c:3765-3796` | PASSIVE, CHAN_STAT (`wmi.c:2564, 2602-2603`) | 0x11 = PASSIVE, CHAN_STAT | no: FILTER_PROBE_REQ (0x20) only filters probe requests, the rate and IE flags shape probe requests we never send (`wmi_unified.h:6366-6385`); promiscuous (0x100) is set by no one |
| RX_RING_SELECTION_CFG filter | no FP management (Study K) | no FP management | L19's | no: management goes by WMI |
| HTT RX_FSE_SETUP_CFG / RX_FISA_CFG | sent after the ring selection (section 2) | not sent | not sent (vrx 0x08: sent) | indirectly: the RX classifier the engine runs once host rings exist [GUESS] |
| Cookie conversion over a stale PPT | overwrites | writes, no pre-read | refused (vrx 0x02: overwrites) | indirectly: REO state with GLOBAL_ENABLE and no base [GUESS] |
| Mgmt frame registration | host-side only (umac mgmt_txrx callbacks; no WMI command) | none | none | no |

Nothing in the vendor's pre-scan traffic is a management-RX enable we lack: by the vendor's own design beacons come up
as WMI MGMT_RX unless the firmware consumes them, and nothing the vendor sends asks for them. [INF]

## 4. Ranking for the next kits

Symptom (a) alone (no MGMT_RX with no RX rings) is not explained by any piece; 0x01 is the readout for it. Symptom (b)
(RX engine runs, scan stalls, firmware alive, events stop) points at the RX engine path between the PHY and RXDMA,
since RXDMA writes nothing to our rings (Study K). Ranked by how likely each changes (b):

1. **0x08 FSE**: the vendor always programs the RXOLE flow search table on kiwi; without it the RX classifier runs
   with whatever base the firmware has, and a stall upstream of RXDMA with the firmware alive fits. [GUESS]
2. **0x02 CCSTALE**: leaves REO in a configured state instead of the phone's GLOBAL_ENABLE-without-base; matters once
   frames reach REO, which (b) says they do not yet. [GUESS]
3. **0x04 IPADIS**: an IPA-built firmware told IPA is off routes RX to the host rings only. [GUESS]
4. **0x01 MGMTREO**: changes nothing (read only); the counts decide between "firmware consumes beacons" and "beacons
   never reach the management path". It can be ORed into any kit without changing what is sent.

Kits are in `docs-lanes/wlan-w20.md`, C2h.

## 5. Safety

Unchanged: allowlists, pre-reads and refusals (two new read-only CMEM allowlists, `wlvrx_ppt_rest_ok` and
`wlvrx_fst_rd_ok`; the only CMEM writes stay C2f's 280 through `wlcm_cmem_ok`); device-writable memory only in the
arenas; frees only after bus master off and PERST; no IRQ or MSI; no PMIC or voltage write; nothing transmits (the two
HTT messages are configuration, the scan stays PASSIVE, no probe request, no TX descriptor); every W30 line within
254 bytes (the static bound and the model's line check).
