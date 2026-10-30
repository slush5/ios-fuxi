# W20-W24: copy engines, HTC and WMI init to WMI READY (WCN7850, `fuxiwlanqrtr=6`, C1), one passive scan (`fuxiwlanqrtr=7`, C2)

Status: C1 (c742af3) passed on the phone; C2 (39c80e5) ran clean on the phone (kit r1c2: 38 channels
tuned, CHAN_INFO on all of them, no MGMT_RX at all); C2b (CHAN_INFO detail and a pdev stats request per
PHY pdev on top of C2) is host-tested only (tools/test-fuxi-wlan-ce.py). Every line below that says what the target
does is what the model does, built from Linux ath12k v6.19. C2 and C2b are described in their own
sections below ([C2: W23/W24](#c2-w23w24-one-passive-scan-fuxiwlanqrtr7),
[C2b](#c2b-chan_info-detail-and-the-pdev-stats-requests-fuxiwlanqrtr7-diagnostic)).

After this boot's W15 WLAN_MODE reply, the radio lane programs the host side of copy engines CE0..CE4 on
SRNG rings, posts RX buffers, completes HTC (READY, HTT and WMI connect, SETUP_COMPLETE_EX) and WMI init
(SERVICE_READY, SERVICE_AVAILABLE, EXT, EXT2, INIT with the WCN7850 resource config, READY, HW mode DBS),
all by polling, then stops and contains as every run does (wl5_abort). No interrupt, MSI or GIC
programming, no pdev parameter, no vdev, no RF activity; nothing is transmitted on the air.

Code: `xnu/fuxi-wlan/fuxi_wlan_ce.h` (pure ring/HTC/WMI code, lands as `bsd/dev/fuxi_wlan_ce.h`, carries
the Clear BSD notice and the L19 file list) and the block `/* ---- W20: ... ---- */` in
`xnu/fuxi-wlan/fuxi_wlan.c` (kernel glue). Patch 0099 carries both (the header as its own new-file hunk
after `fuxi_wlan.c`, before `wlan.c`).

L19 = Linux ath12k v6.19 (05f7e89ab), `drivers/net/wireless/ath/ath12k`, BSD-3-Clause-Clear. Every
file:line below is into that tree. UniXNU-sheng was not used for any line of code.

## Knobs and gating

| Knob | Values | Effect |
|---|---|---|
| `fuxiwlanqrtr=6` | | W11..W15 as `=5`, then W20..W22. Needs all 28 steps (`fuxiwlan=1` or `28`); refused with `fuxiwlankeep=1` or `fuxiwlanstop22=1`. The W15 knobs (`fuxiwlanw15`, `fuxiwlanw16`) apply. |
| `fuxiwlanqrtr=7` | | C2: all of `=6`, then W23/W24 (one passive scan). Same gate and refusals as `=6`; on salami also `fuxiwlansalami=1`. |
| `fuxiwlanqrtr` outside 0..7 | 8 (C3), negative, 9+ | refused (`W20 fuxiwlanqrtr=N is not 0..7 in this build; refused`). Before C1 these ran silently with no QRTR. |
| `fuxiwlanw20=N` | 0 default | With qrtr 6 or 7: 0x01 NOHTT (no HTT connect; WMI then gets the next endpoint), 0x04 MRRS128 (endpoint PCIe Device Control max read request 128 bytes before CE init, low half only), 0x40 NOHWMODE (no PDEV_SET_HW_MODE). With qrtr 7 only: 0x02 HTTVER (HTT version request after HW mode), 0x10 BAND24 (the 13 2.4 GHz channels only), 0x80 PDEVCAP (the EXT MAC/PHY pdev id instead of 0). Refused: 0x08 ACTIVE (would transmit probe requests) and 0x20 FULLBSSID (would log whole BSSIDs; the owner masks every BSSID) at every qrtr, the qrtr-7 bits with qrtr 6, 0x01 together with 0x02, any other bit. |

Refusals set `wl_variant_invalid`; salami refuses at init before any access, the Fuxi build at step 17.
Every boot without `fuxiwlanqrtr=6` or `7` logs and behaves as before C1, and every boot with qrtr absent
or 0..6 logs and behaves as before C2 (three differentials, see the C2 section).

## Sequence (L19 core.c:1274-1300, 868-960)

1. W15 MODE reply; with qrtr 6 the MODE line ends `; CE/HTC next` and the observation is 200 ms (not 2000).
2. `wl_w20_run` (stage W20): optional MRRS128; the arena (512 KiB, aligned to its size, Normal-NC, mapped
   leaf by leaf with live SMMU readback by `wl12_map_target`); seven rings in L19 order (ce.c:912-971) with
   readback of BASE_LSB and MISC; HTC init; 62 + 62 RX buffers; HTC READY (3000 ms; L19 waits 1 s,
   services the CEs and waits 1 s more, htc.c:491-515).
3. `wl_w21_run` (stage W21): connect HTT (unless 0x01) then WMI, SETUP_COMPLETE_EX, SERVICE_READY (5000 ms),
   SERVICE_AVAILABLE (optional, up to 1000 ms), SERVICE_READY_EXT (5000 ms, always: L19 completes service
   ready only from EXT or EXT2, wmi.c:4940-4941, 5645), EXT2 when service 220 is set (5000 ms).
   `num_mem_reqs != 0` stops the run (a plan rule, overnight_plan.md 4.4; the failure line says "plan rule,
   not L19"): L19 never reads it and never hands memory chunks to INIT (wmi.c:4230, 4077-4083).
4. `wl_w22_run` (stage W22): INIT, READY (5000 ms; its status is logged, not acted on, as L19 wmi.c:6910),
   HW mode DBS (unless 0x40),
   500 ms of polling for the HW mode response and unprompted events, then stop.
5. Containment: `wl_ce_contain` is the first step of `wl5_abort`; it latches "stopping" (no post, send,
   consumption or publish after it). The arena is freed with the other DMA buffers only when MHI reset,
   bus master off, PERST and the SMMU undo are proven; otherwise it is quarantined until reboot.

Poll pass (`wl_ce_poll`): `wl_w11_events` (MHI events, QMI indications, the AMSS/M0/SMMU guard), RX on
CE2 then CE1 (one HTC handler for both, as ce.c:154-170), TX completions on CE0/CE3, `wl_w11_guard`.
1 ms steps while a reply is due, 10 ms while observing. One HTC message in flight at a time (TX drain
after each send, as W11 does). WMI commands wait up to 3000 ms for HTC credits (wmi.c:435-456).

## Expected log of a pass (after the W15 lines)

```
FUXI WLAN: W20 selected qrtr=6 (CE/HTC/WMI init to READY) w20 0 valid 1        (at init)
FUXI WLAN: W12 selected qrtr=6 target memory cap 67108864 bytes                 (at init)
FUXI WLAN: W15 wlan mode normal ok; CE/HTC next
FUXI WLAN: W15 observing indications 200 ms (reported only)
FUXI WLAN: W12 DMA window extended IOVA 0x...-0x... (pool end 0xb0000000), live SMMU/leaves readback verified
FUXI WLAN: W13 target DMA SID 0x1401 CB 6 AP[7:6]=1 RW AttrIdx=0 MAIR0=0xff OSH; CPU Normal-NC
FUXI WLAN: W20 ce arena IOVA 0x... bytes 0x80000 Normal-NC mapped; rdp 0x...
FUXI WLAN: W20 ce shadow 29 30 31 32 33 34 35 (from WLAN_CFG table)
FUXI WLAN: W20 ce ring 64 entries 16 base 0x... misc 0x42 readback ok           (7 ring lines: 64 82 101 83 102 67 68)
FUXI WLAN: W20 ce ring 82 entries 512 base 0x... misc 0x42 dest-ctrl 0x800 readback ok   (DST rings 82 and 83)
FUXI WLAN: W20 ce init ok rings 7 writes 70 window restore checked
FUXI WLAN: W20 ce rx posted CE1 62 CE2 62
FUXI WLAN: W20 htc ready credits C size S max-ep E after T ms
FUXI WLAN: W21 htc svc 0x300 status 0 eid 1 maxmsg M pipes 4/1
FUXI WLAN: W21 htc svc 0x100 status 0 eid 2 maxmsg M pipes 3/2 credits C
FUXI WLAN: W21 htc setup complete sent
FUXI WLAN: W21 wmi service ready fw 0x... rf-chains N mem-reqs 0 scan-chans N
FUXI WLAN: W21 wmi services ext 1 ext2 1 reg-cc-ext 1 (available event 1)
FUXI WLAN: W21 wmi service ready ext hw-modes N caps N pdev P
FUXI WLAN: W21 wmi service ready ext2 bytes N
FUXI WLAN: W21 wmi service ready complete
FUXI WLAN: W22 wmi init sent bytes 332 (vdevs 4 peers 16 chains 0x3 flag1 0x40020 flags2 0x200 host-svc 0x10 mem-chunks 0)
FUXI WLAN: W22 wmi ready status 0 mac-zero 1 peers N
FUXI WLAN: W22 hw mode dbs sent
FUXI WLAN: W22 wmi event 0x3a003 bytes N (logged only)                          (any unprompted event, 16 lines at most)
FUXI WLAN: W22 hw mode response seen; wmi events N unknown N
FUXI WLAN: W22 complete; stop/contain
FUXI WLAN: W15 WLAN start diagnostic BHI_EXECENV 0x2 (...) ...
FUXI WLAN: W20 ce contain posted P reaped R tx T
FUXI WLAN: W11 QRTR BYE sent before containment
FUXI WLAN: abort step 28 MHI reset done, bus master off yes, PERST asserted yes
FUXI WLAN: abort buffers freed
```

Unprompted events may come before or after `hw mode dbs sent`. The `W15 complete` line is not printed
with qrtr 6 (W22 prints its own).

Failure lines: `W20|W21|W22 timeout <stage> (<ms> ms); contain, no retry` or `W2x <step> FAILED: <reason>`,
then the snapshot (`W20 snapshot <stage>: ...`, one line per ring with host HP/TP next to its rdp word, TX
and RX counts, HTC credits per endpoint, the last WMI event, then the six-read WLFW diagnostic), then
`W2x failure: <reason>; contain, no retry` and the normal abort lines. Reasons are the string literals of
the header (`CE STATUS length 0 or above the RX buffer`, `HTC trailer length outside 4..payload`, ...).
The failure ladders are overnight_plan.md section 3.4.

## Register and memory facts (all from L19; `[V]` re-derived by the host test from the tree)

| Fact | Value | L19 |
|---|---|---|
| Host CE config WCN7850 | CE0 SRC 16 x 2048; CE1 DST 512; CE2 DST 64; CE3 SRC 32 x 2048; CE4 SRC 2048 x 256 (DIS_INTR); CE5-8 none [V] | ce.c:145-220 |
| Ring ids | SRC 64+n, DST 81+n, DST_STATUS 100+n [V] | hal.h:419-468 |
| Entry sizes, directions | SRC 4 dw (host produces), DST 2 dw (host produces), STATUS 4 dw (device produces) [V] | hal_desc.h:1543-1547, 1631-1634, 1686-1691; hal.c:85-108 |
| CE blocks | SRC n 0x01b80000 + 0x2000n, DST n 0x01b81000 + 0x2000n [V] | hw.c:1273-1276, hal.c:1376-1402 |
| R0 (host-produced) | BASE_LSB +0, BASE_MSB +4 (addr 39:32, size dw 27:8), ID +8, MISC +0x10, TP_ADDR +0x1c/0x20, CONS_IX0/IX1 +0x30/0x34 [V] | hw.c:1196-1208 minus 0x900; hal.h:68-113, 287-298 |
| R0 (STATUS) | at DST block + 0x58: BASE_MSB +4, ID +8 (id 15:8, size 7:0), MISC +0x10, HP_ADDR +0x14/0x18, PRODUCER_INT +0x24 [V] | hw.c:1244-1253 minus 0x4e4; hal.c:252-295; hal.h:205, 313-320 |
| R2 | HP +0x400, TP +0x404; STATUS HP +0x408, TP +0x40c [V] | hal.h:116-122, 172-176, 209-210 |
| DEST_CTRL | DST block + 0xb0, max length 15:0 = 2048 (read-modify-write after the ring is enabled, then read back) [V] | hal.h:206, 339; hal.c:1620-1636, 2437-2442 |
| MISC | SRNG_ENABLE bit 6; MSI_LOOPCNT_DISABLE bit 1 on host-produced rings only [V] | hal.h:291, 295, 320; hal.c:1798-1801, 1709 |
| Setup order | src: BASE, MSB, ID, IX0, IX1, TP_ADDR, R2 HP=0, R2 TP=0, MISC; status: BASE, MSB, ID, PRODUCER_INT, HP_ADDR, R2 HP=0, R2 TP=0, MISC; ring memory and rdp word zeroed first | hal.c:1638-1807, 2369-2370, 1699, 1787 |
| rdp | one u32 per ring id; TP_ADDR/HP_ADDR = rdp + 4 x id | hal.c:1564-1576, 2378, 2410 |
| Shadow | register i = BAR0 + 0x8fc + 4i, 40 of them; SRC-direction rings shadow R2 HP, STATUS rings R2 TP; CE entries in ath12k_ce_shadow_config order = indices 29..35 of the W15 table [V] | hal.h:38-43; hal.c:2471-2507; ce.c:874-910 |
| Window | 0x310c = 0x40000000 \| (off >> 19), data at BAR0 + 0x80000 + (off & 0x7ffff); no static window on WCN7850 | pci.c:24-29, 163-182, 1201-1285, 1631-1635 |
| Always-on | BAR0 below 0xFE0 needs no wake (shadow registers); above it the W11 device-wake doorbell is held | pci.c:33-39 |
| Descriptors | SRC: addr, hi 8, byte swap bit 9 = 0, gather bit 11 = 0, len 31:16, meta 15:0 = endpoint; DST: addr, hi 8; STATUS: len 31:16, cleared by the host | hal.c:1927-1957; hal_desc.h:1531-1541, 1627-1629, 1676-1684 |
| Free entry | next HP != device TP | hal.c:2063-2094 |
| Barriers | read barrier after a device pointer (dma_rmb), write barrier before a pointer publish (MMIO wmb; mb() before a STATUS TP) | hal.c:2144-2164, 2169-2219 |

Differences from L19, all deliberate:
- No MSI address and no interrupt thresholds: CONS_IX0/IX1 and PRODUCER_INT_SETUP are written 0. L19
  writes batch 1 / timer / low-threshold values for CEs without DIS_INTR (ce.c:578-614, hal.c:1676-1685,
  1752-1769); with no MSI enable those only shape interrupts [I]. If the device never moves an rdp word
  this is the first difference to look at.
- Barriers are `dsb sy` on both sides: at least as strong as L19's dma_rmb and its mb() before a STATUS TP
  (the plan's `dsb ld`/`dsb st` would be weaker than L19 for that TP).
- STATUS length 0 or above 2048, a STATUS pointer beyond the posted buffers, a TX TP that passes the HP
  or moves back, an HTC message on an unconnected endpoint, a trailer record longer than the trailer, and
  malformed TLVs stop the run; L19 warns and drops most of them. A STATUS pointer that moves back k
  entries reads as a forward move of (entries - k): the posted-buffer bound stops it only when that
  exceeds the buffers out (on CE2 only k = 1), otherwise it stops as `CE STATUS length 0` at the first
  entry, already consumed (length cleared) or never written.
- DEST_CTRL is read back after its write (L19 does not): it is the only bound on what the engine writes
  into one 2 KiB RX buffer, so a value that did not take stops the run before any buffer is posted.
- 62 RX buffers per pipe instead of nentries - 2 (ce.c:948-949); the device takes what is posted.

## HTC facts (htc.h, htc.c)

| Fact | Value | L19 |
|---|---|---|
| Header | endpoint 7:0, flags 15:8, payload length 31:16; control byte 0 (RX trailer length) 7:0, sequence 15:8 [V] | htc.h:18-23, 58-61; htc.c:53-72 |
| Flags | TX NEED_CREDIT_UPDATE 0x01 on credit-flow endpoints; RX TRAILER_PRESENT 0x02 [V] | htc.h:48-56 |
| Limits | 9 endpoints, 4096-byte messages, 256-byte control copies [V] | htc.h:219-231, 255-257 |
| EP0 | RSVD_CTRL (0x0001), pseudo-connected, no credit flow; pipes from the service map: up CE0, down CE2 [V] | htc.c:430-444, 575-582, 711-716; pci.c:1006-1046 |
| READY | msg 1, credit count 31:16; credit size 15:0, max endpoints 23:16; both non-zero [V] | htc.h:28-30, 99-102; htc.c:491-555 |
| Connect | msg 2 \| service << 16, flags 15:0: HTT (0x0300) DISABLE_CREDIT_FLOW_CTRL (bit 3); WMI (0x0100) RECV_ALLOC 15:8 = all credits as a u8 (one WMI endpoint for a single-pdev chip) [V] | htc.c:446-489, 557-617, 763-777; wmi.c:10198-10216 |
| Connect reply | 12 bytes: msg 3 \| service << 16; status 7:0, endpoint 15:8, max message 31:16; endpoint gets the credits asked for [V] | htc.h:35-39, 114-118; htc.c:635-719 |
| SETUP_COMPLETE_EX | msg 5, flags 0, max bundled 0 (12 bytes) [V] | htc.h:120-124; htc.c:721-748 |
| Credits | a message costs ceil(total length with header / credit size); trailer record 1 = {eid, credits} reports; a report for endpoint >= 9 ends the record [V] | htc.c:90-103, 144-233 |
| RX checks | endpoint < 9, header + payload <= 4096 and <= the CE transfer, trailer 4..payload; a message with no payload left (trailer only, or empty) dropped on every endpoint | htc.c:252-378, 321-323 |

## WMI facts (wmi.h, wmi.c)

Ids and tags [V by the test's enum resolver]: INIT 0x1, PDEV_SET_HW_MODE 0x401e; events SERVICE_READY 0x1,
READY 0x2, SERVICE_AVAILABLE 0x3, SERVICE_READY_EXT 0x4009, PDEV_SET_HW_MODE_RESP 0x4010,
SERVICE_READY_EXT2 0x4022, REG_CHAN_LIST_CC_EXT 0x3a003; tags ARRAY_UINT32 0x10, ARRAY_STRUCT 0x12,
SERVICE_READY_EVENT 0x20, READY_EVENT 0x23, INIT_CMD 0x4a, RESOURCE_CONFIG 0x4b, SERVICE_READY_EXT_EVENT
0x1ac, PDEV_SET_HW_MODE_CMD 0x203, MAC_PHY_CAPABILITIES 0x210, HW_MODE_CAPABILITIES 0x211,
SOC_MAC_PHY_HW_MODE_CAPS 0x212, SERVICE_AVAILABLE_EVENT 0x22f, SERVICE_READY_EXT2_EVENT 0x334. Services
EXT_MSG 77, EXT2_MSG 220, REG_CC_EXT_EVENT_SUPPORT 281 (wmi.h:2103, 2244, 2252) [V].

| Fact | Value | L19 |
|---|---|---|
| Framing | command/event header u32, id 23:0; TLV header tag 31:16, length 15:0 without the header; a length past the buffer is an error | wmi.h:69-83; wmi.c:198-207, 320-368 |
| SERVICE_READY | TLV 0x20 >= 128 bytes: fw_build word 0, phy_capability 7, num_rf_chains 9, num_mem_reqs 18, max_num_scan_channels 19, txrx_chainmask 29; first ARRAY_UINT32 >= 32 words = services 0..127, 4 per word [V] | wmi.h:2607-2644; wmi.c:132-133, 662-760 |
| SERVICE_AVAILABLE | TLV 0x22f: offset + 4 words = services 128..255, 32 per word; ARRAY_UINT32 = services 256..; every TLV >= 16 bytes | wmi.h:2641-2644, 2855-2858; wmi.c:7677-7744 |
| EXT | TLV 0x1ac; 0x212 num_hw_modes; first ARRAY_STRUCT only 0x211 (at most num_hw_modes, each adds fls(phy map)); second only 0x210 (at most that many); pdev = 15:0 of word 1 | wmi.c:4494-4516, 4847-4949; wmi.h:2659-2691 |
| EXT2 | TLV 0x334 required; arrays not used in C1 | wmi.c:5560-5650 |
| READY | TLV 0x23 >= 52 bytes: MAC at 24, status 32, num_extra_mac_addr 40, num_total_peers 44 [V] | wmi.h:2839-2853; wmi.c:166-167, 6895-6953 |
| INIT | cmd 1; TLV 0x4a {host abi 6 x 0, num_host_mem_chunks 0} = 32 bytes; TLV 0x4b resource config = 292 bytes (73 words); ARRAY_STRUCT length 0; no hw-mode TLV for a single-pdev chip; 332 bytes in all [V] | wmi.c:4018-4124, 4218-4243; wmi.h:2512-2605 |
| Resource config | init_wcn7850 through copy_resource_config: vdevs 4, peers 16, offload peers 3, offload reorder buffs 3, peer keys 2, tids 32, ast skid 16, tx/rx chain mask (1 << rf chains) - 1, rx timeout {100,100,100,40}, decap 1, scan pending 4, bmiss 3, roam 3, roam profiles 8, tx dbg log 1024, gtk 2, msdu desc 0x400, frag entries 0xa, tdls vdevs 1, tdls conn 8, beacon offload 2, multicast filters 0x20, wow filters 0x16, use_pdev_id 1, peer map version 1, flag1 0x40020, flags2 0x200, host_service_flags 0x10 with service 281; every other word 0 [V all 73 words] | wmi.c:260-304, 3941-4016; hw.h:33-65; wmi.h:2527-2532 |
| HW mode | cmd 0x401e, TLV 0x203 {pdev SOC 0, mode DBS 1, num_band_to_mac 0}; L19 does not wait for the response | wmi.c:4186-4216; core.c:946-953; wmi.h:192-194, 2497-2502, 3163 |
| Waits | service ready and READY 5 s, a command's credits 3 s | wmi.h:5157-5158 |

Corrections to the plan's `[I]` items, found in L19:
- `rx_batchmode` is set to 1 by `ath12k_wmi_init_wcn7850` (hw.h:64) but `ath12k_wmi_copy_resource_config`
  never copies it, so it is 0 on the wire. `vow_config` (TARGET_VOW_CONFIG 0, hw.h:58), `atf_config`, `twt`
  and `ema` are copied (wmi.c:3974, 3999-4005, 4013-4014) but are 0 for WCN7850. The wire list of the plan
  is right; the test checks all 73 words against L19.
- EXT is awaited whether or not service 77 is set (L19 has no completion without it).
- The INIT, READY, SERVICE_READY and SERVICE_AVAILABLE sizes and word offsets, the R0 offsets and the
  service bit numbers are now `[V]` from L19 (previously from L66).
- A failed `num_mem_reqs != 0` stop is our rule, not L19's (L19 never reads it). The WMI READY status is
  only logged: L19 stores it (wmi.c:6910) and never acts on it.

## C2: W23/W24, one passive scan (`fuxiwlanqrtr=7`)

Code: the second half of `xnu/fuxi-wlan/fuxi_wlan_ce.h` (HTT version request, the W23/W24 section: ids,
tags, encoders, SCAN_EVENT/CHAN_INFO/MGMT_RX parsers, the 802.11 element walk, the BSS table, the SSID
text) and the `/* ---- W23/W24 ... ---- */` part of the W20 block of `fuxi_wlan.c` (`wl_w23_run`,
`wl24_report`, `wl23_stop_scan`). L19 file:line references as above; also mac.c, reg.c, core.h, dp.h,
dp_tx.c, dp_rx.c of the same tree (their notices are in the header).

Owner decisions applied: passive only, nothing transmitted (no probe request, no ACTIVE bit; w20 0x08
stays refused); the vdev MAC is the fixed locally administered address the wifi-ios side gives en0
(`02:46:55:58:49:01`, `fuxi_wifi_ios.c`, both backends; the test checks they are the same bytes); the
proof log prints SSIDs in clear and every BSSID as `aa:bb:cc:xx:xx:xx`, with no exception (the planned
w20 0x20 FULLBSSID is refused at every qrtr); no data path, no peer, no keys; reception only on the 38
channels below, no 6 GHz.

### Sequence (after `W22 complete; W23 scan next`)

1. w20 0x02 only: HTT VERSION_REQ on the HTT endpoint (CE4 up), VERSION_CONF on CE1, 3000 ms, major must
   be 3 (dp_tx.c:1275-1326, dp_rx.c:1931-1942; L19 sends it right after HW mode, core.c:955). CE4 gets 16
   slots of 256 bytes for it (a send while all 16 are in flight is refused); its completions are polled
   only once something was sent there, and the contain and snapshot lines count it.
2. SCAN_CHAN_LIST of the 38 channels, pdev 0 (reg.c:183-250, wmi.c:2880-2997). Split as L19 splits it
   when the WMI endpoint's max message is short (APPEND flag after the first command).
3. VDEV_CREATE (wmi.c:923-1010, mac.c:9727-9780, 10004-10087): vdev 0, STA, subtype NONE, the K-W2 MAC,
   two VDEV_TXRX_STREAMS (2 GHz, 5 GHz: the pdev's chains on each band it has, else 0). No reply exists.
4. What L19 sends right after creating a STA vdev (mac.c:10089-10156), in its order: VDEV_SET_PARAM NSS
   (popcount of the tx chain mask, at least 1), STA_POWERSAVE_PARAM RX_WAKE_POLICY=WAKE,
   TX_WAKE_THRESHOLD=ALWAYS, PSPOLL_COUNT=NO_MAX, STA_POWERSAVE_MODE disabled. Left out on purpose: the
   TX/RX encap offload vdev params (data path), the TX power recalculation and the RTS threshold (transmit
   only), the 11d scan (regulatory; it would also raise the scan priority, wmi.c:2708-2711), the DP vdev
   attach (host only).
5. START_SCAN (wmi.c:2541-2830, mac.c:5232-5388), scan id 0xA000, vdev 0, then wait 1000 ms for STARTED
   (mac.c:5046). START_FAILED stops the run (ladder: w20 0x80, then 0x10, then 0x40).
6. Poll (1 ms) until COMPLETED or DEQUEUED, at most max_scan_time + 200 ms (mac.c:5385-5388): SCAN_EVENT,
   CHAN_INFO and MGMT_RX feed the scan state; every new FOREIGN_CHAN prints a line (64 at most); other
   events are logged by id as in W22 (the 16-line budget is shared), except that with qrtr 7 the first
   RSSI dBm conversion event (0x402b) is logged even past the budget, and every one is counted.
7. W24: one line per BSSID (64 at most), the summary, the drop counters; `W23 complete; stop/contain`.
   COMPLETED with no frame prints the W24b hint. A COMPLETED timeout prints the table too, then the
   snapshot (with a scan line) and stops.
8. Containment: `wl_ce_contain` calls `wl23_stop_scan` before the stopping latch. A STOP_SCAN goes out only
   when a START_SCAN may be pending (sent, and no COMPLETED, DEQUEUED or START_FAILED seen), the transport
   is healthy (no access fault, no CE-layer error, HTC ready, and no poll pass saw the W11 events or guard
   fail: AMSS EE 2, M0, no SYSERR, no MHI reset, doorbell bases unchanged, MEM|MASTER, SMMU CB clean;
   scan_plan.md 4 and 6.4; recorded during polling, not read again in containment) and HTC credits are
   in hand (no wait); then a
   TX drain of at most 100 ms, no RX consumption, nothing that can fail the abort. Values of L19's
   `ath12k_scan_stop` (mac.c:4883-4896): requester 0 and vdev 0 (L19 leaves both unset), scan id 0xA000,
   STOP_ONE, pdev. The latch, the MHI reset and the arena rule of C1 follow unchanged.

### Wire facts (re-derived from L19 by the test; the model's encoder dump is compared word by word)

| Command / event | Id, tag | Content | L19 |
|---|---|---|---|
| SCAN_CHAN_LIST | 0x3003, 0x4f + ARRAY_STRUCT of 0x50 | {num, flags (APPEND 1 after the first), pdev}; per channel {mhz, cfreq1 = mhz, cfreq2 0, info, reg_info_1, reg_info_2}, 28 bytes. The array header says 28 x n - 4: L19 writes it with `ath12k_wmi_tlv_cmd_hdr`, which subtracts the TLV header (kept as sent). 38 channels = 1088 bytes. | wmi.c:204-207, 2880-2997; wmi.h:170, 3613-3644, 4001-4006 |
| channel info | | MODE_11G (1) on 2.4 GHz, MODE_11A (0) on 5 GHz, PASSIVE bit 7 on every channel, ALLOW_HE bit 17 (L19 sets allow_ht/vht/he and the else-if chain sends HE only), DFS bit 10 on 52-64 and 100-144 | reg.c:200-221; wmi.c:2942-2963 |
| reg_info | | min 0, max power and max reg power 40 (half dB, 20 dBm), class 0; antenna 0, max tx power 40. 20 dBm is what cfg80211's world domain allows on these channels; L19 doubles the channel values. Nothing is transmitted, so they bound nothing here [I]. | reg.c:212-215; wmi.c:2964-2975 |
| VDEV_CREATE | 0x5001, 0x56 + ARRAY_STRUCT of 0x19b | {vdev 0, type 2 STA, subtype 0, MAC + 2 pad, num_cfg_txrx_streams 2, pdev, mbssid NON_MBSSID_AP 1, tx vdev 0, stats valid 1, stats id 0}, 48 bytes; streams {band 0/1, tx, rx}, 16 bytes each; array length exact here. 88 bytes. | wmi.c:923-1010; wmi.h:2825-2897; mac.c:9667-9780 |
| VDEV_SET_PARAM | 0x5008, 0x5f | {vdev 0, NSS 0x22, nss} | wmi.c:1877-1909; mac.c:10091-10093 |
| STA_POWERSAVE_PARAM | 0x9002, 0x6d | {vdev 0, 0/1/2, 0/1/0} | wmi.c:1815-1846; wmi.h:4642-4678, 4828 |
| STA_POWERSAVE_MODE | 0x9001, 0x6c | {vdev 0, 0 disabled} | wmi.c:1652-1680; wmi.h:3286-3290, 3647 |
| START_SCAN | 0x3001, 0x4d (160 bytes) + 4 arrays | scan id 0xA000, req id 1, vdev 0, priority 0 on the wire (L19 sets `arg->scan_priority` and never copies it), notify 0x5f (L19's 0x1f + START_FAILED), dwell active 50 / passive 150 / active 2g 0 / active 6g 70 / passive 6g 70, rest 50/500, max_scan_time n x 150 x 3/2 (8550 for 38; L19 20000), probe delay 5, flags PASSIVE 0x1 + CHAN_STAT_EVENT 0x10, dwell mode 0, num_chan n, num_bssid 1, no SSID, no IE, n_probes 0; then the n MHz values, 0 SSIDs, the broadcast BSSID + 2 pad, 0 IE bytes. 340 bytes for 38. | wmi.c:2541-2830; wmi.h:3378-3470, 3597; mac.c:5335-5366 |
| STOP_SCAN | 0x3002, 0x4e | {requester 0, scan id 0xA000, STOP_ONE 0, vdev 0, pdev}, 24 bytes | wmi.c:2832-2878; mac.c:4883-4896; wmi.h:3590 |
| HTT VERSION_REQ | HTT message 0 | {0, tag 5 / len 4 / value 2}, 8 bytes; reply VERSION_CONF message 0, minor 15:8, major 23:16 | dp_tx.c:1277-1326; dp.h:433-491, 1338-1359 |
| SCAN_EVENT | 0x3001, 0x24 (28 bytes) | type 0, reason 1, freq 2, scan id 4, vdev 5 (words). STARTED/START_FAILED while starting, FOREIGN_CHAN/COMPLETED while running (L19 ignores COMPLETED before STARTED), DEQUEUED ends the scan, other vdevs ignored | wmi.c:6473-6504, 7289-7368, 6321-6429; wmi.h:3407-3430, 4525-4538 |
| CHAN_INFO | 0x4002, 0x26 (56 bytes) | freq 1, cmd_flags 2 (0 START, 1 END), noise floor 3, vdev 13. START of vdev 0 while running counted, its noise floor kept (min/max) | wmi.c:6587-6623, 7493-7560; wmi.h:4583-4601 |
| MGMT_RX | 0x7001, 0x2c (68 bytes) + ARRAY_BYTE | channel 0, snr 1, buf_len 4, status 5, pdev 15, chan_freq 16; the last header TLV and the first byte array (L19's parse); dropped: no header or frame, buf_len past the byte array (L19 bounds it by the event), status CRC 0x01 / DECRYPT 0x08 / KEY_CACHE_MISS 0x20, channel neither 1-14 nor 36-173, 6 GHz (not on the list). freq = 2407 + 5ch (2484 for 14) or 5000 + 5ch. dBm = snr + noise floor in u32 as L19, noise floor -95 (ATH12K_DEFAULT_NOISE_FLOOR). | wmi.c:6162-6234, 7113-7230; wmi.h:4473-4513; core.h:601-634 |

pdev: L19 sends `ar->pdev->pdev_id`, which for a single-pdev chip (WCN7850) is set to 0 after the
MAC/PHY caps are read (wmi.c:4688-4691, hw.c `single_pdev_only = true`). So pdev 0 is the default;
w20 0x80 sends the EXT MAC/PHY capability's pdev id instead (what `ath12k_mac_get_target_pdev_id` would
pick). overnight_plan.md named this rung PDEV0, assuming the opposite default; the bit and its place in
the ladder are kept (the rung is "the other pdev id"), the code name is WL20_PDEVCAP.

Streams and NSS: L19's pdev ends with the OR of the supported bands of the preferred hw mode's PHYs and
the chain masks of the last of them (2 GHz masks if it has 2 GHz, else 5 GHz; wmi.c:486-553, 4645-4694).
C2 picks that mode as L19 does: a single-pdev chip starts at SINGLE with no PHY (wmi.c:10207-10211) and
a listed mode of strictly better priority (DBS_SBS, DBS, DBS_OR_SBS, SBS, SBS_PASSIVE, SINGLE;
wmi.c:306-318, 4538-4551) replaces it; its PHYs are the MAC/PHY caps from index fls(phy map of the
hw mode entry listed just before it), 0 when it is first (L19 assigns that index, it does not sum;
wmi.c:505-517), one per bit up to fls(its map) (wmi.c:4663-4686) [V: the test re-derives the table and
these code facts, and the model lists SINGLE before and after DBS]. A PHY with neither band is skipped
(L19 fails the whole EXT event there). Without usable caps C2 falls back to SERVICE_READY's rf chains
on both bands and says so in the vdev line.

### 802.11 parsing and the BSS table (IEEE 802.11 layout facts, not code)

Beacon (subtype 8) or probe response (subtype 5) of type management only; other management frames are
counted as `other-mgmt`. BSSID = address 3; capability bit 4 = privacy. Elements are walked from byte 36;
the walk stops at the first element that does not fit (cfg80211's iterator does the same) and counts
`ie-truncated`; elements before it count. SSID: the first SSID element; length 0 or all zero bytes =
hidden, 1..32 = the name, more = invalid (logged as `<invalid length N>`, never printed); no SSID
element (or a cut one) = `<none>`. Channel: DS parameter set, else HT operation, else the MGMT_RX
channel. RSN element present = `rsn 1`. Table: 64 entries by BSSID in order of first reception; a
repeat adds a frame, keeps the strongest dBm with its frequency, ORs the flags and fills a missing or
hidden SSID from a later frame; a frame of a BSSID past the 64th is counted in `table-full frames` (one
per frame, so it overstates the missed networks). `ssids` = distinct visible SSIDs. SSID text: printable ASCII as is, anything else and `"` `\` as `\xNN`.

### Differences from L19, all deliberate

- START_FAILED is subscribed (L19 handles it without subscribing); max_scan_time is n x dwell x 3/2 so the
  radio finishes before a consumer that waits 1000 ms more (overnight_plan.md 3.2).
- No 11d scan, no regulatory domain, no TX power or RTS parameter, no encap/decap offload, no peer, no
  vdev start; the noise floor stays at L19's default (L19 moves it only on the RSSI dBm conversion event
  0x402b, wmi.c:9740-9796; C2 does not apply it, counts it as `rssi-conv events N` in the W24 detail line
  and logs the first one by id even past the 16-line budget, so a non-zero N says the dBm values may
  differ from L19's).
- A SCAN_EVENT, CHAN_INFO or MGMT_RX TLV that runs past its event stops the run (as every TLV overrun in
  C1); a missing or short fixed TLV, or a MGMT_RX that L19 drops, is counted and dropped as L19 does.
- STOP_SCAN in containment is best effort without waiting for COMPLETED (L19 waits 3 s); the MHI reset
  follows.

### Expected log of a C2 PASS (after the C1 lines through `W22 hw mode response`)

```
FUXI WLAN: W20 selected qrtr=7 (CE/HTC/WMI init, then W23 passive scan) w20 0 valid 1        (at init)
FUXI WLAN: W23 selected passive scan of 38 channels, dwell 150 ms, nothing transmitted; BSSIDs OUI only, pdev 0, htt version not asked   (at init)
FUXI WLAN: W12 selected qrtr=7 target memory cap 67108864 bytes                               (at init)
  ... W15 and W20..W22 lines as for qrtr 6 ...
FUXI WLAN: W22 complete; W23 scan next
FUXI WLAN: W23 chan list 38 passive (2.4 GHz 13, 5 GHz 25, dfs 16) pdev 0 commands 1 bytes 1088
FUXI WLAN: W23 vdev 0 sta created mac 02:46:55:58:49:01 pdev 0 bands 0x3 streams tx 2 rx 2
FUXI WLAN: W23 vdev 0 nss 2, sta ps rx-wake 0 tx-wake 1 pspoll 0, ps mode off
FUXI WLAN: W23 scan start id 0xa000 vdev 0 channels 38 passive dwell 150 ms max 8550 ms events 0x5f flags 0x11
FUXI WLAN: W23 scan started after T ms
FUXI WLAN: W23 scan foreign freq 2412                                                          (one per channel, 2412 .. 5825)
FUXI WLAN: W23 scan completed reason 0 after T ms (foreign 38 chan-info 38)
FUXI WLAN: W24 bss aa:bb:cc:xx:xx:xx ch C freq F dBm S privacy P rsn R beacon frames N ssid "NAME"   (one per BSSID)
FUXI WLAN: W24 scan summary frames N bss M ssids K                                             (PASS: K >= 1, the shop's AP listed)
FUXI WLAN: W24 scan detail mgmt-rx N hidden N table-full frames 0 dropped status N channel N malformed N other-mgmt N ie-truncated N noise-floor -95 dBm (L19 default; rssi-conv events N, not applied)
FUXI WLAN: W23 complete; stop/contain
FUXI WLAN: W15 WLAN start diagnostic BHI_EXECENV 0x2 (...) ...
FUXI WLAN: W23 contain: scan over, no STOP_SCAN
FUXI WLAN: W20 ce contain posted P reaped R tx T                                               (tx: CE0 + CE3 + CE4)
FUXI WLAN: W11 QRTR BYE sent before containment
FUXI WLAN: abort step 28 MHI reset done, bus master off yes, PERST asserted yes
FUXI WLAN: abort buffers freed
```

`W23 WMI ... waits for 1 HTC credits (has 0)` lines may sit between the W23 command lines when the
target returns credits late; the unprompted-event lines (`W23 wmi event 0x... bytes N (logged only)`)
may appear anywhere in the scan. Bands, streams, NSS and the counts follow the target. Failure lines:
`W23 timeout scan STARTED (1000 ms)`, `W23 scan START_FAILED reason R (ladder: ...)`,
`W23 timeout scan COMPLETED (N ms); report, contain, no retry` (the W24 lines still follow), `W23 htt
version M.m`, `W23 timeout HTT VERSION_CONF (3000 ms)`, then `W23 failure: <reason>; contain, no retry`
and the containment lines, where `W23 contain: STOP_SCAN sent (best effort), tx drained 1 after N ms`
(`tx drained 0 after 100 ms` when CE3 never completed it) or `... not sent (no HTC credits | transport
not healthy)` or `W23 contain: scan over|start failed, no STOP_SCAN` tell what happened to the scan. `W24 no beacon or probe response heard (chan-info N); next
rung W24b = w20 0x02` is the R7 ladder's W24b trigger (overnight_plan.md 3.4).

### Proof that qrtr 0..6 are unchanged

Three differentials (C2 working tree against HEAD 2a994c5, scripts in the shop's
op11-tools/salami-port/wlan/r1/impl/c2-verify, rerun after the review fixes from impl/c2-fix): the W15
harness log (qrtr 5, keep 0 and 1) byte for byte; the init/configure logs of the boot-arg sets with qrtr
absent or 0..6 (no w20 or w20 0x1, 0x2, 0x3, 0x8, 0x10, 0x20, 0x45, 0x80, 0x92, 0xb2, 0xd6, 0xf6 or -1;
mlo, w15/w16, keep, stop22 and the S4 set; limits 1/11/27/28) on the salami and the Fuxi build, byte for
byte; and HEAD's own C1 host model (46 scenario logs: the whole qrtr 6 run, its failures and wl5_abort)
built against HEAD's and C2's production code, byte for byte.

## C2b: CHAN_INFO detail and the pdev stats requests (`fuxiwlanqrtr=7`, diagnostic)

Why: on the phone C2 tuned all 38 channels (38 FOREIGN_CHAN, 38 CHAN_INFO, COMPLETED after 2602 ms) and
not one WMI MGMT_RX (0x7001) came back. The leading hypothesis: on WCN7850 the MAC DMAs received MPDUs
into host-provided RX buffers (the refill / rx_mac_buf rings set up through HTT SRING_SETUP), which C2
never provides. C2b adds no transmit, no ring and no register access; it makes the next qrtr 7 boot
report what the radio measured on each channel and what the firmware counted on each MAC, so that
hypothesis can be told apart from a receive-chain/filter problem and from an RF-path problem.

Code: `wl23_ci_line`, `wl23_ci_flush`, `wl23_poll_for`, `wl23_stats_pdev`, `wl23_stats` and
`wl23_stats_report` in the W23 part of the W20 block of `fuxi_wlan.c`; in `fuxi_wlan_ce.h` the CHAN_INFO
word enum (C2's parser keeps the 14 words of each well-formed event) and the C2b section (ids, tags,
sizes, the stats event and pdev stats word enums, `wlwmi_encode_stats_req`, `wlstats_parse`); `wlwmi_rx`
routes UPDATE_STATS to the parser once a request is out; `wlwmi_parse_ext` also keeps L19's `fw_pdev[]`
pdev ids (the distinct ones of the preferred hw mode's PHYs, at most two, each with its PHYs'
supported_bands).

No knob. Both additions are host-to-firmware only (WMI commands; log lines), every wait is bounded,
only the transport can fail the run (a failed poll, or a CE/HTC fault in the send, exactly as in
wl_ce_send), and no C2 line changes except the containment counters, which now include the stats
exchanges. A knob would add gate code and a cmdline change for no safety gain (the stats command is the
same kind of WMI traffic C2 already sends, and L19 sends it whenever mac80211 asks for the TX power), and
every w20 bit is taken. The r1c2 cmdline (`fuxiwlanqrtr=7 fuxiwlanw20=2`) runs C2b as it is.

### Sequence

1. During the scan, every well-formed CHAN_INFO (TLV 0x26, at least 56 bytes, any vdev, any flags) gets
   one `W23 chan-info` line right where it arrives (next to the `scan foreign freq` lines), the 14 words
   as received, 128 lines at most (2 x 64); the rest are counted and the count is printed once, as
   `W23 chan-info lines suppressed N (cap 128)`: just before `W23 complete`; after a COMPLETED timeout,
   just before the snapshot; after a failed STARTED wait, scan wait or stats exchange, right after the
   failure lines. A short CHAN_INFO is counted as C2 counts it and gets no line.
2. After COMPLETED (or DEQUEUED), the W24 lines and the no-frame hint, before `W23 complete`: one
   WMI_REQUEST_STATS (0x16001, TLV 0x8f) {stats id PDEV_STAT 0x4, vdev 0, peer MAC 0 + 2 pad, pdev} per
   pdev id of the preferred hw mode's PHYs, in PHY order, at most two, as
   `ath12k_wmi_send_stats_request_cmd` builds it (wmi.c:776-794, 890-920) for L19's debugfs pdev_stats
   reader (debugfs.c:1388-1410). L19 keeps one `fw_pdev` per PHY with its own pdev id (15:0 of word 1) and
   supported_bands (wmi.c:505-532, 4660-4686); `ath12k_mac_get_target_pdev_id` names `fw_pdev[0]` with no
   vif up (mac.c:990-1009) and, with a vif up, the `fw_pdev` whose bands match the vif's channel
   (mac.c:967-986). C2's scan ran on both DBS MACs (2602 ms for 38 x 150 ms of dwell, 2.4 and 5 GHz
   interleaved), so C2b asks `fw_pdev[0]` first, then the other PHY's pdev. Both PHYs on one pdev id: one
   request, their bands ORed. No PHY in the preferred mode (L19 leaves `fw_pdev[0]` zeroed): one request
   for pdev 0. The request line prints the pdev and its bands (1 = 2 GHz, 2 = 5 GHz, wmi.h:220-221). The
   W21 ext line's `pdev` is the first MAC/PHY entry's, which need not be either of these.
3. Each request takes wl_ce_send's path: credits (wait at most 3000 ms, L19's WMI_SEND_TIMEOUT_HZ, with a
   `W23 stats request waits for N HTC credits (has M)` line), frame, slot, publish, TX drain at most
   1000 ms; then at most 500 ms for an UPDATE_STATS (0x1d004) whose parse yields a pdev entry (L19 waits
   1 s for the first event and 3 s for the last, mac.c:5069-5115). A CE or HTC fault in frame, slot or
   publish ends the run as in wl_ce_send (`W23 WMI REQUEST_STATS FAILED`, the W20 snapshot, containment).
   A request that was not sent (no credits) or whose TX did not drain ends the stats phase: no further
   request. No usable answer in 500 ms does not: the next pdev is still asked. An answer that arrived
   although the TX descriptor never drained is printed after the `TX not drained` line ("after 1000 ms").
4. UPDATE_STATS parse as `ath12k_update_stats_event` (wmi.c:8197-8312, 8389-8434, 8488-8545): the TLV walk
   of `ath12k_wmi_tlv_iter` (a TLV past the event ends it), the fixed TLV 0x46 (`struct wmi_stats_event`,
   44 bytes; L19 has no length policy for it, C2b requires the whole struct) before an ARRAY_BYTE, then in
   each ARRAY_BYTE L19's order: num_vdev_stats x 164 bytes, num_bcn_stats x 12, num_pdev_stats x 176, each
   bounded by what is left; the first pdev entry is the one L19's dump prints. An event without one
   (L19 then never completes `fw_stats_done`) is unusable; so is any parse error. Unusable events are
   counted with the last reason and the wait goes on; the first usable one ends it, later ones (the
   firmware may send them back to back) are counted, not taken. Other TLVs (per-chain RSSI,
   ARRAY_STRUCT) are skipped: C2b does not ask for them. Single-pdev chips accept any event pdev id
   (mac.c:830-838); C2b prints it. The first answer line prints num_vdev_stats, num_bcn_stats and the
   byte array's length, the counts that place the pdev entry: L19 only ever asks for one stats type
   (debugfs.c:1405-1407), so its vdev, beacon, pdev order is not exercised with several counts.
5. Every ending prints the CE messages of its exchange (`ce tx X rx Y`); their sum is also in the
   containment line (`W20 ce contain posted P reaped R tx T`: tx + X, posted and reaped + Y). No vdev
   stats: L19 completes those only for started vdevs (wmi.c:8443-8470) and the C2 vdev is never started;
   they are a connected STA's TX and beacon counters anyway.
6. After the requests, one `W23 stats after-scan` line: what the scan parser took after COMPLETED.
   `wl_wmi.scan` stays attached through the exchanges, so a late MGMT_RX, CHAN_INFO or scan event is
   parsed as C2 parses it (MGMT_RX into the BSS table, CHAN_INFO with its detail line) and counted there,
   with the unknown events; the W24 lines are not printed again.

### Lines

```
FUXI WLAN: W23 chan-info freq F flags START|END|0xN nf N rx-clear R cycle C rx-frame X tx-frame T my-bss-rx M mac-clk K
FUXI WLAN: W23 chan-info lines suppressed N (cap 128)                                           (only past 128 events)
  per request (pdev P, at most two):
FUXI WLAN: W23 stats request waits for N HTC credits (has M)                                    (only without credits)
FUXI WLAN: W23 stats request pdev-stat pdev P bands B vdev 0 sent
FUXI WLAN: W23 stats pdev P after T ms (stats-id 0x4 pdevs N vdevs V bcns C bytes L events E unusable U; ce tx X rx Y)
FUXI WLAN: W23 stats base chan-nf N rx-frame N rx-clear N cycle N tx-frame N phy-err N chan-tx-pwr N
FUXI WLAN: W23 stats rx1 status-rcvd N r0-frags N r1-frags N r2-frags N r3-frags N htt-msdus N htt-mpdus N
FUXI WLAN: W23 stats rx2 loc-msdus N loc-mpdus N oversize-amsdu N phy-errs N phy-err-drop N mpdu-errs N mid-ppdu-route-change N
FUXI WLAN: W23 stats tx1 comp-queued N comp-delivered N msdu-enqued N mpdu-enqued N local-enqued N local-freed N hw-queued N
FUXI WLAN: W23 stats tx2 hw-reaped N underrun N tx-abort N mpdus-requed N tx-ko N self-triggers N pdev-resets N
  once, after the requests:
FUXI WLAN: W23 stats after-scan wmi mgmt-rx +N frames +N bss +N scan-events +N chan-info +N other +N
```

Instead of a request's six answer lines: `W23 stats pdev P no usable UPDATE_STATS in 500 ms (events E
unusable U[, last: REASON]; ce tx X rx Y)` (the next pdev follows). Instead of the sent line and the
answer: `W23 stats request pdev-stat pdev P bands B not sent: no HTC credits after 3000 ms (ce tx X rx Y);
continue to containment` (no further request). After the sent line: `W23 stats request TX not drained
after 1000 ms (ce tx X rx Y); continue to containment`, then the six answer lines only if the answer came
anyway (no further request). A CE/HTC fault in the send: `W23 WMI REQUEST_STATS FAILED: WHY`, the W20
snapshot lines, `W23 failure: WHY; contain, no retry`, no after-scan line and no `W23 complete`. REASON is
one of `stats TLV past the event`, `stats fixed TLV shorter than 44 bytes`, `stats byte array before the
fixed TLV`, `stats byte array shorter than its entries`, `no pdev stats in the event`, `no stats fixed
TLV`. XNU's console keeps at most 255 bytes of a printf line, newline included, and drops the rest with
the newline when `drain_uart_sync` is off, as on the kits (osfmk/console/serial_general.c:159-188); that
is why the answer is six lines: the host test computes every C2b format's worst case (10 bytes per `%u`
or `%#x`, 11 per `%d`, the longest `%s`), at most 215 of 254, and the model fails any longer line.

### What the counters are (L19)

CHAN_INFO (`struct wmi_chan_info_event`, wmi.h:4586-4601; word: field): 1 freq, 2 cmd_flags (0
START_RESP, 1 END_RESP), 3 noise_floor (dBm, signed), 4 rx_clear_count, 5 cycle_count, 8
rx_frame_count, 9 my_bss_rx_cycle_count, 11 tx_frame_cnt, 12 mac_clk_mhz (also 0 err_code, 6/7 tx power
range/tp, 10 11b data duration, 13 vdev_id, not printed). L19 builds the channel's survey from a
START_RESP alone: noise, time = cycle_count / (mac_clk_mhz x 1000) ms, busy = rx_clear_count / (mac_clk_mhz
x 1000) ms (with mac_clk_mhz 0: 320000, mac.c:14874), after clearing the entry (wmi.c:7546-7561); an
END_RESP only ends the report (wmi.c:7514-7517). C2b prints every word of both as received and computes
no delta. The counters are MAC-clock cycles: a 150 ms dwell at 320 MHz is about 48,000,000. rx_clear_count
is the time the channel was busy (energy or a preamble on the air, L19's survey "busy"); rx_frame_count the
time spent receiving frames ("Cycles spent receiving frames", wmi.h:5960); tx_frame_cnt the time spent
transmitting (0 for a passive scan); my_bss_rx_cycle_count frames of our own BSS (0: not associated).

Pdev stats (`struct ath12k_wmi_pdev_stats_params`, wmi.h:5957-6014, base 7 + tx 23 + rx 14 words; L19's
debugfs labels, wmi.c:7893-8031): base chan_nf "Channel noise floor" (signed dBm), tx_frame_count,
rx_frame_count, rx_clear_count, cycle_count (same cycle units as above, for the pdev's current channel),
phy_err_count, chan_tx_pwr (half dB). RX: status_rcvd "Tot. number of statuses" (receive status entries
the MAC produced), r0..r3_frags "Extra frags on rings 0..3", htt_msdus/htt_mpdus "MSDUs/MPDUs delivered
to HTT" (to the host data path), loc_msdus/loc_mpdus "... delivered to stack" (kept by the firmware, the
path management frames take to WMI), oversize_amsdu, phy_errs, phy_err_drop, mpdu_errs "MPDU errors
(FCS, MIC, ENC)", mid_ppdu_route_change. TX: comp_queued/comp_delivered "HTT cookies queued/disp.",
msdu_enqued, mpdu_enqued, local_enqued/local_freed (frames the firmware itself queued), hw_queued "HW
queued", hw_reaped "PPDUs reaped", underrun, tx_abort "PPDUs cleaned", mpdus_requed, tx_ko "Excessive
retries", self_triggers, pdev_resets. The a_sle32 fields print signed. L19 shows them as the firmware
sends them (no delta). There is no rx_ovfl_errs in L19's struct (ath12k v6.19 has none), so an RX
overflow shows as heard-but-not-delivered: rx-frame / status-rcvd up, htt and loc MPDUs not. A pdev's
counters are that MAC's only: with DBS each MAC tuned the channels of its own band(s).

### Reading the next boot

First: (a) classify each channel by whichever of its START or END lines has a non-zero cycle count; if
cycle or rx-clear grow steadily from line to line, read END minus START of each channel instead of the
raw values. (b) Compare each pdev's stats only with the CHAN_INFO lines of the band(s) in its request
line's `bands` (0x1: 2412..2472, 0x2: 5180..5825), and check that its answer line names the same pdev.
(c) Take a pdev's fields as printed only with `vdevs 0 bcns 0` and `bytes` = 176 x `pdevs`; otherwise
they may be misplaced: decide from the CHAN_INFO lines.

| Pattern | Conclusion | Next rung |
|---|---|---|
| CHAN_INFO: rx-frame > 0 on the channels where APs are (the shop's at least), rx-clear >= rx-frame, nf about -90..-105 dBm; W24 frames 0 and after-scan mgmt-rx +0; stats of the pdev with that band: status-rcvd > 0 or rx-frame > 0 with htt-mpdus and loc-mpdus 0 | the radio receives and decodes frames, but no MPDU reaches the firmware's management path or the host: the MAC has nowhere to put them (no host RX buffers) | DP RX: HTT SRING_SETUP of the RXDMA refill / rx_mac_buf rings and the RX filter (the rung the research workflow designs) |
| htt-mpdus > 0 with W24 frames 0 | the firmware handed MPDUs to the host data path, which does not exist | DP RX (confirmed) |
| loc-mpdus > 0 on the pdev of the APs' band, MGMT_RX 0 (after-scan mgmt-rx +0 too) | frames reach the firmware and stop there: WMI management forwarding (filter, vdev state) | management-RX forwarding before DP RX |
| after-scan mgmt-rx > 0 with W24 frames 0 | MGMT_RX is delivered, only after COMPLETED: delayed, not filtered | the receive path works: C3, with the W24 report taken after a short wait past COMPLETED |
| rx-frame 0 on every channel, rx-clear > 0, nf sane, cycle about dwell x mac-clk | energy on the air, nothing decoded or accepted: receive chain or RX filter (chain masks, filter, vdev/pdev state) | receive chain/filter: the masks the firmware applied (INIT, VDEV_TXRX_STREAMS, NSS), the pdev RX filter; L19's vdev start path |
| rx-clear 0 and rx-frame 0 on every channel, cycle and nf sane | no energy at all: antenna/FEM path off (or a quiet spot) | RF path: FEM/antenna switch control, BDF; a second boot next to the AP rules out the spot |
| nf absurd (0, positive, below -120, or identical on every channel) or cycle/mac-clk 0 in both START and END | the RF front end or calibration does not deliver | RF path: BDF and calibration data, FEM control, before anything on the host side |
| mpdu-errs or phy-errs large on the pdev of the APs' band, frames 0 | frames arrive damaged (FCS/PHY errors) | RF path (calibration) |
| any of tx-frame (CHAN_INFO or pdev), hw-queued, mpdu-enqued, local-enqued > 0 | something was queued for or sent on the air: the passive rule is broken | stop the radio lane until it is explained |
| no usable UPDATE_STATS for a pdev (not sent, TX stuck, timeout, unusable) | that pdev's stats are not available in this state | decide from the CHAN_INFO lines of its band(s); the request is not retried |
| `W23 WMI REQUEST_STATS FAILED` | a transport fault in the send, as any C2 send fault | the W20 snapshot decides; the stats are not the question then |
| W24 frames > 0 | C2 PASS | C3 |

### Expected log (the model's silent scan, `fuxiwlanw15=1`, after `W23 scan started`)

```
FUXI WLAN: W23 scan foreign freq 2412
FUXI WLAN: W23 chan-info freq 2412 flags START nf -96 rx-clear 1440000 cycle 48000000 rx-frame 0 tx-frame 0 my-bss-rx 0 mac-clk 320
FUXI WLAN: W23 chan-info freq 2412 flags END nf 0 rx-clear 0 cycle 0 rx-frame 0 tx-frame 0 my-bss-rx 0 mac-clk 0
  ... the same for the other 37 channels ...
FUXI WLAN: W23 scan completed reason 0 after 5704 ms (foreign 38 chan-info 38)
FUXI WLAN: W24 scan summary frames 0 bss 0 ssids 0
FUXI WLAN: W24 scan detail mgmt-rx 0 hidden 0 table-full frames 0 dropped status 0 channel 0 malformed 0 other-mgmt 0 ie-truncated 0 noise-floor -95 dBm (L19 default; rssi-conv events 1, not applied)
FUXI WLAN: W24 no beacon or probe response heard (chan-info 38); next rung W24b = w20 0x02
FUXI WLAN: W23 stats request pdev-stat pdev 1 bands 0x1 vdev 0 sent
FUXI WLAN: W23 stats pdev 1 after 2 ms (stats-id 0x4 pdevs 1 vdevs 0 bcns 0 bytes 176 events 1 unusable 0; ce tx 1 rx 1)
FUXI WLAN: W23 stats base chan-nf -98 rx-frame 2457600 rx-clear 5068800 cycle 47961000 tx-frame 0 phy-err 112 chan-tx-pwr 0
FUXI WLAN: W23 stats rx1 status-rcvd 20 r0-frags 0 r1-frags 0 r2-frags 0 r3-frags 0 htt-msdus 0 htt-mpdus 0
FUXI WLAN: W23 stats rx2 loc-msdus 19 loc-mpdus 19 oversize-amsdu 0 phy-errs 112 phy-err-drop 0 mpdu-errs 1 mid-ppdu-route-change 0
FUXI WLAN: W23 stats tx1 comp-queued 0 comp-delivered 0 msdu-enqued 0 mpdu-enqued 0 local-enqued 0 local-freed 0 hw-queued 0
FUXI WLAN: W23 stats tx2 hw-reaped 0 underrun 0 tx-abort 0 mpdus-requed 0 tx-ko 0 self-triggers 0 pdev-resets 0
FUXI WLAN: W23 stats request pdev-stat pdev 2 bands 0x2 vdev 0 sent
FUXI WLAN: W23 stats pdev 2 after 2 ms (stats-id 0x4 pdevs 1 vdevs 0 bcns 0 bytes 176 events 1 unusable 0; ce tx 1 rx 1)
  ... base, rx1, rx2, tx1, tx2 as for pdev 1 ...
FUXI WLAN: W23 stats after-scan wmi mgmt-rx +0 frames +0 bss +0 scan-events +0 chan-info +0 other +0
FUXI WLAN: W23 complete; stop/contain
FUXI WLAN: W23 contain: scan over, no STOP_SCAN
FUXI WLAN: W20 ce contain posted 261 reaped 137 tx 15
```

The values are the model's (its DBS PHYs are pdev 1 for 2.4 GHz and pdev 2 for 5 GHz, and its pdev
answer is the same in every scenario); on the phone each CHAN_INFO line carries the target's counters,
the END_RESP lines may be fewer or absent, there is one request block per distinct pdev id of the
preferred mode's PHYs, and the containment counts include every exchange (`ce tx 1 rx Y` each).

### Proof that only new lines were added

Four differentials against HEAD 39c80e5 (scripts in the shop's
op11-tools/salami-port/wlan/r1/impl/c2b-fix): the W15 harness log (keep 0 and 1) byte for byte; the
init/configure logs of 761 boot-arg sets with qrtr absent or 0..8 (qrtr 7 included: C2b adds no knob)
on the salami and the Fuxi build, byte for byte; HEAD's model with its C1 scenarios (46 logs) against
HEAD's and C2b's production code, byte for byte; and HEAD's model + code against C2b's model + code for
all 65 HEAD scenario logs: the 46 C1 ones byte for byte, the 19 C2 ones differ only by inserted lines
starting with `W23 chan-info ` or `W23 stats `, except that in the 9 that reach the stats exchanges (two
each, one per DBS PHY pdev) the containment line's counters move by exactly the sum of those exchanges'
`ce tx X rx Y`.

## Host tests

`CC=cc python3 tools/test-fuxi-wlan-ce.py` (also in tools/ci/build.sh): the model in
tools/fuxi-wlan-ce-model.c runs the production glue and header under ASan/UBSan with -Wall -Wextra
-Werror. It owns a BAR0 model (window register with readback, shadow registers routed through the same
`wl15_shadow_v3` table, its own list of CE registers, DEST_CTRL pinned to 2048 and written only after its
ring is enabled; any other access aborts), a DMA map of the arena only (each RX message inside its own
2 KiB buffer), a CE engine (descriptor, slot and buffer checks, DST word 1 = address bits 39:32 only,
rdp writes) and a fake firmware with its own credit ledger (header included) that checks INIT and HW
mode field by field and that INIT comes only after EXT/EXT2 reached the host. Scenarios: happy path with
shadow registers and with `fuxiwlanw15=1` (R2 through the window), services 256.. read only from the
4-word array; w20 0x45; credit exhaustion with a trailer-only refund, credit starvation (3 s timeout), u8
credit allocation, credit size 336 (the INIT frame costs 2 with its header); no HTC READY with proven
and unproven containment (arena freed only when proven); STATUS length 0 and 2049; endpoint 9;
unconnected endpoint; long trailer; trailer length 0; HTC payload longer than the CE transfer; TLV past
the buffer; nested overrun; rdp outside the ring, beyond the posted buffers and misaligned; CE2 STATUS
pointer back 1 entry (posted-buffer bound) and back 2 (zero length); TX TP backwards; mem-reqs;
connect refused and connect to endpoint 0; a guard failure; tolerated credit report for endpoint 12, no
EXT2 service, HTC READY on CE1, empty HTC messages on EP0 and the WMI endpoint, READY status 3, READY
with a MAC (bytes 24..29, non-zero ABI words before it), a 2048-byte transfer; EXT 1500 ms late with and
without EXT2; window readback mismatch on select and on restore (no later CE write); a fault latched
after the 5th CE write (no later write); a DEST_CTRL that does not take (no RX buffer posted); after
`wl_ce_contain` no register write, publish, send, RX post or reap; the RX post bound on a fresh layout
and the shadow writer's bounds; ring wrap (140 commands through 32-entry CE3, 148 events through 64-entry
CE2); every rdp load followed by the read barrier and every publish preceded by the write barrier. With
the L19 tree on disk it also re-derives the ids, tags, sizes, all INIT words, the HTC constants and the
CE geometry; on CI (no tree) that part prints SKIPPED.

C2 in the same model: the fake firmware checks every C2 command in L19 order (HTT version on CE4 first
with w20 0x02, the channel list word by word with its APPEND split and the 4-short array header,
VDEV_CREATE with the K-W2 MAC and the streams of its MAC/PHY caps, NSS, the three STA power-save
parameters, power save off, START_SCAN word by word, STOP_SCAN field by field and only before the latch),
refunds credits in trailer-only messages, and plays a scan: STARTED, per channel FOREIGN_CHAN, CHAN_INFO
START/END and the frames heard there, an unprompted event, a SCAN_EVENT of another vdev, COMPLETED.
Scenarios: the happy path (20 MGMT_RX: duplicate BSSID from an adjacent channel and as a probe response,
hidden zero-length and all-zero SSIDs, a 33-byte SSID, an RSN element and an SSID element cut by the
frame end, a frame shorter than its fixed fields, CRC status, channel 0, 6 GHz, a probe request, no byte
array, buf_len past the array, a 64-byte header TLV, HT-only channel, an SSID with UTF-8 and quotes);
w20 0x92 (HTT 3.4, 13 channels, pdev from caps, BSSIDs masked, the CE4 send in the contain line and the
snapshot), HTT major 2 and a silent HTT; the channel list split by a 300-byte max message; caps without
bands, 2.4 GHz-only 1x1 caps, and SINGLE (1x1) listed after DBS (the DBS PHYs still give the streams);
70 BSSIDs (64 kept; 6 past the table in 8 frames = `table-full frames 8`); START_FAILED; no STARTED
(STOP_SCAN with pdev from caps), and the same with a STOP_SCAN whose TX never completes (drain ends at
100 ms); COMPLETED before STARTED (ignored); a W11 guard failure right after START_SCAN (no STOP_SCAN, no
doorbell, no descriptor); no COMPLETED (table, snapshot, STOP_SCAN); no credits left (STOP_SCAN skipped);
a SCAN_EVENT TLV past its event (stop, no STOP_SCAN); a silent scan with fuxiwlanw15=1; 20 unknown events
before the scan (0x402b still logged and counted); and pure checks of the SSID text, the element walk
(including an element body one byte past the frame), the table, the encoder bounds and the CE4 slots (a
17th send with 16 in flight refused). The model's SERVICE_READY_EXT lists SINGLE (PHY map 0x1) and DBS
(0x3) with the three MAC/PHY caps that follow. The L19 part re-derives the C2 ids, tags, flags, sizes and
field offsets and compares every word of the model's dump of the channel list, VDEV_CREATE, NSS,
START_SCAN, STOP_SCAN and the HTT request with L19's struct order and values
(ath12k_wmi_start_scan_init, the passive request of ath12k_mac_initiate_hw_scan, reg.c's channel flags,
the single-pdev pdev 0, the hw mode priority table and the preferred mode's PHY index rule,
ath12k_scan_stop, the STA follow-up order of ath12k_mac_vdev_create, the MGMT_RX drop rules, the default
noise floor). Static: the scan only after W22 with qrtr 7, STOP_SCAN before the latch, without waits or
RX and not after a recorded W11 guard failure, passive flags, no SSID/IE/probe words in the encoder,
ACTIVE and FULLBSSID outside every accepted set, no whole-BSSID path in W24, the vdev MAC equal to en0's. test-fuxi-wlan-salami.py covers the qrtr 7 gate (salami and
Fuxi builds, cmdline and ADT /defaults), test-fuxi-wlan-w11/w15/w16.py the selector lattice up to 8.

C2b in the same model: CHAN_INFO START_RESP carries realistic counters (320 MHz, about 150 ms per dwell,
3 % busy plus 300 us of air time per frame heard there), END_RESP none; the DBS PHYs have their own pdev
ids (1 for 2.4 GHz, 2 for 5 GHz, as each WCN7850 MAC); the fake firmware checks REQUEST_STATS field by
field (PDEV_STAT, vdev 0, MAC 0, the preferred mode's PHY pdev ids in PHY order, each once), only after
COMPLETED and before the latch, and answers with an L19-shaped UPDATE_STATS naming that pdev. Every FUXI
WLAN line the model prints must fit the console's 254 bytes. Scenarios: the happy path (76 CHAN_INFO
lines, exact values on several channels, two request blocks of six answer lines, the after-scan line,
the contain line with both exchanges); L19's vdev/beacon/pdev entry order (two vdev entries, one beacon
entry) with a distinct value in every pdev word (every printed field at its offset, `vdevs 2 bcns 1 bytes
516`); every pdev word at its widest (0x80000000); the DBS PHYs as pdevs 2 and 3, listed second and first
(the W21 line keeps the first entry's pdev); one pdev id for both PHYs (one request, bands 0x3); SINGLE
alone (pdev 0, bands 0); an unusable event before a usable one; credits back 200 ms after COMPLETED; a
beacon inside the first exchange (after-scan `mgmt-rx +1 frames +1 bss +1`, no W24 line again); no
answer (exactly 500 ms per pdev, the run passes), a TLV past the event, a cut pdev entry, refused (stats
id 0, no entry), the byte array before the fixed TLV, credit starvation (3000 ms, nothing sent, no second
request), a REQUEST_STATS descriptor that never completes (1000 ms, no second request; with an answer the
answer is printed): all logged, the run passes; a W11 guard failure in the exchange and a CE fault in the
send (CE3's TP one entry back: `W23 WMI REQUEST_STATS FAILED`, the snapshot) end the run; 191 CHAN_INFO
events (one with flags 0x2): 128 lines and `suppressed 63`, also before the snapshot of a COMPLETED
timeout (no stats request there) and after a failed poll in the scan wait (58) and in the stats wait
(63). Pure checks: the stats parser on every malformed shape (including 4 billion announced vdev entries
and pdev and vdev entries one byte short), a later different usable event counted and not taken, the
counts and byte length kept, routing only after the request and never a WMI failure, the CHAN_INFO words
kept, the request encoder. Static: the requests only on the COMPLETED path, the three bounds per request,
at most two requests and the next only after one that drained, only the transport fails the run (1 after
a failed poll; `wl_ce_fail` on a send fault, the same call as wl_ce_send), PDEV_STAT only, a void parser,
the 128-line cap and the count flushed on every failed wait from START_SCAN on, every C2b line with a new
prefix and its worst case within 254 bytes. L19: the ids, tags and sizes, the CHAN_INFO, stats event and
pdev stats word orders name by name, the signedness of all 35 printed pdev fields, the request words,
`fw_pdev[0]` with no vif and the band-matched `fw_pdev` with one, L19's waits and parse order. Mutants
(c2b-mutants, c2b-fix): wrong field offsets, unbounded or longer waits, a stats failure failing the run,
only one or the wrong order of pdevs, no dedup, bands not ORed or from the wrong word, the counts swapped,
a send fault taken as a refusal, the after-scan line dropped, the stuck-TX answer dropped, a missing
flush, a line past 254 bytes, a later event overwriting the first and an entry bound off by one are all
caught.

What only the device can answer: whether the firmware fetches from the rings at all with zero interrupt
thresholds, which pipe carries HTC READY (both are polled), the credit numbers, the real pdev id, whether
`num_mem_reqs` is 0, and whether WMI READY comes back. For C2: whether the firmware accepts the channel
list and the vdev without a regulatory domain or a vdev start, which pdev id START_SCAN wants (0 as L19,
or the caps' one), whether beacons arrive over WMI MGMT_RX without host RX data rings (the W24b rung), the
real noise floor and the chain masks. Not written yet: the type-1 segment compare helper of
overnight_plan.md 4.11 (the layout of the firmware's copy in that segment is not known).
