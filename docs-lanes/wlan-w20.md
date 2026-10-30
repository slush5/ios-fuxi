# W20-W25: copy engines, HTC and WMI init to WMI READY (WCN7850, `fuxiwlanqrtr=6`, C1), one passive scan (`fuxiwlanqrtr=7`, C2), a passive scan service for iOS (`fuxiwlanqrtr=8`, C3)

Status: C1 (c742af3) passed on the phone (r1c1: CE/HTC/WMI READY, `W22 wmi ready status 0 mac-zero 0 peers
16`, 2 RF chains, 242 scan channels, clean containment). C2 (39c80e5) is about to boot (r1c2), result not
known. C3 is host-tested only (tools/test-fuxi-wlan-ce.py, built on C2 as committed). Every line below that
says what the target does in C2/C3 is what the model does, built from Linux ath12k v6.19. C2 and C3 have
sections of their own below ([C2: W23/W24](#c2-w23w24-one-passive-scan-fuxiwlanqrtr7),
[C3: W25](#c3-w25-a-passive-scan-service-for-ios-fuxiwlanqrtr8)).

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
| `fuxiwlanqrtr=7` | | C2: all of `=6`, then W23/W24 (one passive scan). Same gate and refusals as `=6`; on salami also `fuxiwlansalami=1` (or `2`, C3). |
| `fuxiwlanqrtr=8` | | C3: all of `=7` up to the station vdev, then W25 (the scan service for iOS, `fuxiwifiios=3`). Same gate and refusals as `=7`. |
| `fuxiwlanqrtr` outside 0..8 | negative, 9+ | refused (`W20 fuxiwlanqrtr=N is not 0..8 in this build; refused`). Before C1 these ran silently with no QRTR. |
| `fuxiwlanw20=N` | 0 default | With qrtr 6, 7 or 8: 0x01 NOHTT (no HTT connect; WMI then gets the next endpoint), 0x04 MRRS128 (endpoint PCIe Device Control max read request 128 bytes before CE init, low half only), 0x40 NOHWMODE (no PDEV_SET_HW_MODE). With qrtr 7 or 8 only: 0x02 HTTVER (HTT version request after HW mode), 0x10 BAND24 (the 13 2.4 GHz channels only; with qrtr 8 iOS then gets only those 13), 0x80 PDEVCAP (the EXT MAC/PHY pdev id instead of 0). Refused: 0x08 ACTIVE (would transmit probe requests) and 0x20 FULLBSSID (would log whole BSSIDs; the owner masks every BSSID) at every qrtr, the qrtr-7 bits with qrtr 6, 0x01 together with 0x02, any other bit. |
| `fuxiwlansalami=2` | salami only | C3 profile (owner decision 7, to be confirmed): the gate of `=1`, plus the defaults `fuxiwlanclk=1`, `fuxiwlanprobe=civac`, `fuxiwlanseq=cycle,parf,dbi,eq,hostirq,msi,por,prefix` (the S4 set and the clock votes of the passing r1c1 kit). A boot-arg or ADT `/defaults` value of the same knob always wins; each default used prints `salami <knob> from the fuxiwlansalami=2 profile`. No `fuxiwlanmlo` default; never CLKON, cyclerails or `fuxiwlancnss=pdc`. Any value but 1 or 2 is refused as before (the refusal line still names `=1`, unchanged). |

Refusals set `wl_variant_invalid`; salami refuses at init before any access, the Fuxi build at step 17.
Every boot without `fuxiwlanqrtr=6`, `7` or `8` logs and behaves as before C1, every boot with qrtr absent
or 0..6 as before C2, and every boot with qrtr absent or 0..7 as before C3 (three differentials each, see
the C2 and C3 sections).

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

## C3: W25, a passive scan service for iOS (`fuxiwlanqrtr=8`)

Code: the `/* ---- W25: the scan service for iOS ... ---- */` part of the W20 block of `fuxi_wlan.c`
(`wl_w25_run`, `wl25_scan`, `wl25_stop`, `wl25_heard`, `wl25_end`), the copy of the 0104 handoff
declarations in the W20 knob region (`/* ---- the scan handoff of patch 0104 ... ---- */`), the hooks in
`fuxi_wlan_init` and `fw_thread`, and in `fuxi_wlan_ce.h` `wlwmi_encode_start_scan_list`, `wlscan_listed`,
the scan-id filter (`wlscan.want_id`) and the MGMT_RX hook (`wlscan.heard`). The iOS side of the handoff is
`bsd/dev/fuxi_wifi_radio.c` (patch 0104, K2; [wifi-ios.md](wifi-ios.md)). Plan:
`op11-tools/salami-port/wlan/r1/ios_scan_plan.md` 2.2-2.5 and `overnight_plan.md` 3.2-3.4.

Owner decisions applied as for C2 (passive only, nothing transmitted, K-W2 MAC, SSIDs in clear only in the
W24 proof lines, every BSSID OUI-only, no data path, peer, key or join, the 38 channels, no 6 GHz), plus
two the shop took for C3, to be confirmed by the owner and kept easy to change: decision 7, the
`fuxiwlansalami=2` profile (one table, `wl_salami_defaults`), and decision 8, a 30-minute service
(`WL25_LIFETIME_MS`), then containment and Wi-Fi off in iOS.

### Hooks (every boot, every qrtr)

`fuxi_wlan_init` tells the handoff why there is no radio, once, at the exit it takes:
`fuxi_wifi_radio_stopped(NOT_REQUESTED)` (no `fuxiwlan`, code 0x10000), `REFUSED | 1` (salami without
`fuxiwlansalami=1/2`, 0x20001), `REFUSED | 2` (salami invalid selector or cyclerails, 0x20002), or
`fuxi_wifi_radio_starting()` just before `kernel_thread_start` and `NO_THREAD` (0x30000) if that fails.
`fw_thread` calls `fuxi_wifi_radio_stopped(EXITED | detail)` once, after `thread exit`, on every exit: after
fw_run (detail = its step << 8 | reason, 0 when every step passed; fw_run has run wl5_abort on every path
that touched the device) or after a window mapping failure (detail 0xffff, nothing ran). The calls print
nothing; without `fuxiwifiios=3` they only store the state word in fuxi_wifi_radio.c, so every radio-lane
line is unchanged. 0099 does not include the 0104 header: the eight prototypes (the seven producer calls it
uses plus `fuxi_wifi_radio_wanted`, below), the request struct and five
`FUXI_WIFI_RADIO_*` defines are copied into fuxi_wlan.c, and two tests check them against the header word
for word (tools/test-fuxi-wlan-ce.py, tools/test-fuxi-wifi-radio.py T6). The host harnesses that compile the
W20 block without the handoff get link stubs that fail if called (tools/test-fuxi-wlan-m6.py, the salami
gate harness records the calls).

### Sequence (after the W23 vdev lines)

1. `fuxi_wifi_radio_ready(freqs, n)` with the channels W23 gave the firmware: the 38 (or 13 with w20
   0x10). C2 records no regulatory channel event to intersect with (`0x3a003` is logged only), so the list
   is the one SCAN_CHAN_LIST sent; a channel the firmware cannot use shows as START_FAILED or nothing heard.
2. Idle: every 10 ms a poll pass (`wl_ce_poll`: W11 events and guard, RX on CE2/CE1 with each buffer posted
   again as it is consumed, TX completions, HTC credit reports), the lifetime check, `fuxi_wifi_radio_take`;
   when that gives nothing, `fuxi_wifi_radio_wanted()`, false once the iOS side is STOPPED for the boot (its
   own 3 timeouts in a row, no usable channel; terminal, so no request can come): the service ends there.
   With `fuxiwifiios=3` absent, or with the user's Wi-Fi switch off, it stays true (the radio idles to its
   lifetime, as before).
3. One request: not passive -> `done(seq, EINVAL)`, nothing sent; its channels kept only where the W23 list
   has them, each once, in its order; none left -> `done(seq, EINVAL)`, nothing sent. Else one START_SCAN
   (`wlwmi_encode_start_scan_list`: the C2 command with only the scan id, the passive dwell and the channel
   array changed; PASSIVE | CHAN_STAT_EVENT forced, no SSID, IE or probe), dwell = the request's when
   50..200 ms (the iOS side's own rule, so both sides compute the same number), else 150; max_scan_time
   n x dwell x 3/2 (the iOS side waits 1000 ms more); scan id 0xa001..0xafff in turn (ATH12K_SCAN_ID's block
   with a 12-bit counter; SCAN_EVENTs of another id are counted in `wrong_id` and ignored, so a late event of
   a stopped scan never ends the next one; they are said: `W25 ignored scan event` lines, 8 in the service,
   the count and the last such id on each W25 line, the total in the service end, the count in a snapshot).
4. While it runs, a poll pass every 1 ms. Every beacon or probe response the MGMT_RX parser keeps goes to
   `fuxi_wifi_radio_frame(seq, frame, len, rx MHz, snr + noise floor, noise floor)` inside the call that has
   its RX buffer, before the buffer is posted again (the iOS side parses and copies during the call).
5. The end: COMPLETED -> `done(seq, 0)`; START_FAILED or DEQUEUED -> `done(seq, EIO)`;
   `fuxi_wifi_radio_cancelled(seq)` -> STOP_SCAN (ath12k_scan_stop's values with this scan's id), at most
   300 ms for its COMPLETED or DEQUEUED, `done(seq, ECANCELED)`; no STARTED in 1000 ms (L19 mac.c:5046-5053)
   or no end by max_scan_time + 200 ms (mac.c:5385-5388), whichever comes first -> the same STOP_SCAN,
   `done(seq, ETIMEDOUT)`. A STOP_SCAN not confirmed in 300 ms: `done()` all the same (that keeps the iOS
   side's 1000 ms margin), then, before the next `take()`, poll passes every 1 ms for that scan's COMPLETED or
   DEQUEUED until 3 s after the STOP_SCAN (L19 waits 3 s, mac.c:4902), so the next START_SCAN never meets an
   abort still under way. A failed scan = START_FAILED, DEQUEUED, a timeout, or a cancel whose STOP_SCAN is
   never confirmed in those 3 s; a completed scan resets the count; a cancel confirmed (in time or late) and a
   refused request change nothing.
6. The service ends after 1800 s (`W25 complete: lifetime over (1800 s); stop/contain`, then the normal
   step 28 end into wl5_abort), when the iOS side has stopped (`W25 complete: the iOS side stopped;
   stop/contain`, the same normal end), after 3 failed scans in a row (`W25 failure: W25 3 failed scans in a
   row; contain, no retry`), or at the first failure of the transport, the guard, a credit wait (3000 ms) or
   an encoder (the usual `W25 failure: ...` and snapshot). `wl_ce_contain` prints the W25 summary, then C2's
   containment STOP_SCAN rule applies to the scan in flight (its id), then the latch. `fw_thread` reports
   EXITED and iOS turns Wi-Fi off. Elapsed times come from mach_absolute_time, not from counted sleeps, so
   the 1 ms and 10 ms polls taking longer than asked cannot stretch the bounds.

Long runs: RX buffers are posted again one for one as they are consumed (`wlce_reap_rx`), so 62 stay out on
each pipe whatever the number of scans; every send waits for its TX completion; credits come back in the
firmware's trailer reports and an endpoint's count stops at 0xffff (`WLHTC_CREDITS_MOST`; L19 adds without
a bound); every counter a decision uses is either reset per scan (the scan table, the failure run) or a
difference of 64-bit milliseconds; the rest are only printed.

### Logs

```
FUXI WLAN: W25 selected scan service for iOS (fuxiwifiios=3): passive only, nothing transmitted; 1800 s, or 3 failed scans in a row, then contain; first scan W24 lines, then one W25 line a scan   (at init)
  ... W15, W20..W22 and the W23 chan list / vdev lines as for qrtr 7 ...
FUXI WLAN: W25 service ready: 38 channels passive to iOS (2.4 GHz 13, 5 GHz 25), lifetime 1800 s
FUXI WLAN: W23 scan foreign freq F                                         (the first scan sent only)
FUXI WLAN: W24 bss aa:bb:cc:xx:xx:xx ch C freq F dBm S privacy P rsn R beacon frames N ssid "NAME"   (the first completed scan only)
FUXI WLAN: W24 scan summary frames N bss M ssids K
FUXI WLAN: W24 scan detail mgmt-rx N ...
FUXI WLAN: W25 scan K seq S id 0xa001: 38 channels passive dwell 150 ms, completed after T ms, reason 0, status 0; frames N bss M, to iOS N kept K; scan events E, of another id 0 (last 0)   (scans 1..24, every 100th, the first 24 failed ones)
FUXI WLAN: W25 scan K seq S: not passive, refused, nothing sent, status 22      (or "no listed channel, refused")
FUXI WLAN: W25 complete: lifetime over (1800 s); stop/contain                  (or "the iOS side stopped")
FUXI WLAN: W25 service end: lifetime over after 1800 s; scans N (completed A, failed F, cancelled C, refused R), frames to iOS N kept K; stops confirmed late L, scan events of another id W, credit waits H; rx CE1 P/R CE2 P/R tx CE3 P/R credits N
FUXI WLAN: W23 contain: scan over, no STOP_SCAN
```

Other ends of a W25 line: `start failed`, `dequeued`, `cancelled`, `no STARTED`, `no end by max + 200 ms`,
the last three followed by `, stop confirmed`, `, stop confirmed late` or `, stop not confirmed` (Darwin
errno: EIO 5, EINVAL 22, ETIMEDOUT 60, ECANCELED 89); `reason` is the firmware's, from COMPLETED,
START_FAILED or DEQUEUED (0 when none came). An ignored SCAN_EVENT of another id prints
`W25 ignored scan event 0xT of id 0xI (in flight 0xF)` (8 in the service); a snapshot taken while a W25 scan
is in flight adds `W20 snapshot scan id 0xF: events of another id W (last 0xI), of another vdev V`. In the
service an HTC credit wait (`W25 WMI START_SCAN waits for ...`) is said the first 16 times and every 100th,
all of them counted in the service end. The W25 lines and the summary carry counts, reasons and scan ids
only, never an SSID or a BSSID; the iOS side logs none either.

### Host tests (C3)

In tools/test-fuxi-wlan-ce.py, on the model's simulated clock (wl_sleep moves it, mach_absolute_time reads
it), behind a fake iOS side that queues requests and records ready(), frames and done(): six requests in a
row (the 38-channel type-3 scan, dwell 0 -> 150, a mixed list with 2484 MHz and 6 GHz dropped and a repeat,
a non-passive one and an unlisted one refused with nothing sent, a DFS channel at 200 ms, a stale COMPLETED
of the previous id ignored, every frame checked to be handed over inside the CE1/CE2 RX buffers); a cancel
400 ms into a 38-channel scan (STOP_SCAN with its id, the target's leftovers dropped); START_FAILED, a reset by
a good scan, an overrun (STOP_SCAN at max + 200 = 650 ms), DEQUEUED, no STARTED (STOP_SCAN at 1000 ms), the
third failure in a row contained; an unanswered STOP_SCAN (300 ms, a failure, the next START_SCAN 3 s after
it); 3 x START_FAILED; three unanswered STOP_SCANs in a row (contained with the third scan in flight on a
healthy transport: containment's STOP_SCAN goes out and names that scan, 0xa003); an overrun at the
request's dwell (200 ms: STOP_SCAN at 600 + 200 = 800 ms); a cancel whose STOP_SCAN is never answered
(ECANCELED to iOS, a failed scan) and one answered 1 s late (a cancel, `stop confirmed late`, the next
START_SCAN right after the late end); a guard failure after a START_SCAN (no STOP_SCAN, no doorbell);
credit starvation (a START_SCAN waits 3000 ms, then containment, STOP_SCAN skipped); 30 idle minutes
(180000 polls of 10 ms, the end at 1800 s); w20 0x10 (13 channels to iOS); 4100 scans in a row (62 RX
buffers out on each pipe after 36900+ messages, every TX reaped, every credit back, ids wrapping after
0xafff, W25 lines 1..24 and every 100th); the iOS side stopping after 2 scans (the idle service ends at its
next poll, not at 1800 s); a target answering every SCAN_EVENT with 0xa000 (8 `ignored scan event` lines,
the per-scan count and last id, the total; 3 timeouts contain); a target returning each START_SCAN credit
only after its COMPLETED (247 waits in 250 scans, 18 wait lines, all counted; the START_FAILED 30th scan
still gets its line); pure checks of the list encoder (equal to C2's but for id, dwell, channels; every
refusal), the id filter and the credit bound. The model also fails any C3 run in which a due RX message
finds no posted buffer for 1 s.

END TO END: the real `xnu/fuxi-wifi-ios/fuxi_wifi_radio.c` (one unit, as tools/test-fuxi-wifi-radio.py
builds it) linked with the radio lane model (another unit, `-DFUXI_WLAN_E2E`): the consumer asks
`fuxi_wifi_radio_scan({2437, 5180}, passive, 150 ms)`; the firmware must see one START_SCAN of exactly those
channels, PASSIVE, dwell 150, max 450; the two networks it beacons there come back through
`fuxi_wifi_radio_results` with SSID, BSSID, frequency (the DS / HT operation channel), RSSI (SNR + floor),
noise -95; `stopped(EXITED)` turns Wi-Fi off (`code 0x40000`); the iOS log names no SSID or BSSID.

Mutants (16, each must fail for its own reason): the passive check dropped (the firmware sees a non-passive
request), the NOT_REQUESTED stop dropped (salami gate suite), the EXITED stop dropped (static), no RX
re-post (RX ring starved), the lifetime bound broken (ran past 1800 s), the scan-id filter dropped, no
STOP_SCAN on a cancel, 3 failures never contain, containment's STOP_SCAN back to C2's 0xa000, the overrun
deadline from the 150 ms dwell, an unconfirmed cancel not a failure, no wait for the end of an unconfirmed
stop, the iOS side's stop never ending the service, the ignored-id lines dropped, the credit-wait line
uncapped, failed scans past 24 not said. tools/test-fuxi-wifi-radio.py checks `fuxi_wifi_radio_wanted`
(true while READY, with the user's Wi-Fi off, without a consumer and with `=4`; false after the 3 timeouts
and after a stop; T9 mutant: always true). tools/test-fuxi-wlan-salami.py covers the qrtr 8 gate and
the profile (defaults, explicit knobs winning on the cmdline and in ADT /defaults, the M2 recipe running
with exactly the r1c1 knobs) and the hooks' order and codes; test-fuxi-wlan-w11/w15/w16.py the selector
lattice up to 9.

### Proof that qrtr 0..7 are unchanged

Scripts in op11-tools/salami-port/wlan/r1/impl/c3-verify (verify.sh), run again after the review fixes from
impl/c3-fix (verify.sh, the same checks plus a link stub for `fuxi_wifi_radio_wanted`): the W15 harness log (qrtr 5, keep 0
and 1) of 39c80e5 and C3, byte for byte; the init/configure logs of 763 boot-arg sets (the refusals and
every C2 set with qrtr absent or 0..7) on the salami and the Fuxi build, both through 39c80e5's own gate
harness, byte for byte; 39c80e5's C1+C2 host model (65 scenario logs: the whole qrtr 6 and 7 runs, their
failures, STOP_SCAN and wl5_abort) built against 39c80e5's and C3's production code, byte for byte.

### Kit (M2)

The r1c1/r1c2 recipe with the C3 kernel: KIT_BASE `fuxitouch=1 fuxitclk=1000 fuxitouch_poll_us=2000
fuxitouch_idle_us=4000 fuxicpuload=10`, cmdline extra `fuxicores=0x7c sweetfb_tegate=1 sweetfb_teguard=0
fuxiwlan=28 fuxiwlanqrtr=8 fuxiwifiios=3`, KIT_ADT_EXTRA `--wlan-node --wlan-defaults fuxiwlansalami=2`,
KIT_RO=0 (op11-tools/salami-port/wlan/r1/impl/c3-kit/build_c3_kit.sh). Computed the way
logo-boot/build_kit_rel.sh and s31_build.sh build it (c3-kit/budget.sh, which reproduces the r1c1 and k2m1
numbers): cmdline 526 characters (592 of 607 with the loader's 66, 15 free), ADT 16348 bytes (36 of 16384
free). Without the profile (S4 + the wlan node) the ADT would be 16472 bytes and mkadt-salami.py refuses it.
`sweetiolog=Preferences` does not fit on this KIT_BASE (615 of 607); dropping `fuxicpuload=10` (owner
decision 4) would make it fit (600).

Run M2 with `SESSION_SECONDS=3000 bash run_first_boot.sh --yes` (after `--local-only` and `--check-only`).
The 1800 s lifetime starts at `W25 service ready`, minutes after the handoff, while the kit runner monitors
1800 s from the handoff by default and renews the loader reset (15 min) only while it runs: with the default,
the service end, its containment and the iOS `Wi-Fi reported off` line come after the monitor stopped, and
the reset (2100 to 2700 s after the handoff) can come before them. With 3000 s the end is captured when W25
is ready within 20 min of the handoff, and the reset comes at 3300 s or later.

### What only the device can answer (C3)

Whether the firmware takes scan ids other than 0xa000 (L19 always sends 0xa000; the vendor host driver is
believed to number its scans the same way, a 12-bit counter under the 0xa000 prefix, but that is from
memory, not checked here). If every W25 scan gets START_FAILED while C2's scan started, the id is the first
suspect: a build with `WLSCAN_ID_LAST` equal to `WLSCAN_ID_FIRST` = 0xa000 tests it (the id filter then
cannot tell scans apart; the STOP_SCAN waits still bound that). If the target answers with an id of its own,
the filter drops even START_FAILED, and every scan reads `no STARTED`: the log then shows `W25 ignored scan
event ... of id 0xI` lines and `of another id N (last 0xI)` > 0 on the W25 lines with no STARTED. Ladder:
0xI = 0xa000 -> the next build sets `WLSCAN_ID_FIRST` = `WLSCAN_ID_LAST` = 0xa000; any other fixed 0xI ->
those two set to 0xI; an id that changes -> a build that sets no `want_id`. Also: how often wifid scans with
Settings open and whether it cancels, the real time a 38-channel scan takes, and whether Settings lists
what GET 11 returns (ios_scan_plan.md M2).

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

What only the device can answer: whether the firmware fetches from the rings at all with zero interrupt
thresholds, which pipe carries HTC READY (both are polled), the credit numbers, the real pdev id, whether
`num_mem_reqs` is 0, and whether WMI READY comes back. For C2: whether the firmware accepts the channel
list and the vdev without a regulatory domain or a vdev start, which pdev id START_SCAN wants (0 as L19,
or the caps' one), whether beacons arrive over WMI MGMT_RX without host RX data rings (the W24b rung), the
real noise floor and the chain masks. Not written yet: the type-1 segment compare helper of
overnight_plan.md 4.11 (the layout of the firmware's copy in that segment is not known).
