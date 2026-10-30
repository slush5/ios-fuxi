# W20-W25: copy engines, HTC and WMI init to WMI READY (WCN7850, `fuxiwlanqrtr=6`, C1), one passive scan (`fuxiwlanqrtr=7`, C2), a passive scan service for iOS (`fuxiwlanqrtr=8`, C3)

Status: C1 (c742af3) passed on the phone; C2 (39c80e5) ran clean on the phone (kit r1c2: 38 channels
tuned, CHAN_INFO on all of them, no MGMT_RX at all); C2b (7caf3ed: CHAN_INFO detail and a pdev stats
request per PHY pdev on top of C2) is committed, its readout boot is next; C2c (W26: host RX rings, the
WBM idle link-descriptor list and the pdev chain masks, w20 0x100/0x200/0x400, all default off) ran on the
phone (r1c2c-all: the RX engine woke, nothing landed, the scan wedged); C2d (W27: RX ring selection and the
REO/WBM buffer return path, w20 0x800/0x1000, default off) ran on the phone (r1c2d-l: everything programmed, nothing
written, the scan wedged after 3 channels); C2e (W28: the RXDMA monitor status rings and a read-only crash line, w20
0x2000, default off) is host-tested only (tools/test-fuxi-wlan-ce.py). C3 (W25, the scan service for iOS,
rebased onto C2e, which it carries under qrtr 8 as under qrtr 7: W26, W27 and W28) is host-tested only. Every line below that says what the target does is what
the model does, built from Linux ath12k v6.19. C2, C2b, C2c, C2d, C2e and C3 are described in their own sections below
([C2: W23/W24](#c2-w23w24-one-passive-scan-fuxiwlanqrtr7),
[C2b](#c2b-chan_info-detail-and-the-pdev-stats-requests-fuxiwlanqrtr7-diagnostic),
[C2c: W26](#c2c-w26-host-rx-rings-the-wbm-idle-list-and-the-chain-masks-fuxiwlanqrtr7-and-8-w20-0x100-0x200-0x400),
[C2d: W27](#c2d-w27-rx-ring-selection-and-the-reowbm-buffer-return-path-fuxiwlanqrtr7-and-8-w20-0x800-0x1000),
[C2e: W28](#c2e-w28-the-rxdma-monitor-status-rings-and-the-crash-line-fuxiwlanqrtr7-and-8-w20-0x2000),
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
| `fuxiwlanqrtr=8` | | C3: all of `=7` up to the station vdev (with the C2c, C2d and C2e setup when their bits are given), then W25 (the scan service for iOS, `fuxiwifiios=3`). Same gate and refusals as `=7`. |
| `fuxiwlanqrtr` outside 0..8 | negative, 9+ | refused (`W20 fuxiwlanqrtr=N is not 0..8 in this build; refused`). Before C1 these ran silently with no QRTR. |
| `fuxiwlanw20=N` | 0 default | With qrtr 6, 7 or 8: 0x01 NOHTT (no HTT connect; WMI then gets the next endpoint), 0x04 MRRS128 (endpoint PCIe Device Control max read request 128 bytes before CE init, low half only), 0x40 NOHWMODE (no PDEV_SET_HW_MODE). With qrtr 7 or 8 only: 0x02 HTTVER (HTT version request after HW mode), 0x10 BAND24 (the 13 2.4 GHz channels only; with qrtr 8 iOS then gets only those 13), 0x80 PDEVCAP (the EXT MAC/PHY pdev id instead of 0), and C2c's 0x100 RXRING (host RX rings with 4095 buffers and their 4 HTT SRING_SETUP; only with 0x02), 0x200 IDLELINK (the WBM idle link-descriptor list), 0x400 CHAINMASK (PDEV_SET_PARAM TX/RX chain mask), and C2d's 0x800 RXSEL (RX_RING_SELECTION_CFG per MAC ring) and 0x1000 REORX (REO/release rings, buffers back to ring 160), each only with 0x100, and C2e's 0x2000 MONSTAT (the monitor status rings 185/193 and the crash line), only with 0x100. Refused: 0x08 ACTIVE (would transmit probe requests) and 0x20 FULLBSSID (would log whole BSSIDs; the owner masks every BSSID) at every qrtr, the qrtr-7 bits with qrtr 6, 0x01 together with 0x02, 0x100 without 0x02, 0x800, 0x1000 or 0x2000 without 0x100, any other bit. |
| `fuxiwlansalami=2` | salami only | C3 profile (owner decision 7: recommended yes, pending owner): the gate of `=1`, plus the defaults `fuxiwlanclk=1`, `fuxiwlanprobe=civac`, `fuxiwlanseq=cycle,parf,dbi,eq,hostirq,msi,por,prefix` (the S4 set and the clock votes of the passing r1c1 kit). A boot-arg or ADT `/defaults` value of the same knob always wins; each default used prints `salami <knob> from the fuxiwlansalami=2 profile`. No `fuxiwlanmlo` default; never CLKON, cyclerails or `fuxiwlancnss=pdc`. Any value but 1 or 2 is refused as before (the refusal line still names `=1`, unchanged). |

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

## C2c: W26, host RX rings, the WBM idle list and the chain masks (`fuxiwlanqrtr=7` and `8`, w20 0x100, 0x200, 0x400)

Why: C2 (r1c2) tuned all 38 channels and heard zero MGMT_RX, and r1c2b (w20 0x02) got VERSION_CONF 3.112
and still zero. L19 never scans without two host resources C2 does not give: the WBM idle list of MSDU
link descriptors that RXDMA pushes (dp.c:835 "for HW internal usage", programmed in dp_alloc before HTC,
core.c:1291 < 1300) and host RX buffers on the refill and per-MAC rings it announces by HTT SRING_SETUP
(dp_rx.c:4610-4646). L19 also sends PDEV_SET_PARAM TX/RX_CHAIN_MASK at mac start, before its channel list
(mac.c:9418, 9422). C2c gives each of the three behind its own w20 bit, in L19's geometry and order, and
reports from host memory alone whether the firmware took them, whether RXDMA wrote into them or past
them, and what it reported as errors. Plan: op11-tools/salami-port/wlan/r1/rx_path_plan.md (stage W26),
with the shop's decisions D1..D6 on the verifier's open items.

Code: the C2c section of `fuxi_wlan_ce.h` (constants, `struct wlrx*`, `wlrx_layout`, `wlrx_fill_refill`,
`wlrx_publish_refill`, `wlrx_sring_words`, `wlrx_link_layout/expect/fill`, `wlrx_pre_addr`,
`wlrx_idle_regs`, `wlrx_wbm_ok`, `wlrx_idle_owned/why`, `wlrx_watch_read/derive`, `wlrx_guard_count`,
`wlrx_content`, `wlrx_link_content`, `wlrx_htt_count`, `wlrx_note_ci/stats`, `wlrx_next`; pure,
bounded, no allocation) and the W26 part of the W20 block of `fuxi_wlan.c` (`wl26_wbm`, `wl26_alloc`,
`wl26_idle`, `wl26_setup`, `wl26_sring`, `wl26_chainmask`, `wl26_rings`, `wl26_watch`, `wl26_list`,
`wl26_report`, `wl26_contain`, `wl26_free`; `wl_win` is the one window implementation both register
accessors share). The arenas are `wl_rx_arena` and `wl_link_arena`, both in `wl_owned_alias`.

### Gate and kits

`WL23_BITS` gains 0x100 RXRING, 0x200 IDLELINK and 0x400 CHAINMASK; `WL20_BITS` (qrtr 6) does not, so
qrtr 6 refuses all three. qrtr 8 (C3) takes `WL23_BITS` too, so the three bits, their rules and the W26
setup are the same there (see C3, [With C2b and C2c](#with-c2b-and-c2c-the-rebase-onto-c18f99d)). 0x100 needs 0x02 (L19 asks the HTT version before any ring announcement,
core.c:955 then 1143); with 0x01 the existing 0x01 + 0x02 rule refuses it too. 0x200 and 0x400 need no
other bit. keep/stop22, 0x08, 0x20 and unknown bits stay refused. A valid qrtr 7 boot with a W26 bit
prints, after the W23 selected line:
`W26 selected: rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100)|no; wbm idle list 32767 link
descriptors (w20 0x200)|no; chain mask pdev tx/rx (w20 0x400)|no; nothing selected to host rings`.

| Kit | w20 | Decimal | Adds to C2 + C2b |
|---|---|---|---|
| r1c2c-b | 0x200 | 512 | the WBM idle list only (no HTT message at all) |
| r1c2c-m | 0x402 | 1026 | HTT version + the two chain masks |
| r1c2c | 0x302 | 770 | HTT version + idle list + RX rings with 4095 buffers + 4 SRING_SETUP |
| r1c2c-all | 0x702 | 1794 | all of the above |
| r1c2c-a / r1c2c-a5 | 0x102 / 0x502 | 258 / 1282 | fallbacks when the firmware owns the idle list |
| repeats | 0x210 / 0x312 / 0x712 | 528 / 786 / 1810 | the 2.4 GHz repeat after an exhaustion flag |

Route: a 3- or 4-digit w20 does not fit the cmdline (r1c2b's is 540 characters, 606 of s31's 607), so
every C2c kit keeps r1c2's cmdline byte for byte (`fuxiwlan=28 fuxiwlanclk=1 fuxiwlanqrtr=7`, no w20)
and puts `fuxiwlanw20=<decimal>` in ADT /defaults (`--wlan-defaults`, a 4-byte int); `wl_arg` reads it
from there and logs `salami fuxiwlanw20 from ADT /defaults`. The order of the kits is the ladder of
rx_path_plan.md 1.2, read from the C2b readout.

### Sequence (L19 order)

1. W20, after the seven CE rings are set up and read back, before the CE RX buffers are posted (L19:
   ce_init_pipes core.c:1285, dp_alloc core.c:1291, core_start core.c:1300). Both arenas first (link
   arena with 0x200, RX arena with 0x100; IOVA aligned to 2 MiB, Normal-NC, mapped with live SMMU
   readback): an allocation, NC or map failure stops the run before any WBM access or HTT ring message
   (`W20 failure: W26 rx arena allocation/NC/map`, `... link arena ...`). Then dp_alloc's own order:
   - W26b (0x200): nine allowlisted pre-reads (D1: the seven of the plan plus the scattered ring base
     0x00a34250/0x00a34254, hw.c:1217-1218), one line. Refusal, with the reason printed and no WBM write at
     all: any read ~0u (`unreadable, ownership not established`), SRNG_ENABLE (`firmware owns the list`),
     a non-zero ring base (`firmware owns the list`), scatter mode with base 0 and SRNG_ENABLE 0
     (`AMBIGUOUS`, not called firmware-owned); `w20 0x200 dropped for this boot`, the run goes on. Else
     ring 120's eight writes in L19's order (BASE_LSB, BASE_MSB, ID, IX0, IX1, R2 HP, R2 TP, MISC;
     hal.c:1714-1806, no MSI and no TP_ADDR for this ring), BASE_LSB and MISC read back (a mismatch fails the
     run before the HP), the 32767 entries, the write barrier, HP = 65534 through the shadow register
     WLAN_CFG gave ring 120 (looked up in `wl15_shadow_v3`: index 19, BAR0 0x948) or, with fuxiwlanw15=1,
     R2 HP 0x00a370b8 through the window.
   - W26a (0x100): the RX arena zeroed, the 0xa5 sentinel over bytes 0..63 of slots 0..4094, 4095 refill
     entries (slot start, RBM 6, cookie = slot), the write barrier, wrp[0] = 8190 (L19's access_end for an
     LMAC source ring, hal.c:2173-2184). The device learns nothing of this until SRING_SETUP.
2. W21/W22 unchanged; with any W26 bit the HTT handler is installed at the HTT connect, so every HTT T2H
   message is counted by type (VERSION_CONF handling unchanged).
3. W23, right after VERSION_CONF (where it would be without 0x02) and before the channel list: with 0x100
   the four SRING_SETUP (refill 160, mac0 161, mac1 169, rxdma-err 186) back to back on the HTT endpoint
   through `wl_ce_send` (one in flight, TX drained), no reply asked, then a 100 ms look (non-fatal); the
   setup watch (without the wait for the idle list alone); with 0x400 PDEV_SET_PARAM TX_CHAIN_MASK then
   RX_CHAIN_MASK for `wl23_pdev` with the preferred hw mode's masks (nothing sent when a mask is 0).
4. The C2 scan unchanged; the watch `started` right after STARTED, `mid` inside the COMPLETED wait at the
   first poll pass with foreign >= (n + 1) / 2 (19 of 38, 7 of 13), `completed` as soon as that wait
   returns, before the W24 lines.
5. After the C2b stats lines and the CHAN_INFO flush (also when the stats exchange ended the run, after
   its failure line; on a COMPLETED timeout, before the snapshot): the content, guard, error-ring, link and
   counter lines, the D4 line, the D5 hint (host reads only); `W23 complete`.
6. Containment (2.10 of the plan): `wl26_contain` first in `wl_ce_contain` (no host write to either arena
   and no WBM access from then on), STOP_SCAN, the CE latch, MHI reset, bus master off, PERST; only in
   `wl5_abort`'s proven branch (bus master off and PERST confirmed, M6o undo clean, SMMU undo done) the
   guards counted again (D3, host reads), both arenas zeroed through their NC mappings and freed; otherwise
   quarantined until reboot with the rest.

Register accesses, nothing else: the nine pre-reads, the eight writes, the two readbacks, the one HP
publish, R2 HP and TP at each watch, through `wl26_wbm` (allowlist `wlrx_wbm_ok`, refused after the latch)
and `wl_win` (0x310c saved, selected, verified and restored, as every CE access). No interrupt, MSI, GIC,
power or PMIC access; the W11 device wake stays held as for every W20 access.

### Wire, register and memory facts (all re-derived from L19 by the host test)

| Ring | HAL id | HTT ring, type, pdev field | Entries x bytes | HP / TP words |
|---|---|---|---|---|
| refill | 160 | HOST1_TO_FW_RXBUF 5, SW_TO_SW 2, pdev 0 | 4096 x 8 | wrp[0] / rdp[160] |
| mac0 | 161 | RXDMA_HOST_BUF 0, SW_TO_HW 1, pdev 1 | 2048 x 8 | wrp[1] / rdp[161] |
| mac1 | 169 (161 + 8, PMAC stride) | RXDMA_HOST_BUF 0, SW_TO_HW 1, pdev 2 | 2048 x 8 | wrp[9] / rdp[169] |
| rxdma err | 186 | NON_MONITOR_DEST 7, HW_TO_SW 0, pdev 1 | 1024 x 32 | rdp[186] / wrp[26] |
| wbm idle | 120 | (registers) | 32768 x 8, 32767 listed | R2 0x00a370b8 / 0x00a370bc |

rdp is u32[207] (HAL_SRNG_RING_ID_MAX = 183 + 3 x 8), wrp u32[47]. SRING_SETUP (msg 0xb, 13 words):
info0 = msg | pdev << 8 | ring << 16 | type << 24 (pdev = mac for SW_TO_SW, mac + 1 otherwise); info1 =
ring size in words | entry words << 16 | LOOP_CNT_DIS (bit 25) for SW_TO_HW; hp/tp addresses; MSI 0; intr
= batch x entry words | (timer >> 3) << 16 (RXDMA_BUF batch 0, 500 us; RXDMA_DST batch 1, 256 us); info2
= the low threshold (entries >> 3, in words) for RXDMA_BUF only; RESPONSE_REQUIRED never. With the RX
arena at A: refill 0x0205000b, A + 0x4000, info1 0x00022000, hp A + 0x400, tp A + 0x280, intr 0x003e0000,
low 1024; mac0 0x0100010b, A + 0xc000, 0x02021000, A + 0x404, A + 0x284, 0x003e0000, 512; mac1 0x0100020b,
A + 0x10000, 0x02021000, A + 0x424, A + 0x2a4, 0x003e0000, 512; err 0x0007010b, A + 0x14000, 0x00082000,
A + 0x2e8, A + 0x468, 0x00200008, 0. Entries (struct ath12k_buffer_addr): address 31:0; address 39:32 in
7:0, RBM 11:8 (6 SW1_BM for buffers, 1 WBM_DEV0_IDLE_DESC_LIST for links), cookie 31:12 (ours: the slot
for buffers, the DP_LINK_DESC_COOKIE_SET shape with id 32766 - i, page 0, for links). Ring 120 (WCN7850:
R0 0x00a34d3c, TCL1 layout): BASE_LSB = ring IOVA, BASE_MSB = 0x01000000 (65536 words << 8), ID 2, IX0
0x01000002 (timer 256 raw, batch 1 x 2), IX1 0, R2 HP 0, R2 TP 0, MISC 0x43 (RING_ID_DISABLE |
LOOPCNT_DISABLE | SRNG_ENABLE). PDEV_SET_PARAM: cmd 0x4003, TLV 0x0052000c, {pdev, 1, tx mask}, then
{pdev, 2, rx mask}.

RX arena (0x1040000 bytes): rdp at 0, wrp at 0x400, refill ring 0x4000, mac0 0xc000, mac1 0x10000, err
0x14000, 4096 slots of 4 KiB from 0x20000 (bytes 0..2047 of slots 0..4094 posted, 2048..4095 and slot 4095
a guard, never posted), a 128 KiB tail. Link arena (0x440000): ring 120, then 32768 descriptor slots of
128 bytes from 0x40000 (slot 32767 never listed). SRING_SETUP has no length field; C2c sends no ring
selection, so any write past 2048 bytes lands in the slot's own guard first and is counted. The gaps the
device was given no address in (the first 16 KiB except rdp 160/161/169/186 and wrp 0/1/9/26, and the 16 KiB
after the error ring, where an error ring overrun past entry 1023 would land) are counted as well.

### Lines

```
FUXI WLAN: W26 link arena IOVA I bytes 0x440000 Normal-NC mapped                                        (0x200)
FUXI WLAN: W26 rx arena IOVA I bytes 0x1040000 Normal-NC mapped; rdp I wrp I                            (0x100)
FUXI WLAN: W26 wbm idle before: base-lsb R base-msb R misc R r2-hp R r2-tp R idle-list-cntl R idle-list-size R scatter-base R/R; not owned|refused
FUXI WLAN: W26 wbm idle list not programmed: REASON; w20 0x200 dropped for this boot                    (refused)
FUXI WLAN: W26 wbm idle ring 120 base B 65536 words, 32767 link descriptors (rbm 1, our cookie) misc M readback ok, hp 65534 via shadow 19 (0x948)|R2 0xa370b8 (fuxiwlanw15=1); writes 8 window restore checked
FUXI WLAN: W26 refill ring 160 filled 4095 buffers (rbm 6, 2048 B at a 4096 B stride, sentinel 0xa5 x64, slots S-E) hp 8190 in wrp[0]
FUXI WLAN: W26 htt sring setup refill|mac0|mac1|rxdma-err ID info0 X info1 X intr X low N           (x4)
FUXI WLAN: W26 rx watch POINT refill tp T taken N left L mac0 hp H tp T avail A mac1 hp H tp T avail A err hp H fill F; exhausted none|refill mac0 mac1 err[; refill-empty]
FUXI WLAN: W26 idle watch POINT hp H tp T avail A; exhausted none|idle
FUXI WLAN: W26 chain mask pdev P tx X rx X sent (PDEV_SET_PARAM 0x4003 ids 1, 2)
FUXI WLAN: W26 rx content mac0|mac1 entries N valid V foreign F rbm-other R cookie-other C
FUXI WLAN: W26 rx content refill taken T kept K; buffers written W of 4095
FUXI WLAN: W26 rx guard [SPILL ]nonzero dwords: slot-upper N (slots S), slot 4095 N, tail N, gaps N[; stop; owner review before any further kit]
FUXI WLAN: W26 rxdma err entries E (hp H[ bad]) push err A routing B flush C other D; rbm link N buf N other N
FUXI WLAN: W26 rxdma err codes overflow N mpdu-len N ...                                (non-zero only, wrapped)
FUXI WLAN: W26 rxdma err entry K w1 X w5 X w6 X w7 X                                    (the first four, frame fields masked)
FUXI WLAN: W26 link content descriptors written D of 32767, unlisted slot clean|written; idle ring entries changed C
FUXI WLAN: W26 counters htt-t2h N htt-rx N wmi-events N unknown N bad-reports N sring-sent N
FUXI WLAN: W26 htt-t2h types T:N ...|none                                                (non-zero types, wrapped)
FUXI WLAN: W26 TX COUNTERS NON-ZERO: owner review (chan-info tx-frame events N; stats answers N with local-enqued N hw-queued N mpdu-enqued N hw-reaped N base tx-frame N)
FUXI WLAN: W24 no beacon or probe response heard (chan-info N); next rung NEXT       (D5, with a W26 bit only)
FUXI WLAN: W26 rx contain: host writes off; arenas held until bus master off, PERST and SMMU undo
FUXI WLAN: W26 rx guard after containment nonzero dwords: slot-upper N (slots S), slot 4095 N, tail N, gaps N[; SPILL: owner review before any further kit]
FUXI WLAN: W26 arenas zeroed BYTES bytes before free
```

POINT is `setup+100ms` (with 0x100) or `setup`, `started`, `mid (foreign N)`, `completed`. A watch word
that is ~0u prints `unreadable`, one that is not a multiple of its entry or not below its ring prints the
raw word and `bad`; neither is ever acted on. Derived: refill taken = tp / 2, left = ((8190 - tp) mod
8192) / 2; macN avail = ((hp - tp) mod 4096) / 2; err fill = hp / 8; idle avail = ((HP - TP) mod 65536) /
2. Exhaustion flags: macN at avail 0 once its tp moved, err at fill >= 1023, idle at avail 0, and refill
at left 0 only together with an exhausted MAC ring (avail 0 once its tp moved: a ring the firmware never
used, hp = tp = 0, is not one) or `kept` (refill taken minus the MAC ring entries, hp / 2) above the setup
watch's (D2); a refill left 0 without either is the `refill-empty` note. Content (host reads once, after
the C2b lines): a MAC entry is valid when its address (bits 39:0) is the start of a posted slot 0..4094,
foreign otherwise; RBM and cookie differences among valid ones are counted, never a stop; buffers written
= posted slots whose bytes 0..63 are no longer all 0xa5. Nothing read from a received frame is printed:
only counts, and the four raw error entries with the frame's fields masked out (w5 MPDU_BYTE_COUNT 21:8,
w6 MPDU_FRAG_NUM 10:7 and MPDU_SEQ_NUM 30:19, hal_desc.h:1033-1044). A non-zero gap or guard count is a
SPILL. The D4 line prints on a W26 boot when any CHAN_INFO tx-frame or any stats answer's local-enqued,
hw-queued, mpdu-enqued, hw-reaped or base tx-frame is non-zero (a reading rule: the run contains
normally). With a W26 bit the C2b hint moves after the W26 lines and
names the next step of the plan's 2.11 table (D5): stop for a guard spill or a foreign MAC entry; the same
kit with 0x10 after an exhaustion flag (owner review when 0x10 is already set); the r1c2c-a / 0x502
fallback when the firmware owns the idle list (owner review when the pre-read was ambiguous or
unreadable); r1c2c 0x302 after r1c2c-b; C2d step 1 when the firmware ignored the rings or frames landed
without a forward; r1c2c-m (or BDF/calibration) when buffers were placed but nothing was written; for
r1c2c-m, rows L3/L4 of 1.2 when rx-frame stayed 0 everywhere, else r1c2c-all; otherwise owner review.
Boots without a W26 bit keep C2b's exact hint line. Every W26 line is at most 254 bytes with every field
at its widest (240 by the host test's count).

### Expected log, r1c2c-all (w20 0x702; the model's happy path, values from the model)

```
FUXI WLAN: salami fuxiwlanw20 from ADT /defaults
FUXI WLAN: W20 selected qrtr=7 (CE/HTC/WMI init, then W23 passive scan) w20 0x702 valid 1
FUXI WLAN: W23 selected passive scan of 38 channels, dwell 150 ms, nothing transmitted; BSSIDs OUI only, pdev 0, htt version asked
FUXI WLAN: W26 selected: rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100); wbm idle list 32767 link descriptors (w20 0x200); chain mask pdev tx/rx (w20 0x400); nothing selected to host rings
  ... W15 and W20 lines through `W20 ce init ok rings 7 writes 70 window restore checked`, with the W12/W13 map lines of each arena ...
FUXI WLAN: W26 link arena IOVA 0x... bytes 0x440000 Normal-NC mapped
FUXI WLAN: W26 rx arena IOVA 0x... bytes 0x1040000 Normal-NC mapped; rdp 0x... wrp 0x...
FUXI WLAN: W26 wbm idle before: base-lsb 0 base-msb 0 misc 0 r2-hp 0 r2-tp 0 idle-list-cntl 0 idle-list-size 0 scatter-base 0/0; not owned
FUXI WLAN: W26 wbm idle ring 120 base 0x... 65536 words, 32767 link descriptors (rbm 1, our cookie) misc 0x43 readback ok, hp 65534 via shadow 19 (0x948); writes 8 window restore checked
FUXI WLAN: W26 refill ring 160 filled 4095 buffers (rbm 6, 2048 B at a 4096 B stride, sentinel 0xa5 x64, slots 0x...-0x...) hp 8190 in wrp[0]
FUXI WLAN: W20 ce rx posted CE1 62 CE2 62
  ... W20 htc ready, W21, W22 lines as C2 ...
FUXI WLAN: W23 htt version 3.M (w20 0x02)
FUXI WLAN: W26 htt sring setup refill 160 info0 0x0205000b info1 0x00022000 intr 0x003e0000 low 1024
FUXI WLAN: W26 htt sring setup mac0 161 info0 0x0100010b info1 0x02021000 intr 0x003e0000 low 512
FUXI WLAN: W26 htt sring setup mac1 169 info0 0x0100020b info1 0x02021000 intr 0x003e0000 low 512
FUXI WLAN: W26 htt sring setup rxdma-err 186 info0 0x0007010b info1 0x00082000 intr 0x00200008 low 0
FUXI WLAN: W26 rx watch setup+100ms refill tp T taken N left L mac0 hp H tp T avail A mac1 hp H tp T avail A err hp E fill F; exhausted none
FUXI WLAN: W26 idle watch setup+100ms hp 65534 tp T avail A; exhausted none
FUXI WLAN: W26 chain mask pdev 0 tx 0x3 rx 0x3 sent (PDEV_SET_PARAM 0x4003 ids 1, 2)
  ... C2 lines: chan list, vdev, nss/ps, scan start, `W23 scan started after T ms` ...
FUXI WLAN: W26 rx watch started ...
FUXI WLAN: W26 idle watch started ...
  ... foreign-channel and chan-info lines ...
FUXI WLAN: W26 rx watch mid (foreign 19) ...
FUXI WLAN: W26 idle watch mid (foreign 19) ...
  ... foreign-channel and chan-info lines ...
FUXI WLAN: W23 scan completed reason 0 after T ms (foreign 38 chan-info 38)
FUXI WLAN: W26 rx watch completed ...
FUXI WLAN: W26 idle watch completed ...
  ... W24 bss / summary / detail lines, the C2b stats lines, `W23 stats after-scan ...` ...
FUXI WLAN: W26 rx content mac0 entries N valid V foreign 0 rbm-other R cookie-other C
FUXI WLAN: W26 rx content mac1 entries N valid V foreign 0 rbm-other R cookie-other C
FUXI WLAN: W26 rx content refill taken T kept K; buffers written W of 4095
FUXI WLAN: W26 rx guard nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 0
FUXI WLAN: W26 rxdma err entries E (hp H) push err A routing B flush C other D; rbm link N buf N other N
FUXI WLAN: W26 link content descriptors written D of 32767, unlisted slot clean; idle ring entries changed C
FUXI WLAN: W26 counters htt-t2h N htt-rx N wmi-events N unknown N bad-reports 0 sring-sent 4
FUXI WLAN: W26 htt-t2h types 0:1 ...
FUXI WLAN: W24 no beacon or probe response heard (chan-info 38); next rung NEXT       (only when frames is 0)
FUXI WLAN: W23 complete; stop/contain
FUXI WLAN: W26 rx contain: host writes off; arenas held until bus master off, PERST and SMMU undo
FUXI WLAN: W23 contain: scan over, no STOP_SCAN
FUXI WLAN: W20 ce contain posted P reaped R tx T
FUXI WLAN: W11 QRTR BYE sent before containment
FUXI WLAN: abort step 28 MHI reset done, bus master off yes, PERST asserted yes
FUXI WLAN: W26 rx guard after containment nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 0
FUXI WLAN: W26 arenas zeroed 0x1480000 bytes before free
FUXI WLAN: abort buffers freed
```

r1c2c-b (w20 0x200, 512) prints of these only the selected line (`rx rings no; ... chain mask no`), with
`htt version not asked` in the W23 one, the link arena / pre-read / ring 120 lines (or the not-programmed
line), four `W26 idle watch` lines (the setup one without the 100 ms look, right before the channel
list), the link content, counter and types lines, the D4/D5 lines where they apply, the contain line and
`W26 arenas zeroed 0x440000 bytes before free`. r1c2c-m (0x402) prints the selected line, the chain mask
line, the counter and types lines and the D4/D5 lines; it allocates nothing.

### What the host tests cover, and what only the device can answer

Host-tested (model and static checks, see Host tests): every word, register, address and order above,
the refusal of each pre-read case, every failure path, the line widths, containment order and the zeroing
before the free. Only the phone can say: whether the firmware takes the rings and the idle list at all,
whether RXDMA lands frames without ring selection and with which length and TLV layout, whether it writes
past 2048 bytes, whether WBM recycles descriptors (B7), what the error ring holds, and whether any of it
makes the firmware forward MGMT_RX. The cookie values are ours and never come back (no release ring).

### Deviations from the plan's text, all deliberate

- Both arenas are allocated before the idle list is programmed, so the RX arena line prints before the
  pre-read line (2.11 listed it after the ring 120 line): 2.7's rule that an allocation failure leaves the
  WBM untouched holds for 0x302/0x702 too.
- Ring 120 follows L19's order (registers, readback, entries, barrier, HP) as 2.6.4 lists it; 2.3's
  sentence put the entries first. Either is safe (the ring is empty until the HP moves).
- The pre-read line carries nine registers (D1), the content line is three lines (per MAC and the refill
  line), the watch lines print the raw word with `bad`/`unreadable` instead of derived values when a word
  is not usable, the counter line is followed by `W26 htt-t2h types` lines: all to keep the 254-byte rule.
- When the completed watch's refill word is not usable the refill line says `refill taken unknown (no
  usable completed watch)` instead of a meaningless kept.
- A faulting WBM write is itself the one reported (`W26 wbm idle write N of 8 (ADDR) failed`), as for CE
  writes.
- The raw error entries (2.8: w1, w5, w6, w7) are printed with the frame's byte count, fragment and
  sequence numbers masked out: the owner rule (counts only) is stricter than 2.8's wording.
- The guard and D3 lines add `gaps N`: the regions 2.7 marks "device access: none" besides the slot guards
  and the tail; non-zero is a SPILL like the others.
- The D4 rule also watches the pdev stats base tx-frame (the pdev-wide twin of CHAN_INFO's tx-frame).
- A failed stats exchange (C2b's transport failure) still prints the W26 lines, D4 and D5 before
  containment, as the COMPLETED timeout does.

### Proof that boots without a W26 bit are unchanged

Four differentials against 7caf3ed (scripts in the shop's op11-tools/salami-port/wlan/r1/impl/c2c-impl):
the W15 harness log (keep 0 and 1) byte for byte; the init/configure logs of 843 boot-arg sets with qrtr
absent or 0..8 and w20 values without a W26 bit (the r1c2 and r1c2b kit sets included) on the salami and
the Fuxi build, byte for byte; 7caf3ed's model with all its C1, C2 and C2b scenarios (88 logs) against
7caf3ed's and C2c's production code, byte for byte; and C2c's model + code against 7caf3ed's model + code:
the 88 shared scenario logs byte for byte, C2c only adds scenarios after them.

## C2d: W27, RX ring selection and the REO/WBM buffer return path (`fuxiwlanqrtr=7` and `8`, w20 0x800, 0x1000)

Why: on the phone r1c2c-all (w20 0x702) woke the RX engine (CHAN_INFO rx-clear became a real per-channel CCA) and the
firmware took all 4095 refill buffers into its MAC rings (mac0 2047 valid, mac1 2016 valid, 32 kept, refill left 0),
but RXDMA wrote nothing (0 of 4095 sentinels changed, error ring 0, link descriptors written 0), the scan stopped
after about 15 channels (no more CHAN_INFO, COMPLETED timeout at 8750 ms, the BHI ERRDBG words changed), and no WMI
MGMT_RX came. 0x402 and 0x200 alone changed nothing. Reading: the rings plus SRING_SETUP turn RX on, but L19 never
scans without two more things: (1) RX_RING_SELECTION_CFG on each MAC ring (buffer size, TLV offsets, filters;
dp_rx.c:4567-4605, dp_tx.c:1363-1510), sent right after the SRING_SETUPs (dp_rx.c:4670); (2) the UMAC RX path and the
buffer return: the WBM2SW3 release ring, the REO exception ring and the REO setup in dp_alloc (dp.c:491-595), the
eight REO destination rings after service ready (core.c:925), and host replenish of refill ring 160 as buffers come
back. In L19 STA mode management frames are in no filter flag: they reach the firmware and come back as WMI MGMT_RX;
the host rings only have to keep the RX pipeline moving. C2d gives each half behind its own w20 bit, in L19's words
and order, and returns every buffer the device hands back.

Code: the C2d section of `fuxi_wlan_ce.h` (`WLRD_*`, `struct wlrd`, `wlrd_sel_words`, `wlrd_layout`,
`wlrd_pre_addr/owned/why`, `wlrd_ring_regs`, `wlrd_hw_regs`, `wlrd_reg_ok`, `wlrd_slots_init`, `wlrd_recycle`,
`wlrd_link`, `wlrd_entry`, `wlrd_reap`, `wlrd_replenish`, `wlrd_guard_count`, `wlrd_next`; pure, bounded, no
allocation) and the W27 part of the W20 block of `fuxi_wlan.c` (`wl27_reo`, `wl27_reo_rd`, `wl27_shadow`, `wl27_tp`,
`wl27_ring`, `wl27_setup`, `wl27_dest`, `wl27_sel`, `wl27_poll`, `wl27_watch`, `wl27_report`; the REO arena
`wl_reo_arena` is in `wl_owned_alias` and is allocated, counted, zeroed and freed with the W26 arenas).

### Gate and kits

`WL23_BITS` gains `WL27_BITS` (0x800 RXSEL, 0x1000 REORX); `WL20_BITS` (qrtr 6) does not, so qrtr 6 refuses both.
Each needs 0x100 (refused without it, which with C2c's rule also means 0x02); 0x200 and 0x400 are optional;
keep/stop22, 0x08, 0x20 and unknown bits stay refused. A valid boot with a W27 bit prints, after the W26 selected line:
`W27 selected: ring selection mac0/mac1 2048 B, mpdu-start/packet/msdu-end tlvs, data filters (w20 0x800)|no; reo rx
path rings 131/0/1-8, buffers back to ring 160 (w20 0x1000)|no; nothing transmitted`.

| Kit | w20 | Decimal | Adds to C2 + C2b |
|---|---|---|---|
| r1c2d | 0x1902 | 6402 | HTT version, RX rings (0x100), ring selection, the REO path |
| r1c2d-l | 0x1b02 | 6914 | as r1c2d, with the WBM idle list (0x200): REO exception link descriptors |
| r1c2d-s | 0x902 | 2306 | fallback: ring selection alone (when the REO pre-reads refuse) |
| r1c2d-r | 0x1102 | 4354 | diagnostic: the REO path alone |

Route as C2c: r1c2's cmdline byte for byte (`fuxiwlan=28 fuxiwlanclk=1 fuxiwlanqrtr=7`, no w20) and
`fuxiwlanw20=<decimal>` in ADT /defaults (`--wlan-defaults`); `wl_arg` logs `salami fuxiwlanw20 from ADT /defaults`.
r1c2d first; r1c2d-l next whatever r1c2d shows (L19 always has the idle list); the two halves only to tell them apart.

### Sequence (L19 order)

1. W20, inside `wl26_setup`: the REO arena (1 MiB, 2 MiB aligned, Normal-NC, live SMMU readback) right after the RX
   arena, so an allocation failure leaves every WBM and REO register untouched. After the idle list (0x200) and before
   the refill fill (dp_alloc: idle ring, common setup, RX alloc):
   - 38 allowlisted pre-reads, one line: BASE_LSB, BASE_MSB, MISC of the ten rings, GEN_ENABLE, MISC_CTRL, the four
     aging thresholds, DEST_RING_CTRL_IX_2/_3. Refusal, with no REO or release-ring write at all (`w20 0x1000 dropped
     for this boot`, the run goes on, 0x800 still runs): any ~0u, a ring with SRNG_ENABLE, a ring base, or the hash
     map already in IX_2/IX_3.
   - The WBM2SW3 release ring (131), then the REO2SW0 exception ring (0): nine writes each in
     `ath12k_hal_srng_dst_hw_init`'s order without its MSI writes (BASE_LSB, BASE_MSB, ID, PRODUCER_INT, HP_ADDR
     LSB/MSB, R2 HP 0, R2 TP 0, MISC SRNG_ENABLE), BASE_LSB and MISC read back (a mismatch stops the run).
   - `ath12k_hal_reo_hw_setup`: GEN_ENABLE read and written with the two aging bits, MISC_CTRL read and written with
     the FRAG and BAR destination fields = REO2SW0, the four aging thresholds, the hash map into IX_2 and IX_3.
   - After the refill publish: slots 0..4094 are out with the device, slot 4095 is never posted.
2. W21, after `W21 wmi service ready complete` and before INIT (core.c:925 between service ready and cmd_init): the
   eight destination rings REO2SW1..8, each read again first (a ring found enabled or based now stops the run: the
   release and exception rings are already live), nine writes, two readbacks.
3. W23, right after the four SRING_SETUP and before the 100 ms look: two RX_RING_SELECTION_CFG (mac0 then mac1)
   through `wl_ce_send` on the HTT endpoint, no reply asked (L19 asks none).
4. Every poll pass (`wl_ce_poll`, after the CE reaps, before the W11 guard), from the first ring on: for each live
   ring in setup order, the device HP from the REO arena's rdp page (read barrier), every entry checked and sorted as
   L19 sorts it, the ring's TP published (write barrier, then the shadow register WLAN_CFG gave its R2 TP, or R2 TP
   through the window with fuxiwlanw15=1); then the replenish: held buffers posted at the refill HP while the
   firmware's TP leaves room, one write barrier, one wrp[0] publish.
5. Returned buffers: the address must be the start of a posted slot that is out with the device, or the run stops
   (`W27 reo RING entry refused: WHY; contain`); the cookie is ours, another one is counted. The slot is held, its
   2048 posted bytes zeroed, its 0xa5 sentinel written again, then queued for ring 160. REO exception link
   descriptors (buffer type 1) must be one of our listed descriptors (0x200), their MSDUs (up to 6) are returned and
   the descriptor is held (no SW2WBM ring here); without our idle list a link entry is counted as `not ours` and not
   read. Release entries other than REL_MSDU from RXDMA or REO with RBM SW3 are skipped as L19 skips them (the buffer
   stays lost, counted).
6. Watches as C2c (setup, started, mid, completed) plus one `W27 reo watch` line each; after the W26 report lines the
   W27 report, then (D5) the no-beacon hint with `wlrd_next`'s next step.
7. Containment: `wl26_contain` latches `wl_rd.stopping` first (no reap, return, replenish or REO access from then on);
   the REO arena is recounted (D3), zeroed and freed with the W26 arenas only in `wl5_abort`'s proven branch, else
   quarantined.

Register accesses, nothing else: the 38 pre-reads, the 9 x 10 ring writes, 2 x 10 readbacks, 3 x 8 W21 re-reads, the
two read-modify-writes and six writes of the REO setup, and TP publishes, all through `wl27_reo` (allowlist
`wlrd_reg_ok`, refused after the latch) and `wl_win`, or to the ten shadow registers. No interrupt, MSI, GIC, power
or PMIC access, no REO command, no TCL, nothing transmitted.

### Wire, register and memory facts (all re-derived from L19 by the host test)

RX_RING_SELECTION_CFG (msg 0xc, 17 words, struct htt_rx_ring_selection_cfg_cmd dp.h:1202-1219):

| Word | mac0 / mac1 | L19 |
|---|---|---|
| 0 info0 | 0x1400010c / 0x1400020c: msg 0xc, pdev mac + 1 (SW_TO_HW ring), ring id RXDMA_HOST_BUF 0, OFFSET_VALID 26, EN_RXMON 28 (rxmon_disable 0) | dp_tx.c:1392-1415, 1102-1130; dp.h:840-847 |
| 1 info1 | 0x800: buffer size DP_RXDMA_REFILL_RING_SIZE 2048 (what L19 passes), conf lengths 0 | dp_rx.c:4600-4602; dp.h:191, 849-852 |
| 2, 3 | 0, 0 (no management or control flag: mgmt stays with the firmware) | dp_rx.c:4578-4582 |
| 4 | 0x01000000: FP control BAR | dp.h:971 |
| 5 | 0x01240000: FP data MCAST 18, UCAST 21, NULL_DATA 24 | dp.h:1001-1007 |
| 6 | 0xd: MPDU_START 0, RX_PACKET 2, MSDU_END 3 (HTT_RX_TLV_FLAGS_RXDMA_RING) | dp.h:876-884, 1194-1197 |
| 7 | 0x01100190: packet offset 400 = sizeof(hal_rx_desc_wcn7850), header 272 = its pkt_hdr_tlv | rx_desc.h:1521-1529; hal.c:1505-1508 |
| 8 | 0x00900000: mpdu_end 0, mpdu_start 144 | hal.c:1242-1245 |
| 9 | 0: msdu_end 0 (hal.c:1247-1250), msdu_start 0 | dp_tx.c:1467-1473 |
| 10..16 | 0 (no word masks on WCN7850) | dp_tx.c:1475-1505 |

UMAC rings (entries of 8 words; the REO arena at A):

| Ring | id | Entries | R0 | R2 HP / TP | Base | PRODUCER_INT | TP shadow |
|---|---|---|---|---|---|---|---|
| WBM2SW3 release | 131 | 1024 | 0x00a34f70 | 0x00a370e0 / e4 | A + 0x92000 | 0x00200008 | 24 (0x95c) |
| REO2SW0 exception | 0 | 128 | 0x00a388a4 | 0x00a3b088 / 8c | A + 0x90000 | 0x00200008 | 8 (0x91c) |
| REO2SW1..8 destination | 1..8 | 2048 | 0x00a384e4 + 0x78 n | 0x00a3b048 + 8 n / + 4 | A + 0x10000 (n + 1) | 0x003e0400 | n (0x8fc + 4 n) |

BASE_MSB = entries x 8 << 8, ID = id << 8 | 8, HP_ADDR = A + 4 x id, MISC = 0x40. REO setup: GEN_ENABLE |= 0xc;
MISC_CTRL &= ~0x01fe0000 (FRAG and BAR destination REO2SW0); aging 0x00a38b08..b14 = 100000, 100000, 100000, 40000;
DEST_RING_CTRL_IX_2 0x00a3800c and IX_3 0x00a38010 = 0x43214321. All in window 0x14.

REO arena (0x100000): rdp page (only words 0..8 and 131 are HP words), destination rings at 0x10000..0x90000, exception
0x90000, release 0x92000, tail from 0x9a000. Its gaps and tail are counted like C2c's (`W27 reo guard`, again after
containment); non-zero is a SPILL.

### Lines

```
FUXI WLAN: W27 reo arena IOVA I bytes 0x100000 Normal-NC mapped; rdp I
FUXI WLAN: W27 reo before: rings 10 enabled N based N unreadable N gen-enable X misc-ctrl X aging X/X/X/X dest-ctrl X/X; not owned|refused
FUXI WLAN: W27 reo not programmed: REASON; w20 0x1000 dropped for this boot                            (refused)
FUXI WLAN: W27 reo ring ID NAME entries N base B misc 0x40 readback ok; tp via shadow S (0x...)|R2 0x... (fuxiwlanw15=1)
FUXI WLAN: W27 reo setup gen-enable X -> X misc-ctrl X -> X aging 100000/100000/100000/40000 hash 0x43214321; writes 26 window restore checked
FUXI WLAN: W27 htt rx ring selection macM ID info0 X buf 2048 flags 0/0/0x1000000/0x1240000 tlv 0xd offsets 0x01100190/0x00900000/0 sent
FUXI WLAN: W27 reo watch POINT reaped rel N exc N dst N; returned N reposted N pending N (max N) refill hp H
FUXI WLAN: W27 rx ring selection sent N of 2
FUXI WLAN: W27 reo reaped rel N exc N dst N N N N N N N N
FUXI WLAN: W27 reo entries msdu N link N (held N, msdus N, skipped N, not ours N) push routing N error N continuation N
FUXI WLAN: W27 reo error codes 0xC:N ...                                                       (non-zero only, wrapped)
FUXI WLAN: W27 release src rxdma N reo N other N desc-other N rbm-other N cookie-other N
FUXI WLAN: W27 release rxdma codes 0xC:N ...                                                   (non-zero only)
FUXI WLAN: W27 buffers returned N reposted N pending N (max N) publishes N refill hp H refill-bad N
FUXI WLAN: W27 reo guard [SPILL ]nonzero dwords: gaps N, tail N[; stop; owner review before any further kit]
FUXI WLAN: W27 reo RING entry refused: WHY; contain                                             (then the W23 failure line)
FUXI WLAN: W27 reo guard after containment nonzero dwords: gaps N, tail N[; SPILL: owner review before any further kit]
```

Every W27 line is at most 254 bytes with every field at its widest (the host test computes the worst case; the
model fails any longer line). Nothing read from a frame is printed: counts, codes and our own ring words only.

### Expected log, r1c2d (w20 0x1902; the model's happy path, values from the model)

```
FUXI WLAN: salami fuxiwlanw20 from ADT /defaults
FUXI WLAN: W20 selected qrtr=7 (CE/HTC/WMI init, then W23 passive scan) w20 0x1902 valid 1
FUXI WLAN: W23 selected passive scan of 38 channels, dwell 150 ms, nothing transmitted; BSSIDs OUI only, pdev 0, htt version asked
FUXI WLAN: W26 selected: rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100); wbm idle list no; chain mask no; nothing selected to host rings
FUXI WLAN: W27 selected: ring selection mac0/mac1 2048 B, mpdu-start/packet/msdu-end tlvs, data filters (w20 0x800); reo rx path rings 131/0/1-8, buffers back to ring 160 (w20 0x1000); nothing transmitted
  ... W15 and W20 lines through `W20 ce init ok rings 7 writes 70 window restore checked` ...
FUXI WLAN: W26 rx arena IOVA 0x... bytes 0x1040000 Normal-NC mapped; rdp 0x... wrp 0x...
FUXI WLAN: W27 reo arena IOVA 0x... bytes 0x100000 Normal-NC mapped; rdp 0x...
FUXI WLAN: W27 reo before: rings 10 enabled 0 based 0 unreadable 0 gen-enable G misc-ctrl M aging X/X/X/X dest-ctrl X/X; not owned
FUXI WLAN: W27 reo ring 131 rel entries 1024 base 0x... misc 0x40 readback ok; tp via shadow 24 (0x95c)
FUXI WLAN: W27 reo ring 0 exc entries 128 base 0x... misc 0x40 readback ok; tp via shadow 8 (0x91c)
FUXI WLAN: W27 reo setup gen-enable G -> G|0xc misc-ctrl M -> M&~0x1fe0000 aging 100000/100000/100000/40000 hash 0x43214321; writes 26 window restore checked
FUXI WLAN: W26 refill ring 160 filled 4095 buffers (rbm 6, 2048 B at a 4096 B stride, sentinel 0xa5 x64, slots 0x...-0x...) hp 8190 in wrp[0]
FUXI WLAN: W20 ce rx posted CE1 62 CE2 62
  ... W20 htc ready, W21 lines through `W21 wmi service ready complete` ...
FUXI WLAN: W27 reo ring 1 dst1 entries 2048 base 0x... misc 0x40 readback ok; tp via shadow 0 (0x8fc)
  ... dst2 .. dst8 (shadow 1 .. 7) ...
  ... W22 lines as C2 ...
FUXI WLAN: W23 htt version 3.M (w20 0x02)
  ... the four `W26 htt sring setup` lines as C2c ...
FUXI WLAN: W27 htt rx ring selection mac0 161 info0 0x1400010c buf 2048 flags 0/0/0x1000000/0x1240000 tlv 0xd offsets 0x01100190/0x00900000/0 sent
FUXI WLAN: W27 htt rx ring selection mac1 169 info0 0x1400020c buf 2048 flags 0/0/0x1000000/0x1240000 tlv 0xd offsets 0x01100190/0x00900000/0 sent
FUXI WLAN: W26 rx watch setup+100ms ...
FUXI WLAN: W27 reo watch setup+100ms reaped rel 0 exc 0 dst 0; returned 0 reposted 0 pending 0 (max 0) refill hp 8190
  ... C2 lines: chan list, vdev, nss/ps, scan start, `W23 scan started after T ms` ...
FUXI WLAN: W27 reo watch started reaped rel 0 exc 0 dst 0; returned 0 reposted 0 pending 0 (max 0) refill hp 8190
  ... foreign-channel and chan-info lines, all 38 channels ...
FUXI WLAN: W27 reo watch mid (foreign 19) reaped rel 270 exc 540 dst 1350; returned 2160 reposted 2160 pending 0 (max 120) refill hp 4318
FUXI WLAN: W23 scan completed reason 0 after T ms (foreign 38 chan-info 38)
FUXI WLAN: W27 reo watch completed reaped rel 570 exc 1140 dst 2850; returned 4560 reposted 4560 pending 0 (max 120) refill hp 926
FUXI WLAN: W24 bss aa:bb:cc:xx:xx:xx ... ssid "NAME"                                              (one per BSSID)
FUXI WLAN: W24 scan summary frames 12 bss 9 ssids 5
  ... C2b stats lines, the W26 content, guard, err and counter lines ...
FUXI WLAN: W27 rx ring selection sent 2 of 2
FUXI WLAN: W27 reo reaped rel 570 exc 1140 dst 570 1140 570 570 0 0 0 0
FUXI WLAN: W27 reo entries msdu 3990 link 0 (held 0, msdus 0, skipped 0, not ours 0) push routing 2850 error 1140 continuation 0
FUXI WLAN: W27 reo error codes 0x5:1140 0x9:570
FUXI WLAN: W27 release src rxdma 0 reo 570 other 0 desc-other 0 rbm-other 0 cookie-other 0
FUXI WLAN: W27 buffers returned 4560 reposted 4560 pending 0 (max 120) publishes 38 refill hp 926 refill-bad 0
FUXI WLAN: W27 reo guard nonzero dwords: gaps 0, tail 0
FUXI WLAN: W23 complete; stop/contain
FUXI WLAN: W26 rx contain: host writes off; arenas held until bus master off, PERST and SMMU undo
FUXI WLAN: W23 contain: scan over, no STOP_SCAN
FUXI WLAN: W20 ce contain posted P reaped R tx T
FUXI WLAN: W11 QRTR BYE sent before containment
FUXI WLAN: abort step 28 MHI reset done, bus master off yes, PERST asserted yes
FUXI WLAN: W26 rx guard after containment nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 0
FUXI WLAN: W27 reo guard after containment nonzero dwords: gaps 0, tail 0
FUXI WLAN: W26 arenas zeroed 0x1140000 bytes before free
FUXI WLAN: abort buffers freed
```

The model sends five frames in eight to destination rings 1..4 (by its own hash), two to the exception ring (code 5)
and one to the release ring (REO source, code 9); on the phone the split, the codes and the
counts are the firmware's. Before association almost nothing should be routed to a destination ring (no peer): expect
most returns on the exception and release rings. The 0x1b02 log is the same plus C2c's link arena, idle list and idle
watch lines, `W26 link content descriptors written D`, and link entries on the exception ring:
`W27 reo entries msdu 3420 link 570 (held 570, msdus 570, ...)`, `W27 reo error codes 0x3:570 0x5:570 0x9:570`,
`W26 arenas zeroed 0x1580000 bytes before free`.

With 0x1000 the refill HP moves, so C2c's watch and content fields that assume the fixed HP 8190 (`refill ... taken
N left L`, `exhausted refill`, `refill-empty`, `refill taken T kept K`, `buffers written`) are ring positions after a
wrap, not counts: read the W27 lines for the refill (`refill hp`, `returned`, `reposted`, `pending`). The MAC ring,
error ring, guard and idle-list lines keep their meaning.

### Reading the next boot

| Pattern | Conclusion | Next |
|---|---|---|
| W24 frames > 0 | C2 PASS: the RX pipeline moves and the firmware forwards beacons | C3 |
| all 38 channels tuned (foreign 38), returned > 0 and reposted = returned, frames 0 | the pipeline moves, nothing is forwarded to WMI | C3 management forwarding (`wlrd_next`'s hint) |
| the scan still stops after about 15 channels, reaped 0 everywhere | the path is not what the firmware waits for | owner review: the REO command/status rings and cookie conversion (open questions) |
| `W27 reo not programmed: ... already owned` | the firmware programmed the REO rings itself | r1c2d-s (0x902, 2306) |
| `not programmed: ... unreadable` or `... already applied` | ownership unknown | owner review |
| `W27 reo RING entry refused` | the device returned a buffer we do not own, or one twice | stop, owner review before any further kit |
| `SPILL` on either guard line | a device write where no address was given | stop, owner review |
| `rbm-other` > 0 on release entries | WBM returned with another manager: those buffers are lost (as L19) | read `W27 release src`, owner review if large |
| `pending` > 0 at the end, `refill-bad` > 0 | the refill TP word was not usable, buffers held | owner review |
| `link ... not ours` > 0 | link descriptors from a list we did not give (no 0x200, or the firmware's own) | r1c2d-l (0x1b02) |
| any D4 line | the passive rule is broken | stop the radio lane |

### Deviations from L19, all deliberate

- No MSI registers (MSI1 base/data) on any of the ten rings; the PRODUCER_INT thresholds are L19's (they only shape
  interrupts, which nothing enables).
- No REO command or status ring: L19 sets them up in the same common setup, but sends REO commands only for peer
  queues (TID setup, delete, flush), which a passive scan without a peer never needs. Open question if the firmware
  waits for them.
- No cookie conversion (`ath12k_dp_cc_config`, CMEM tables): the entries carry our cookie in buffer word 1 and the
  host returns by address (the cookie only counted); L19's own manual lookup path reads the same cookie.
- No REO reinject, TCL, TX completion or SW2WBM release ring: REO exception link descriptors are held until
  containment instead of going back to the idle list (L19 returns them on SW2WBM).
- Returned buffers are the same 4095 slots posted again with a fresh sentinel (L19 posts fresh buffers); a foreign or
  twice-returned buffer stops the run (L19 warns).
- The destination rings are read again right before their writes at W21; one found configured stops the run.

### What the host tests cover, and what only the device can answer

Host-tested: every word, register, address, value and order above against L19, the refusals, the latch, the free,
the line widths, the wedge without the path (the model reproduces the phone's r1c2c-all readout), landing with it,
exhaustion and replenish, foreign and twice-returned buffers, an unknown return manager, spills, containment while
the device still pushes, and a mutant run per rule. Only the phone can say: whether the firmware needs the REO
command/status rings or cookie conversion before it moves frames, which rings it routes to without a peer, whether it
then forwards MGMT_RX, and whether WBM returns on WBM2SW3 with RBM SW3.

### Proof that boots without a W27 bit are unchanged

Three differentials against c18f99d, each tree built from its own sources (scripts in this session's scratchpad,
recorded in the commit body): the W15, W11 and W16 harness logs (every line the harnesses print through their
test_printf, 14819, 14016 and 17011 lines) byte for byte; the init/configure gate (salami and Fuxi builds, cmdline
and ADT /defaults) over 6176 boot-arg sets with qrtr absent, 0, 4..8 or -1, 32 w20 values without a W27 bit (the
C2c kit values included), mlo, w15/w16, keep, stop22, clk and limits 1/11/27/28: 24704 runs per tree, byte for byte,
plus the 207 gate runs both trees' salami tests share; and c18f99d's model with all its C1, C2, C2b and C2c scenarios
(17611 lines) equal byte for byte to the first part of the C2d model's log, the C2d scenarios only adding lines after
them.

## C2e: W28, the RXDMA monitor status rings and the crash line (`fuxiwlanqrtr=7` and `8`, w20 0x2000)

Why: on the phone C2c (r1c2c-all, 0x702; rings 160/161/169/186 with 4095 buffers) woke the RX engine, the firmware moved
every refill buffer into the MAC rings within 100 ms, RXDMA wrote nothing, and the scan wedged after about 15 channels
(no more events, WMI quiet, BHI ERRDBG2 0x152ee53 / ERRDBG3 0x492, the MHI reset unconfirmed afterwards): a firmware
crash. C2d (r1c2d-l, 0x1b02: C2c + the idle list + RX_RING_SELECTION_CFG with L19's STA words + the REO/WBM return path)
programmed and read back everything and still wrote and reaped nothing; the wedge came earlier (3 channels) with the
same ERRDBG words. Reading: once host RX is live the firmware crashes on the first PPDUs, and sooner with L19's ring
selection (info0 0x1400010c sets EN_RXMON). The one ring set L19 always gives WCN7850 before any scan that C2d still
lacks is the two RXDMA monitor status rings with host status buffers (dp_rx.c:4733-4748 allocate, 450-517 fill,
4657-4670 announce, right before the ring selection at 4672). C2e gives them behind one w20 bit, in L19's geometry and
order, watches them without reaping (as L19 in STA mode), and adds a read-only crash line.

Code: the C2e section of `fuxi_wlan_ce.h` (`WLMS_*`, `struct wlms`, `wlms_layout`, `wlms_expect`, `wlms_fill`,
`wlms_publish`, `wlms_sring_words` over C2c's `wlrx_sring_ring`, `wlms_written`, `wlms_watch`, `wlms_guard_count`,
`wlms_content`, `wlms_next`, `wlms_diag_addr/ok`; pure, bounded, no allocation) and the W28 part of the W20 block of
`fuxi_wlan.c` (`wl28_rd`, `wl28_crash`, `wl28_sring`, `wl28_watch`, `wl28_report`; the arena `wl_mon_arena` is in
`wl_owned_alias` and is allocated, counted, zeroed and freed with the W26 arenas). C2c's `wlrx_guard_count` names
rdp[185]/[193] and wrp[25]/[33] (not gaps) once the monitor status arena is laid out; without 0x2000 it counts exactly
as before.

### Gate and kits

`WL23_BITS` gains 0x2000 MONSTAT; `WL20_BITS` (qrtr 6) does not, so qrtr 6 refuses it. It needs 0x100 (refused without
it, which with C2c's rule also means 0x02); every other C2c/C2d bit stays optional; keep/stop22, 0x08, 0x20 and unknown
bits stay refused. qrtr 8 (C3, rebased onto C2e) takes it as qrtr 7 does, as it takes W26 and W27 (see C3,
[With C2b, C2c, C2d and C2e](#with-c2b-c2c-c2d-and-c2e-the-rebase-onto-0f48029)). A valid boot with 0x2000 prints, after the
W27 selected line (or the W26 one): `W28 selected: monitor status rings 185/193, 1023 buffers each (w20 0x2000),
watched, not reaped; crash line at the COMPLETED timeout and before MHI reset; nothing transmitted`.

| Kit | w20 | Decimal | What it is |
|---|---|---|---|
| r1c2e | 0x3b02 | 15106 | C2d's r1c2d-l (0x1b02) + the monitor status rings: L19's whole RX setup but REO cmd/status and cookie conversion |
| r1c2e-r | 0x2302 | 8962 | HTT version, RX rings, the idle list and the monitor status rings only: no ring selection, no REO path |

Route as C2c/C2d: r1c2's cmdline byte for byte (`fuxiwlan=28 fuxiwlanclk=1 fuxiwlanqrtr=7`, no w20) and
`fuxiwlanw20=<decimal>` in ADT /defaults (`--wlan-defaults`); `wl_arg` logs `salami fuxiwlanw20 from ADT /defaults`.
r1c2e first; r1c2e-r tells whether the monitor status rings alone keep RX alive without the selection (the C2c wedge
had no selection).

### Sequence (L19 order)

1. W20, inside `wl26_setup`: the monitor status arena (0x820000 bytes, 2 MiB aligned, Normal-NC, live SMMU readback)
   after the RX, link and REO arenas, so an allocation failure leaves every WBM and REO register untouched
   (`W20 failure: W28 mon arena allocation/NC/map`). After the refill fill and its wrp[0] publish
   (`ath12k_dp_rxdma_buf_setup`: the refill ring, then each status ring): for ring 185, then ring 193, the 0xa5
   sentinel over bytes 0..63 of slots 0..1022, the 1023 entries (slot start, RBM 6, cookie = the slot index from 0),
   the write barrier, HP 2046 into wrp[25] (wrp[33]). The device learns nothing of this until SRING_SETUP.
2. W23, right after the four C2c SRING_SETUP and before the two RX_RING_SELECTION_CFG (0x800) and the 100 ms look: two
   SRING_SETUP (mon0 185, mon1 193) through `wl_ce_send` on the HTT endpoint, no reply asked.
3. Every W26 watch (setup+100ms, started, mid, completed): after the W26 and W27 lines one `W28 mon watch` line (both
   TPs from rdp[185]/[193] after the read barrier, the HP we published, taken, left, buffers written, exhaustion).
4. After the W27 report: `W28 mon sring sent`, the content per ring and the guard line; then the D5 hint.
5. Crash line, read-only: right after `W23 timeout scan COMPLETED`, and in `wl5_abort` after the CE containment and
   the W11 stop, right before the MHI reset write.
6. Containment: `wl26_contain` latches `wl_ms.stopping` (no fill or publish from then on; nothing else ever writes the
   arena); only in `wl5_abort`'s proven branch is the arena recounted, zeroed and freed, else quarantined.

Register accesses, nothing else: the six reads of the crash line through `wl28_rd` (`wlms_diag_ok`: MHISTATUS 0x48,
BHI EXECENV, ERRCODE, ERRDBG1..3 at BHIOFF + 0x28, 0x30, 0x34..0x3c, the same six the W11/W15 diagnostic reads; BHIOFF
inside the mapped 16 KiB and word aligned). No register write at all (SRING_SETUP is an HTT message), no reap, no TP
or HP write after W20, no interrupt, MSI, GIC, power or PMIC access, no WMI command; nothing transmitted (PASSIVE
forced, 0x08/0x20 refused, the D4 owner-review line unchanged).

### Wire, register and memory facts (all re-derived from L19 by the host test)

| Ring | HAL id | HTT ring, type, pdev | Entries x bytes | HP / TP words | Buffers |
|---|---|---|---|---|---|
| mon0 | 185 (HAL_SRNG_RING_ID_WMAC1_SW2RXDMA1_STATBUF, mac 0) | RXDMA_MONITOR_STATUS 1, SW_TO_HW 1, pdev 1 | 1024 x 8 | wrp[25] / rdp[185] | 1023 |
| mon1 | 193 (+ 8, PMAC stride) | RXDMA_MONITOR_STATUS 1, SW_TO_HW 1, pdev 2 | 1024 x 8 | wrp[33] / rdp[193] | 1023 |

SRING_SETUP with the RX arena at A and the monitor status arena at M: mon0 info0 0x0101010b, base M, info1 0x02020800
(2048 words, entry 2 words, LOOP_CNT_DIS), hp A + 0x464, tp A + 0x2e4, MSI 0/0/0, intr 0x003e0002 (batch 1 x 2 words,
500 us >> 3), low 256 (1024 >> 3 entries in words); mon1 0x0101020b, M + 0x4000, 0x02020800, A + 0x484, A + 0x304,
0x003e0002, 256. L19's thresholds are dp.c:284-289 (low threshold with LOW_THRESH_INTR_EN, batch 1, timer RX); L19
fills the three MSI words from its WCN7850 rx_mon_status ring mask (hw.c:898-903); here they are 0, as for every ring.

Entries (`ath12k_hal_rx_buf_addr_info_set`): address 31:0; address 39:32 in 7:0, RBM rx_buf_rbm SW1 6 in 11:8, cookie
in 31:12 = the buffer's idr id from 0 (DP_RXDMA_BUF_COOKIE_BUF_ID 17:0, pdev bits 19:18 left 0). Count: L19 asks size
/ entry size = 1024 (dp.c pads the ring by 7 bytes, the division drops them) and takes min(asked, bufs_max, num_free) =
1023, a fresh source ring's num_free.

Buffer size: L19 allocates `dev_alloc_skb(RX_MON_STATUS_BUF_SIZE)` = RX_MON_STATUS_BASE_BUF_SIZE 2048 less the 128-byte
reservation, the 128-byte alignment and the skb tail, 128-aligned, and maps the skb's whole data area: never more than
the 2048-byte base budget. No ring selection tells the firmware a status buffer length in STA mode (only the monitor
mode config does, dp_tx.c:1655-1665). We post the whole 2048-byte budget at a 4096-byte stride (the upper 2048 bytes a
guard), so any write past it lands in the slot's own guard and is counted.

Monitor status arena (0x820000): ring 185 at 0, a gap to 0x4000, ring 193 at 0x4000, a gap to 0x10000, 2 x 1024 slots
of 4 KiB from 0x10000 (ring 185's slots 0..1023, then ring 193's; slot 1023 of each never posted), a 64 KiB tail from
0x810000. Size: the two rings need 16 KiB and the 2046 status buffers 2046 x 2 KiB = 4 MiB device-writable; at C2c's
4 KiB stride with guards that is 8 MiB of slots plus the 64 KiB head and tail; the RX arena's 128 KiB tail could not
hold it. The lane now holds up to 0x80000 + 0x1040000 + 0x440000 + 0x100000 + 0x820000 = 0x1e20000 bytes (30.1 MiB) of
IOVA-mapped DMA, inside the 256 MiB W12 pool.

### Lines

```
FUXI WLAN: W28 mon arena IOVA I bytes 0x820000 Normal-NC mapped; rings I I slots I
FUXI WLAN: W28 mon status ring 185|193 filled 1023 buffers (rbm 6, cookie 0-1022, 2048 B at a 4096 B stride, sentinel 0xa5 x64, slots S-E) hp 2046 in wrp[25|33]
FUXI WLAN: W28 htt sring setup mon0|mon1 ID info0 X info1 X intr X low N                         (x2)
FUXI WLAN: W28 htt sring setup monM not sent: REASON                                               (then the W23 failure line)
FUXI WLAN: W28 mon watch POINT mon0 hp H tp T taken N left L written W mon1 ...; exhausted none|mon0|mon1|mon0 mon1
FUXI WLAN: W28 mon sring sent N of 2
FUXI WLAN: W28 mon content mon0|mon1 written W of 1023; ring entries changed C (foreign F)
FUXI WLAN: W28 mon guard [SPILL ]nonzero dwords: slot-upper N (slots S), unposted slots N, tail N, gaps N[; stop; owner review before any further kit]
FUXI WLAN: W28 crash scan COMPLETED timeout|contain before MHI reset: mhistatus X (state X syserr N) ee X bhi errcode X errdbg X/X/X; access-faults N; read-only
FUXI WLAN: W28 crash POINT: not read (MHI/BHI offsets not established)
FUXI WLAN: W28 mon guard after containment nonzero dwords: slot-upper N (slots S), unposted slots N, tail N, gaps N[; SPILL: owner review before any further kit]
```

POINT is the W26 watch point. A TP word that is ~0u prints `tp 4294967295 unreadable`, one that is odd or not below
2048 the raw word and `bad`; neither is acted on. taken = tp / 2, left = ((2046 - tp) mod 2048) / 2, exhausted at left 0
(all 1023 buffers with the device: nothing gives one back). written = posted slots whose bytes 0..63 are no longer all
0xa5. changed = ring entries that differ from what the host wrote (the device only reads a source ring), foreign = of
those, the ones whose address is not a posted slot start of that ring. Nothing read from a status buffer is printed.
Every W28 line is at most 218 bytes with every field at its widest (the host test's count).

### Expected log, r1c2e (w20 0x3b02; the model's happy path, values from the model)

```
FUXI WLAN: salami fuxiwlanw20 from ADT /defaults
FUXI WLAN: W20 selected qrtr=7 (CE/HTC/WMI init, then W23 passive scan) w20 0x3b02 valid 1
FUXI WLAN: W23 selected passive scan of 38 channels, dwell 150 ms, nothing transmitted; BSSIDs OUI only, pdev 0, htt version asked
FUXI WLAN: W26 selected: rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100); wbm idle list 32767 link descriptors (w20 0x200); chain mask no; nothing selected to host rings
FUXI WLAN: W27 selected: ring selection mac0/mac1 2048 B, mpdu-start/packet/msdu-end tlvs, data filters (w20 0x800); reo rx path rings 131/0/1-8, buffers back to ring 160 (w20 0x1000); nothing transmitted
FUXI WLAN: W28 selected: monitor status rings 185/193, 1023 buffers each (w20 0x2000), watched, not reaped; crash line at the COMPLETED timeout and before MHI reset; nothing transmitted
  ... W15 and W20 lines through `W20 ce init ok rings 7 writes 70 window restore checked` ...
FUXI WLAN: W26 link arena IOVA 0xa1400000 bytes 0x440000 Normal-NC mapped
FUXI WLAN: W26 rx arena IOVA 0xa1a00000 bytes 0x1040000 Normal-NC mapped; rdp 0xa1a00000 wrp 0xa1a00400
FUXI WLAN: W27 reo arena IOVA 0xa2c00000 bytes 0x100000 Normal-NC mapped; rdp 0xa2c00000
FUXI WLAN: W28 mon arena IOVA 0xa2e00000 bytes 0x820000 Normal-NC mapped; rings 0xa2e00000 0xa2e04000 slots 0xa2e10000
  ... the W26 idle list lines and the W27 pre-read, ring 131/0 and setup lines, as r1c2d-l ...
FUXI WLAN: W26 refill ring 160 filled 4095 buffers (rbm 6, 2048 B at a 4096 B stride, sentinel 0xa5 x64, slots 0xa1a20000-0xa2a1e000) hp 8190 in wrp[0]
FUXI WLAN: W28 mon status ring 185 filled 1023 buffers (rbm 6, cookie 0-1022, 2048 B at a 4096 B stride, sentinel 0xa5 x64, slots 0xa2e10000-0xa320e000) hp 2046 in wrp[25]
FUXI WLAN: W28 mon status ring 193 filled 1023 buffers (rbm 6, cookie 0-1022, 2048 B at a 4096 B stride, sentinel 0xa5 x64, slots 0xa3210000-0xa360e000) hp 2046 in wrp[33]
FUXI WLAN: W20 ce rx posted CE1 62 CE2 62
  ... W20 htc ready, W21 lines and the eight W27 destination rings, W22 lines as r1c2d-l ...
FUXI WLAN: W23 htt version 3.M (w20 0x02)
  ... the four `W26 htt sring setup` lines as C2c ...
FUXI WLAN: W28 htt sring setup mon0 185 info0 0x0101010b info1 0x02020800 intr 0x003e0002 low 256
FUXI WLAN: W28 htt sring setup mon1 193 info0 0x0101020b info1 0x02020800 intr 0x003e0002 low 256
  ... the two `W27 htt rx ring selection` lines as C2d ...
  ... `W26 rx watch setup+100ms`, `W26 idle watch setup+100ms`, `W27 reo watch setup+100ms` ...
FUXI WLAN: W28 mon watch setup+100ms mon0 hp 2046 tp 0 taken 0 left 1023 written 0 mon1 hp 2046 tp 0 taken 0 left 1023 written 0; exhausted none
  ... C2 lines: chan list, vdev, nss/ps, scan start, `W23 scan started after T ms`, the started watches ...
FUXI WLAN: W28 mon watch started mon0 hp 2046 tp 0 taken 0 left 1023 written 0 mon1 hp 2046 tp 0 taken 0 left 1023 written 0; exhausted none
  ... foreign-channel and chan-info lines, all 38 channels ...
FUXI WLAN: W28 mon watch mid (foreign 19) mon0 hp 2046 tp 260 taken 130 left 893 written 130 mon1 hp 2046 tp 100 taken 50 left 973 written 50; exhausted none
FUXI WLAN: W23 scan completed reason 0 after T ms (foreign 38 chan-info 38)
FUXI WLAN: W28 mon watch completed mon0 hp 2046 tp 260 taken 130 left 893 written 130 mon1 hp 2046 tp 500 taken 250 left 773 written 250; exhausted none
  ... W24 bss / summary lines, the C2b stats lines, the W26 content, guard, err, link and counter lines, the W27 report ...
FUXI WLAN: W28 mon sring sent 2 of 2
FUXI WLAN: W28 mon content mon0 written 130 of 1023; ring entries changed 0 (foreign 0)
FUXI WLAN: W28 mon content mon1 written 250 of 1023; ring entries changed 0 (foreign 0)
FUXI WLAN: W28 mon guard nonzero dwords: slot-upper 0 (slots 0), unposted slots 0, tail 0, gaps 0
FUXI WLAN: W23 complete; stop/contain
FUXI WLAN: W26 rx contain: host writes off; arenas held until bus master off, PERST and SMMU undo
FUXI WLAN: W23 contain: scan over, no STOP_SCAN
FUXI WLAN: W20 ce contain posted P reaped R tx T
FUXI WLAN: W11 QRTR BYE sent before containment
FUXI WLAN: W28 crash contain before MHI reset: mhistatus 0x201 (state 0x2 syserr 0) ee 0x2 bhi errcode 0 errdbg 0/0/0; access-faults 0; read-only
FUXI WLAN: abort step 28 MHI reset done, bus master off yes, PERST asserted yes
FUXI WLAN: W26 rx guard after containment nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 0
FUXI WLAN: W27 reo guard after containment nonzero dwords: gaps 0, tail 0
FUXI WLAN: W28 mon guard after containment nonzero dwords: slot-upper 0 (slots 0), unposted slots 0, tail 0, gaps 0
FUXI WLAN: W26 arenas zeroed 0x1da0000 bytes before free
FUXI WLAN: abort buffers freed
```

The model writes ten status PPDUs per channel (mac0 on the 13 2.4 GHz channels, mac1 on the 25 5 GHz ones): 380 status
buffers, none given back. On the phone the PPDU count is RXDMA's: if it writes a status TLV for every PPDU it hears,
expect both rings `exhausted` within the first channels (L19 STA mode reaps none either; read `left` and `written`).
MHISTATUS 0x201 / EE 0x2 is a healthy M0/AMSS; the ERRDBG words are whatever the firmware left there.

### Expected log, r1c2e-r (w20 0x2302; values from the model)

The same as r1c2e without any W27 line and without the REO arena: `W28 mon arena IOVA 0xa2c00000`, the two `W28 mon
status ring` lines (slots 0xa2c10000-0xa300e000 and 0xa3010000-0xa340e000), the two `W28 htt sring setup` lines right
after the four C2c ones, the four `W28 mon watch` lines as above, all 38 channels, then

```
FUXI WLAN: W24 scan summary frames 0 bss 0 ssids 0
FUXI WLAN: W26 rx content refill taken 2048 kept 0; buffers written 0 of 4095
FUXI WLAN: W28 mon sring sent 2 of 2
FUXI WLAN: W28 mon content mon0 written 130 of 1023; ring entries changed 0 (foreign 0)
FUXI WLAN: W28 mon content mon1 written 250 of 1023; ring entries changed 0 (foreign 0)
FUXI WLAN: W28 mon guard nonzero dwords: slot-upper 0 (slots 0), unposted slots 0, tail 0, gaps 0
FUXI WLAN: W24 no beacon or probe response heard (chan-info 38); next rung 0x3b02 (15106): the scan ran with the monitor status rings; add ring selection, the REO path and the idle list
FUXI WLAN: W23 complete; stop/contain
  ... containment as r1c2e, `W26 arenas zeroed 0x1ca0000 bytes before free` ...
```

(The model's firmware takes 2048 refill buffers here and RXDMA writes no data buffer without ring selection; on the
phone those counts are the firmware's.)

### The crash, as the model plays it (`C.mon_crash`, the C2e scenarios only)

With RX live and the monitor status rings missing (C2c's 0x302, C2d's 0x1b02, or 0x3b02 with a firmware that drops the
two SRING_SETUP) the model's firmware stops the scan after 15 channels (3 with ring selection), sends nothing more,
sets MHISTATUS 0xff04 (state SYS_ERR 0xff, SYSERR 1) and ERRDBG2/3 to the phone's 0x152ee53 / 0x492, and never
confirms the MHI reset. 0x3b02 then prints:

```
FUXI WLAN: W23 timeout scan COMPLETED (8750 ms); report, contain, no retry
FUXI WLAN: W28 crash scan COMPLETED timeout: mhistatus 0xff04 (state 0xff syserr 1) ee 0x2 bhi errcode 0 errdbg 0/0x152ee53/0x492; access-faults 0; read-only
  ... the completed watches (W28: tp 0 written 0 on both rings), W24, the W26/W27/W28 report lines, the W20 snapshot ...
FUXI WLAN: W28 crash contain before MHI reset: mhistatus 0xff04 (state 0xff syserr 1) ee 0x2 bhi errcode 0 errdbg 0/0x152ee53/0x492; access-faults 0; read-only
FUXI WLAN: abort step 28 MHI reset unconfirmed, bus master off yes, PERST asserted yes
  ... the recounts, `W26 arenas zeroed`, `abort buffers freed` ...
```

The EE and ERRCODE values in that block are the model's guesses; only the ERRDBG2/3 words are the phone's.

### Reading the next boot

| Pattern | Conclusion | Next |
|---|---|---|
| W24 frames > 0, all 38 channels | C2 PASS: with the monitor status rings the firmware survives and forwards beacons | C3 |
| all 38 channels, `written` > 0 on either ring, frames 0 | the crash is gone; status lands; nothing forwarded | r1c2e-r showed it alone: r1c2e; r1c2e: C3 management forwarding |
| all 38 channels, `written` 0 on both rings | the firmware survives but never used the status rings | owner review (RXDMA status needs the monitor mode ring config?) |
| `exhausted mon0` / `mon1` at a watch | RXDMA filled all 1023 status buffers (no reap, as L19 STA) | if the scan still ran on: fine; if it wedged right after: owner review (reaping the status rings is the next rung) |
| the scan stops early again, `W28 crash ... syserr 1` | the firmware still crashes with the status rings in place | owner review: compare `taken`/`written` at the last watch, the REO cmd/status rings and cookie conversion are left |
| `W28 crash ... syserr 0`, state 0x2 at the timeout | the scan stalled without an MHI SYS_ERR | owner review |
| `changed` > 0 or `foreign` > 0 | the device wrote a source ring entry | stop, owner review before any further kit |
| `SPILL` on any guard line | a device write where no address was given | stop, owner review |
| `W28 htt sring setup ... not sent` | the HTT endpoint or CE4 was not ready | owner review |
| any D4 line | the passive rule is broken | stop the radio lane |

### Deviations from L19, all deliberate

- No MSI on the two rings (L19 has them in its rx_mon_status ext group); the thresholds are L19's and shape nothing.
- The status buffers are the 2048-byte RX_MON_STATUS_BASE_BUF_SIZE budget at a 4 KiB stride with a guard, not an skb;
  L19 maps at most the same 2048 bytes.
- Cookies are the slot index per ring (L19's idr hands out 0, 1, 2, ... on a fresh ring: the same numbers).
- No reap and no replenish (as L19 in STA mode), so a ring is exhausted after 1023 PPDUs; L19 behaves the same.
- The crash line is new (read-only, six reads of the existing diagnostic).

### What the host tests cover, and what only the device can answer

Host-tested: every word, ring id, size, count, order, the allowlist, the latch, the free, the line widths, the
phone's crash pattern without the rings and survival with them, exhaustion, spills, changed and foreign entries, bad
and unreadable TPs, containment mid-scan with status still written during the MHI reset, unproven containment, an
allocation failure, a guard failure between the two SRING_SETUP, and a mutant per rule. Only the phone can say: whether
the missing monitor status rings are what crashes the firmware, whether RXDMA writes status into them in STA mode
without a status-ring selection, how fast it fills them, whether exhaustion matters to the firmware, and what the BHI
words mean (ERRDBG2 0x152ee53 looks like a firmware PC; nothing here decodes it).

### Proof that boots without 0x2000 are unchanged

Differentials against a5a04bf, each tree built from its own sources (scripts in this session's scratchpad, recorded in
the commit body): the W15, W11 and W16 harness logs byte for byte; the init/configure gate over boot-arg sets without
0x2000 (salami and Fuxi builds, cmdline and ADT /defaults) byte for byte; a5a04bf's model with all its C1, C2, C2b, C2c
and C2d scenarios equal byte for byte to the first part of the C2e model's log (every printed line, same
instrumentation in both), the C2e scenarios only adding lines after them.

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
two the shop took for C3, kept easy to change: decision 7, the `fuxiwlansalami=2` profile (one table,
`wl_salami_defaults`), and decision 8, a 30-minute service (`WL25_LIFETIME_MS`), then containment and Wi-Fi
off in iOS. With decision 4 (drop `fuxicpuload=10`, Kit (M2c) below) all three are recommended yes, pending
owner; the code and the recipes below follow them as documented, and each is one line to change.

C3 was written on C2 (39c80e5), rebased onto C2c (c18f99d, branch feat/c3-rebased) and then onto C2e
(0f48029: C2d d30c59d + a5a04bf, C2e, on C2c). Log prefixes stay distinct: W23/W24 are C2's lines,
`W23 chan-info` and `W23 stats` C2b's, W25 C3's, W26 C2c's, W27 C2d's, W28 C2e's.

### With C2b, C2c, C2d and C2e (the rebase onto 0f48029)

- C2c runs under qrtr 8 exactly as under qrtr 7. Its bits are `WL23_BITS` (0x100 only with 0x02; 0x08,
  0x20 and every other bit still refused; keep/stop22 still refused), read in W20 init with the same
  `W26 selected` line, which a qrtr 8 boot prints before its `W25 selected` line. The W26 setup sits in the
  path both stages share: at W20 the arenas, the idle list and the refill ring (`wl26_setup`), at W23
  right after VERSION_CONF the four SRING_SETUP, the setup watch and the chain masks before the channel
  list (`wl26_rings`), all before the W25 hook after the vdev lines. So a C3 boot can carry the w20
  combination of any RX rung (C2c woke the RX engine on the phone; C2d and C2e build on it), with the same
  register allowlists, the same containment (`wl26_contain` first, after the W25 summary) and the same
  free (only in wl5_abort's proven branch after bus master off and PERST).
- C2d (W27, 0x800 RXSEL, 0x1000 REORX) and C2e (W28, 0x2000 MONSTAT) run under qrtr 8 exactly as under
  qrtr 7, the same way: their bits are in `WL23_BITS` with the same rules (each needs 0x100), their
  `W27 selected` / `W28 selected` lines print in W20 init before `W25 selected`, and their setup is in the
  shared path before the W25 hook: the REO arena, the release and exception rings and the REO setup inside
  `wl26_setup` at W20, the eight REO destination rings after service ready at W21 (`wl27_dest`), the two
  monitor status SRING_SETUP and the two RX_RING_SELECTION_CFG inside `wl26_rings` at W23, before the
  channel list. Their watches and reports are called from `wl26_watch` and `wl26_report`, so they follow the
  first W25 scan only, with the W26 ones (`W27 reo watch` and `W28 mon watch` at setup, started, mid and
  completed; `W27 buffers returned ...` and the W28 sring, content and guard lines after the first scan's
  W25 line). The REO return path is polled in every `wl_ce_poll` pass (`wl27_poll`), so it runs in every
  idle and scanning poll of the whole service. The W28 crash line stays where C2e put it in containment
  (before the MHI reset), so it covers every end of the service; the qrtr 7 one at the COMPLETED timeout
  has no W25 twin, since a W25 scan timeout is not fatal (the service stops the scan and goes on).
- The C2c watches and the W26 report follow the first W25 scan only: `started` at the first poll pass
  that sees STARTED, `mid` at the first pass with foreign >= (n + 1) / 2 (n = that request's channels),
  `completed` as soon as that scan ended (whatever its end), before its W24 lines; then, after the first
  scan's W25 line, the C2b flush and the W26 content, guard, error-ring, link, counter, D4 and D5 lines
  (the D5 hint only when that scan completed with no frame). Why only the first: the four watch slots are
  one-shot (`wl_rx.w[].taken`) and are what the r1c2c readout compares, a W26 report is about ten lines,
  and 30 minutes of scans would bury the W25 lines; the containment recount (D3, `W26 rx guard after
  containment`) still covers the whole service. The CHAN_INFO notes behind D4 (`wlrx_note_ci`) keep
  counting in every scan, but are printed only in that first report.
- C2b's post-scan stats belong to the one-shot qrtr 7 run only: the W25 service never calls `wl23_stats`
  (no REQUEST_STATS in any scan, checked by the model and statically). C2b's `W23 chan-info` lines print in
  the first W25 scan only (`wl25_ci_off`, the cap of 128 still applies), like C2's foreign-channel lines.
- Refill exhaustion (open on feat/c3-rebased) is covered by C2d: with 0x1000 every buffer the REO/WBM
  path gives back is posted again to ring 160 in each poll pass, over the whole service. The model shows it
  (`s_svc_c2e`: three 38-channel scans, 13680 frames on 4095 buffers, all returned and posted again,
  frames to iOS in every scan). With C2c alone (no 0x1000) the buffers are still never given back, so a
  C3 kit without 0x1000 can still run dry; the phone's readouts say that kit wedges anyway (C2d, C2e).
- C2b's rule that no line passes 254 bytes is now checked for C3 too: the service end is two lines
  (`W25 service end: ...;` and `W25 service end rings: rx CE1 ... credits N`).

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
FUXI WLAN: W23 chan-info freq F flags START ...                            (C2b, the first scan sent only)
FUXI WLAN: W26 rx watch started ... / W26 idle watch started ...           (C2c with a W26 bit, the first scan only; mid likewise)
FUXI WLAN: W26 rx watch completed ... / W26 idle watch completed ...       (the first scan only, before its W24 lines)
FUXI WLAN: W24 bss aa:bb:cc:xx:xx:xx ch C freq F dBm S privacy P rsn R beacon frames N ssid "NAME"   (the first completed scan only)
FUXI WLAN: W24 scan summary frames N bss M ssids K
FUXI WLAN: W24 scan detail mgmt-rx N ...
FUXI WLAN: W25 scan K seq S id 0xa001: 38 channels passive dwell 150 ms, completed after T ms, reason 0, status 0; frames N bss M, to iOS N kept K; scan events E, of another id 0 (last 0)   (scans 1..24, every 100th, the first 24 failed ones)
FUXI WLAN: W26 rx content ... / rx guard ... / rxdma err ... / link content ... / counters ...   (C2c, after the first scan's W25 line only)
FUXI WLAN: W25 scan K seq S: not passive, refused, nothing sent, status 22      (or "no listed channel, refused")
FUXI WLAN: W25 complete: lifetime over (1800 s); stop/contain                  (or "the iOS side stopped")
FUXI WLAN: W25 service end: lifetime over after 1800 s; scans N (completed A, failed F, cancelled C, refused R), frames to iOS N kept K; stops confirmed late L, scan events of another id W, credit waits H;
FUXI WLAN: W25 service end rings: rx CE1 P/R CE2 P/R tx CE3 P/R credits N
FUXI WLAN: W26 rx contain: host writes off; arenas held until bus master off, PERST and SMMU undo   (with a W26 arena)
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

With C2b and C2c: `s_svc_c2c` runs the service behind a firmware that hands MGMT_RX on only with host
buffers and links (C2c's `rx_rule` 2): without a W26 bit no frame reaches iOS and no W26 line or C2b stats
line appears (C2b's 76 chan-info lines do); with w20 0x702 the W26 setup comes out as under qrtr 7 (link
and RX arenas, ring 120, 4095 refill buffers, the four SRING_SETUP after VERSION_CONF, the chain masks before
the channel list), the frames reach iOS in both of two 38-channel scans (24), the four watches and the W26
report appear once, after the first scan, the chan-info and foreign lines in the first scan only, no
REQUEST_STATS is sent, and containment prints the W25 summary, `W26 rx contain`, zeroes and frees both
arenas. Static checks: `WL23_BITS` holds the W26 bits, `wl26_rings` runs before the W25 hook, each watch and
`wl26_report` appears once in the service, behind `first`, and the service has no stats call. Every line
of every C3 scenario is held to 254 bytes (C2b's `model_line`). tools/test-fuxi-wlan-salami.py: qrtr 8
with each W26 kit value (and the refusals: 0x100, 0x301, 0x708, 0x720, 0xf02, keep), the same from ADT
/defaults alone and split, and the C3 kit route below for the 13 rung kit values.

With C2d and C2e: `s_svc_c2e` runs the service with w20 0x3b02 (r1c2e) behind the phone's firmware as the
C2e scenarios play it (MGMT_RX only with the whole RX path, the crash without the monitor status rings):
the W27 and W28 setup as under qrtr 7, three 38-channel scans of 120 C2d frames and 10 status PPDUs per
channel (13680 frames on 4095 refill buffers, every one returned and posted again), frames to iOS in all
three, four `W27 reo watch` and four `W28 mon watch` lines and one W27/W28 report (after the first scan),
no REQUEST_STATS, every arena zeroed and freed. The model's firmware plays the C2d wedge and the C2e crash in
W25 scans as in the qrtr 7 scan, and its device action list starts again once every action is done (many
scans). Static: `WL23_BITS` holds the W27 bits and 0x2000, the service itself calls nothing of W27/W28, and
`wl27_poll` is in `wl_ce_poll`. tools/test-fuxi-wlan-salami.py: qrtr 8 with 15106 and 6914 (W28 and W27
selected) and the refusals 0x2002, 0x1002, 0x210a, 0x4f02; qrtr 9 refused.

### Proof that qrtr 0..7 are unchanged

Before the rebase (C3 on C2): scripts in op11-tools/salami-port/wlan/r1/impl/c3-verify (verify.sh), run
again after the review fixes from impl/c3-fix: the W15 harness log (qrtr 5, keep 0 and 1) of 39c80e5 and
C3, byte for byte; the init/configure logs of 763 boot-arg sets on the salami and the Fuxi build, both
through 39c80e5's own gate harness; 39c80e5's C1+C2 host model (65 scenario logs) built against 39c80e5's
and C3's production code, byte for byte.

Each rebase is proven the same way, against the salami it lands on, by three differentials, each tree built
from its own sources (a worktree of the base next to the rebased tree; the driver's own cut functions, no
copied code):

1. Model, each tree its own: in each tree, `production()` of that tree's tools/test-fuxi-wlan-ce.py cuts
   that tree's fuxi_wlan.c, and that tree's tools/fuxi-wlan-ce-model.c is built against it and that tree's
   fuxi_wlan_ce.h with the test's own flags (ASan/UBSan, -Werror), with one hook added in a copy of the
   model: at every `reset()` the scenario log so far (`logbuf`) is written to a numbered file. Both models
   run all their scenarios; the base's logs must be the rebased model's first ones, byte for byte, and its
   summary lines the rebased model's first ones.
2. Model, crossed: the base's model (with the same hook) built against the rebased tree's cut and header;
   the W25 entry points it has no use for (`fuxi_wifi_radio_*`, `mach_absolute_time`,
   `absolutetime_to_nanoseconds`) are stubs that abort if called; its logs must equal the base model on the
   base code, byte for byte.
3. Gate: in each tree, the salami and the Fuxi init harness assembled exactly as that tree's
   tools/test-fuxi-wlan-salami.py `gate_suite` does (its `GATE_PRE`, the K-W1b knob source, the milestone
   5 region, `fuxi_wlan_init`, its `GATE_MAIN`), run over every combination of `fuxiwlan` absent/0/1/11/27/28,
   `fuxiwlansalami` absent/0/1/3, `fuxiwlanqrtr` absent or 0..7, the `fuxiwlanw20` values below and five
   extras (none, keep, stop22, `fuxiwlanw15=1 fuxiwlanw16=3`, `fuxiwlanclk=1 fuxiwlanmlo=0x2d`), on the
   cmdline, and every seventh set again split (the first knob on the cmdline, the rest in ADT /defaults as
   mkadt-salami.py encodes them). The whole output (every console line, RESULT, NAMES) is compared byte for
   byte, less the harness's `RADIO` line, which only the C3 harness has (it records what `fuxi_wlan_init`
   tells patch 0104; the calls print nothing).

Results:

- feat/c3-rebased against c18f99d (C2c): 151 model logs (1563024 bytes) equal, the rebased model adding 19
  C3 scenarios after them, 34 summary lines equal; crossed, the 151 equal; gate 66642 runs equal, with 27
  w20 values (absent, the C1 and C2 bits, 0x08, 0x20, -1, every C2c kit value, 0x100 without 0x02, 0x708,
  0x720, 0xf02, 0x1000).
- feat/c3-on-c2e against 0f48029 (C2e): 198 model logs (2224474 bytes) equal, the rebased model adding 20
  C3 scenarios after them, 50 summary lines equal; crossed, the 198 equal; gate 88856 runs equal, with the
  27 w20 values above plus the C2d and C2e kit values and edges (6402, 6914, 0x902, 15106, 8962, 0x2002,
  0x2000, 0x3f92, 0x4f02).

Left out by design, because C3 changes them: `fuxiwlanqrtr` 8, and -1 or 9 and beyond (the refusal line now
says `not 0..8`), and `fuxiwlansalami=2` (refused before C3, the C3 profile now). The model's device action
list now starts again once every queued action is done (for many W25 scans); the base logs above show that
no qrtr 0..7 scenario changed by it. The scripts (c3diff.py, cross.py) were run from the session's scratch
space and are not part of the tree; the method above is what they do.

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

### Kit (M2c: C3 with the RX rungs)

The same kernel, with the w20 combination of the RX rung whose readout makes frames arrive. On the phone
r1c2c (0x702) and r1c2d-l (0x1b02) both wedged; C2e's r1c2e (15106 = 0x3b02: L19's whole RX setup with the
monitor status rings) is the next readout, so `<n>` = 15106 until that readout says otherwise (r1c2e-r,
8962 = 0x2302, if it points there). 0x100 needs 0x02; 0x800, 0x1000 and 0x2000 need 0x100. The cmdline is
full, so w20 goes through ADT /defaults (`--wlan-defaults fuxiwlanw20=<n>`, a 4-byte int, 40 bytes of ADT),
as for the C2c/C2d/C2e kits; `wl_arg` logs `salami fuxiwlanw20 from ADT /defaults`. Budget (the M2 numbers above plus the
40-byte property; c3-kit/budget.sh has to confirm them before a build):

- On M2's KIT_BASE as is, `--wlan-node --wlan-defaults "fuxiwlansalami=2 fuxiwlanw20=<n>"` makes the ADT
  16388 bytes, 4 over the 16384 cap: mkadt-salami.py refuses it, and `fuxiwlansalami=2` (17 characters
  with its space) does not fit the cmdline's 15 free either.
- Recipe (owner decision 4: recommended yes, pending owner): drop `fuxicpuload=10` from KIT_BASE (15
  characters with its space) and move
  the profile to the cmdline. KIT_BASE `fuxitouch=1 fuxitclk=1000 fuxitouch_poll_us=2000
  fuxitouch_idle_us=4000`, cmdline extra `fuxicores=0x7c sweetfb_tegate=1 sweetfb_teguard=0 fuxiwlan=28
  fuxiwlanqrtr=8 fuxiwifiios=3 fuxiwlansalami=2` (528 characters, 594 of 607 with the loader's 66),
  KIT_ADT_EXTRA `--wlan-node --wlan-defaults fuxiwlanw20=<n>` (16348 bytes: one number property in place of
  the profile's), KIT_RO=0. The gate reads both the same way as the M2 shape (tools/test-fuxi-wlan-salami.py,
  C3 kit route, for the 13 C2c/C2d/C2e kit values: profile defaults used, `w20 0x... valid 1`,
  `W26 selected`, `W25 selected`).
- Without owner decision 4 there is no fit on this KIT_BASE; the rung kits' own route (r1c2's cmdline with
  w20 in /defaults) stays the way to read a rung itself at qrtr 7.

Expected, on top of M2's lines: `W26 selected`, `W27 selected` and `W28 selected` after `W20 selected qrtr=8
... w20 0x3b02 valid 1`, the W26/W27/W28 setup lines as in r1c2e before `W25 service ready`, the watches and
the W26/W27/W28 reports around the first scan only, then the W25 lines; containment with the W25 summary,
`W26 rx contain`, the W28 crash line before the MHI reset, and `W26 arenas zeroed` before `abort buffers
freed`. Run it as M2 (`SESSION_SECONDS=3000`).

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

C2c in the same model: the RX and link arenas with the device's permission per region (a device read of
an unposted half or slot, a write to a host-only word, or any access outside the arenas fails the test,
except the injected spills; ring 120 is device read/write; nothing after bus master off), a WBM register
file holding exactly the nine pre-read and eight write addresses (the pre-reads in order before any write,
L19's write order and values, BASE_LSB then MISC read back, one HP publish of 65534 by the route the
shadow mode says, right after the write barrier, with all 32767 entries in place; R2 HP/TP reads at the
watches; nothing after `wl26_contain`), shadow 19 routed to it through `wl15_shadow_v3`, and a fake
firmware that checks every SRING_SETUP word against its own table, after VERSION_CONF reached the host and
before the channel list, takes refill entries (slot starts, RBM 6, the slot as cookie, each once), posts
them to the MAC rings (with RBM, cookie or address changes by scenario), lets RXDMA receive frames into
them (with 0x200 also taking link descriptors and maybe returning them to ring 120), pushes RXDMA error
entries, checks both PDEV_SET_PARAM (cmd 0x4003, TLV 0x0052000c, {pdev, 1, tx} then {pdev, 2, rx}, after
the SRING_SETUPs and before SCAN_CHAN_LIST) and forwards MGMT_RX per a rule (needs links, buffers and
links, buffers, the chain masks, never). The host's wrp[0] publish is checked when it happens (barrier
right before it, every entry, sentinel and zero in place), the host-owned words again before containment,
and every arena byte at its free (zero, only after bus master off, PERST and the SMMU undo). Scenarios:
0x702 with shadow 19 and with fuxiwlanw15=1; 0x200 alone with each rule (no RX arena, no ring message,
idle watches, link content, HTT types counted without 0x02); the four pre-read refusals with 0x200 and
with 0x302 (nine reads, no WBM write, 0x200 dropped, the RX part still runs; D1's ambiguous case);
0x302 with the needs-links rule; 0x402 with the needs-chain-masks rule; masks 0 (nothing sent, the WMI
command stream as without 0x400); 0x480 (the EXT pdev); 1x1 PHY masks with 2 rf chains; tx mask 0x3 with
rx mask 0x1 (each parameter its own mask); refill entries taken 50 ms after the 4th SRING_SETUP (the
setup watch waits for them); rings ignored; foreign MAC entries (64 bytes into a slot, slot 4095's start,
address bits 39:32 set); spills into a slot's upper half, slot 4095, both ends of the tail and the gaps
(SPILL, D3's second line), the gaps alone; a write at offset 32 only; kept 952 with RBM 4 and other
cookies; mac0 drained before the mid watch; the error ring full (1023 entries, with 0x10: a second
inconclusive; entries carry a byte count, fragment and sequence number that must not reach the log); D2
refill-empty against kept rising and against a never-used MAC ring, and refill exhausted with mac0
drained; rdp[186] past the ring, refill tp backwards and past the HP, R2 reads of ~0u; the D4 line (with
the base tx-frame alone, and none without a W26 bit); an RX arena allocation failure and a link arena map
failure (no WBM access, no ring message); BASE_LSB and MISC readback mismatches (no HP); a fault after the
3rd WBM write; 16 CE4 sends in flight; HTT major 2; a guard failure after the 2nd SRING_SETUP; a guard
failure in the stats exchange and no COMPLETED (the W26, D4 and D5 lines before containment); every
received frame, link descriptor and error entry carries a marker no log line may show; unproven containment (arenas kept, not
zeroed), proven with RXDMA writing until bus master off (zeroed after it, freed), and the latch (no fill,
publish or WBM access after `wl26_contain`); every W26 line at its widest; qrtr 7 boots without a W26
bit (no W26 line, arena, WBM access or ring message). Pure checks: the allowlist equals the model's set
and meets no CE, TCL or REO register; the D1 readings; the four SRING_SETUP words, the ring 120 list, the
first and last entries; the watch arithmetic with D2; the D5 rows; the latch. Static: the gate, W26's
place in L19's order, one window implementation for the two allowlisted accessors, one shadow-19 publish,
no WBM write before the pre-reads or after a refusal, no store into the arenas from the glue and the H
fills only from W20 (refused after the latch), SRING_SETUP the only HTT message and RESPONSE_REQUIRED never,
PDEV_SET_PARAM ids 1 then 2 only, D2/D4/D5, `wl26_contain` first, the free only in the proven branch after
the D3 recount and the zeroing, no arena word as an argument of any W26 printf, every W26 line within 254
bytes. L19: the error entry's frame-field masks, the ring ids (enum, PMAC stride,
setups), rdp/wrp, the RX alloc and HTT announcement order, sizes (struct replicas compiled with the host
compiler), the idle list on the ring path, HTT enums, fields and encoder rules, thresholds, remote pointer
words, RBMs, entry fields, the WCN7850 WBM registers and L19's ring 120 write order and values, shadow 19,
PDEV_SET_PARAM ids and order, then every word of the model's dump. test-fuxi-wlan-salami.py covers the
gate for the W26 bits (salami and Fuxi builds, cmdline and ADT /defaults, the kit route with r1c2's
cmdline and a decimal w20 in /defaults). Mutants (c2c-impl/mutants.py: the plan's 2.13 list, D1..D5, the
gate, the allowlist and the HTT handler) are all caught.

C2d in the same model: the REO arena with the device's permission per region (the HP words of the ten rings and the
rings; anything else, or any access with bus mastering off, fails the test except the injected spills), the REO
register conversation written down again from L19 (the 38 pre-reads in order, the release and exception rings and
the REO setup before any CE RX buffer is posted, the eight destination rings after service ready and before INIT,
each ring's nine writes with L19's values, the readbacks, nothing after a refusal or a latched fault or
`wl26_contain`), TP publishes that must follow the write barrier, take the route the shadow mode says and never pass
the device HP, RX_RING_SELECTION_CFG checked word by word after the four SRING_SETUPs and before the channel list, and
a REO/WBM device that, with the whole path live, writes each received frame (the 400-byte descriptor and 128 bytes of
MSDU, inside the buffer's 2048 bytes) and returns the buffer on a destination, exception (MSDU or link descriptor)
or release ring; the firmware refills its MAC rings from the host's refill ring and each host republish is checked
(write barrier, only returned slots, RBM 6 and the slot as cookie, posted half zero but the sentinel, not past the
firmware's TP). Without the path (C.wedge) it plays the phone's r1c2c-all readout. Scenarios: 0x1902 with shadow and
with fuxiwlanw15=1, 0x1b02 (link descriptors held, their MSDUs returned): 4560 frames through 4095 buffers; the wedge
for 0x702, 0x902, 0x1102 and 0x1302 (15 channels, COMPLETED timeout 8750 ms, mac0 2047 / mac1 2016 valid, kept 32,
nothing written, no MGMT_RX); a 4000-frame burst (929 without a buffer, 3071 returned and re-posted at once, later
channels land again); a foreign address, a buffer returned twice and a REO HP off its entries (stop, contain); release
entries with RBM SW1 (counted, skipped as L19) and destination entries with RBM 4 (counted, returned by address); D5
with frames returned and no forward; spills in the REO arena's gaps and tail; frames pushed during the containment
drain (never reaped or re-posted); unproven containment (REO arena kept); the four pre-read refusals; a readback
mismatch; a destination ring based at W21; a fault latched mid-setup; the REO arena allocation failure; every W27 line
at its widest. Pure checks: the selection words, every ring and REO setup write equal to the model's conversation, the
allowlist equal to its address set and apart from the WBM and CE sets, the pre-read readings, the recycle and
replenish rules, the guard counts. Static (`static_c2d`): the gate, W27's place in L19's order, the one REO accessor
through `wl_win`, the shadow TP path, no REO write before the pre-reads or after a refusal, returns refused after the
latch and zeroed before their sentinel, one barriered republish, the free, no arena word in any W27 print, every W27
line within 254 bytes. L19 (`l19_c2d`): the ring selection enum, masks, struct, encoder and WCN7850 filter, rx_desc.h
compiled for the 400/272/144/0 offsets, ring ids, sizes, entry structs (compiled), entry fields and RBMs, the WCN7850
REO and WBM registers, dst_hw_init's order, thresholds, dp_alloc and core placement, reo_hw_setup's order and values
with the hash map; the model's dump word by word. Mutants (`C2D_MUTANTS`, 34 rules broken once each in the production
code: selection words, write order, thresholds, the REO setup values, the refusals, the slot checks, the sentinel and
zeroing, barriers, bounds, RBM checks, link returns, the shadow index, the W21 re-read, the latch, the gate, the
zeroing before free, the TP publish) are all caught. test-fuxi-wlan-salami.py covers the gate for the W27 bits
(salami and Fuxi builds, cmdline and ADT /defaults, the kit route with r1c2's cmdline and 6402 / 6914 in /defaults).

C2e in the same model: the monitor status arena with the device's permission per region (ring entries read, the
posted half of a posted slot written, the TP words rdp[185]/[193] written and the HP words wrp[25]/[33] read only with
0x2000; anything else fails the test except the injected spills), both HP publishes checked when they happen (write
barrier right before, ring 185 then 193, after the refill publish and before any SRING_SETUP, every entry, sentinel
and zero word of the arena in place), the 5th and 6th SRING_SETUP word by word against the model's own table (and the
selection, chain masks and channel list only after all six), a fake RXDMA that per PPDU takes the status buffer at its
TP (a posted slot of that ring, RBM 6, its cookie) and writes one status TLV (frame data the log must never show),
nothing given back, and the six MHI/BHI registers (any other BAR0 read, or one without 0x2000, fails the test). In the
C2e scenarios only (`C.mon_crash`) the firmware plays the phone: RX live without the monitor status rings stops the
scan after 15 channels (3 with ring selection), SYS_ERR with the phone's ERRDBG words, the MHI reset never confirmed.
Scenarios (`c2e`): pure checks (the allowlist exactly six reads and none at a bad BHIOFF, the layout and both
SRING_SETUP, entries 0/1022/1023, the TP watch, written/guard/changed counts, no write after the latch); 0x3b02 (380
status TLVs, frames landed and returned as C2d, recounted, zeroed, freed); 0x2302 (all 38 channels, the hint names
0x3b02); the phone's crash for 0x302 (15 channels) and 0x1b02 (3), with no W28 line and no MHI/BHI read without 0x2000,
and 0x3b02 with the SRING_SETUP dropped (crash lines at the timeout and before the MHI reset, still freed); full rings
(mon1, then both: exhausted, 227 PPDUs without a buffer, the hint); spills in a slot's upper half, an unposted slot,
the tail and a head gap; a foreign entry and one with another cookie; bad and unreadable TPs; containment mid-scan
with a status written during the MHI reset; unproven containment; the arena allocation failure; a guard failure
after the 5th SRING_SETUP. Static (`static_c2e`): the gate, W28's place in L19's order, the one read path through the
allowlist (the W11/W15 diagnostic's six offsets) and no register write, the crash line in exactly its two places,
SRING_SETUP the only message, arena writes only at W20 and refused after the latch, never reaped, the free, no arena
word in any W28 print, every W28 line within 254 bytes. L19 (`l19_c2e`): ring ids, the hal.c config, sizes, the
entry struct, the alloc, fill and announcement order, the fill count, RBM and cookie, the buffer budget and alignment,
the HTT ring id and type, thresholds, and the MSI, reap and status-selection facts the deviations rest on; the model's
dump word by word. Mutants (`C2E_MUTANTS`, 31 rules broken once each) are all caught. test-fuxi-wlan-salami.py covers
the gate for 0x2000 (salami and Fuxi builds, cmdline and ADT /defaults, the kit route with 15106 / 8962 in /defaults).

What only the device can answer: whether the firmware fetches from the rings at all with zero interrupt
thresholds, which pipe carries HTC READY (both are polled), the credit numbers, the real pdev id, whether
`num_mem_reqs` is 0, and whether WMI READY comes back. For C2: whether the firmware accepts the channel
list and the vdev without a regulatory domain or a vdev start, which pdev id START_SCAN wants (0 as L19,
or the caps' one), whether beacons arrive over WMI MGMT_RX without host RX data rings (the W24b rung), the
real noise floor and the chain masks. Not written yet: the type-1 segment compare helper of
overnight_plan.md 4.11 (the layout of the firmware's copy in that segment is not known).
