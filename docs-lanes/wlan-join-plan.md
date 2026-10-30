# Join plan, part 1: regulatory and the receive data path on WCN7850 (study, no code)

Status: study only. No code in this change and nothing here has run on the phone. Every
`file:line` points into L19, unless it names a file in this repo.

Scope. This part covers four things: the regulatory setup (country IL), the data-path rings and
their DMA budget with the containment order, the RX data path on top of C2c, and an iOS `en0` radio
backend that receives only. Three steps a connection also needs are left out on purpose: the
station's frames on the air (authentication, association), peer setup, and the TX data path. They
belong to a separate study, written only after the owner approves TX on the phone. So this part
does not reach "associated, DHCP over the radio". It takes the lane to the point where the radio
delivers data-path entries into host memory, and the host can parse them, return them and hand
frames to `en0`. The [last section](#7-what-the-next-part-needs) lists what the transmit study has
to add.

Sources and conventions:

- L19 = Linux ath12k v6.19, commit 05f7e89ab (the same tree `docs-lanes/wlan-w20.md` uses),
  `drivers/net/wireless/ath/ath12k`, BSD-3-Clause-Clear. This study fetched it from the GitHub
  mirror of the mainline tree (tag v6.19, commit 05f7e89ab9731565d8a62e3b5d1ec206485eeb0b), because
  this session's network policy refused git.kernel.org; the commit id is the same. Only ath12k
  files were read. No Linux MHI, QRTR or QMI core file and no other GPL file was read, and
  UniXNU-sheng was not used.
- IEEE 802.11-2020 for frame formats; RFC 1042 for the LLC/SNAP header.
- `[V]`: read in L19 at the cited line(s). For a value computed from L19 text (an enum resolved
  by counting, a struct size or offset), the method is stated. `[I]`: inferred, not read anywhere
  as such. Everything said about C2c is `[I]`, because C2c's code is not on `salami` (see
  [C2c](#22-what-this-part-assumes-about-c2c)).
- Ids written as hex are WMI/HTT wire values. Ring ids are L19's `enum hal_srng_ring_id`
  (hal.h:386-526), resolved by counting the enum [V].

## Contents

1. [What L19 does, in its order](#1-what-l19-does-in-its-order)
2. [Rungs](#2-rungs)
3. [Rings, buffers, DMA budget, containment](#3-rings-buffers-dma-budget-containment)
4. [The iOS side: a radio data backend for en0 (`fuxiwifiios=5`)](#4-the-ios-side-a-radio-data-backend-for-en0-fuxiwifiios5)
5. [Risks and open questions](#5-risks-and-open-questions)
6. [Host-model test plan](#6-host-model-test-plan)
7. [What the next part needs](#7-what-the-next-part-needs)

## 1. What L19 does, in its order

### 1.0 The runtime order, from firmware ready to the first data-path entry

| # | L19 step | Where | Note |
|---|---|---|---|
| 1 | QMI WLAN mode, CE pipe init | core.c:1279, 1285 | done in our W15/W20 [V] |
| 2 | `ath12k_dp_alloc`: WBM idle link ring, link descriptor banks, cookie-conversion tables, `ath12k_dp_srng_common_setup` (UMAC rings + REO hardware setup), RX rings and their buffers | core.c:1291; dp.c:1755, 1763, 1770, 1782, 1813 | before HTC [V] |
| 3 | HTC READY, HTT connect, WMI connect | core.c:892, 898, 904 | done (C1) [V] |
| 4 | Cookie-conversion registers, REO destination rings | core.c:923, 925 | [V] |
| 5 | WMI INIT, READY, HW mode DBS, HTT version request | core.c:933, 939, 948, 955 | done (C1/C2) [V] |
| 6 | `ath12k_mac_register`: waits up to 3 s for the boot-time regulatory event, applies it, sends SET_CURRENT_COUNTRY when a country is known | core.c:1127; mac.c:14493-14511; reg.c:282-289 | [V] |
| 7 | `ath12k_core_pdev_create` -> `ath12k_dp_pdev_alloc` -> `ath12k_dp_rx_htt_setup`: one HTT SRING_SETUP per LMAC RX ring, then RX_RING_SELECTION_CFG | core.c:1143; dp.c:1057; dp_rx.c:4609-4672 | [V] |
| 8 | Interrupts on; RX rings serviced from NAPI | core.c:1150 | we poll instead (C1 rule) [I] |

Steps 2 and 7 hold the data path. Our C1/C2 do steps 1, 3, the CE part of 2 and 5. They don't
do the DP part of 2, step 4, step 6 or step 7. C2's MGMT_RX silence fits the hypothesis that
firmware wants step 7 before it receives anything; that is C2c's question [I].

### 1.1 Regulatory, country IL

**What L19 sends, and when [V].**

- WCN7850 has `current_cc_support = true` (hw.c:1489), so L19 uses WMI_SET_CURRENT_COUNTRY,
  not SET_INIT_COUNTRY (reg.c:107-120; mac.c:14499-14511).
- L19 sends it on two occasions only: at mac registration when `ab->new_alpha2` is set
  (mac.c:14499), or when a user hint arrives (reg.c:48-130). `new_alpha2` comes from the ACPI
  or SMBIOS country (core.c:744-751) or from the firmware's 11d event (wmi.c:6779). A boot with
  neither sends no country command: the firmware's boot-time REG_CHAN_LIST_CC_EXT, which C1
  already receives unprompted (`wlan-w20.md`, "W22 wmi event 0x3a003"), becomes the default
  regulatory domain (reg.c:952-961). Setting IL is therefore the user-hint path.
- After any country command L19 stops the 11d scan (reg.c:125-127). We never start one (C2).

**WMI_SET_CURRENT_COUNTRY** [V]: command 0x3a001 (`WMI_TLV_CMD(WMI_GRP_REGULATORY 0x3a)`,
wmi.h:299, 309, 688), TLV tag 0x25f (wmi.h:1821, counted), struct wmi.h:4236-4240, encoder
wmi.c:3433-3465. 12 bytes:

| Word | Value | Source |
|---|---|---|
| 0 | TLV header 0x025f0008 (tag << 16 \| length without the header) | wmi.c:3446-3448, 204-207 |
| 1 | pdev_id = 0 (single-pdev chip, `wlan-w20.md` "pdev") | wmi.c:3450 |
| 2 | new_alpha2: bytes 'I' 'L' 0 0, so the u32 is 0x00004c49 (memcpy of 2 bytes into a zeroed buffer) | wmi.c:3451; wmi.c:790-791 |

No reply exists for the command itself. The answer is the event below [V] (reg.c:105, 117-118;
mac.c:14505).

**WMI_REG_CHAN_LIST_CC_EXT_EVENT** (0x3a003, wmi.h:914; handler wmi.c:9820-9822 ->
6807-6893) [V]:

- First TLV: tag 0x3aa (wmi.h:1990, counted), struct wmi.h:4312-4355, 78 words = 312 bytes
  (word offsets counted from the struct). Words used here: status_code 0, phy_id 1, alpha2 2
  (two bytes), country_id 4, domain_code 5, dfs_region 6, min/max bw 2 GHz 8/9, min/max bw 5 GHz
  10/11, num_2g_reg_rules 12, num_5g_reg_rules 13. The 6 GHz counts sit at words 63 to 77 and
  are ignored here (no 6 GHz, C2 rule).
- Rules: L19 reads them right after the fixed TLV plus one 4-byte array header
  (wmi.c:5929-5931): 2 GHz rules first, then 5 GHz, then the 6 GHz sets (wmi.c:5933-5998). Each
  rule is 5 words (wmi.h:4357-4363):
  - word 0: TLV header.
  - word 1, freq_info: start MHz 15:0, end MHz 31:16.
  - word 2, bw_pwr_info: max bw 15:0, reg power 23:16 (dBm), antenna gain 31:24.
  - word 3, flag_info: 15:0.
  - word 4, psd_power_info: PSD flag bit 2, EIRP 31:16.

  Masks wmi.h:3169-3176; decode wmi.c:5693-5718.
- Status codes: 0 PASS, 1 CURRENT_ALPHA2_NOT_FOUND, 2 INIT_ALPHA2_NOT_FOUND, 3 CHANGE_NOT_ALLOWED,
  4 NO_MEMORY, 5 FAIL (wmi.h:4302-4307; wmi.c:5851-5870). L19 rejects more than MAX_REG_RULES 2
  or 5 GHz rules (wmi.c:5793-5799) and an event with no rules (wmi.c:5836-5840).
- A single-pdev chip drops an event whose phy_id is not 0 but is below num_rxdma_per_pdev
  (reg.c:905-916).

**From rules to the channel list [V].**

- L19 builds a cfg80211 regdomain, applies it, then sends SCAN_CHAN_LIST
  (`ath12k_reg_update_chan_list`, reg.c:133-250). It sends it only after any running scan or 11d
  scan has ended (reg.c:805-850).
- Per channel: disabled channels are skipped (reg.c:190-191). NO_IR or DFS makes the channel
  passive (reg.c:207-209). Max power, max reg power and antenna max are the regdomain values
  doubled, in half dB (reg.c:213-215).
- C2 already encodes SCAN_CHAN_LIST word by word (`wlan-w20.md`, wire facts). The only change
  is that the list and power now come from the event instead of the fixed 38 channels.

**What must precede any TX.**

- L19 sends nothing that transmits before mac registration has waited for the regulatory event
  (step 6) and SCAN_CHAN_LIST carries the per-channel power. The wait is 3 s and continues on
  timeout with the boot-time data (reg.c:282-289) [V].
- For us: the rung that first transmits (a later part) must require all of the following, else
  refuse to start [I, plan rule]:
  - a CC_EXT with status 0 for alpha2 IL received after our SET_CURRENT_COUNTRY;
  - its rules parsed with no overrun;
  - a SCAN_CHAN_LIST built from them sent before any vdev start.
- Channels the rules flag NO_IR or DFS stay passive, and nothing is ever initiated on them [V
  for the passive mapping, I for the "never initiate" rule].

**Which channels IL gets** is whatever the firmware's board regulatory database returns, not
something this study can know. J1 logs the rules, and the C2 channel list becomes the
intersection of the 38 with the rules [I].

### 1.2 Data-path rings: what L19 allocates and how each is programmed

WCN7850 hardware parameters [V]:

| Parameter | Value | Where |
|---|---|---|
| single_pdev_only | true | hw.c:1416 |
| rxdma1_enable | false | hw.c:1433 |
| num_rxdma_per_pdev | 2 | hw.c:1434 |
| num_rxdma_dst_ring | 1 | hw.c:1435 |
| rx_mac_buf_ring | true | hw.c:1436 |
| vdev_start_delay | true | hw.c:1437 |
| supports_shadow_regs | true | hw.c:1451 |
| max_tx_ring | 3 | hw.c:1454 |
| rx_buf_rbm | SW1_BM = 6 (enum counted, hal.h:809-816) | hw.c:1291 |

Two kinds of ring [V]:

- **UMAC** rings (REO, WBM, TCL): the host programs them through registers, like the CE rings.
  The base, size, id and MISC registers sit at `reg_start[0] + n * reg_size[0]`; HP/TP sit in the
  R2 group (hal.c:1327-1426 for WCN7850). Host-side pointer updates go through the shadow
  registers, whose order is the W15 WLAN_CFG table (`xnu/fuxi-wlan/fuxi_wlan.c`
  `wl15_shadow_v3`, built from hal.c:2509-2530). Shadow index i is register BAR0 + 0x8fc + 4i
  (`wlan-w20.md`).
- **LMAC** rings (DMAC/PMAC types, ids >= 160): the host only allocates them. Firmware
  programs the hardware from one HTT SRING_SETUP message per ring (dp_tx.c:1159-1269). Their
  pointers live in host memory, in two arrays:
  - **rdp** (one u32 per ring id, the same array as C1's);
  - **wrp** (one u32 per DMAC/PMAC ring, 4 x (23 + 24) = 188 bytes; hal.c:1592-1600).

  For a source ring, head = wrp + 4 x (id - 160) and tail = rdp + 4 x id. For a destination
  ring, head = rdp + 4 x id and tail = wrp + 4 x (id - 160) (hal.c:2392-2396, 2410, 2426-2430,
  1881-1912).

Ring ids for the LMAC rings (`ath12k_hal_srng_get_ring_id`, hal.c:1818-1838) [V]:

- RXDMA_BUF is a PMAC type on WCN7850 (hal.c:1421-1423), and HAL_SRNG_RINGS_PER_PMAC is
  192 - 184 = 8 (hal.h:519-524). That gives refill = ring 0 of mac 0 = 160, and mac buffer ring
  i = 160 + 1 + 8i = 161 and 169.
- RXDMA_DST: 186.
- RXDMA_MONITOR_STATUS: 185 and 193.

### 1.3 HTT messages for the RX rings (on the HTT endpoint, CE4 up)

**HTT_H2T SRING_SETUP**, message type 0xb (dp.h:472), struct dp.h:662-676 (13 words, 52 bytes),
fields dp.h:642-660, encoder dp_tx.c:1159-1269 [V]:

| Word | Field | Bits |
|---|---|---|
| 0 | info0 | msg type 7:0 = 0xb; pdev 15:8 (mac_id + 1 for SW_TO_HW and HW_TO_SW rings, else mac_id); ring id 23:16 (`enum htt_srng_ring_id`, dp.h:499-513); ring type 31:24 (0 HW_TO_SW, 1 SW_TO_HW, 2 SW_TO_SW, dp.h:493-497) |
| 1, 2 | ring base | low 32 bits, then the high bits (`>> 32`) |
| 3 | info1 | ring size in words 15:0 (entries x entry words); entry size in words 23:16; LOOP_CNT_DIS bit 25 on SW_TO_HW rings only; the swap bits 27/28/29 are 0 (no swap flags set) |
| 4, 5 | head pointer address | the rdp or wrp word above |
| 6, 7 | tail pointer address | the rdp or wrp word above |
| 8, 9, 10 | MSI address and data | L19 fills these from its MSI setup |
| 11 | intr_info | batch threshold (entries x entry words) 14:0; timer threshold (us >> 3) 31:16 |
| 12 | info2 | low threshold 15:0 only when LOW_THRESH_INTR_EN is set; RESPONSE_REQUIRED bit 19 is never set, so no reply comes (dp_tx.c:1248-1252) |

The ring mapping (`ath12k_dp_tx_get_ring_id_type`, dp_tx.c:1101-1157) [V], and the info0/info1
words it gives. The words are computed by applying the masks above to L19's ring sizes (dp.h:183-193):

| Ring | id | HTT ring id / type | pdev | info0 | Entry words | info1 with L19 sizes | Head / tail |
|---|---|---|---|---|---|---|---|
| refill (host fills) | 160 | HOST1_TO_FW_RXBUF 5 / SW_TO_SW 2 | 0 | 0x0205000b | 2 (`hal_wbm_buffer_ring`, hal_desc.h:2117-2119) | 4096 entries: 0x00022000 | wrp+0x0 / rdp+0x280 |
| mac buffer 0 (firmware fills) | 161 | RXDMA_HOST_BUF 0 / SW_TO_HW 1 | 1 | 0x0100010b | 2 | 2048 entries: 0x02021000 | wrp+0x4 / rdp+0x284 |
| mac buffer 1 | 169 | RXDMA_HOST_BUF 0 / SW_TO_HW 1 | 2 | 0x0100020b | 2 | 0x02021000 | wrp+0x24 / rdp+0x2a4 |
| RXDMA error destination | 186 | NON_MONITOR_DEST 7 / HW_TO_SW 0 | 1 | 0x0007010b | 8 (`hal_reo_entrance_ring`, hal_desc.h:1051-1059; hal.c:1425-1426) | 1024 entries: 0x00082000 | rdp+0x2e8 / wrp+0x68 |
| monitor status i | 185, 193 | MONITOR_STATUS 1 / SW_TO_HW 1 | i+1 | 0x0101010b, 0x0101020b | 2 | 1024 entries: 0x02020800 | wrp+0x64/rdp+0x2e4, wrp+0x84/rdp+0x304 |

Order in L19 [V]: refill, the two mac buffer rings, the error destination ring, the two monitor
status rings (dp_rx.c:4609-4668), then RX_RING_SELECTION_CFG (dp_rx.c:4672).

**HTT_H2T RX_RING_SELECTION_CFG**, message type 0xc (dp.h:473), struct dp.h:1202-1219 (17 words,
68 bytes), fields dp.h:840-873, encoder dp_tx.c:1363-1505. WCN7850 sends it once for each mac
buffer ring (`ath12k_dp_rxdma_ring_sel_config_wcn7850`, dp_rx.c:4567-4605) [V]:

| Word | Value for mac buffer ring i | Source |
|---|---|---|
| 0 info0 | 0xc \| (i+1) << 8 \| ring id 0 << 16 \| OFFSET_VALID bit 26 \| EN_RXMON bit 28 (set because `rxmon_disable` is 0) = 0x1400010c / 0x1400020c | dp_tx.c:1392-1415 |
| 1 info1 | buffer size field = 2048 (L19 passes DP_RXDMA_REFILL_RING_SIZE there, dp_rx.c:4600-4602); conf lengths 0 | dp_tx.c:1417-1424 |
| 2, 3 | packet filter flags 0, 1 = 0 | dp_tx.c:1425-1426 |
| 4 | flags2 = BAR, bit 24 = 0x01000000 | dp.h:971; dp_rx.c:4579 |
| 5 | flags3 = MCAST bit 18 \| UCAST bit 21 \| NULL_DATA bit 24 = 0x01240000 | dp.h:1001-1007; dp_rx.c:4580-4582 |
| 6 | TLV filter = MPDU_START \| RX_PACKET \| MSDU_END = 0x0000000d | dp.h:876-884, 1194-1197 |
| 7 | rx_packet_offset 400 15:0 \| header offset 272 31:16 = 0x01100190 | dp_tx.c:1451-1457; layout below |
| 8 | mpdu_end 0 15:0 \| mpdu_start 144 31:16 = 0x00900000 | dp_tx.c:1459-1465 |
| 9 | msdu_end 0 15:0 \| msdu_start 0 31:16 = 0 | dp_tx.c:1467-1473 |
| 10..16 | attention 0, info2 0, reserved 0, 0, word masks 0 (WCN7850 sets none), info3 0 | dp_tx.c:1475-1497 |

Management frames aren't in this filter; they keep coming through WMI MGMT_RX (C2's parser) [V
for the filter, I that the firmware path is unchanged].

### 1.4 UMAC setup L19 does around the REO rings [V]

- **REO hardware** (`ath12k_hal_reo_hw_setup`, hal_rx.c:896-930, called at dp.c:587; REO block
  0x00a38000, hal.h:49):
  - GEN_ENABLE (+0x0): read, then set aging list enable bit 2 and aging flush enable bit 3.
  - MISC_CTRL (+0xb7c on WCN7850, hw.c:1239): fragment and BAR destination fields 20:17 and
    24:21 set to ring REO2SW0 = 0 (hal.h:325-326).
  - Aging thresholds IX_0..IX_3 (+0xb08..+0xb14, hw.c:1254-1257): 100000, 100000, 100000 and
    40000 us (hal.h:559-560).
  - DEST_RING_CTRL_IX_2 and IX_3 (+0xc, +0x10, hal.h:144-145): the hash map 0x43214321, meaning
    SW1..SW4 twice, built at dp.c:578-585 from HAL_HASH_ROUTING_RING_SW1..SW4 = 1..4
    (hal.h:915-918).
- **REO destination rings**: `ath12k_dp_rx_pdev_reo_setup` (dp_rx.c:539-560) sets up all 8, as
  REO2SW1..REO2SW8 = ids 1..8, each 2048 entries x 8 words (`hal_reo_dest_ring`,
  hal_desc.h:862-869).
  - On WCN7850 ring n's R0 block is at 0x00a38000 + 0x4e4 + 0x78n: base +0, base MSB +4, id +8,
    MISC +0x10, HP address +0x14/+0x18, producer interrupt +0x24 (hw.c:1238-1251).
  - R2: HP 0x00a3b048 + 8n, TP +4 (hal.h:172-176).
  - Shadow for the TP: W15 table index n (0..7).
- **REO exception ring** (REO2SW0, id 0): 128 entries, shadow index 8 (dp.c:547-548).
- **WBM idle link ring** (id 120) and link descriptor banks: needed whenever REO builds MSDU and
  MPDU link lists (dp.c:760-892).
  - L19 sizes it at 18560 descriptors rounded up to 32768 (dp.c:766-784; dp.h:159-165;
    hal.h:16-22): 32768 x 128 bytes = 4 MiB of banks (more than 2 MiB, so two banks + 1,
    dp.c:809-825) plus 32768 x 8 bytes of ring.
  - Each idle entry is {address 39:0, RBM 11:8 = idle link RBM, cookie 31:12} (dp.c:859-875).
  - Shadow index 19 (HP).
- **WBM release ring for RX errors**: WBM2SW3 (id 131, HAL_WBM2SW_REL_ERR_RING_NUM = 3,
  hal.h:378), 1024 x 8 words (dp.c:539-541). Shadow index 24.
- **Cookie conversion**: `ath12k_dp_cc_config` (dp.c:1330-1380) points REO SW_COOKIE_CFG0/1
  (+0x50/+0x54, hw.c:1240-1241) and the WBM equivalents at a CMEM base that comes from QMI
  `dev_mem` (dp.c:1332). The CMEM page tables are written by `ath12k_dp_cmem_init`
  (dp.c:1533-1580). With conversion on, the hardware writes the host virtual address of the
  buffer descriptor into REO entry words 5 and 6 (dp_rx.c:2976-2978).

### 1.5 The RX data path: entry, descriptor, buffer return

**Posting a buffer** (`ath12k_dp_rx_bufs_replenish`, dp_rx.c:308-408) [V]:

- Buffers are 2048 + 128 bytes, aligned to 128 (dp_rx.c:353-363; dp.h:206-208) and mapped
  from-device.
- The refill entry is two words (hal_rx.c:268-279): word 0 = address 31:0; word 1 = address
  39:32 in 7:0, RBM 6 (SW1) in 11:8, cookie in 31:12 (hal_desc.h:11-15).
- The ring holds bufs_max = entries (dp_rx.c:470-479). L19 refills when more than 3/4 of the
  ring is free (dp_rx.c:332-334).

**Reaping a REO destination ring** (`ath12k_dp_rx_process`, dp_rx.c:2931-3089) [V]. The REO
destination entry is 8 words (hal_desc.h:862-869, bits hal_desc.h:590-625, 692-707, 852-860):

| Word | Content |
|---|---|
| 0, 1 | buffer address 39:0, RBM, cookie 31:12 of word 1 |
| 2 | MPDU info: MSDU count 7:0, fragment 8, retry 9, AMPDU 10, BAR 11, valid PN 12, raw 13, more fragments 14, QoS control valid 27, TID 31:28 |
| 3 | peer metadata (peer id, vdev id; layout depends on the metadata version, hal_desc.h:605-625) |
| 4 | MSDU info: first 0, last 1, continuation 2, length 16:3, drop 17, valid SA/DA 18/19, DA multicast 20, checksum fail 22/23, from/to DS 24/25, decap format 30:29 |
| 5, 6 | buffer virtual address (cookie conversion) |
| 7 | info0: buffer type 0, push reason 2:1 (0 ERR_DETECTED, 1 ROUTING_INSTRUCTION), error code 7:3, source link 15:13, ring id 27:20, looping count 31:28 |

Per entry, L19 does the following:

1. Find the descriptor from words 5/6, falling back to the cookie (dp_rx.c:2970-2999).
2. Unmap the buffer (dp_rx.c:3010-3012).
3. Free any entry whose push reason is not ROUTING_INSTRUCTION (dp_rx.c:3017-3024).
4. Take first/last/continuation, the peer id and the TID (dp_rx.c:3026-3040).
5. Stop at the budget (dp_rx.c:3050-3051) and re-read the head once when an MPDU is still
   open (dp_rx.c:3060-3063).
6. Then the return: post as many fresh buffers to the refill ring as were reaped
   (dp_rx.c:3072-3082), before processing the frames (dp_rx.c:3084).

**The RX descriptor at the start of every buffer** [V]. `struct hal_rx_desc_wcn7850` is
rx_desc.h:1521-1529, and its size is the ring's packet offset (hal.c:1505-1508, dp.c:1047-1048).
Offsets come from compiling the struct (gcc, packed, little endian):

| Bytes | Content |
|---|---|
| 0 | msdu_end TLV tag (8) |
| 8 | msdu_end (128): info5 (u16) at buffer byte 50, with L3 padding 11:10, first MSDU bit 12, last MSDU bit 13 (rx_desc.h:694-696); info10 at 80, MSDU length 13:0 (rx_desc.h:731); info11 at 84, decap format 9:8 (rx_desc.h:739); phy_meta_data at 108 (frequency, hal.c:1181-1184); info13 at 128, multicast/broadcast bit 2, length error 17, TCP/UDP checksum 18, IP checksum 19, TKIP MIC 28, decrypt 29, FCS 31 (rx_desc.h:766-793); info14 at 132, MSDU done bit 31 (rx_desc.h:799) |
| 136 | padding (8) |
| 144 | mpdu_start TLV tag (8) |
| 152 | mpdu_start (120): info2 at 180, encryption type 5:2, TID 18:15 (rx_desc.h:46, 51); sw_peer_id at 194; info4 at 196, sequence number 31:20 (rx_desc.h:78); addr2 at 218 |
| 272 | packet header TLV (128) |
| 400 | the MSDU, after L3 padding |

Processing one MSDU (`ath12k_dp_rx_process_msdu`, dp_rx.c:2777-2849) [V]:

1. MSDU done must be set in the last buffer (dp_rx.c:2801-2805).
2. A single-buffer MSDU needs length + 400 <= 2048 (dp_rx.c:2814-2820). The frame is then bytes
   400 + pad to 400 + pad + length (dp_rx.c:2821-2822). A continued MSDU is coalesced from
   several buffers (dp_rx.c:2823-2831).
3. Decap format 0 RAW, 1 NATIVE_WIFI, 2 ETHERNET2_DIX, 3 802.3 (dp_rx.h:66-69). The station
   vdev asks for native Wi-Fi unless decap offload is enabled (mac.c:9812-9818;
   TARGET_DECAP_MODE_NATIVE_WIFI = 1, hw.h:44-46).
4. Native Wi-Fi is an 802.11 header without QoS control or HT control: L19 rebuilds QoS from
   the TID and clears ORDER (dp_rx.c:2202-2248) [V]. What follows the header is an LLC/SNAP body
   [I: consistent with mac80211 then converting it to 802.3].

**Errors, which carry the buffers back too** [V]:

- REO exception ring: `ath12k_dp_rx_process_err` (dp_rx.c:3830).
- WBM release ring: `ath12k_dp_rx_process_wbm_err` (dp_rx.c:4211), which classifies REO errors
  (dp_rx.c:4073), RXDMA errors (dp_rx.c:4144) and the "null queue descriptor" case
  (dp_rx.c:4003). The last one is what a frame from a station with no peer or no REO queue gets
  [I: by the name and its use for unknown peers].
- In every case the buffer is returned to the refill ring.

## 2. Rungs

### 2.1 Gating

The new knob is `fuxiwlanjoin=N`. N is a rung number, not a bit set: 0 (the default) changes
nothing. Each rung includes the earlier ones.

- **Gate.** It needs `fuxiwlanqrtr=7` and, on salami, `fuxiwlansalami=1`. It is refused, with
  `wl_variant_invalid` and one line, with any other qrtr, with `fuxiwlankeep=1`, or for a value
  outside 0..4.
- **No transmission.** No rung here transmits, and the W20 refusal of w20 0x08 ACTIVE stays.
- **TX rule (owner).** The first rung that transmits will not be a value of `fuxiwlanjoin`. It
  gets its own knob, `fuxiwlantx=1`, default 0, refused in this build at every value but 0, and
  documented in the transmit study.
- **iOS side.** `fuxiwifiios=5` (section 4) is independent: without a radio producer it behaves
  as `=3` does when no radio lane reports.
- **Differential.** Every boot with `fuxiwlanjoin` absent or 0 logs and behaves as C2b (the
  same proof style as `wlan-w20.md` "Proof that qrtr 0..6 are unchanged").

### 2.2 What this part assumes about C2c

C2c ("RX data path", the next-rung label at `xnu/fuxi-wlan/fuxi_wlan.c:5408`) is being tested on
the phone, and its code is not on `salami`. `docs-lanes/wlan-w20.md` documents only the CE arena
(512 KiB, `WLCE_ARENA_BYTES`) and C1's rdp of 128 ids (`fuxi_wlan_ce.h:84`); it describes no DP
ring. So this part assumes the following C2c layout. Everything in this subsection is [I]; each
point names the L19 fact it copies.

1. A second arena, `wl_dp_arena`: Normal-NC like the CE arena, aligned to its size, mapped
   through the same W12/W13 path.
2. **rdp.** C1's rdp words stay at CE arena offset 0. Its 0x1000 bytes before CE0_SRC already
   hold 1024 ids, so the ids up to 193 need only `WLCE_RDP_IDS` raised to 256 and no layout
   move. **wrp** is a new 4 KiB page in the DP arena.
3. **Rings, in L19 order:**
   - the WBM idle link ring and link banks;
   - the REO exception ring and the WBM2SW3 release ring;
   - the REO hardware setup of 1.4;
   - REO destination rings REO2SW1..REO2SW4 (the hash map routes only to SW1..SW4);
   - the refill ring, the two mac buffer rings and the error destination ring;
   - HTT SRING_SETUP for those four, then RX_RING_SELECTION_CFG.

   Entries in section 3. The monitor status rings are left out (a ladder bit adds them, see 5).
   No TCL, no TX completion ring, no REO command or status ring (both serve peers).
4. **Cookie conversion off.** SW_COOKIE_CFG1 is not written, so REO is expected to return the
   20-bit cookie the host posted in word 1 31:12, and words 5/6 are ignored. The cookie is the
   index of the buffer in C2c's table.
5. **Polling.** One poll pass reads the REO destination, exception, release and error
   destination rings after the CEs. No MSI, as in C1.
6. **Placement.** All of this runs after `W22 complete` and before W23's START_SCAN, so the
   scan runs with the RX path live (C2c's question: does MGMT_RX now arrive?).

If C2c differs, the J rungs adopt C2c's layout; the J proof lines don't depend on it.

### 2.3 The rungs

Each rung ends, as every run does, with `wl5_abort` containment (section 3.3).

**J0, host only** (no boot). This is the model and test of section 6, run before any kit:
`python3 tools/test-fuxi-wlan-ce.py` with the J scenarios.

**J1 (`fuxiwlanjoin=1`): regulatory IL. Nothing transmitted.**

1. After `W22 hw mode`, keep the boot-time CC_EXT C1 already receives and parse it (1.1).
2. Send SET_CURRENT_COUNTRY "IL", then wait up to 3000 ms for a CC_EXT whose alpha2 is IL
   (L19 waits 3 s, reg.h:16).
3. Parse it, and log the status and the rules (counts and MHz ranges only).
4. Build the scan channel list as the intersection of C2's 38 channels with the rules: NO_IR or
   DFS stays passive, and every channel is passive anyway (C2 rule).
5. Run C2's scan with that list.

Proof:

```
FUXI WLAN: J1 cc boot alpha2 XX status 0 rules 2g N 5g N (dfs-region R)
FUXI WLAN: J1 set current country IL sent (0x3a001, 12 bytes)
FUXI WLAN: J1 cc IL status 0 phy 0 rules 2g N 5g N dfs-region R after T ms
FUXI WLAN: J1 rule 2g A-B MHz bw W pwr P flags 0xF          (one per rule, at most 16)
FUXI WLAN: J1 rule 5g A-B MHz bw W pwr P flags 0xF
FUXI WLAN: J1 channel list N of 38 (2.4 GHz N, 5 GHz N, passive all)
FUXI WLAN: W23 scan started after T ms                       (C2 lines follow unchanged)
```

Failure lines (each contains and stops):

- `J1 timeout cc IL (3000 ms)`;
- `J1 cc IL status S (<name>)`;
- `J1 cc rules overrun`;
- `J1 channel list empty`.

**J2 (`fuxiwlanjoin=2`): J1 + C2c's RX rings (as assumed in 2.2). Nothing transmitted.** Proof:
C2c's own lines, then:

```
FUXI WLAN: J2 dp arena IOVA 0x... bytes 0x200000 Normal-NC mapped; wrp 0x... rdp ids 256
FUXI WLAN: J2 umac ring 120 entries 2048 base 0x... misc 0x... readback ok   (idle link; then 0, 131, 1..4)
FUXI WLAN: J2 idle link posted 2048 descriptors (banks 1, 256 KiB)
FUXI WLAN: J2 reo setup hash 0x43214321 misc 0x... aging 100000/40000 readback ok
FUXI WLAN: J2 htt sring 160 info0 0x0205000b info1 0x00020400 sent      (then 161, 169, 186)
FUXI WLAN: J2 htt rx ring selection 161 info0 0x1400010c tlv 0xd sent    (then 169)
FUXI WLAN: J2 refill posted 512 (cookie 0..511), wrp[0] 512
FUXI WLAN: W23 scan completed ...                                        (C2 lines)
FUXI WLAN: J2 rings after scan: refill tail X mac0 H/T mac1 H/T reo1..4 H H H H exc H rel H err H
FUXI WLAN: J2 complete; stop/contain
```

The value of J2 is the last ring line plus C2's MGMT_RX count:

- the refill tail moving (firmware took buffers into its mac rings);
- non-zero frames in the scan summary.

Either one answers C2c's question. All of it is reported only, never acted on.

**J3 (`fuxiwlanjoin=3`): J2 + parsing and returning RX entries. Nothing transmitted.**

What the poll pass does with each entry it reaps (REO destination, exception, release, error
destination):

1. **Check the entry** against our own table. The cookie must be < 512 and marked posted, and
   the entry's address must equal that slot's IOVA. Any mismatch stops the run (C1 style: a
   device that writes something we didn't post is a containment event).
2. **Read the descriptor** of 1.5: MSDU done, length, pad, decap, multicast, the error bits.
3. **Count, never print, any frame byte.** Counters by ring, by push reason and error code, by
   decap format, by frame type (data, QoS data, null) and by multicast. Multi-buffer MSDUs are
   counted and dropped.
4. **Recycle the buffer.** Zero its first 400 bytes, so that a stale MSDU done bit can't pass
   the next check, then re-post the same buffer and cookie to the refill ring. L19 posts fresh
   buffers instead (dp_rx.c:3079-3081) [I].

Before association almost nothing should reach the REO destination rings. Frames for an
unknown peer go to the error paths [I], so the numbers are a measurement, not a pass condition.

PASS =

- every reaped entry passed the checks;
- every buffer posted is either posted again or held, with the counts adding up;
- the run contains.

Proof:

```
FUXI WLAN: J3 rx reaped N (reo N exc N rel N err N) bad 0 recycled N in-flight 512
FUXI WLAN: J3 rx push routing N error N codes [c:n ...] decap raw N nwifi N eth N; mcast N
FUXI WLAN: J3 rx data N qos N null N mgmt N other N; multi-buffer dropped N; done-bit missing N
```

**J4 (`fuxiwlanjoin=4` + `fuxiwifiios=5`): J3 + the iOS radio data backend, RX only, link down.**

On the radio side, each native-Wi-Fi MSDU that passes J3's checks:

1. is converted by the pure 802.11 to 802.3 function of section 4;
2. is offered to the handoff with `fuxi_wifi_radio_rx()`, which copies it and never sleeps.

On the iOS side the frames are counted, but none is handed to `en0` while the link is down. It
stays down in this part, because the link comes up only after an association.

Proof:

```
FUXI WLAN: J4 rx to ios offered N converted N dropped-format N (protected N, 4-addr N, no-snap N, long N)
FUXI WIFI IOS: radio-data: rx N frames held 0, link down (no association in this build), tx refused N
```

The iOS line is printed on every link change and at most once a second while frames move (the
usbnet backend's pacing).

## 3. Rings, buffers, DMA budget, containment

### 3.1 Every ring

"Writer" is the side that produces entries. Ids and entry sizes are [V] (sections 1.2 to 1.4).
The proposed sizes are [I].

| Ring | id | Entry bytes | L19 entries | J entries | Producer (writes entries) | Consumer | Programmed by |
|---|---|---|---|---|---|---|---|
| CE0..CE4 (C1) | 64.., 82.., 101.. | 16 / 8 / 16 | see `wlan-w20.md` | unchanged | host or CE engine | | host registers |
| WBM idle link | 120 | 8 | 32768 | 2048 | host (initial fill), then WBM | REO/RXDMA (take link descriptors) | host registers; shadow 19 |
| REO exception (REO2SW0) | 0 | 32 | 128 | 128 | REO | host | host registers; shadow 8 |
| REO destination REO2SW1..8 | 1..8 | 32 | 8 x 2048 | rings 1..4 x 512 | REO | host | host registers; shadow 0..7 |
| WBM2SW3 RX error release | 131 | 32 | 1024 | 256 | WBM | host | host registers; shadow 24 |
| refill (SW2RXDMA_BUF0) | 160 | 8 | 4096 | 512 | host | firmware | HTT SRING_SETUP |
| mac buffer 0 / 1 | 161 / 169 | 8 | 2 x 2048 | 2 x 2048 (kept: firmware fills them, the size is its to use) | firmware | RXDMA hardware | HTT SRING_SETUP |
| RXDMA error destination | 186 | 32 | 1024 | 256 | RXDMA | host | HTT SRING_SETUP |
| monitor status 0 / 1 | 185 / 193 | 8 | 2 x 1024 (+ status buffers) | 0 (ladder bit: 2 x 64) | host buffers, RXDMA writes | host | HTT SRING_SETUP |
| REO command / status | 20 / 21 | | 256 / 2048 | 0 (peer queues; next part) | | | host registers |
| TCL data, TX completion, SW2WBM release, REO reinject | | | | 0 (TX side; next part) | | | |

Entry layouts are in sections 1.4 and 1.5. Every ring base is aligned to 4 KiB, which is stronger
than L19's 8-byte alignment.

### 3.2 Buffers and the DMA budget

**J layout [I]:** one 2 MiB arena, aligned to its size (fits the W12 pool 0xa0000000 to
0xb0000000 and stays below 4 GiB, so every "high" word is 0):

| Offset | Bytes | Content |
|---|---|---|
| 0x000000 | 0x1000 | wrp (188 bytes used) |
| 0x001000 | 0x4000 | WBM idle link 2048 x 8 |
| 0x005000 | 0x1000 | REO exception 128 x 32 |
| 0x006000 | 0x10000 | REO destination 1..4, 512 x 32 each (0x4000 apart) |
| 0x016000 | 0x2000 | WBM2SW3 256 x 32 |
| 0x018000 | 0x1000 | refill 512 x 8 |
| 0x019000 | 0x8000 | mac buffer 0 and 1, 2048 x 8 each |
| 0x021000 | 0x2000 | RXDMA error destination 256 x 32 |
| 0x023000 | 0x1000 | spare (monitor status ladder bit: 2 x 64 x 8) |
| 0x040000 | 0x40000 | link descriptors 2048 x 128 (128-aligned) |
| 0x080000 | 0x100000 | RX buffers 512 x 2048 (cookie i at 0x080000 + 2048 i) |
| 0x180000 | 0x80000 | headroom (unused, never posted) |

**Totals:**

- L19 RX side, for comparison [computed from the table above]: rings about 0.9 MiB, link
  descriptors 4 MiB, RX buffers 4096 x 2176 bytes, about 8.5 MiB. About 13.4 MiB in all.
- J: 2 MiB, with 1.28 MiB device-writable at any time (rings, link descriptors, posted buffers).
- Together with C1's 512 KiB CE arena, the lane holds 2.5 MiB of IOVA-mapped DMA.

**Buffer rules [I]:**

- A buffer is posted at most once; the slot table holds its state (free, posted, held).
- A buffer is recycled only after one of the host-consumed rings has returned it.
- Its first 400 bytes are zeroed before it is posted again.
- Nothing in the arena is ever freed or reused for anything else while the device can reach it.
  Ring memory has no "unconfigure" in L19: no HTT message withdraws an LMAC ring [V by absence
  in `enum htt_h2t_msg_type`, dp.h:470-478].

### 3.3 Containment order

Device-writable memory now includes:

- the firmware-filled mac buffer rings, rdp and wrp;
- the REO, WBM and RXDMA destination rings;
- the link descriptors, the RX buffers and the refill ring (the firmware reads it and may write
  its tail).

The order is `wl5_abort`'s (`xnu/fuxi-wlan/fuxi_wlan.c:6086-6131`). J changes nothing in it and
adds the DP arena to the same proven-free list:

1. `wl_ce_contain` latches stopping: no more posts, refills, HTT or WMI sends, and no
   consumption. J adds "no refill post" and "no ring index publish" to that latch (the C2
   STOP_SCAN rule stays before the latch).
2. W11 QRTR BYE and stop, then MHI reset (BAR0 + 0x38), checked.
3. **Bus master off** (PCI command bit 2 cleared, read back), then the MSI undo.
4. **PERST asserted** (gpio94 OUT bit cleared, read back), then `dsb sy` and a 10 ms drain.
5. SMMU undo (`wl_smmu_undo`).
6. Only when 2 to 5 are all proven: `wl_dma_free(&wl_dp_arena)` next to
   `wl_dma_free(&wl_ce_arena)`. Otherwise the DP arena is quarantined with everything else
   until reboot (`abort buffers quarantined ...`).

Rules [I]:

- J never frees, unmaps or shrinks any part of the DP arena on a success path. The arena lives
  until step 6, the same as the CE arena.
- An RX buffer handed to `fuxi_wifi_radio_rx()` is copied during the call, so no DMA address or
  pointer leaves the radio lane.

## 4. The iOS side: a radio data backend for en0 (`fuxiwifiios=5`)

The model is route U (`fuxiwifiios=2`, `docs-lanes/wifi-ios.md`, `fuxi_wifi_ios.c:202-327`):
same backend boundary (`fuxi_wifi_ios.h`), different `tx_frame`, `start` and RX source.

**Station.** The K2 radio scan backend (`fuxiwifiios=3`) as it is: real scan results; join
refused with EOPNOTSUPP until the transmit part. The scan producer stays the radio lane.

**RX handoff.** RX gets a new pair of calls in `fuxi_wifi_radio.c`, which keeps its rule that
the producer never sleeps and never calls the iOS side:

- **Producer:** `int fuxi_wifi_radio_rx(const uint8_t *frame8023, size_t length)` copies one
  Ethernet frame (14..1514 bytes) into a bounded ring of 32 slots x 1514. When the ring is
  full it drops the frame and counts it (`rx_full`). It returns 0 or a drop reason. It is
  called from the radio lane's poll thread only.
- **Consumer:** a thread `wifi-radio-rx`, started by `start()`, drains the ring like
  `usbnet_rx`: every 1 ms while frames moved in the last second, else every 20 ms. It copies
  each frame out under the handoff's leaf lock and calls `input->rx_frame()` only while the link
  is up; otherwise it counts `held`. It never touches MMIO or DMA.

**TX.** `tx_frame` counts and returns false while no TX knob is approved. The en0 output path
already copies the mbuf (`fuxi_wifi_ios.c:31`). The future transmit path is the next part.

**Link.** `link_state(true)` is called only by the station worker after a join completes.
Nothing in this part can complete one, so the link is down for the boot. A stop of the radio
lane (`fuxi_wifi_radio_stopped`) calls `link_state(false)` and the `fuxi_wifi_sta_lost` path, as
the K2 backend does for state.

**802.11 to 802.3** (pure, in the host-tested part of `fuxi_wifi_radio.c`):
`int fuxi_wifi_radio_to_8023(const uint8_t *in, size_t len, const uint8_t bssid[6], const uint8_t self[6], uint8_t *out, size_t *out_len)`.
The rules are IEEE 802.11-2020 frame format facts (9.2.4.1 Frame Control, 9.3.2.1 data frame
format and its address-field table) and RFC 1042:

1. Need at least 24 bytes. FC type must be data (2). Subtypes Data (0) and QoS Data (8) are
   accepted; any "no data" subtype (bit 2 of the subtype: Null, QoS Null) is dropped. Protected
   bit set: drop (open networks only). More Fragments set, or a non-zero fragment number in
   Sequence Control: drop.
2. To DS 0 and From DS 1 only (AP to STA). DA = Address 1, BSSID = Address 2, SA = Address 3.
   Address 2 must equal the BSSID we are joined to, and DA must be `self` or a group address
   (bit 0 of byte 0). Anything else drops, including four-address frames.
3. Header length is 24, plus 2 when the subtype has QoS. Native Wi-Fi from the firmware has
   no QoS or HT control (1.5), but a raw 802.11 frame with HT control (QoS + Order) adds 4.
4. Body: `AA AA 03 00 00 00` (RFC 1042) or `AA AA 03 00 00 F8` (bridge tunnel), then a 2-byte
   type. Output = DA, SA, type, payload; at most 1514 bytes, at least 14. Any other body
   (plain LLC, short) drops.

**The reverse**, `fuxi_wifi_radio_from_8023()`, is specified here for completeness and is only
host-tested; nothing calls it in this part. Given DA, SA and type it builds:

- Frame Control: Data, To DS 1;
- Address 1 = BSSID, Address 2 = self, Address 3 = DA;
- Sequence Control 0, left to the firmware [I];
- then the RFC 1042 header, or the bridge-tunnel one for types 0x80f3 and 0x8137, then the type.

**Privacy.** Logs carry counts only. No SSID, no BSSID, no MAC, no frame byte (the K2 rule).

## 5. Risks and open questions

### Top risks

1. **Cookie conversion off is untested anywhere.** L19 always enables it (dp.c:1330-1380), and
   it needs QMI `dev_mem` (CMEM), which our W15 doesn't record.
   - If REO rewrites words 0/1 or needs the CMEM tables even when conversion is disabled, J3's
     entry check will stop the run at the first entry. That stop is safe, but it ends the rung.
   - Ladder: record `dev_mem` from the QMI capability response and implement the PPT/SPT tables
     (dp.c:1533-1640) behind a bit.
2. **Reduced ring depth and fewer REO destination rings.** L19 always programs 8 REO2SW rings of
   2048 and a 4 MiB link pool. Firmware or hardware may assume the hash map's targets are all
   enabled, or may stall when the idle link list is short.
   - Ladder: 8 REO rings (still with the hash to 1..4), then L19's idle link count.
3. **Monitor status rings left out.** L19 HTT-configures them on WCN7850 (dp_rx.c:4652-4667).
   Firmware may refuse RX_RING_SELECTION_CFG or the RX enable without them.
   - Ladder bit: 2 x 64 entries with 64 status buffers each.
4. **LMAC rings can't be withdrawn.** After the first HTT SRING_SETUP the firmware owns pointers
   into our arena until reset. Any early failure must go straight to containment, never "free
   and retry" (section 3.3). This is the main reason for the single arena and the latch rule.
5. **The regulatory answer for IL is unknown until J1 runs.** The board regulatory database may
   return CHANGE_NOT_ALLOWED (3), a different alpha2, or rules that remove channels C2 scans.
   J1 logs and stops; it never guesses a default.
6. **rdp growth.** Raising `WLCE_RDP_IDS` to 256 changes a C1 constant. The test must show C1's
   seven rings and their rdp words are byte-identical.
7. **Polling cost.** Four more destination rings per poll pass at 1 ms (C1 cadence). The window
   register is needed for every R0/R2 access above 0xFE0. HP reads of UMAC destination rings
   come from rdp, not registers [V: hal.c:2410], so the pass reads memory only.

### Open questions

- Does any RX need a vdev up or a peer on WCN7850? Before association the expected answer is
  error paths only, which J3 measures; the next part's peer setup answers the rest.
- Does the firmware answer HTT SRING_SETUP at all when RESPONSE_REQUIRED is clear? L19 never
  asks (dp_tx.c:1248-1252). J2 can't prove acceptance except by the refill tail moving.
- Peer metadata version: L19 picks the layout from the HTT version exchange (dp_rx.c:3036-3037,
  hal_desc.h:605-625). J3 only counts peer ids, so a wrong version mislabels counters; it can't
  corrupt anything.
- Is C2c's actual layout the one assumed in 2.2? Reconcile when C2c lands; only the J2 setup
  lines would change.

## 6. Host-model test plan

The style is `tools/fuxi-wlan-ce-model.c` and `tools/test-fuxi-wlan-ce.py`:

- the production code is cut out of `fuxi_wlan.c` into an include and built with ASan/UBSan and
  -Werror against the pure header;
- a fake target enforces every access;
- L19 facts are re-derived from the tree when `$ATH12K_SRC` is set.

**Model additions (`tools/fuxi-wlan-ce-model.c`):**

- **Registers.** The REO block (GEN_ENABLE, MISC_CTRL, aging, DEST_RING_CTRL IX_2/3, the R0
  registers of rings 0..4) and the WBM idle link / WBM2SW3 R0 registers, reached through the
  window. Shadow indices 0..4, 8, 19 and 24. Any other access aborts.
- **HTT parser on CE4.** It checks every SRING_SETUP word against values written down in the
  model (not taken from the header): info0/info1 per ring, the base inside the DP arena, head
  and tail equal to the rdp/wrp words of 1.2, MSI 0, interrupt 0, info2 0. It checks
  RX_RING_SELECTION_CFG's 17 words.
- **Fake RXDMA/REO.** It takes refill entries, checks address, RBM 6 and cookie, and writes the
  400-byte descriptor (MSDU done, length, pad, decap 1) and a native-Wi-Fi frame into the
  buffer. It then writes a REO destination entry (push reason 1) to ring 1..4 by a hash of its
  own, and moves rdp.
- **Error scripts:**
  - an entry with an unknown cookie, an address mismatch, or a cookie already returned (each
    must stop the run before the buffer is touched);
  - push reason 0 on the exception ring;
  - an MSDU over one buffer;
  - MSDU done clear;
  - a stale buffer that still has MSDU done set but was zeroed by J3, so it must fail the
    done check.
- **CC_EXT scripts:** status 0 IL with 2 + 4 rules; status 3; alpha2 mismatch; num rules past
  the TLV; zero rules; phy 1.
- **Containment.** After the latch the fake device may still write any posted buffer and every
  ring (as hardware can until PERST). The model then checks that the host does not read, post
  or publish anything, and that the DP arena is freed only on the proven path.

**`tools/test-fuxi-wlan-ce.py` additions:**

- Static checks:
  - the J code is gated by `fuxiwlanjoin` and qrtr 7;
  - no J path sends any WMI command id outside {SET_CURRENT_COUNTRY, SCAN_CHAN_LIST,
    START_SCAN/STOP_SCAN and C2's existing set}: no VDEV_START, PEER_*, MGMT_TX or TCL access
    can exist in this part;
  - `fuxiwlantx` is refused at every value but 0;
  - the DP arena free sits inside the proven branch of `wl5_abort`;
  - every new log line fits 254 bytes and prints no frame byte or address;
  - no em or en dash.
- With `$ATH12K_SRC`, re-derive from L19 and compare with the header and the model's dump:
  - the ids and tags of 1.1;
  - the CC_EXT struct size and word offsets;
  - the HTT message types, both struct sizes and every mask;
  - the HTT ring id/type enums and the mapping of `ath12k_dp_tx_get_ring_id_type`;
  - ring ids 0..8, 120, 131, 160, 161, 169, 185, 186, 193 from the hal enum and PMAC stride;
  - entry sizes from the struct definitions;
  - the WCN7850 REO register offsets (hw.c:1238-1270) and the hash map;
  - the WCN7850 RX descriptor offsets (compile `rx_desc.h` with a shim, as this study did, and
    compare every offset in 1.5);
  - the filter flags and the descriptor size 400.
- Differential: qrtr 6 and 7 with `fuxiwlanjoin=0` produce the C2b log byte for byte.

**iOS side (`tools/test-fuxi-wifi-radio.py`, `tools/test-fuxi-wifi-ios.py`):**

- `fuxi_wifi_radio_to_8023` and `_from_8023` against hand-built vectors for every rule in
  section 4, plus a round trip.
- The RX handoff ring: full, wraparound, a link-down hold, and a stop while frames are queued.
- The backend with `fuxiwifiios=5`: TX refused and counted, the link never up, join refused,
  the scan unchanged from `=3`.
- 30 or more mutants of the bounds (length 13/14/1514/1515, DS bits, protected, subtype bit 2,
  SNAP variants).

**Device steps, after J0 is green, one kit each:** J1, then J2, J3, J4, each with the previous
rung's line set as the baseline, under `fuxiwlansalami=1`, never with `fuxiwlantx`.

## 7. What the next part needs

These are listed so the transmit study starts from here. Nothing is specified in this part:

- the station steps that put frames on the air and their completions;
- the peer and its REO queues (the REO command and status rings above);
- the TX data path rings (TCL, TX completion, cookie conversion for WBM);
- vdev start/up (`vdev_start_delay = true`, hw.c:1437);
- the iOS backend's TX wiring and the join;
- DHCP over the radio.

The transmit rung's knob is `fuxiwlantx`, default 0, and it requires J1's regulatory gate
(1.1, "What must precede any TX").
