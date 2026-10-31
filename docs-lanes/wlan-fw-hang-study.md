# WLAN firmware thread assert: hang data, hang events, polled-host duties

Study only. No code changes. Scope: the stall seen on WCN7850 where BHI
ERRDBG points at the string `cmnos_thread_patch.c` and line 1170/1174, MHI
stays in M0, there is no SYSERR, and WMI events stop. On the Android firmware
this follows RX going live; on the upstream c7-00108 firmware it happens during
the last BDF segment. QMI CAP reports `hang_data_length` 384.

## Sources and citation keys

| Key | Source | License |
| --- | --- | --- |
| `L19:` | torvalds/linux tag v6.19, `drivers/net/wireless/ath/ath12k/` | BSD-3-Clause-Clear (SPDX line, `coredump.c:1`) |
| `V:` | LineageOS/android_kernel_oneplus_sm8550-modules at 9c16bd2, `qcom/opensource/wlan/` | ISC; every file cited below was checked for the ISC "Permission to use, copy, modify, and/or distribute" header |
| `F:` | this repo, `xnu/fuxi-wlan/fuxi_wlan.c` and `docs-lanes/` | project |

Not read: `platform/`, cnss2, MHI core (`drivers/bus/mhi`), all GPL. Where a
fact lives only there, this doc says so instead of guessing. Inferences are
marked [I].

## Short answer

1. In the ISC vendor host, the firmware hang data is **not** fetched by the
   WLAN driver. It arrives from the GPL platform driver as an opaque
   `(pointer, length)` pair in a `CNSS_HANG_EVENT` uevent
   (`V:qcacld-3.0/core/pld/src/pld_pcie.c:383-391`). HDD copies at most 195
   bytes of it to offset 195 of a 390 byte buffer and sends that to user space
   (`V:qcacld-3.0/core/hdd/src/wlan_hdd_driver_ops.c:2197-2217`,
   `V:qca-wifi-host-cmn/qdf/inc/qdf_hang_event_notifier.h:29-33`). The layout
   of the firmware's own bytes is not defined in any ISC or BSD file studied.
   L19 does not decode `hang_data_*` at all (no match for `hang_data` under
   `L19:`). The only location hint is QMI CAP TLV `0x1e`
   `hang_data_addr_offset` (u32), which this project already logs
   (`docs-lanes/wlan-w14.md:71-72`, `F:3801-3805`). [I] It is most plausibly an
   offset into a firmware-requested host memory segment (the `HOST_DDR`
   type 1 region that L19 dumps as "remote memory",
   `L19:coredump.c:17-18`, `L19:qmi.h:175`), so a polled host can read it
   from its own DMA memory with no device access.
2. No `WMI_PDEV_FW_HANG` event exists in fw-api. The only hang related WMI
   items are the host to target `WMI_FORCE_FW_HANG_CMDID`
   (`V:fw-api/fw/wmi_unified.h:1278`) and UMAC hang recovery over HTT. The
   vendor "fw hang" path is driven by the platform driver (uevent), and the
   dbglog parser has a raw (non TLV) mode for assert time diag payloads
   (`V:qca-wifi-host-cmn/utils/fwlog/dbglog_host.c:1883-1889`).
3. The strongest gap for an interrupt-less host is the **UMAC force wake
   handshake** the vendor does before every UMAC register write above BAR+4K:
   MHI wake vote, write `1` to BAR `0x3004`, then poll BAR
   `0x01E04040` (PCIE_SCRATCH_0) until it reads `0x2`
   (`V:qca-wifi-host-cmn/hif/src/pcie/if_pci.c:4046-4079`,
   `V:.../hif/src/pcie/if_pci.h:34-48`, `V:.../hal/wifi3.0/hal_api.h:287-296`).
   Second: periodic servicing (CE poll every 10 ms, DP ring poll every 5 ms
   when in poll mode), which a host that stops polling after a phase skips
   entirely.

## 1. QMI hang data mechanism

### 1.1 What the ISC host code shows

Producer and transport:

- `pld_pcie_update_event()` receives `struct cnss_uevent_data` from the
  platform driver. On `CNSS_HANG_EVENT` it takes
  `hang_event->hang_event_data` and `hang_event->hang_event_data_len` and
  forwards them as `PLD_FW_HANG_EVENT`
  (`V:qcacld-3.0/core/pld/src/pld_pcie.c:371-391`). The definition of
  `struct cnss_hang_event` and the code that fills it are in cnss2 (GPL, not
  read).
- PLD carries it as `hang_data.hang_event_data` (`void *`) plus
  `hang_data.hang_event_data_len` (`u16`)
  (`V:qcacld-3.0/core/pld/inc/pld_common.h:201-251`).

Consumer (HDD) and the 390 byte buffer:

- Static buffer `g_fw_host_hang_event[QDF_HANG_EVENT_DATA_SIZE]`
  (`V:qcacld-3.0/core/hdd/src/wlan_hdd_driver_ops.c:71`).
- Sizes: `QDF_HANG_EVENT_DATA_SIZE 390`, host may write up to offset
  `QDF_WLAN_MAX_HOST_OFFSET 194`, firmware data starts at
  `QDF_WLAN_HANG_FW_OFFSET 195`
  (`V:qca-wifi-host-cmn/qdf/inc/qdf_hang_event_notifier.h:27-33`).
- On `PLD_FW_HANG_EVENT`: zero the buffer, run host notifiers that append host
  TLVs from offset 0, then copy `min(fw_len, 195)` firmware bytes to offset
  195, then send all 390 bytes with the recovery reason
  (`wlan_hdd_driver_ops.c:2197-2217`, `:985-995`).
- Host TLV header: one `u16` = `(tag << 8) | (len & 0xff)`
  (`qdf_hang_event_notifier.h:61-67`); tags `HANG_EVT_TAG_CDS` (0) through
  `HANG_EVT_TAG_BUS_INFO` (9) (`:48-59`). Host producers include CDS version
  (`V:qcacld-3.0/core/cds/src/cds_api.c:106-112`, `:593-633`), WMI command and
  event history (`V:qca-wifi-host-cmn/wmi/src/wmi_hang_event.c:22-110`), HTC
  credit history (`V:qca-wifi-host-cmn/htc/htc_credit_history.c:35`,
  `:173-211`), CE and bus info (`V:qca-wifi-host-cmn/hif/src/hif_main.c:689-694`).

Consequences:

- The firmware reports 384 bytes, but the vendor host forwards at most 195 of
  them (`wlan_hdd_driver_ops.c:2205-2208`). [I] So Qualcomm treats the start
  of the firmware blob as the useful part.
- The firmware bytes are copied verbatim with no parsing in ISC code
  (`wlan_hdd_driver_ops.c:2210-2214`). Their internal layout is **not
  documented in any studied ISC or BSD file**. [I] Given the shared buffer and
  the host side convention, it is likely the same `u16 tag<<8|len` TLV stream,
  but that must be checked against a real dump.

### 1.2 Where the 384 bytes live

- QMI CAP (`QMI_WLFW_CAP_RESP`) TLV `0x1e` `hang_data_addr_offset` u32 and TLV
  `0x1f` `hang_data_length` u16 (`docs-lanes/wlan-w14.md:71-72`; decoded at
  `F:3801-3805`, value currently "reported only").
- It is not a QMI indication: no WLAN side handler for one exists in ISC code;
  the delivery is the platform uevent above (`pld_pcie.c:383`).
- Not CMEM: L19 defines CMEM only as `dev_mem` index 0
  (`L19:qmi.h:32`); nothing ties hang data to it.
- [I] The name "addr_offset" plus the uevent being raised by the platform
  driver (which owns the firmware memory segments) suggests: base = the
  `HOST_DDR_REGION_TYPE` (1) segment the host gave in RESPOND_MEM
  (`L19:qmi.h:175`), offset = `hang_data_addr_offset`, length =
  `hang_data_length`. L19 already classifies that segment as firmware crash
  "remote memory" for its coredump (`L19:coredump.c:17-18`,
  `L19:coredump.h:11-20`). If the offset exceeds the type 1 segment size, try
  the next candidate: the RDDM image (needs RDDM, see 1.3).

### 1.3 Is there a host-readable thread assert record?

| Record | Read-only after a silent stall? | Content | Cite |
| --- | --- | --- | --- |
| BHI `ERRCODE`, `ERRDBG1..3` (BHI base + 0x30..0x3c) | Yes, already read | ERRDBG holds string pointer and line (observed) | `F:97`, `F:4625-4645` |
| Hang data (384 B) in host memory [I] | Yes if [I] holds; it is host DMA memory | Unknown TLVs; [I] includes thread and PC | 1.1, 1.2 |
| `HOST_DDR` (type 1) segment, whole | Yes, host memory | Firmware remote memory; L19 dumps it as `FW_CRASH_DUMP_REMOTE_MEM_DATA` | `L19:coredump.c:17-18` |
| `M3_DUMP` (type 3) segment | Yes, host memory | M3 dump | `L19:coredump.c:20-21` |
| RDDM image (`rddm_size` 0x780000 on WCN7850 row) | Only after RDDM; not read-only (forces a mode change) | Full target RAM, thread control blocks, stacks | `L19:hw.c:1469`, `L19:mhi.c:281-292`, `L19:pci.c:1362-1377`, `L19:mhi.c:589-590` |
| WMI_DIAG / WMI_DEBUG_MESG dbglog | Only if the WMI pipe still flows | Raw assert dump format: "when fw assert occurs, host can't use TLV format" | `V:.../utils/fwlog/dbglog_host.c:1883-1889`, `:2005-2011` |

L19 coredump file format, if RDDM is ever used: `struct ath12k_dump_file_data`
(16 B magic "ATH12K-FW-DUMP", `len`, `version`, `chip_id`, `qrtr_id`,
`bus_id`, guid, `tv_sec`, `tv_nsec`, 128 B pad) followed by
`struct ath12k_tlv_dump_data {le32 type; le32 tlv_len; data}` records, type from
`enum ath12k_fw_crash_dump_type` (`L19:coredump.h:9-58`). Types 0..6: PAGING,
RDDM, REMOTE_MEM, PAGEABLE, M3_DUMP, NONE, MLO_GLOBAL (`L19:coredump.h:11-20`).

No studied file gives a fixed struct for "thread id, PC, reason". [I] The
thread name is in RAM (RDDM) or possibly in the hang blob; the BHI ERRDBG
values give file and line only.

## 2. Firmware hang event handling in the vendor driver

- `WMI_PDEV_FW_HANG` and `wlan_fw_hang_data`: no match anywhere in `V:` (grep
  of `qca-wifi-host-cmn`, `qcacld-3.0`, `fw-api`).
- Host to target `WMI_FORCE_FW_HANG_CMDID` (`V:fw-api/fw/wmi_unified.h:1278`),
  gated by `WMI_SERVICE_FORCE_FW_HANG` = 24
  (`V:fw-api/fw/wmi_services.h:64`). L19 sends it as
  `{tlv_header, type, delay_time_ms}` with type 1 = assert, delay 0
  (`L19:wmi.h:3655-3666`, `L19:wmi.c:1848-1868`, `L19:debugfs.c:80-84`).
  Vendor equivalent: `wma_crash_inject()` (`V:qcacld-3.0/core/wma/src/wma_main.c:9973-9983`).
- UMAC hang recovery: `WMI_SERVICE_UMAC_HANG_RECOVERY_SUPPORT` = 340
  (`V:fw-api/fw/wmi_services.h:593`), HTT H2T types 0x21 and 0x22
  (`V:fw-api/fw/htt.h:920-921`) with a shared memory message
  `htt_umac_hang_recovery_msg_shmem_t` (`htt.h:10376-10471`). [I] This is a
  WIN/MLO feature and not the path WCN7850 mobile takes.
- `PLD_FW_HANG_EVENT` handling: HDD logs "Received firmware hang event",
  builds the 390 B buffer and sends it to user space; it does not itself
  trigger recovery (`wlan_hdd_driver_ops.c:2197-2217`). Recovery shutdown and
  re-init come from separate platform events in the same switch [I].
- dbglog: `wma->is_fw_assert` switches the WMI_DIAG and WMI_DEBUG_MESG
  handlers to raw payloads (`dbglog_host.c:1883-1889`, `:2005-2011`), but no
  studied ISC file sets it to 1 (grep finds only the two clears). Handlers are
  registered on `wmi_dbg_msg_event_id`, `wmi_diag_container_event_id` and
  `wmi_diag_event_id` (`dbglog_host.c:4512-4532`). L19 only traces
  `WMI_DIAG_EVENTID` (`L19:wmi.c:8946-8949`, `:9908-9909`).
- L19 crash path: only via MHI callbacks. `MHI_CB_SYS_ERROR` warns;
  `MHI_CB_EE_RDDM` queues `reset_work` (`L19:mhi.c:270-292`). With no SYSERR
  and no RDDM, L19 would notice nothing either. [I] The stall here is a
  firmware internal assert that the firmware did not escalate to SYSERR, so
  only out-of-band reads (BHI, host memory) show it.

## 3. Periodic or per-access host work an interrupt-less host may skip

Ranked by [I] likelihood of mattering to a cmnos thread watchdog.

| # | Host duty | Vendor | L19 | Polled host now |
| --- | --- | --- | --- | --- |
| A | UMAC force wake around UMAC MMIO: MHI wake vote, write `0x3004=1`, poll `0x01E04040 == 0x2` every 5 ms up to 50 ms | `if_pci.c:4027-4089`, `if_pci.h:34-48`, `HIF_POLL_UMAC_WAKE 0x2` at `if_pci.c:4014`; called from `hal_write32_mb` for offsets >= 4K when not in init phase (`hal_api.h:287-296`) | MHI `mhi_device_get_sync` before any access >= `0xFE0` after INIT_DONE (`L19:pci.c:33-38`, `:139-161`, `:1206-1212`); `0x3004=1` + 5 ms once after power up (`L19:pci.c:342-346`, `:1515`) | Writes `0x3004=1` once (`F:1798`); holds MHI wake doorbell ch 127 (`F:4080`); never polls SCRATCH_0 |
| B | Shadow register ring pointer updates (WCN7850 uses shadow regs at BAR `0x8fc`, 40 regs, below 4K so no wake needed) | n/a | `supports_shadow_regs = true` on the WCN7850 row (`L19:hw.c:1451`), `L19:hal.h:38-43`; `ath12k_hal_srng_shadow_update_hp_tp` exists but has no caller (`L19:hal.c:2541-2552`) | Shadow v3 sent at QMI (`F:4337-4369`) |
| C | CE servicing: CE poll timer every 10 ms on poll-enabled CEs | `ce_main.c:45`, `:484-492`, `:2543-2552` | IRQ driven; RX replenish retry timer (`L19:core.c:2265`, `L19:ce.c:491`, `:859`) | Polled during conversations; "polling ended" after keep (`F:4119`) |
| D | DP ring servicing: poll mode timer every 5 ms over all srngs; LMAC reap timer | `dp_main.c:287`, `:2999-3024`, `:3148`, `:5200-5214` | ext IRQ NAPI [I] | Refill ring filled at W26 (`F:5680-5683`); REO dst reaping status unknown here |
| E | MHI event ring consumption and event ring doorbells | MHI core (GPL, not read) | Events: ctrl ring 32 elems, data ring 256 elems, `irq_moderation_ms` 0 and 1 (`L19:mhi.c:117-131`); `timeout_ms` 2000 (`:140-142`) | Polled during conversations only [I] |
| F | HTC UL poll timer for polled endpoints | `htc_services.c:370-378` | n/a | n/a |
| G | Sleep timer / keep awake count | `if_pci.c:1688-1696` | n/a | n/a |

Not periodic pings: the vendor latency detect timer only checks host tasklet
and CE latency (`V:.../hif/src/hif_main.c:1045-1061`, `:1012-1030`). No
periodic WMI "host wake", "D0" or keepalive to firmware was found in L19 (only
per vdev STA keepalive, `L19:wmi.c:10799-10834`). [I] So the firmware does not
expect a heartbeat; it expects (1) its rings to drain and (2) UMAC accesses to
follow the wake handshake.

[I] Why `cmnos_thread` asserts fit A, C/D/E:
- When the host writes a UMAC ring HP/TP register while UMAC is power
  collapsed, the write can be lost; firmware then waits on a ring that never
  moves. The vendor added the SCRATCH_0 handshake precisely to "validate UMAC
  is awake" (`if_pci.h:37`). A lost doorbell during the last BDF segment (QMI
  over MHI) or when RX ring traffic starts matches both observed triggers.
- A full destination ring (CE target to host, MHI event ring, REO dst, WBM
  release) blocks the firmware thread that produces into it; a cmnos thread
  watchdog fires if a thread does not yield or check in within its period.
  This would also explain "alive in M0, no WMI events".

## 4. Experiments for a polled host, ranked

1. **Read the hang blob from host memory (read-only).** After the stall,
   take the type 1 `HOST_DDR` segment base the host gave in RESPOND_MEM
   (`F:4397-4414`, `F:4422-4459`), add CAP `0x1e` `hang_data_addr_offset`
   (`F:3801-3802`), hexdump `hang_data_length` (384) bytes. Also hexdump the
   same window before RX goes live as a baseline. If all zero, try the same
   offset in each other requested segment. Parse candidates as `u16
   (tag<<8|len)` TLVs (`qdf_hang_event_notifier.h:61-67`) and look for ASCII
   thread names and code addresses. Zero device side effect.
2. **Implement the vendor UMAC force wake handshake** before every UMAC MMIO
   write at offset >= 4K (ring HP/TP outside shadow, REO/WBM/TCL regs): keep
   the ch 127 wake vote (`F:4080`), write BAR `0x3004 = 1`, poll BAR
   `0x01E04040` until `0x2`, 5 ms step, 50 ms limit
   (`if_pci.c:4063-4079`, `if_pci.h:34-48`). Log every handshake failure with
   the read value. Run it first on the upstream firmware: if the assert in the
   last BDF segment goes away, A is confirmed.
3. **Log SCRATCH_0 alongside BHI** in the existing diagnostic line
   (`F:4645`): read BAR `0x01E04040` and `0x3004` at each poll tick and at the
   stall. Read-only. Shows whether UMAC was awake when the assert happened.
4. **Keep polling after "keep".** Ensure CE target to host rings, MHI event
   rings (with event ring doorbells), REO dst and WBM release rings are
   serviced at least every 5 to 10 ms indefinitely after RX goes live, mirroring
   `CE_POLL_TIMEOUT 10` (`ce_main.c:45`) and `DP_INTR_POLL_TIMER_MS 5`
   (`dp_main.c:287`). Log each ring's HP/TP at the stall (REO dst TP addresses
   already listed at `F:4345`). A full ring at stall time points at E or D.
5. **Route ring pointer writes only through shadow registers** for WCN7850
   (BAR `0x8fc + 4*i`, `L19:hal.h:38-43`) per the shadow v3 table sent at QMI
   (`F:4344`), so no ring doorbell needs UMAC wake. Verify no HP/TP write goes
   to a raw UMAC address after shadow config.
6. **Enable firmware dbglog to WMI_DIAG before RX goes live** and capture the
   last diag events; on assert, parse the payload raw, not as TLV
   (`dbglog_host.c:1883-1889`). Useful only if WMI still flows up to the
   assert.
7. **Controlled assert as a reference.** On a healthy session send
   `WMI_FORCE_FW_HANG_CMDID` `{type 1, delay 0}` (`L19:wmi.h:3655-3666`) if
   `WMI_SERVICE_FORCE_FW_HANG` (24) is set, then repeat 1 and 3. This shows
   what a known assert writes into the hang window and BHI, giving a decoder
   baseline. Destructive to the session.
8. **Last resort: RDDM.** Force RDDM (L19 `mhi_force_rddm_mode`,
   `L19:mhi.c:589-590`) and pull the image (`L19:pci.c:1362-1377`,
   `rddm_size` `L19:hw.c:1469`) to find the thread control block by the
   `cmnos_thread_patch.c` string pointer from ERRDBG. Changes device state;
   needs the BHIe RDDM path the project does not have yet.
