# WLAN HD1: hang window and force-wake state (fuxiwlandiag)

Read-only diagnostic for the WCN7850 firmware thread assert (BHI ERRDBG2/3 =
`cmnos_thread_patch.c` line 1170/1174, MHI still M0, no SYSERR, WMI silent).
It implements experiments 1 and 3 of the hang study
(`docs-lanes/wlan-fw-hang-study.md` on `study/fw-hang-data`, 9d5a2ad5): which
thread died, and whether UMAC was awake. Nothing is written to the device and
nothing is sent to the firmware.

## Knob

`fuxiwlandiag=N` (boot-arg, else ADT `/defaults`, like every `fuxiwlan*` knob),
any stage:

| Bit | Part |
| --- | --- |
| 0x01 | hexdump of the hang window in our first type-1 (HOST_DDR) QMI segment |
| 0x02 | raw reads of BAR `0x3004` (PCIE_SOC_WAKE) and SoC `0x01E04040` (PCIE_SCRATCH_0) |

Absent or 0: no HD1 line, no HD1 access, the boot as before. Negative or bits
outside 0x3: refused before any access (invalid selector).

## Where it prints

At every point the lane prints BHI ERRDBG: M4 step 16, `wl5_status` (every M6
step, and so `wl_bhi_diag`), the M6f boot logs, `wl_wlfw_diagnostics` (W13
RESPOND_MEM, W15 MODE timeout, W15 WLAN start, W20 snapshot), the W28 crash
line (only when it prints, w20 0x2000), and once more in `wl5_abort` right
before the MHI reset write (`contain before MHI reset`), which is before every
DMA free.

## Lines

```
FUXI WLAN: HD1 hang <where>: segment S type 1 size Z IOVA I; offset O (cap 0x1e), cap length 384; dump N bytes
FUXI WLAN: HD1 hex +0x...: 64 bytes as hex, a space every 4      (N/64 rows)
FUXI WLAN: HD1 ascii R runs C chars[ (cut)]: run|run|...
FUXI WLAN: HD1 wake <where>: soc-wake 0x3004 V, scratch0 0x1e04040 V (umac awake|umac down|other|unknown); win W->W restored; faults F; read-only
```

Variants: `no type-1 segment allocated (requested n); nothing read` before
QMI REQUEST_MEM or when no type-1 segment holds an allocation; `offset O not
inside the segment; nothing read`; `not read (BAR/MHI not established)` for
the wake part before M5 step 17. Without CAP TLV 0x1e the offset is 0 (marked
`no cap 0x1e, 0`). Every line fits the console's 254 bytes; the ascii line is
cut at 191 characters (`(cut)`), the hex rows always carry every byte.

## Part 1: the hang window

- Segment: the first `wl_target[i]` with type 1 that holds a Normal-NC
  allocation (W13 leaves unsupported types unallocated; those are skipped).
- Offset: QMI CAP TLV 0x1e `hang_data_addr_offset`, stored when parsed (the
  existing `W14 cap` lines are unchanged). Length: 384, cut to the segment.
- Read: `dsb sy`, then byte loads from our own NC DMA buffer into a stack
  copy; the buffer is never written, cleaned or invalidated.
- Lifetime: `wl_target` is freed only in `wl5_abort`, after the MHI reset,
  bus master off, PERST and the SMMU undo, and never reallocated (an allocated
  slot fails W12, a duplicate REQUEST_MEM is refused), so every dump reads
  live memory.

The firmware's layout of these bytes is not documented in any ISC or BSD
source (study 1.1); the dump is raw. Raw firmware bytes can include addresses
of nearby networks once RX is live: read the dump before sharing a log.

## Part 2: force-wake state

| Register | Vendor ISC (qca-wifi-host-cmn @ 9c16bd2) | L19 (Linux ath12k v6.19) | Access here |
| --- | --- | --- | --- |
| BAR `0x3004` PCIE_SOC_WAKE | `hif/src/pcie/if_pci.h:36` `PCIE_REG_WAKE_UMAC_OFFSET 0x3004`; written 1 at `if_pci.c:4063`, 0 at `:4109` | `pci.h:26` `PCIE_SOC_WAKE_PCIE_LOCAL_REG 0x3004`; written at `pci.c:344` | one `wl_mrd` (below 0x80000, in our 16 KiB map) |
| SoC `0x01E04040` PCIE_SCRATCH_0 | `if_pci.h:38` `(0x01E04000 + 0x40)`; polled for `HIF_POLL_UMAC_WAKE 0x2` (`if_pci.c:4014`, `:4070-4078`), 0x1 while UMAC is down (`:4020-4023`); read through the window (`hif/src/hif_io32.h:33-45`, `:279-296`) | `pci.h:23` `PCIE_SCRATCH_0_SOC_PCIE_REG 0x4040` as a BAR offset, never used in pci.c; WCN7850 uses the dynamic window (`pci.c:1635`, `:163-182`, `:1214-1238`) | `wl_soc_read`: window 0x3c, read BAR+0x84040, restore, read back |

Both are right for WCN7850 (KIWI). L19's comments on `pci.h:22` and `:25` are
swapped (the wake comment sits on 0x4040); the values agree with the vendor.
L19's 0x4040 is the unwindowed alias of the same register; it lies outside our
mapped first 16 KiB of BAR0, so HD1 takes the vendor's windowed path through
the existing read-only helper. The vendor reads SCRATCH_0 while UMAC may still
be powered down (that is what 0x1 means), so the read is safe in that state.

Writes: none to either register. `wl_soc_read` writes only the window register
`0x310c` (select, then restore by the containment writer) and refuses after a
latched fault; HD1 reads `0x310c` before and after and prints both. No
force-wake request, no MHI wake vote, no doorbell.

## Reading the next boot

- `scratch0 ... (umac awake)` at the stall: UMAC was up, the study's case A
  (lost UMAC write while collapsed) is unlikely; look at ring servicing.
- `(umac down)` with `soc-wake 0x3004 0x1`: the host's one-shot wake vote is
  still set but UMAC is collapsed: case A is live; next is the vendor
  handshake (study 4.2).
- The ascii line: a thread or task name next to `cmnos_thread_patch.c` names
  the thread that died; compare the window at `W15 WLAN start` (baseline) with
  the one before the MHI reset.

## Host tests

`tools/test-fuxi-wlan-hd.py`: the HD1 block, its defines, the knob parse and
`wl_soc_read` cut from `fuxi_wlan.c`, run under ASan/UBSan against a BAR model
(only `0x310c`, `0x3004` and SoC `0x01E04040` through window 0x3c exist; any
other access, any write but the window select/restore, fails) and heap
segments of their exact size (ASan bounds every load; a copy proves no byte
changes). Scenarios: hang, wake, knob; every line within 254 bytes. Mutants
caught: a force-wake write, a write into the segment, the dump not cut to the
segment, a widened allowlist, the `wl5_abort` call moved after the MHI reset.
With `$KIWI_VENDOR_SRC` and `$ATH12K_SRC` it re-derives both offsets and the
window constants, else prints SKIPPED.
