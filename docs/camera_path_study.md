# Camera path study (salami, OnePlus 11, SM8550): re-check and a realistic path

Status: research and documentation only. No code, no device access, nothing booted. Written
2026-10-30 on branch `study/camera-path` (based on `salami` c18f99d). Information for the owner, not
a commitment to build a camera lane.

Tags used below:

- **[V]** verified, with a source URL or a `file:line` into a pinned public tree or this repo.
- **[I]** inferred: reasoning from verified facts, press specs or general knowledge. Needs a device
  read before anyone builds on it.

Pinned trees (cited as `L:` and `V:` below):

- **L:** mainline Linux, `torvalds/linux` master at `d89e1d186ec9` (committed 2026-10-29, top-level
  Makefile says 7.3.0). Every `L:path:line` is into that commit.
  <https://github.com/torvalds/linux/tree/d89e1d186ec9271bf78e62771779e9fc03bf0fc3>
- **V:** vendor device trees, `LineageOS/android_kernel_oneplus_sm8550-devicetrees` branch
  `lineage-23.2` at `200c441349246b6d893f1e9c5443e0aca8debc2a`. Every `V:path:line` is into that
  commit. <https://github.com/LineageOS/android_kernel_oneplus_sm8550-devicetrees/tree/200c441349246b6d893f1e9c5443e0aca8debc2a>
- **repo:** this repository at `salami` c18f99d.

Nothing from either tree is copied here. Register names, addresses, GPIO numbers and voltages are
quoted as facts with their location, the same way the WLAN lane cites bindings by name only
(`docs-lanes/wlan-rails.md`).

---

## 0. The old survey is not in the repo

The brief asks for a re-check of `docs/camera_survey_salami.md` (2026-10-29). **That file does not
exist** on `salami`, `main`, or any other remote branch (`feat/c3-wip`, `feat/wifi-data-backend`,
`fuxi-1f28342`, `study/eapol`, `study/join-result`, `study/rung3-join`), and no commit message in
`git log --all` mentions a camera survey. It was probably never pushed. **[V]** (checked with
`git ls-tree -r` on every remote head and `git grep` on `salami`.)

So this study re-checks the survey's claims as the brief summarises them: sensors IMX890 (main),
IMX581, IMX709, front Samsung S5K3P9; CCI and CSIPHY slots; CAMSS block map; Android CamX stack. If
the survey file turns up, compare it against the table in section 1.4.

Camera-related facts the repo already holds:

| Fact | Where | Tag |
|---|---|---|
| `0x41b00` is `ldon7` = pm8010 "N" L7, the camera PMIC, max 2960 mV. The Fuxi touch_power vote (3200 mV) would over-volt it, so `--touch*` is refused on salami. | `tools/run-ios-salami.py:73-76`, `:193-195`; `shim/fuxi-probe/probe.c:978-979` | [V] |
| Fuxi touch AVDD is `ldom6` (3.2 V). On salami `L6M` is the main camera's AF supply at 2.8 V (V: section 2.2). Fuxi rail addresses must never be reused on salami. | `shim/fuxi-probe/probe.c:610`; V: `salami_camera_overlay_common.dtsi:125` | [V] |
| `--product-camera` adds an empty `/product/camera` node, so MobileGestalt reports `video-camera`. Default off. | `docs-lanes/sys-port.md:65-82`, `tools/mkadt-fuxi.py:141-145` | [V] |
| mediaserverd's camera pipeline (CMCapture, AppleCVAPhoto) preloads Metal shaders and aborts without a GPU. FuxiMetal hard-denies mediaserverd a Metal device. | `docs-lanes/cooling.md:73-85` | [V] |
| `StartRecorderServer=false` in MediaToolbox, because the camera filter prewarm calls a NULL OpenGL function and crash-loops mediaserverd. | `tools/sweet-root-tweaks.py:20-21`, `:231` | [V] |
| TSENS has `camera-0` and `camera-1` sensors on tsens1. | `docs-lanes/thermal.md:314` | [V] |

---

## 1. Re-verification from public sources

### 1.1 Mainline Linux CAMSS on SM8550: supported now

| Claim | Status | Source |
|---|---|---|
| Mainline `qcom-camss` has an SM8550 match entry, `qcom,sm8550-camss` -> `sm8550_resources` (`.version = CAMSS_8550`) | [V] | L:`drivers/media/platform/qcom/camss/camss.c:5768-5779`, `:5828` |
| SM8550 CAMSS map: 8 CSIPHY (3-phase "1.0" ops, i.e. the gen2 v2.1.2 lane table), 3 full CSID + 2 CSID-lite (CSID gen3 ops), 3 full VFE + 2 VFE-lite (VFE gen3 ops), and a separate `csid_wrapper` region | [V] | L:`camss.c:2970-3107` (CSIPHY), `:3109-3189` (CSID), `:3191-3306` (VFE) |
| SM8550 CSIPHY lane register table | [V] | L:`camss-csiphy-3ph-1-0.c:608` (`lane_regs_sm8550`), `:1140` |
| VFE 780 (SM8550) write masters: RDI0..2 are WM 23..25 on the full VFE, and lite VFEs have RDI0..4 from WM 0 | [V] | L:`camss-vfe-gen3.c:39-48` |
| Mainline VFE gen3 on SM8550 does RDI (raw) output only. The `line_num = 3` on full VFEs means three RDI lines, with no pixel (ISP) path. | [V] for `line_num`, [I] for "no PIX" (consistent with `camss-vfe-gen3.c` writing only RDI WMs) | L:`camss.c:3207`, `:3231`, `:3255`, `camss-vfe-gen3.c:46-48` |
| Mainline CAMSS loads **no firmware** at all (no `request_firmware` or MDT loader anywhere in the driver) | [V] | `grep -rn request_firmware L:drivers/media/platform/qcom/camss/` returns nothing |
| SM8550 CAMSS DT node `camss: isp@acb7000` in `sm8550.dtsi`, `status = "disabled"` by default | [V] | L:`arch/arm64/boot/dts/qcom/sm8550.dtsi:3862-4070` |
| SM8550 CAMSS DT binding `qcom,sm8550-camss.yaml` (GPL-2.0 OR BSD-2-Clause) | [V] | L:`Documentation/devicetree/bindings/media/qcom,sm8550-camss.yaml` |
| Driver series history: Depeng Shao, "media: qcom: camss: Add sm8550 support", v1 March 2024 to v9 January 2025; the DT node was applied mid-2025 | [V] for the postings, [I] for the exact kernel release (likely 6.15 for the driver, 6.17 for the DT) | <https://lwn.net/Articles/1004967/>, <https://lkml.iu.edu/hypermail/linux/kernel/2501.1/00848.html>, <https://lkml.indiana.edu/hypermail/linux/kernel/2506.2/03648.html> |

### 1.2 CCI, camcc and the CAMSS addresses (mainline DT, BSD-3-Clause)

| Block | Address | IRQ (GIC SPI) | Notes | Source |
|---|---|---|---|---|
| CCI0 | `0x0ac15000` (0x1000) | 426 | two buses: bus0 SDA/SCL gpio110/111, bus1 gpio112/113 | L:`sm8550.dtsi:3770-3801`, `:4872-4935` |
| CCI1 | `0x0ac16000` | 427 | one bus in mainline: gpio114/115 | L:`sm8550.dtsi:3803-3827`, `:4936-4967` |
| CCI2 | `0x0ac17000` | 428 | two buses: gpio74/75 and gpio0/1 | L:`sm8550.dtsi:3829-3860`, `:4968-5031` |
| CSID0/1/2 | `0x0acb7000` / `0x0acb9000` / `0x0acbb000` | 601 / 603 / 431 | | L:`sm8550.dtsi:3865-3867`, `:3977-3979` |
| CSID-lite 0/1 | `0x0acca000` / `0x0acce000` | 605 / 376 | | same |
| CSID wrapper | `0x0acb6000` (0x1000) | | | L:`sm8550.dtsi:3870` |
| CSIPHY0..7 | `0x0ace4000` + n * 0x2000 | 477, 478, 479, 448, 122, 89, 278, 277 | | L:`sm8550.dtsi:3871-3878`, `:3982-3989` |
| VFE0/1/2 | `0x0ac62000` / `0x0ac71000` / `0x0ac80000` (0xf000 each) | 602 / 604 / 688 | | L:`sm8550.dtsi:3879-3881`, `:3990-3992` |
| VFE-lite 0/1 | `0x0accb000` / `0x0accf000` (0x1800) | 606 / 377 | | L:`sm8550.dtsi:3882-3883`, `:3993-3994` |
| CAMSS SMMU stream | `apps_smmu 0x800 mask 0x20` | | | L:`sm8550.dtsi:4021` |
| Power domains | camcc `IFE_0/1/2_GDSC`, `TITAN_TOP_GDSC` | | CCI only needs `TITAN_TOP_GDSC` | L:`sm8550.dtsi:3774`, `:4023-4030` |
| Interconnect | `ahb` (APPSS -> CAMERA_CFG) and `hf_0_mnoc` (CAMNOC_HF -> EBI1) | | 2 GB/s avg/peak in the driver table | L:`sm8550.dtsi:4014-4019`, L:`camss.c:3308-3319` |
| camcc | `0x0ade0000` (0x20000), needs `RPMHPD_MMCX` and `RPMHPD_MXC` at least low_svs | | | L:`sm8550.dtsi:4072-4085` |
| MCLK pins | function `cam_mclk` on gpio100..107 (MCLK0..7) | | | L:`sm8550.dtsi:4728-4870` |
| `qcom,sm8550-cci` compatible, falls back to `qcom,msm8996-cci` (CCI v2) | | | | L:`Documentation/devicetree/bindings/i2c/qcom,i2c-cci.yaml:44`; L:`drivers/i2c/busses/i2c-qcom-cci.c:847`, `:861` |

The vendor tree agrees on CCI and CSID/IFE bases: V:`qcom/camera/kalama-camera.dtsi:964-967`
(cci0 `0xac15000`), `:1052-1055` (cci1 `0xac16000`), `:1140-1143` (cci2 `0xac17000`), `:2444`
(`csid780` at `0xacb7000`), `:2478` (`vfe780` at `0xac62000`). **[V]** One difference: the vendor
names the IFE-lite at `0xacca000` (V:`kalama-camera.dtsi:2708`) where mainline puts CSID-lite0 at
`0xacca000` and VFE-lite0 at `0xaccb000`. **[I]** these are the same 0x2800 window split differently.

### 1.3 Sensor drivers in mainline

| Sensor (OnePlus 11 role) | Mainline driver? | Nearest mainline driver | Chip-ID register | Tag |
|---|---|---|---|---|
| Sony IMX890, 50 MP, main | **No** | none in the same family. `imx412.c` (IMX412/IMX577) is the closest large Sony with the standard Sony register layout. | `0x0016` (16-bit model id) on Sony parts, e.g. IMX412 at L:`drivers/media/i2c/imx412.c:30` | [V] no driver (L:`drivers/media/i2c/Kconfig`, no `VIDEO_IMX890`); [I] expected value `0x0890` |
| Sony IMX581, 48 MP, ultrawide | **No** | same as above | `0x0016`, expected `0x0581` | [V] no driver; [I] value |
| Sony IMX709, 32 MP RGBW, tele | **No** | same as above | `0x0016`, expected `0x0709` | [V] no driver; [I] value |
| Front 16 MP: **IMX471 per press specs**, S5K3P9 per the survey | **IMX471: yes** (`VIDEO_IMX471`, new) | `imx471.c` | IMX471: `0x0016` = `0x0471` (L:`drivers/media/i2c/imx471.c:25-26`). Samsung parts: `0x0000`, e.g. S5K3M5 = `0x30d5` (L:`drivers/media/i2c/s5k3m5.c:22-23`). S5K3P9 expected `0x3109`. | [V] IMX471 driver (L:`Kconfig:303`); [I] S5K3P9 value |

The Samsung drivers in mainline are `s5k3m5`, `s5k5baf`, `s5k6a3`, `s5kjn1`, `s5kjn5`
(L:`drivers/media/i2c/Kconfig:839-871`). There is no S5K3P9 driver. **[V]**

Mainline has a working example on the same SoC: the SM8550 QRD and the HDK rear camera card wire a
Samsung S5K3M5 tele at CCI1 bus0 address `0x10`, CSIPHY3 (4 data lanes, clock lane 4), reset
gpio119, AF on `vreg_l7n_2p96`, at a 602.5 MHz link frequency (L:`arch/arm64/boot/dts/qcom/sm8550-qrd.dts:805-848`,
L:`sm8550-hdk-rear-camera-card.dtso:15-62`). **[V]** That slot lines up with salami's tele slot
(section 2.1), which is why section 4 starts there.

### 1.4 What the survey (as summarised) got wrong or left open

1. **Front sensor model.** The brief records the front as Samsung S5K3P9. Public spec sources say
   Sony IMX471, 16 MP, f/2.45 ([OnePlus spec page](https://www.oneplus.com/ie/11/specs),
   [GSMArena](https://www.gsmarena.com/oneplus_11_camera_details_officially_revealed-news-57060.php)).
   **[I]** Both can be true, because OnePlus may second-source the front module. The vendor DT
   carries no sensor names (CamX keeps them in userspace sensor modules), so the DT cannot settle it.
   **Rung C1 settles it:** the chip ID at `0x0016` (`0x0471`) or at `0x0000` (`0x3109`). If it is
   IMX471, mainline has a driver to read for register semantics.
2. **"No mainline CAMSS for SM8550"** (if the survey said so, or assumed CamX is the only path).
   That is out of date: mainline has CAMSS, CCI, camcc and the DT node (1.1, 1.2). **[V]**
3. **Front camera bus.** The front sensor is on **CCI1 master 1**, and on kalama that master is
   wired to the **AON CCI pins gpio208/209 (function `aon_cci`)**, not to normal TLMM CCI pins
   (V:`kalama-camera.dtsi:172-180`, `:1052-1078`; V:`salami_camera_overlay_common.dtsi:453`,
   `:492`, `aon-user` at `:436`, `:475`). **[V]** Mainline `sm8550.dtsi` gives CCI1 only one bus
   (gpio114/115) and has no AON CCI. So the front camera is the hardest one to reach, not the easiest.
4. **Ultrawide and main share CCI0.** Main is CCI0 master 0 and ultrawide is CCI0 master 1; tele
   is CCI1 master 0 (2.1). **[V]**
5. **CSIPHY slots.** Main CSIPHY2, ultrawide CSIPHY1, tele CSIPHY3, front CSIPHY4 (2.1). **[V]**
6. **MCLK is 19.2 MHz** on every salami sensor (vendor `clock-rates = <19200000>`), not the
   24 MHz used by the mainline QRD tele. **[V]** Any sensor PLL table must match 19.2 MHz.

---

## 2. Hardware path on salami

### 2.1 Slot map (vendor DT)

All rows: V:`qcom/camera/oplus/salami_camera_overlay_common.dtsi`. Sensor names per slot are
**[I]** (from the slot role, the EEPROM/actuator labels and press specs). Everything else is **[V]**.

| cell | Role (sensor) | CCI / master | CSIPHY | MCLK (gpio) | Reset gpio | Extras | Lines |
|---|---|---|---|---|---|---|---|
| 0 | main (IMX890) | CCI0 M0 | 2 | MCLK2 (gpio102) | 117 (`CAM_RESET1`) | EEPROM, actuator, OIS on CCI0 M0, flash; `is-power-up-advance = <2>` | 194-238 |
| 1 | ultrawide (IMX581) | CCI0 M1 | 1 | MCLK1 (gpio101) | 118 (`CAM_RESET2`) | EEPROM, actuator, aux VDIG LDO | 291-333 |
| 2 | tele (IMX709) | CCI1 M0 | 3 | MCLK0 (gpio100) | 119 (`CAM_RESET3`) | EEPROM, actuator | 385-425 |
| 3 | front (IMX471 or S5K3P9) | CCI1 M1 (AON pins 208/209) | 4 | MCLK4 (gpio104) | 120 (`CAM_RESET4`) | EEPROM, `aon-user` | 461-499 |

The salami overlay applies to `oplus,project-id = <22811 22861 22862 22863 22864>`
(V:`qcom/camera/oplus/salami-camera-overlay.dts`). **[V]**

### 2.2 Sensor rails (vendor names -> expected cmd-db names)

Vendor rail names are PMIC letter + LDO number. `M` and `N` are the two pm8010 camera PMICs, `E` is
pm8550ve (L:`sm8550-hdk.dts:828`, `:895` show two `qcom,pm8010-rpmh-regulators` blocks). The
cmd-db names follow the form this repo already uses (`ldob10`, `ldon7`). Only `ldon7` = `0x41b00`
is a known address here **[V]** (`tools/run-ios-salami.py:73`). Every other address must be resolved
by name from cmd-db at run time, as the WLAN M3 step 1 does (`docs-lanes/wlan-rails.md`). Never
hard-code a Fuxi address.

| Slot | Supply | Vendor rail | cmd-db name [I] | Volts (min = max) |
|---|---|---|---|---|
| main | vana | L3M | `ldom3` | 2.800 V |
| main | v_custom1 | L5M | `ldom5` | 1.800 V |
| main | vdig | L1M | `ldom1` | 1.104 V |
| main | vio | L5N | `ldon5` | 1.800 V |
| main | vaf | L6M | `ldom6` | 2.800 V |
| main | v_custom2 (OIS) | L4M | `ldom4` | 2.800 V |
| ultrawide | vio / vana / v_custom1 / vaf / vdig | L5N / L3N / gpio LDO / L6N / L1N | `ldon5` / `ldon3` / PMIC GPIO / `ldon6` / `ldon1` | 1.800 / 2.904 / 1.824 / 2.800 / 1.104 V |
| tele | vana / vdig / vio / vaf | L7N / L2M / L5N / L7M | `ldon7` / `ldom2` / `ldon5` / `ldom7` | 2.904 / 0.816 / 1.800 / 2.800 V |
| front | vana / vdig / vio | L4N / L2N / L5N | `ldon4` / `ldon2` / `ldon5` | 2.800 / 1.056 / 1.800 V |
| CSIPHY (all) | 1p2 / 0p9 | L3E / L1E | `ldoe3` / `ldoe1` | 1.200 V / 0.875 to 0.925 V |

Sources: main V:`:205-216`; ultrawide V:`:301-311`; tele V:`:394-404`;
front V:`:469-478`; CSIPHY V:`:60-66`, `:93-99`. The ultrawide's aux VDIG LDO is a
`regulator-fixed` switched by **pm8550vs_c GPIO4** at 1.824 V (V:`:32-44`, `:501-510`). **[V]**
That is a PMIC GPIO over SPMI, not an RPMh vote.

Notes:

- `L5N` (`ldon5`, 1.8 V VIO) is shared by all four sensors. **[V]**
- The tele's VANA is `ldon7` at 2.904 V, inside the 2960 mV ceiling the repo already records.
  **[V]** This is the same rail the salami runner refuses to vote at 3200 mV.
- The mainline QRD names its tele rails `vreg_l7n_2p96`, `vreg_l4m_2p8`, `vreg_l2m_1p056`,
  `vreg_l5n_1p8` (L:`sm8550-qrd.dts:836-839`). **[V]** Salami's tele uses L2M at 0.816 V instead of
  1.056 V and L7N for VANA instead of AF. **[I]** different module, same PMIC instances, so the
  slot is similar but not identical. Use salami's numbers, never the QRD's.

### 2.3 Clocks and power domains needed per rung

| Need | Clock / domain | Source |
|---|---|---|
| Any CAMSS access | MMCX and MXC RPMh ARC votes (camcc's own power domains), `GCC_CAMERA_AHB_CLK`, camcc `TITAN_TOP_GDSC` | L:`sm8550.dtsi:4072-4085`, `:3774` |
| CCI | `CAM_CC_CAMNOC_AXI_CLK`, `CAM_CC_CPAS_AHB_CLK`, `CAM_CC_CCI_n_CLK` (vendor runs the CCI source at 37.5 MHz) | L:`sm8550.dtsi:3775-3777`; V:`kalama-camera.dtsi:975-979` |
| Sensor | `CAM_CC_MCLKn_CLK` at 19.2 MHz, `cam_mclk` pinmux, reset GPIO | V: slot table above |
| CSIPHY | `CAM_CC_CSIPHYn_CLK`, `CAM_CC_CSInPHYTIMER_CLK` (400 MHz), `CAM_CC_CPHY_RX_CLK_SRC` (400/480 MHz), rails L3E and L1E | L:`camss.c:2970-3107`; V:`salami_camera_overlay_common.dtsi:51-82` |
| CSID | `CAM_CC_CSID_CLK`, `CAM_CC_CSID_CSIPHY_RX_CLK` (400/480 MHz) | L:`camss.c:3113-3189` |
| IFE + RDI write | `IFE_n_GDSC`, `CAM_CC_IFE_n_CLK` (466 to 785 MHz), `IFE_n_FAST_AHB`, `CPAS_IFE_n`, `CPAS_FAST_AHB`, `gcc_axi_hf`, CAMNOC HF bandwidth vote, SMMU context for SID `0x800` | L:`camss.c:3191-3306`, L:`sm8550.dtsi:4021` |

**[I]** The XNU side has no camcc driver today. Every camera rung therefore needs a small camcc
helper (RCG configure, branch enable, GDSC on, with readback). That is the same shape as the GPU
lane's clock work (`docs-lanes/gpu-clock.md`), so the pattern exists in this repo.

### 2.4 First proof: read a chip ID over CCI

CCI v2 is a command-queue I2C engine: reset, per-master SCL/SDA timing, queue a write of the
16-bit register address, queue a read of N bytes, start the queue, poll done and read the FIFO.
The register offsets are public in L:`drivers/i2c/busses/i2c-qcom-cci.c:16-60`. **[V]** That driver
is GPL-2.0, so XNU code must be written fresh from the register semantics, with no code copied.

The minimal sequence for one read:

1. Vote MMCX/MXC, enable the camera AHB clock and `TITAN_TOP_GDSC`; read back the GDSC status.
2. Enable CAMNOC_AXI, CPAS_AHB and CCI clocks; read `CCI_HW_VERSION` (offset 0). This is a
   bus-alive check before any sensor power. **[V]** offset from L:`i2c-qcom-cci.c:16`.
3. Power the sensor in the vendor order (VIO, then VANA/VDIG and the rest, then MCLK at 19.2 MHz,
   then release reset). **[I]** order from typical Sony sequences and the vendor regulator-name
   order. `is-power-up-advance` on the main slot hints at an OPLUS-specific early power-up.
4. Mux the CCI pins, set 400 kHz timing, read 2 bytes at `0x0016` (Sony) or `0x0000` (Samsung).
5. Power down in reverse; leave the rails off.

**Slave address [I]:** Sony IMX parts usually answer at 7-bit `0x1a` or `0x10`. The QRD's Samsung
tele sits at `0x10` (L:`sm8550-qrd.dts:827-829`). The real salami addresses live in the CamX sensor
modules in `/vendor` on the phone, not in the DT. The rung reads both candidates and never writes
to a sensor register.

### 2.5 CSIPHY and CSID: receive MIPI

- Sensors are D-PHY, 4 data lanes on the QRD example (L:`sm8550-qrd.dts:813-815`). **[V]** for
  the QRD; **[I]** that salami's rear sensors also use 4-lane D-PHY. The vendor driver supports
  both C-PHY and D-PHY (`qcom,csiphy-v2.1.2-enhance`), so salami could use C-PHY for the big
  sensors. **That must be read from the CamX sensor module before C3.** **[I]**
- The CSIPHY needs the SM8550 lane settings table (L:`camss-csiphy-3ph-1-0.c:608`) plus the
  link-frequency dependent settle count. **[V]**
- CSID gen3 routes one virtual channel and data type (RAW10 is DT `0x2b`) to an RDI path and
  counts frames with SOF/EOF interrupts. **[V]** for gen3 existence (L:`camss-csid-gen3.c`).

### 2.6 IFE RDI raw dump: the "first real frame"

With mainline as the model: the full VFE's RDI0 write master (WM 23 on VFE 780,
L:`camss-vfe-gen3.c:40`, `:47`) writes the raw Bayer packets unchanged into a buffer described by
an image address, stride and frame size. **[V]** No ISP, no firmware and no tuning are involved,
which is why this is the realistic first frame. The buffer must be mapped in the camera SMMU
context (SID `0x800`), which is the same kind of work the WLAN lane already does for its arena
(`docs-lanes/wlan-w20.md`, `wl12_map_target`).

A RAW10 4-lane IMX709 mode (for example 3264 x 2448) is about 10 MB per frame. **[I]** One frame
into a reserved buffer is plenty for the proof; streaming comes later.

---

## 3. The iOS side

### 3.1 What iOS 16 on the iPhone X expects

Apple publishes no source for any of this. XNU, IOKit and IOSurface headers are open, but nothing
camera-specific is. **[V]** (<https://github.com/apple-oss-distributions/xnu> has no camera
driver). What follows is from public writeups and from what this repo has already seen in logs.

| Layer | What it is | Tag and source |
|---|---|---|
| Camera.app, third-party apps | AVFoundation `AVCaptureSession` / `AVCaptureDevice` | [V] public API |
| Capture server | On iOS 16 it runs inside **mediaserverd** (CMCapture / FigCapture in MediaToolbox). HackiOS logs show CMCapture and AppleCVAPhoto in mediaserverd. A separate `cameracaptured` daemon is seen on later iOS (18.x crash reports); there is no public evidence that it exists on 16.7. | [V] `docs-lanes/cooling.md:73`; [V] cameracaptured on 18.x: <https://developer.apple.com/forums/thread/776833>; [I] that 16.7 has no cameracaptured |
| Access control | tccd plus mediaserverd decide camera access; the green dot indicator follows that. | [V] <https://www.jamf.com/blog/how-ios-malware-can-spy-on-users-silently/> |
| GPU use | The capture pipeline preloads Metal shaders and uses GPU filters even for preview. | [V] `docs-lanes/cooling.md:73-80`, `tools/sweet-root-tweaks.py:20-21` |
| ISP userland | A private H10 ISP framework talks to the kext through an IOKit user client with numbered selectors (a public RE gist lists selectors 41, 42, 72, 73 on an iOS 13 device). | [V] that the gist exists: <https://gist.github.com/b1n4r1b01/8a79465e89de8a70a3cb12680a304b17>; [I] everything about its protocol. **Not used here** because it is decompiler output from Apple binaries. |
| Kernel | **AppleH10CamIn** kext, which matches the ISP node in the ADT, maps the ISP and loads the ISP firmware. | [I] from the kext name and the Asahi analogy below |
| ISP firmware | Runs on the ISP's own ARM coprocessor and owns sensor control (I2C, power, modes), 3A and tuning. The host never talks to sensors. | [V] for Apple's M1 ISP: Asahi `apple-isp` series, "the coprocessor firmware manages all sensor access", closed firmware required (<https://www.phoronix.com/news/Apple-Web-Camera-Linux-Patches>, <https://gitlab.freedesktop.org/asahi/linux/-/commit/618f14cf48b94094b3fb5366de818e246d609e44>). [I] same model on A11/H10. |
| Device identity | MobileGestalt reports `video-camera` when `/product/camera` exists; flash enables the Control Center torch. | [V] `docs-lanes/sys-port.md:76-82` |

**Consequence [I]:** Apple's stack expects an Apple ISP that does everything. The host only sends
high-level commands and gets back processed YUV/HEVC-ready surfaces plus metadata (face rects,
focus, exposure). Nothing on salami can run Apple's ISP firmware, and the Qualcomm silicon needs
the opposite model: the host drives the sensor directly and the ISP (IFE/IPE/BPS) is programmed
register by register or by Qualcomm's ICP firmware.

### 3.2 Options

**(a) Emulate enough of Apple's camera interface for the Camera app.**
Two depths:

- **(a1) Kernel-level:** an `AppleH10CamIn` look-alike kext that answers the private user-client
  selectors and returns frames as if from the ISP. This needs the selector protocol, which can only
  come from reverse engineering Apple binaries. The repo rules forbid decompiled code, and the
  protocol is large (sessions, channels, metadata, firmware messages). **Odds: very low.**
  **Effort: many months.**
- **(a2) Userland-level:** inject a fake capture source into mediaserverd so AVFoundation
  sees a device. iOS 16 has no public camera plugin API (CoreMediaIO DAL/extension plugins are
  macOS-only, and USB external cameras arrived in iPadOS 17, not iOS 16). **[I]** It would need
  hooking private CMCapture classes, unlike the audio lane, which used a supported HAL plugin model
  (`docs-lanes/sys-port.md`, FuxiAudio). Even then mediaserverd's preview and photo paths want Metal,
  which this port denies it on purpose (`docs-lanes/cooling.md:73-85`). **Odds: low until GPU
  acceleration works in iOS. Effort: months.**

**(b) Simpler routes.**

- **(b1) Raw frame dump plus a host viewer.** The kernel driver captures one RDI frame into a
  buffer and exposes it read-only (a sysctl or a small user client). A device tool writes it to
  `/var/mobile`; the host pulls it over USB networking (Route U, `docs-lanes/wifi-ios.md:202`) and
  converts RAW10 Bayer to PNG with a tiny host script (unpack, demosaic, gray-world white balance).
  **Odds: high once C4 works. Effort: days on top of C4.**
- **(b2) On-device viewer app.** A small test app that polls the kernel buffer, debayers on the CPU
  at low resolution (binned or a sensor crop mode) and draws with CoreGraphics. No AVFoundation,
  no Metal, no mediaserverd. **Odds: good. Effort: 1 to 3 weeks after b1.** The CPU cost is the
  limit; a 2x2 binned 1632 x 1224 preview at 5 to 10 fps is a realistic target. **[I]**
- **(b3) Virtual device later.** Once (b2) works and iOS has GPU acceleration, revisit (a2) with a
  known-good frame source.

**Ranking:**

| Option | Effort | Odds | Verdict |
|---|---|---|---|
| (b1) raw dump + host viewer | low, after C4 | high | **do this first** |
| (b2) on-device viewer app | medium | good | natural second goal |
| (a2) fake capture source in mediaserverd | high | low (blocked on Metal) | park until GPU lane lands |
| (a1) AppleH10CamIn emulation | very high | very low, and conflicts with the no-decompilation rule | do not plan |

---

## 4. Rung ladder

Style follows the other lanes: one master knob `fuxicam=N`, **default 0 (off)**, every rung gated
and refused unless asked for, host-model tests before any device boot, and one proof line per rung
in the log. Names are proposals only. Prefix `CAM` in log lines, like `W20` in the radio lane.
Every rung ends by powering the sensor down, dropping its votes and logging `CAM off`.

Target sensor for C1 to C4: **tele (cell 2)**. Reasons: CCI1 bus 0 on normal TLMM pins
(gpio114/115), CSIPHY3 and reset gpio119, the same slot as the mainline QRD example; only four
rails, and its VANA rail (`ldon7`) is already known and bounded in this repo. **[V]** for those
facts. Main is next (shares nothing risky but has OIS and six rails). Front is last (AON CCI).

| Rung | Knob | What it does | Proof log line (proposal) | Effort [I] |
|---|---|---|---|---|
| **C0** host model | none (host only) | `tools/test-fuxi-cam.py`: slot table from section 2, CCI command-queue model (encode a 2-byte register read, decode FIFO words), refusal matrix for knobs, a check that no Fuxi rail address appears in the salami path. | `PASS: fuxi-cam C0 slot table, CCI queue model, refusals` | 2 to 3 days |
| **C1a** bus alive | `fuxicam=1` | MMCX/MXC vote, camera AHB, `TITAN_TOP_GDSC` on with readback, CAMNOC/CPAS/CCI1 clocks, read `CCI_HW_VERSION`. No sensor power. | `CAM C1a gdsc titan_top on; cci1 hw 0x<ver>` | 3 to 5 days (camcc helper is most of it) |
| **C1b** chip ID | `fuxicam=2` | Resolve `ldon5`, `ldom2`, `ldon7` (and `ldom7` only if AF is needed, it is not) by cmd-db name; refuse if any name is missing or any address equals a Fuxi touch address. Power tele, MCLK0 19.2 MHz, release gpio119, read `0x0016` at `0x1a` then `0x10`. **First proof rung.** | `CAM C1b tele cci1.m0 0x1a id 0x0709 ok` (or `id ffff nack` per address) | 3 to 5 days |
| **C1c** all IDs | `fuxicam=2 fuxicamslot=0x7` | Same for main and ultrawide (ultrawide needs the pm8550vs_c GPIO4 aux LDO). Front only with a separate bit, after the AON CCI pinmux is understood. Settles the front sensor question. | `CAM C1c main id 0x0890; uw id 0x0581` | 3 to 6 days |
| **C2** sensor stream on | `fuxicam=3` | Load a sensor mode table (pulled from the phone's CamX sensor module at run time, never in git), start streaming (`0x0100 = 1`), keep the CSIPHY off. Only proof is the sensor's own frame counter register. | `CAM C2 tele mode 3 streaming frame_cnt 12 -> 42 in 1000 ms` | 1 to 3 weeks (mode tables are the unknown) |
| **C3** receive | `fuxicam=4` | CSIPHY3 with the SM8550 lane table and settle count, CSID0 routing VC0/RAW10 to RDI0 with frames counted but the write master off. | `CAM C3 csiphy3 lanes 4 err 0; csid0 vc0 dt 0x2b sof 30 eof 30 in 1000 ms` | 1 to 2 weeks |
| **C4** first real frame | `fuxicam=5` | IFE0 RDI0 WM 23 into one SMMU-mapped buffer (SID `0x800`), one frame, then stop and contain like `wl5_abort`. | `CAM C4 rdi0 frame 1 3264x2448 raw10 bytes 9987840 crc32 0x<...> nonzero 98%` | 1 to 2 weeks |
| **C5** dump to host (b1) | `fuxicam=5 fuxicamdump=1` | Read-only user-facing export of the C4 buffer; device tool writes `/var/mobile/cam/frame.raw10`; host script makes a PNG. | `CAM C5 export 9987840 bytes`; host `PASS: frame.png 3264x2448` | 3 to 5 days |
| **C6** viewer app (b2) | `fuxicam=6` | Continuous RDI into a small ring, binned mode, CPU debayer in a test app. | `CAM C6 ring 3 frames 8.0 fps drops 0` | 1 to 3 weeks |
| **C7** autoexposure (minimal) | `fuxicam=6` | Host-side gray-world AWB and a simple AE loop writing sensor gain/exposure over CCI. No Qualcomm ISP. | `CAM C7 ae exp 1/60 gain 4.0 mean 118` | 1 week |

Total to the recommended goal (C5): **about 6 to 10 weeks** of focused work. **[I]** C6/C7 add about
3 to 4 weeks. Option (a) is not in the ladder on purpose.

### 4.1 Risks

| Risk | Effect | Mitigation |
|---|---|---|
| **Sensor mode tables** (PLL, timing, mode registers) for IMX890/581/709 are not public. | C2 cannot start without them. | Pull the CamX sensor module from the phone at run time, like the audio lane pulls firmware (`docs-lanes/audio.md` section 6), and never commit it. Or derive a minimal mode from the Sony standard register layout plus the mainline IMX412/IMX471 drivers' semantics. |
| **Wrong voltage on a shared PMIC.** The camera PMICs share instances with rails other lanes vote (the touch hazard is already recorded). | Hardware damage. | Resolve every rail by name, refuse on missing names, compare against a min/max table from section 2.2 in the host test, never send a Fuxi address. |
| **ISP firmware.** The Qualcomm ICP firmware (for IPE/BPS) is proprietary. | No hardware demosaic, noise reduction or scaling. | Not needed for RDI. Mainline CAMSS works with no firmware (1.1). Stay on RDI and debayer on the CPU. |
| **Tuning.** Colour, lens shading, defect pixels and AF all live in CamX tuning binaries. | Images look raw (green tint, vignetting). | Accept it for C4/C5; gray-world AWB in C7 is enough to recognise a scene. |
| **C-PHY vs D-PHY** on the large sensors. | C3 fails with lane errors. | Read the PHY type from the CamX sensor module before C3; start with the tele. |
| **Front camera on AON CCI.** AON pins (gpio208/209) may be owned by another subsystem (sensor hub on ADSP). **[I]** | Front never answers on CCI1 M1. | Leave the front for last; read the TLMM function state before touching it. |
| **SMMU context for SID `0x800`.** | IFE writes fault. | Reuse the WLAN lane's mapping and fault-readback pattern. |
| **Licences.** Mainline CAMSS and CCI drivers are GPL-2.0; the vendor camera DT carries no SPDX header; CamX userspace is proprietary; Apple's camera stack is closed. | Copying any of it into the repo would break the project rules. | Facts only (addresses, IRQs, offsets, voltages) with citations; fresh XNU code. The SM8550 `.dtsi` and the HDK overlay are BSD-3-Clause and the CAMSS binding is GPL-2.0 OR BSD-2-Clause, but quote facts from them all the same. No UniXNU-sheng code. No Apple binaries or decompiled code (rules out option a1). |
| **Metal in mediaserverd.** | Any AVFoundation route crashes or stalls. | Stay with b1/b2 until the GPU lane gives mediaserverd a device. |
| **Thermal.** The camera is a heat source on tsens1 `camera-0/1`. | Long streaming warms the phone. | Single frames first; the thermal lane already watches those sensors. |

---

## 5. Recommendation

**Top findings:**

1. Mainline Linux now supports SM8550 CAMSS (CSIPHY, CSID gen3, VFE 780 RDI), CCI and camcc with
   a DT node, and needs no firmware. That makes a raw-frame path on salami a documented,
   register-level job instead of a CamX reverse-engineering job.
2. The OnePlus vendor DT gives the full salami slot map: CCI master, CSIPHY, MCLK, reset GPIO and
   every rail with its voltage. The tele slot (CCI1 M0, CSIPHY3, reset gpio119, `ldon7`) matches the
   mainline SM8550 QRD tele layout. The front sensor sits on the AON CCI pins and is the hardest one.
3. The iOS Camera app path is not realistic now. Apple's stack expects an ISP that runs Apple
   firmware and owns the sensors, and the capture server needs Metal, which this port withholds
   from mediaserverd. A raw dump with a host viewer, then a CPU viewer app, is the realistic goal.

**First rung to build:** C0 (host model) then **C1b, a tele chip-ID read over CCI1**, gated by
`fuxicam=2`, default off, rails resolved by cmd-db name, reads only. Its expected proof is
`CAM C1b tele cci1.m0 0x1a id 0x0709 ok`. The same rung run on the front slot later (C1c) settles
whether the front sensor is IMX471 or S5K3P9.

---

## 6. Sources

Mainline Linux (`torvalds/linux` at `d89e1d186ec9`):

- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/drivers/media/platform/qcom/camss/camss.c>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/drivers/media/platform/qcom/camss/camss-vfe-gen3.c>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/drivers/media/platform/qcom/camss/camss-csiphy-3ph-1-0.c>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/drivers/media/platform/qcom/camss/camss-csid-gen3.c>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/drivers/i2c/busses/i2c-qcom-cci.c>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/arch/arm64/boot/dts/qcom/sm8550.dtsi>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/arch/arm64/boot/dts/qcom/sm8550-qrd.dts>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/arch/arm64/boot/dts/qcom/sm8550-hdk-rear-camera-card.dtso>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/Documentation/devicetree/bindings/media/qcom,sm8550-camss.yaml>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/Documentation/devicetree/bindings/i2c/qcom,i2c-cci.yaml>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/drivers/media/i2c/Kconfig>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/drivers/media/i2c/imx471.c>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/drivers/media/i2c/imx412.c>
- <https://github.com/torvalds/linux/blob/d89e1d186ec9271bf78e62771779e9fc03bf0fc3/drivers/media/i2c/s5k3m5.c>

SM8550 CAMSS series and DT history:

- <https://lwn.net/Articles/1004967/> (v9 cover, January 2025)
- <https://lkml.iu.edu/hypermail/linux/kernel/2501.1/00848.html> (v8)
- <https://patchew.org/linux/20240320141136.26827-1-quic._5Fdepengs@quicinc.com> (v2, March 2024)
- <https://lkml.indiana.edu/hypermail/linux/kernel/2506.2/03648.html> (sm8550 camss DT applied)

Vendor device tree (LineageOS mirror of the OnePlus SM8550 trees, `lineage-23.2` at `200c441`):

- <https://github.com/LineageOS/android_kernel_oneplus_sm8550-devicetrees/blob/200c441349246b6d893f1e9c5443e0aca8debc2a/qcom/camera/oplus/salami_camera_overlay_common.dtsi>
- <https://github.com/LineageOS/android_kernel_oneplus_sm8550-devicetrees/blob/200c441349246b6d893f1e9c5443e0aca8debc2a/qcom/camera/oplus/salami-camera-overlay.dts>
- <https://github.com/LineageOS/android_kernel_oneplus_sm8550-devicetrees/blob/200c441349246b6d893f1e9c5443e0aca8debc2a/qcom/camera/kalama-camera.dtsi>

Sensor specs:

- <https://www.oneplus.com/ie/11/specs>
- <https://www.gsmarena.com/oneplus_11_camera_details_officially_revealed-news-57060.php>
- <https://en.wikipedia.org/wiki/OnePlus_11> (lists IMX890 for the front, which looks like an error)

Apple camera stack (public writeups only):

- <https://www.phoronix.com/news/Apple-Web-Camera-Linux-Patches> (Asahi apple-isp, firmware owns sensors)
- <https://gitlab.freedesktop.org/asahi/linux/-/commit/618f14cf48b94094b3fb5366de818e246d609e44> (apple-isp driver layout)
- <https://www.jamf.com/blog/how-ios-malware-can-spy-on-users-silently/> (tccd and mediaserverd camera access)
- <https://developer.apple.com/forums/thread/776833> (cameracaptured on iOS 18.x)
- <https://gist.github.com/b1n4r1b01/8a79465e89de8a70a3cb12680a304b17> (AppleH10CamIn user-client notes; cited for existence only, contents not used because they are decompiler output)
- <https://github.com/apple-oss-distributions/xnu> (no camera driver in Apple open source)

This repository (`salami` c18f99d):

- `tools/run-ios-salami.py:73-76`, `:193-195`; `shim/fuxi-probe/probe.c:610`, `:978-979`
- `docs-lanes/sys-port.md:65-82`; `tools/mkadt-fuxi.py:141-145`
- `docs-lanes/cooling.md:73-85`; `tools/sweet-root-tweaks.py:20-21`
- `docs-lanes/thermal.md:314`; `docs-lanes/wlan-rails.md`; `docs-lanes/wlan-w20.md`; `docs-lanes/wifi-ios.md:202`; `docs-lanes/audio.md`
