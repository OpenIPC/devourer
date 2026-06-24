# Jaguar3 (RTL8822CU / RTL8812EU / RTL8822EU) bring-up runbook

Status: **M0 scaffold landed; M1+ require hardware.** This document is the
working plan for adding the Realtek "Jaguar3" 802.11ac USB family to devourer.
The motivation is FPV **narrowband (5/10 MHz) TX**, which is achievable on
Jaguar3 but not on the Jaguar1 chips devourer already supports (Jaguar1's
baseband has no clock divider; Jaguar3's does — regs `0x9b0`/`0x9b4`).

## Why a whole new family (not a branch in the existing code)

Jaguar3 differs from Jaguar1 (8812AU/8814AU/8821AU) at every layer:

- **Baseband:** new IP generation, `0x9bx` register map (vs Jaguar1 `0x8xx`).
- **MAC:** new multi-port design.
- **TX/RX descriptor:** different field map (`SET_TX_DESC_*_8822C`), not the
  Jaguar1 `SET_TX_DESC_*_8812` 40-byte layout.
- **Bring-up:** routed through Realtek's **HalMAC** library (firmware download,
  power-on, queue/page init) — devourer has none of it.
- **phydm:** newer generation; channel/bandwidth is **procedural**
  (`config_phydm_switch_channel_8822c` / `..._switch_bandwidth_8822c`) rather
  than the Jaguar1 `phy_PostSetBwMode8812` register writes.

So Jaguar3 is a second device implementation behind the shared `IRtlDevice`
interface, reusing only the thin generic substrate.

## Architecture

- `src/IRtlDevice.h` — the device contract (`Init` / `InitWrite` /
  `SetMonitorChannel` / `SetTxPower` / `send_packet` / `GetSelectedChannel`).
- `RtlJaguarDevice` (Jaguar1) and `RtlJaguar3Device` (Jaguar3, `src/jaguar3/`)
  both implement it.
- `WiFiDriver::CreateRtlDevice` dispatches on chip family.

**Reused as-is:** `RtlUsbAdapter` (vendor-control register I/O, bulk transfers,
EFUSE byte reads, endpoint discovery), `Radiotap.c` + `RadiotapBuilder`.

**NOT reused — corrected during M1:** `src/PhyTableLoader` walks the Jaguar1
`check_positive` encoding (opcodes `0x8/0x9/0xA/0xB`, with `0x80000000` /
`0x40000000` condition halves). The Jaguar3 halhwimg tables use a *different*,
newer **"halbb"** encoding (a `>>28 == 0xf` headline section selected by
{cut, rfe} priority, then `PARA_IF/ELSE_IF/ELSE/END/CHK` body opcodes). So
Jaguar3 gets its own walker, `src/jaguar3/PhyTableLoader8822c`, a faithful port
of `odm_read_and_config_mp_8822c_*`. Also: there is **no phydm MAC table** on
Jaguar3 — MAC register init is done by HalMAC.

**New (`src/jaguar3/`):** `RtlJaguar3Device` (orchestrator), `Hal8822c`
(power/queue/tables), `Halmac8822cFw` (verbatim HalMAC DLFW port),
`RadioManagement8822c` (channel/BW/power + narrowband recipe), `FrameParser8822c`
(8822C descriptors). Plus `tools/extract_8822c_phy_tables.py` (BB/AGC/RF tables).

### Chip-family detection (important caveat)

PID matching is reliable only for RTL8822CU (`0xC82C` / `0xC82E` / `0xC812`).
The **8812EU/8822EU defaults are ambiguous and an 8812EU can enumerate as
`0x8812` — the same PID as the Jaguar1 RTL8812AU.** Distinguishing those needs
the SYS_CFG / HalVerDef chip-id read (**M1**). For now, force the family with
`DEVOURER_FAMILY=jaguar3` (or `jaguar1`).

## Milestones

| # | Goal | Needs HW | Gate / how to verify |
|---|------|----------|----------------------|
| M0 | Scaffold: interface, factory dispatch, skeleton modules, this runbook | no | `cmake --build` green; Jaguar3 stub throws "not yet implemented (Mn)" |
| M1 | **DONE (table pipeline).** Vendor `rtl8822c` BB+RF sources; `extract_8822c_phy_tables.py` → `Hal8822c_PhyTables.{c,h}` (agc_tab/phy_reg/phy_reg_pg/radioa/radiob); halbb walker `PhyTableLoader8822c`; `Hal8822c::apply_bb_rf_agc_tables` applies BB+AGC | no | build green; walker unit-checked vs real tables (phy_reg→1289 writes, first `0x1d0c=0x00410000`; agc→450) |
| M2 | **Chip-version decode DONE** (`Hal8822c::read_chip_version`, port of rtl8822c_ops.c: SYS_CFG1 0xF0 → cut/vendor/RF/test → `_phy_ctx.cut_version`; bit-math unit-tested). NIC fw blob **vendored** (`hal/hal8822c_fw.c`, 200624B) + `download_default_firmware`. **Remaining (HW):** hand-rolled power-on (`Hal8822cPwrSeq`), EFUSE read → `_phy_ctx.rfe_type`, RF-table apply via `odm_set_rf_reg` | partial (decode+blob done) | MAC alive; register canaries match kernel `rtl88x2eu` via usbmon diff |
| M3 | **HalMAC DLFW** state machine ported end-to-end (`Halmac8822cFw` + `Halmac8822cRegs.h`): download_firmware/start_dlfw/dlfw_to_mem/iddma/check_chksum/dlfw_end_flow/wlan_cpu_en/pltfm_reset, **plus `send_fw_page`** (dl_rsvd_page bracket + 8822C TX desc + checksum + bulk-OUT + download-OK poll). Header-parse VALIDATED vs real NIC fw (200624B → exact). **Remaining:** vendor the fw blob + M2-supplied `_rsvd_boundary`/HIQ-EP | code complete (parse validated) | on HW: FW_INIT_RDY (`REG_MCUFW_CTRL==0xC078`) — the hard gate (cf. 8814 3081 boot) |
| M4 | BB/RF/AGC apply + 20 MHz channel set → **RX first** | yes | first RX frame decoded |
| M5 | **TX path DONE (sw).** `FrameParser8822c.h`: 48-byte TX + 24-byte RX field macros + `cal_txdesc_chksum_8822c` + `fill_data_tx_desc_8822c`; `RtlJaguar3Device::send_packet` parses radiotap (legacy/HT/VHT, mirrors Jaguar1) → desc → bulk-OUT. Unit-tested (14 TX fields + checksum; RX parse). **Remaining:** on-air (QSEL/endpoint confirm) | code complete (unit-tested) | sniffer/SDR sees the frame |
| M6 | **Narrowband** `CHANNEL_WIDTH_5/10` via `0x9b0`/`0x9b4` | yes + SDR | SDR shows ~½ (10 MHz) / ~¼ (5 MHz) occupied BW |
| M7 | regress.py integration, docs, PR | — | — |

## Narrowband recipe (M6 payoff — already known)

From `config_phydm_switch_bandwidth_8822c` (Jaguar3). Set as a channel/PHY
state; the radiotap header stays 20 MHz, so **only an SDR confirms it**.

| field | reg | 5 MHz | 10 MHz | 20 MHz |
|-------|-----|-------|--------|--------|
| small BW | `0x9b0[7:6]` | 0x1 | 0x2 | 0x0 |
| DAC clk | `0x9b4[10:8]` | 0x4 (120M) | 0x6 (240M) | 0x7 (480M) |
| ADC clk | `0x9b4[22:20]` | 0x4 (40M) | 0x5 (80M) | 0x6 (160M) |
| RX DFIR | `0x810[13:4]` | 0x2ab | 0x2ab | 0x19b |

Constants live in `src/jaguar3/RadioManagement8822c.h`. Caveats: 5 MHz has known
DAC mirror/leakage (10 MHz is the reliable target); TX power can't be changed
while in narrowband (set at 20 MHz, then switch width).

## Bring-up method (M2–M5)

Same approach that landed the 8814: capture the kernel `rtl88x2eu` driver doing
the same operation with `usbmon`, capture devourer, diff the vendor-control
register writes, and converge. RX before TX. Validate narrowband on the SDR (the
regress.py sniffer/radiotap matrix cannot see it — RX reports 20 MHz).

## Reference drivers

`libc0607/OpenHD rtl88x2eu` and `rtl88x2cu` — `hal/rtl8822c|rtl8822e/*`,
`hal/phydm/rtl8822c/phydm_hal_api8822c.c`, `hal/halmac/halmac_88xx/*`,
`hal/hal_halmac.c`.
