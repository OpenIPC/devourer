# reference/ — vendor kernel drivers (git submodules)

devourer is a userspace re-implementation of Realtek's out-of-tree Wi-Fi
drivers. The vendor kernel modules live here as **git submodules** so every
developer works against the same fork + commit devourer treats as ground-truth,
without committing driver source into devourer's own history.

## Fetching

```sh
git submodule update --init --recursive        # all
git submodule update --init reference/rtl8812au # just one
```

The submodules are marked `shallow` — inits fetch only the pinned tip, not full
upstream history.

## Layout

| path | upstream | branch | chip / role |
|---|---|---|---|
| `rtl8812au`      | `josephnef/rtl8812au` (fork of OpenHD) | `devourer-kbuild` | Jaguar1 — 8812/8814/8821AU |
| `rtl88x2bu`      | `josephnef/rtl88x2bu` (fork of OpenHD) | `devourer-kbuild` | Jaguar2 — 8822BU |
| `rtl88x2cu`      | `josephnef/rtl88x2cu` (fork of OpenHD) | `devourer-kbuild` | Jaguar3 — 8822CU |
| `rtl88x2eu`      | `josephnef/rtl88x2eu` (fork of OpenHD) | `devourer-kbuild` | Jaguar3 — 8822EU (maintained mainline) |
| `8821cu`         | `morrownr/8821cu-20210916` | `main` | Jaguar2 — 8811CU/8821CU |
| `rtl88x2eu-5mhz` | `libc0607/rtl88x2eu-20230815` | `5mhz_bw` | Jaguar3 — 8822EU 5 MHz narrowband reference |
| `rtl8852bu`      | `josephnef/rtl8852bu` (fork of morrownr/rtl8852bu-20250826) | default | Kestrel — 8852BU/8832BU (Wi-Fi 6) |
| `rtl8852cu`      | `josephnef/rtl8852cu` (fork of morrownr/rtl8852cu-20251113) | default | Kestrel — 8852CU/8832CU (Wi-Fi 6) |

## Why these forks

- **OpenHD lineage for AU/BU/CU/EU.** OpenHD is the FPV/wifibroadcast fork with
  runtime TX-power-index control and raw injection tuning — the same behaviour
  devourer mirrors. (aircrack-ng targets pen-testing; morrownr targets
  managed/AP client mode without injection.)
- **`josephnef/*` forks on a `devourer-kbuild` branch.** The four OpenHD drivers
  need small kernel-6.x kbuild-compat patches to build their `.ko` on modern
  host/VM kernels (`EXTRA_CFLAGS` → `ccflags-y`, `from_timer` →
  `timer_container_of`, …). Those patches are committed on the branch so the
  reference `.ko` builds identically for everyone. The branch tip is the pinned
  upstream commit plus that one patch commit; no other divergence.
- **morrownr for 8821cu.** The canonical monitor-capable 8811CU/8821CU driver;
  builds clean, tracked directly at upstream.
- **morrownr drops (via `josephnef/*` forks) for the Kestrel pair.** The Wi-Fi 6
  generation's ground truth is Realtek's unified "G6 phl" vendor codebase
  (`core/` → `phl/` → `phl/hal_g6/` with `mac/mac_ax` fw/H2C plane, `phy/bb`
  halbb, `phy/rf` halrf); each public drop ships exactly one chip's HAL + FW
  payload, hence two submodules. morrownr's date-named repos carry the newest
  Realtek drops (v1.19.21 for 8852B, v1.19.22 for 8852C) with only
  kernel-compat/installer changes on top — and the v1.19 line is the one whose
  fwcmd surface includes the TWT-OFDMA + F2P trigger-frame H2C that issue #236
  needs (mainline `rtw89` does not carry it; the 2021 lwfinger v1.15 8852A drop
  has only basic TWT). Forked for pinning; a `devourer-kbuild` branch gets
  added only if the regress kernel cells need build patches.
- **libc0607 `5mhz_bw` for `rtl88x2eu-5mhz`.** The only fork carrying the
  working 5 MHz DAC-clock divider write (`R_0x9b4`) that devourer ported for
  Jaguar3 narrowband. It is the on-air ground-truth for
  `tests/jaguar3_eu_kernel_5mhz_sdr.sh` and `tests/jaguar3_eu_5mhz_mirror_ab.sh`.
  Kept as a distinct submodule alongside the maintained `rtl88x2eu` because
  OpenHD's mainline does not carry the narrowband branch.

## Consumers

- `tools/extract_*.py` read PHY / firmware / txpwr tables out of these trees to
  generate `hal/`.
- `tests/bench_init.py` and the `tests/*.sh` kernel-baseline scripts build and
  insmod the vendor `.ko` for devourer-vs-kernel regression cells.
- `tests/8821cu_hwtest.sh` binds `reference/8821cu/8821cu.ko` as a functional
  RX proof.
