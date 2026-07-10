# PCIe hardware-timestamping / PTP prototypes (RTL8821CE)

Reference tooling behind the PCIe timing numbers in
[`docs/timing-accuracy.md`](../../docs/timing-accuracy.md). These are kernel-side
experiments on the Radxa X4's onboard RTL8821CE (`rtw88`), run out-of-band — they
are **not** part of the devourer library build.

## Contents

- `rtl8821ce_phc.c` + `Makefile` — a passive PTP hardware clock (`/dev/ptpN`)
  backed by the 8821CE's 802.11 MAC TSF, read over BAR2 MMIO. It does **not** bind
  the PCI device: `rtw88` stays bound and keeps the chip alive, and the module
  only `ioremap`s the BAR and reads the free-running TSF, exposed as a
  disciplinable clock via `timecounter`/`cyclecounter`.
- `validate.sh` — builds, loads, and exercises the PHC with `phc_ctl` / `phc2sys`.
- `pcie_phc_lat.c` — measures the PHC gettime read window
  (`PTP_SYS_OFFSET_EXTENDED`): the PCIe MMIO floor (~3.7 µs), versus tens of µs
  for the equivalent read over USB.
- `ptp_crosscheck.sh` — runs `ptp4l` on the board's Intel I226 Ethernet (a real
  IEEE-1588 NIC) and cross-compares the Wi-Fi TSF PHC against it on one machine.
- `rtw88_rx_hwtstamp.patch` — a one-hunk `rtw88` patch attaching the RX-descriptor
  TSF to `skb_hwtstamps` before `ieee80211_rx_napi`, so a `SO_TIMESTAMPING` socket
  reads the 802.11 hardware RX timestamp on the PCIe / `mac80211` path.

## Requirements

Kernel headers, `linuxptp`, and `ethtool` on the target. The MAC TSF is
power-gated while the card is idle — bring the interface up in monitor mode (or
associate) so the TSF is clocked before loading the PHC.

## Scope

These explore hardware-RX timestamping and a real PHC on the PCIe part. Two-way
`ptp4l` over the Wi-Fi radio remains blocked on a host-reported TX-egress
timestamp (a firmware limitation); the board's I226 Ethernet is the real-PTP
reference. See `docs/timing-accuracy.md` for the measured results and that limit.
