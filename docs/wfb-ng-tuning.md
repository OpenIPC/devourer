# wfb-ng efficient configuration & on-air TX throughput

This documents (a) the most efficient wfb-ng configuration for the RTL8812AU and
(b) an SDR-measured on-air TX throughput comparison between **devourer**
(userspace libusb) and **wfb-ng** (kernel `svpcom/rtl8812au` driver).

## Results

On-air channel occupancy measured with a USRP B210 (`tests/sdr_duty.py`) on a
clean 5 GHz channel (ch149), 1500 B frames. `on_air_Mbps = duty × PHY_rate`:

| Config | devourer | wfb-ng (svpcom + `wfb_tx`) |
| ------ | -------- | -------------------------- |
| MCS1 / 20 MHz | 94.6 % duty → 12.3 Mbps | 94.5 % duty → 12.3 Mbps |
| MCS7 / 20 MHz | 80.1 % duty → 52.1 Mbps | 79.8 % duty → 51.9 Mbps |
| MCS7 / 40 MHz | 62.7 % duty → ~85 Mbps  | — |

devourer and wfb-ng deliver the same on-air injection throughput. wfb-ng's
*useful* goodput is then × the FEC ratio `k/n = 8/12 ≈ 0.67`, so MCS1/20 ≈ 8 Mbps
and MCS7/20 ≈ 35 Mbps — consistent with wfb-ng's ~7 Mbps default and OpenIPC's
~52 Mbps-total / 36 Mbps-video real-world figures.

Two regimes are visible: at low MCS the link is **airtime-limited** (≈95 % duty,
the channel is nearly saturated); at high MCS the **host feed** becomes the limit
(duty drops to ~80 % at 20 MHz, ~63 % at 40 MHz) while absolute throughput keeps
rising. Larger frames (up to the 3993 B max payload) raise duty at high MCS by
amortising per-frame overhead.

**Bare-metal vs VM**: the same svpcom driver + injector, run bare-metal and inside
the libvirt VM via qemu-xhci USB passthrough, give identical occupancy
(80.5 % vs 80.4 %). USB passthrough adds no throughput cost here — the limit is
airtime / chip TX, not the USB transport.

## Most efficient wfb-ng config (RTL8812AU)

- **Driver: `github.com/svpcom/rtl8812au`** (module `88XXau_wfb`,
  `sudo ./dkms-install.sh`). It is the wfb-ng injection-tuned driver. Set
  `rtw_tx_pwr_idx_override` 30–45 (≤63; higher needs active cooling). The
  in-tree rtw88 driver's monitor injection is much slower (~6 Mbps) — use svpcom
  for wfb-ng. It builds on modern host kernels (6.18 here) as well as the pinned
  5.15.
- **Throughput levers** (`/etc/wifibroadcast.cfg`, or `wfb_tx -M/-B/-G/-S/-L`):
  - `mcs_index` — the primary lever (MCS1 ≈ 7 Mbps → MCS5–7 + 40 MHz ≈ 36–52 Mbps).
  - `bandwidth = 40` — ~doubles capacity.
  - `short_gi = True` — ~+11 %.
  - `ldpc = 1` — RTL8812AU supports it; better FEC robustness.
  - `stbc = 1` — TX diversity on dual-antenna cards.
- **Channel**: a clean 5 GHz channel (ch149/165). 2.4 GHz is congested, so
  mac80211 CSMA backoff sharply lowers injection rate.
- **FEC**: `fec_k = 8`, `fec_n = 12` (33 % overhead) is the common default.
- **MTU**: `radio_mtu` / `MAX_PAYLOAD_SIZE = 3993` is wfb-ng's max single-frame
  payload.

## Measuring on-air throughput

Counting frames at a Wi-Fi monitor sniffer caps around ~2900 fps, so it
undercounts a fast transmitter. `tests/sdr_duty.py` measures the fraction of time
the (clean) channel's received power is above the idle noise floor = the
transmitter's airtime occupancy (duty cycle), which has no such ceiling:
`on_air_Mbps ≈ duty × PHY_rate(MCS, BW, GI)`. Calibrate the idle noise floor once
(`--noise-db`, ≈ −62 dB here) — a percentile auto-floor mis-reads once the channel
is ~saturated because the low tail becomes signal.

## Reproduce

```sh
# kernel side: build + load the wfb-ng driver, build wfb_tx
git clone https://github.com/svpcom/rtl8812au && cd rtl8812au && make && \
  sudo insmod 88XXau_wfb.ko rtw_tx_pwr_idx_override=30
git clone https://github.com/svpcom/wfb-ng && cd wfb-ng && make
# devourer side: build/WiFiDriverTxDemo with DEVOURER_TX_RATE=MCS7/20 + DEVOURER_TX_PAYLOAD_BYTES + DEVOURER_TX_GAP_US=0
# measure (ceiling-free) while either side floods a clean 5 GHz channel:
sudo python3 tests/sdr_duty.py --freq 5745e6 --secs 4 --mcs 7 --bw 20 --noise-db -62
```

Hardware here: RTL8812AU `0bda:8812`, USRP B210 `2500:0020`, libvirt VM
`devourer-testrig` (kernel 5.15) for the passthrough comparison.
