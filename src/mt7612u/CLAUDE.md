
## Channel busy airtime (`GetChannelBusy`)

`Mt7612uRadio::GetChannelBusy()` implements the vendor-neutral
`IRadio::GetChannelBusy` from the MAC channel timers, via `mt7612u_ch_time()`.
Register pair, arming, read-and-clear and refusal semantics are documented at
that function's declaration (`include/mt7612u/mt7612u.h`) — the one place they
can be kept true.

The subtree-specific facts: the arming configuration is mt76's
`mt76x02_mac_cc_reset()` exactly, which counts TX as busy, so a transmitting
radio measures its own airtime and must read inside a quiet window. Timers are
armed on both the RX and the transmit-only path and re-armed on every live
retune — but arming is not the same as counting: with the receiver down the
idle timer advances and the busy one does not, so a transmit-only session gets
NO reading rather than the plausible 0% it used to return. `ChannelBusy::energy_pct` is left invalid here because the only
candidate counter, `MT_RX_STAT_1`'s false-CCA field, is owned by
`mt7612u_phy_tick()`'s AGC loop.

**Validated on hardware** (`docs/rx-spectrum-sensing.md`): 64.0-64.3% under a
flooder two Realtek generations put at 61-63%, 0.0% on a quiet channel, and
8.1-9.5% per second on a ~9%-duty bursty interferer — the arm the Realtek 2 ms
sample fails. `busy_airtime_measured` is true.

Two facts that measurement added. First, the reading **requires a running
receiver**: with RX down the idle timer still advances, so busy+idle is
non-zero and the ratio is a perfectly plausible 0% — a fabricated zero that a
channel ranker reads as "emptiest". `GetChannelBusy` and `ArmChannelBusy` both
refuse unless `_rx_active`. Second, the adapter can be brought up while an
interferer already runs (one peer measured): "MCU replies and EP 4" below.

`ArmChannelBusy` here resets the timers and the interval mark through
`mt7612u_ch_time_arm` — NOT `mt7612u_link_stats_start`, which would also clear
the MIB block and the link-stats interval that the 1 Hz tick owns, once per
dwell — so the window is exactly the caller's arm-to-read
gap and is reported that way in `window_us`. The mark is stamped at arm rather
than zeroed — zeroing made the first read after an arm report `window_us=0`,
i.e. a percentage with no denominator.

Polling at dwell cadence costs the receiver nothing measurable at survey
load: `rxdemo` with `DEVOURER_RX_BUSY_MS=100` (arm / wait / read, the survey
executor's dwell shape) against an RTL8822BU airing 1400-byte HT-MCS7 at its
~700 fps ceiling on ch6, four arms × 2 reps × 30 s — tick+poll 14841 / 17534
frames, tick alone 17417 / 17531, no tick + poll 18211 / 15097, neither 17810 /
14201 — all inside the run-to-run spread, with the poller reading ~80% busy
throughout (317 windows per run, none spoiled). The no-tick control is what
limits the claim: at ~700 fps it does NOT collapse, so the regime the tick's
own benchmark is about (a SuperSpeed peer at ~3000 fps, where no tick means 3
frames in 10 s) was never entered, and whether the poll disturbs gain tracking
*there* is still unmeasured. Nothing on this bench injects that fast: the
8822BU's synchronous USB 2.0 send tops out near 700 fps at that size, the
8812CU at ~670, a Kestrel 8832CU on the same hub at ~330, and the 8821AU
submits 3500 fps host-side while an independent witness decodes none of it.
`DEVOURER_MT7612U_PHY_TICK=0` is the control arm's switch; harness shape in
the `mt7612u_phy_tick` doc comment.

## MCU replies and EP 4

Data parked in the RX bulk endpoint (EP 4) holds back the MCU's replies on
EP 5: with EP 4 undrained a command's reply does not arrive, and every reply of
the burst then appears within ~7 ms of the first EP 4 read. This is reachable
before any RX ring exists. During the RXDCOC calibration of a channel set
(`CMD_CALIBRATION_OP`, `MCU_CAL_RXDCOC`), received data reaches EP 4 while
`MT_MAC_SYS_CTRL` reads 0 (MAC RX and TX off) and the USB RX bulk enable
(`MT_USB_U3DMA_CFG` bit 22) is set; the one read decoded was a data frame from
the peer's address. What in the calibration lets it through is not
established. Undrained, RXDCOC and every command after it ran out the 3 s reply
budget, so a ~1 s bring-up took 27-34 s.

`mcu_wait_resp()` therefore drains EP 4 once per command, on its first 300 ms
timeout, and only while `mt7612u_dev::ep4_owner` says nobody else reads it (the
field's comment says why it lives on the device, not the ring). A reply on time
costs nothing; a late one costs one `mt_rx_flush()`. The ten-read reply budget
above it predates this and is probably the same symptom; it was not re-measured.
Measurements and their limits: `docs/mt7612u.md`, "Bring-up on a busy channel";
reproducer `tests/mt7612u_busy_bringup.sh`.

## TX retries

Where each piece lives: NOACK vs. ACK-requesting radiotap -
`build_stream_radiotap` in `src/RadiotapBuilder.h`; the retry-limit knob -
`tx.retry_limit` in `src/DeviceConfig.h`, applied at every bring-up by
`mt7612u_set_retry_limit` (`caps.cpp`); measurements -
`docs/mt7612u-tx-retry.md`; reproducer - `mt7612uprobe txs` (`gate_txs` in
`tools/bringup.cpp`).

## Station identity

Contract: `mt7612u_set_station_identity` (`include/mt7612u/mt7612u.h`) and
`IRadio::SetStationIdentity` (`src/IRadio.h`). Measurements, the receive-filter
decision and the retractions: `docs/mt7612u-station-identity.md`. Headless cell
`mt7612u_station_identity`; hardware gate `mt7612uprobe staid`.

The arm OWNS `MT_RX_FILTR_CFG`: it installs `MT_RX_FILTR_CFG_MANAGED`
(`0x00015f97`, `regs.h` spells the bits) and records what it replaced; the
clear and a port-identity drop put that back. While armed,
`mt7612u_set_monitor_rx()` only records its request
(`mt7612u_sta_rx_filter_request`, `StationIdentity.h`), so an RX loop
(re)started under a station does not knock it promiscuous. Anything new that
writes the filter must go through the same request, or it silently disarms
the station's receive half.
