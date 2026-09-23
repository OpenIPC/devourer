
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
refuse unless `_rx_active`. Second, bring-up does not reliably complete while
the channel is already saturated (the arm then never becomes available, and the
MCU times out), so in a two-adapter test bring this one up BEFORE the
interferer.

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
