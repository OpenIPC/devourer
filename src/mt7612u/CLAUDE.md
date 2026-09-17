
## Channel busy airtime (`GetChannelBusy`)

`Mt7612uRadio::GetChannelBusy()` implements the vendor-neutral
`IRadio::GetChannelBusy` from the MAC channel timers — `MT_CH_BUSY` (0x1134) /
`MT_CH_IDLE` (0x1130), armed by `mt7612u_link_stats_start()` with
`TX|RX|NAV|EIFS_AS_BUSY`, exactly mt76's `mt76x02_mac_cc_reset()`
configuration. Because TX counts as busy, a transmitting radio measures its own
airtime too — read it in a quiet window.

It goes through `mt7612u_ch_time()`, **not** `mt7612u_link_stats()`, and that
is the point: `link_stats` also reads `MT_RX_STAT_1`, whose false-CCA field is
read-and-clear and owned by `mt7612u_phy_tick()`'s AGC loop. A second reader at
caller cadence would both misreport the figure and starve the gain tracking.
For the same reason `ChannelBusy::energy_pct` is left **invalid** here — the
only candidate counter has an owner. Reads are checked (`mt_rr_chk`): `mt_rr`
returns `~0u` on a failed transfer, which would surface as a 100%-busy channel.

**Not validated on hardware** — no MT7612U was available. `busy_airtime_ok` is
true, `busy_airtime_measured` is false. Two things to measure when one is on
the bench: that busy/idle track real occupancy beyond repetition noise, and
that polling at dwell cadence does not disturb `phy_tick`'s gain tracking
(frame rate against a steady peer with and without the poller, the methodology
behind the tick's 5415-5470 fps figure).
