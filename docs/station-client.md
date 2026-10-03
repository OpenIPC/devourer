# The station client (`tests/sta_client.cpp`)

The in-tree caller of the station core (`docs/station-core.md`) and of
`IRadio::SetStationIdentity`. It joins a WPA2-PSK or open BSS through any
`IRadio`: scan, authenticate, associate, run the four-way as the supplicant,
and carry CCMP-protected traffic to and from the host through a TAP device.
The protocol is `src/sta/`; this file owns what the core leaves to its
integrator - the scanner, the re-join policy and the data plane.

## The station identity

- Armed only when `AdapterCaps::station_mode_ok` is true: MT7612U, and the
  Realtek 8822C / 8822B arm (`docs/realtek-station-arm.md`). A backend that
  reports false is refused at start-up with exit status 2;
  `DEVOURER_STA_ARM=0` runs it unarmed instead.
- Armed for the BSSID actually joined, after `StartRxLoop` (IRadio's ordering
  rule) and outside the mutex the RX callback takes (IRadio's lock rule).
- Cleared on the way out whenever an arm was attempted; the result is printed
  (`station identity clear: restored (verified)` / `NOT VERIFIED`). On
  MT7612U the clear puts the monitor receive filter back and is true once it
  reads back.

On MT7612U the arm writes no identity register: it verifies that the
station's address is the adapter's own `MT_MAC_ADDR` and that the
auto-responder is enabled (`docs/mt7612u-station-identity.md`). That is why
the station's address always comes from `GetPermanentMacAddress`. The one
register it writes is the receive filter: armed, the station runs the managed
filter `0x00015f97`, which drops unicast not addressed to it but keeps every
BSS's beacons and group traffic; unarmed (`DEVOURER_STA_ARM=0`) it stays
promiscuous. `StationSm::on_rx` is the address filter either way; its ledger
`not-for-us` count (our BSS, someone else's unicast) is the witness that the
managed filter is on.

## What the station transmits

The arm covers receive and acknowledgement only. Unicast is sent with an
ACK-requesting radiotap (`DEVOURER_STA_ACK=0` turns that off), and
`tx.retry_limit` defaults to `kStationRetryLimit` (7) unless the library took
a numeric `DEVOURER_TX_RETRY_LIMIT` (an empty or non-numeric value is not
one; `apply_station_retry_limit`). `DEVOURER_TX_RETRY_LIMIT=0` therefore asks
for a single-shot uplink, and the library warns about it at arm time
(`Mt7612uRadio::SetStationIdentity`; `docs/mt7612u-tx-retry.md`).

## Running it

```
sudo DEVOURER_VID=0x0e8d DEVOURER_PID=0x7612 DEVOURER_CHANNEL=6 \
     DEVOURER_STA_SSID=devourerSTA DEVOURER_STA_PSK=devourer123 \
     DEVOURER_STA_TAP=dvsta0 build/sta_client 60
```

Built by the `StaClientSelftest` CMake target (Linux, OpenSSL). Station
variables: `DEVOURER_STA_SSID`, `_PSK` (empty: open), `_TAP`,
`_SCAN_CHANNELS`, `_SCAN_DWELL_MS`, `_RECONNECT`, `_BACKOFF_MS`, `_ARM`,
`_ACK`; plus the library's `DEVOURER_*` (`examples/common/env_config.cpp`).
SIGINT/SIGTERM (handled from the start of `main`, so a stop during bring-up
ends the run once the bring-up returns) leave the BSS, clear the identity and
print the ledger. The ledger is printed at every exit once `sta_client up:`
has printed, and separates "heard nothing", "heard another BSS" and "our AP
refused us". Its first line is the state the run ENDED in, before the
teardown's leave: `Connected`, or `Failed reason=<why>` for a run that gave
up. While it runs, the station also logs each association
(`station connected (association N)`) and each failure (`station link lost:
<reason>` or `station join failed: <reason>`).

Re-join policy: after a lost link or a failed join the station waits
`DEVOURER_STA_BACKOFF_MS` and joins again, for as long as the run lasts.
With `DEVOURER_STA_RECONNECT=0` the first failure - a lost link, or a first
join that fails - ends the attempts: the station logs it, stays
unassociated until its time is up, and the ledger ends `Failed` with the
reason.

The moment an association response is accepted - open or WPA2 - the
station sends one probe request (the "nudge", counted as `nudges` in the
ledger): see "AP quirk" below. On WPA2 a second one follows if no EAPOL has
arrived 1 s later; the four-way timeout re-joins if even that is not
enough.

An open association is confirmed by the AP's first unicast frame to the
station. A station cannot see the AP's side: if the AP never saw the
association response acknowledged, it does not hold the station, drops its
traffic and may never say so. So once the host has asked something - three
frames whose answer the AP must forward back: an ARP request, an ICMP /
ICMPv6 echo request, a unicast IPv6 neighbour solicitation, TCP, a DNS
query - and no unicast reply has come within 5 s of the first, the link is
lost as `unconfirmed` (`StationSm::link_lost`) and re-joined under the
policy above. One-way traffic (a UDP video or telemetry uplink), multicast
chatter, gratuitous and probe ARPs and an idle host are never judged; an
unheld association under one-way traffic alone is found when the host's
stack next asks something (its neighbour re-verification is a unicast ARP
request). WPA2 needs no such rule (the four-way is the confirmation). The
ledger counts
these (`unconfirmed=`), and repeated association responses
(`assoc_repeat=`).

Exit status: 0 the run completed; 1 setup failed; 2 refused
(`station_mode_ok` false, or the duration, `DEVOURER_CHANNEL`,
`DEVOURER_STA_SCAN_DWELL_MS` (10..10000, default 250) or
`DEVOURER_STA_BACKOFF_MS` (0..60000, default 1000) is not a valid number in
range); 3 a fault - an exception was caught, the TAP failed mid-run, the RNG
failed, or `ClearStationIdentity` could not verify its rollback. A fault still leaves, clears and prints the ledger, whose
first line then reads `fault=1`.

## What the tests pin

- **Headless** - ctest `sta_client_headless` (`build/sta_client --self-test`,
  `tests/sta_client_selftest.inc`): the cells play the authenticator and feed
  real frames into the real receive path - scan selection and sweep, re-join
  policy, key selection by key id, replay and duplicate windows, PTK/GTK
  rekeys, plaintext/fragment/A-MSDU refusal, the FCS trim, the ledger's
  identities. No device, no root.
- **On air** - `tests/sta_client_onair.sh` against hostapd in a network
  namespace. The station (DUT) is an MT7612U, an RTL8812CU (8822C) or an
  RTL8812BU (8822B); the AP is any adapter whose in-kernel driver supports AP
  mode and can change network namespace (`iw phy <phy> info` lists
  `set_wiphy_netns`: mt76, rtw88 - not the out-of-tree rtl88x2cu / 88x2bu,
  which the cell refuses).

  | Cell | Scored |
  |---|---|
  | `open` | with a ping running from the start, the AP associates our address within 30 s (recovering an unconfirmed first association counts); ping 0% loss over the TAP; ledger plaintext only; armed; the clear |
  | `wpa2` | four-way, group and pairwise rekeys at the AP; ping before and after; one association; no four-way MIC failure, data-plane MIC failures <= PTK installs; armed; the clear; no `tx.retry_limit=0` warning. MT7612U: the managed filter - plaintext unicast injected from the AP's BSSID at the station (`plaintext refused` at least half of it, else INCONCLUSIVE) and at a foreign address (`not-for-us` under 1% of it - a PASS counts only once the `noarm` control of the same run has seen that stream arrive, else INCONCLUSIVE) |
  | `noarm` | control, `DEVOURER_STA_ARM=0`: no arm and no clear ran. Realtek: the station tried and the AP did NOT complete the four-way - a completed one FAILs; INCONCLUSIVE unless the armed `wpa2` cell of the same run got in (the positive control). MT7612U: under the monitor filter both injected streams arrive (each at least half); the link over a 30 s ping window is reported, not scored |
  | `retry0` | `DEVOURER_TX_RETRY_LIMIT=0`: the arm-time warning; the clear (the link over a 30 s ping window is reported, not scored) |
  | `reconnect` | hostapd stopped and restarted: the station reports the lost link; second four-way within the bound, measured from hostapd being started again; ping 0% loss over a 30 s window; ledger 2 associations, 1 reconnect; one arm across the re-join; the clear |
  | `noreconnect` | as `reconnect` with `DEVOURER_STA_RECONNECT=0`: the lost link reported; no re-join; the ledger ends Failed after 1 association |

  The arm differs by die. On MT7612U it writes only the receive filter, so
  an unarmed link may work and `noarm` is the filter's control. On a Realtek
  die the arm writes the port registers and unarmed the MAC does not
  acknowledge own-addressed unicast (`docs/realtek-station-arm.md`), so
  `noarm` is the arm's control and can fail. On both, the clear must verify.

  The injections (`INJECT_S`, `INJECT_PPS` each, `FOREIGN`) ride a monitor
  vif on the AP's phy (`tests/sta_unicast_inject.py`) and run only for an
  MT7612U DUT; a phy that cannot add one makes the filter check
  INCONCLUSIVE, not the cell.

  The arm is per BSSID: a re-join to the same BSSID keeps it rather than
  arming again, and on Realtek the second association is the proof it still
  holds.

  Exit 0 pass, 1 fail (including a station fault, exit 3, with its cause
  named), 2 inconclusive (rig refused, AP not up, route not through the TAP,
  the station exited or stalled before `sta_client up:`, station out of
  time), 3 interrupted. `FW_DIR` (an MT7612U DUT) must hold the decompressed
  MT7612U blobs.

## AP quirk: an MT7612U AP holds the association's TX status

An MT7612U running as the AP (kernel mt76x2u, hostapd) can hold a
transmitted frame's TX status until its NEXT transmission - the same
silicon behaviour `docs/mt7612u-tx-retry.md` records for this tree's own
MT7612U driver ("status posted only on the next TX"). hostapd acts on an
association only once the Association Response's status is in: it counts
the station associated - dropping its data until then - and on WPA2 starts
the four-way only from that status too, so both an open association and
the WPA2 key exchange stall the same way. In hostapd's debug log
(`HOSTAPD_DEBUG=1`):

- "association OK (aid 1)", the station added, the Association Response
  sent;
- no TX status for it for six seconds, while the station - which had
  received the response and acknowledged it - believed it was associated;
- then the station's next frame made the AP transmit, and the status
  arrived with `ack=1`: too late, the station had already given up on the
  association ("handle_assoc_cb: STA ... not found").

The station is not at fault: it acknowledged the response, at 1 Mb/s CCK,
within ~0.3 ms in captures. The stall is intermittent and depends on
whether anything else makes the AP transmit soon after the response, which
is why a given station can pass several runs and then fail several in a
row; on WPA2 it shows as the four-way never starting, each attempt ending
in the station's handshake timeout. sta_client sends its nudge for exactly
this, for open and WPA2 alike; the confirmation rule (open) and the
four-way timeout (WPA2) re-join if the association still never becomes the
AP's.

## What it does not do

- The on-air cell covers the dies that report `station_mode_ok` (MT7612U,
  8822C, 8822B). Another Realtek die can be named with `DUT_VID` / `DUT_PID`;
  `sta_client` refuses it (exit 2) unless it reports `station_mode_ok`.
- Software CCMP only; no PMF/802.11w, WPA2-PSK/CCMP or open only.
- No fragment reassembly and no A-MSDU: both are refused and counted.
- One BSS at a time, chosen by SSID; no roaming and no background scan while
  associated (a retune would lose the association).
- A pairwise rekey can cost one received frame (802.11-2016 12.7.6.5); the
  note is at the `ccmp_decrypt` call in `rx_frame()`.
- The host stack owns ARP, IP and DHCP on the TAP.
