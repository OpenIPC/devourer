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
  MT7612U the clear is trivially true, since the arm wrote nothing.

On MT7612U the arm writes no register: it verifies that the station's address
is the adapter's own `MT_MAC_ADDR` and that the auto-responder is enabled
(`docs/mt7612u-station-identity.md`). That is why the station's address always
comes from `GetPermanentMacAddress`.

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
refused us".

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
- **On air** - `tests/mt7612u_sta_onair.sh` against hostapd in a network
  namespace, MT7612U as the station:

  | Cell | Scored |
  |---|---|
  | `open` | AP associates our address; ping 0% loss over the TAP; ledger plaintext only; armed; the clear ran on exit |
  | `wpa2` | four-way, group and pairwise rekeys at the AP; ping before and after; one association; MIC failures <= PTK installs; armed; the clear ran on exit; no `tx.retry_limit=0` warning |
  | `noarm` | control, `DEVOURER_STA_ARM=0`: no arm and no clear ran (link outcome reported, not scored) |
  | `retry0` | `DEVOURER_TX_RETRY_LIMIT=0`: the arm-time warning; the clear ran on exit (link outcome reported, not scored) |

  The clear's result is printed as information, not scored: on MT7612U
  `ClearStationIdentity` is trivially true.

  Exit 0 pass, 1 fail (including a station fault, exit 3, with its cause
  named), 2 inconclusive (rig refused, AP not up, route not through the TAP,
  the station exited or stalled before `sta_client up:`, station out of
  time), 3 interrupted. `FW_DIR` must hold the decompressed MT7612U blobs. The AP's phy must be able to change
  network namespace (`iw phy <phy> info` lists `set_wiphy_netns`): an
  in-kernel cfg80211 driver such as rtw88 or mt76. Out-of-tree drivers such
  as rtl88x2cu / 88x2bu cannot, and the cell refuses them.

## What it does not do

- The on-air cell takes an MT7612U only (`sta_dut_take`) until a generic
  DUT take / hand-back exists. The client itself arms a Realtek 8822C /
  8822B selected with `DEVOURER_VID` / `DEVOURER_PID`; that path is not
  covered by an on-air cell here.
- Software CCMP only; no PMF/802.11w, WPA2-PSK/CCMP or open only.
- No fragment reassembly and no A-MSDU: both are refused and counted.
- One BSS at a time, chosen by SSID; no roaming and no background scan while
  associated (a retune would lose the association).
- A pairwise rekey can cost one received frame (802.11-2016 12.7.6.5); the
  note is at the `ccmp_decrypt` call in `rx_frame()`.
- The host stack owns ARP, IP and DHCP on the TAP.
