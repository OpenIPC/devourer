# MT7612U as a fully-userspace access point

**Status: it works.** devourer's own AP harnesses run against this part
unmodified — beacon, probe, auth, assoc, and the ARP/ICMP data plane — with a
real Linux station associated. `StartBeacon`, `UpdateBeaconPayload` and
`StopBeacon` are implemented on `Mt7612uRadio`, so nothing in
`tests/ap_responder.cpp` or `tests/ap_wpa2.cpp` branches on the backend. (Both
carry one MediaTek-specific *comment* now, explaining why they silence the
beacon before `_exit`; no code depends on it.)

**WPA2-PSK works too.** `tests/ap_wpa2.cpp`, also unmodified, completes the
4-way handshake against a real `wpa_supplicant` station and carries encrypted
traffic. It needs the same five `IRadio` methods as the open-network harness -
`InitWrite`, `StartBeacon`, `StartRxLoop`, `send_packet`, `StopBeacon` - and no
others. CCMP is done in software in the harness, so no key API is involved;
that is a separate point from the method count.

What is NOT done: **hardware** CCMP. `MT_WCID_KEY` and `MT_SKEY` are untouched,
so the claim below that hardware crypto is a capability *gain* on this part
remains unmeasured — what is measured is that the software path devourer
already had works here. Key install is the one item of the original gap list
that is still open.

File:line references below are to the merged subtree (`src/mt7612u/`, all
`.cpp` since the C++ migration) and to `reference/mt76 @ be5ce79`. Some of the
citations in "The gap" section still name the pre-migration `.c` filenames and
pre-merge line numbers; they are kept because the reasoning is still correct,
but do not expect them to resolve.

## Verified on hardware (2026-09-08, MT7612U at USB 2-1)

Measured against a second MT7612U bound to the kernel `mt76x2u` driver as a
station, plus an RTL8812AU running `rxdemo` as an independent on-air witness.

| Claim | Evidence |
|---|---|
| Beacon reaches the air | Kernel station's `iw scan` lists `SSID: MT7612U-AP`, `beacon interval: 100 TUs`, `capability: ESS (0x0001)`, correct `DS Parameter set` and basic-rate flags — on ch149, ch36 **and** ch6 (both bands) |
| Independent radio decodes it | 8812AU witness: 177 frames on a channel measured empty beforehand, each `len:71` (our 67-byte MPDU + FCS) at `rate:4` (OFDM 6M), ~9.8/s |
| HW TSF timestamp (`FLAGS_TS`) | `wlan.fixed.timestamp` advances **102400 µs** per beacon — exactly 100 TU |
| HW sequence (`ACK_CTL_NSEQ`) | `wlan.seq` increments **+1 per beacon** (2140, 2141, 2142 …) |
| Beacon interval math | On-air spacing 102.4 ms, confirming `INTVAL = interval_tu << 4` (1/16 TU) |
| Corrected MBSS masks | `MT_MAC_BSSID_DW1` reads `0x003fa127` — upper bits exactly mt76's `MBSS_MODE=3 / MBEACON_N=7 / LOCAL_BIT` |
| Hardware auto-ACK (Gate B) | A real station's **3 auth frames, 0 retried**. An un-ACKed frame is retransmitted with FC Retry set, so retry=0 is the ACK |
| APC BSSID slot programmed | `MT_MAC_APC_BSSID_L(0)=0x50efa540` (device MAC `40:a5:ef:50:…`) |

Not yet done: probe **responses**, auth/assoc **responses** and the data plane —
those are the existing backend-agnostic C++ harnesses' job (Stages C–E), not
driver work.

## Verified through IRadio (2026-09-10) — devourer as the AP

The section above is the bring-up gates driving the C library directly. This
one is devourer itself: `tests/ap_responder.cpp`, unmodified, built against
`libdevourer.a` and pointed at an MT7612U. A second MT7612U on the kernel
`mt76x2u` driver is the station. ch36, `iw reg set SE`.

| Claim | Evidence |
|---|---|
| `StartBeacon` arms the MAC | `MT7612U beaconing every 100 TU`, then `ap_responder up on ch36 SSID devourerAP (beacon OK)` |
| The beacon is on air and correct | station `iw scan`: `SSID: devourerAP`, `BSS 02:42:75:05:d6:00`, `beacon interval: 100 TUs`, `capability: ESS (0x0001)`, `DS Parameter set: channel 36`, −32 dBm |
| A locally-administered BSSID works | that BSSID is `02:…`, so it lands in APC slot 1 by mt76's rule. The first draft of `mt7612u_beacon_start()` refused it outright |
| A real station associates | `wlx…: connected to 02:42:75:05:d6:00`, `freq: 5180.0` |
| The MAC auto-ACKs | AP side, three runs: `AUTH req … alg=0 seq=1 retry=0` and `ASSOC req … retry=0`. An un-ACKed frame is retransmitted with FC Retry set, so retry=0 IS the ACK |
| The data plane works | `6 packets transmitted, 6 received, 0% packet loss, rtt avg 0.808 ms`; AP side `data(arp/icmp)=8 responses_sent=16` |
| `StopBeacon` silences it | `tests/mt7612u_beacon_stop_check.cpp`: armed → SSID seen; stopped → gone; re-armed → seen again |
| WPA2-PSK 4-way completes | AP side: `msg2 OK (SNonce, MIC verified) — PTK derived`, `sent msg3 (GTK, MIC)`, `msg4 OK — 4-WAY HANDSHAKE COMPLETE (station keyed)` against `wpa_supplicant` with `proto=RSN pairwise=CCMP group=CCMP` |
| Encrypted traffic flows | `6 packets transmitted, 6 received, 0% packet loss, rtt avg 1.156 ms` after the handshake — which requires both ends to agree on CCMP |

### What this does not show

- **The encryption was not independently captured.** The 4-way completing with
  a verified MIC, and traffic flowing to a CCMP-only station, is strong
  evidence that frames are protected — but no third radio sniffed the air to
  confirm the Protected bit directly, and `wpa_cli` could not be queried for
  the negotiated cipher (the supplicant was started without a control socket).
- **Hardware CCMP is untested.** The 4-way above is devourer's software CCMP,
  the same code the Realtek backends use. `MT_WCID_KEY` / `MT_SKEY` are not
  wired up, so the "crypto becomes hardware on MediaTek" claim is still a
  claim.
- **The station is the same silicon** (MT7612U on `mt76x2u`), so this is not an
  independent-generation witness. The RTL8812AU witness in the section above is.
- **One AP, one station, ~20 cm apart.** Every RSSI here is near-field.
- **Longest run 70 s.** No soak, no second station, no rekey, no roaming, and
  no channel change while beaconing.
- **`iw scan` alone is not a witness for a beacon *stopping*.** Its BSS cache
  holds an entry ~30 s after the beacon dies, and it reported a stopped beacon
  as present until `iw scan flush` was used. A re-arm also takes long enough
  (a 1600-byte page copy over EP0) that a scan at +8 s still misses it.
- **The beacon-stop evidence is from a purpose-built harness, not from process
  exit.** Both AP harnesses used to end in `_exit(0)`, skipping the destructor,
  so `StopBeacon` never ran and "the SSID was gone after exit" measured nothing
  - it looked true once, by luck, and was false. They call `StopBeacon`
  explicitly now, and `tests/mt7612u_beacon_stop_check.cpp` is what actually
  exercises the transition.

## The claim, and why it holds

The gap between "MT7612U injector" (what the subtree is) and "MT7612U userspace
AP" is small — a few hundred lines of C in the backend, and no new AP logic at
all. Two reasons:

1. **devourer already has the AP brain, and it is backend-agnostic.** The
   probe/auth/assoc responder, the DHCP/ARP/ICMP data plane, and the WPA2 4-way
   handshake with software CCMP all live in `tests/` (`ap_responder.cpp`,
   `ap_wpa2.cpp`, `probe_responder.cpp`, `beacon_*.cpp`), driven entirely
   through the `IRadio` interface — `StartBeacon` + the RX callback +
   `send_packet`. `docs/ap-mode.md` documents a complete open and WPA2-PSK AP
   validated against real Linux stations on this stack. None of it is
   Realtek-specific; it works against any backend that implements the beacon
   and ACK primitives.

2. **MediaTek's MAC offloads in hardware exactly what an AP needs most.** The
   beacon is auto-transmitted from a reserved page at each TBTT, TSF-stamped by
   the MAC; ACK is SIFS-timed by the MAC against the programmed address;
   802.11 sequence numbering is a MAC function; and CCMP has real per-station
   key hardware (`MT_WCID_KEY`, `MT_SKEY`). On Realtek, devourer does CCMP in
   **software** because the security TX-desc field is absent on most
   generations (`docs/ap-mode.md`: "only Jaguar1 has
   `SET_TX_DESC_SEC_TYPE_8812`"). So on MT the encrypted data plane, and GTK
   rekey which is explicitly out of scope on Realtek, become **hardware** —
   this part is a capability *gain*, not a gap.

## What the MT7612U backend already has

Verified in the merged subtree:

| AP need | present today | where |
|---|---|---|
| Port MAC + BSSID programmed | yes — `MT_MAC_ADDR_DW0/1`, `MT_MAC_BSSID_DW0/1`, MBSS_MODE=3, MBEACON_N | `init.c:206‑216` (`mac_setaddr`) |
| Station table (WCID) | yes — `mt_wcid_setup(idx, mac)` writes `MT_WCID_ATTR` + address; all zeroed at init | `tx.c:95`, `init.c:236` |
| Crypto key slots | the shared-key store is present and zeroed at init (`MT_SKEY`, `MT_SKEY_MODE`, `src/mt7612u/init.cpp` `wcid_and_key_clear()`). The per-station key store is NOT defined in this tree - `MT_WCID_KEY` does not exist here, which is part of why hardware CCMP is unreached |
| ACKed unicast TX | yes — `no_ack=0` sets `MT_TXWI_ACK_CTL_REQ`; BA-window field present | `tx.c:164‑167` |
| Beacon-interval timer regs | defined — `MT_BEACON_TIME_CFG` INTVAL/TIMER_EN/TBTT_EN/BEACON_TX, `MBEACON_N` | `regs.h:176‑180,169` |
| RX filter control | yes — managed default `0x00015f97`, monitor clears to error-only | `init.c:278,494‑509` |
| Register block copy | yes — `mt_wr_copy()` for reserved-page writes | used in `init.c` |

So the addressing, the station table, the crypto slots, the ACK path and the
beacon *timer* are already in place. The receiver runs (per the #414 tick), and
`mt_tx_build()` already produces `[TXWI][802.11]` which is exactly the reserved-
page beacon shape.

## What is still missing: hardware key install

One item, and it is the only thing between this and a fully hardware-accelerated
AP. Everything else in the original gap list — the beacon load and arm, the APC
address match, the TSF/sequence offload, the RX filter — is implemented and on
air; git has the history.

The MAC has real per-station key hardware, and none of it is reached:

- `MT_WCID_KEY(idx)` is not even defined in this tree. `MT_WCID_ATTR`,
  `MT_SKEY` and `MT_SKEY_MODE` are, and are zeroed at init
  (`wcid_and_key_clear()` in `init.cpp`) — the "encrypt nothing" configuration
  an injector wants, and the same registers a key install writes.
- mt76's recipe is small: `mt76x02_mac_wcid_set_key` is ~40 lines of
  `wr_copy` + `rmw_field` over primitives this subtree already has, plus
  `mt76x02_mac_shared_key_setup` for the GTK.
- **There is no per-frame encrypt flag.** TX encryption is selected entirely by
  `txwi->wcid` pointing at a WCID whose `ATTR.PKEY_MODE` is set, and this
  backend hardcodes `wcid = 0xff` (the no-station index). So it is all-or-
  nothing per station: install a key and every frame to that WCID is encrypted
  in hardware; you cannot mix with software CCMP on the same peer.
- On RX the hardware sets `MT_RXINFO_DECRYPT` and **strips IV, MIC and MMIC**,
  so `mt_rx_parse()` would need to handle a changed frame layout, not just
  report a flag.

The blocker is not the driver. `IRadio` has no key surface at all — no install,
no cipher enum — because devourer does CCMP in software on every backend, which
is a reasonable choice when only Jaguar1 has the Realtek TX-descriptor security
field. Reaching MediaTek's crypto therefore means adding an interface member
most backends cannot implement, which is a design decision rather than a port.

## Limitations and shortfalls of a userspace AP on MediaTek — and workarounds

1. **Power-save / TIM is the real fight — USB has no pre-TBTT interrupt.**
   A dynamic beacon (TIM bitmap, buffered multicast/broadcast) must be rewritten
   just before each TBTT. The kernel fakes the interrupt with an hrtimer firing
   8 ms pre-TBTT plus a high-priority workqueue that refills up to `N_BCN_SLOTS`
   buffered frames (`mt76x02u_pre_tbtt_work`, `mt76x02_usb_core.c:128‑217`). In
   userspace that is a timer thread racing a 1–2 ms USB write, and a dense
   25 TU beacon (needed so a scanning supplicant catches the AP) fires it ~40×/s.
   - **Workaround (recommended for FPV/video-return):** a **static** beacon is
     free — the MAC auto-beacons from the reserved page with no host
     involvement. Advertise DTIM=1, no buffering, and do not support
     power-saving clients (the return-video use case has always-on clients).
     Then `StartBeacon` alone suffices and the pre-TBTT machinery is never
     needed.
   - **If PS clients are required:** port the hrtimer+worker as a userspace
     timer thread that calls `UpdateBeaconPayload` pre-TBTT. This is the one
     place the USB-userspace shape genuinely fights the protocol; budget for
     jitter and missed updates.

2. **BlockAck RX reordering is software.** TX aggregation is hardware, but if a
   client sends A-MPDU the AP must own a reorder buffer.
   - **Workaround:** negotiate **no** BlockAck (decline ADDBA) — clients fall
     back to non-aggregated data, lower uplink throughput but correct. Implement
     a reorder window only if client→AP throughput demands it.

3. **No firmware rate control.** `txwi.rate` airs verbatim (proven — there is no
   rate LUT), so the AP picks every client's TX rate in host software.
   - **Workaround:** a fixed rate, or a simple picker off the per-chain RSSI the
     RX path already reports (`rssi[0..1]`). Adequate for a handful of clients.

4. **Multiple concurrent clients.** The WCID table is 256 entries and SKEY is
   per-BSS in hardware, so the silicon supports many clients; the current AP
   harness handles one (`docs/ap-mode.md` scope). Extending is **harness** work
   (a client table, per-client PTK), not driver work.

5. **Out of scope, standard AP-stack breadth.** WMM/QoS EDCA parameter sets,
   DFS/radar on 5 GHz DFS channels, 802.11w management-frame protection, band
   steering, and a real DHCP pool. None are MT-specific gaps; they are AP-stack
   features the harness does not implement.

6. **Management-frame timing is fine.** Probe/auth/assoc responses are tens-of-ms
   tolerant and the userspace RX→TX round-trip is a few ms, proven on the
   Realtek stack (`docs/ap-mode.md`); only SIFS-timed ACK must be hardware, and
   it is.

**Net:** an open or WPA2-PSK AP serving a few always-on clients is very
achievable and *easier* on MT7612U than on Realtek (hardware CCMP + GTK). The
static-beacon path avoids the one hard USB limitation entirely. Power-save
clients and BlockAck reordering are where a userspace MT AP stops being worth
it, and both have clean "don't support it" workarounds for the return-video
use case.

## End-to-end verification (reuse, don't rebuild)

devourer's existing AP checks are backend-agnostic and become the acceptance
suite once the MT backend implements the primitives:

- `tests/beacon_wire_check.cpp` — beacon frame control, +1 seq per beacon, live TSF.
- `tests/beacon_kernel_scan.sh` — a real `rtw88` station's `iw scan` lists the AP.
- `tests/probe_responder.cpp` — active-scan probe response, no beacon.
- `tests/ap_responder.cpp` + `tests/ap_ping_demo.sh` — open assoc → DHCP lease →
  ping 0% loss.
- `tests/ap_wpa2.cpp` + `tests/ap_wpa2_demo.sh` — WPA2 4-way → encrypted DHCP →
  encrypted ping (here, exercise the **hardware** CCMP path).

Success = a real Linux station associates and passes IP traffic against the
MT7612U backend, open and WPA2-PSK, on both 2.4 and 5 GHz, with the static
beacon.
