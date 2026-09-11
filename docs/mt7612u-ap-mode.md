# MT7612U as a fully-userspace access point

**Status: it works.** devourer's own AP harnesses run against this part with no
change to their AP logic — beacon, probe, auth, assoc, and the ARP/ICMP data
plane — with a real Linux station associated. `StartBeacon`, `UpdateBeaconPayload` and
`StopBeacon` are implemented on `Mt7612uRadio`, so nothing in
`tests/ap_responder.cpp` or `tests/ap_wpa2.cpp` branches on the backend. Their AP
logic is untouched; each gained a dozen lines that silence the beacon before
`_exit`, and nothing in that is MediaTek-specific — the beacon is
hardware-autonomous on the Realtek parts too.

**WPA2-PSK works too.** `tests/ap_wpa2.cpp`, its AP logic equally untouched, completes the
4-way handshake against a real `wpa_supplicant` station and carries encrypted
traffic. It needs the same five `IRadio` methods as the open-network harness -
`InitWrite`, `StartBeacon`, `StartRxLoop`, `send_packet`, `StopBeacon` - and no
others. CCMP is done in software in the harness, so no key API is involved;
that is a separate point from the method count.

What is NOT done: **hardware** CCMP. `MT_WCID_KEY` is absent and the key path
is unreached (`MT_SKEY` is defined, and zeroed at init), so whether hardware
crypto is a capability *gain* on this part is unmeasured — what is measured is
that the software path devourer already had works here. See "What is still
missing" below.

File:line references below are to the merged subtree (`src/mt7612u/`, all
`.cpp` since the C++ migration) and to `reference/mt76 @ be5ce79`.

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
one is devourer itself: `tests/ap_responder.cpp`, built unchanged against
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

## Three findings that shaped the implementation

Not a status list — these are the things that were not obvious from mt76 and
that the code now depends on.

1. **On this MAC, "the AP" is an address match plus a beacon; there is no
   responder register.** The immediate-response engine ACKs frames whose
   address 1 matches `MT_MAC_ADDR_DW0/DW1`, gated by `MT_AUTO_RSP_EN` — which
   init already leaves on (`mac_reset()` writes `MT_AUTO_RSP_CFG = 0x13`,
   `init.cpp:174`, reached from `mt_init_hardware()` at `:408`). So arming
   an ACK responder means *retargeting the port identity*, and closing the gate
   does not stop a die that matches on identity — restoring the address does.
   The consequence is that `MT_MAC_ADDR` has two users, the beacon and
   `SetAckResponder`, sharing one register and one save slot; ownership belongs
   to whoever wrote last, and both paths have to hand it over explicitly.

2. **The APC BSSID slot index is derived from the address, and getting it wrong
   is silent.** Under `MBSS_MODE=3` mt76 computes
   `idx = 1 + (((macaddr[0] ^ addr[0]) >> 2) & 7)` for a locally-administered
   address and 0 otherwise (`mt76x02_util.c:310`) — *after* `mt76x02_mac_setaddr`
   has moved both the port MAC and the MBSS base, so its XOR is zero by
   construction. A beacon that retargets only `MT_MAC_ADDR` leaves the base at
   the factory address, the hardware derives a different slot, and the AP
   beacons perfectly and acknowledges nobody. `mt_mac_set_bss_base()`
   (`beacon.cpp`) is what makes the mt76 identity hold here. The masks
   themselves were also transcribed two bits high in an earlier draft
   (`regs.h:170-179`); `MBSS_MODE=4` is not a valid mode.

3. **The RX filter's *default* is not what an AP wants — `set_monitor_rx` is.**
   The init value is `0x00015f97` (`init.cpp:290`), and `MT_RX_FILTR_CFG_DUP`
   is set in it. What leaves DUP clear is `mt7612u_set_monitor_rx()`
   (`init.cpp:560-569`), deliberately, because duplicate suppression hides the
   retransmissions an ACK-responder test counts — a station's retry with the FC
   Retry bit set is exactly how you learn whether your ACKs are landing, and
   `auth … retry=0` in the on-air harness is that evidence. Every AP path
   reaches it (`StartRxLoop` calls it, and an AP must receive); a TX-only
   consumer does not, and has no receiver to count retries with anyway. The
   beacon path therefore touches the filter in neither direction.

## What is still missing: hardware key install

The MAC has real per-station key hardware and none of it is reached. This is
the last item, but it is more than one register:

- **Two registers are absent from this tree.** `MT_WCID_KEY` and `MT_WCID_IV`
  are both undefined here; `mt76x02_mac_wcid_set_key` writes both
  (`mt76x02_mac.c`). `MT_WCID_ATTR`, `MT_SKEY` and `MT_SKEY_MODE` *are* defined
  and are zeroed at init by `wcid_and_key_clear()` — the "encrypt nothing"
  configuration an injector wants, and the same registers a key install writes.
- **The per-frame encrypt gate is set the wrong way for crypto.** There IS such
  a flag — `MT_TXD_INFO_WIV` (`regs.h`) — and `mt_tx_build()` sets it
  unconditionally, meaning "no hardware IV insertion, this frame is not
  encrypted". mt76 gates it on whether the WCID has a key
  (`mt76x02_usb_core.c`, `mt76x02_txrx.c`: `!wcid || hw_key_idx == 0xff ||
  sw_iv`). A key install has to make WIV conditional too, not just fill the key
  slots.
- **TX selects encryption by WCID, so it is per-station and all-or-nothing.**
  `txwi->wcid` chooses the key; `mt_tx_build()` is called with `0xff` (the
  no-station index) from every library path. `tools/bringup.cpp` does pass a
  real WCID for its rate-LUT gate, so the plumbing exists — but no
  `Mt7612uRadio` path or AP harness installs a station.
- **RX needs real work, not a flag.** The hardware strips the MIC and MMIC, but
  **not** the IV/PN: mt76 removes that in the driver using `MT_RXINFO_PN_LEN`,
  and deliberately does not on a fragment. `MT_RXINFO_PN_LEN` is already
  defined here and unused; `MT_RXINFO_DECRYPT` is not defined at all.

The blocker is still not the driver. `IRadio` has no key surface at all — no
install, no cipher enum — because devourer does CCMP in software on every
backend, which is reasonable when only Jaguar1 has the Realtek TX-descriptor
security field. The maintainer's guidance (PR #424) is to keep it that way for
now and design the key surface against two backends rather than one: a crypto
key interface is a much larger contract than a feature flag — key lifetime, GTK
vs PTK, rekey, who owns the replay counter — and expensive to undo once callers
exist.

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

## End-to-end verification

devourer's existing AP checks are backend-agnostic, so they are the acceptance
suite for this backend with no AP-logic change:

- `tests/beacon_wire_check.cpp` — beacon frame control, +1 seq per beacon, live TSF.
- `tests/beacon_kernel_scan.sh` — a real station's `iw scan` lists the AP.
- `tests/probe_responder.cpp` — active-scan probe response, no beacon.
- `tests/ap_responder.cpp` + `tests/ap_ping_demo.sh` — open assoc → DHCP lease →
  ping 0% loss.
- `tests/ap_wpa2.cpp` + `tests/ap_wpa2_demo.sh` — WPA2 4-way → encrypted DHCP →
  encrypted ping. Software CCMP; the **hardware** CCMP path is the open item.

`tests/mt7612u_ap_onair.sh` is the one MT-specific piece. Its three cells drive
`ap_responder`, `ap_wpa2` and `tests/mt7612u_beacon_stop_check.cpp` against a
real station and grade them by `iw scan` (matched on BSSID *and* SSID) and by
the AP's own log, so a run is a pass/fail line rather than an operator reading
output. It is a root harness — `iw`, `wpa_supplicant`, and between cells a USB
power-cycle when `AP_VBUS` names a hub port, otherwise an `authorized` toggle,
which is not a cold cycle but does end this MAC's autonomous beacon (measured,
see the comment there). Environment: `CH`, `BUILD`, `FW_DIR`, `PSK`, `SECS`,
`AP_SYSFS`, `STA_SYSFS`, `AP_VBUS`, plus an optional cell argument (`open`,
`wpa2`, `stop`, `all`).

Success = a real Linux station associates and passes IP traffic against the
MT7612U backend, open and WPA2-PSK, on both 2.4 and 5 GHz, with the static
beacon. Measured: 14/14 on ch36 and 14/14 on ch6.
