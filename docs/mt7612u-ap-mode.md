# MT7612U as a fully-userspace access point — what is missing

**Status: Stages A and B implemented and device-verified** (see "Verified on
hardware" below); Stages C–E (association responder, data plane, WPA2) still
need the `IRtlDevice` wrapper and the existing C++ harnesses. File:line
references are to the merged subtree at `daabab7` (`src/mt7612u/`), the mt76
reference at `be5ce79`, and devourer's existing AP work.

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
| StopBeacon contract | After the process exits the BSSID is absent from a fresh scan — nothing left airing |

Not yet done: probe **responses**, auth/assoc **responses** and the data plane —
those are the existing backend-agnostic C++ harnesses' job (Stages C–E), not
driver work.

## The claim, and why it holds

The gap between "MT7612U injector" (what the subtree is) and "MT7612U userspace
AP" is small — a few hundred lines of C in the backend, and no new AP logic at
all. Two reasons:

1. **devourer already has the AP brain, and it is backend-agnostic.** The
   probe/auth/assoc responder, the DHCP/ARP/ICMP data plane, and the WPA2 4-way
   handshake with software CCMP all live in `tests/` (`ap_responder.cpp`,
   `ap_wpa2.cpp`, `probe_responder.cpp`, `beacon_*.cpp`), driven entirely
   through the `IRtlDevice` interface — `StartBeacon` + the RX callback +
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
| Crypto key slots | present and zeroed — `MT_WCID_KEY`, `MT_SKEY`, `MT_SKEY_MODE` | `regs.h:239‑245`, `init.c:242‑246` |
| ACKed unicast TX | yes — `no_ack=0` sets `MT_TXWI_ACK_CTL_REQ`; BA-window field present | `tx.c:164‑167` |
| Beacon-interval timer regs | defined — `MT_BEACON_TIME_CFG` INTVAL/TIMER_EN/TBTT_EN/BEACON_TX, `MBEACON_N` | `regs.h:176‑180,169` |
| RX filter control | yes — managed default `0x00015f97`, monitor clears to error-only | `init.c:278,494‑509` |
| Register block copy | yes — `mt_wr_copy()` for reserved-page writes | used in `init.c` |

So the addressing, the station table, the crypto slots, the ACK path and the
beacon *timer* are already in place. The receiver runs (per the #414 tick), and
`mt_tx_build()` already produces `[TXWI][802.11]` which is exactly the reserved-
page beacon shape.

## The gap — the driver primitives to add

Each has a direct mt76 recipe. Estimates are the C-library side only.

1. **`StartBeacon` — load the beacon + arm the MAC beacon function (~70 LOC).**
   - Add two register defines our `regs.h` lacks: `MT_BEACON_BASE` (0xc000) and
     `MT_BCN_OFFSET(n)` (0x041c + n·4). Everything else is present.
   - Program the 5 USB beacon slots: `slot_size = (8192/5) & ~63 = 1600`,
     offsets via `MT_BCN_OFFSET` — mt76 `mt76x02_set_beacon_offsets`
     (`mt76x02_beacon.c:10`, `N_BCN_SLOTS=5` in `mt76x02_usb_core.c:126`).
   - Write `[TXWI][beacon MPDU]` into `MT_BEACON_BASE` with `mt_wr_copy()` —
     mt76 `mt76x02_write_beacon`/`mt76x02_mac_set_beacon`
     (`mt76x02_beacon.c:24,54`). `mt_tx_build()` already emits that shape.
   - Enable: set `MT_BEACON_TIME_CFG` `BEACON_TX | TBTT_EN | TIMER_EN` with
     `INTVAL = interval_tu` — mt76 `mt76x02_mac_set_beacon_enable`
     (`mt76x02_beacon.c:69`). Point `MT_MAC_BSSID_*` at the AP BSSID, **unicast**
     (`0x02…`) — `docs/ap-mode.md`'s hardest-won finding: an I/G-set BSSID makes
     the station drop auth before it reaches the air.
   - 802.11 sequence numbering: let the MAC number it (the HW-seq path mt76
     uses for beacons) or number it per update in software; `beacon_wire_check`
     expects +1 per beacon.
   - **Trap — a silent no-transmit.** `MT_BCN_BYPASS_MASK` (0x108c) is
     **inverted**. `0xffff`, the value `mt76x02_init_beacon_config()` writes, is
     "suppress every slot" — the guard mt76 raises *during* an update ("prevent
     corrupt transmissions during update"). A slot only airs once its bit is
     **cleared**, which mt76 does after loading the page:
     `0xff00 | ~(0xff00 >> beacon_data_count)` (`mt76x02_usb_core.c:223`,
     identically `mt76x02_mmio.c:43`); for one beacon in slot 0 that is
     `0xffffff7f`. Leave it at `0xffff` and the beacon timer runs, the TSF
     advances and every register reads correct — while nothing reaches the air.
   - **Trap — the beacon TXWI needs two extra bits.** `MT_TXWI_FLAGS_TS` (BIT 3;
     MAC inserts the TSF timestamp) and `MT_TXWI_ACK_CTL_NSEQ` (MAC assigns the
     sequence number), exactly as `mt76x02_mac_write_txwi()` sets them for
     beacon/probe-resp subtypes. Without them the beacon airs with a frozen
     timestamp and a constant sequence number.
   - **Trap — `regs.h` had the MBSS masks two bits high.** `MBSS_MODE`,
     `MBEACON_N` and `MBSS_LOCAL_BIT` were transcribed as 19:18 / 22:20 / 23
     instead of mt76's 17:16 / 20:18 / 21. Harmless while the port only
     injected (only `mac_setaddr` uses them, and beacons were never generated),
     but it programmed MBSS_MODE=4 (invalid) and MBEACON_N=15. Corrected;
     `MT_MAC_BSSID_DW1` now reads `0x003f____`, matching mt76 bit for bit.

2. **`StopBeacon` (~10 LOC).** Clear `BEACON_TX | TBTT_EN | TIMER_EN`. Note the
   `IRtlDevice` contract: the chip beacons autonomously, so a session that ends
   without a power-cycle **must** call this or the beacon contaminates the next
   run (`src/IRtlDevice.h:424`).

3. **Per-station / group key install (~50 LOC).** `MT_WCID_KEY(idx)` +
   `MT_WCID_ATTR` PKEY_MODE/PAIRWISE for pairwise, `MT_SKEY` + `MT_SKEY_MODE`
   for the GTK — mt76 `mt76x02_mac_wcid_set_key` / `mac_shared_key_setup`
   (`mt76x02_mac.c`). The cipher enum (`MT76X02_CIPHER_*`) is small. This buys
   **hardware CCMP**; the software CCMP in `ap_wpa2.cpp` still works as the
   fallback/portable path.

4. **AP RX filter — one line, not ~15 LOC.** Measured: the managed default
   `0x00015f97` already leaves `OTHER_BSS`, `BCAST` and `MCAST` **undropped**,
   so a probe request with a wildcard BSSID and auth addressed to us both
   arrive unchanged; mt76 clears `OTHER_BSS` for every mode too
   (`mt76x02_configure_filter`), and defines no AP-specific filter. The single
   change an AP wants is clearing `DUP` (`0x00015f17`) so a station's
   retransmission stays visible — dropping duplicates hides exactly the
   retry evidence that tells you whether your ACKs are landing.

5. **`SetAckResponder` — covered by address match. OPEN ITEM NOW RESOLVED.**
   On MT the MAC auto-ACKs frames matching the programmed `MT_MAC_ADDR`, so
   pointing it at the BSSID (done in `mac_setaddr`) is the ACK responder.
   Confirmed on hardware: **there is no AP op-mode/net-type register on this
   part.** mt76 sets none — its only AP-specific work in `mt76x02_sta_add` is a
   *software* PS flag (`MT_WCID_FLAG_CHECK_PS`) plus a GTK restriction — and a
   real station's auth arrived at **retry=0** with nothing but address match +
   beacon enable. Address match + beacon *is* the AP.
   - **But one register the gap analysis missed:** `mac_setaddr()` zeroes all
     eight APC BSSID slots, which is right for an injector and wrong for an AP.
     The MAC matches the BSS against `MT_MAC_APC_BSSID_L/H(idx)` for the slot
     the MBSS index selects, so an AP must publish its BSSID there —
     `mt76x02_mac_set_bssid()` (`mt76x02_mac.c`), 4 lines. Without it the MAC
     matches nothing for the BSS.
   - The simplest consistent choice is to make the AP BSSID the device's own
     MAC: `mac_setaddr()` already programs it into `MT_MAC_ADDR` (what the MAC
     ACKs against) and `MT_MAC_BSSID`, so only the APC slot is left to write.
     Advertising any *other* BSSID in the beacon leaves a station addressing
     auth to an address the MAC does not answer for.

6. **`UpdateBeaconPayload` (~10 LOC, optional).** Re-write the reserved page in
   place. Only needed for dynamic beacon content (below).

### The two integration layers

- **The mt7612u C library** gains items 1–4 above (~150 LOC, all with recipes).
- **`RtlMt7612uDevice`** (the wrapper from the integration PR) exposes them as
  `StartBeacon`/`StopBeacon`/`UpdateBeaconPayload`/`SetAckResponder` over the C
  ABI, so the **existing** C++ AP harnesses in `tests/` run unchanged. No AP
  logic is written — it already exists.

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
