# Infrastructure AP mode — devourer as a Wi-Fi access point

devourer's hardware-beacon path (`StartBeacon`, see `docs/time-distribution.md`)
is the foundation for acting as a real infrastructure **access point**: a Linux
station discovers devourer, authenticates, associates, gets an IP, and exchanges
IP traffic with it — open or WPA2-PSK encrypted. These are experimental example
harnesses under `tests/`, not a library API — the pieces (beacon, RX callback,
`send_packet`) are the same ones the demos use; the AP logic (probe/auth/assoc
responses, DHCP/ARP/ICMP, the WPA2 4-way handshake, software CCMP) lives in the
harness. All bench-validated against the real Linux `rtw88`/`mac80211` stack.

A dense beacon interval (`DEVOURER_BCN_TU=25`) is needed throughout, so a
supplicant's fast channel-hopping scan catches the AP. The AP adapter must
run `StartBeacon` (all generations) plus full-duplex `StartRxLoop` — the AP
harnesses are bench-validated on J2/J3 adapters; the station is a second
Realtek adapter bound to the kernel (rtw88 auto-probes a VBUS-cold dongle to a
`wlp*` interface). `send_packet` must run **off** the RX event thread (queue the
requester, TX from another thread — libusb returns BUSY from the RX callback).

## The beacon is accepted by a real kernel scan

`tests/beacon_wire_check.cpp` checks the on-wire beacon: right frame control
(0x0080), an 802.11 sequence number that **increments by 1 per beacon** (the
hardware sequence numbering a kernel AP does via `EN_HWSEQ`), and the live TSF —
on all three generations (exception: the 8814A airs its stored beacon with the
sequence pinned at 0, matching the kernel rtw88 driver on the same chip).
`tests/beacon_kernel_scan.sh` goes further: a real
station bound to `rtw88` runs `iw scan` and lists devourer, parsing every element:

    BSS 57:42:75:05:d6:00   TSF <live>   freq 2437   beacon interval 25 TUs
    capability: ESS   SSID: devourerAP
    Supported rates: 1.0* 2.0* 5.5* 11.0* 18.0 24.0 36.0 54.0
    DS Parameter set: channel 6   TIM: DTIM Count 0 Period 1   ERP: <no flags>

Confirmed on **both bands** — ch6 (2437 MHz) and ch36 (5180 MHz).

## Active scans — probe response

`tests/probe_responder.cpp` runs full-duplex, matches probe-requests in the RX
callback and `send_packet`s a probe-response to the requester. With **no beacon
aired**, a real station's `iw scan` still lists `devourerAP` — discovery via the
probe response. This works because management-frame timing is tens of ms (the
userspace RX→TX round-trip is a few ms), unlike SIFS-timed ACKs.

## Open-network association + data plane

`tests/ap_responder.cpp` completes the whole open handshake (probe → auth →
assoc): a real station authenticates, associates, and stays `Connected`
(wpa_supplicant `CTRL-EVENT-CONNECTED`), with auth/assoc requests arriving at
**retry=0** — devourer hardware-ACKs them (the ACK engine matches `REG_MACID`,
which `StartBeacon` sets to the BSSID).

The one non-obvious requirement: **the BSSID must be unicast**. The canonical test
SA `57:42:75:05:d6:00` has the I/G (multicast) bit set in `0x57`; a station cannot
unicast-auth to a multicast address, so rtw88 silently drops the auth before it
hits the air (bench-proven across two station chips — `0x57` → no auth on air,
`0x02` → auth on air and association completes). Use a locally-administered unicast
BSSID (`0x02..`).

It then answers the data plane over 802.11 from-DS data frames: a minimal **DHCP
server** (leasing 192.168.99.2), **ARP** replies for the AP IP, and **ICMP** echo
replies. So a station associates, `dhcpcd` **auto-leases 192.168.99.2**, and
`ping 192.168.99.1` returns **0% packet loss, ~2 ms RTT** — the full self-service
chain (beacon → auth → assoc → DHCP → ARP/ICMP → ping), like a hostapd+dnsmasq AP
in one userspace process. `tests/ap_ping_demo.sh` runs it end to end.

## WPA2-PSK (4-way handshake + CCMP)

`tests/ap_wpa2.cpp` advertises an RSN IE and runs the WPA2 **4-way handshake**
authenticator — msg1→4 with PMK/PTK (PBKDF2+PRF), HMAC-SHA1 MIC and an
AES-key-wrapped GTK (openssl in userspace). A real station completes it:
`WPA: Key negotiation completed … [PTK=CCMP GTK=CCMP]` + `CTRL-EVENT-CONNECTED`.
The handshake needs no CCMP (EAPOL-Key frames are cleartext, MIC-protected), so it
works without the chip crypto engine.

The **data plane is encrypted too**: the AP decrypts the station's CCMP frames
(software AES-CCM with the TK) and answers ARP + ICMP + DHCP encrypted, so a real
station **leases 192.168.99.2 over encrypted DHCP** and **pings over WPA2/CCMP at
0% loss, ~2.5 ms RTT**. The station runs hardware CCMP, the AP software CCMP — they
interoperate. So devourer is a complete zero-config WPA2-PSK AP: associate → 4-way
→ encrypted DHCP → encrypted IP.

CCMP framing detail: the AAD masks the FC subtype/retry/pwr-mgmt/more-data bits and
sets protected, and masks the sequence number (keeping the fragment number); the
nonce is `0 | A2 | PN(6, big-endian)`; the CCMP header carries the 48-bit PN + the
ext-IV key id. A hardware CCMP offload would instead need the J3 security TX/RX
descriptor fields, which are absent in devourer (only Jaguar1 has
`SET_TX_DESC_SEC_TYPE_8812`) — software CCMP sidesteps that.

## Scope

These harnesses implement enough AP-side logic to interoperate with a real station
end to end. What is intentionally out of scope (AP-*stack* breadth, not driver
parity): multiple concurrent clients, GTK broadcast/rekey, routing/NAT, and a real
DHCP address pool. The bench caveat is that a single clean end-to-end run of the
whole WPA2 chain is flaky after many cycles when the AP adapter has no VBUS reset
(an xhci root-hub port) — cold-cycle the station between runs, and prefer a
VBUS-cyclable AP adapter.
