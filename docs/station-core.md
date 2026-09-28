# The 802.11 station core (`src/sta/`)

A device-free implementation of the protocol half of an 802.11 station:
frame building and parsing, the scan table, the association state machine,
the WPA2-PSK supplicant and CCMP framing. It is header-only and pure: no
`IRadio`, no libusb, no clock, no threads. Time is an argument, frames go in
through `on_rx()` and out through `pop_tx()`, and crypto is a `CryptoOps`
vtable the caller fills, so `libdevourer` gains no dependency and everything
runs under plain `ctest`.

This page is an overview. The contracts (what is refused, when a key is
installed, what the replay window accepts) are documented once, at their
declarations. The maintainer's map of which header holds which rule is
`src/sta/CLAUDE.md`.

## Reading order

`Dot11.h` → `BssTable.h` → `CryptoOps.h` / `Ccmp.h` → `Eapol.h` →
`Supplicant.h` → `StationSm.h`. Each header depends only on the ones before
it.

## What the tests pin

Each area below is a contract documented at the declaration named, and
pinned by the ctest cell named. The per-rule map with individual test
functions is `src/sta/CLAUDE.md`.

- **Key installation and the replay gate**, including key reinstallation
  (the CVE-2017-13077/13078/13080 class): `Supplicant.h` (`on_eapol`,
  `install_gtk`). Cell: `supplicant`.
- **The RSNE downgrade check** (802.11-2016 12.7.6.4):
  `Supplicant::rsn_equivalent`, `RsnInfo` in `Dot11.h`. Cells: `supplicant`,
  `station_sm`.
- **The data-plane replay window and GTK RSC seeding**: `CcmpReplay` in
  `Ccmp.h`. Cell: `ccmp_framing`.
- **What the CCMP calls refuse**: `ccmp_decrypt`, `ccmp_encrypt`. Cell:
  `ccmp_framing`.
- **What the EAPOL-Key parser, MIC check and builder refuse**:
  `parse_eapol_key`, `eapol_mic_ok`, `build_eapol_key`, `find_gtk_kde` in
  `Eapol.h`. Cell: `supplicant`.
- **Which BSS is offered or joined, on which channel**: `BssTable::observe`,
  `StationSm::join`, and the predicates in `Dot11.h`. Cells: `bss_table`,
  `station_sm`.
- **The group rekey path** after association, `StationSm::on_decrypted_msdu`:
  without it hostapd deauthenticates the station after "group key handshake
  failed". Cell: `station_sm`.
- **Cleartext EAPOL-Key once a PTK is installed**:
  `StationSm::cleartext_eapol_allowed`. Cell: `station_sm`
  (`test_cleartext_eapol_after_keying`).

## Where the known answers come from

- **Linux kernel CCMP frames** at all eight TIDs, and **a real hostapd /
  wpa_supplicant four-way**, both captured off `mac80211_hwsim` and checked
  in. They are independent of this code's author, but they are an interop
  reference, not the IEEE Annex J vector: if mac80211 and this code misread
  the same clause the same way, no cell would notice. The captures are not
  in the tree, so these two headers cannot be regenerated from it.
- **IEEE 802.11i Annex H.4.2** passphrase-to-PSK vectors.
- **python-cryptography CCM vectors** (`tests/ccmp_vectors.h`), regenerable
  and checked by the `ccmp_vectors_generated` cell. These are a same-author
  transcription of the framing, so they pin the cipher plumbing only. A
  zero CCM nonce Flags octet passes them, which is why the kernel captures
  exist.

## What this does not do

No device, no hardware crypto offload, no PMF/802.11w, WPA2-PSK with CCMP
only, and no AP-side per-station state. The data plane is the caller's:
`DupDetector` and the MSDU<->Ethernet helpers in `Dot11.h` are tested but
have no in-tree caller, and `StationSm` runs no duplicate cache. Three limits are stated at their
declarations rather than here:
- the replay-window width, and why it must grow before HE/EHT use: `CcmpReplay`;
- the SNonce policy: `Supplicant::start`;
- what a forged message 1 can still cost: `Supplicant::on_msg1`.
