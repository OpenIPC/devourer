# src/sta/ — the device-free 802.11 station core

Deep facts for this subtree, loaded alongside the root CLAUDE.md. Everything
here is header-only and pure: no `IRadio`, no libusb, no clock, no threads, no
environment. Time is a `now_ms` argument, frames go in through `on_rx()` and
out through `pop_tx()`, and crypto is the `CryptoOps` vtable the caller fills,
so `libdevourer` links nothing new. The contracts are documented at their
declarations; this file only maps them. Overview for readers:
`docs/station-core.md`.

## File map, in dependency order

Each header includes only headers above it, which is also the reading order.

| Header | Holds | Includes |
|---|---|---|
| `Dot11.h` | frame builders/parsers, IE walker, `parse_rsn`/`RsnInfo`, `DupDetector`, `SeqCounter`, MSDU<->Ethernet | — |
| `BssTable.h` | per-BSSID scan table, `select()`/`select_open()` | Dot11 |
| `CryptoOps.h` | the crypto interface (CCM, HMAC-SHA1, PBKDF2, key unwrap) | — |
| `Ccmp.h` | CCMP AAD/nonce/header/PN, `ccmp_encrypt`/`ccmp_decrypt`, `CcmpReplay` | CryptoOps, Dot11 |
| `Eapol.h` | EAPOL-Key format, PRF/PTK, `eapol_mic_ok`, GTK KDE, `pmk_from_psk`, `secure_wipe` | CryptoOps |
| `Supplicant.h` | 4-way + group-key decisions, the refusal counters | CryptoOps, Dot11, Eapol |
| `StationSm.h` | auth → assoc → 4-way → connected, timeouts, bounded TX queue | all of the above except Ccmp |

`Ccmp.h` is not included by the state machine on purpose: the data plane
(encrypt/decrypt, per-key PN and replay state) belongs to the caller, which
learns when to reset it from `Supplicant::ptk_generation()`/`gtk_generation()`
and seeds the group window from `gtk_rsc()`.

## Where each rule lives

The rules themselves are the comments at these declarations; this table says
only what, where, and which cell pins it. Change the rule at the declaration
and the cell, never here.

| What | Where (the contract) | Cells |
|---|---|---|
| When a key may be installed; the replay gate; retransmissions | top of `Supplicant.h`, `Supplicant::on_eapol`, `classify` | supplicant: `test_forged_mic_is_rejected`, `test_replay_counter_rules`, `test_group_rekey_replay_rejected`, `test_msg3_without_secure_installs_nothing` |
| Key reinstallation (KRACK class) | `Supplicant::install_gtk`, `on_msg3` | supplicant: `test_msg3_retransmit_does_not_reinstall`, `test_group1_same_gtk_does_not_reinstall` |
| Message 1: the candidate, its cache, what a forgery costs | `Supplicant::on_msg1`, `on_eapol` | supplicant: `test_forged_msg1_cannot_poison_the_counter`, `test_forged_msg1_does_not_orphan_msg3_retransmit`, `test_forged_msg1_first_does_not_poison_the_genuine_one`, `test_msg1_crypto_failure_keeps_the_candidate` |
| Which GTKs install; the key-data walk (one walker for the GTK KDE and message 3's RSNE) | `Supplicant::kGtkLenCcmp`, `walk_key_data`, `find_gtk_kde`, `Supplicant::find_rsn_element` | supplicant: `test_32_byte_gtk_is_refused`, `test_gtk_kde`, `test_gtk_followed_by_truncated_element_is_refused`, `test_rsn_element_after_an_empty_dd` |
| RSNE downgrade check | `Supplicant::rsn_equivalent`, `RsnInfo`/`parse_rsn` | supplicant: `test_rsne_downgrade_check`, `test_parse_rsn_sets`; station_sm: `test_rsn_downgrade_is_refused` |
| Data-plane replay window | `CcmpReplay` | ccmp_framing: `test_replay`, `test_replay_seed` |
| What `ccmp_decrypt` refuses | `ccmp_decrypt` | ccmp_framing: `test_mic_rejected`, `test_short_output_refused`, `test_null_output_and_zero_body`, `test_ext_iv_required`, `test_protected_bit_required` |
| What `ccmp_encrypt` refuses | `ccmp_encrypt`, `kCcmpPnMax` | ccmp_framing: `test_pn_is_48_bits` |
| EAPOL-Key format, MIC compare, builder refusals | `parse_eapol_key`, `eapol_mic_ok`, `build_eapol_key` | supplicant: `test_parse_bounds`, `test_mic_ok_refuses_a_bad_key`, `test_build_length_limit`, `test_mic_crypto_failure_is_not_a_mic_failure` |
| PSK spellings | `pmk_from_psk` | supplicant: `test_psk_known_answers` |
| What a failure / `leave()` drops | `StationSm::fail`, `leave` | station_sm: `test_peer_deauth_drops_the_association_keys`, `test_leave_drops_stale_frames` |
| When the station is Connected | `StationSm::promote_if_keyed` | station_sm: `test_dropped_msg4_does_not_connect` |
| Which EAPOL-Key may arrive in the clear once a PTK is installed | `StationSm::cleartext_eapol_allowed`, `Supplicant::is_installed_msg3` | station_sm: `test_cleartext_eapol_after_keying` |
| Which BSS `join()` accepts, what it tears down first, the beacon-loss window | `StationSm::join`, `kMaxBeaconIntervalTu` | station_sm: `test_join_refuses_an_ibss`, `test_join_refuses_rsn_without_privacy`, `test_5ghz_beacon_without_ds_uses_the_rx_channel`, `test_join_on_a_live_association`, `test_beacon_interval_is_capped` |
| Which BSS selection offers; its channel | `parse_beacon`, `BssTable::observe`, `best_matching`, `channel_valid`, `bss_is_infrastructure`, `bss_is_wpa2_psk` | bss_table: `test_rx_channel`, `test_channel_validity`, `test_only_infrastructure_is_offered`, `test_rsn_without_privacy_is_not_selected`, `test_mfp_required_is_skipped`, `test_tie_break_across_the_wrap`, `test_overlong_ssid_is_refused` |
| Which EAPOL packets reach the supplicant | `eapol_is_key`, `StationSm::on_decrypted_msdu`, `on_rx` | station_sm: `test_only_eapol_key_is_claimed`, `test_group_rekey_through_the_decrypted_path` |
| PMK lifetime across reconfigures | `StationSm::configure`, `configure_open` | station_sm: `test_failed_reconfigure_wipes_the_old_pmk`, `test_reconfigure_forgets_the_keys` |
| Received frames: beacons, deauth, auth and (re)assoc responses, no-data subtypes | `StationSm::on_rx`, `on_auth`, `on_assoc_resp` | station_sm: `test_header_only_beacons_do_not_hold_off_loss`, `test_short_deauth_is_malformed`, `test_deauth_during_handshake`, `test_reassoc_resp_does_not_complete_a_join`, `test_truncated_auth_and_assoc_are_malformed`, `test_qos_null_is_ignored_and_alive` |
| The handshake deadline | `StationSm::eapol_reply`, `on_eapol` | station_sm: `test_dropped_reply_does_not_move_the_deadline` |
| The TX queue: its bound, what `pop_tx` refuses | `StationSm::queue`, `pop_tx`, `kMaxTxQueue` | station_sm: `test_transmit_queue_is_bounded`, `test_join_clears_the_transmit_queue`, `test_pop_tx_refuses_null` |
| Duplicate cache (no in-tree consumer yet) | `DupDetector` | dot11_frames: `test_dup_detector` |

## Tests

| ctest | Binary / script | Covers | Needs |
|---|---|---|---|
| `dot11_frames` | `tests/dot11_selftest.cpp` | Dot11.h incl. `DupDetector` | — |
| `bss_table` | `tests/bss_table_selftest.cpp` | BssTable.h | — |
| `ccmp_framing` | `tests/ccmp_selftest.cpp` | Ccmp.h + both CCMP vector sets | OpenSSL |
| `supplicant` | `tests/supplicant_selftest.cpp` | Eapol.h, Supplicant.h, the hostapd four-way | OpenSSL |
| `station_sm` | `tests/station_sm_selftest.cpp` | StationSm.h incl. the group rekey path | OpenSSL |
| `ccmp_vectors_generated` | `tests/ccmp_gen_vectors.py --check` | `tests/ccmp_vectors.h` is what the generator emits | Python3 + python-cryptography (else skipped) |

The OpenSSL cells are simply not registered without OpenSSL (configure
WARNING). `-DDEVOURER_REQUIRE_STA_CRYPTO_TESTS=ON` makes that a configure
error; CI sets it on every job that runs ctest. `tests/openssl_crypto_ops.h`
is the one complete `CryptoOps`; `tests/ccmp_software.h` is its CCM.

## Vectors

What each vector file can and cannot catch is stated in its own header
comment and its generator's; read those, not a summary.

- `tests/ccmp_vectors.h` — generated by `tests/ccmp_gen_vectors.py`,
  reproducible (`--check`, the `ccmp_vectors_generated` cell).
- `tests/ccmp_kernel_vectors.h`, `tests/eapol_kernel_vectors.h` — cut by
  `tests/{ccmp,eapol}_extract_vectors.py` from captures made by
  `tests/{ccmp,eapol}_capture_vectors.sh` (root + hwsim; never in CI). Not
  reproducible from the tree: edit them only by re-capturing.

## Deliberately out of scope

No device or radio calls; no hardware crypto offload; no PMF/802.11w
(`StationSm::on_rx` states what that costs); key descriptor v2 only (top of
`Eapol.h`); no AP-side per-station table. SNonce policy: `Supplicant::start`.
Replay-window width and why it must change before any HE/EHT use:
`CcmpReplay`.

`Dot11.h`'s MSDU<->Ethernet conversion and `DupDetector` have no in-tree
caller yet (`StationSm` does not run the duplicate cache; its contract is
at `DupDetector`), and this tree's AP harnesses (`tests/ap_responder.cpp`,
`tests/ap_wpa2.cpp`) carry their own inline builders rather than using this
module.
