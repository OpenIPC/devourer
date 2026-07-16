# HE ER SU — the 802.11ax extended-range PPDU (Kestrel)

This document describes devourer's HE ER SU (Extended Range Single User)
support on the Kestrel generation (RTL8852BU/8832BU and RTL8852CU/8832CU). ER
SU is 802.11ax's *standards-based* range lever: a boosted, repeated HE-SIG-A
preamble plus a low-order single-stream payload, decodable by any compliant AX
receiver. Pre-AX generations (Jaguar1/2/3, 802.11ac) have no ER equivalent —
the format simply does not exist in their PHYs, and `AdapterCaps.he_er_su_ok`
reports accordingly.

## The range ladder

ER SU is a 20 MHz-only format with three stackable pieces:

| Mode | Payload | Nominal gain vs HE SU | Constraint |
|---|---|---|---|
| ER 242-tone RU | full-width 20 MHz payload | preamble-acquisition gain (~3 dB SIG-A boost + repetition) | MCS 0–2, NSS 1 |
| ER 106-tone RU | upper ~8 MHz of the channel | +~3 dB more (power concentrated in half the tones) | MCS 0, NSS 1 |
| DCM | dual-carrier modulation, each symbol on two tones | +~3 dB payload diversity, halves rate | MCS 0/1 (0/1/3/4 outside ER); excludes STBC |

Fully stacked (ER 106-tone + DCM at MCS0) the format buys roughly 8–10 dB over
plain HE SU MCS0. Compare devourer's **narrowband** (5/10 MHz re-clock,
`docs/narrowband.md`): narrowband's gain is a deterministic noise-bandwidth
reduction (~3 dB per octave, whole link) but is private — both ends must run
devourer. ER SU is smaller and acquisition-weighted but interoperates with any
802.11ax device. On a devourer↔devourer link prefer narrowband; use ER SU when
the far end is standard AX gear.

## API

Per-packet, radiotap-driven (like rate/LDPC/STBC): a frame whose radiotap HE
field (type 23) carries `data1 FORMAT = EXT_SU(1)` airs as ER SU. A
`data5 BW_RU_ALLOC` of 106-tone (6) selects the 106-tone variant; the DCM bit
(`data3` bit 12) selects DCM. Out-of-spec combos (NSS>1, MCS above the mode's
ceiling, DCM+STBC) are clamped with a `W` diagnostic rather than handed to the
MAC — an unresolvable rate word stalls the AX scheduler and NAKs the bulk-OUT.

Session default via the TX-mode grammar (HE rates only):

```sh
DEVOURER_TX_RATE=HE1SS_MCS0/ER        # ER SU, 242-tone RU
DEVOURER_TX_RATE=HE1SS_MCS0/ER106     # ER SU, 106-tone RU (MCS0 only)
DEVOURER_TX_RATE=HE1SS_MCS0/ER/DCM    # + dual-carrier modulation
DEVOURER_TX_RATE=HE1SS_MCS0/DCM       # DCM on plain HE SU (no ER)
```

Programmatic: `TxMode.he_er` (0 = off, 1 = 242-tone, 2 = 106-tone) and
`TxMode.he_dcm`, through the ordinary `SetTxMode` path (HE modes ride the
radiotap-HE builder). Capability flag: `AdapterCaps.he_er_su_ok` (true on both
Kestrel dies; the `adapter.caps` event emits it as `he_er_su`).

## Hardware mechanics

Normal TX goes through the MAC TX descriptor (WD), not the halbb PLCP
generator, so ER SU is three descriptor bits:

- **8852B** (24-byte wd_body): `AX_TXD_DATA_ER` (BIT15), `AX_TXD_DATA_BW_ER`
  (BIT8, the 106-tone selector) and `AX_TXD_DATA_DCM` (BIT14) all live in
  wd_info dword0 alongside LDPC/STBC/DISDATAFB.
- **8852C** (32-byte `wd_body_t_v1`): DCM moved into the wd_body dword7 rate
  word (`AX_TXD_DATA_DCM_V1`, BIT30); `DATA_ER`/`DATA_BW_ER` stay in wd_info
  dword0 at the 8852B positions.

On RX the AX descriptor classifies every frame's PPDU format in dword1[3:0]
(`RX_DESC_PPDU_T_*`: 7 = HE_SU, 8 = HE_ERSU). devourer surfaces it as
`RxAtrib.ppdu_type` (0xff on pre-AX generations) and in the `rx.txhit` /
`adapter.caps` events — a received `ppdu_type=8` is the on-air proof that the
ER SU format actually flew, since the payload rate alone can't distinguish ER
SU MCS0 from plain SU MCS0.

## Validation

Headless: `tests/he_radiotap_selftest.cpp` (grammar → radiotap → decode
round-trip, clamp rules) and `tests/kestrel_rxparse_selftest.cpp` (RXD
ppdu_type nibble), both in ctest.

On-air: `tests/he_er_su_cross_rx.sh` — 15 s cells, TX one Kestrel die, `rxdemo`
on the other; every `rx.txhit` with `ppdu_type=8` proves an ER SU frame
decoded. The plain-SU control must report `ppdu_type=7`. A monitor sniffer's
radiotap bandwidth is not a witness here; the RXD format nibble is.

**Measured matrix** (ch6, ~3–6 k frames per 15 s cell, three-adapter
triangulation: 2× 8852C + 1× 8852B): every mode delivers and classifies
correctly in every pairing — cross-die both directions and same-die
8852C → 8852C:

| Cell | Delivers | Classified |
|---|---|---|
| HE SU MCS0 (control) | yes | 7 (HE_SU) |
| ER 242-tone MCS0/MCS1/MCS2 | yes | 8 (HE_ERSU) |
| ER 106-tone MCS0 | yes | 8 |
| ER 242-tone MCS0 + DCM | yes | 8 |
| SU MCS0 + DCM | yes | 7 |

One bench trap at ER MCS1/2: a same-die pair at point-blank range fails
QPSK while MCS0 (BPSK) still passes — near-field saturation at the default
20 dBm, not a protocol limit (`DEVOURER_TX_PWR=6` restores delivery; the
EVM-vs-RSSI tell in `docs/bench-testing-near-field.md`).

Judging 8852C-side RX requires the per-die RX-descriptor drv_info unit
(8 B on the 8852B, 16 B on the 8852C — `FrameParserKestrel.h`): with the
wrong unit the C die "receives" frames with correct lengths but unreadable
bodies, which a txhit-based harness misreads as a deaf receiver.
