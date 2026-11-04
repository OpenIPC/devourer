# M2 — reliability and link adaptation on the reference cell

M2 (#256, EPIC #254) makes the [M1 cell](cell-m1.md) reliable:

- **HARQ** (`src/cell/Harq.h`): DL retransmissions driven by explicit UE
  feedback.
- **Link adaptation** (`src/cell/LinkAdapt.h`): a per-UE DL rate, published in
  the grant map.

Both are headless, and the `cell_m2` ctest covers them, including a lossy-channel
simulation. `examples/cell` wires them in.

## Why HARQ is software, from UE feedback

Kestrel has no per-frame TX report (no CCX path). The DU therefore cannot see
whether a hardware ACK came back. On any chip, a no-ACK "TX complete" proves
only that the frame aired. So delivery is what the UE says it received:

- **On air:** each DL frame carries its HARQ seq, attempt and the sender's
  window low edge. The UE's uplink beacon carries its `HarqFeedback` (a
  cumulative base plus a 32-seq bitmap) and its DL SNR. Both are versioned,
  and tagged with the cell PSK when `CELL_PSK` is set. The byte layouts live
  in one place: `examples/cell/cell.h`, at `build_dl_frame` /
  `build_ue_beacon`.
- **DU sender rules:**
  - ACK: the seq is below `base`, or its bitmap bit is set.
  - NACK: a later seq was received. It is judged by when the UE *generated*
    the report (`gen_sf`, carried in the IE), not by when it arrived. Only an
    attempt older than the report counts, so a delayed report that predates
    a retransmission never NACKs it.
  - Timeout after 3 superframes, and up to 4 attempts.
  - Retransmissions go before new data.
  - The in-flight seq span stays below 32, so every report covers every
    in-flight frame.
- **UE receiver:** suppresses duplicates, and abandons gaps below `low`.

## Link adaptation

The DU judges each UE every TA window (2 s):

- It steps one rate down when the retransmission ratio reaches 20%, then holds
  for 5 windows.
- It drops straight to the best rate the UE-reported DL SNR supports once that
  SNR no longer carries the current rate.
- It steps one rate up after 3 clean windows, if the SNR reaches the next
  rate's threshold + 2 dB.

The rate rides in the grant (`Grant::rate_mbps`), and the DU sends at the rate
of the map in force. A window is judged only if the UE's uplink was heard in
≥ 75% of its superframes, and also in the window before. At the edges of a UE's
presence, the HARQ timeouts and drops belong to frames sent while it wasn't
listening. Without this gate, those edges stepped the control UE down on the
rig.

## Measured on the epic-254 rig

The rig: RTL8852CE over PCIe for all roles (DU, two UEs, witness), ch149, 120 s
per arm, DL starting at 54 Mb/s. **UE1's DL is sent 10 dB below the session
power** (`CELL_ATTEN_UE=1 CELL_ATTEN_DB=-10`, a per-packet radiotap delta)
as a path-loss stand-in. UE2 is the control. Run via testrig
`scripts/m2_cell_reliability.sh`; `tests/cell_2ue_analyze.py` counts the
`harq` section.

| arm | UE1 delivered / originated | UE1 rate | UE2 delivered | UE2 rate |
|---|---|---|---|---|
| baseline (`CELL_HARQ=0`) | 560 / 1156 (48.4 %) | 54 | 1155 / 1155 | 54 |
| HARQ (`CELL_LA=0`) | 481 / 547 (87.9 %), 608 retx | 54 | 1156 / 1156 | 54 |
| HARQ + LA | **1136 / 1136 (100 %)**, 19 retx | 54 → 9 → 6 | **1154 / 1154** | **54** |

The UL stayed 100 % in slot in every arm (1.5–2.0 µs rms). An earlier run of the
same three arms, before the NACK rule judged report age, read 632/1156,
620/650 and 1133/1134 for UE1. The control UE was untouched in both runs.

- **HARQ alone** makes delivery reliable but costs throughput. UE1 has one DL
  slot per superframe, and retransmissions take it, so UE1 originated only
  547 new frames against 1156. At ~50 % loss, four attempts still fail about
  6 % of the time.
- **Link adaptation** moves UE1 to a rate that link can carry. Delivery and
  throughput both recover, and the control UE is untouched.

### Rig caveats found on the way

- **A per-packet power delta also reaches the next beacon.** On Kestrel it is
  a global BB TX target rewritten between frames ("HW beacons follow",
  `AdapterCaps.h`).
  - When the attenuated frame was the last one before the DU's TBTT, the DU
    beacon aired attenuated too. The beacons heard fell from 339 at −10 dB to
    1 at −18 dB, and the UEs lost sync.
  - The test therefore attenuates UE1, whose DL slot is followed by UE2's
    full-power frame.
- **The power delta is not linear.** The UE-reported SNR fell from 32 dB at
  −6 dB to 10 dB at −10 dB. That is not explained here.
- **The SNR thresholds are starting points.** At −10 dB, UE1 reported a DL
  SNR of 4.5 dB in a judged window, below even the 6 Mb/s threshold, so LA
  went to 6 Mb/s. Whether 9 or 12 Mb/s would have held is not
  measured.

## Open

- A DU keeps scheduling, retransmitting and dropping for a UE that has stopped
  uplinking. Admission and departure belong to M3.
- HARQ uses only the DL slot each UE already has. Retransmissions displace new
  data rather than taking spare airtime.
