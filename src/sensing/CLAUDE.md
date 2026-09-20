# src/sensing/ — the device-touching helper layer

Deep facts for this subtree, loaded alongside the root CLAUDE.md. Namespace
`devourer::sensing`, header-mostly, ctest-gated.

## Why this directory exists

It is the **one** helper subtree under `src/` that calls device methods.
`src/chanmig/` and `src/hopset/` both assert in their own CLAUDE.md that they
are pure — no device access, no env, no clock — and that property is what makes
them testable without hardware. The channel-survey acquisition loop is
inherently impure (it retunes and reads registers), so it lives here rather
than bending either of those contracts.

`src/cell/` is not the same thing despite the root file's wording: it is built
on the device API's *outputs* (it takes raw scalars) and includes no `IRadio.h`.

The contract this subtree keeps, and the reason it can sit next to the pure
ones without eroding them:

- **owns no thread.** `IRadio::StartRxLoop` runs on the caller's thread by
  documented contract, and the caller joins it. Nothing here spawns one.
- **performs no sleep.** The dwell's settle and observation waits are the
  caller's, because their *duration policy* differs per consumer — chanscout
  wants a chunked stop-aware nap so SIGINT stays responsive; a TX-side sensor
  must outlast its own queued frames draining.
- **takes no clock of record.** Timestamps are passed in, chanmig-style. The
  injectable monotonic clock measures durations *within* one call only, and
  exists so the selftest can make every duration an exact integer.
- **reads no env.** The demos map their env onto the config structs.

## The two layers

- **`SenseWindow.h`** — the shared observation discipline: settle, discard
  barrier, observe, read. The sequence, the units, the null-radio case and the
  neutral-vs-Realtek read are documented at the API itself; what belongs here
  is *why* it is shared. The barrier is the reason: the chip's counters are
  delta-on-read, so a reading only describes the channel you are on if a
  throwaway read resets them after the retune has settled and before the window
  opens. Miss it and the record silently carries the previous channel's energy,
  and nothing downstream can tell.

  `examples/tx`'s `hopset_sense_window` still carries its own copy (its comment
  says "the discipline is chanscout's"). Converting it is a later, mechanical
  change; this header is shaped for it, which is why the retune and the frame
  fold are deliberately *not* in this layer.

- **`DwellExecutor.h`** — the survey-shaped layer: three phases
  (`begin`/`barrier`/`finish`) the caller drives, producing a
  `chanmig::SurveyDwell`. The dependency points sensing → chanmig and never
  back.

  It owns one piece of cross-dwell state and it is the subtle one:
  `FastRetune` is the lean **same-width** path, so the first bin dwell after a
  full-width verification dwell must go through the full `SetMonitorChannel`
  gate. Lose that and every later "bin" silently observes at the candidate's
  width — a wrong reading that still looks entirely plausible. The same latch
  is set when a full-gate tune *throws*, because the chip may have been
  part-way reconfigured and its width is then unknown.

- **`SurveyFrameAgg.h`** — the push-side frame fold, written from the RX thread
  and drained at the dwell boundaries. The ours-vs-foreign attribution key is
  **configuration**, not a constant: a library that hardcoded the canonical
  devourer SA would score an integrator's own video as interference.

## Validation

Headless: `dwell_executor` (barrier ordering, the width-restoration rule and
its throw path, counter plausibility at the exact ceiling, the read-failure
latch, frame attribution, a non-Realtek radio still producing evidence, round
bookkeeping against the real `ScanScheduler`). Before this existed the loop was
demo code and a regression in it was findable only on air.

On-air: `tests/chanscout_stress.sh` is the gate — the `survey.dwell` stream
must be field-comparable against a pre-change capture. The width-restoration
rule specifically needs `DEVOURER_SCOUT_FULLWIDTH_MS > 0` against a 40/80 MHz
candidate, because a headless test can only prove the call sequence, never that
the chip actually came back to 20 MHz.

## Known gap: a backend with busy airtime and no phydm counters

`SenseWindow::read` chooses its source by whether the `IRtlRadio*` is
non-null, not by `AdapterCaps::rx_energy_ok`. The RTL8733B derives from
`IRtlRadio`, implements no `GetRxEnergy`, and answers `GetChannelBusy()` only
through an armed CLM window — so on that die the window takes the phydm branch,
gets the all-invalid base energy read, and never calls `GetChannelBusy()`.
`examples/chanscout` constructs exactly that shape (a `dynamic_cast` that
succeeds on the die) and reports neither CLM nor NHM there. Every other backend
is unaffected: on the Jaguar families both flags are true, and on the MT7612U
the cast fails and the neutral branch runs.

Gating the branch on the capability is necessary but not sufficient. An unarmed
`GetChannelBusy()` on that backend falls back to the energy read and yields
nothing, and nothing in this subtree calls `ArmChannelBusy` — the arm needs an
observation window sized to the dwell, which no layer between
`ScanPlanConfig::dwell_ms` and `SenseWindow` carries (`DwellExecConfig` stamps a
settle but no dwell). Arming also changes behaviour on the Jaguar families,
where the NHM read spoils the same engine, which is what `dwell_executor`
pins. So the fix is a capability gate plus a dwell-sized arm, with its own
selftest arm — a design change here, not a rider on a backend port.
