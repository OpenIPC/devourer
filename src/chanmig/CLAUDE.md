# src/chanmig/ — adaptive channel migration

Deep facts for this subtree, loaded alongside the root CLAUDE.md. Slow,
evidence-driven whole-link channel moves — the deliberate complement to
per-slot FHSS (`src/hopset/`). Namespace `devourer::chanmig`, header-mostly,
each piece ctest-covered and pure: the library reads no env, the demos map it.
Docs: `docs/adaptive-channel-migration.md`,
`docs/channel-migration-protocol.md`, `docs/channel-migration-validation.md`.

## Five composable layers

- **Scout** (`chanscout`, `ScanPlan.h` / `SurveyRecord.h`): a second adapter
  passively surveys a candidate plan (`DEVOURER_SCOUT_PLAN`) while the primary
  RX stays on the video channel, emitting versioned `survey.dwell` records
  with a counter-hygiene discard barrier — the FA/CCA counters are
  delta-on-read. The dwell itself — retune, settle, barrier, observe, read — is
  `devourer::sensing::DwellExecutor` (`src/sensing/`), so `chanscout` is now the
  env-mapping and event-emitting shell around it and this subtree stays pure.
  v2 records also carry `clm` / `nhm_env`
  (`docs/rx-spectrum-sensing.md`); the parser still accepts v1.
  Measures only; retunes nothing but itself. Grid-legality
  validation, **no regulatory DB** — the caller owns compliance.
- **Scoring** (`ChannelScore`, `DEVOURER_SCOUT_ADVISE`): a pure two-leg
  recommendation engine. The primary receiver's *delivery* is authoritative on
  the active channel (scout energy there is confounded by the wanted video);
  the scout's occupancy is authoritative on candidates. Emits explainable
  `channel.recommend` / `channel.hold`. That occupancy has exactly two terms —
  foreign decoded airtime and a bounded false-alarm rate; `cca_rate`, `igi`, the
  NHM fields and `clm` are recorded in the dwell and deliberately not scored
  (NHM because its naive busy% is generation-dependent).
- **Protocol** (`examples/chanmig --role ground|drone`): an authenticated
  ground-proposes / drone-commits migration — SipHash-MAC'd wire codec
  (`MigWire.h`), pure `MigProposer` / `MigResponder` state machines, random
  per-boot epochs. The drone arms activation only after the ground echoes its
  nonce, and the ground follows the drone's authoritative STATUS; every
  failure-matrix row converges without split-brain. Control frames are their
  own canonical-SA 802.11 frames — video PSDUs are never touched.
- **Automation** (`MigGate`, `DEVOURER_MIG_MODE=off|advisory|manual|automatic`,
  default `advisory`): a pure, deterministic gate hedged with cooldown /
  residency / per-channel backoff / move-cap / probation, plus an operator kill
  switch.
- **Drone-side validation**: legality and caps checks are the free product
  default; the pre-commit probe (`DEVOURER_MIG_PROBE`) is opt-in research with
  a real outage cost. Probation belongs to the gate.

## Validation

Headless: `chanmig_wire_kat`, `chanmig_proto_matrix` (a 14-row failure matrix +
a drop-every-message sweep), `chan_score_policy`, `chanmig_gate_policy`,
`chanmig_clock_math`, plus `dwell_executor` for the acquisition side
(`src/sensing/`).

On-air: `tests/chanmig_endurance.sh`, `tests/chanscout_stress.sh`,
`tests/chanmig_soak.sh`.

**Near-field bench note**: two adapters ~30 cm apart saturate the RX front end
at full power (`link.health` SATURATED), so migration tests reduce TX power
(`DEVOURER_TX_PWR=12`). See `docs/bench-testing-near-field.md`.
