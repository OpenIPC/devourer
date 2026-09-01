# Logging

Devourer's output is split into two planes:

| Plane | Stream | Format | Audience |
|---|---|---|---|
| **Machine events** | stdout | JSON Lines — one JSON object per line | test scripts, AI agents, telemetry consumers |
| **Human diagnostics** | stderr | `devourer [I] message` (level letter T/D/I/W/E) | people debugging |

`rxdemo 2>/dev/null` therefore yields a pure JSONL stream; `2>&1` into one log
is also safe — event parsers skip non-event lines.

## The event contract

Every event line serializes the event name **first**, exactly:

```json
{"ev":"rx.txhit","hits":3,"total_rx":10,"len":247}
```

so shell consumers may `grep -F '"ev":"rx.txhit"'` without a JSON parser, and
everything richer goes through `json.loads` — see `tests/devourer_events.py`
(`iter_events` / `parse_event`), the shared helper the python test scripts use.

Conventions:

- Event names are lowercase dotted namespaces (`rx.frame`, `txpwr.state`).
- Register addresses/values/masks are hex **strings** (`"addr":"0x0808"`).
- Per-chain metrics are arrays (`"rssi":[-52,-60]`).
- A field the chip can't provide is JSON `null`.
- `t` is monotonic **ms since process start**; periodic/marker events carry it,
  per-frame events rely on `seq`/`tsfl` instead.
- Byte blobs (frame bodies, C2H payloads) are lowercase hex strings.
- A line is hard-capped at 64 KiB: an overflowing field is dropped (→ `null`)
  and `"truncated":true` is appended.

Emission is a single `fwrite` of the complete line followed by `fflush`
(`src/Event.h`) — per-line atomicity across threads (RX loop / coex / TX), and
a piped consumer (python subprocess, AI agent) sees each event immediately;
there is no libc full-buffering stall. `DEVOURER_EVENT_FLUSH=0` drops the
per-line flush for max-rate benches.

## Demo environment knobs

Mapped in `examples/common/env_config.cpp` (`apply_logging_env`); the library
itself reads no environment — programmatic consumers configure the same things
on the `Logger` object (`set_level`, `set_diag_stream`, `events().configure`).

| Var | Values | Meaning |
|---|---|---|
| `DEVOURER_LOG_LEVEL` | `trace`,`debug` (default),`info`,`warn`,`error`,`silent` | stderr diagnostic verbosity |
| `DEVOURER_EVENTS` | `stdout` (default), `stderr`, `off` | event-stream destination — `sense` and `streamtx` default to stderr themselves (their stdout is a display / data path) |
| `DEVOURER_EVENT_FLUSH` | `0` | disable per-event flush |

## Compile-time gating

`DEVOURER_LOG_MAX_LEVEL` (CMake cache var: `TRACE`/`DEBUG`/`INFO`/`WARN`/
`ERROR`/`SILENT`) is the compile-time floor: calls below it — including their
argument expressions at `DVR_TRACE`/`DVR_DEBUG` macro sites — compile to
nothing. Unset, the floor derives from `NDEBUG`: release builds drop
trace/debug, debug builds keep everything. Production firmware build:

```sh
cmake -S . -B build -DDEVOURER_LOG_MAX_LEVEL=WARN
```

Hot-path trace/debug sites in the library go through `DVR_TRACE(logger, ...)`
/ `DVR_DEBUG(logger, ...)` so a disabled level also skips evaluating the
arguments (one branch). `info/warn/error` are plain `logger->` methods.

## Event schema

Emitters: L = library, RX/TX/... = demo. Optional fields in [brackets];
`|null` marks fields that go null when the chip doesn't expose them.

### Init / infrastructure
| ev | emitter | fields |
|---|---|---|
| `init.timing` | L (`src/InitTimer.h`) + demos | stage ("scope.stage", e.g. "demo.first_rx_frame", "txdemo.first_tx_submit"), ms |
| `adapter.caps` | RX, TX, doctor, txpower (`examples/common/caps_event.h`) | supported, chip, names, chip_id "0x..", gen, variant, transport, tx_chains, rx_chains, n_ss, stbc, ldpc, sgi, bw_max, bw[] (MHz), txpwr_max, txpwr_step_qdb, txpwr_step_measured, txpwr_min_qdb, txpwr_max_qdb, txpwr_rate_diffs, txpwr_rate_diffs_hw, txpwr_rate_diffs_measured, tune_2g4[]\|null, tune_5g[]\|null, char_2g4[]\|null, char_5g[]\|null, ldpc_rx_ht, ldpc_rx_vht, ldpc_rx_flag, per_pkt_txpwr, narrowband, fastretune, ack_responder, tx_retry_limit, he_er_su, per_chain_rssi |
| `debug.wreg` | L (`DEVOURER_LOG_WRITES`) | addr "0x0nnn", width, val "0x…" |
| `hop.prof` | L (`DEVOURER_HOP_PROF`) | gen, ch, `<stage>_us`…, total_us |
| `tx.fail` | L (send failure; regress.py keys on it) | {status, actual_len, timeout} or {rc, timeout} |

### RX plane
| ev | emitter | fields |
|---|---|---|
| `rx.pkt` | RX | n, len (first 10 + every 100th frame) |
| `rx.frame` | RX (`DEVOURER_STREAM_OUT`), duplex | rate, len, crc, icv, rssi[2], evm[2], snr[2], seq, tsfl, bw, stbc, ldpc, sgi, paggr, ppdu, fc1 (FC flags byte; bit3 = 802.11 RETRY), sa hex, body hex; `tx_tsf` (sender's hardware egress TSF) on beacons/probe-responses only. SA gate: canonical-SA with `DEVOURER_RX_AGG_SA` unset; when set, that filter ("canon"/mac/"any") selects the stream instead |
| `rx.body` | RX (`DEVOURER_DUMP_BODY`) | rate, rssi[2], evm[2], snr[2], crc, len, body hex |
| `rx.corrupt` | RX (`DEVOURER_RX_DUMP_ALL`) | len, crc, icv, rate, bw, stbc, ldpc, sgi, rssi[2], evm[2], snr[2] |
| `rx.txhit` | RX, TX | hits, total_rx, len, seq, paggr, ppdu, rate, bw, stbc, ldpc, ppdu_type — canonical-SA (57:42:75:05:d6:00) matcher; rate/ldpc prove what encoding was decoded (8814A reports ldpc=0 always — no HW indicator); ppdu_type is the AX RXD format nibble (7=HE_SU, 8=HE_ERSU; 255 on pre-AX chips) |
| `rx.seq` | RX, duplex (`DEVOURER_RX_PCTR`) | pctr, tsfl, seq, crc, paggr, ppdu, rate (hw rate index of this copy — retransmissions of one pctr can air at different rates, so per-copy rate + tsfl order reconstructs the fw fallback ladder on air, `tests/retry_ladder_probe.sh`) — the ground-truth per-frame delivery sequence for the RX-ring loss study: pctr is the u32 the txdemo QoS-Data path stamps at MPDU offset 26, so gaps in it are per-frame loss; paggr/ppdu carry the aggregate structure the host-vs-RF discriminator keys on. Lean by design (no body hex) so the emit can't perturb the pump thread. SA gate follows `DEVOURER_RX_AGG_SA` (required in duplex, whose canonical-SA `rx.frame` stream is a different transmitter), else canonical SA |
| `rx.blockack` | RX (`DEVOURER_RX_CONTROL`) | crc, rate, ctrl, start_seq, ra hex, ta hex, bitmap hex — one event per received 0x94 BlockAck control frame (bitmap carries the first 64 bits); narrow metadata-only stream used by `tests/rtl8733b_blockack_onair.sh` to prove the response subtype/address/bitmap without mirroring every A-MPDU data body |
| `rx.parse_abort` | RX (every generation, always on) | t, off, buf_len, remaining, frame_len, drvinfo, shift, total (cumulative) — the RX descriptor walk abandoned the rest of a bulk-IN aggregate; semantics doc-commented at `src/RxParseAbort.h`. No event in a session = the walk never aborted |
| `rx.ring` | L (`DEVOURER_RX_RING_MS`) | t, mode ("async"/"sync"/"reorder-pool"/"spsc-fat"), n_urbs, armed (URBs posted to the HCD and awaiting a frame — the depth that starves under a slow inline consumer), min_armed (low-water mark since the last emit), cb_max_us (worst inline-consume latency in the window), resubmit_fail, completions (cumulative URB callbacks), empties (cumulative callbacks that left the ring with zero posted URBs), pool_free (−1 = no host pool), qdepth (spsc-fat consumer-queue depth; 0 in the other ring modes), pool_dropped (cumulative received frames discarded at spsc-fat pool exhaustion under the `drop` policy — pool exhausted, or a failed re-arm, the latter also ticking resubmit_fail; each was already chip-ACKed, so a hardware-ARQ peer counts it delivered), pool_stalls (the `backpressure`-policy counterpart, cumulative URB park events: the payload still reaches the consumer, the ring shrinks and the chip declines further ACKs, so overload loss stays ARQ-visible — `DEVOURER_RX_POOL_EXHAUST`, default backpressure; `tests/arq_e2e_delivery.sh` measures both). Sync mode emits a reduced line (pool_free pinned at −1; no qdepth/pool_dropped/pool_stalls/completions/empties). The mechanism-proof telemetry: empties/completions is the host-starvation rate — near-0 under RF loss (the ring stays armed because frames don't arrive), high under host starvation (frames out-race resubmit and drain the ring). Counted in the callback, so robust to the pump-thread starvation that makes the periodic emit sparse — but blind while the pump itself is frozen: a stalled consumer drops frames these counters never see, which the per-frame `rx.seq` ledger exists to catch |
| `rx.count` | TX (its RX thread) | total, len |
| `rx.path` | RX (`DEVOURER_RX_ALLPATHS`) | seq, rssi[4], snr[4], evm[4] |
| `rx.path_mask` | L (toggle spec) | t, mask "0xNN" |
| `rx.scrambler` | RX (`DEVOURER_DUMP_SCRAMBLER`) | seed "0xNN", rate, hits, len |
| `rx.energy` | RX (`DEVOURER_RX_ENERGY_MS` / sweep) | t, [ch], cca_ofdm\|null, cca_cck\|null, fa_ofdm\|null, fa_cck\|null, igi\|null, abs_noise_floor_dbm\|null, [retune_us], frames, frames_ldpc, frames_stbc, crc_err, icv_err, rssi_mean, rssi_max, snr_mean, snr_min, evm_mean — crc/icv nonzero only under `DEVOURER_RX_KEEP_CORRUPTED` (the parser drops failed frames otherwise) |
| `rx.nhm` | RX | [ch], peak, busy, dur, hist[12] |
| `rx.quality` | RX (`DEVOURER_RXQUALITY`) | verdict, frames, rssi_mean_dbm, rssi_max_dbm, snr_mean_db, snr_min_db, evm_db\|null, noise_floor_dbm\|null, abs_noise_floor_dbm\|null, igi |
| `adapter.rxpaths` | RX (`DEVOURER_RXQUALITY`) | active_mask "0xNN", n_active, n_chains, frames, rssi_dbm[], snr_db[], evm_db[] — GetActiveRxPaths live per-chain activity (the caps rx_chains companion); snr_db/evm_db only when a chain carried the metric this window (EVM -128 no-stream rail excluded) |
| `link.health` | RX (`DEVOURER_LINKHEALTH`) | verdict, rssi_dbm, snr_db, evm_db\|null, frames, fa_ofdm\|null, igi\|null, [igi_floor], [igi_ceil], cause, fix |
| `fw.c2h` | RX, duplex (`DEVOURER_TX_STATUS`) | len, bytes hex |

### TX plane
| ev | emitter | fields |
|---|---|---|
| `tx.frame` | TX | n, rc — precoder demo variant: n, ok |
| `tx.stats` | TX | submitted, failed, was_timeout, last_rc |
| `tx.agg` | L (`DEVOURER_TX_USB_AGG`, send_packets) | frames, bytes, shim, ok — one per multi-frame bulk-OUT URB. The sync-TX generations (Jaguar2/Jaguar3/RTL8733B) also emit `sent` — bytes actually transferred, OR the negative libusb rc on a transport error (deliberately raw: this event is the only machine-readable carrier of the aggregated-path error code) — and set `ok` only on a FULL write, so `ok=false` splits as `sent < 0` transport error vs `0 <= sent < bytes` short write. Jaguar1 TX is async: its `ok` means URB accepted by the transport and there is no `sent` field (bytes resolve at completion reaping) |
| `tx.report` | L (`DEVOURER_TX_REPORT`, CCX decode) | t, state (0=delivered, 1=retry-drop), ok, retries, final_rate, queue_time_raw, bmc, macid, fmt ("8812"\|"halmac"); halmac adds tag (SW_DEFINE echo), rts_retries, missed (fw-stuffed constant on Jaguar3 — tag gaps are the drop signal; `tests/txrpt_coverage_attrib.py`) — t is the achieved-report-rate timebase (the CCX emission ceiling is reports/s) |
| `tx.status` | RX, duplex (C2H TX_RPT decode) | hoff, queue, retry, airtime_us, rate |
| `tx.receipt` | TX (its RX thread, `DEVOURER_TX_RECEIPTS`) | t, fresh, total, covered, receipts, tlv hex — one event per absorbed windowed RX receipt (src/cell/RxReceipt.h), WITH the raw TLV so `tests/receipt_verify.py` can replay the merge and compare frame-exactly against the receiver's rx.seq ledger; decimating would break that comparison |
| `receipt.tx` | duplex (`DEVOURER_RX_RECEIPT_MS`) | t, n, ok, tlv_len, late — receipt-injection cadence; late counts frames that arrived below the receipt window (reordered past coverage — size the window per the RxReceipt.h sizing rule) |
| `tx.queue` | RX (`DEVOURER_QUEUE_POLL_MS`, 8814) | q1…q5 "0x%08x" |
| `tx.contx` | TX (continuous mode) | mcs, t_ms |

### TX power / thermal
| ev | emitter | fields |
|---|---|---|
| `txpwr.set` | TX | index, t_ms |
| `txpwr.readback` | TX | index, cck1m, ofdm6m, mcs7, rb |
| `txpwr.state` | txpower | flat, offset_qdb, steps, satlo, sathi, cck, ofdm, mcs7, rb |
| `txpwr.caps` | txpower | supported, max, step_qdb, step_measured, min_qdb, max_qdb, rate_diffs, rate_diffs_hw (0 = software send-time fold), rate_diffs_measured |
| `txpwr.offset` | txpower | requested, applied |
| `thermal` | RX, TX, txpower | t, raw, baseline\|null, [delta], status |

### Hopping
| ev | emitter | fields |
|---|---|---|
| `hop.dwell` | TX, duplex | dwell, round, channel, frame, switch_us, t_ms, [mode] |
| `hop.done` | TX, duplex | frames, dwells |
| `hop.rx` | RX (lockstep, `DEVOURER_HOP_CHANNELS`+`_SLOT_MS`) | state (acquire\|track\|retune\|decode\|lost) + per-state fields: channel, slot, epoch, retune_us, dead_us |
| `hopset.propose` | TX/RX (`DEVOURER_HOP_ADAPTIVE`) | t, v, role (tx\|rx), slot, gen, mask "0x…", obs, reasons "0x…" |
| `hopset.commit` | TX/RX (`DEVOURER_HOP_ADAPTIVE`) | t, v, role, slot, gen, mask "0x…", activate_round, activate_slot — TX = commit issued, RX = commit adopted as pending |
| `hopset.activate` | TX/RX (`DEVOURER_HOP_ADAPTIVE`) | same fields — the committed state swapped in at the boundary |
| `hopset.reject` | TX/RX (`DEVOURER_HOP_ADAPTIVE`) | t, v, role, slot, reason (`hopset_reason_name`: bad_mac…timeout) |
| `hopset.gen_mismatch` | RX (`DEVOURER_HOP_ADAPTIVE`) | t, v, role, slot, seen_gen, cur_gen, cur_mask "0x…" — a v2 marker advertised a state we don't hold |
| `hopset.recover` | RX (`DEVOURER_HOP_ADAPTIVE`) | commit/activate fields — re-synchronized from an authenticated commit/status after a missed transition |
| `hopset.sense` | TX (`DEVOURER_TX_SENSE`) | t, v, role, slot, round, gen, base_idx, ch, phase (pre\|post), probe, window_us (MEASURED, barrier-read-end→read-end), settle_us, read_us, valid_fa + cca_ofdm/cca_cck/fa_ofdm/fa_cck + cca_rate/fa_rate (per ms), valid_igi + igi, valid_nhm + nhm_busy/nhm_dur, flags "0x…" (`TxSenseFlag`), scored, occ — one quiet-window observation |
| `hopset.decision` (TX) | TX (`DEVOURER_TX_SENSE`) | the RX envelope plus mode (rx\|veto\|either\|failsafe), origin (rx\|tx\|fused\|failsafe), kind (hold\|exclude\|restore\|delay), veto (`fusion_veto_name`: none\|tx_broad\|tx_alt_dirty\|tx_sensor_absent\|tx_insufficient), rx_wanted, tx_wanted, disagree, tx_valid, tx_target_occ/tx_band_min/tx_band_max, fb_age (rounds or "never"), fusion "0x…" — both endpoints' views on one line, so a log reader sees the argument and not just the conclusion |
| `hopset.decision` | RX (`DEVOURER_HOP_POLICY`) | t, v, role, slot, round, kind (hold\|exclude\|restore), obs, policy "0x…" (config hash), then either hold (`hopset_hold_name`: insufficient_rounds\|cooldown\|proposal_outstanding\|no_impairment\|not_persistent\|broad_degradation\|no_healthy_alternative\|min_active_floor\|max_excluded_frac\|probe_hysteresis_pending) or mask "0x…" + reasons "0x…" (bit0 delivery_floor, 1 persistent, 2 healthy_alt, 3 crc_dominant, 4 energy_interference, 5 weak_signal, 6 sync_loss, 7 probe_recovery, 8 probe_evidence) + target; `DEVOURER_HOP_POLICY_EVENTS=2` adds holds and per-channel c0..cN ("on\|off d=<delivery> p=<probe delivery> v=<visits> imp=<impaired run>") |
| `hopset.probe` | TX/RX (`DEVOURER_HOP_PROBE_ROUNDS`) | t, v, role, slot, round, base_idx, ch, delivered, frames — one keyed recovery-probe dwell on an excluded channel |

### Channel migration (chanscout — docs/adaptive-channel-migration.md)
| ev | emitter | fields |
|---|---|---|
| `scout.id` | chanscout | t, role, usb_id "vvvv:pppp", bus, port, usb_speed, chip, gen, scout_id "0x…" (calibration-domain key), [note] |
| `scout.plan` | chanscout | t, v, plan "0x…" (immutable plan hash), epoch_unix_ms (one realtime sample for offline stream alignment), dwell_ms, settle_ms, backup_ms, bg_ms, fullwidth_ms, max_age_ms, n_candidates, spec |
| `scout.cand` | chanscout | i, chan (canonical "band:primary/width[u\|l]"), center_mhz, backup, no_ir, dfs, bins[] |
| `survey.dwell` | chanscout (`src/chanmig/SurveyJsonl.h` owns the schema) | v, seq, chan, round, plan "0x…", start_ms, end_ms, retune_us, settle_ms, observe_ms, cca_ofdm\|null, cca_cck, fa_ofdm, fa_cck, igi\|null, nhm_busy\|null, nhm_peak, nhm_dur, nhm[12], frames, rssi_mean, rssi_max, snr_mean, snr_min, evm_mean\|null, dvr_frames, dvr_air_us, oth_air_us (canonical-SA vs foreign decoded-airtime split), flags (validity bitmask), scout_id "0x…", agen |
| `scout.health` | chanscout | t, state (ok\|degraded\|wedged\|fatal), reason (retune_fail\|energy_read_fail\|rx_stalled\|usb_congested\|thermal\|stale_survey\|plan_error), detail — emitted on state transitions only |
| `channel.recommend` | chanscout (`DEVOURER_SCOUT_ADVISE`) | t, v, gen "0x…", plan "0x…", policy "0x…", from, to, reason (RecommendBetterCandidate), active_verdict, active_domain, impaired_windows, score, conf, occ, rounds, obs_ms, text — advisory only, retunes nothing |
| `channel.hold` | chanscout (`DEVOURER_SCOUT_ADVISE`) | same envelope as recommend (no to/score), reason (HoldActiveHealthy\|HoldImpairmentNotPersistent\|HoldImpairmentNotChannel\|HoldBroadDegradation\|HoldCooldown\|HoldPrimaryTelemetryStale\|HoldNoQualifiedCandidate\|HoldImprovementMargin\|HoldScoutUnhealthy), text = the counterfactual |
| `channel.ranking` | chanscout (`DEVOURER_SCOUT_ADVISE`) | t, gen "0x…", n, c0..cN ("chan q=<0\|1> score occ rej") — the full per-candidate ranking, so a dashboard renders the counterfactual from the log alone |
| `channel.reject` | chanscout (`DEVOURER_SCOUT_ADVISE`) | t, gen "0x…", chan, reason (Rej*), occ, age_ms, rounds |
| `migrate.id` | chanmig | t, role (ground\|drone), chip, source, link "0x…", epoch "0x…" |
| `migrate.state` | chanmig | t, role, code (MigState value) — one per state transition |
| `migrate.retune` | chanmig | t, role, chan — a retune executed at activation / follow / recovery |
| `migrate.op` | chanmig (ground) | t, cmd — an operator command echoed for audit (stdin is local) |
| `migrate.status` | chanmig (ground) | t, state, chan, clock_ready, resid_p99_us — on the `status` operator command |
| `migrate.done` | chanmig | t, role, code (0=confirmed on new, 1=held/rolled-back to old, 2=rescue) |
| `migrate.gate_notify` | chanmig | t, role, result (0=confirmed, 1=rolledback) — for the #279 automation gate |
| `migrate.drop` | chanmig (`DEVOURER_MIG_DROP`/`_DROP_RX`) | dir (tx\|rx), type — a deterministic fault-injection drop |
| `migrate.gate` | chanmig (ground, `DEVOURER_MIG_MODE`) | t, mode, verdict (0=hold,1=propose,2=abort,3=rollback), reason (GATE reason string) — the #279 automation gate decision |
| `migrate.shadow` | chanmig (`DEVOURER_MIG_SHADOW`) | t, to, gen — a gate Propose recorded but NOT actuated (shadow-actuation ladder rung) |
| `migrate.probe` | chanmig (drone, `DEVOURER_MIG_PROBE`) | t, busy, valid — a variant-B pre-commit probe's measured destination occupancy |
| `migrate.validation` | chanmig (ground) | t, chan, method (0=checks,1=probe,2=probation,3=scout), result (0=accept,1=veto,2=unknown), reason, nhm_busy, valid — the drone's TargetValidation, reporting-only |
| `migrate.disagree` | chanmig (ground) | t, chan, ground_view, drone_view — the drone vetoed a target the ground's scout called clean (asymmetric-interference diagnostic) |

### Beamforming / CSI
| ev | emitter | fields |
|---|---|---|
| `bf.report` | BfReportDetect.h | kind, n, sa, nc, nr, bw, ng, len |
| `bf.any` | BfReportDetect.h | fc "0xNNNN", cat, act, crc, len |
| `bf.report_raw` | BfReportDetect.h, sense (stderr) | frame hex |
| `bf.csi` | BfReportDetect.h (mode 3) | len, csi hex |
| `csi.hit` / `csi.wedged` | RX (`DEVOURER_RX_DUMP_CSI`) | hit, selector "0x…", value "0x…" / selector |

### Stream demos / misc
| ev | emitter | fields |
|---|---|---|
| `stream.rx` / `stream.ctl` / `stream.eof` / `stream.tx` | duplex (stdout), streamtx (stderr) | hits / op, len / tx_count, [bytes] / n, ok, psdu, [total] |
| `stream.done` | streamtx (stderr) | sent |
| `svc.stats` | svctx | frames, crit, t0, t1, t2, t3plus |
| `doctor.verdict` | doctor | verdict, reasons "0x…", efuse_reads, efuse_mismatch, efuse_bad_id, efuse_id, fw_attempted, fw_ready, rx_ok, rx_crc, init |

## Not JSON by design

Register dumps meant for line-by-line diffing against kernel output keep their
text format on the **diagnostic plane** (stderr, `devourer [I] ` prefix):
the `DEVOURER_DUMP_CANARY` canary block (`KIND 0xADDR = 0xVALUE`, matching
`tools/canary_kernel_dump.sh` / `iwpriv read`; `tests/canary_diff.py` strips
the prefix itself), `DEVOURER_BB_DUMP`, `DEVOURER_EFUSE_DUMP`,
`DEVOURER_LOG_TXPWR`. The doctor tool's human report stays on stdout — its
machine summary is the `doctor.verdict` event.

## Schema stability

Event names and listed fields are an interface: test scripts and out-of-tree
consumers (OpenIPC integrations) parse them. Additive changes (new fields, new
events) are safe; renames/removals need the same-PR consumer sweep this schema
landed with. This table is the source of truth — update it with any emitter
change.
