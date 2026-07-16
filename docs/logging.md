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
- A field the chip can't provide is JSON `null` (the old text format used `-`).
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
| `adapter.caps` | RX, TX, doctor, txpower (`examples/common/caps_event.h`) | supported, chip, names, chip_id "0x..", gen, variant, transport, tx_chains, rx_chains, n_ss, stbc, ldpc, sgi, bw_max, bw[] (MHz), txpwr_max, txpwr_step_qdb, txpwr_step_measured, txpwr_min_qdb, txpwr_max_qdb, tune_2g4[]\|null, tune_5g[]\|null, char_2g4[]\|null, char_5g[]\|null, ldpc_rx_ht, ldpc_rx_vht, ldpc_rx_flag, per_pkt_txpwr, narrowband, fastretune, he_er_su, per_chain_rssi |
| `debug.wreg` | L (`DEVOURER_LOG_WRITES`) | addr "0x0nnn", width, val "0x…" |
| `hop.prof` | L (`DEVOURER_HOP_PROF`) | gen, ch, `<stage>_us`…, total_us |
| `tx.fail` | L (send failure; regress.py keys on it) | {status, actual_len, timeout} or {rc, timeout} |

### RX plane
| ev | emitter | fields |
|---|---|---|
| `rx.pkt` | RX | n, len (first 10 + every 100th frame) |
| `rx.frame` | RX (`DEVOURER_STREAM_OUT`), duplex | rate, len, crc, icv, rssi[2], evm[2], snr[2], seq, tsfl, bw, stbc, ldpc, sgi, paggr, ppdu, fc1 (FC flags byte; bit3 = 802.11 RETRY), body hex; `tx_tsf` (sender's hardware egress TSF) on beacons/probe-responses only |
| `rx.body` | RX (`DEVOURER_DUMP_BODY`) | rate, rssi[2], evm[2], snr[2], crc, len, body hex |
| `rx.corrupt` | RX (`DEVOURER_RX_DUMP_ALL`) | len, crc, icv, rate, bw, stbc, ldpc, sgi, rssi[2], evm[2], snr[2] |
| `rx.txhit` | RX, TX | hits, total_rx, len, seq, paggr, ppdu, rate, bw, stbc, ldpc, ppdu_type — canonical-SA (57:42:75:05:d6:00) matcher; rate/ldpc prove what encoding was decoded (8814A reports ldpc=0 always — no HW indicator); ppdu_type is the AX RXD format nibble (7=HE_SU, 8=HE_ERSU; 255 on pre-AX chips) |
| `rx.count` | TX (its RX thread) | total, len |
| `rx.path` | RX (`DEVOURER_RX_ALLPATHS`) | seq, rssi[4], snr[4], evm[4] |
| `rx.path_mask` | L (toggle spec) | t, mask "0xNN" |
| `rx.scrambler` | RX (`DEVOURER_DUMP_SCRAMBLER`) | seed "0xNN", rate, hits, len |
| `rx.energy` | RX (`DEVOURER_RX_ENERGY_MS` / sweep) | t, [ch], cca_ofdm\|null, cca_cck\|null, fa_ofdm\|null, fa_cck\|null, igi\|null, [retune_us], frames, frames_ldpc, frames_stbc, rssi_mean, rssi_max, snr_mean, snr_min, evm_mean |
| `rx.nhm` | RX | [ch], peak, busy, dur, hist[12] |
| `rx.quality` | RX (`DEVOURER_RXQUALITY`) | verdict, frames, rssi_mean_dbm, rssi_max_dbm, snr_mean_db, snr_min_db, evm_db\|null, noise_floor_dbm\|null, igi |
| `adapter.rxpaths` | RX (`DEVOURER_RXQUALITY`) | active_mask "0xNN", n_active, n_chains, frames, rssi_dbm[] — GetActiveRxPaths live per-chain activity (the caps rx_chains companion) |
| `link.health` | RX (`DEVOURER_LINKHEALTH`) | verdict, rssi_dbm, snr_db, evm_db\|null, frames, fa_ofdm\|null, igi\|null, [igi_floor], [igi_ceil], cause, fix |
| `fw.c2h` | RX, duplex (`DEVOURER_TX_STATUS`) | len, bytes hex |

### TX plane
| ev | emitter | fields |
|---|---|---|
| `tx.frame` | TX | n, rc — precoder demo variant: n, ok |
| `tx.stats` | TX | submitted, failed, was_timeout, last_rc |
| `tx.agg` | L (`DEVOURER_TX_USB_AGG`, send_packets) | frames, bytes, shim, ok — one per multi-frame bulk-OUT URB |
| `tx.report` | L (`DEVOURER_TX_REPORT`, CCX decode) | state (0=delivered, 1=retry-drop), ok, retries, final_rate, queue_time_raw, bmc, macid, fmt ("8812"\|"halmac"); halmac adds tag (SW_DEFINE echo), rts_retries, missed |
| `tx.status` | RX, duplex (C2H TX_RPT decode) | hoff, queue, retry, airtime_us, rate |
| `tx.queue` | RX (`DEVOURER_QUEUE_POLL_MS`, 8814) | q1…q5 "0x%08x" |
| `tx.contx` | TX (continuous mode) | mcs, t_ms |

### TX power / thermal
| ev | emitter | fields |
|---|---|---|
| `txpwr.set` | TX | index, t_ms |
| `txpwr.readback` | TX | index, cck1m, ofdm6m, mcs7, rb |
| `txpwr.state` | txpower | flat, offset_qdb, steps, satlo, sathi, cck, ofdm, mcs7, rb |
| `txpwr.caps` | txpower | supported, max, step_qdb, step_measured, min_qdb, max_qdb |
| `txpwr.offset` | txpower | requested, applied |
| `thermal` | RX, TX, txpower | t, raw, baseline\|null, [delta], status |

### Hopping
| ev | emitter | fields |
|---|---|---|
| `hop.dwell` | TX, duplex | dwell, round, channel, frame, switch_us, t_ms, [mode] |
| `hop.done` | TX, duplex | frames, dwells |

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
