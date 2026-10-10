# Per-frame air-side timing in the stream link

What the transmitter knew about a frame when it sent it — how long the frame
waited on the host, how deep the backlog behind it was, when the producer
captured it — travels with the frame, in header bytes no receiver otherwise
reads, so the ground station can show a measured latency per frame and per
stage instead of an estimate. The idea is kestrel-air's slice-header
telemetry; the clock underneath is devourer's own: the MAC TSF.

## What travels, and where

**Every stream frame** (`streamtx`, `svctx`, `duplex`) carries six bytes in
the 802.11 header's addr3 — the BSSID field of a probe request, which the
stream demos used to fill with the source address and which no consumer reads
(every one keys on addr2 and slices the body at +24). The FEC bodies are
byte-for-byte untouched and the MTU is unchanged:

| bytes | field |
|---|---|
| 0 | version and flags: capture stamp present, TSF present, depth present, async TX |
| 1 | depth: frames handed to the transport whose completion has not been reaped |
| 2–3 | the transmitter's predicted TSF at `send_packet`, low 16 bits in 10 µs units |
| 4–5 | capture → `send_packet`, 10 µs units, clipped at 655 ms |

The version is chosen so the old addr3 contents (the canonical SA) decode as
"no telemetry" rather than as garbage values.

**A periodic marker** (`DEVOURER_STREAM_TIMING=N`, every N data frames, its
own frame like the hop sync marker) carries the absolute pair (predicted TSF,
host clock), the state of the transmitter's host↔TSF fit, and the window since
the previous marker: p50/max of stdin-read→send, of the `send_packet` wall
time (kestrel-air's T_WRITE), of capture→send, the deepest backlog, and how
many frames had a producer stamp. The marker is a probe response with the
canonical SA, and on the parts whose MAC stamps an injected probe response
with its egress TSF, the marker's own timestamp field is the hardware pair
the receiver's clock fit needs.

**The producer's capture time** arrives through the stdin control escape
shared with the duplex demo's live knobs: a length word with its top bit set
introduces `<op><args>`; opcode 4 is `CAPTURE_TS <u64 LE ns, CLOCK_MONOTONIC>`
and applies to the next record. The Python producers emit it with
`--capture-ts` (and `--capture-delay N:MS` for the check below). A stamp taken
at record emission is a proxy; a real encoder stamps at capture.

## The clock, and why the marker must be hardware-stamped

The transmitter never reads a register per frame (the standing send-path
rule: a control transfer is ~200–340 µs on USB). A poller samples `ReadTsf()`
once per 100 ms against the host's monotonic clock and a least-squares line
predicts the TSF at every `send_packet` call. Its residual is the read-latency
jitter, tens of µs. A sample more than 50 ms off the line means the chip's TSF
was reset under the fit (arming the Jaguar1 beacon pulses it; a re-init zeroes
it), and the fit starts over rather than averaging across the jump: a fit
that averages across the beacon arm reads a 62 ms "latency" that decays over
the session as the poisoned intercept washes out (measured on the 8821AU,
which is why the beacon is armed before the fit samples).

The receiver maps each frame's hardware arrival (`tsfl`) onto the
transmitter's TSF with a fit of its own, and that fit is fed **only from
hardware egress pairs**: frames whose timestamp field the transmitter's MAC
wrote at the instant of transmission. A pair built from a software stamp would
fold the mean one-way latency into the fit's offset and leave only jitter.
Which frames qualify was measured, not assumed
(`tests/probe_resp_egress_tsf_check.sh`, a constant in the field and an
independent witness reading it back):

| transmitter | injected probe response / beacon | hardware TBTT beacon |
|---|---|---|
| Jaguar2 8812BU | stamped, 34 µs spread | stamped |
| Jaguar3 8812CU, 8812EU | stamped, 38–41 µs spread | stamped |
| Kestrel 8832CU | stamped, 35 µs spread | stamped |
| Jaguar1 8821AU | **not a TSF**: a counter that is neither TSF port, held ~7 frames at a time | stamped, 3.2 µs spread |

So on Jaguar2, Jaguar3 and Kestrel the marker frame itself is the pair, on
any channel, hopping included. On Jaguar1 the marker's flags say its stamp is
not to be trusted and, on a fixed channel, the transmitter arms the hardware
beacon with the same SA as the clock carrier (`AdapterCaps::hw_injected_mgmt_txtsf`
is the per-part fact). A hopping Jaguar1 session has no egress pair the
beacon can follow and reports stage durations without an absolute latency;
the 8812AU and 8814AU are unmeasured and treated as the 8821AU. The receiver
also restarts its fit when a pair lands more than 50 ms off the line: a
transmitter that restarts resets its TSF.

The per-frame one-way latency is then arrival − unwrapped predicted submit
TSF: host jitter on the transmit side only, hardware on the receive side.

## Measured

`tests/stream_timing_onair.sh`, one CF-924AC (8822BU) witness, 20 s runs,
producer paced at 2 ms. The floor is measured first: identical runs,
repeated, so a later difference has something to be judged against.

| transmitter | runs | submit→air p50 (µs) | run-to-run sd | capture→air p50 | capture→send p50 | depth max | fit residual |
|---|---|---|---|---|---|---|---|
| Jaguar3 8812CU, ch6 | 5 | 98–146 | 3–16 | 148–172 | 30 | 0 | < 1 µs |
| Kestrel 8832CU, ch6 | 2 | 90 | 0 | 126 | 30 | 0 | < 1 µs |
| Jaguar1 8821AU, ch6 (beacon clock) | 5 | 358–1337 | 34 (first three) | 436–1373 | 30–40 | 2–19 | < 1 µs |
| Jaguar3 8812EU, ch36, producer at 15 ms | 2 | 109–122 | 6 | 150 | 30 | 0 | < 1 µs |
| Jaguar3 8812CU, `svctx` (no producer stamp) | 1 | 155 | — | — | — | 0 | < 1 µs |
| Jaguar3 8812CU, `duplex` (TX+RX on one chip) | 1 | 124 | — | 150 | 30 | 0 | < 1 µs |

The checks every cell passes or fails on:

- **The producer delay is recovered.** With `--capture-delay 10:20` the
  producer sleeps 20 ms between the stamp and every 10th record: the receiver
  sees 10.2% (Jaguar3) and 9.8% (Kestrel) of stamped frames standing 20.08
  and 20.09 ms above the rest. The field tracks the producer, not the pipe.
- **Hopping survives.** A slot-hopping Jaguar3 transmitter (1/6/11 at 50 ms)
  keeps the marker and the clock through retunes — 4837 frames with an
  absolute latency in 20 s — and the window's stdin-read→send maximum, 5.3 ms,
  is the retune showing up where it should.
- **Corrupted frames never feed the clock.** With the witness keeping
  CRC-failed frames the fit's residual stays below 1 µs.

The adversarial readings, in the same breath:

- **The depth field is honest about the transport.** Jaguar1's asynchronous
  bulk-OUT shows a backlog of 2 to 19 URBs and a 400 µs to 1.3 ms one-way
  figure, run to run, where the synchronous families show 0 and ~100 µs. That
  is the transport's shape (the async path blocks only at 256 in flight), not
  a defect the telemetry found — and the reason the field exists.
- **A hardware beacon outlives the process that armed it.** On Jaguar1 the
  timing clock rides `StartBeacon`, and a transmitter killed by SIGTERM left
  its beacon airing at 100 TU with the canonical SA; the next transmitter's
  witness then saw two clock sources and its fit reset on every pair. The TX
  demos now end on SIGINT/SIGTERM through their ordinary exit path (beacon
  disarmed, device stopped — verified: zero canonical-SA beacons on the air
  after a timeout-ended run), and the receiver takes beacon pairs only when
  the live marker says a beacon carries the clock.
- **A producer that outruns the chip pins capture→send at its clip.** The
  8812EU on 5 GHz stalls its synchronous send for the full 20 ms bulk-OUT
  timeout now and then (the window's send-time maximum reads 20.99 ms, the
  known 5 GHz flood behaviour of that module, `docs/8822e-quirks.md`); at a
  2 ms producer pace the stdin pipe fills and every capture→send reads
  655 ms — a true reading of the backlog, and useless as a steady-state
  number (that run's submit→air median was 206–235 µs, with a 10 ms p99 from
  frames queued behind the stalls). The harness runs that part at a 15 ms
  pace (`PACE_US`), where it reads like the 8812CU.
- **A record pushed into a chip still coming up wedges the TXDMA.** A duplex
  whose TX thread ran ahead of its bring-up got every send timed out for the
  whole run when fed at once, marker or no marker. duplex now brings the chip
  up synchronously before its TX thread exists and emits `stream.ready`; the
  harness feeds it immediately and passes. The timing fit still arms on the
  first record rather than at thread start, so no register read or beacon arm
  lands inside a bring-up on any demo.
- **The first seconds are the pipe, not the link.** The producer fills the
  stdin pipe while the chip is brought up, so the first ~1000 records arrive
  with stamps seconds old. The analyzer drops a 4 s warm-up for that reason.
- **svctx has no producer stamp.** It pre-reads and replays a clip, so its
  capture→send is each NAL's loop iteration; a live stdin mode is a follow-up.

## Reading it

`rxdemo` with `DEVOURER_STREAM_OUT=1`: every `rx.frame` carries `fc0` (so a
consumer can keep the marker and a Jaguar1 beacon out of its video
accounting: only `0x40` frames carry the stream envelope) and `a3`, and when
addr3 decodes, `tel`, `depth`, `c2s_us`, `cap`, and — once the receiver has
the transmitter's clock — `lat_us` and `c2a_us`. A receiver without a
hardware RX stamp (`hw_rx_timestamp` false, the MT7612U) decodes the field
and the marker but never fits a clock or reports a latency. Each marker is an `rx.timing`
event with the transmitter's window and this receiver's fit state; the
transmitter logs the same window as `stream.timing`. Schema: `docs/logging.md`.
`tests/stream_timing_analyze.py` turns one capture into a summary line and the
checks above.
