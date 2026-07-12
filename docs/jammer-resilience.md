# Jammer resilience

How frequency hopping and the fused-FEC link hold up against a narrowband
jammer, and where a *following* jammer stops being able to keep up. Two
experiments, both on the two-adapter + B210 bench (data TX = RTL8812AU, data RX =
RTL8812CU, jammer = USRP B210).

The metric is end-to-end FEC-decoded delivery of the fused-FEC link
(`fused_fec_tx.py → streamtx → rxdemo → fused_fec_rx.py`): `fec_delivery =
recovered packets / sent packets`. For a hopping TX the RX tracks it in lockstep
(the RX follows the same schedule via the sync marker), so a static RX doesn't
mask the jamming by only hearing 1/N of the hops.

## Experiment 1 — parked narrowband jammer

`tests/run_jammer_resilience.sh` parks the B210 on one channel of the hopset and
measures delivery under four transmit strategies:

```sh
sudo ./tests/run_jammer_resilience.sh \
    --modes static_clean,static_jammed,sequential,keyed \
    --jammer-mode cw --jammer-gain 89 --data-tx-pwr 6 --no-jammer-baseline
```

Bench calibration matters: the RX and data TX are near-field (very high SNR), so
a strong link ignores the jammer. Backing the data TX off (`--data-tx-pwr`) sets
a realistic link margin the jammer can actually contest; the jammer's `tx-gain`
is the interferer knob. A concentrated CW tone denies a channel where 20 MHz
noise (its power spread thin) does not.

Measured (channels 36/40/44/48, jammer on ch 40, 50 ms slots):

| mode          | fec_delivery |
|---------------|--------------|
| static @ clean channel  | ~1.00 |
| static @ jammed channel | ~0.00 |
| sequential hop          | ~0.95 |
| keyed hop               | ~0.95 |

Reading: a static link on the jammed channel is fully denied; hopping bounds the
loss to the ~1/N of dwells that land on the jammed channel, and the RS+SBI code
recovers that erasure fraction, so delivery stays ~0.95. **Sequential and keyed
are statistically identical against a blind parked jammer** (repeat runs:
sequential 0.948/0.950, keyed 0.947/0.950) — both spend 1/N of dwells jammed and
a blind jammer can't exploit the order. That equivalence is exactly why the
order has to be unpredictable only against a *reactive* jammer, which is
experiment 2.

## Experiment 2 — following jammer

`tests/sdr_follower_jammer.py` chases the hopping TX. The B210 is a 2×2 (two RX +
two TX frontends off one AD9361), so it senses and jams **at the same time** — no
time-multiplexing:

- RX frontend: a wideband burst (one FFT spans the hopset) finds the strongest
  hopset channel other than the one it's currently jamming — that's the TX.
- TX frontend: a CW tone on the target, retuned when the target moves.

Two strategies, matched to the TX order:

- **reactive** (vs a keyed TX): jam where the TX was last sensed. A hit needs the
  TX still there after the sense+retune latency; once the slot dwell drops below
  that latency the keyed TX has already jumped to an unpredictable channel.
- **predictive** (vs a sequential TX): the public round-robin order lets the
  follower jam the channel it expects *next*, cancelling its own latency.

Measured:

- Full-duplex sense costs ~0.3 ms; the follower's reaction is dominated by the
  B210 TX retune (`set_tx_freq`) at **~3.5 ms**. That retune latency is the floor
  on how fast any reactive follower here can move — and it *favours the
  defender*.
- Chase dynamics diverge sharply. Over a 20 s run against a 50 ms-slot TX the
  **reactive** follower issues ~2100 retunes (constant correction — it is always
  a step behind an unpredictable hop), while the **predictive** follower issues
  ~450 (it pre-positions and holds). The ~5× gap is the keyed schedule forcing
  the jammer into a latency-bound reactive chase.
- A co-channel CW follower does deny the link (a keyed run dropped to ~0.11), but
  delivery-level denial is stochastic run-to-run because the reactive chase is
  only intermittently co-channel.

**Dwell threshold.** Following breaks when the slot dwell falls below the
follower's reaction latency (~3.5 ms here, retune-limited). Below that a reactive
jammer cannot land on a keyed hopper; a predictive jammer against a sequential
hopper is limited only by its retune time, so it holds to much shorter dwells.
The gap between those two thresholds is what a keyed permutation buys.

## Notes / limitations

- The follower needs ≥60 MS/s to span a 60 MHz hopset in one FFT. On a B210,
  61.44/56/50 MS/s trip a UHD tuning assertion (`std::lcm` overflow); 60 MS/s is
  clean, so the follower experiment uses a 3-channel hopset (36/40/44, 40 MHz)
  that fits comfortably.
- A single B210 cannot run a *tight* real-time reactive loop by rebuilding
  streamers per cycle (streamer setup is ~hundreds of ms and rapid RX/TX
  switching corrupts the B200 control channel). Persistent streamers with
  serialized control (send burst → sense → retune, one thread) are stable.
- The hop schedule visits every configured channel; there is no adaptive
  exclusion of a persistently-jammed one — the FEC absorbs those dwells (the
  ~1/N loss in experiment 1).
