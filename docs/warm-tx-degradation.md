# Warm-session TX degradation — chip temperature caps the modulation

A devourer session used to leave a Jaguar1 chip powered up. Not just enumerated
— in the ACT power state with the RF front end live, indefinitely, long after
the process that brought it up had exited. Nothing else in the tree does this:
Jaguar2 resets the chip at init and Jaguar3 has run a full card-disable at
teardown for a long time.

The consequence was a transmitter that quietly lost its dense constellations.

## The signature

Across back-to-back sessions on one RTL8812AU, ch6/20 MHz, constant TX config:

| | MCS7 (64-QAM 5/6) | MCS1 control | thermal | RSSI |
| --- | --- | --- | --- | --- |
| cold baseline | 83.0% | 98.0% | 43 | 69 |
| after 24 max-duty sessions | 71.3% | 98.0% | **53** | 69 |
| 120 s idle, chip still powered | 70.8% | 98.3% | 49 | 69 |
| after VBUS cold cycle | 79.2% | 97.7% | **44** | 69 |

**RSSI never moves.** The transmitter is not losing power; it is losing signal
quality, and the chip's own thermal meter tracks the loss one-for-one. The
robust rates are untouched throughout, because they need far less margin.

This is dangerous to diagnose because nothing reports an error: every frame is
submitted, the receiver reports a strong RSSI and a clean EVM *on the frames it
still decodes* (only the low-order ones), and the result is indistinguishable
from a link too weak to carry the constellation. It produced one confident and
completely wrong "rig SNR limit" conclusion before the mechanism was found.

Note what the idle row says: **idling does not cool the part.** 120 s of no
traffic bought 4 thermal units; a 12 s VBUS cycle bought 9. A chip held in ACT
with its RF on keeps dissipating whether or not you are transmitting.

## The fix

`RtlJaguarDevice::Stop()` and the destructor now run
`HalModule::rtw_hal_deinit()` — halt the MAC engines (`REG_CR`, `REG_RCR`) then
the die's card-disable power sequence. Those sequence tables
(`rtl8812_card_disable_flow` and the 8814A/8821A equivalents) had been carried
in `hal/Hal88*PwrSeq.c` since the port began with nothing calling them.

The chip now powers down whenever no session owns it, so it cools. Same repro,
after the fix:

| stage | before | after |
| --- | --- | --- |
| 60 s idle, **no** power cycle | thermal 48, MCS7 73.0% | thermal **40**, MCS7 **85.8%** |

Before, no amount of idling recovered anything and only pulling VBUS helped.
Now an ordinary gap between sessions restores the part completely, without
touching the hardware.

## What this does not fix

**Continuous transmission still heats the chip.** Inside one uninterrupted
high-duty session the part reaches thermal equilibrium and stays there —
~80% → ~75% at MCS7 on this adapter. That is physics, not a defect, and it is
the honest steady-state capability of a hot radio.

There is deliberately **no in-flight recovery API**. Recovery requires the
radio to stop transmitting long enough to cool, and the measurement says that
is around a minute (60 s of powered-down idle moved thermal 47 → 40 and MCS7
75% → 86%). A call that restores the rate by killing the link for a minute is
not a recovery for an airborne link, so shipping one would be theatre. For a
long flight the lever is thermal-aware operation instead: poll the meter with
`DEVOURER_THERMAL_POLL_MS`, watch the `thermal` event's rising `delta`, and
back off with `SetTxPowerOffsetQdb` or reduce duty before the constellation
starts failing.

**A killed process leaves the chip hot.** `SIGKILL` runs neither `Stop()` nor
the destructor, so the chip stays powered and keeps heating until something
opens it again. An init-side power-off was considered and rejected: it would
run immediately before power-on and so buys no cooling time at all, while
adding risk to a bring-up path that works. The real answer for an unattended
deployment is a supervisor that does not `SIGKILL` its radio process.

## Per-chip findings

- **RTL8812AU** — where the effect is characterized. Thermal 43 → 53, MCS7
  −12 points. The fix is validated here.
- **RTL8814AU** — barely heats at the same duty (34 → 37) and shows no
  degradation. It re-inits cleanly after the power-down; there was simply
  nothing to recover. The effect scales with how hot the part actually gets.
- **RTL8821AU** — re-inits cleanly after the power-down; not separately
  characterized for the thermal curve.
- **Jaguar2 (RTL8822BU)** — audited and **inconclusive**: its thermal meter did
  not move (33–34) across the same run, and the link available for the test was
  too marginal at MCS7 to resolve a small effect. No teardown power-down was
  added there on the strength of a non-measurement; it remains the one
  generation that leaves the chip powered at exit.

## Reproducing

```sh
sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3;2357:012d=10,2" \
     tests/warm_tx_degradation_repro.sh
```

Holds one ground receiver up for the whole run so the reference never moves,
then alternates probes with warm sessions, recording delivery at a test rate and
a robust control rate alongside the chip thermal meter and the receiver's RSSI
and EVM. `WARM_PWR=63 WARM_GAP=0` turns the warm sessions into max-power,
max-duty heating. Two controls close it out: an idle with no power cycle, and a
VBUS cycle.

Reading it: falling delivery with **flat RSSI and rising thermal** is heat;
falling delivery with **falling RSSI** would be a transmitter losing power,
which is a different bug.
