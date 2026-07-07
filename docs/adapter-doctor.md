# Adapter doctor: is this dongle dying?

A Realtek adapter can be electrically dead-ish and still *look* fine:
it enumerates, USB transfers are clean, driver init completes green —
and the radio is stone-deaf. Measured on a degrading RTL8812AU
([#205](https://github.com/OpenIPC/devourer/issues/205)), the failure
lives in places no "did init succeed" check reaches:

- **EFUSE reads return stochastic garbage** — a different EEPROM ID /
  calibration map on every physical read. The driver then configures
  the wrong RFE / PA / LNA tables and the radio hears nothing. A
  healthy chip returns byte-identical maps every read, cold or warm.
- **The MCU never boots firmware** — the download checksum report
  never asserts. On Jaguar1 that is deliberately non-fatal (monitor RX
  runs without the 8051), so nothing aborts.
- The failure **drifts**: the same unit alternates between garbage
  EFUSE, stable-but-wrong reads, and valid-ID-but-still-dead phases.
  Any single-symptom check misses some phase; probing independent
  subsystems catches every one.

`doctor` (examples/doctor) runs the battery and grades it:

```sh
build/doctor                       # first Realtek adapter, ambient traffic
build/doctor --pid 0x8812 --channel 6 --expect-traffic
build/doctor --bus 3 --port 2.3.3  # topology select (two same-PID adapters)
```

1. **Bring-up** — `InitWrite`; an abort is an immediate FAILING.
2. **EFUSE stability** — N fresh *physical* map reads
   (`IRtlDevice::ProbeEfuseStability`), cross-compared byte-for-byte +
   EEPROM-ID (0x8129) validated. Any read-to-read mismatch is
   conclusive by itself. Not probed on the 8822E — its OTP is not
   reliably readable after bring-up by design, so probing would flag
   healthy units.
3. **FW boot** — checksum + MCU-ready outcome of the bring-up's
   download (`IRtlDevice::GetFwBootStatus`).
4. **RX smoke** — FCS-clean frame count over `--listen-secs`. Ambient
   traffic counts. Hearing *nothing* is only SUSPECT unless
   `--expect-traffic` vouches for a source on the channel — an
   RF-quiet room is otherwise indistinguishable from a deaf radio
   (same trap as [startup-time](startup-time.md) benchmarking).

Verdict → exit code: `HEALTHY`=0, `SUSPECT`=1, `FAILING`=2 (3 = tool /
open error), plus a `<devourer-doctor>` machine line and per-reason
text. The classifier is pure logic (`src/AdapterHealth.h`, ctest'd in
`tests/adapter_health_selftest.cpp`); grading rules and their
rationale live next to the code.

## Getting a definitive verdict

One warm run can miss a cold-only pathology (the #205 unit read valid
EFUSE on some warm passes while staying deaf). For a hard verdict,
give the doctor per-rep true cold + vouched traffic:

```sh
sudo bash tests/adapter_doctor_cold.sh <hub> <hubport> <sysfs> <bus> <portpath> [reps]
# bench example — suspect unit on nested smart-hub port 3-2.3.3:
sudo bash tests/adapter_doctor_cold.sh 3-2.3 3 3-2.3.3 3 2.3.3 3
```

Per rep the wrapper VBUS-cycles the DUT's uhubctl-switchable hub port,
keeps the in-tree rtw88 module away (temp blacklist — udev reloads it
on every re-enumeration otherwise, and its probe firmware-download
would contaminate first-touch), runs a radiation-verified beacon flood,
and invokes `doctor --expect-traffic`. Exit code is the worst verdict
across reps.

Validated on the bench pair: the healthy unit grades HEALTHY (stable
0x8129 EFUSE ×4, FW ready, thousands of frames) and the dying unit
grades FAILING on every cold rep — regardless of which face the
sickness shows that day (garbage EFUSE, or valid-ID-with-dead-MCU +
deaf-to-flood).

## Reading a SUSPECT

- `heard nothing (no traffic guarantee)` — re-run with a known traffic
  source on the channel and `--expect-traffic`.
- `EFUSE is blank (autoload fail)` — all-0xFF map. Legitimate on some
  bare dev boards; on a retail adapter it means the calibration data
  is gone.
- `MCU never booted firmware` alone — can be a transient warm-state
  hang (the vendor drivers retry this too). VBUS power-cycle and
  re-run; persistent across cold reps = hardware.

A cheap independent cross-check on any modern kernel: the in-tree
rtw88 driver's probe (dmesg) fails with `failed to validate firmware`
on a unit with this pathology and binds cleanly on a healthy one.
