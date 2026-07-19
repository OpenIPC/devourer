# RTL8852C (RTL8852CU / RTL8832CU) — definitive quirks

New to the vocabulary used here (DMAC/CMAC, IQK, DIG, physts, NHM, efuse…)? The
[visual driver primer](driver-primer.md) defines all of it with animations.

The Kestrel **C8852C** die (WiFi-6 / 802.11ax, `35bc:0101` on the reference
adapters, die-id `0x52`) as devourer drives it today. Each entry states what the
chip does, what devourer does about it, the residual cost if any, and the
reproducer that proves it. The 8852C shares one HAL with the C8852B; most of its
divergences are the `_V1` register-bank moves called out in
`src/kestrel/CLAUDE.md`. This file is for the behavioural quirks that aren't a
simple bank remap.

## The frame-free NHM absolute noise floor is 2.4 GHz-only (firmware DIG)

devourer's active/frame-free absolute noise floor (`DEVOURER_RX_NOISE_FLOOR` →
`RxEnergy.abs_noise_floor_dbm`) is the halbb env-monitor NHM: it triggers an
in-band idle-power measurement and reports `nhm_pwr − 110` dBm without needing
any received frames. On the C8852C it is **enabled on 2.4 GHz only**; on 5 GHz it
falls back to the passive rssi−snr floor (`GetRxQuality`).

### What the chip does

On 2.4 GHz the NHM reads correctly and stably — on-air it lands near −90 dBm and
cross-matches the passive floor within ~2 dB.

On 5 GHz the NHM reads **correctly for ~1.5 s after a channel tune, then locks
~24 dB high** for the rest of the session (e.g. ch36 `−93 → −70`, ch149 `−68`).
Each fresh channel tune resets it to correct for another ~1.5 s, then it
re-locks. The result is bimodal, not a fixed offset, so it cannot be corrected
by a constant per-band bias.

### Root cause — the firmware DIG biases the NHM's measurement reference

It is **not** an RX gain change. Instrumenting the baseband over time (dump the
candidate registers each measurement window alongside `abs_noise_floor_dbm`)
shows exactly one register move at the lock instant:

- **`0x4b74[30]`** (the CCK PD lower-bound): `0x58c5318c` (bit 30 **set**) while
  the NHM is correct, `0x18c5318c` (bit 30 **clear**) once it locks. Bit 30 = 0
  is the "max sensitivity" / PD-lower-bound-disabled state.
- The initial-gain registers **do not move** (path-0 LNA `0x472c[26:24]`, RXBB
  `0x46a8[14:10]` hold constant across the transition). So the LNA/TIA/RXBB gain
  index is unchanged — the ~24 dB is a **measurement-reference bias** on the
  shared dccl/CCA power path the NHM taps, driven by the PD-lower-bound state,
  not a gain-index change. The per-packet physts RSSI does not use this path and
  stays correct throughout (which is why the passive floor is accurate on both
  bands).

The register is owned by the **on-chip firmware DIG** (dynamic initial gain),
not by devourer:

- devourer runs no halbb DIG loop; its `set_channel` sets the PD lower-bound to
  the disabled/"max monitor sensitivity" state (OFDM `0x481C[29]=0`, CCK
  `0x4b74[30]=0`) once, at tune.
- The firmware then manages `0x4b74[30]` autonomously: after the tune it briefly
  sets a bound (bit 30 = 1, NHM correct), then on the quiet 5 GHz channel it
  settles to its no-link `IGI_NOLINK` max-sensitivity state (bit 30 = 0, NHM
  locks high) ~1.5–2 s later.
- Forcing bit 30 = 1 at `set_channel`, or re-writing it right before each NHM
  read, does **not** hold — the firmware re-clears it within the ~100 ms
  measurement window. RX keeps decoding normally with bit 30 in either state, so
  the PD state is a monitor-sensitivity knob, not an RX-liveness one.

2.4 GHz stays correct because its higher ambient noise keeps the firmware DIG in
a bounded state (bit 30 = 1) rather than the deepest no-link max-sensitivity
state. A truly silent 2.4 GHz channel is untested and could in principle lock
the same way; 2.4 GHz is rarely idle, and the plausibility guard below rejects a
gross error.

### What devourer does about it

`GetRxEnergy` band-gates the C8852C NHM to 2.4 GHz (`channel ≤ 14`) and keeps a
plausibility guard (−60…−105 dBm) as a backstop. On 5 GHz the absolute-floor
need is met by the passive rssi−snr floor, which is accurate on both bands
whenever there is traffic to sample.

### Residual cost

No frame-free absolute floor on a **silent** 5 GHz channel — the one case the
passive floor cannot cover (it needs decodable frames). A correct 5 GHz NHM
would require pinning/disabling the firmware DIG, which needs a BB-DM/DIG
control H2C that is not part of the ported mac_ax command surface, or a vendored
NHM path that reads the PD/CCA state; neither is currently worth the cost given
the passive floor.

### Reproducer

On a 5 GHz channel, poll `abs_noise_floor_dbm` per window with the NHM enabled
and watch `0x4b74[30]` (wide/`wIndex=1` window, `0x4b74 + 0x10000`) alongside
it: the value jumps ~24 dB the same window bit 30 clears, ~1.5 s after tune,
while the LNA/RXBB gain registers stay constant. The shipped
`tests/kestrel_nhm_2g_onair.sh` validates the current band gate (2.4 GHz emits a
plausible floor; 5 GHz is null → passive fallback).
