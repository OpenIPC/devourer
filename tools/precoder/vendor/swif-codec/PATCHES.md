# Local patches to upstream swif-codec

Upstream: <https://github.com/irtf-nwcrg/swif-codec>
Vendored commit: see `COMMIT` (currently `de8cd8e`).

A handful of small fixes are applied in-place so this source tree builds
with a strict C99 compiler (gcc 13+, clang) and runs without flooding
stdout. They were not yet addressed upstream at the time of vendoring.

## Patch 1 — `src/swif_api.h` line ~394

Declaration of `swif_decoder_set_callback_functions`'s
`decoded_source_symbol_callback` parameter listed `void (*)(…)` (no return
value) while the underlying `swif_decoder_t` function-pointer table
(line ~290) and every codec-specific header (`swif_rlc_api.h:84`,
`swif_rlc_cb.h:106`) use `void* (*)(…)`. Fixed in our copy to match the
function-pointer table; this is the canonical signature used by callers
and the decoder dispatch path.

## Patch 2 — `src/swif_api.c` line ~160

Definition of the same dispatcher function had the same outdated `void`
return-type signature. Fixed to `void*` to match the patched header and
the underlying pointer table.

## Patch 3 — `src/swif_api.c` line 6

Removed `#define DEBUG 1` so the conditional `DEBUG_PRINT(...)` macros
in `swif_general.h` stay no-ops at runtime. The upstream `#define DEBUG`
forced every codec to dump every Gaussian-elimination step and every
coefficient vector to stderr, which buries useful logs.

## Patch 4 — `src/swif_rlc_api.c`

Two unconditional `printf("notify decoded: ...")` calls and three
unconditional `full_symbol_dump(..., stdout)` calls were removed (replaced
with `/* PATCHED: silenced */`). Upstream's intent was a verbose mode for
the integrated test harness; in a library context they're noise.

## Rationale

Patches 1–2 are correctness fixes (without them `gcc -std=c99 -Wall`
refuses to compile `swif_api.c`). Patches 3–4 are noise-suppression for
library use. No semantic changes to the encoder / decoder algorithms or
wire format.
