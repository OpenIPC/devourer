"""devourer_events — shared parser for devourer's JSONL event stream.

The demos emit machine events as JSON Lines on stdout (docs/logging.md):
one event per line, first field always the event name, serialized exactly as

    {"ev":"rx.txhit","hits":3,"total_rx":10,"len":247}

Diagnostics are plain text on stderr and never look like events. Any consumer
should tolerate non-JSON lines interleaved (libusb chatter redirected, etc.),
which is what iter_events() does.

Usage:
    from devourer_events import iter_events, parse_event

    for ev in iter_events(proc.stdout, ev="rx.txhit"):
        hits = ev["hits"]
"""

import json

EVENT_PREFIX = '{"ev":"'


def parse_event(line, ev=None):
    """Parse one line; return the event dict, or None if the line is not an
    event (or not the requested event name). Accepts str or bytes."""
    if isinstance(line, (bytes, bytearray)):
        try:
            line = line.decode("utf-8", "replace")
        except Exception:
            return None
    line = line.strip()
    if not line.startswith(EVENT_PREFIX):
        return None
    if ev is not None and not line.startswith(EVENT_PREFIX + ev + '"'):
        return None
    try:
        obj = json.loads(line)
    except ValueError:
        return None
    if not isinstance(obj, dict) or "ev" not in obj:
        return None
    if ev is not None and obj["ev"] != ev:
        return None
    return obj


def iter_events(lines, ev=None):
    """Yield event dicts from an iterable of lines (file object, list, or a
    whole-text .splitlines()). Non-event lines are skipped. `ev` filters to
    one event name."""
    for line in lines:
        obj = parse_event(line, ev)
        if obj is not None:
            yield obj


def desc_rate_to_mcs(code):
    """rx.frame's `rate` (the chip's DESC_RATE index) -> ("ht"|"vht", mcs).

    Returns None for legacy/CCK and for multi-SS VHT: the consumers (the
    adaptive link's rate-verified probe attribution) fly 1SS, and an
    unclassifiable rate must read as "no rate evidence", never as a match.
    HT keeps the full 0..31 index range so a multi-SS HT frame maps outside
    a 1SS mcs_set and is ignored naturally."""
    if code is None:
        return None
    if 12 <= code <= 43:                  # DESC_RATEMCS0..MCS31
        return ("ht", code - 12)
    if 44 <= code <= 53:                  # DESC_RATEVHTSS1MCS0..9
        return ("vht", code - 44)
    return None
