#!/usr/bin/env python3
"""chanmig dashboard — a plain-terminal, stdlib-only view of the adaptive
channel-migration state, rendered ENTIRELY from the JSONL logs.

That it can render the full picture — active-link health, candidate ranking,
evidence age, and the counterfactual reason a move is (not) recommended —
from nothing but the logged events IS the proof that every decision is
explainable from its logged score components.

  python3 tools/chanmig_dash.py --scout scout.jsonl [--primary primary.jsonl]

Tails both files (append-only; never blocks the producers) and repaints at
1 Hz. No curses — SSH/serial friendly, like the other devourer displays.
"""
import argparse
import json
import sys
import time

RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
RED = "\033[31m"
GRN = "\033[32m"
YEL = "\033[33m"
CYN = "\033[36m"


def follow(paths):
    """Yield (path_key, event_dict) as lines are appended to any file."""
    handles = {}
    for key, p in paths.items():
        if p:
            try:
                handles[key] = open(p, errors="replace")
            except OSError:
                handles[key] = None
    buf = {k: "" for k in handles}
    while True:
        any_data = False
        for key, fh in handles.items():
            if fh is None:
                continue
            chunk = fh.read()
            if chunk:
                any_data = True
                buf[key] += chunk
                while "\n" in buf[key]:
                    line, buf[key] = buf[key].split("\n", 1)
                    if '"ev":"' in line:
                        try:
                            yield key, json.loads(line)
                        except json.JSONDecodeError:
                            pass
        if not any_data:
            yield None, None


class State:
    def __init__(self):
        self.scout_id = None
        self.plan = None
        self.cands = {}
        self.dwells = 0
        self.health = ("ok", "")
        self.decision = None        # last channel.recommend/hold
        self.ranking = None         # last channel.ranking
        self.primary_verdict = None
        self.primary_frames = 0

    def update(self, key, ev):
        e = ev.get("ev")
        if e == "scout.id":
            self.scout_id = ev
        elif e == "scout.plan":
            self.plan = ev
        elif e == "scout.cand":
            self.cands[ev["i"]] = ev
        elif e == "survey.dwell":
            self.dwells += 1
        elif e == "scout.health":
            self.health = (ev.get("state", "?"), ev.get("reason", ""))
        elif e in ("channel.recommend", "channel.hold"):
            self.decision = ev
        elif e == "channel.ranking":
            self.ranking = ev
        elif e in ("rx.quality", "link.health"):
            self.primary_verdict = ev.get("verdict")
        elif e == "rx.txhit":
            self.primary_frames += 1


def color_verdict(v):
    if v in ("HEALTHY",):
        return GRN + v + RESET
    if v in ("MARGINAL", "WEAK"):
        return YEL + v + RESET
    if v in ("SATURATED", "INTERFERENCE", "NO_SIGNAL"):
        return RED + (v or "?") + RESET
    return v or "-"


def render(st):
    out = ["\033[2J\033[H"]  # clear + home
    out.append(BOLD + "adaptive channel migration — scout advisory" + RESET)
    if st.scout_id:
        out.append(DIM + f"scout {st.scout_id.get('chip','?')} "
                   f"{st.scout_id.get('usb_id','?')} bus{st.scout_id.get('bus','?')} "
                   f"id={st.scout_id.get('scout_id','?')} "
                   f"note={st.scout_id.get('note','')}" + RESET)
    if st.plan:
        out.append(DIM + f"plan {st.plan.get('plan','?')}  "
                   f"{st.plan.get('n_candidates','?')} candidates  "
                   f"dwell {st.plan.get('dwell_ms','?')}ms" + RESET)
    hs, hr = st.health
    hc = GRN if hs == "ok" else (RED if hs == "wedged" else YEL)
    out.append(f"dwells={st.dwells}  scout={hc}{hs}{RESET}"
               + (f"({hr})" if hr else "")
               + f"  primary={color_verdict(st.primary_verdict)}"
               f"  frames={st.primary_frames}")
    out.append("")

    d = st.decision
    if d:
        kind = d["ev"].split(".")[1]
        kc = GRN if kind == "recommend" else CYN
        out.append(BOLD + f"decision: {kc}{kind.upper()}{RESET}"
                   f"  from {d.get('from','?')}"
                   + (f" -> {GRN}{d.get('to','?')}{RESET}" if "to" in d else ""))
        out.append(f"  reason: {d.get('reason','?')}")
        out.append(f"  active: {color_verdict(d.get('active_verdict'))} "
                   f"domain={d.get('active_domain','?')} "
                   f"impaired_windows={d.get('impaired_windows','?')}")
        out.append(DIM + "  " + d.get("text", "") + RESET)
    else:
        out.append(DIM + "decision: (waiting for evidence)" + RESET)
    out.append("")

    out.append(BOLD + "candidate ranking (why each is / isn't a target)" + RESET)
    if st.ranking:
        rank = 0
        while f"c{rank}" in st.ranking:
            out.append(f"  {rank+1}. {st.ranking[f'c{rank}']}")
            rank += 1
    else:
        out.append(DIM + "  (no ranking yet)" + RESET)
    out.append("")
    out.append(DIM + f"updated {time.strftime('%H:%M:%S')}  "
               "(rendered purely from JSONL — this IS the explainability proof)"
               + RESET)
    sys.stdout.write("\n".join(out) + "\n")
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scout", required=True)
    ap.add_argument("--primary")
    ap.add_argument("--once", action="store_true",
                    help="ingest all available, render once, exit (for tests)")
    args = ap.parse_args()

    st = State()
    if args.once:
        for key, p in (("scout", args.scout), ("primary", args.primary)):
            if not p:
                continue
            try:
                for line in open(p, errors="replace"):
                    if '"ev":"' in line:
                        try:
                            st.update(key, json.loads(line))
                        except json.JSONDecodeError:
                            pass
            except OSError:
                pass
        render(st)
        return

    last = 0.0
    for key, ev in follow({"scout": args.scout, "primary": args.primary}):
        if ev is not None:
            st.update(key, ev)
        now = time.time()
        if now - last >= 1.0:
            render(st)
            last = now
        if ev is None:
            time.sleep(0.2)


if __name__ == "__main__":
    main()
