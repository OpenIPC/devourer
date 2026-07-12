#!/usr/bin/env python3
# ack_txreport_analyze.py — M0 contract 3: turn one matrix cell's TX-side
# stream (tx.report JSONL + a sent-frame count) into a capability verdict:
#   ack_rate        — fraction of reports with state==0 (hardware-ACKed)
#   retries mean/max— hardware retransmission counts (pinned at the descriptor
#                     limit when nobody ACKs)
#   report_coverage — reports received / frames sent (the C2H delivery rate)
#   tag_loss        — HalMAC only: gaps in the rotating 8-bit SW_DEFINE tag
#                     echo (per-frame correlation loss), plus the firmware's
#                     own missed_rpt tally
# Cell semantics (the matrix): responder ON -> expect ack_rate ~1, retries ~0;
# responder OFF -> expect ack_rate ~0, retries pinned; retarget cell = ON with
# a different unicast RA (SetAckResponder re-armed) -> expect ON behavior.
#
# Usage: python3 ack_txreport_analyze.py <tx.jsonl> --sent N --cell NAME \
#            --expect on|off
#        python3 ack_txreport_analyze.py --selftest
import json, sys

def load(path):
    out = []
    for line in open(path):
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            r = json.loads(line)
        except Exception:
            continue
        if r.get("ev") == "tx.report":
            out.append(r)
    return out

def analyze(reports, sent, cell="", expect=None):
    n = len(reports)
    v = {"ev": "ackrep.verdict", "cell": cell, "sent": sent, "reports": n}
    if n == 0:
        v["report_coverage"] = 0.0
        v["capability_ok"] = False
        v["error"] = "no tx.report events (C2H path dead in this session shape)"
        return v
    acked = sum(1 for r in reports if r.get("ok"))
    rts = [r.get("retries", 0) for r in reports]
    v["ack_rate"] = round(acked / n, 3)
    v["retries_mean"] = round(sum(rts) / n, 2)
    v["retries_max"] = max(rts)
    v["report_coverage"] = round(n / sent, 3) if sent else None
    # HalMAC per-frame correlation: the descriptor stamps a rotating 8-bit tag;
    # count sequence gaps in the echo (mod 256) + the fw's own missed counter.
    tags = [r["tag"] for r in reports if "tag" in r]
    if tags:
        gaps = 0
        for a, b in zip(tags, tags[1:]):
            gaps += (b - a - 1) % 256
        v["tag_gaps"] = gaps
        v["fw_missed"] = sum(r.get("missed", 0) for r in reports)
    if expect == "on":
        v["capability_ok"] = bool(v["ack_rate"] >= 0.9 and v["retries_mean"] < 2)
    elif expect == "off":
        # Nobody ACKs: delivery must FAIL and retries pin at the limit — this
        # proves the no-ACK outcome is visible, not that the link is bad.
        v["capability_ok"] = bool(v["ack_rate"] <= 0.1 and v["retries_max"] >= 8)
    else:
        v["capability_ok"] = None
    return v

def selftest():
    fails = 0
    def check(cond, msg):
        nonlocal fails
        if not cond:
            print("FAIL:", msg); fails += 1
    on = [{"ev": "tx.report", "ok": True, "state": 0, "retries": 0,
           "tag": i % 256, "missed": 0} for i in range(100)]
    v = analyze(on, 100, "on", expect="on")
    check(v["capability_ok"] and v["ack_rate"] == 1.0 and
          v["report_coverage"] == 1.0 and v["tag_gaps"] == 0, "clean ON passes")
    off = [{"ev": "tx.report", "ok": False, "state": 1, "retries": 12,
            "tag": i % 256, "missed": 0} for i in range(100)]
    v = analyze(off, 100, "off", expect="off")
    check(v["capability_ok"] and v["retries_max"] == 12, "pinned OFF passes")
    v = analyze(off, 100, "off-as-on", expect="on")
    check(not v["capability_ok"], "OFF behavior fails an ON expectation")
    v = analyze([], 100, "dead", expect="on")
    check(not v["capability_ok"] and "error" in v, "no reports = no capability")
    # Tag gaps: every 10th report lost — 9 interior gaps are visible (a
    # trailing loss has no successor; coverage-vs-sent catches it instead).
    lossy = [r for i, r in enumerate(on) if i % 10 != 9]
    v = analyze(lossy, 100, "lossy", expect="on")
    check(v["tag_gaps"] == 9, "tag gaps count lost reports (got %s)" % v["tag_gaps"])
    # J1 (no tag field) still verdicts on coverage alone.
    j1 = [{"ev": "tx.report", "ok": True, "state": 0, "retries": 1}
          for _ in range(90)]
    v = analyze(j1, 100, "j1", expect="on")
    check(v["capability_ok"] and "tag_gaps" not in v, "8812 format (no tag) ok")
    print("ack_txreport_analyze selftest:",
          "%d FAILURE(S)" % fails if fails else "all passed")
    return 1 if fails else 0

def main():
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    args = sys.argv[1:]
    sent, cell, expect = 0, "", None
    if "--sent" in args:
        i = args.index("--sent"); sent = int(args[i + 1]); del args[i:i + 2]
    if "--cell" in args:
        i = args.index("--cell"); cell = args[i + 1]; del args[i:i + 2]
    if "--expect" in args:
        i = args.index("--expect"); expect = args[i + 1]; del args[i:i + 2]
    v = analyze(load(args[0]), sent, cell, expect)
    print(json.dumps(v))
    sys.exit(0 if v.get("capability_ok") else 1)

if __name__ == "__main__":
    main()
