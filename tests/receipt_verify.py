#!/usr/bin/env python3
"""Frame-exact verification of the windowed RX receipts (issue: delivery
truth at the app layer).

Replays every receipt TLV the transmitter absorbed (tx.receipt events carry
the raw hex) through the same merge the ReceiptLedger performs, and compares
the resulting delivered-set against the RECEIVER's own ground truth — the
rx.seq ledger the arq_e2e bench already trusts frame-exactly. The comparison
domain is [0, covered): `covered` is the exclusive top of receipt coverage,
so frames the receiver saw after the last receipt flew are out of contract.

PASS means the transmitter's receipt-derived view of "what the app received"
is identical, frame for frame, to what the app actually received.

  python3 tests/receipt_verify.py --dut <run>/dut.jsonl --drone <run>/drone.jsonl
"""
import argparse
import json
import sys

HDR = 16


def replay(drone_path):
    got = set()
    covered = 0
    receipts = 0
    for line in open(drone_path, errors="replace"):
        if not line.startswith('{"ev":"tx.receipt"'):
            continue
        try:
            ev = json.loads(line)
        except Exception:
            continue
        tlv = bytes.fromhex(ev.get("tlv", ""))
        if len(tlv) < HDR or tlv[0:2] != b"DR" or tlv[2] != 1:
            print(f"BAD TLV in event stream: {ev.get('tlv', '')[:32]}...")
            return None, 0, 0
        base = int.from_bytes(tlv[10:14], "little")
        nbits = int.from_bytes(tlv[14:16], "little")
        # Prefix-strict like receipt_decode: the captured body carries the
        # 4-byte FCS after the TLV.
        if len(tlv) < HDR + (nbits + 7) // 8:
            print("BAD TLV length")
            return None, 0, 0
        for i in range(nbits):
            if tlv[HDR + i // 8] >> (i % 8) & 1:
                got.add(base + i)
        covered = max(covered, base + nbits)
        receipts += 1
    return got, covered, receipts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dut", required=True)
    ap.add_argument("--drone", required=True)
    a = ap.parse_args()

    receipt_set, covered, receipts = replay(a.drone)
    if receipt_set is None or receipts == 0:
        print("VERDICT: NO-RECEIPTS — nothing to verify "
              "(DEVOURER_TX_RECEIPTS + DEVOURER_RX_RECEIPT_MS both set?)")
        return 3

    truth = set()
    for line in open(a.dut, errors="replace"):
        if line.startswith('{"ev":"rx.seq"'):
            try:
                truth.add(json.loads(line)["pctr"])
            except Exception:
                pass
    truth_in_domain = {k for k in truth if k < covered}

    missing = truth_in_domain - receipt_set   # app got it, receipts never said
    phantom = receipt_set - truth_in_domain   # receipts claim it, app didn't
    print(f"receipts={receipts} covered=[0,{covered}) "
          f"receipt_set={len(receipt_set)} truth_in_domain="
          f"{len(truth_in_domain)}")
    if missing:
        print(f"MISSING from receipts ({len(missing)}): "
              f"{sorted(missing)[:10]}{'...' if len(missing) > 10 else ''}")
    if phantom:
        print(f"PHANTOM in receipts ({len(phantom)}): "
              f"{sorted(phantom)[:10]}{'...' if len(phantom) > 10 else ''}")
    ok = not missing and not phantom
    print(f"VERDICT: {'FRAME-EXACT' if ok else 'MISMATCH'} — the "
          f"transmitter's receipt-derived delivered-set "
          f"{'==' if ok else '!='} the receiver's rx.seq ledger")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
