# M3 — joining a running cell

M3 (#257, EPIC #254) lets a station join a running [M1/M2 cell](cell-m1.md)
without disturbing the UEs already scheduled. The pieces:

- **`src/cell/UeRegistry.h`:** the UE pool. The grid is laid out for a fixed
  capacity, so slot positions never move when a station joins or leaves.
- **`src/cell/Rach.h`:** the 2-step random access. The headless parts are
  covered by the `cell_m3` ctest, including a contention simulation.
- **`examples/cell`:** wires both in. A joiner runs as `CELL_ROLE=ue CELL_UE=0`.

## The 2-step RACH

1. **msgA** (joiner → DU), sent in the grid's RACH slot (`CELL_RACH_US`).
   - It is a broadcast data frame from the joiner's own random MAC, so
     nothing ACKs it and there is no contention-window escalation.
   - It carries the MAC, a 64-bit nonce (the station's temporary identity),
     the attempt number, a freshness stamp (the DU superframe), and a tag.
   - The joiner times it on its own free-running counter through the
     `TsfSync` fit of the DU beacons. It draws a random start that leaves
     room for the whole frame plus a departure guard before the slot ends.
2. **msgB** (DU → joiner): a second vendor IE in the DU's next beacons.
   - Each entry carries the recipient MAC and nonce, granted / denied-full /
     denied-bad, the UE id, its DL/UL slots, and a tag.
   - At most 8 entries fit per beacon. Longer queues are paged, and each
     answer is published in 4 beacons.

**Authentication comes before allocation.** Both tags are HMAC-SHA1, keyed
by the cell PSK (`CELL_PSK`) and truncated to 8 bytes. The primitive is
injected, so the library links no crypto; the example uses OpenSSL. The DU
verifies a request's tag and freshness before the registry is touched. A
forged or replayed request gets nothing.

**Contention rules:**

| case | rule |
|---|---|
| no answer within 4 superframes | the attempt was lost; retry the same nonce, backoff window doubling (4 → 64 superframes) |
| still no answer after 10 attempts | give up: the client ends in a defined Failed state |
| a duplicate request | answered identically while its grant is still valid |
| the same MAC re-joining | keeps its id |
| a full pool | answers denied-full, which ends that client's attempt (no retry storm) |

## Measured on the epic-254 rig

The rig: RTL8852CE over PCIe, ch149. Capacity 3 with UE1 static, an 8 ms RACH
slot, and HARQ + LA on. The DU and UE1 run scheduled traffic. At 35 s a
joiner is hot-plugged on a third box, and a fourth radio joins with the
**wrong PSK**. Run via testrig `scripts/m3_cell_rach.sh`.

| | result |
|---|---|
| joiner | granted UE id 2 on its **first attempt**, **2 superframes** after it |
| joined UE | UL 1.56 µs rms, 100 % in slot; DL 613/613 in slot |
| existing UE1, after the join | UL 2.0 µs rms, 604/604 in slot (1.7 µs before it) |
| wrong-PSK joiner | 10 requests, all heard in the RACH slot, **none answered**, then Failed |
| witness | 0 overlaps; beacons, DL and UE1's UL 100 % in their own slots |

### Found on the way

- **The TA loop estimates with the window median.** A joining UE's first
  uplink was its beacon from before its first TSF step, 39 ms off. With a
  mean, that one sample pushed the UE out of its slot for ~6 s.
- **A msgA can still leave late.** In one run the joiner's msgA aired 4.7 ms
  after its target and landed past the RACH slot, in idle air. The DU still
  heard and granted it. This is the rare multi-ms `send_packet` departure
  tail M0 measured on Kestrel (max ≈ 4.8 ms on a clean channel). A
  contention slot can budget the p99.9 guard, but not that tail. Keep the
  RACH slot away from the next beacon.

## Not in M3 (yet)

- **The full 802.11 association** (probe / auth / assoc / WPA2 4-way, as in
  `tests/ap_wpa2.cpp`) is not run. The join is authenticated with the cell
  PSK instead.
- **Departure** (releasing a silent UE's position) is not done. The registry
  supports `release`; deciding when to call it is open.
