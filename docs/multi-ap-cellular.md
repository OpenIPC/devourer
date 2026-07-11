# Building a small cellular network out of Wi-Fi — what the shared clock enables

![Warehouse cellular — coordinated slots and make-before-break handover](img/warehouse_cellular.gif)

*Two AP cells on one wired timebase, a robot driving between them. Act 1: no
coordination — the cell-edge is a collision zone. Act 2: the backhaul assigns
orthogonal slots and the edge goes clean. Act 3: the robot crosses cells and
its schedule simply changes owner between two slots — no scan, no
re-association, no clock re-acquisition; the ghost bar shows the ~100 ms+ hole
an ordinary Wi-Fi roam would have punched.*

The [time-distribution machinery](time-distribution.md) ends with a specific
capability: **every radio in a facility — wired or wireless — can share one
microsecond-grade clock**. A wired PTP grandmaster disciplines each AP's Wi-Fi
clock over ordinary Ethernet (~hundreds of ns); each AP's hardware beacon
distributes that clock over the air (~1 µs held); any listening station
inherits it passively (~sub-µs lock). Add the other bench-proven primitives —
millisecond channel switching ([frequency-hopping.md](frequency-hopping.md)),
per-frame hardware receive timestamps, a closed timing-advance loop
([time-distribution.md](time-distribution.md)), TDMA slotting
([narrowband.md](narrowband.md)) — and the ingredient list reads like a small
LTE deployment, minus the licensed spectrum and the basestation price tag.

This page is the conceptual map of what those ingredients build. Everything
below is *architecture riding on proven primitives*: the per-link mechanics all
have bench numbers in the linked docs; the multi-cell coordination itself is
application-layer software — no new silicon tricks required.

## 1. Coordinated scheduling — ICIC over your existing Ethernet

Two APs with overlapping coverage are, in ordinary Wi-Fi, two contenders:
CSMA arbitrates every frame, and at the cell edge — where a client hears both —
throughput and latency degrade exactly where a moving platform can least
afford it. LTE solved this as **Inter-Cell Interference Coordination**:
neighboring cells negotiate who transmits when (or on which resource), so the
edge sees orthogonal, not colliding, transmissions.

The classic obstacle for Wi-Fi is that ICIC presumes the cells agree on *when*:
a slot grid is meaningless if each AP's clock wanders tens of ppm from its
neighbor's. That is precisely what the shared timebase removes. With every
AP's on-air schedule held to the same wired clock at ~1 µs, a slot boundary is
a real, network-wide instant — guard intervals can be tens of microseconds
instead of "re-sync and hope".

The coordination plane is free: the same Ethernet that carries PTP carries the
schedule. Either shape works —

- **centralized**: one scheduler process owns the facility's slot map and
  pushes per-AP assignments (simple, global optimum, single point of failure),
- **peer**: adjacent APs exchange load/interference reports and negotiate
  pairwise (no controller, converges locally — the LTE X2 pattern).

Orthogonality can be in **time** (adjacent cells transmit in disjoint slots —
cheapest, halves the edge airtime), in **frequency** (adjacent cells sit on
different channels and a client retunes in ~ms as it moves — full airtime,
needs the channel-switch primitive), or both (a reuse pattern, exactly like
cellular frequency planning). The right split is a policy decision in the
scheduler, not a driver feature.

## 2. Seamless handover — the big win for anything that moves

An ordinary Wi-Fi roam is a gap: scan for candidates (the client leaves the
serving channel to listen), re-authenticate, re-associate, re-learn the new
AP's timing — ~100 ms on a good day, far more on a bad one. For a stationary
laptop that's a hiccup; for a moving robot streaming control and video it's a
hole in the link, at the worst possible place (the cell edge).

The shared clock dissolves most of that:

- **No time re-acquisition.** The robot's slot schedule is expressed against
  the *network* clock, and every AP's beacons carry that same clock. Timing
  learned in cell A is already valid in cell B — the one thing an ordinary
  roam must painfully rebuild is simply never lost.
- **No discovery scan.** The robot passively hears the neighbor APs' beacons
  (they are on the same timebase; if cells use different channels, a
  millisecond retune during an idle slot samples a neighbor and returns —
  cheap enough to do continuously).
- **Network-side decision.** Each AP hardware-timestamps and RSSI-tags every
  robot uplink it hears — including uplinks addressed to a *neighbor* cell.
  Those per-AP observations flow over the backhaul, so the network sees the
  robot's radio horizon better than the robot does, and picks the moment: LTE
  measurement reports, without asking the client to do anything.
- **Make-before-break.** The "handover" is then only a schedule update pushed
  over the backhaul: *your slots now belong to AP B (and retune to channel Y)*.
  The robot flips its serving cell between one slot and the next — the
  schedule never stops being valid, so there is nothing to break before the
  make.

What remains of the classic roam cost is security context (whatever key
material the deployment uses must move or pre-stage — the 802.11r-style
problem, solvable over the same backhaul) and the ~ms retune when cells are on
different channels. Both fit inside a single slot guard.

## 3. Robots as roaming UEs — the whole picture

Put 1 and 2 together and each robot is, functionally, an LTE UE:

- it **camps** on a serving cell, locked to the network clock through that
  cell's beacons;
- it holds **scheduled resources**: downlink slots for control, uplink slots
  for telemetry and video — sized and placed by the facility scheduler, with
  cell-edge robots given interference-coordinated slots (section 1);
- its uplink timing is closed-loop: the serving AP phase-measures every uplink
  against the slot grid and feeds back a timing advance, so the robot's frames
  land inside their slots even as its propagation delay changes with motion;
- when it crosses a cell boundary, the network moves its schedule to the next
  AP (section 2) and the robot follows — make-before-break, no gap in either
  direction;
- determinism composes: because *everyone* is slotted on one clock, worst-case
  medium access is a schedule property, not a contention statistic — the
  latency budget of a control loop can be written down before deployment.

The honest boundary line, so this page ages well: what is **bench-proven** is
every per-link primitive above, with numbers, in
[time-distribution.md](time-distribution.md) (the clock chain end to end),
[frequency-hopping.md](frequency-hopping.md) (the retune), and
[narrowband.md](narrowband.md) (slotted TDMA on a shared clock). What is
**architecture** is the multi-cell layer: the scheduler, the measurement
aggregation, the handover controller, and the security-context motion — all
ordinary distributed software on the wired side, none of it blocked on the
radio. The primitives were the hard part; they exist and hold their numbers.
