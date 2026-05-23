# devourer regression test rig

Cross-driver matrix test that compares this project's userspace stack
against the kernel-driver baseline for both TX and RX on plugged-in USB
Wi-Fi adapters.

```
                  TX = devourer       TX = kernel
RX = devourer     end-to-end devourer  does dvr RX a kernel-TX frame?
RX = kernel       does dvr emit valid  baseline / rig sanity check
                  frames?
```

Each cell injects/receives the canonical beacon (SA `57:42:75:05:d6:00`,
matching `txdemo/main.cpp`) for `--duration` seconds and counts hits.
The baseline cell runs first — if it fails the rig itself is broken and
the remaining cells are skipped (override with `--no-baseline-abort`).

## Two run modes

### Local mode

The kernel-side cells run against whatever driver is bound to the DUTs on
the **host** (mainline `rtw88_*` or whatever's loaded). Cheap to set up
but limited to drivers that build cleanly against the host kernel — that's
a moving target as kernels evolve, especially for the out-of-tree
`aircrack-ng/rtl8812au` driver.

```bash
sudo python3 tests/regress.py --channel 100
```

### VM mode (recommended)

The kernel-side cells run inside a **pinned-kernel libvirt VM** that has
the OOT `aircrack-ng/rtl8812au` driver built and loaded. DUTs are
transferred between host and VM per cell via `virsh attach-device` /
`detach-device`. The VM's kernel never moves so the driver never breaks.

Provision the VM once with the included script (Ubuntu 22.04 LTS,
kernel 5.15 — where aircrack-ng's driver builds without patches):

```bash
sudo tests/setup_vm.sh                    # provision; ~5-10 min
sudo tests/setup_vm.sh --status           # show VM IP, ssh hint
```

Then run the matrix in VM mode:

```bash
sudo python3 tests/regress.py --channel 100 \
    --vm-name devourer-testrig \
    --vm-ssh <user>@<VM-IP-from-status>
```

VM mode is what unblocks chipsets where the host kernel driver doesn't
work — e.g. RTL8814AU, where mainline `rtw88_8814au` currently fails to
probe on kernels 6.15+ (`failed to download firmware`, `error -22`), but
`aircrack-ng/rtl8812au` claims it cleanly on the pinned kernel 5.15.

## Prerequisites

### On the host (both modes)

- 2 supported USB Wi-Fi adapters plugged in
- devourer built (`build/WiFiDriverDemo`, `build/WiFiDriverTxDemo`)
- Python 3.9+ with `scapy` (`pip install scapy` or `python3-scapy`)
- `iw`, `tcpdump`, `ip` on PATH
- Passwordless `sudo`, or run directly as root

### For local mode

- Kernel driver(s) installed and `modprobe`-able for your DUTs (rtw88 or
  aircrack-ng — script auto-detects whatever's bound via sysfs)
- NetworkManager users: stop NM, or `nmcli device set <iface> managed no`
  on the test interfaces

### For VM mode (in addition)

- `libvirtd` + `virsh` + `virt-install` on the host
- `xorriso` (for the cloud-init seed ISO that `setup_vm.sh` generates)
- An Ubuntu 22.04 cloud image at `/var/lib/libvirt/images/jammy-base.qcow2`
  (download from <https://cloud-images.ubuntu.com/jammy/current/>)
- Working USB hot-plug on libvirt (`xhci` controller; `setup_vm.sh` adds it)
- The host user's SSH key in `~/.ssh/id_rsa.pub` (or set `SSH_PUBKEY=...`
  before `setup_vm.sh`) — gets baked into the VM's user account
  (defaults to your invoking user; override with `VM_USER=foo`)

The script does a preflight check and prints distro-agnostic install
hints for anything missing.

## Output

Markdown table to stdout, ready to paste into PR comments:

```
## Regression matrix — channel 100, 2026-05-23 13:22:14

- TX adapter: `0bda:8812` (RTL8812AU)
- RX adapter: `0bda:8813` (RTL8814AU)
- Kernel host: VM devourer-testrig via <user>@<VM-IP>
- Cell duration: 10s
- Pass threshold: ≥ 3 hits

|   | TX = devourer | TX = kernel |
|---|---|---|
| RX = devourer | 0 hits / 4500 TX ✗ | 0 hits / 258 TX ✗ |
| RX = kernel | 4172 hits / 4500 TX ✓ | 229 hits / 259 TX ✓ |
```

Pass/fail per cell on hit-count threshold (default ≥ 1 — generous because
air interference makes absolute counts unreliable). Bump for higher-
confidence runs on a quieter channel.

For debugging a specific cell that failed, re-run with `--keep-logs` —
per-cell stdout/stderr logs end up at `/tmp/devourer-regress-last/`.

## CLI knobs

- `--channel N` — Wi-Fi channel for both adapters (default 36; pick the
  channel your nearest AP is on for guaranteed traffic)
- `--duration SECONDS` — per-cell injection/measurement window (default 15)
- `--pass-threshold N` — min hits to pass (default 1)
- `--tx-pid 0xNNNN` / `--rx-pid 0xNNNN` — pick specific DUTs (defaults to
  the first two auto-detected)
- `--no-baseline-abort` — run all 4 cells even if kernel-kernel fails
  (useful when one chipset has no working kernel driver on this rig)
- `--vm-name NAME` / `--vm-ssh USER@HOST` — enter VM mode
- `--keep-logs` — symlink the temp log dir at `/tmp/devourer-regress-last`

Environment variable equivalents: `DEVOURER_VM_NAME`, `DEVOURER_VM_SSH`.

## Supported DUTs

Listed in `SUPPORTED_DUTS` at the top of `regress.py`. Extend the dict
to add new chipsets — the rest of the script is chipset-agnostic.

## Architecture notes

- All kernel-side operations (modprobe / sysfs reads / `iw` / `tcpdump` /
  scapy) go through one abstraction (`KernelHost`). Local mode runs them
  via `subprocess.run`; VM mode wraps them in `ssh ... sudo`. Adding a
  third backend (e.g. remote bare-metal box) is a new class.
- DUT routing in VM mode uses `virsh attach-device` (USB hot-plug). The
  matrix moves DUTs between host and VM per cell as needed, restoring all
  DUTs to the host on exit so the next cell starts from a clean baseline.
- `inject_beacon.py` is shipped to the VM via `scp` each run (small file)
  and exits when its `--duration` elapses — orchestrator waits rather
  than killing, so the final "sent N frames" line is captured.

## Known limitations

- Tests "signal of life", not throughput — air noise makes absolute
  counts unreliable; pass-threshold is deliberately generous.
- Per matrix run: ~100s in local mode, ~3-4 min in VM mode (USB hot-plug
  adds ~5s per cell transition).
- Two-adapter scope today. To extend to >2, add a pairing loop in
  `main()` that runs the 4-cell matrix per chipset pair.
- VM mode assumes a single libvirt host running both `virsh` (locally)
  and the VM. Pulling the VM onto a different host is a `--vm-ssh
  user@vmhost` away on the kernel cell side, but `virsh attach-device`
  still runs locally; if the VM is on a different host, run virsh there
  (via your own wrapper).
- Cell 4 (`devourer-TX → devourer-RX`) requires both DUTs to be on the
  host and devourer-claimable simultaneously. Works fine, but means both
  chipsets need working devourer RX — if one is RX-broken (e.g. current
  RTL8814AU TODO), that cell will always show 0 hits regardless of TX.
