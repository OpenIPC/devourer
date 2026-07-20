#!/usr/bin/env bash
# kchansw_vm.sh — run the kchansw phase-b vendor-driver measurements inside
# the pinned-kernel devourer-testrig VM.
#
# Why a VM here: insmod-ing the vendor 88x2bu module on the host and
# kprobing its internal switch path deadlocked the host kernel outright
# (ssh dead, dmesg blocked — power cycle to recover) — twice. In the VM the
# same wedge costs a `virsh reset`.
#
# The DUT is passed through by vid:pid; the on-air oracles stay on the
# host, so dead-air timing keeps its host-clock precision. The host bench
# (`kchansw_bench.py phase-b-vm`) measures the VM→host CLOCK_MONOTONIC
# offset and stamps it into each config's meta.json.
#
# Usage (root):
#   tests/kchansw_vm.sh prepare   # start VM, push sources, build ko (tdls),
#                                 # move DUT to its USB2 port, attach, load
#   tests/kchansw_vm.sh mcc       # rebuild VARIANT=mcc inside VM + reload
#   tests/kchansw_vm.sh restore   # rmmod in VM, detach DUT, shut VM down
#   tests/kchansw_vm.sh status
#
# Env: VM_NAME (devourer-testrig), VM_USER (invoking user), DUT_VIDPID
# (2357:012d), USB3_PORT_DISABLE (usb10-port2 — empty to skip the USB2 pin).

set -euo pipefail
cd "$(dirname "$0")/.."

MODE="${1:-status}"
VM_NAME="${VM_NAME:-devourer-testrig}"
VM_USER="${VM_USER:-${SUDO_USER:-$USER}}"
DUT_VIDPID="${DUT_VIDPID:-2357:012d}"
VID="${DUT_VIDPID%%:*}"
PID="${DUT_VIDPID##*:}"
# The T3U wedges its USB3/SuperSpeed link under switch churn (measured in
# phase A); pinning it to the USB2 companion port is load-bearing.
USB3_PORT_DISABLE="${USB3_PORT_DISABLE:-/sys/bus/usb/devices/usb10/10-0:1.0/usb10-port2/disable}"
VM_DIR=/opt/kchansw
HOSTDEV_XML=/tmp/kchansw-hostdev.xml

need_root() { [ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }; }

vm_ip() {
  virsh domifaddr "$VM_NAME" 2>/dev/null \
    | awk '/ipv4/ {print $4}' | cut -d/ -f1 | head -1
}

SSH_KEY="${SSH_KEY:-$(ls "$(eval echo "~$VM_USER")"/.ssh/id_ed25519 \
  "$(eval echo "~$VM_USER")"/.ssh/id_rsa 2>/dev/null | head -1 || true)}"
SSH_OPTS=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null
          -o ConnectTimeout=8 -o LogLevel=ERROR -i "$SSH_KEY")
vssh() { ssh "${SSH_OPTS[@]}" "$VM_USER@$(vm_ip)" "$@"; }

hostdev_xml() {
  cat > "$HOSTDEV_XML" <<XML
<hostdev mode='subsystem' type='usb' managed='yes'>
  <source>
    <vendor id='0x$VID'/>
    <product id='0x$PID'/>
  </source>
</hostdev>
XML
}

wait_ssh() {
  local i ip
  for i in $(seq 1 30); do
    ip="$(vm_ip)"
    [ -n "$ip" ] && vssh true 2>/dev/null && return 0
    sleep 3
  done
  echo "FAIL: no ssh to $VM_NAME"; return 1
}

vm_build() {
  local variant="$1" flags
  case "$variant" in
    tdls) flags="CONFIG_TDLS=y" ;;
    mcc)  flags="CONFIG_MCC_MODE=y CONFIG_BT_COEXIST=n USER_EXTRA_CFLAGS=-DCONFIG_CONCURRENT_MODE" ;;
    *) echo "FAIL: variant $variant"; exit 2 ;;
  esac
  echo "== building VARIANT=$variant in VM =="
  vssh "cd $VM_DIR/rtl88x2bu && make clean >/dev/null 2>&1; \
        make $flags -j\$(nproc) >/tmp/build.log 2>&1 \
        && echo BUILD-OK || { grep -m5 'error:' /tmp/build.log; exit 1; }"
}

vm_load() {
  # The fork names the module per-variant (88x2bu_ohd.ko for the default
  # build, 8812bu.ko when MCC make vars override the Makefile's name).
  vssh "sudo rmmod 88x2bu_ohd 2>/dev/null; sudo rmmod 8812bu 2>/dev/null; \
        ko=\$(ls -t $VM_DIR/rtl88x2bu/*.ko | head -1); \
        sudo modprobe cfg80211 && \
        sudo insmod \$ko rtw_wifi_spec=1 && \
        for i in \$(seq 1 20); do \
          w=\$(ls /sys/class/net | grep -v -e lo -e enp | head -1); \
          [ -n \"\$w\" ] && { echo NETDEV=\$w; exit 0; }; sleep 0.5; \
        done; echo FAIL-no-netdev; exit 1"
}

case "$MODE" in
  prepare)
    need_root
    if [ "$(virsh domstate "$VM_NAME")" != "running" ]; then
      virsh start "$VM_NAME"
    fi
    wait_ssh
    echo "VM up at $(vm_ip)"
    vssh "sudo mkdir -p $VM_DIR && sudo chown $VM_USER: $VM_DIR"
    echo "== pushing sources =="
    tar -C reference -c --exclude=.git --exclude='*.o' --exclude='*.ko' \
        --exclude='*.cmd' --exclude='*.mod*' rtl88x2bu \
      | vssh "tar -C $VM_DIR -x"
    for f in kchansw_trace.py kchansw_inject.py kchansw_vm_dut.py; do
      scp -q "${SSH_OPTS[@]}" "tests/$f" "$VM_USER@$(vm_ip):$VM_DIR/"
    done
    vm_build tdls
    # Pin the DUT to USB2 before handing it to the VM: passthrough URBs
    # still ride the host controller, and phase A wedged it on USB3.
    if [ -n "$USB3_PORT_DISABLE" ] && [ -e "$USB3_PORT_DISABLE" ]; then
      echo 1 > "$USB3_PORT_DISABLE" || true
      sleep 4
    fi
    lsusb -d "$DUT_VIDPID" >/dev/null || { echo "FAIL: DUT gone"; exit 1; }
    hostdev_xml
    if virsh dumpxml "$VM_NAME" | grep -q "0x$PID"; then
      echo "  DUT already attached to $VM_NAME"
    else
      virsh attach-device "$VM_NAME" "$HOSTDEV_XML" --live
      sleep 3
    fi
    vm_load
    echo "PASS: VM DUT ready — run: sudo tests/kchansw_setup.sh phase-b-vm"
    ;;
  mcc)
    need_root
    vm_build mcc
    vm_load
    ;;
  restore)
    need_root
    vssh "sudo rmmod 88x2bu_ohd 2>/dev/null; sudo rmmod 8812bu 2>/dev/null" \
      || true
    hostdev_xml
    virsh detach-device "$VM_NAME" "$HOSTDEV_XML" --live 2>/dev/null || true
    virsh shutdown "$VM_NAME" 2>/dev/null || true
    echo "restore done (VM shutting down; DUT back on host bus)"
    ;;
  status)
    echo "VM: $VM_NAME ($(virsh domstate "$VM_NAME" 2>/dev/null)) ip=$(vm_ip)"
    virsh dumpxml "$VM_NAME" 2>/dev/null | grep -A2 "type='usb'" \
      | grep -E 'vendor|product' | sed 's/^/  /' || true
    lsusb -d "$DUT_VIDPID" | sed 's/^/  host: /' || true
    ;;
  *) echo "usage: $0 {prepare|mcc|restore|status}"; exit 2 ;;
esac
