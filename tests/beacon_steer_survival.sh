#!/usr/bin/env bash
# beacon_steer_survival.sh — validate the Jaguar2 steer-then-re-download TBTT
# actuator (issue #241): the beacon must KEEP AIRING across N repeated
# AdjustBeaconTimingFine / AdjustBeaconTiming steers, with the observed 802.11
# seq still incrementing (frozen seq = the J2 post-re-latch drop) and an
# arrival-phase step per steer.
#
# Master  = tests/beacon_steer_check.cpp — StartBeacon(100 TU) + SetCcaMode,
#           then N steers at a cadence. USB (MASTER_VID/MASTER_PID) or PCIe
#           (MASTER_PCIE_BDF=0000:01:00.0; can run on a remote rig — then run
#           the master there by hand and this script observer-only: OBS_ONLY=1).
# Observer= logs each canonical-SA beacon's seq + arrival phase
#           (its own hardware tsfl % interval).
#
# Usage: sudo -v; tests/beacon_steer_survival.sh [n_steers] [steer_us] [period_s]
#   MASTER_VID/MASTER_PID  master USB ids   (default 0x2357/0x012d — 8812BU)
#   OBS_PID                observer USB pid (default 0xc811 — 8811CU)
#   STEER_MODE=coarse      use AdjustBeaconTiming instead of the fine variant
#   OBS_ONLY=1             observer only (master runs elsewhere, e.g. PCIe rig)
#   MASTER_CMD='ssh rig …' run this command as the master instead of the local
#                          USB binary (e.g. the PCIe master on the radxa-x4);
#                          its stdout is analyzed as the master log. The two
#                          hosts' clocks must be NTP-close (~tens of ms) for
#                          the epoch-ms line-up.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
N="${1:-5}"; STEER_US="${2:--5000}"; PERIOD="${3:-5}"
CH="${DEVOURER_CHANNEL:-36}"
LIB="build/libdevourer.a"
CXXFLAGS="-std=c++20 -O2 -Isrc -Iexamples/common"
LDLIBS="$(pkg-config --cflags --libs libusb-1.0) -lpthread"
TMP="$(mktemp -d)"
MLOG="$TMP/master.log"; OLOG="$TMP/obs.log"
OBS_SECS=$(( 4 + PERIOD * (N + 2) + 6 ))

cleanup() {
  sudo pkill -9 -x bsteer_master 2>/dev/null
  sudo pkill -9 -x bsteer_obs 2>/dev/null
  rm -f /tmp/bsteer_master /tmp/bsteer_obs
}
trap cleanup EXIT INT TERM

[ -f "$LIB" ] || { echo "build $LIB first (cmake --build build -j)"; exit 1; }

# ---- observer source (seq + phase per canonical-SA beacon) -----------------
cat > "$TMP/obs.cpp" <<'EOF'
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <memory>
#include <thread>
#include <libusb.h>
#include "RxPacket.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
static const uint8_t kSa[6]={0x57,0x42,0x75,0x05,0xd6,0x00};
static long now_ms(){return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();}
int main(int argc,char**argv){
  int secs = argc>1?atoi(argv[1]):40;
  auto logger=std::make_shared<Logger>();
  libusb_context*ctx=nullptr; libusb_init(&ctx);
  libusb_set_option(ctx,LIBUSB_OPTION_LOG_LEVEL,LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid=0x0bda,pid=0xc811;
  if(const char*p=std::getenv("OBS_PID")) pid=(uint16_t)strtoul(p,0,0);
  uint8_t ch=36; if(const char*c=std::getenv("DEVOURER_CHANNEL")) ch=atoi(c);
  auto*h=libusb_open_device_with_vid_pid(ctx,vid,pid);
  if(!h){fprintf(stderr,"obs open %04x:%04x fail\n",vid,pid);return 1;}
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if(devourer::claim_interface_then_reset(h,0,logger,true,lk)!=0)return 1;
  WiFiDriver wifi(logger);
  auto dev=wifi.CreateRtlDevice(h,ctx,lk,devourer_config_from_env());
  if(!dev)return 1;
  auto cb=[](const Packet&p){
    if(p.Data.size()<32||p.RxAtrib.crc_err)return;
    if(p.Data[0]!=0x80)return;                      // beacons only
    if(std::memcmp(p.Data.data()+10,kSa,6)!=0)return;
    uint16_t sc=(uint16_t)(p.Data[22]|(p.Data[23]<<8));
    uint32_t tsfl=p.RxAtrib.tsfl;
    printf("BCN_MS %ld seq=%u tsfl=%u phase=%u\n", now_ms(), sc>>4, tsfl,
           tsfl%102400u);
    fflush(stdout);
  };
  std::thread rx([&]{dev->Init(cb,SelectedChannel{ch,0,CHANNEL_WIDTH_20});});
  rx.detach();
  std::this_thread::sleep_for(std::chrono::seconds(secs));
  _exit(0);
}
EOF

echo "building observer + master ..."
g++ $CXXFLAGS "$TMP/obs.cpp" examples/common/env_config.cpp "$LIB" $LDLIBS \
    -o /tmp/bsteer_obs || exit 1
if [ -z "${OBS_ONLY:-}" ] && [ -z "${MASTER_CMD:-}" ]; then
  g++ $CXXFLAGS tests/beacon_steer_check.cpp examples/common/env_config.cpp \
      "$LIB" $LDLIBS -o /tmp/bsteer_master || exit 1
fi

echo "== observer pid=${OBS_PID:-0xc811} ch$CH for ${OBS_SECS}s =="
sudo DEVOURER_CHANNEL="$CH" OBS_PID="${OBS_PID:-0xc811}" \
    /tmp/bsteer_obs "$OBS_SECS" >"$OLOG" 2>/dev/null &
OBS=$!
sleep 3   # let observer RX settle

if [ -n "${MASTER_CMD:-}" ]; then
  echo "== master (MASTER_CMD): $N x ${STEER_US}us every ${PERIOD}s (${STEER_MODE:-fine}) =="
  bash -c "$MASTER_CMD" >"$MLOG" 2>"$TMP/master.err" ||
      { echo "MASTER FAILED"; tail -3 "$TMP/master.err"; }
  cat "$MLOG"
elif [ -z "${OBS_ONLY:-}" ]; then
  echo "== master ${MASTER_VID:-0x2357}:${MASTER_PID:-0x012d} — $N x ${STEER_US}us every ${PERIOD}s (${STEER_MODE:-fine}) =="
  sudo DEVOURER_CHANNEL="$CH" DEVOURER_VID="${MASTER_VID:-0x2357}" \
      DEVOURER_PID="${MASTER_PID:-0x012d}" ${STEER_MODE:+STEER_MODE="$STEER_MODE"} \
      /tmp/bsteer_master "$N" "$STEER_US" "$PERIOD" >"$MLOG" 2>"$TMP/master.err" ||
      { echo "MASTER FAILED"; sed -n '$p' "$TMP/master.err"; }
  cat "$MLOG"
else
  echo "== OBS_ONLY: start the master on the remote rig now =="
fi
wait $OBS 2>/dev/null

echo "---- analysis ----"
python3 - "$MLOG" "$OLOG" <<'PYEOF'
import sys
mlog, olog = sys.argv[1], sys.argv[2]
steers = []
try:
    for ln in open(mlog):
        if ln.startswith("STEER_MS"):
            f = dict(kv.split("=") for kv in ln.split()[2:])
            steers.append((int(ln.split()[1]), int(f["req_us"]), int(f["applied_us"])))
except FileNotFoundError:
    pass
bcns = []
for ln in open(olog):
    if ln.startswith("BCN_MS"):
        p = ln.split()
        f = dict(kv.split("=") for kv in p[2:])
        bcns.append((int(p[1]), int(f["seq"]), int(f["phase"])))
if not bcns:
    print("FAIL: observer saw no beacons at all"); sys.exit(1)
print(f"{len(bcns)} beacons observed")
ok = True
for i, (t, req, app) in enumerate(steers):
    pre  = [b for b in bcns if t - 3000 <= b[0] < t]
    post = [b for b in bcns if t + 500 <= b[0] < t + 4000]
    if not post:
        print(f"steer {i+1} @ {t}: FAIL — no beacons within 4 s after"); ok = False; continue
    # seq must advance across the steer
    if pre and post[-1][1] == pre[-1][1]:
        print(f"steer {i+1} @ {t}: FAIL — seq frozen at {pre[-1][1]}"); ok = False; continue
    dphase = ""
    if pre:
        d = (post[0][2] - pre[-1][2]) % 102400
        if d > 51200: d -= 102400
        dphase = f" phase_step={d}us (req {req}, applied {app})"
    # real beacon downtime: max consecutive inter-beacon gap around the steer
    win = [b[0] for b in bcns if t - 1000 <= b[0] < t + 4000]
    gap = max((b - a for a, b in zip(win, win[1:])), default=0)
    print(f"steer {i+1} @ {t}: OK — {len(post)} beacons after, "
          f"max_gap={gap}ms (~{max(0, round(gap/102.4) - 1)} lost){dphase}")
if steers:
    print("SURVIVAL:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
print("(no STEER_MS lines — observer-only run; inspect phases by hand)")
PYEOF
rc=$?
echo "logs: $TMP"
trap - EXIT; cleanup
exit $rc
