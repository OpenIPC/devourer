#!/usr/bin/env bash
# beacon_interval_shift.sh — de-risk the uplink timing-advance actuator.
#
# WriteTsf (REG_TSFTR 0x0560) was shown NOT to move the beacon TBTT air-time
# (the beacon engine runs off a separate/per-port timer). This exercises +
# validates the productized actuator IRtlDevice::AdjustBeaconTiming(us): a
# ONE-SHOT beacon-interval tweak (REG_BCN_INTERVAL 0x0554) — run one interval at
# (nominal +/- delta) TU then restore, and a clean interval-phased engine
# advances/retards the next TBTT by delta TU, resuming cadence phase-shifted.
#
# Master  = 8822CU (0xc812): StartBeacon(100 TU) + SetCcaMode(true); after a
#           steady window, AdjustBeaconTiming((SHORT-100)*1024 us) — a one-shot
#           phase advance of (100-SHORT) TU via the bare-0x0554 path (no beacon
#           re-download).
# Observer= 8822EU (0xa81a): logs each canonical-SA beacon's arrival phase
#           (its own hardware tsfl % 102400 us). A clean actuator shows a
#           ~ (100-SHORT)*1024 us step in the arrival phase right after the tweak,
#           standing well clear of the slow crystal-drift ramp.
#
# Both binaries print CLOCK epoch-ms (shared system_clock, same machine) so the
# observer's phase jump can be lined up against the master's tweak instant.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
SHORT="${1:-80}"          # short interval in TU (100->80 => +20 TU = 20.48 ms advance)
CH="${DEVOURER_CHANNEL:-36}"
LIB="build/libdevourer.a"
CXXFLAGS="-std=c++20 -O2 -Isrc -Iexamples/common"
LDLIBS="$(pkg-config --cflags --libs libusb-1.0) -lpthread"
TMP="$(mktemp -d)"
MLOG="$(mktemp)"; OLOG="$(mktemp)"

cleanup() {
  sudo pkill -9 -x bshift_master bshift_obs 2>/dev/null
  rm -rf "$TMP" "$MLOG" "$OLOG" 2>/dev/null
}
trap cleanup EXIT INT TERM

[ -f "$LIB" ] || { echo "build $LIB first (cmake --build build -j)"; exit 1; }

# ---- master source -------------------------------------------------------
cat > "$TMP/master.cpp" <<'EOF'
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>
#include <vector>
#include <libusb.h>
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
static long now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}
static std::vector<uint8_t> beacon(int itu) {
  return {0x00,0x00,0x0a,0x00,0x00,0x80,0x00,0x00,0x08,0x00,   // radiotap
          0x80,0x00,0x00,0x00, 0xff,0xff,0xff,0xff,0xff,0xff,
          0x57,0x42,0x75,0x05,0xd6,0x00, 0x57,0x42,0x75,0x05,0xd6,0x00,
          0x00,0x00, 0,0,0,0,0,0,0,0,
          (uint8_t)(itu&0xff),(uint8_t)((itu>>8)&0xff), 0x00,0x00,
          0x00,0x03,'T','B','T', 0x01,0x01,0x82};
}
int main(int argc,char**argv){
  int SHORT = argc>1?atoi(argv[1]):80;
  uint8_t ch = 36; if(const char*c=std::getenv("DEVOURER_CHANNEL")) ch=atoi(c);
  auto logger=std::make_shared<Logger>(); apply_logging_env(*logger);
  libusb_context*ctx=nullptr; libusb_init(&ctx);
  libusb_set_option(ctx,LIBUSB_OPTION_LOG_LEVEL,LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid=0x0bda,pid=0xc812;
  if(const char*p=std::getenv("DEVOURER_PID")) pid=(uint16_t)strtoul(p,0,0);
  auto*h=libusb_open_device_with_vid_pid(ctx,vid,pid);
  if(!h){fprintf(stderr,"master open fail\n");return 1;}
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if(devourer::claim_interface_then_reset(h,0,logger,true,lk)!=0)return 1;
  WiFiDriver wifi(logger);
  auto dev=wifi.CreateRtlDevice(h,ctx,lk,devourer_config_from_env());
  if(!dev)return 1;
  dev->InitWrite(SelectedChannel{ch,0,CHANNEL_WIDTH_20});
  std::this_thread::sleep_for(std::chrono::seconds(2));
  dev->SetCcaMode(true);
  auto b100=beacon(100);
  dev->StartBeacon(b100.data(),b100.size(),100);
  fprintf(stderr,"MASTER beacon up @100TU, steady 7s then AdjustBeaconTiming\n");
  std::this_thread::sleep_for(std::chrono::seconds(7));
  int us = (SHORT-100)*1024;   // SHORT<100 => negative => advance (earlier)
  printf("TWEAK_MS %ld short=%d req_us=%d\n", now_ms(), SHORT, us); fflush(stdout);
  int applied = dev->AdjustBeaconTiming(us);                // productized primitive
  printf("RESTORE_MS %ld applied_us=%d\n", now_ms(), applied); fflush(stdout);
  std::this_thread::sleep_for(std::chrono::seconds(7));
  return 0;
}
EOF

# ---- observer source -----------------------------------------------------
cat > "$TMP/obs.cpp" <<'EOF'
#include <atomic>
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
int main(){
  auto logger=std::make_shared<Logger>();
  libusb_context*ctx=nullptr; libusb_init(&ctx);
  libusb_set_option(ctx,LIBUSB_OPTION_LOG_LEVEL,LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid=0x0bda,pid=0xa81a;
  if(const char*p=std::getenv("DEVOURER_PID")) pid=(uint16_t)strtoul(p,0,0);
  uint8_t ch=36; if(const char*c=std::getenv("DEVOURER_CHANNEL")) ch=atoi(c);
  auto*h=libusb_open_device_with_vid_pid(ctx,vid,pid);
  if(!h){fprintf(stderr,"obs open fail\n");return 1;}
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if(devourer::claim_interface_then_reset(h,0,logger,true,lk)!=0)return 1;
  WiFiDriver wifi(logger);
  auto dev=wifi.CreateRtlDevice(h,ctx,lk,devourer_config_from_env());
  if(!dev)return 1;
  auto cb=[](const Packet&p){
    if(p.Data.size()<32||p.RxAtrib.crc_err)return;
    if(std::memcmp(p.Data.data()+10,kSa,6)!=0)return;
    uint32_t tsfl=p.RxAtrib.tsfl;
    printf("BCN_MS %ld tsfl=%u phase=%u\n", now_ms(), tsfl, tsfl%102400);
    fflush(stdout);
  };
  std::thread rx([&]{dev->Init(cb,SelectedChannel{ch,0,CHANNEL_WIDTH_20});});
  rx.detach();
  std::this_thread::sleep_for(std::chrono::seconds(17));
  _exit(0);
}
EOF

echo "building master + observer ..."
g++ $CXXFLAGS "$TMP/master.cpp" examples/common/env_config.cpp "$LIB" $LDLIBS -o "$TMP/bshift_master" || exit 1
g++ $CXXFLAGS "$TMP/obs.cpp"    examples/common/env_config.cpp "$LIB" $LDLIBS -o "$TMP/bshift_obs"    || exit 1
cp "$TMP/bshift_master" /tmp/bshift_master; cp "$TMP/bshift_obs" /tmp/bshift_obs

echo "== run: master 8822CU one-shot 100->$SHORT->100 TU, observer 8822EU, ch$CH =="
sudo DEVOURER_CHANNEL="$CH" /tmp/bshift_obs >"$OLOG" 2>/dev/null &
sleep 3   # let observer's RX settle first
sudo DEVOURER_CHANNEL="$CH" /tmp/bshift_master "$SHORT" >"$MLOG" 2>/tmp/bshift_m.err &
wait
rm -f /tmp/bshift_master /tmp/bshift_obs

TWEAK=$(awk '/TWEAK_MS/{print $2}' "$MLOG")
echo "---- master ----"; cat "$MLOG"
echo "---- observer beacons (phase in us; watch for a step at TWEAK_MS=$TWEAK) ----"
awk -v tw="$TWEAK" '
  /BCN_MS/{ ms=$2; sub("tsfl=","",$3); sub("phase=","",$4);
            tag=(tw!=""&&ms>=tw)?"  <-- after tweak":"";
            printf "  ms=%s phase=%6s us%s\n", ms, $4, tag }' "$OLOG"
echo "---- interpretation ----"
echo "phase ~constant across the tweak  => interval-tweak does NOT move TBTT"
echo "phase steps ~$(( (100-SHORT)*1024 )) us at the tweak => interval-tweak IS the uplink-TA actuator"
