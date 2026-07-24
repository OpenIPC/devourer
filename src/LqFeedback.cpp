// ============================================================================
//  LqFeedback.cpp — downlink LQ sender to aalink UDP 192.168.0.1:12345 (impl)
//  Declarations in LqFeedback.h. Modeled on aalink's documented input
//  ("rssi : %d" / per-antenna form) and RSSI_SAMPLE_INTERVAL_MS=33. Two modes.
// ============================================================================
#include "LqFeedback.h"
#include <cstdio>
#if defined(__ANDROID__)
#include <android/log.h>
#endif
#include <cstring>
#include <cstdlib>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <functional>
#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#define close closesocket
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif
#if defined(__ANDROID__)
#include <android/log.h>
#include <sys/system_properties.h>
#endif

namespace apfpv {

// LQ send interval override, live-tunable without a rebuild. Default (33ms/30Hz, matching
// aalink's own RSSI_SAMPLE_INTERVAL_MS) sends 2 packets/cycle through the SAME shared dongle
// TX path (sendIpPacket -> sendStationFrameSync -> a real synchronous USBDEVFS_BULK ioctl) that
// video RX competes with -- 60 TX calls/sec, continuously, for the whole streaming session,
// independent of any other feature (SSH route, etc). aalink's OWN decision cadence is on the
// order of SECONDS (UP_COOLDOWN_MS=2000 in aalink.conf), so 30Hz RSSI resolution is almost
// certainly far finer than its control loop needs -- worth testing whether a slower cadence
// reduces TX-path contention (and therefore RX hiccups at high throughput) without harming
// aalink's actual bitrate adaptation. DEVOURER_LQ_MS (host) / debug.pixelpilot.lqms (Android);
// 0/unset = unchanged default behavior.
static int lqSendIntervalOverrideMs() {
    if (const char* e = std::getenv("DEVOURER_LQ_MS")) { int n = std::atoi(e); if (n > 0) return n; }
#if defined(__ANDROID__)
    char v[PROP_VALUE_MAX] = {0};
    if (__system_property_get("debug.pixelpilot.lqms", v) > 0) {
        int n = std::atoi(v);
        if (n > 0) return n;
    }
#endif
    return 0;
}

// internal state (PImpl-lite via file-static per instance would be cleaner;
// kept as members through a small struct mirrored from the header).
struct LqState {
    LqFeedback::Config cfg;
    int sock = -1;
    sockaddr_in dst{};
    std::atomic<int> a{-80}, b{-80};
    bool haveB = false;
    std::atomic<bool> run{false};
    std::thread th;
    std::mutex m; std::condition_variable cv; bool fresh = false;
    std::function<void(const char*, int)> sink;  // if set, route emit here (dongle IP TX) not the socket
};

// Single global map would be needed for many instances; APFPV uses one link,
// so we keep one state object behind the class via a static (documented).
static LqState g;

int LqFeedback::rssiPct(int dbm) {
    // EXACT OpenIPC ground-station mapping (from sbc-groundstations gsmenu.sh:
    //   sig_pct = int(2 * (signal_dBm + 100)), clamped 0..100).
    // This is the calibration the air's aalink THRESH tables were tuned against,
    // and it is STABLE across greg08/09/10/10.1/10.2 and current sbc-gs builds
    // (the air-side THRESH/SCALE_RSSI values are essentially unchanged). Our
    // earlier linear (-90..-20 => 0..100) ran ~30 points COLD vs this — feeding
    // aalink low percentages and biasing it to over-conservative MCS.
    int pct = 2 * (dbm + 100);
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

LqFeedback::LqFeedback() { g.cfg = Config{}; }
LqFeedback::LqFeedback(Config cfg) { g.cfg = cfg; }
void LqFeedback::setSink(std::function<void(const char*, int)> fn) { g.sink = std::move(fn); }

static void emit(double& sA, double& sB, bool& init) {
    int a = g.a.load(), b = g.b.load();
    double alpha = g.cfg.smoothing_alpha;
    if (!init) { sA = a; sB = b; init = true; }
    else { sA = alpha*a + (1-alpha)*sA; sB = alpha*b + (1-alpha)*sB; }
    char buf[64]; int n;
    // aalink parses: "%*[^=]=%*s rssi_a = %d(%%), rssi_b = %d(%%)" — i.e. it
    // skips a leading "token=word " prefix, THEN reads the rssi fields. Without
    // the prefix the sscanf fails and the ground RSSI is ignored. Match the
    // firmware exactly (confirmed against the greg10.2 aalink binary).
    int pctA = LqFeedback::rssiPct((int)sA), pctB = LqFeedback::rssiPct((int)sB);
    if (g.haveB)
        n = std::snprintf(buf, sizeof(buf), "gs_string=gs rssi_a = %d(%%), rssi_b = %d(%%)\n", pctA, pctB);
    else
        n = std::snprintf(buf, sizeof(buf), "gs_string=gs rssi_a = %d(%%)\n", pctA);
    if (n > 0) {
        if (g.sink) g.sink(buf, n);
        else if (g.sock >= 0) ::sendto(g.sock, buf, (size_t)n, 0, (sockaddr*)&g.dst, sizeof(g.dst));
    }
    // Also send the BARE percentage that the phone-Wi-Fi path uses (ApfpvWifiManager sends
    // Integer.toString(pct)) — that variant is confirmed to move the VTX downlink %, whereas the
    // verbose "gs_string=" form above may not parse on the current aalink. Belt-and-suspenders.
    char bare[16]; int bn = std::snprintf(bare, sizeof(bare), "%d", pctA);
    if (bn > 0) {
        if (g.sink) g.sink(bare, bn);
        else if (g.sock >= 0) ::sendto(g.sock, bare, (size_t)bn, 0, (sockaddr*)&g.dst, sizeof(g.dst));
    }
#if defined(__ANDROID__)
    // DIAGNOSTIC: surface the RSSI→pct we are actually sending (rules out a 0-value RSSI conversion).
    static int dbgN = 0;
    if ((dbgN++ % 30) == 0)
        __android_log_print(4, "apfpv-lq", "LQ send rssiA=%.0f pctA=%d (haveB=%d pctB=%d)",
                            sA, pctA, (int)g.haveB, pctB);
#endif
}

static void loopFixed() {
    using namespace std::chrono; double sA=0,sB=0; bool init=false;
    while (g.run) {
        emit(sA,sB,init);
        int ms = lqSendIntervalOverrideMs();
        std::this_thread::sleep_for(milliseconds(ms > 0 ? ms : g.cfg.send_interval_ms));
    }
}
static void loopFrameDriven() {
    using namespace std::chrono; double sA=0,sB=0; bool init=false;
    while (g.run) {
        std::unique_lock<std::mutex> lk(g.m);
        auto wait = g.cfg.keepalive_ms > 0 ? milliseconds(g.cfg.keepalive_ms) : milliseconds(1000);
        g.cv.wait_for(lk, wait, []{ return g.fresh || !g.run; });
        bool wasFresh = g.fresh; g.fresh = false; lk.unlock();
        if (!g.run) break;
        if (wasFresh || g.cfg.keepalive_ms > 0) {
            emit(sA,sB,init);
            std::this_thread::sleep_for(milliseconds(g.cfg.min_interval_ms));
        }
    }
}

bool LqFeedback::start(const char* airIp, uint16_t port) {
    // A prior start() may have left a running thread (e.g. on replug/reconnect
    // the connect chain re-runs without stop()). Move-assigning g.th below while
    // it is still joinable calls std::terminate(). Tear the old one down first.
    if (g.th.joinable()) stop();
    // When a sink is set (libusb dongle path), emit routes through it — no host socket needed.
    // The host ::socket() also needs WSAStartup on Windows (only run lazily elsewhere), so a
    // socket-open failure must NOT prevent the emit thread from starting.
    if (!g.sink) {
        g.sock = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (g.sock < 0) return false;
        std::memset(&g.dst, 0, sizeof(g.dst));
        g.dst.sin_family = AF_INET; g.dst.sin_port = htons(port);
        ::inet_pton(AF_INET, airIp, &g.dst.sin_addr);
    }
    g.run = true;
    g.th = std::thread(g.cfg.mode == Mode::FixedTimer ? loopFixed : loopFrameDriven);
    return true;
}
void LqFeedback::stop() {
    g.run = false; g.cv.notify_all();
    if (g.th.joinable()) g.th.join();
    if (g.sock >= 0) { ::close(g.sock); g.sock = -1; }
}
void LqFeedback::update(int rssiA_dbm, int rssiB_dbm) {
    g.a.store(rssiA_dbm);
    if (rssiB_dbm != INT32_MIN) { g.b.store(rssiB_dbm); g.haveB = true; }
    if (g.cfg.mode == Mode::FrameDriven) {
        { std::lock_guard<std::mutex> lk(g.m); g.fresh = true; }
        g.cv.notify_one();
    }
}

} // namespace apfpv
