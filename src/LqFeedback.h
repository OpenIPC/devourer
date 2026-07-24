#pragma once
#include <cstdint>
#include <functional>
namespace apfpv {
class LqFeedback {
public:
    enum class Mode { FixedTimer, FrameDriven };
    // send_interval_ms: RSSI link-feedback uplink rate to greg's aalink on UDP 192.168.0.1:12345.
    // 33 ms (~30 Hz) = aalink's RSSI_SAMPLE_INTERVAL_MS, confirmed against the greg10.2 aalink binary
    // (port 12345 + "rssi_a = %d(%%)" format). NOTE: this is greg's *aalink*, NOT the public
    // OpenIPC/adaptive-link *alink* (port 9999, colon score 1000-2000, ~1-5 Hz) — different protocol,
    // don't conflate the two. Keep 33 ms to match greg10.2 (the dongle did 130 fps at this rate).
    struct Config { Mode mode = Mode::FixedTimer; int send_interval_ms = 33;
        int min_interval_ms = 20; int keepalive_ms = 0; double smoothing_alpha = 0.3; };
    static int rssiPct(int dbm);
    LqFeedback() ; explicit LqFeedback(Config cfg);
    ~LqFeedback() { stop(); }   // join thread + close socket on destroy (reconnect-safe)
    bool start(const char* airIp = "192.168.0.1", uint16_t port = 12345);
    // Route each LQ datagram through the caller instead of a host UDP socket. Needed on the
    // libusb dongle path (Win/WSL): the host has NO route to the VTX, so ::sendto dead-ends;
    // the sink lets ApfpvStation TX the payload over the dongle's own IP stack (sendIpPacket).
    void setSink(std::function<void(const char* buf, int len)> fn);
    void stop();
    void update(int rssiA_dbm, int rssiB_dbm = INT32_MIN);
};
}
