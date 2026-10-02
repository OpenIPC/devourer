/* DEVOURER_STA_IDENTITY / DEVOURER_STA_CLEAR_AFTER_MS - drive
 * IRadio::SetStationIdentity / ClearStationIdentity from rxdemo and txdemo.
 *
 * Demo-local (no DeviceConfig field): the seam is a runtime call that IRadio
 * orders after the RX loop is running, which a construction-time config
 * cannot express. The knob means the same on every backend - it calls the
 * seam and reports the return value; a backend that has not ported it returns
 * false, and the event says so.
 *
 *   DEVOURER_STA_IDENTITY=<own|self>,<bssid>
 *     own   the station's address, or `self` for the adapter's permanent
 *           (EFUSE) MAC via IRadio::GetPermanentMacAddress.
 *     bssid the AP's address.
 *   DEVOURER_STA_CLEAR_AFTER_MS=N
 *     N ms after a successful arm, call ClearStationIdentity - the on-air
 *     form of "Clear returns to the pre-arm state".
 *
 * Events (docs/logging.md): `sta.arm` {ok, own, bssid, attempts, [why]} once
 * the arm has been tried; `sta.clear` {ok} after a scheduled clear. A refused arm is
 * retried (a backend refuses before bring-up), ten times 500 ms apart; the
 * event reports the last outcome. */
#ifndef DEVOURER_STATION_ARM_ENV_H
#define DEVOURER_STATION_ARM_ENV_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <thread>

#include "DeviceConfig.h"
#include "Event.h"
#include "IRadio.h"
#include "logger.h"

namespace devourer {

struct StationArmRequest {
  bool own_self = false;
  MacAddr own{};
  MacAddr bssid{};
  uint32_t clear_after_ms = 0; /* 0 = never */
};

/* nullopt when DEVOURER_STA_IDENTITY is unset. `bad` is set when it is set
 * but malformed - the demo refuses to run rather than measure an unarmed
 * station as an armed one. */
inline std::optional<StationArmRequest>
station_arm_request_from_env(const Logger_t &log, bool &bad) {
  bad = false;
  const char *e = std::getenv("DEVOURER_STA_IDENTITY");
  if (e == nullptr || *e == '\0')
    return std::nullopt;
  const std::string v(e);
  const size_t comma = v.find(',');
  StationArmRequest r;
  std::optional<MacAddr> bssid;
  if (comma != std::string::npos) {
    const std::string own = v.substr(0, comma);
    bssid = parse_mac(v.substr(comma + 1));
    if (own == "self") {
      r.own_self = true;
    } else if (auto m = parse_mac(own)) {
      r.own = *m;
    } else {
      bssid.reset();
    }
  }
  if (!bssid) {
    log->error("DEVOURER_STA_IDENTITY='{}' is not <own|self>,<bssid>", e);
    bad = true;
    return std::nullopt;
  }
  r.bssid = *bssid;
  if (const char *c = std::getenv("DEVOURER_STA_CLEAR_AFTER_MS")) {
    char *end = nullptr;
    const unsigned long ms = std::strtoul(c, &end, 10);
    if (end == c || *end != '\0' || c[0] == '-' || ms > 3600000ul) {
      log->error("DEVOURER_STA_CLEAR_AFTER_MS='{}' is not 0..3600000", c);
      bad = true;
      return std::nullopt;
    }
    r.clear_after_ms = static_cast<uint32_t>(ms);
  }
  return r;
}

inline std::string station_mac_str(const MacAddr &m) {
  char b[18];
  std::snprintf(b, sizeof(b), "%02x:%02x:%02x:%02x:%02x:%02x", m.bytes[0],
                m.bytes[1], m.bytes[2], m.bytes[3], m.bytes[4], m.bytes[5]);
  return b;
}

/* Arm (with the bounded retry), emit `sta.arm`, and return the outcome. Call
 * after the RX loop has started (IRadio's ORDERING clause). `stop` aborts the
 * retry wait. `rx_ended`, when given, is set by the caller once its RX worker
 * has returned or thrown: an arm is refused while it is set (a station with no
 * receive loop is not one), and an arm that lands just as the worker ends is
 * cleared again and reported refused. Never holds a lock the RX callback
 * takes. */
inline bool station_arm_run(IRadio *dev, const StationArmRequest &req,
                            EventSink &ev, const Logger_t &log,
                            const std::atomic<bool> &stop,
                            const std::atomic<bool> *rx_ended = nullptr) {
  auto rx_gone = [&] { return rx_ended != nullptr && rx_ended->load(); };
  MacAddr own = req.own;
  if (req.own_self) {
    uint8_t m[6];
    if (!dev->GetPermanentMacAddress(m)) {
      log->error("DEVOURER_STA_IDENTITY: own=self, but this backend reports "
                 "no permanent MAC");
      Ev(ev, "sta.arm").f("ok", 0).f("own", nullptr).f("bssid",
          station_mac_str(req.bssid)).f("attempts", 0).f("why", "no_mac");
      return false;
    }
    for (int i = 0; i < 6; ++i)
      own.bytes[i] = m[i];
  }
  bool ok = false;
  int attempts = 0;
  while (attempts < 10 && !stop.load() && !rx_gone()) {
    ++attempts;
    ok = dev->SetStationIdentity(own, req.bssid);
    if (ok)
      break;
    for (int s = 0; s < 500 && !stop.load(); s += 50)
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const char *why = ok ? nullptr : "refused";
  if (rx_gone()) {
    if (ok)
      (void)dev->ClearStationIdentity();
    ok = false;
    why = "rx_not_running";
  }
  Ev e(ev, "sta.arm");
  e.f("ok", ok ? 1 : 0)
      .f("own", station_mac_str(own))
      .f("bssid", station_mac_str(req.bssid))
      .f("attempts", attempts);
  if (why)
    e.f("why", why);
  if (ok)
    log->info("DEVOURER_STA_IDENTITY: station identity armed (own {}, "
              "BSSID {}, attempt {})",
              station_mac_str(own), station_mac_str(req.bssid), attempts);
  else if (rx_gone())
    log->error("DEVOURER_STA_IDENTITY: not armed - the RX loop is not "
               "running (it failed or ended)");
  else
    log->error("DEVOURER_STA_IDENTITY: SetStationIdentity refused after {} "
               "attempt(s)",
               attempts);
  return ok;
}

/* The scheduled clear, after a successful arm. Emits `sta.clear`. */
inline void station_clear_after(IRadio *dev, const StationArmRequest &req,
                                EventSink &ev, const Logger_t &log,
                                const std::atomic<bool> &stop) {
  if (req.clear_after_ms == 0)
    return;
  for (uint32_t s = 0; s < req.clear_after_ms && !stop.load(); s += 50)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (stop.load())
    return;
  const bool ok = dev->ClearStationIdentity();
  Ev(ev, "sta.clear").f("ok", ok ? 1 : 0);
  if (ok)
    log->info("DEVOURER_STA_CLEAR_AFTER_MS: station identity cleared and "
              "verified");
  else
    log->error("DEVOURER_STA_CLEAR_AFTER_MS: ClearStationIdentity could not "
               "verify its rollback");
}

} /* namespace devourer */

#endif /* DEVOURER_STATION_ARM_ENV_H */
