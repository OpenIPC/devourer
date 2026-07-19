/* chanscout — passive second-adapter ground spectrum scout.
 *
 * Continuously surveys a configured candidate-channel plan while a separate
 * primary receiver stays parked on the live video channel. This process only
 * MEASURES: it retunes nothing but its own scout adapter and emits versioned
 * survey.dwell records (plus scout.id / scout.plan / scout.cand identity and
 * scout.health state) as JSONL on stdout. Channel-change decisions live
 * entirely in downstream consumers.
 *
 * Dwell discipline (the reason this is not just DEVOURER_RX_SWEEP): the
 * chip's FA/CCA counters are delta-on-read, so after the retune + settle the
 * executor performs a DISCARD read — resetting the hardware deltas and
 * draining frames still in the USB pipeline from the previous channel —
 * before opening the observation window. The record's counters therefore
 * span exactly observe_ms on exactly this bin.
 *
 * Wide candidates are surveyed as their 20 MHz constituent bins (the cheap
 * FastRetune path); DEVOURER_SCOUT_FULLWIDTH_MS optionally adds a periodic
 * real-width verification dwell through the full SetMonitorChannel gate.
 *
 * Env (demo-local; the library reads no environment):
 *   DEVOURER_SCOUT_PLAN        required — see the grammar in ChannelDef.h
 *   DEVOURER_SCOUT_DWELL_MS    observation window per dwell   (default 100)
 *   DEVOURER_SCOUT_SETTLE_MS   post-retune settle             (default 30)
 *   DEVOURER_SCOUT_BACKUP_MS   backup-candidate revisit bound (default 1000)
 *   DEVOURER_SCOUT_BG_MS       background revisit bound       (default 5000)
 *   DEVOURER_SCOUT_MAX_AGE_MS  evidence freshness bound       (default 60000)
 *   DEVOURER_SCOUT_FULLWIDTH_MS  wide verification cadence    (default 0=off)
 *   DEVOURER_SCOUT_NOTE        free text echoed in scout.id (antenna, slot)
 *   DEVOURER_VID/PID/USB_BUS/USB_PORT  adapter binding (usb_select.h)
 */
#include <libusb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "RxPacket.h"
#include "SignalStop.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "caps_event.h"
#include "chanmig/ChannelDef.h"
#include "chanmig/ChannelEvents.h"
#include "chanmig/ChannelScore.h"
#include "chanmig/PrimaryFeed.h"
#include "chanmig/ScanPlan.h"
#include "chanmig/SurveyJsonl.h"
#include "chanmig/SurveyRecord.h"
#include "env_config.h"
#include "logger.h"
#include "usb_select.h"
#include <optional>

#if defined(DEVOURER_HAVE_JAGUAR1)
#include "jaguar1/RtlJaguarDevice.h"
#endif

namespace cm = devourer::chanmig;

static devourer::EventSink *g_ev = nullptr;
static std::atomic<int> g_rx_count{0};

/* The canonical devourer TX SA (examples/tx, examples/streamtx, regress.py):
 * frames from it are OUR OWN video/probe traffic, split out of the occupancy
 * picture so the scoring layer never mistakes wanted airtime for an
 * interferer. */
static const uint8_t kDvrSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

/* Rolling per-dwell aggregate fed by the RX callback, drained at dwell
 * boundaries (rxdemo RxAgg shape + the airtime attribution buckets). */
struct ScoutAgg {
  uint32_t n = 0;
  int32_t rssi_sum = 0, rssi_max = -128, snr_sum = 0, snr_min = 127;
  int32_t evm_sum = 0;
  uint32_t evm_n = 0;
  uint32_t dvr_frames = 0;
  uint64_t dvr_air_us = 0, oth_air_us = 0;
};
static std::mutex g_agg_mu;
static ScoutAgg g_agg;

static void packetProcessor(const Packet &packet) {
  if (packet.RxAtrib.pkt_rpt_type == RX_PACKET_TYPE::C2H_PACKET)
    return;
  g_rx_count.fetch_add(1, std::memory_order_relaxed);
  const auto &a = packet.RxAtrib;
  const uint32_t air = cm::frame_airtime_us(
      a.data_rate, static_cast<uint32_t>(packet.Data.size()), a.bw,
      a.sgi != 0);
  const bool ours = packet.Data.size() >= 16 &&
                    std::memcmp(packet.Data.data() + 10, kDvrSa, 6) == 0;
  std::lock_guard<std::mutex> lk(g_agg_mu);
  if (a.rssi[0] > 0) {
    ++g_agg.n;
    g_agg.rssi_sum += a.rssi[0];
    if (a.rssi[0] > g_agg.rssi_max)
      g_agg.rssi_max = a.rssi[0];
    g_agg.snr_sum += a.snr[0];
    if (a.snr[0] < g_agg.snr_min)
      g_agg.snr_min = a.snr[0];
    if (a.evm[0] != 0) {
      g_agg.evm_sum += a.evm[0];
      ++g_agg.evm_n;
    }
  }
  if (ours) {
    ++g_agg.dvr_frames;
    g_agg.dvr_air_us += air;
  } else {
    g_agg.oth_air_us += air;
  }
}

static long long steady_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

static uint32_t env_u32(const char *name, uint32_t def) {
  const char *e = std::getenv(name);
  return (e && *e) ? static_cast<uint32_t>(std::strtoul(e, nullptr, 0)) : def;
}

/* Chunked stop-aware sleep (the repo's 50 ms convention keeps SIGINT
 * latency bounded); returns false when interrupted. */
static bool nap_ms(int64_t ms) {
  for (int64_t s = 0; s < ms; s += 50) {
    if (g_devourer_should_stop)
      return false;
    const int64_t step = std::min<int64_t>(50, ms - s);
    std::this_thread::sleep_for(std::chrono::milliseconds(step));
  }
  return !g_devourer_should_stop;
}

/* Health-state tracker: emits scout.health only on state/reason transitions
 * so a long degraded stretch is one line, not a firehose. */
struct HealthReporter {
  std::string last_key;
  void report(const char *state, const char *reason, long long detail) {
    std::string key = std::string(state) + "/" + reason;
    if (key == last_key)
      return;
    last_key = key;
    devourer::Ev(*g_ev, "scout.health")
        .t()
        .f("state", state)
        .f("reason", reason)
        .f("detail", detail);
  }
  void ok() {
    if (last_key == "ok/")
      return;
    last_key = "ok/";
    devourer::Ev(*g_ev, "scout.health").t().f("state", "ok").f("reason", "");
  }
};

int main() {
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  g_ev = &logger->events();
  install_devourer_signal_handlers();

  /* --- plan --- */
  const char *plan_env = std::getenv("DEVOURER_SCOUT_PLAN");
  cm::ScanPlanConfig cfg;
  {
    std::vector<cm::PlanParseError> errs;
    const bool ok = cm::parse_scan_plan(plan_env, cfg.candidates, errs);
    for (const auto &e : errs) {
      logger->error("DEVOURER_SCOUT_PLAN token '{}': {}", e.token, e.reason);
      devourer::Ev(*g_ev, "scout.health")
          .t()
          .f("state", "fatal")
          .f("reason", "plan_error")
          .f("token", e.token.c_str())
          .f("what", e.reason.c_str());
    }
    /* A candidate list with ANY unparseable token must not run: a silently
     * missing migration candidate is a field hazard, not a warning. */
    if (!ok) {
      logger->error("DEVOURER_SCOUT_PLAN is required and must parse cleanly "
                    "(grammar: src/chanmig/ChannelDef.h)");
      return 2;
    }
  }
  cfg.dwell_ms = static_cast<int>(env_u32("DEVOURER_SCOUT_DWELL_MS", 100));
  cfg.settle_ms = static_cast<int>(env_u32("DEVOURER_SCOUT_SETTLE_MS", 30));
  cfg.backup_revisit_ms =
      static_cast<int>(env_u32("DEVOURER_SCOUT_BACKUP_MS", 1000));
  cfg.bg_revisit_ms = static_cast<int>(env_u32("DEVOURER_SCOUT_BG_MS", 5000));
  cfg.fullwidth_ms =
      static_cast<int>(env_u32("DEVOURER_SCOUT_FULLWIDTH_MS", 0));
  cfg.max_age_ms = env_u32("DEVOURER_SCOUT_MAX_AGE_MS", 60000);
  const uint32_t plan_hash = cfg.plan_hash();

  /* --- advise mode (DEVOURER_SCOUT_ADVISE=1) --- runs the RecommendEngine
   * in-process (it already owns the survey records) and tails the primary
   * receiver's JSONL for the active-link delivery evidence. It emits only
   * advice; it retunes nothing. */
  const bool advise = std::getenv("DEVOURER_SCOUT_ADVISE") != nullptr &&
                      std::strcmp(std::getenv("DEVOURER_SCOUT_ADVISE"), "0") != 0;
  std::optional<cm::RecommendEngine> engine;
  cm::PrimaryFeedReader feed;
  cm::ChannelDef active_def;
  if (advise) {
    const char *act = std::getenv("DEVOURER_SCOUT_ACTIVE");
    std::string aerr;
    if (act == nullptr || !cm::parse_chan_token(act, active_def, aerr)) {
      logger->error("DEVOURER_SCOUT_ADVISE needs a valid DEVOURER_SCOUT_ACTIVE "
                    "channel token ({})", act ? aerr : "unset");
      return 2;
    }
    cm::PolicyConfig pol;
    if (const char *pf = std::getenv("DEVOURER_SCOUT_POLICY")) {
      std::string perr;
      if (!cm::parse_policy_file(pf, pol, &perr))
        logger->warn("DEVOURER_SCOUT_POLICY {}: {} — using defaults", pf, perr);
    }
    engine.emplace(pol, cfg.candidates, plan_hash, active_def);
  }

  /* --- adapter --- */
  libusb_context *ctx = nullptr;
  int rc = libusb_init(&ctx);
  if (rc < 0)
    return rc;
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL,
                    std::getenv("DEVOURER_USB_DEBUG") ? LIBUSB_LOG_LEVEL_DEBUG
                                                      : LIBUSB_LOG_LEVEL_WARNING);
  /* The scout accepts any devourer-supported chip; the default PID list is
   * rxdemo's. Two identical adapters are told apart per-process via
   * DEVOURER_USB_BUS/PORT (strict, no fallback). */
  static constexpr uint16_t kPids[] = {
      0x8812, 0x0811, 0xa811, 0xb811, 0x8813, 0xb812, 0xb82c, 0xc811,
      0xc82c, 0xc82e, 0xc812, 0x881a, 0x881b, 0x881c, 0xa81a, 0xe822,
      0xa82a,
  };
  UsbPick pick;
  libusb_device_handle *handle = open_selected_usb(
      ctx, logger, kPids, sizeof(kPids) / sizeof(kPids[0]), &pick);
  if (handle == nullptr) {
    libusb_exit(ctx);
    return 1;
  }
  std::shared_ptr<devourer::UsbDeviceLock> usb_lock;
  rc = devourer::claim_interface_reset_reopen(
      ctx, handle, logger, std::getenv("DEVOURER_SKIP_RESET") == nullptr,
      usb_lock);
  if (rc != 0) {
    if (handle != nullptr)
      libusb_close(handle);
    libusb_exit(ctx);
    return 1;
  }

  WiFiDriver driver(logger);
  auto dev = driver.CreateRtlDevice(handle, ctx, usb_lock,
                                    devourer_config_from_env());
  if (!dev) {
    logger->error("No driver for this chip in this build — exiting");
    return 1;
  }
  devourer::emit_adapter_caps(*g_ev, dev.get());
  const devourer::AdapterCaps caps = dev->GetAdapterCaps();

  /* Stable scout identity = the physical binding + silicon (FNV-1a). This is
   * the calibration-domain key: evidence is only comparable within one
   * scout_id, and the aggregator resets when it changes. */
  uint32_t scout_id = 2166136261u;
  {
    auto fold = [&scout_id](const void *p, size_t n) {
      const uint8_t *b = static_cast<const uint8_t *>(p);
      for (size_t i = 0; i < n; i++) {
        scout_id ^= b[i];
        scout_id *= 16777619u;
      }
    };
    fold(&pick.vid, sizeof(pick.vid));
    fold(&pick.pid, sizeof(pick.pid));
    fold(&pick.bus, sizeof(pick.bus));
    fold(pick.port.data(), pick.port.size());
    fold(caps.chip_name, std::strlen(caps.chip_name));
  }

  {
    devourer::Ev ev(*g_ev, "scout.id");
    ev.t().f("role", "scout");
    char id[16];
    std::snprintf(id, sizeof(id), "%04x:%04x", pick.vid, pick.pid);
    ev.f("usb_id", id)
        .f("bus", pick.bus)
        .f("port", pick.port.empty() ? "?" : pick.port.c_str())
        .f("usb_speed", pick.speed)
        .f("chip", caps.chip_name)
        .f("gen", devourer::generation_name(caps.generation));
    ev.hexf("scout_id", scout_id, 8);
    if (const char *note = std::getenv("DEVOURER_SCOUT_NOTE"))
      ev.f("note", note);
  }
  {
    /* One CLOCK_REALTIME sample so offline tooling can align this stream
     * with the primary receiver's (both processes log monotonic t). */
    const long long epoch_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    devourer::Ev ev(*g_ev, "scout.plan");
    ev.t().f("v", cm::kSurveySchemaV);
    ev.hexf("plan", plan_hash, 8);
    ev.f("epoch_unix_ms", epoch_ms)
        .f("dwell_ms", cfg.dwell_ms)
        .f("settle_ms", cfg.settle_ms)
        .f("backup_ms", cfg.backup_revisit_ms)
        .f("bg_ms", cfg.bg_revisit_ms)
        .f("fullwidth_ms", cfg.fullwidth_ms)
        .f("max_age_ms", (long long)cfg.max_age_ms)
        .f("n_candidates", (unsigned long long)cfg.candidates.size())
        .f("spec", plan_env);
  }
  for (size_t i = 0; i < cfg.candidates.size(); ++i) {
    const cm::ChannelDef &c = cfg.candidates[i];
    uint8_t bins[4];
    const int nb = cm::constituent_bins(c, bins);
    int ib[4];
    for (int k = 0; k < nb; k++)
      ib[k] = bins[k];
    devourer::Ev ev(*g_ev, "scout.cand");
    ev.f("i", (unsigned long long)i)
        .f("chan", c.str().c_str())
        .f("center_mhz", c.center_mhz())
        .f("backup", c.backup)
        .f("no_ir", c.no_ir)
        .f("dfs", c.dfs);
    ev.arr("bins", ib, static_cast<size_t>(nb));
  }

  cm::ScanScheduler sched(cfg);

  /* --- RX loop on a worker thread (rxdemo sweep pattern) --- */
  IRtlDevice *devp = dev.get();
  const cm::ScanScheduler::DwellPlan first = sched.next(steady_ms());
  std::thread rx([devp, first, &logger]() {
    try {
      devp->Init(packetProcessor, first.def.to_selected());
    } catch (const std::exception &e) {
      logger->error("scout RX bring-up failed: {}", e.what());
      g_devourer_should_stop = true;
    }
  });
  /* Let bring-up finish before the first retune (a retune racing FW download
   * or calibration interleaves register writes). A frame proves RX is live;
   * a silent band falls through after the conservative 10 s cap. */
  for (int w = 0; w < 10000 && !g_devourer_should_stop && g_rx_count == 0;
       w += 50)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

#if defined(DEVOURER_HAVE_JAGUAR1)
  /* Thermal health where the sensor exists (Jaguar1 RF 0x42 poller). */
  auto *j1 = dynamic_cast<RtlJaguarDevice *>(devp);
  const uint32_t thermal_ms = env_u32("DEVOURER_THERMAL_POLL_MS", 0);
  if (j1 != nullptr && thermal_ms > 0)
    j1->start_thermal_poller(thermal_ms,
                             static_cast<int>(env_u32(
                                 "DEVOURER_THERMAL_WARN_DELTA", 15)));
#endif

  HealthReporter health;
  uint64_t seq = 0;
  int energy_invalid_streak = 0;
  bool energy_ever_valid = false;
  int quiet_dwells = 0;
  bool ever_active = false;
  std::vector<int64_t> retune_ring;
  int64_t retune_p95_baseline = 0;
  bool last_was_wide = false;
  /* Per-bin last-successful-observation, for the stale-survey health check. */
  struct BinAge {
    uint8_t ch;
    int64_t last_ok_ms;
  };
  std::vector<BinAge> bin_ages;

  /* Advise-mode feed follower + decision cadence. The primary feed is a plain
   * append-only file (never a FIFO — a wedged scout must not stall the video
   * receiver); open it lazily and read to EOF each pump. */
  std::FILE *feed_fp = nullptr;
  std::string feed_path;
  if (advise) {
    if (const char *fp = std::getenv("DEVOURER_SCOUT_PRIMARY_FEED"))
      feed_path = fp;
    else
      logger->warn("DEVOURER_SCOUT_ADVISE without DEVOURER_SCOUT_PRIMARY_FEED "
                   "— recommendations will lack active-link evidence");
  }
  std::string feed_buf;
  int64_t last_decide_ms = 0, last_emit_ms = 0;
  cm::Reason last_reason = cm::Reason::HoldPrimaryTelemetryStale;
  const int64_t kDecideEveryMs = 2000, kHoldEveryMs = 10000;
  auto pump_feed = [&](int64_t now) {
    if (!advise || feed_path.empty() || engine == std::nullopt)
      return;
    if (feed_fp == nullptr) {
      feed_fp = std::fopen(feed_path.c_str(), "r");
      if (feed_fp == nullptr)
        return; /* primary not up yet; retry next pump */
    }
    char chunk[4096];
    size_t n;
    while ((n = std::fread(chunk, 1, sizeof(chunk), feed_fp)) > 0) {
      feed_buf.append(chunk, n);
      size_t nl;
      while ((nl = feed_buf.find('\n')) != std::string::npos) {
        std::string_view line(feed_buf.data(), nl);
        cm::ActiveLinkWindow w;
        if (feed.line(line, now, w))
          engine->ingest_active(w);
        feed_buf.erase(0, nl + 1);
      }
    }
    std::clearerr(feed_fp); /* EOF now, but more may append — keep the handle */
  };

  while (!g_devourer_should_stop) {
    const int64_t t_start = steady_ms();
    cm::ScanScheduler::DwellPlan plan = sched.next(t_start);
    if (!plan.valid)
      break;

    cm::SurveyDwell d;
    d.seq = seq++;
    d.def = plan.def;
    d.round = plan.round;
    d.plan_hash = plan_hash;
    d.t_start_ms = t_start;
    d.settle_ms = cfg.settle_ms;
    d.scout_id = scout_id;
    d.adapter_gen = static_cast<uint8_t>(caps.generation);
    if (plan.full_width)
      d.flags |= cm::kFlagFullWidth;

    /* retune. FastRetune is the same-width lean path, so the first bin dwell
     * after a wide verification dwell must go through the full gate to
     * restore 20 MHz — otherwise every subsequent "bin" would observe at the
     * candidate's width. */
    bool tuned = false;
    const auto rt0 = std::chrono::steady_clock::now();
    try {
      if (plan.full_width) {
        devp->SetMonitorChannel(plan.def.to_selected());
        last_was_wide = true;
      } else if (last_was_wide) {
        devp->SetMonitorChannel(plan.def.to_selected());
        last_was_wide = false;
      } else {
        devp->FastRetune(plan.bin_ch, /*cache_rf=*/true);
      }
      tuned = true;
    } catch (const std::exception &e) {
      logger->warn("scout retune ch{} failed: {}", plan.bin_ch, e.what());
    }
    d.retune_us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now() - rt0)
                      .count();

    if (!tuned) {
      d.flags |= cm::kFlagRetuneFailed;
      d.t_end_ms = steady_ms();
      d.observe_ms = 0;
      cm::emit_survey_dwell(*g_ev, d);
      sched.complete(plan, steady_ms(), false);
      const int cf = sched.consecutive_failures();
      if (cf >= 15) {
        health.report("wedged", "retune_fail", cf);
        break; /* supervisor restarts us; exit code says why */
      }
      if (cf >= 5)
        health.report("degraded", "retune_fail", cf);
      nap_ms(50);
      continue;
    }

    /* settle, then the DISCARD BARRIER: reset the delta counters and drain
     * frames that raced in from the previous channel. */
    if (!nap_ms(cfg.settle_ms))
      d.flags |= cm::kFlagTruncated;
    (void)devp->GetRxEnergy();
    {
      std::lock_guard<std::mutex> lk(g_agg_mu);
      g_agg = ScoutAgg{};
    }
    const int64_t t_obs = steady_ms();

    if (!nap_ms(cfg.dwell_ms))
      d.flags |= cm::kFlagTruncated;

    RxEnergy e = devp->GetRxEnergy();
    ScoutAgg agg;
    {
      std::lock_guard<std::mutex> lk(g_agg_mu);
      agg = g_agg;
      g_agg = ScoutAgg{};
    }
    d.t_end_ms = steady_ms();
    d.observe_ms = d.t_end_ms - t_obs;

    d.valid_fa = e.valid_fa;
    d.fa_ofdm = e.fa_ofdm;
    d.fa_cck = e.fa_cck;
    d.cca_ofdm = e.cca_ofdm;
    d.cca_cck = e.cca_cck;
    d.valid_igi = e.valid_igi;
    d.igi = e.igi;
    d.valid_nhm = e.valid_nhm;
    if (e.valid_nhm) {
      uint32_t total = 0, peak = 0;
      int peak_k = 0;
      for (int k = 0; k < 12; k++) {
        d.nhm[k] = e.nhm[k];
        total += e.nhm[k];
        if (e.nhm[k] > peak) {
          peak = e.nhm[k];
          peak_k = k;
        }
      }
      d.nhm_dur = e.nhm_duration;
      d.nhm_peak = static_cast<uint8_t>(peak_k);
      d.nhm_busy_pct = static_cast<uint8_t>(
          total ? 100 * (total - e.nhm[0]) / total : 0);
    } else {
      d.flags |= cm::kFlagNhmMissing;
    }
    /* Producer-side counter plausibility (the aggregator re-checks): a delta
     * beyond ~1000 events/ms of observation is a wrapped/reset counter. */
    if (e.valid_fa && d.observe_ms > 0) {
      const uint64_t ceiling =
          1000ull * static_cast<uint64_t>(d.observe_ms);
      if (d.cca_ofdm > ceiling || d.fa_ofdm > ceiling ||
          d.cca_cck > ceiling || d.fa_cck > ceiling)
        d.flags |= cm::kFlagCounterSuspect;
    }
    d.frames = agg.n;
    if (agg.n) {
      d.rssi_mean_raw = agg.rssi_sum / static_cast<int>(agg.n);
      d.rssi_max_raw = agg.rssi_max;
      d.snr_mean_raw = agg.snr_sum / static_cast<int>(agg.n);
      d.snr_min_raw = agg.snr_min;
    }
    if (agg.evm_n) {
      d.evm_mean_raw = agg.evm_sum / static_cast<int>(agg.evm_n);
      d.evm_valid = true;
    }
    d.dvr_frames = agg.dvr_frames;
    d.dvr_air_us = agg.dvr_air_us;
    d.oth_air_us = agg.oth_air_us;

    /* A generation whose energy counters were once valid going invalid is a
     * read failure, not a quiet channel — flag the record before it flies. */
    if (e.valid_fa)
      energy_ever_valid = true;
    energy_invalid_streak = e.valid_fa ? 0 : energy_invalid_streak + 1;
    if (energy_ever_valid && !e.valid_fa)
      d.flags |= cm::kFlagReadFailed;

    cm::emit_survey_dwell(*g_ev, d);
    const bool ok = (d.flags & (cm::kFlagRetuneFailed | cm::kFlagTruncated)) == 0;
    sched.complete(plan, d.t_end_ms, ok);
    if (advise && engine != std::nullopt)
      engine->ingest_dwell(d, d.t_end_ms);

    /* --- health --- */
    if (energy_ever_valid && energy_invalid_streak >= 10)
      health.report("degraded", "energy_read_fail", energy_invalid_streak);
    const bool active = agg.n > 0 || (e.valid_fa && (e.cca_ofdm | e.cca_cck));
    if (active) {
      ever_active = true;
      quiet_dwells = 0;
    } else if (++quiet_dwells >= 50 && ever_active) {
      /* Everything silent after having heard traffic: a wedged RX front end
       * looks exactly like a suddenly-empty world — flag it, let the bench
       * decide. */
      health.report("degraded", "rx_stalled", quiet_dwells);
    }
    if (!plan.full_width) {
      retune_ring.push_back(d.retune_us);
      if (retune_ring.size() > 100)
        retune_ring.erase(retune_ring.begin());
      if (retune_ring.size() == 100) {
        std::vector<int64_t> sorted = retune_ring;
        std::sort(sorted.begin(), sorted.end());
        const int64_t p95 = sorted[95];
        if (retune_p95_baseline == 0)
          retune_p95_baseline = p95 > 0 ? p95 : 1;
        else if (p95 > 5 * retune_p95_baseline)
          health.report("degraded", "usb_congested", p95);
      }
    }
#if defined(DEVOURER_HAVE_JAGUAR1)
    if (j1 != nullptr && thermal_ms > 0) {
      const auto t = j1->get_thermal_snapshot();
      if (t.valid && t.delta >= static_cast<int>(env_u32(
                                    "DEVOURER_THERMAL_WARN_DELTA", 15)))
        health.report("degraded", "thermal", t.delta);
    }
#endif
    {
      /* Stale-survey: any bin unobserved past the freshness bound. */
      BinAge *slot = nullptr;
      for (BinAge &b : bin_ages)
        if (b.ch == plan.bin_ch)
          slot = &b;
      if (slot == nullptr) {
        bin_ages.push_back(BinAge{plan.bin_ch, d.t_end_ms});
        slot = &bin_ages.back();
      }
      if (ok)
        slot->last_ok_ms = d.t_end_ms;
      bool stale = false;
      long long worst = 0;
      for (const BinAge &b : bin_ages) {
        const int64_t age = d.t_end_ms - b.last_ok_ms;
        if (age > cfg.max_age_ms) {
          stale = true;
          if (age > worst)
            worst = age;
        }
      }
      if (stale)
        health.report("degraded", "stale_survey", worst);
      else if (sched.consecutive_failures() == 0 &&
               energy_invalid_streak < 10 && quiet_dwells < 50)
        health.ok();
    }

    /* --- advise: pump the primary feed, decide on a cadence --- */
    if (advise && engine != std::nullopt) {
      const int64_t now = d.t_end_ms;
      pump_feed(now);
      engine->note_scout_health(sched.consecutive_failures() < 5);
      if (now - last_decide_ms >= kDecideEveryMs) {
        last_decide_ms = now;
        cm::Decision dec = engine->decide(now);
        /* Emit on a recommendation, a reason change, or the steady hold
         * cadence — so a dashboard's counterfactual stays fresh without spam. */
        if (dec.kind == cm::Decision::Kind::Recommend ||
            dec.primary_reason != last_reason ||
            now - last_emit_ms >= kHoldEveryMs) {
          cm::emit_decision(*g_ev, dec, active_def);
          cm::emit_ranking(*g_ev, dec);
          last_reason = dec.primary_reason;
          last_emit_ms = now;
        }
      }
    }
  }

  if (feed_fp != nullptr)
    std::fclose(feed_fp);
  devp->StopRxLoop();
  if (rx.joinable())
    rx.join();
  devp->Stop();
  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(ctx);
  return sched.consecutive_failures() >= 15 ? 3 : 0;
}
