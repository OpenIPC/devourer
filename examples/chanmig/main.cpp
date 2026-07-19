/* chanmig — the coordinated channel-migration endpoint (issue #278).
 *
 *   --role ground : owns the primary RX (duplex: RX video + TX proposals on one
 *                   claimed adapter), runs the MigProposer + PeerClock, takes
 *                   operator commands on stdin, and follows the drone's
 *                   authoritative status.
 *   --role drone  : the video TX. Runs the MigResponder, is the final schedule
 *                   authority, arms its ACK responder, and airs generation-
 *                   tagged markers on the destination at activation.
 *
 * Control frames are their OWN 802.11 frames (canonical-SA probe-req header +
 * the "CM" magic + an authenticated MigWire message) — video PSDUs are never
 * touched, satisfying "do not bury migration commands in caller-owned payload
 * bytes". The pure state machines (src/chanmig/Mig*.h) do all the logic; this
 * binary only stamps TSFs, encodes/MACs, transmits, retunes, and emits the
 * migrate.* JSONL. Both endpoints derive a control key from DEVOURER_MIG_KEY.
 *
 * Env (demo-local): DEVOURER_MIG_ROLE (or --role), DEVOURER_MIG_KEY (control
 * key hex), DEVOURER_MIG_ALLOWED (drone allowed-channel list),
 * DEVOURER_MIG_RESCUE, DEVOURER_MIG_LINK (link id), DEVOURER_MIG_SYNTH_PPS/
 * _LEN (drone synthetic video), DEVOURER_MIG_LEAD_MS, DEVOURER_MIG_DROP /
 * _DROP_RX (deterministic fault filters), DEVOURER_CHANNEL (start channel),
 * plus the usual device-selection vars. */
#include <libusb.h>

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

#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "SignalStop.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "chanmig/ChannelDef.h"
#include "chanmig/JsonlLite.h"
#include "chanmig/MigClock.h"
#include "chanmig/MigConfig.h"
#include "chanmig/MigGate.h"
#include "chanmig/MigProposer.h"
#include "chanmig/MigResponder.h"
#include "chanmig/MigWire.h"
#include "env_config.h"
#include "logger.h"
#include "usb_select.h"

namespace cm = devourer::chanmig;
using devourer::Ev;

static devourer::EventSink *g_ev = nullptr;
static IRtlDevice *g_dev = nullptr;
static std::mutex g_dev_mu; /* serialize send/retune against the RX thread */
/* The pure state machines are single-threaded by design; the demo drives them
 * from the RX callback, the tick loop, and (ground) the operator thread, so
 * every machine entry point takes this lock. Ordering: g_sm_mu before g_dev_mu
 * (send/retune/read_tsf take g_dev_mu inside an already-held g_sm_mu). */
static std::recursive_mutex g_sm_mu;
static cm::MigKey g_key;
static uint32_t g_link = 0;
static std::atomic<bool> g_stop{false};
static const uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

/* deterministic fault filters (demo-side, for the on-air failure scripts) */
static std::vector<cm::DropRule> g_drop_tx, g_drop_rx;
static int g_tx_idx[8] = {}, g_rx_idx[8] = {};

static long long now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

static std::vector<uint8_t> probe_req_hdr() {
  std::vector<uint8_t> h = {0x40, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff,
                            0xff, 0xff, 0xff};
  h.insert(h.end(), kCanonicalSa, kCanonicalSa + 6);
  h.insert(h.end(), kCanonicalSa, kCanonicalSa + 6);
  h.push_back(0x80);
  h.push_back(0x00);
  return h;
}

/* Build + transmit one migration control/marker frame: robust 6M radiotap +
 * probe-req header + the authenticated MigWire bytes. The caller has already
 * stamped tx_tsf. Honors the TX drop filter (deterministic fault injection). */
static void tx_mig(const cm::MigMsg &m) {
  if (cm::drop_matches(g_drop_tx, m.type, ++g_tx_idx[m.type & 7])) {
    Ev(*g_ev, "migrate.drop").f("dir", "tx").f("type", m.type);
    return;
  }
  std::vector<uint8_t> frame =
      devourer::build_stream_radiotap(devourer::parse_tx_mode_str("6M"));
  const auto hdr = probe_req_hdr();
  frame.insert(frame.end(), hdr.begin(), hdr.end());
  const auto body = cm::mig_encode(m, g_key);
  frame.insert(frame.end(), body.begin(), body.end());
  std::lock_guard<std::mutex> lk(g_dev_mu);
  if (g_dev)
    g_dev->send_packet(frame.data(), frame.size());
}

/* Emit a migrate.state / migrate.* event for a MigAction of the logging kind. */
static void emit_action(const char *role, const cm::MigAction &a) {
  if (a.kind == cm::MigAction::EmitEvent) {
    Ev(*g_ev, "migrate.state")
        .t()
        .f("role", role)
        .f("code", a.code);
  } else if (a.kind == cm::MigAction::GateNotify) {
    Ev(*g_ev, "migrate.gate_notify").t().f("role", role).f("result", a.code);
  } else if (a.kind == cm::MigAction::Done) {
    Ev(*g_ev, "migrate.done").t().f("role", role).f("code", a.code);
  }
}

/* ---------------- ground ---------------- */

struct Ground {
  cm::MigProposer prop;
  cm::PeerClock clock;
  cm::MigParams params;
  cm::ChannelDef source, rescue;
  /* #279 automation gate */
  cm::MigMode mode = cm::MigMode::Advisory;
  cm::GateState gate_state;
  cm::GatePolicy gate_policy;
  cm::Decision latest_rec;   /* reconstructed from the recommend feed */
  bool have_rec = false;
  int64_t rec_age_ms = 0;    /* when the latest rec arrived (monotonic) */
  bool in_flight = false;
  int64_t inhibit_until_ms = INT64_MIN;
  bool approve_next = false;
  bool pinned = false;
  cm::ChannelDef pinned_ch;
  Ground(cm::MigParams p, uint32_t epoch, cm::ChannelDef src, cm::ChannelDef rsc)
      : prop(p, g_link, epoch, src), params(p), source(src), rescue(rsc) {}
};
static Ground *g_ground = nullptr;

static void ground_do(const std::vector<cm::MigAction> &acts) {
  for (const cm::MigAction &a : acts) {
    switch (a.kind) {
    case cm::MigAction::SendUnicast:
    case cm::MigAction::SendBroadcast: {
      cm::MigMsg m = a.msg; /* ground stamps no tx_tsf (drone owns the clock) */
      tx_mig(m);
      break;
    }
    case cm::MigAction::RetuneTo: {
      Ev(*g_ev, "migrate.retune").t().f("role", "ground")
          .f("chan", a.channel.str().c_str());
      {
        std::lock_guard<std::mutex> lk(g_dev_mu);
        if (g_dev)
          g_dev->SetMonitorChannel(a.channel.to_selected());
      }
      ground_do(g_ground->prop.on_retune_done(now_ms()));
      break;
    }
    case cm::MigAction::GateNotify:
      /* keep the autonomous-gate anti-oscillation state honest */
      if (a.code == 0)
        cm::mig_gate_on_confirmed(g_ground->gate_state,
                                  g_ground->prop.current_channel(), now_ms());
      else
        cm::mig_gate_on_rolledback(g_ground->gate_state,
                                   g_ground->prop.current_channel(),
                                   g_ground->gate_policy, now_ms());
      g_ground->in_flight = false;
      emit_action("ground", a);
      break;
    default:
      emit_action("ground", a);
      break;
    }
  }
}

/* Parse one channel.recommend / channel.hold feed line into the ground's
 * latest advisory Decision (a minimal reconstruction — target, gen, score,
 * confidence — sufficient for the gate). */
static void ground_ingest_rec(std::string_view line, int64_t now) {
  Ground &g = *g_ground;
  const bool rec = devourer::chanmig::jsonl_ev_is(line, "channel.recommend");
  const bool hold = !rec && devourer::chanmig::jsonl_ev_is(line, "channel.hold");
  if (!rec && !hold)
    return;
  cm::Decision d;
  std::string s, hexs;
  long long iv;
  double dv;
  if (devourer::chanmig::jsonl_str(line, "gen", &hexs))
    d.evidence_gen = std::strtoull(hexs.c_str(), nullptr, 16);
  else if (devourer::chanmig::jsonl_int(line, "gen", &iv))
    d.evidence_gen = static_cast<uint64_t>(iv);
  if (rec && devourer::chanmig::jsonl_str(line, "to", &s)) {
    std::string err;
    cm::ChannelDef tgt;
    if (!cm::parse_chan_token(s, tgt, err))
      return;
    d.kind = cm::Decision::Kind::Recommend;
    d.target = tgt;
    d.primary_reason = cm::Reason::RecommendBetterCandidate;
    cm::CandidateScore c;
    c.def = tgt;
    c.qualified = true;
    c.score = devourer::chanmig::jsonl_num(line, "score", &dv) ? dv : 1.0;
    c.confidence = devourer::chanmig::jsonl_num(line, "conf", &dv) ? dv : 1.0;
    d.ranking.push_back(c);
  } else {
    d.kind = cm::Decision::Kind::Hold;
  }
  g.latest_rec = d;
  g.have_rec = true;
  g.rec_age_ms = now;
}

/* Run the automation gate on a cadence; in automatic/manual mode a Propose
 * verdict starts a migration through the proposer. Emits migrate.gate. */
static void ground_gate_tick(int64_t now) {
  Ground &g = *g_ground;
  if (g.mode == cm::MigMode::Off || g.mode == cm::MigMode::Advisory)
    return;
  cm::GateInputs in;
  in.mode = g.mode;
  in.rec = g.have_rec ? &g.latest_rec : nullptr;
  in.scout_survey_age_ms = g.have_rec ? now - g.rec_age_ms : INT64_MAX;
  in.scout_max_age_ms = 5000;
  in.clock_synced = g.clock.ready();
  in.control_link_margin = 1.0; /* status/ack margin — simplified for the demo */
  in.in_flight = g.in_flight ||
                 g.prop.state() != cm::MigState::Stable;
  in.approve_next = g.approve_next;
  in.inhibit_until_ms = g.inhibit_until_ms;
  in.pinned = g.pinned ? &g.pinned_ch : nullptr;
  cm::GateOutcome o = cm::mig_gate_decide(in, g.gate_state, g.gate_policy, now);
  Ev(*g_ev, "migrate.gate").t().f("mode", cm::mig_mode_name(g.mode))
      .f("verdict", static_cast<int>(o.verdict))
      .f("reason", cm::gate_reason_name(o.reason));
  if (o.verdict == cm::GateVerdict::Propose) {
    g.approve_next = false;
    /* Shadow actuation (DEVOURER_MIG_SHADOW): the gate decides and the verdict
     * is recorded, but no proposal is actually sent — the #279 ladder's first
     * rung, quantifying false moves without touching the live link. */
    static const bool shadow = std::getenv("DEVOURER_MIG_SHADOW") != nullptr;
    if (shadow) {
      Ev(*g_ev, "migrate.shadow").t().f("to", o.target.str().c_str())
          .f("gen", (unsigned long long)o.evidence_gen);
      return;
    }
    g.in_flight = true;
    ground_do(g.prop.start(o.target, static_cast<uint32_t>(o.evidence_gen),
                           g.rescue, now));
  }
}

static void ground_rx(const Packet &pkt) {
  if (pkt.RxAtrib.pkt_rpt_type == RX_PACKET_TYPE::C2H_PACKET)
    return;
  if (pkt.Data.size() < 24 + cm::kMigHeader)
    return;
  const uint8_t *body = pkt.Data.data() + 24;
  const size_t blen = pkt.Data.size() - 24;
  cm::MigMsg m;
  if (cm::mig_decode(body, blen, g_key, g_link, m) == cm::MigReason::None) {
    if (cm::drop_matches(g_drop_rx, m.type, ++g_rx_idx[m.type & 7]))
      return;
    std::lock_guard<std::recursive_mutex> lk(g_sm_mu);
    /* feed the clock from the drone's TSF-stamped frames */
    if (m.tx_tsf != 0)
      g_ground->clock.add(m.tx_tsf, pkt.RxAtrib.tsfl);
    /* #280: the drone's target-validation result is reporting-only — the
     * ground logs it and flags a disagreement with its own scout evidence,
     * but never applies a mask/target from it (ground authority for downlink
     * quality; drone authority only to veto a locally-unsafe destination). */
    if (m.type == cm::MT_VALIDATION) {
      Ev(*g_ev, "migrate.validation").t().f("chan", m.target.str().c_str())
          .f("method", m.method).f("result", m.result)
          .f("reason", cm::mig_reason_name(static_cast<cm::MigReason>(m.reason)))
          .f("nhm_busy", m.nhm_busy_pct).f("valid", m.energy_valid);
      if (m.result == 1 && g_ground->have_rec &&
          g_ground->latest_rec.kind == cm::Decision::Kind::Recommend &&
          g_ground->latest_rec.target.same_rf(m.target))
        Ev(*g_ev, "migrate.disagree").t().f("chan", m.target.str().c_str())
            .f("ground_view", "clean").f("drone_view", "veto");
      return;
    }
    ground_do(g_ground->prop.on_message(m, now_ms()));
    return;
  }
  /* not a control frame — canonical-SA video counts toward verify */
  if (pkt.Data.size() >= 16 &&
      std::memcmp(pkt.Data.data() + 10, kCanonicalSa, 6) == 0) {
    std::lock_guard<std::recursive_mutex> lk(g_sm_mu);
    ground_do(g_ground->prop.on_video(now_ms()));
  }
}

/* ---------------- drone ---------------- */

struct Drone {
  cm::MigResponder resp;
  cm::MigParams params;
  std::atomic<bool> markers_on{false};
  cm::ChannelDef marker_ch;
  uint32_t marker_seq = 0;
  std::atomic<bool> pump_on{true};
  Drone(cm::MigParams p, uint32_t epoch, cm::ChannelDef cur, cm::MigCaps caps)
      : resp(p, g_link, epoch, cur, caps), params(p) {}
};
static Drone *g_drone = nullptr;

static uint64_t read_tsf() {
  std::lock_guard<std::mutex> lk(g_dev_mu);
  return g_dev ? g_dev->ReadTsf() : 0;
}

static void drone_do(const std::vector<cm::MigAction> &acts) {
  for (const cm::MigAction &a : acts) {
    switch (a.kind) {
    case cm::MigAction::SendUnicast:
    case cm::MigAction::SendBroadcast: {
      cm::MigMsg m = a.msg;
      m.tx_tsf = read_tsf(); /* stamp at send time */
      tx_mig(m);
      break;
    }
    case cm::MigAction::RetuneTo: {
      Ev(*g_ev, "migrate.retune").t().f("role", "drone")
          .f("chan", a.channel.str().c_str());
      bool ok = true;
      {
        std::lock_guard<std::mutex> lk(g_dev_mu);
        try {
          if (g_dev)
            g_dev->SetMonitorChannel(a.channel.to_selected());
        } catch (const std::exception &) {
          ok = false;
        }
      }
      drone_do(g_drone->resp.on_retune_done(ok, now_ms(), read_tsf()));
      break;
    }
    case cm::MigAction::StartDrain:
      g_drone->pump_on.store(false);
      drone_do(g_drone->resp.on_drain_done(now_ms()));
      break;
    case cm::MigAction::ResumePump:
      g_drone->pump_on.store(true);
      break;
    case cm::MigAction::ArmMarkers:
      g_drone->marker_ch = a.channel;
      g_drone->markers_on.store(true);
      break;
    case cm::MigAction::StopMarkers:
      g_drone->markers_on.store(false);
      break;
    case cm::MigAction::EmitEvent:
      /* the variant-B probe asks us to sample the destination now (we are on
       * the target channel after the probe retune + a settle). */
      if (a.code == 200) {
        static const int settle_ms =
            std::getenv("DEVOURER_MIG_PROBE_SETTLE_MS")
                ? std::atoi(std::getenv("DEVOURER_MIG_PROBE_SETTLE_MS"))
                : 5;
        static const int dwell_ms =
            std::getenv("DEVOURER_MIG_PROBE_DWELL_MS")
                ? std::atoi(std::getenv("DEVOURER_MIG_PROBE_DWELL_MS"))
                : 30;
        std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
        double busy = 0.0;
        bool valid = false;
        {
          std::lock_guard<std::mutex> lk(g_dev_mu);
          if (g_dev) {
            (void)g_dev->GetRxEnergy(); /* reset the delta counters */
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(dwell_ms));
        {
          std::lock_guard<std::mutex> lk(g_dev_mu);
          if (g_dev) {
            RxEnergy e = g_dev->GetRxEnergy();
            if (e.valid_nhm) {
              uint32_t total = 0;
              for (int k = 0; k < 12; k++)
                total += e.nhm[k];
              busy = total ? static_cast<double>(total - e.nhm[0]) / total : 0.0;
              valid = true;
            } else if (e.valid_fa) {
              const double sec = dwell_ms / 1000.0;
              busy = (e.cca_ofdm / (sec > 0 ? sec : 1)) / 1000.0;
              if (busy > 1.0)
                busy = 1.0;
              valid = true;
            }
          }
        }
        Ev(*g_ev, "migrate.probe").t().f("busy", busy).f("valid", valid);
        drone_do(g_drone->resp.on_probe_sample(busy, valid, now_ms()));
      } else {
        emit_action("drone", a);
      }
      break;
    default:
      emit_action("drone", a);
      break;
    }
  }
}

static void drone_rx(const Packet &pkt) {
  if (pkt.RxAtrib.pkt_rpt_type == RX_PACKET_TYPE::C2H_PACKET)
    return;
  if (pkt.Data.size() < 24 + cm::kMigHeader)
    return;
  cm::MigMsg m;
  /* The RX frame carries a trailing FCS after the message, so mig_decode reads
   * the fixed-length body and locates the MAC after it (trailer ignored). */
  if (cm::mig_decode(pkt.Data.data() + 24, pkt.Data.size() - 24, g_key, g_link,
                     m) == cm::MigReason::None) {
    if (cm::drop_matches(g_drop_rx, m.type, ++g_rx_idx[m.type & 7]))
      return;
    /* Read the TSF (a USB control transfer) OUTSIDE the SM lock so it never
     * blocks the machine behind the pump thread's send. */
    const uint64_t tsf = read_tsf();
    std::lock_guard<std::recursive_mutex> lk(g_sm_mu);
    drone_do(g_drone->resp.on_message(m, now_ms(), tsf));
  }
}

/* ---------------- shared helpers ---------------- */

static cm::ChannelDef chan_from_env(const char *name, cm::ChannelDef def) {
  const char *e = std::getenv(name);
  if (!e || !*e)
    return def;
  cm::ChannelDef d;
  std::string err;
  if (cm::parse_chan_token(e, d, err))
    return d;
  return def;
}

int main(int argc, char **argv) {
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  g_ev = &logger->events();
  install_devourer_signal_handlers();

  std::string role;
  for (int i = 1; i + 1 < argc; i++)
    if (std::strcmp(argv[i], "--role") == 0)
      role = argv[i + 1];
  if (role.empty())
    if (const char *r = std::getenv("DEVOURER_MIG_ROLE"))
      role = r;
  if (role != "ground" && role != "drone") {
    logger->error("chanmig needs --role ground|drone (or DEVOURER_MIG_ROLE)");
    return 2;
  }

  /* control key + link id */
  const char *keytext = std::getenv("DEVOURER_MIG_KEY");
  g_key = cm::MigKey::derive(keytext ? devourer::HopSchedule::parse_seed(keytext)
                                     : devourer::HopSchedule::Key{});
  g_link = 0xC1;
  if (const char *l = std::getenv("DEVOURER_MIG_LINK"))
    g_link = static_cast<uint32_t>(std::strtoul(l, nullptr, 0));
  if (const char *d = std::getenv("DEVOURER_MIG_DROP"))
    cm::parse_drop_spec(d, g_drop_tx);
  if (const char *d = std::getenv("DEVOURER_MIG_DROP_RX"))
    cm::parse_drop_spec(d, g_drop_rx);

  int channel = 36;
  if (const char *c = std::getenv("DEVOURER_CHANNEL"))
    channel = std::atoi(c);
  cm::ChannelDef source;
  source.band = channel <= 14 ? 2 : 5;
  source.primary = static_cast<uint8_t>(channel);
  source.width = CHANNEL_WIDTH_20;
  cm::normalize(source);
  cm::ChannelDef rescue = chan_from_env("DEVOURER_MIG_RESCUE", source);

  cm::MigParams params;
  if (const char *l = std::getenv("DEVOURER_MIG_LEAD_MS"))
    params.lead_ms = std::atoll(l);

  /* random boot epoch (never persisted — orphans stale commits for free) */
  uint32_t epoch;
  {
    FILE *ur = std::fopen("/dev/urandom", "rb");
    if (!ur || std::fread(&epoch, sizeof(epoch), 1, ur) != 1)
      epoch = static_cast<uint32_t>(now_ms());
    if (ur)
      std::fclose(ur);
    epoch |= 1;
  }

  /* open the adapter (duplex: RX + TX on one claimed handle) */
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0)
    return 1;
  static constexpr uint16_t kPids[] = {0x8812, 0xc812, 0xa81a, 0x881a,
                                       0xc82c, 0xe822, 0x8813};
  UsbPick pick;
  libusb_device_handle *handle = open_selected_usb(
      ctx, logger, kPids, sizeof(kPids) / sizeof(kPids[0]), &pick);
  if (!handle) {
    libusb_exit(ctx);
    return 1;
  }
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_reset_reopen(
          ctx, handle, logger, std::getenv("DEVOURER_SKIP_RESET") == nullptr,
          lock) != 0) {
    libusb_close(handle);
    libusb_exit(ctx);
    return 1;
  }
  /* Jaguar3 needs the RX filters opened at bring-up for reliable duplex. */
#ifdef _WIN32
  _putenv_s("DEVOURER_TX_WITH_RX", "thread");
#else
  ::setenv("DEVOURER_TX_WITH_RX", "thread", 1);
#endif
  WiFiDriver driver(logger);
  auto dev = driver.CreateRtlDevice(handle, ctx, lock, devourer_config_from_env());
  if (!dev) {
    logger->error("no driver for this chip");
    return 1;
  }
  g_dev = dev.get();

  Ev(*g_ev, "migrate.id").t().f("role", role.c_str())
      .f("chip", pick.pid).f("source", source.str().c_str())
      .hexf("link", g_link, 0).hexf("epoch", epoch, 8);

  if (role == "drone") {
    cm::MigCaps caps;
    if (const char *al = std::getenv("DEVOURER_MIG_ALLOWED")) {
      std::vector<cm::PlanParseError> errs;
      cm::parse_mig_allowed(al, caps.allowed, errs);
    }
    /* #280 variant B: opt-in single-radio pre-commit probe (research). Off by
     * default — variant A (legality/caps checks) is the product baseline. */
    caps.probe = std::getenv("DEVOURER_MIG_PROBE") != nullptr;
    if (const char *v = std::getenv("DEVOURER_MIG_VETO_BUSY"))
      caps.veto_busy_frac = std::atof(v);
    static Drone drone(params, epoch, source, caps);
    g_drone = &drone;
    dev->InitWrite(source.to_selected());
    /* link-derived unicast for the ACK responder */
    devourer::MacAddr am{{0x57, 0x42, 0x75, static_cast<uint8_t>(g_link >> 8),
                          static_cast<uint8_t>(g_link), 0x01}};
    dev->SetAckResponder(am);

    std::thread rx([&] { dev->StartRxLoop(drone_rx); });
    /* synthetic video pump */
    const int pps = std::getenv("DEVOURER_MIG_SYNTH_PPS")
                        ? std::atoi(std::getenv("DEVOURER_MIG_SYNTH_PPS"))
                        : 200;
    const int plen = std::getenv("DEVOURER_MIG_SYNTH_LEN")
                         ? std::atoi(std::getenv("DEVOURER_MIG_SYNTH_LEN"))
                         : 200;
    std::thread pump([&] {
      auto rt = devourer::build_stream_radiotap(devourer_tx_mode_from_env());
      auto hdr = probe_req_hdr();
      std::vector<uint8_t> f;
      const int gap_us = pps > 0 ? 1000000 / pps : 5000;
      while (!g_stop.load() && !g_devourer_should_stop) {
        if (g_drone->pump_on.load()) {
          f.clear();
          f.insert(f.end(), rt.begin(), rt.end());
          f.insert(f.end(), hdr.begin(), hdr.end());
          f.resize(f.size() + plen, 0xAB);
          std::lock_guard<std::mutex> lk(g_dev_mu);
          if (g_dev)
            g_dev->send_packet(f.data(), f.size());
        }
        std::this_thread::sleep_for(std::chrono::microseconds(gap_us));
      }
    });
    /* marker + tick loop */
    long long next_marker = 0;
    while (!g_stop.load() && !g_devourer_should_stop) {
      const long long t = now_ms();
      const uint64_t tsf = read_tsf(); /* USB control read OUTSIDE the SM lock */
      {
        std::lock_guard<std::recursive_mutex> lk(g_sm_mu);
        drone_do(drone.resp.on_tick(t, tsf));
        if (drone.markers_on.load() && t >= next_marker) {
          next_marker = t + 20;
          cm::MigMsg mk;
          mk.type = cm::MT_MARKER;
          mk.link_id = g_link;
          mk.drone_epoch = epoch;
          mk.generation = drone.resp.generation();
          mk.aired_on = drone.marker_ch;
          mk.tx_tsf = tsf;
          tx_mig(mk);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    g_stop.store(true);
    dev->StopRxLoop();
    if (pump.joinable())
      pump.join();
    if (rx.joinable())
      rx.join();
  } else {
    static Ground ground(params, epoch, source, rescue);
    g_ground = &ground;
    if (const char *m = std::getenv("DEVOURER_MIG_MODE")) {
      std::string ms(m);
      ground.mode = ms == "off"          ? cm::MigMode::Off
                    : ms == "manual"     ? cm::MigMode::Manual
                    : ms == "automatic"  ? cm::MigMode::Automatic
                                         : cm::MigMode::Advisory;
    }
    dev->InitWrite(source.to_selected());
    std::thread rx([&] { dev->StartRxLoop(ground_rx); });

    /* operator command reader (stdin) — echoed as migrate.op for audit. */
    std::thread ops([&] {
      char line[128];
      while (!g_stop.load() && std::fgets(line, sizeof(line), stdin)) {
        std::string s(line);
        while (!s.empty() && (s.back() == '\n' || s.back() == '\r'))
          s.pop_back();
        Ev(*g_ev, "migrate.op").t().f("cmd", s.c_str());
        const char *sp = std::strchr(s.c_str(), ' ');
        if (s.rfind("propose", 0) == 0) {
          cm::ChannelDef tgt;
          std::string err;
          if (sp && cm::parse_chan_token(sp + 1, tgt, err)) {
            std::lock_guard<std::recursive_mutex> lk(g_sm_mu);
            ground.in_flight = true;
            ground_do(ground.prop.start(tgt, 0, rescue, now_ms()));
          } else
            logger->warn("propose <chanspec> — bad target ({})", err);
        } else if (s.rfind("mode", 0) == 0 && sp) {
          std::string ms(sp + 1);
          ground.mode = ms == "off"         ? cm::MigMode::Off
                        : ms == "manual"    ? cm::MigMode::Manual
                        : ms == "automatic" ? cm::MigMode::Automatic
                                            : cm::MigMode::Advisory;
        } else if (s == "approve-next" || s == "approve") {
          ground.approve_next = true;
        } else if (s.rfind("inhibit", 0) == 0 && sp) {
          ground.inhibit_until_ms = now_ms() + std::atoll(sp + 1) * 1000;
        } else if (s.rfind("pin", 0) == 0) {
          cm::ChannelDef p;
          std::string err;
          if (sp && cm::parse_chan_token(sp + 1, p, err)) {
            ground.pinned = true;
            ground.pinned_ch = p;
          } else
            ground.pinned = false; /* "pin none" */
        } else if (s == "status" || s == "why") {
          Ev(*g_ev, "migrate.status").t()
              .f("state", cm::mig_state_name(ground.prop.state()))
              .f("mode", cm::mig_mode_name(ground.mode))
              .f("chan", ground.prop.current_channel().str().c_str())
              .f("clock_ready", ground.clock.ready())
              .f("resid_p99_us", (long long)ground.clock.residual_p99());
        }
      }
    });

    /* recommend feed follower (automatic/manual mode consumes it) */
    std::FILE *feed = nullptr;
    std::string feed_buf, feed_path;
    if (const char *fp = std::getenv("DEVOURER_MIG_RECOMMEND_FEED"))
      feed_path = fp;

    int64_t next_gate = 0;
    while (!g_stop.load() && !g_devourer_should_stop) {
      const int64_t now = now_ms();
      std::unique_lock<std::recursive_mutex> lk(g_sm_mu);
      ground_do(ground.prop.on_tick(now));
      if (!feed_path.empty()) {
        if (!feed)
          feed = std::fopen(feed_path.c_str(), "r");
        if (feed) {
          char chunk[2048];
          size_t n;
          while ((n = std::fread(chunk, 1, sizeof(chunk), feed)) > 0) {
            feed_buf.append(chunk, n);
            size_t nl;
            while ((nl = feed_buf.find('\n')) != std::string::npos) {
              ground_ingest_rec(std::string_view(feed_buf.data(), nl), now);
              feed_buf.erase(0, nl + 1);
            }
          }
          std::clearerr(feed);
        }
      }
      if (now >= next_gate) {
        next_gate = now + 2000;
        ground_gate_tick(now);
      }
      lk.unlock(); /* release before sleeping so the RX thread isn't starved */
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (feed)
      std::fclose(feed);
    g_stop.store(true);
    dev->StopRxLoop();
    if (ops.joinable())
      ops.detach(); /* blocked on fgets; process exit reaps it */
    if (rx.joinable())
      rx.join();
  }

  {
    std::lock_guard<std::mutex> lk(g_dev_mu);
    g_dev = nullptr;
  }
  dev->Stop();
  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(ctx);
  return 0;
}
