/* kestrelprobe — staged bring-up driver for the Kestrel (Wi-Fi 6 / 802.11ax,
 * RTL8852BU / RTL8852CU), the USB sibling of pcieprobe. Validates the layers
 * one at a time, bottom-up:
 *
 *   id     USB open + PID-table variant + AX-native identity: the die-id
 *          at R_AX_SYS_CHIPINFO (0x00FC) must match the PID-selected
 *          variant (0x51 = 8852B, 0x52 = 8852C); cut version from
 *          R_AX_SYS_CFG1[15:12].
 *   power  + mac_ax power-on sequence + efuse logical map.
 *   fw     + firmware download (cut-selected image) + fw-ready poll.
 *   trx    + MAC TRX init (DMAC half).
 *   phy    + BB/RF bring-up (tables, gain, RX path).
 *
 * Usage: sudo kestrelprobe [id|power|fw|trx|phy] [--vid 0xNNNN] [--pid 0xNNNN]
 * Default stage: id. Without --vid/--pid the open loop scans the Kestrel PID
 * table (kestrel/KestrelUsbIds.h). Two cold-plug traps: the TX20U Nano first
 * enumerates as a ZeroCD disk (0bda:1a2b) until usb_modeswitch flips it to
 * 35bc:0108, and the in-kernel rtw89_8852bu module auto-probes the NIC at
 * every enumeration — temp-blacklist it (modprobe -r does not survive a
 * re-enumeration; see docs/adapter-doctor.md).
 *
 * Events (stdout JSONL): kestrel.<stage> with ok:true|false — exit code 0
 * only if the requested stage passed. */
#if defined(__ANDROID__) || defined(_MSC_VER) || defined(__APPLE__)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "Event.h"
#include "RtlAdapter.h"
#include "SignalStop.h" /* g_devourer_should_stop — SIGINT/SIGTERM flag */
#include "UsbOpen.h"
#include "logger.h"

#include "kestrel/ChipVariant.h"
#include "kestrel/KestrelUsbIds.h"
#include "kestrel/RtlKestrelDevice.h"

namespace {

const char *variant_name(kestrel::ChipVariant v) {
  return v == kestrel::ChipVariant::C8852B ? "8852B" : "8852C";
}

bool parse_hex16(const char *s, uint16_t &out) {
  char *end = nullptr;
  long v = std::strtol(s, &end, 0);
  if (end == s || *end != '\0' || v < 0 || v > 0xFFFF)
    return false;
  out = static_cast<uint16_t>(v);
  return true;
}

} /* namespace */

int main(int argc, char **argv) {
  std::string stage = "id";
  uint16_t want_vid = 0, want_pid = 0; /* 0 = scan the Kestrel PID table */
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    if (k == "id" || k == "power" || k == "fw" || k == "trx" || k == "phy" ||
        k == "trig" || k == "twt")
      stage = k;
    else if (k == "--vid" && i + 1 < argc && parse_hex16(argv[++i], want_vid))
      ;
    else if (k == "--pid" && i + 1 < argc && parse_hex16(argv[++i], want_pid))
      ;
    else {
      std::fprintf(stderr,
                   "usage: %s [id|power|fw|trx|phy|trig|twt] "
                   "[--vid 0xNNNN] [--pid 0xNNNN]\n",
                   argv[0]);
      return 2;
    }
  }
  /* trig/twt (5/6) go past phy to a full TX bring-up (InitWrite) + fire the
   * scheduled-UL command; a 5 GHz channel where trigger-UL matters. */
  const int want = stage == "twt" ? 6 : stage == "trig" ? 5
                                    : stage == "phy" ? 4
                                    : stage == "trx" ? 3
                                    : stage == "fw" ? 2
                                    : stage == "power" ? 1
                                                       : 0;
  int chan = 36;
  if (const char *e = std::getenv("DEVOURER_CHANNEL"))
    chan = std::atoi(e);

  auto logger = std::make_shared<Logger>();

  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) {
    logger->error("libusb_init failed");
    return 1;
  }

  libusb_device_handle *handle = nullptr;
  kestrel::ChipVariant variant = kestrel::ChipVariant::C8852B;
  uint16_t vid = 0, pid = 0;
  if (want_vid && want_pid) {
    auto v = kestrel::variant_for_usb_id(want_vid, want_pid);
    if (!v) {
      logger->error("{:04x}:{:04x} is not in the Kestrel PID table "
                    "(kestrel/KestrelUsbIds.h)",
                    want_vid, want_pid);
      libusb_exit(ctx);
      return 2;
    }
    handle = libusb_open_device_with_vid_pid(ctx, want_vid, want_pid);
    variant = *v;
    vid = want_vid;
    pid = want_pid;
  } else {
    for (const auto &kid : kestrel::kKestrelUsbIds) {
      handle = libusb_open_device_with_vid_pid(ctx, kid.vid, kid.pid);
      if (handle) {
        variant = kid.variant;
        vid = kid.vid;
        pid = kid.pid;
        break;
      }
    }
  }
  if (!handle) {
    logger->error("no Kestrel adapter found (scanned the KestrelUsbIds "
                  "table). If lsusb shows 0bda:1a2b the dongle is still in "
                  "ZeroCD disk mode — usb_modeswitch it first; if a rtw89_* "
                  "module holds it, temp-blacklist (not modprobe -r).");
    devourer::Ev(logger->events(), "kestrel.id")
        .f("ok", false)
        .f("why", "open");
    libusb_exit(ctx);
    return 1;
  }

  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(handle, 0, logger, true, lock) !=
      0) {
    libusb_close(handle);
    libusb_exit(ctx);
    return 1;
  }

  /* ---- stage id: register plane only, no power, no DMA ---- */
  RtlKestrelDevice dev(RtlAdapter(handle, logger, ctx, lock, {}), logger,
                       variant);
  const kestrel::ChipInfo info = dev.ReadChipInfo();
  const bool id_ok = info.matches(variant);
  logger->info("id: {:04x}:{:04x} -> {} | die-id=0x{:02x} (want 0x{:02x}) "
               "cut={}",
               vid, pid, variant_name(variant), info.die_id,
               variant == kestrel::ChipVariant::C8852B ? 0x51 : 0x52,
               info.cut);
  devourer::Ev(logger->events(), "kestrel.id")
      .f("ok", id_ok)
      .f("variant", variant_name(variant))
      .hexf("vid", vid, 4)
      .hexf("pid", pid, 4)
      .hexf("die_id", info.die_id, 2)
      .f("cut", info.cut);
  if (!id_ok || want < 1) {
    libusb_close(handle);
    libusb_exit(ctx);
    return id_ok ? 0 : 1;
  }

  /* ---- stage power: mac_ax power-on + efuse dump ---- */
  kestrel::EfuseInfo efuse;
  bool power_ok = false;
  try {
    power_ok = dev.PowerOnAndReadEfuse(efuse);
  } catch (const std::exception &e) {
    logger->error("power: power/efuse threw: {}", e.what());
  }
  char mac[18] = "??:??:??:??:??:??";
  if (power_ok)
    snprintf(mac, sizeof(mac), "%02x:%02x:%02x:%02x:%02x:%02x", efuse.mac[0],
             efuse.mac[1], efuse.mac[2], efuse.mac[3], efuse.mac[4],
             efuse.mac[5]);
  logger->info("power: power_ok={} MAC={} xtal=0x{:02x} rfe=0x{:02x} "
               "thermalA=0x{:02x} thermalB=0x{:02x} autoload={}",
               power_ok, mac, efuse.xtal_cap, efuse.rfe_type, efuse.thermal_a,
               efuse.thermal_b, efuse.autoload_ok);
  devourer::Ev(logger->events(), "kestrel.power")
      .f("ok", power_ok)
      .f("mac", mac)
      .hexf("xtal", efuse.xtal_cap, 2)
      .hexf("rfe", efuse.rfe_type, 2)
      .f("autoload", efuse.autoload_ok);
  if (!power_ok || want < 2) {
    libusb_close(handle);
    libusb_exit(ctx);
    return power_ok ? 0 : 1;
  }

  /* ---- stage fw: firmware download ---- */
  bool fw_ok = false;
  try {
    /* Re-run the whole sequence so the stage is self-contained (power-on is
     * cheap and idempotent from a clean handle). trx re-runs fw internally.
     * trig/twt (>=5) take the full TX bring-up (InitWrite = phy + scheduler +
     * self-STA role) so the fw can accept the scheduled-UL H2Cs. */
    if (want >= 5) {
      dev.InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(chan),
                                    .ChannelOffset = 0,
                                    .ChannelWidth = CHANNEL_WIDTH_20});
      fw_ok = true;
    } else {
      fw_ok = (want >= 4)   ? dev.PowerOnTrxAndPhy(efuse)
              : (want >= 3) ? dev.PowerOnFwAndTrx(efuse)
                            : dev.PowerOnEfuseAndFw(efuse);
    }
  } catch (const std::exception &e) {
    logger->error("fw/trx/phy: bring-up threw: {}", e.what());
  }
  if (want < 3) {
    logger->info("fw: fw_ok={}", fw_ok);
    devourer::Ev(logger->events(), "kestrel.fw").f("ok", fw_ok);
    libusb_close(handle);
    libusb_exit(ctx);
    return fw_ok ? 0 : 1;
  }
  if (want < 4) {
    /* ---- stage trx: DMAC + CMAC MAC TRX init ---- */
    logger->info("trx: trx_ok={}", fw_ok);
    devourer::Ev(logger->events(), "kestrel.trx").f("ok", fw_ok);
    libusb_close(handle);
    libusb_exit(ctx);
    return fw_ok ? 0 : 1;
  }

  if (want < 5) {
    /* ---- stage phy: BB + RF table apply ---- */
    logger->info("phy: phy_ok={}", fw_ok);
    devourer::Ev(logger->events(), "kestrel.phy").f("ok", fw_ok);
    libusb_close(handle);
    libusb_exit(ctx);
    return fw_ok ? 0 : 1;
  }

  /* ---- stage trig: air a Basic Trigger (UL-OFDMA grant) via the F2P command.
   * Self-contained fw probe — no associated STA needed; a HE monitor (or an
   * 11ac witness on the legacy PPDU) decodes the aired trigger as rx.trigger.
   * Fires a short burst so an SDR reads a duty cycle. ---- */
  if (want == 5) {
    int count = 200;
    if (const char *e = std::getenv("DEVOURER_TRIGGER_COUNT"))
      count = std::atoi(e);
    /* Register the self MACID as an AP role (not the CLIENT role InitWrite set)
     * so the fw runs its AP-side scheduler — the beacon port NET_TYPE is AP but
     * the fw role must be AP too for the trigger transmitter to engage
     * (role.c). SMA=TMA=BSSID for an AP self_role. */
    const uint8_t ap_bssid[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x00};
    bool aprole = dev.hal().register_ap_role(ap_bssid);
    logger->info("trig: register_ap_role -> {}", aprole);

    devourer::TriggerConfig cfg;
    cfg.ul_bw = 0;      /* 20 MHz */
    cfg.n_users = 1;
    cfg.users[0].aid12 = 1;
    cfg.users[0].macid = 0;
    cfg.users[0].ru_alloc = devourer::he_ru_alloc(242, 0); /* full 20 MHz */
    cfg.users[0].ul_mcs = 0;
    cfg.users[0].ss = 1;
    cfg.users[0].tgt_rssi_dbm = -60;
    /* The trigger frame's OWN PPDU rate (tf_wd.datarate). Must be an OFDM rate
     * on 5 GHz — rate 0 is CCK 1 Mbps, which does not exist above ch14, so the
     * fw cannot air the trigger there (same CCK-on-5GHz trap as the AP rate
     * set). OFDM 6M (AX rate code 4) airs on both bands. */
    cfg.trig_rate = chan > 14 ? 4 : 0;
    if (const char *e = std::getenv("DEVOURER_TRIG_RATE"))
      cfg.trig_rate = static_cast<uint16_t>(std::atoi(e));
    /* mode/frexch_type are the runtime-discovered fw fields; DEVOURER_TRIG_MODE
     * sweeps the 2-bit mode to find the one that airs a Basic Trigger. */
    if (const char *e = std::getenv("DEVOURER_TRIG_MODE"))
      cfg.mode = static_cast<uint8_t>(std::atoi(e) & 0x3);

    /* DEVOURER_UL_FIXINFO=1 programs the production UL-OFDMA scheduler table
     * (mode=tf_periodic) instead of the F2P test command — the fw then airs
     * Triggers autonomously at `interval` for the granted macid. This is the
     * canonical path (F2P_TEST has no vendor production caller). */
    if (std::getenv("DEVOURER_UL_FIXINFO")) {
      /* DEVOURER_UL_PEER=1 registers a distinct peer STA (macid 1) first, so the
       * scheduler grants to a peer rather than the self-STA — a cheap probe of
       * whether the UL-OFDMA engine airs a trigger for a registered (but not
       * associated) peer, short of the full tier-C association. */
      uint8_t grant_macid = 0;
      if (std::getenv("DEVOURER_UL_PEER")) {
        const uint8_t peer_mac[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
        const bool pok = dev.RegisterPeerSta(peer_mac, /*macid=*/1,
                                             /*addr_cam_idx=*/1);
        logger->info("trig: register_peer_sta(macid=1) -> {}", pok);
        grant_macid = 1;
      }
      devourer::UlOfdmaConfig ul;
      ul.mode = 3;        /* tf_periodic */
      ul.interval_s = 1;
      ul.tf_type = 0;     /* Basic */
      ul.ppdu_bw = 0;
      ul.n_stas = 1;
      ul.stas[0].macid = grant_macid;
      ul.stas[0].ru_pos = 61; /* full 20 MHz */
      ul.stas[0].mcs = 0;
      ul.stas[0].ss = 1;
      ul.stas[0].tgt_rssi_dbm = -60;
      const bool ulok = dev.ConfigureUlOfdma(ul);
      logger->info("trig: ul_fixinfo(tf_periodic) -> {}", ulok);
      /* Hold so the fw's periodic engine airs several rounds for the witness. */
      for (int i = 0; i < 40 && !g_devourer_should_stop; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      devourer::Ev(logger->events(), "kestrel.trig")
          .f("ok", ulok)
          .f("mode", "ul_fixinfo")
          .f("chan", chan);
      dev.Stop();
      libusb_close(handle);
      libusb_exit(ctx);
      return ulok ? 0 : 1;
    }

    int sent = 0;
    for (int i = 0; i < count && !g_devourer_should_stop; ++i) {
      if (dev.SendTrigger(cfg))
        ++sent;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const bool ok = sent > 0;
    logger->info("trig: sent {}/{} triggers (mode={})", sent, count, cfg.mode);
    devourer::Ev(logger->events(), "kestrel.trig")
        .f("ok", ok)
        .f("sent", sent)
        .f("count", count)
        .f("mode", cfg.mode)
        .f("chan", chan);
    /* InitWrite started the WP-drain thread — join it before libusb_exit or a
     * thread still inside libusb trips usbi_mutex_destroy (SIGABRT). */
    dev.Stop();
    libusb_close(handle);
    libusb_exit(ctx);
    return ok ? 0 : 1;
  }

  /* ---- stage twt: create an individual TWT agreement + (attempt) TWT-OFDMA
   * cadence, and drain any TWT C2H the fw emits. Discovery: whether the shipped
   * fw honors the (non-canonical) TWT-OFDMA func 0x03. ---- */
  devourer::TwtConfig twt;
  twt.broadcast = false;
  twt.trigger = true;
  twt.config_id = 0;
  twt.flow_id = 0;
  twt.ap_role = true;
  twt.wake_exp = 10;
  twt.wake_man = 512;
  twt.trgt_tsf = dev.ReadTsf() + 100000; /* 100 ms out */
  const bool twt_ok = dev.ConfigureTwt(twt);
  devourer::TwtOfdmaConfig ofd;
  ofd.twt_id = 0;
  ofd.round_num = 4;
  ofd.round_interval_us = 2000;
  ofd.max_tf_retry = 3;
  const bool ofdma_ok = dev.ConfigureTwtOfdma(ofd);
  /* Drain the bulk-IN briefly so any TWT_NOTIFY / WAIT_ANNOUNCE C2H is routed
   * (handle_c2h logs twt.notify / twt.wait_anno on the diagnostic plane). */
  logger->info("twt: configure={} ofdma_cmd={} (watch for twt.notify C2H)",
               twt_ok, ofdma_ok);
  devourer::Ev(logger->events(), "kestrel.twt")
      .f("ok", twt_ok)
      .f("ofdma_cmd", ofdma_ok)
      .f("chan", chan);
  dev.Stop(); /* join the WP-drain thread before libusb teardown (see trig) */
  libusb_close(handle);
  libusb_exit(ctx);
  return twt_ok ? 0 : 1;
}
