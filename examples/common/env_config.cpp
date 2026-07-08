#include "env_config.h"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "RadiotapBuilder.h"

namespace {

/* Flag semantics: set and not "0". (The library's historical readers were a
 * mix of presence-checks and =='1' checks; no script sets a flag to 0, so this
 * covers both.) */
bool env_flag(const char *name) {
  const char *e = std::getenv(name);
  return e != nullptr && std::strcmp(e, "0") != 0;
}

const char *env_str(const char *name) { return std::getenv(name); }

/* Case-insensitive ASCII string equality (strcasecmp is POSIX-only). */
bool str_ieq(const char *a, const char *b) {
  for (; *a && *b; ++a, ++b)
    if (std::tolower(static_cast<unsigned char>(*a)) !=
        std::tolower(static_cast<unsigned char>(*b)))
      return false;
  return *a == *b;
}

/* strtol with base auto-detect (0x-hex or decimal), matching the parse the
 * library's readers used per knob. */
bool env_long(const char *name, long *out) {
  const char *e = std::getenv(name);
  if (!e || !*e)
    return false;
  *out = std::strtol(e, nullptr, 0);
  return true;
}

} // namespace

devourer::DeviceConfig devourer_config_from_env() {
  devourer::DeviceConfig cfg;
  long v = 0;

  /* ---- rx ---- */
  cfg.rx.keep_corrupted = env_flag("DEVOURER_RX_KEEP_CORRUPTED");
  cfg.rx.enable_with_tx = env_str("DEVOURER_TX_WITH_RX") != nullptr;
  if (const char *e = env_str("DEVOURER_RX_CSI_MASK"))
    cfg.rx.csi_mask = e;
  if (const char *e = env_str("DEVOURER_RX_NBI"))
    cfg.rx.nbi = e;
  if (const char *e = env_str("DEVOURER_RX_PATHS"))
    cfg.rx.path_spec = e;
  if (env_long("DEVOURER_RX_URBS", &v))
    cfg.rx.urbs = static_cast<int>(v);
  cfg.rx.phy_status_8821c = !env_flag("DEVOURER_8821C_NO_PHYST");
  if (env_long("DEVOURER_IGI", &v))
    cfg.rx.igi = static_cast<uint8_t>(v & 0x7f);

  /* ---- tx ---- */
  if (env_long("DEVOURER_TX_EP", &v))
    cfg.tx.ep = static_cast<uint8_t>(v);
  if (env_long("DEVOURER_TX_TIMEOUT_MS", &v))
    cfg.tx.timeout_ms = static_cast<unsigned>(v);
  cfg.tx.legacy_8812_desc = env_flag("DEVOURER_TX_LEGACY_8812_DESC");
  if (env_long("DEVOURER_TX_PWR", &v))
    cfg.tx.power_index = static_cast<int>(v & 0x3f);
  if (env_long("DEVOURER_TX_RF_BW", &v))
    cfg.tx.rf_bw = static_cast<uint8_t>(v & 0x3);
  cfg.tx.cw_tone = env_flag("DEVOURER_CW_TONE");
  if (env_long("DEVOURER_CW_TONE_GAIN", &v))
    cfg.tx.cw_tone_gain = static_cast<uint8_t>(v) & 0x1F;

  /* ---- bf ---- */
  if (const char *snd = env_str("DEVOURER_BF_ARM_SOUNDER")) {
    cfg.bf.arm_sounder = true;
    /* "aa:bb:..:ff" also programs the self-MAC; a bare "1" arms only. */
    cfg.bf.sounder_self_mac = devourer::parse_mac(snd);
  }
  if (const char *e = env_str("DEVOURER_BF_ARM_BFEE"))
    cfg.bf.beamformee_of = devourer::parse_mac(e);
  cfg.bf.mu = env_flag("DEVOURER_BF_ARM_BFEE_MU");
  if (const char *e = env_str("DEVOURER_BF_TXBF"))
    cfg.bf.txbf_peer = devourer::parse_mac(e);
  if (const char *e = env_str("DEVOURER_TX_NDPA")) {
    int p = std::atoi(e);
    cfg.bf.ndpa_period = p > 0 ? p : 1;
  }

  /* ---- tuning ---- */
  cfg.tuning.skip_iqk = env_flag("DEVOURER_SKIP_IQK");
  cfg.tuning.force_iqk = env_flag("DEVOURER_FORCE_IQK");
  cfg.tuning.disable_iqk = env_flag("DEVOURER_DISABLE_IQK");
  cfg.tuning.skip_txpwr = env_flag("DEVOURER_SKIP_TXPWR");
  cfg.tuning.skip_txgapk = env_flag("DEVOURER_SKIP_TXGAPK");
  cfg.tuning.skip_trx_reassert = env_flag("DEVOURER_SKIP_TRX_REASSERT");
  cfg.tuning.skip_rfe_init = env_flag("DEVOURER_SKIP_RFEINIT");
  cfg.tuning.skip_coex = env_flag("DEVOURER_SKIP_COEX");
  cfg.tuning.skip_dig = env_flag("DEVOURER_SKIP_DIG");
  /* Default-on knob: unset = tracking on; only "0" disables it. */
  if (const char *e = env_str("DEVOURER_THERMAL_TRACK"))
    cfg.tuning.thermal_track = std::strcmp(e, "0") != 0;
  cfg.tuning.disable_cca = env_flag("DEVOURER_DIS_CCA");
  if (env_long("DEVOURER_RFE", &v))
    cfg.tuning.rfe_type = static_cast<uint8_t>(v);
  if (env_long("DEVOURER_NB_DAC", &v))
    cfg.tuning.nb_dac = static_cast<uint8_t>(v & 0x7);
  if (env_long("DEVOURER_NB_ADC", &v))
    cfg.tuning.nb_adc = static_cast<uint8_t>(v & 0x7);
  if (env_long("DEVOURER_XTAL_CAP", &v))
    cfg.tuning.xtal_cap = static_cast<uint8_t>(v & 0x7f);
  if (const char *e = env_str("DEVOURER_REGULATION")) {
    if (str_ieq(e, "ETSI"))
      cfg.tuning.regulation = devourer::Regulation::ETSI;
    else if (str_ieq(e, "MKK"))
      cfg.tuning.regulation = devourer::Regulation::MKK;
    else if (str_ieq(e, "WW"))
      cfg.tuning.regulation = devourer::Regulation::WW;
    else
      cfg.tuning.regulation = devourer::Regulation::FCC;
  }
  cfg.tuning.txpwr_by_rate = env_flag("DEVOURER_ENABLE_TXPWR_BY_RATE");
  cfg.tuning.phydm_watchdog = env_flag("DEVOURER_PHYDM_WATCHDOG");
  if (const char *e = env_str("DEVOURER_8814_FWDL");
      e && std::strcmp(e, "rtw88") == 0)
    cfg.tuning.fwdl_8814 = devourer::Fwdl8814Path::Rtw88;
  if (env_long("DEVOURER_8814_FWDL_CHUNK", &v))
    cfg.tuning.fwdl_8814_chunk = static_cast<uint32_t>(v);

  /* ---- debug ---- */
  cfg.debug.dump_canary = env_flag("DEVOURER_DUMP_CANARY");
  cfg.debug.bb_dump = env_flag("DEVOURER_BB_DUMP");
  cfg.debug.efuse_dump = env_flag("DEVOURER_EFUSE_DUMP");
  cfg.debug.log_writes = env_flag("DEVOURER_LOG_WRITES");
  cfg.debug.log_txpwr = env_flag("DEVOURER_LOG_TXPWR");
  if (const char *e = env_str("DEVOURER_REPLAY_WSEQ"))
    cfg.debug.replay_wseq = e;
  cfg.debug.hop_prof = env_flag("DEVOURER_HOP_PROF");
  cfg.debug.gaintab_dbg = env_flag("DEVOURER_GAINTAB_DBG");

  /* ---- usb ---- */
  if (const char *e = env_str("TMPDIR"); e && *e)
    cfg.usb.lock_dir = e;

  return cfg;
}

devourer::TxMode devourer_tx_mode_from_env() {
  const char *raw = std::getenv("DEVOURER_TX_RATE");
  return devourer::parse_tx_mode_str(raw ? raw : "");
}

void apply_logging_env(Logger &logger) {
  if (const char *e = std::getenv("DEVOURER_LOG_LEVEL")) {
    if (str_ieq(e, "trace"))
      logger.set_level(Logger::Level::Trace);
    else if (str_ieq(e, "debug"))
      logger.set_level(Logger::Level::Debug);
    else if (str_ieq(e, "info"))
      logger.set_level(Logger::Level::Info);
    else if (str_ieq(e, "warn"))
      logger.set_level(Logger::Level::Warn);
    else if (str_ieq(e, "error"))
      logger.set_level(Logger::Level::Error);
    else if (str_ieq(e, "silent"))
      logger.set_level(Logger::Level::Silent);
    else
      std::fprintf(stderr, "devourer [W] DEVOURER_LOG_LEVEL='%s' unknown — "
                           "keeping default\n", e);
  }

  const auto flush = env_flag("DEVOURER_EVENT_FLUSH") ||
                             std::getenv("DEVOURER_EVENT_FLUSH") == nullptr
                         ? devourer::EventSink::FlushPolicy::EveryLine
                         : devourer::EventSink::FlushPolicy::Never;
  if (const char *e = std::getenv("DEVOURER_EVENTS")) {
    if (str_ieq(e, "off"))
      logger.events().disable();
    else if (str_ieq(e, "stderr"))
      logger.events().configure(stderr, flush);
    else
      logger.events().configure(stdout, flush);
  } else {
    logger.events().configure(stdout, flush);
  }
}
