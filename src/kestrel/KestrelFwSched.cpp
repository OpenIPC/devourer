/* KestrelFwSched — the 802.11ax trigger-based UL + TWT H2C senders, a second
 * translation unit of the KestrelFw class (declared in KestrelFw.h). Kept
 * apart from KestrelFw.cpp (the FWDL/bring-up plane) so each file stays
 * focused; both share the private send_h2c_cmd + _variant.
 *
 * The byte encoding lives in SchedEncode.h (pure, device-free, unit-tested by
 * tests/kestrel_sched_selftest.cpp); each method here is encode_* +
 * send_h2c_cmd + a log line. */

#include "KestrelFw.h"

#include <vector>

#include "MacRegAx.h"
#include "SchedEncode.h"

namespace kestrel {

namespace {
namespace r = kestrel::reg;
} /* namespace */

bool KestrelFw::f2p_trigger(const devourer::TriggerConfig &cfg) {
  const bool v1 = (_variant == ChipVariant::C8852C);
  std::vector<uint8_t> c = sched::encode_f2p_trigger(cfg, v1);
  uint8_t n = cfg.n_users == 0 ? 1 : cfg.n_users;
  const uint8_t max_users = v1 ? r::F2P_V1_MAX_USERS : r::F2P_V0_MAX_USERS;
  if (n > max_users)
    n = max_users;
  bool ok = send_h2c_cmd(r::FWCMD_H2C_CAT_MAC, r::FWCMD_H2C_CL_FR_EXCHG,
                         r::FWCMD_H2C_FUNC_F2P_TEST, c.data(),
                         static_cast<uint32_t>(c.size()));
  _logger->info("Kestrel: f2p_trigger ({} v{} users={} bw={} rate=0x{:x} mode={}"
                " frexch={}) -> {}",
                v1 ? "8852C" : "8852B", v1 ? 1 : 0, n, cfg.ul_bw, cfg.trig_rate,
                cfg.mode, cfg.frexch_type, ok ? "sent" : "FAILED");
  return ok;
}

bool KestrelFw::twt_info_upd(const devourer::TwtConfig &cfg, uint8_t act) {
  std::vector<uint8_t> c = sched::encode_twtinfo(cfg, act);
  bool ok = send_h2c_cmd(r::FWCMD_H2C_CAT_MAC, r::FWCMD_H2C_CL_TWT,
                         r::FWCMD_H2C_FUNC_TWTINFO_UPD, c.data(),
                         static_cast<uint32_t>(c.size()));
  _logger->info("Kestrel: twt_info_upd (act={} {} id={} flow={} trig={} "
                "wake_exp={} man={} tsf=0x{:x}) -> {}",
                act, cfg.broadcast ? "bcast" : "indiv", cfg.config_id,
                cfg.flow_id, cfg.trigger, cfg.wake_exp, cfg.wake_man,
                cfg.trgt_tsf, ok ? "sent" : "FAILED");
  return ok;
}

bool KestrelFw::twt_act(const devourer::TwtStaAct &a) {
  std::vector<uint8_t> c = sched::encode_twt_act(a);
  bool ok = send_h2c_cmd(r::FWCMD_H2C_CAT_MAC, r::FWCMD_H2C_CL_TWT,
                         r::FWCMD_H2C_FUNC_TWT_STANSP_UPD, c.data(),
                         static_cast<uint32_t>(c.size()));
  _logger->info("Kestrel: twt_act (macid={} id={} act={}) -> {}", a.macid,
                a.config_id, a.action, ok ? "sent" : "FAILED");
  return ok;
}

bool KestrelFw::twt_announce(uint8_t macid) {
  std::vector<uint8_t> c = sched::encode_twt_announce(macid);
  bool ok = send_h2c_cmd(r::FWCMD_H2C_CAT_MAC, r::FWCMD_H2C_CL_TWT,
                         r::FWCMD_H2C_FUNC_TWT_ANNOUNCE_UPD, c.data(),
                         static_cast<uint32_t>(c.size()));
  _logger->info("Kestrel: twt_announce (macid={}) -> {}", macid,
                ok ? "sent" : "FAILED");
  return ok;
}

bool KestrelFw::twt_ofdma_info_upd(const devourer::TwtOfdmaConfig &cfg) {
  std::vector<uint8_t> c = sched::encode_twt_ofdma(cfg);
  bool ok = send_h2c_cmd(r::FWCMD_H2C_CAT_MAC, r::FWCMD_H2C_CL_TWT,
                         r::FWCMD_H2C_FUNC_TWT_OFDMA_INFO_UPD, c.data(),
                         static_cast<uint32_t>(c.size()));
  _logger->info("Kestrel: twt_ofdma_info_upd (twt_id={} rounds={} interval={}us "
                "tf_retry={}) -> {} [func 0x03 non-canonical: fw may ignore]",
                cfg.twt_id, cfg.round_num, cfg.round_interval_us,
                cfg.max_tf_retry, ok ? "sent" : "FAILED");
  return ok;
}

bool KestrelFw::ul_fixinfo(const devourer::UlOfdmaConfig &cfg) {
  std::vector<uint8_t> c = sched::encode_ul_fixinfo(cfg);
  uint8_t nsta = cfg.n_stas == 0 ? 1 : cfg.n_stas;
  if (nsta > r::UL_FIXINFO_MAX_RU_NUM)
    nsta = r::UL_FIXINFO_MAX_RU_NUM;
  bool ok = send_h2c_cmd(r::FWCMD_H2C_CAT_MAC, r::FWCMD_H2C_CL_FR_EXCHG,
                         r::FWCMD_H2C_FUNC_TBLUD, c.data(),
                         static_cast<uint32_t>(c.size()));
  _logger->info("Kestrel: ul_fixinfo (mode={} interval={}s stas={} bw={} "
                "tf_type={}) -> {}",
                cfg.mode, cfg.interval_s, nsta, cfg.ppdu_bw, cfg.tf_type,
                ok ? "sent" : "FAILED");
  return ok;
}

} /* namespace kestrel */
