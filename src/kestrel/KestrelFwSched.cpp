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

bool KestrelFw::set_snd_para(const devourer::SoundingConfig &cfg) {
  const bool v1 = (_variant == ChipVariant::C8852C);
  std::vector<uint8_t> c = sched::encode_set_snd_para(cfg, v1);
  const uint8_t func =
      v1 ? r::FWCMD_H2C_FUNC_SET_SND_PARA_V1 : r::FWCMD_H2C_FUNC_SET_SND_PARA;
  bool ok = send_h2c_cmd(r::FWCMD_H2C_CAT_MAC, r::FWCMD_H2C_CL_SOUND, func,
                         c.data(), static_cast<uint32_t>(c.size()));
  _logger->info("Kestrel: set_snd_para ({} v{} frexch={} bfrp0_users={} "
                "sta0_aid={} macid0={}) -> {}",
                v1 ? "8852C" : "8852B", v1 ? 1 : 0, cfg.frexgtype,
                cfg.bfrp0_user_num, cfg.he_sta[0].aid, cfg.macid[0],
                ok ? "sent" : "FAILED");
  return ok;
}

bool KestrelFw::fw_upd_cctl_bf(uint8_t macid, uint8_t addr_cam_idx,
                               const devourer::StaBfCaps &bf) {
  /* mac_set_csi_para_cctl: the CSI/bf fields live in CCTL value dword8 (byte
   * 32) with the matching mask in dword16 (byte 64). We also re-assert
   * addr_cam_index (value dword7 / mask dword15) so the entry resolves. */
  uint8_t c[68] = {0};
  auto sw = [](uint32_t v, uint32_t msk, uint32_t sh) {
    return (v & msk) << sh;
  };
  put_le32(c + 0, (macid & r::FWCMD_H2C_CCTLINFO_UD_MACID_MSK) |
                      r::FWCMD_H2C_CCTLINFO_UD_OP);
  /* value dword7: addr_cam_index. */
  put_le32(c + 28, sw(addr_cam_idx, r::FWCMD_H2C_CCTRL_ADDR_CAM_INDEX_MSK,
                      r::FWCMD_H2C_CCTRL_ADDR_CAM_INDEX_SH));
  /* value dword8: nc/nr/ng/cb/cs + csi_para_en=1 (csi_txbf_en stays 0). */
  const uint32_t dw8 =
      sw(bf.nc, r::FWCMD_H2C_CCTRL_NC_MSK, r::FWCMD_H2C_CCTRL_NC_SH) |
      sw(bf.nr, r::FWCMD_H2C_CCTRL_NR_MSK, r::FWCMD_H2C_CCTRL_NR_SH) |
      sw(bf.ng, r::FWCMD_H2C_CCTRL_NG_MSK, r::FWCMD_H2C_CCTRL_NG_SH) |
      sw(bf.cb, r::FWCMD_H2C_CCTRL_CB_MSK, r::FWCMD_H2C_CCTRL_CB_SH) |
      sw(bf.cs, r::FWCMD_H2C_CCTRL_CS_MSK, r::FWCMD_H2C_CCTRL_CS_SH) |
      r::FWCMD_H2C_CCTRL_CSI_PARA_EN |
      sw(bf.csi_fix_rate, r::FWCMD_H2C_CCTRL_CSI_FIX_RATE_MSK,
         r::FWCMD_H2C_CCTRL_CSI_FIX_RATE_SH) |
      sw(bf.csi_gi_ltf, r::FWCMD_H2C_CCTRL_CSI_GI_LTF_MSK,
         r::FWCMD_H2C_CCTRL_CSI_GI_LTF_SH) |
      sw(bf.csi_bw, r::FWCMD_H2C_CCTRL_CSI_BW_MSK, r::FWCMD_H2C_CCTRL_CSI_BW_SH);
  put_le32(c + 32, dw8);
  /* mask dword15 <- dword7 addr_cam_index; mask dword16 <- dword8 fields. */
  put_le32(c + 60, sw(r::FWCMD_H2C_CCTRL_ADDR_CAM_INDEX_MSK,
                      r::FWCMD_H2C_CCTRL_ADDR_CAM_INDEX_MSK,
                      r::FWCMD_H2C_CCTRL_ADDR_CAM_INDEX_SH));
  put_le32(c + 64,
           sw(r::FWCMD_H2C_CCTRL_NC_MSK, r::FWCMD_H2C_CCTRL_NC_MSK,
              r::FWCMD_H2C_CCTRL_NC_SH) |
               sw(r::FWCMD_H2C_CCTRL_NR_MSK, r::FWCMD_H2C_CCTRL_NR_MSK,
                  r::FWCMD_H2C_CCTRL_NR_SH) |
               sw(r::FWCMD_H2C_CCTRL_NG_MSK, r::FWCMD_H2C_CCTRL_NG_MSK,
                  r::FWCMD_H2C_CCTRL_NG_SH) |
               sw(r::FWCMD_H2C_CCTRL_CB_MSK, r::FWCMD_H2C_CCTRL_CB_MSK,
                  r::FWCMD_H2C_CCTRL_CB_SH) |
               sw(r::FWCMD_H2C_CCTRL_CS_MSK, r::FWCMD_H2C_CCTRL_CS_MSK,
                  r::FWCMD_H2C_CCTRL_CS_SH) |
               r::FWCMD_H2C_CCTRL_CSI_PARA_EN |
               sw(r::FWCMD_H2C_CCTRL_CSI_FIX_RATE_MSK,
                  r::FWCMD_H2C_CCTRL_CSI_FIX_RATE_MSK,
                  r::FWCMD_H2C_CCTRL_CSI_FIX_RATE_SH) |
               sw(r::FWCMD_H2C_CCTRL_CSI_GI_LTF_MSK,
                  r::FWCMD_H2C_CCTRL_CSI_GI_LTF_MSK,
                  r::FWCMD_H2C_CCTRL_CSI_GI_LTF_SH) |
               sw(r::FWCMD_H2C_CCTRL_CSI_BW_MSK, r::FWCMD_H2C_CCTRL_CSI_BW_MSK,
                  r::FWCMD_H2C_CCTRL_CSI_BW_SH));
  bool ok = send_h2c_cmd(r::FWCMD_H2C_CAT_MAC, r::FWCMD_H2C_CL_FR_EXCHG,
                         r::FWCMD_H2C_FUNC_CCTLINFO_UD, c, sizeof(c));
  _logger->info("Kestrel: upd_cctl_bf (macid={} a_idx={} nc={} nr={} ng={} "
                "cb={} bw={}) -> {}",
                macid, addr_cam_idx, bf.nc, bf.nr, bf.ng, bf.cb, bf.csi_bw,
                ok ? "sent" : "FAILED");
  return ok;
}

} /* namespace kestrel */
