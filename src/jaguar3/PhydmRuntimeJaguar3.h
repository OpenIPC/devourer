#ifndef DEVOURER_JAGUAR3_PHYDM_RUNTIME_H
#define DEVOURER_JAGUAR3_PHYDM_RUNTIME_H

#include <cstdint>

#include "logger.h"
#include "RtlAdapter.h"

namespace jaguar3 {

/* 8822C IGI floor at 40/80 MHz. A wide-channel TX needs the secondary 20 MHz
 * to read idle, and below IGI 0x24 the 8822C receiver parks, on a QUIET
 * channel, in a gain state where the secondary never does: the MAC holds
 * every 40 MHz frame (measured on 144/52/100/120 — the first frame never
 * airs and the TX FIFO fills; 0x24 clears it, 0x22 does not; busy channels
 * self-heal because each received frame re-runs the AGC). One step of margin
 * on top. Costs ~6 dB of idle-channel RX sensitivity against DIG_MIN_DFS at
 * those widths only, and only once the device has transmitted a wide frame
 * (RtlJaguar3Device::_wide_tx_armed): a 40 MHz receiver-only session or a
 * ground station sending 20 MHz control frames keeps the normal floor. 20 MHz
 * and the 8822E are untouched. 80 MHz inherits the floor untested. */
constexpr uint8_t DIG_MIN_8822C_WIDE = 0x26;

/* Port of the phydm dynamic mechanisms the vendor kernel runs from its ~2 s
 * watchdog in a no-link (monitor) session — the RX-side adaptation loop:
 *
 *   phydm_false_alarm_counter_statistics (phydm_fa_cnt_statistics_jgr3)
 *   phydm_dig                            (unlinked algorithm, IGI in 0x1d70)
 *   phydm_cck_pd_th                      (type4, 2.4 GHz CCK packet detection)
 *   phydm_adaptivity                     (EDCCA thresholds in 0x84c)
 *
 * The other watchdog mechanisms self-disable without a link and are not
 * ported: CFO tracking returns unless is_linked && one entry, RA and the
 * beamforming watchdog have no station entries, antenna diversity is off on
 * the 2T2R parts.
 *
 * tick() is called from the coex/housekeeping thread every ~2 s under the
 * device register lock. All state lives here; nothing is read from the
 * environment. */
class PhydmRuntimeJaguar3 {
public:
  PhydmRuntimeJaguar3(RtlAdapter device, Logger_t logger)
      : _device{device}, _logger{logger} {}

  /* One watchdog pass: FA/CCA statistics + reset, DIG, CCK-PD (2 GHz),
   * EDCCA. `channel` = current center channel (band + DFS decision),
   * `edcca_track` = false leaves 0x84c untouched (DEVOURER_DIS_CCA /
   * SetCcaMode owns the thresholds then). */
  void tick(uint8_t channel, bool edcca_track);

  /* A channel/bandwidth change invalidates the CCK-PD applied-level cache
   * (the kernel re-checks bw/nrx from registers each apply; the level is
   * re-applied when they change). */
  void on_channel_change() { _cck_lv_valid = false; }

  /* Lower bound for DIG: applied at once (IGI raised to it if below) and
   * honoured by every later tick. 0 = none. See DIG_MIN_8822C_WIDE. */
  void set_igi_floor(uint8_t floor);

private:
  /* --- false-alarm / CCA statistics (this tick's window) --- */
  struct FaStats {
    uint32_t fa_all = 0;
    uint32_t cca_all = 0;
    uint32_t cck_fa = 0;
    bool cck_enabled = false;
  };
  FaStats fa_statistics_and_reset();

  void dig(const FaStats &fa, bool is_dfs);
  void cck_pd(const FaStats &fa);
  void edcca(uint8_t igi);

  uint8_t get_igi();
  void set_igi(uint8_t igi);

  RtlAdapter _device;
  Logger_t _logger;

  /* DIG state */
  int _last_igi_logged = -1;
  uint8_t _igi_floor = 0;

  /* CCK-PD state (phydm_cckpd_type4) */
  bool _cck_tbl_valid = false;
  uint8_t _cckpd_tbl[2][2][2][5] = {}; /* [bw20/40][1r/2r][pd/cs][lv0..4] */
  uint32_t _cck_fa_ma = 0xffffffff;    /* CCK_FA_MA_RESET sentinel */
  bool _cck_lv_valid = false;
  uint8_t _cck_lv = 0, _cck_bw = 0, _cck_nrx = 0;

  /* EDCCA state (log-on-change) */
  int _last_l2h_logged = 0x7fff;
};

} // namespace jaguar3

#endif /* DEVOURER_JAGUAR3_PHYDM_RUNTIME_H */
