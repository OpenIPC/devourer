#ifndef PHYDM_WATCHDOG_H
#define PHYDM_WATCHDOG_H

#include "EepromManager.h"
#include "RtlAdapter.h"
#include "logger.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

class RadioManagementModule;

/* Periodic phydm DM watchdog — runs every ~2s to drive dynamic
 * management modules (FA counter statistics, DIG, RSSI tracking,
 * etc.) the way upstream's `phydm_watchdog` does.
 *
 * Upstream `phydm_watchdog` (hal/phydm/phydm.c:1985) chains together:
 *   phydm_phy_info_update, phydm_rssi_monitor_check,
 *   phydm_false_alarm_counter_statistics, phydm_noisy_detection,
 *   phydm_dig, phydm_cck_pd_th, phydm_adaptivity, phydm_ra_info_watchdog,
 *   phydm_tx_path_diversity, phydm_cfo_tracking, phydm_dynamic_tx_power,
 *   odm_antenna_diversity, phydm_beamforming_watchdog, halrf_watchdog,
 *   phydm_primary_cca, ...
 *
 * Devourer's port scope (this header):
 *   - FA counter statistics for the AC family (8812/8814/8821) —
 *     reads BB OFDM/CCK FA+CCA counters at 0xfcc..0xfd0, resets at
 *     tick boundary so successive ticks see only the delta.
 *
 * Out of scope (added as separate ports):
 *   - RSSI monitor / CFO tracking / adaptivity — depend on TX/RX
 *     activity which devourer drives via its own paths.
 *   - Beamforming, antenna diversity — out of scope for monitor mode.
 *
 * Thread model: spawns a single background `std::thread` that wakes
 * every 2s, runs `TickOnce()`, sleeps again. Stops cleanly on
 * destruction (sets _stop, joins). `TickOnce()` is also callable
 * directly (used at end-of-init for an immediate first cycle so the
 * canary capture sees post-watchdog state). */
class PhydmWatchdog {
public:
  PhydmWatchdog(RtlAdapter device,
                std::shared_ptr<EepromManager> eepromManager,
                RadioManagementModule *radio, Logger_t logger);
  ~PhydmWatchdog();

  /* Spawn the watchdog thread. Idempotent. */
  void Start();
  /* Signal stop + join. Idempotent. Called from destructor. */
  void Stop();
  /* Run one watchdog cycle synchronously on the calling thread. */
  void TickOnce();

  /* EDCCA threshold tracking (the SetCcaMode enable path): when on, each
   * tick re-derives the BB 0x8a4 L2H/H2L from the IGI DIG just wrote —
   * the vendor couples the EDCCA threshold to IGI per watchdog cycle
   * (phydm_adaptivity). Off = leave 0x8a4 alone (SetCcaMode owns the
   * parked/static value). */
  void SetEdccaTrack(bool on) { _edcca_track.store(on, std::memory_order_relaxed); }

  /* Most-recent FA counter snapshot — exposed for diagnostics /
   * future DIG integration. */
  struct FaCnt {
    uint32_t cnt_ofdm_fail;
    uint32_t cnt_cck_fail;
    uint32_t cnt_ofdm_cca;
    uint32_t cnt_cck_cca;
    uint32_t cnt_ht_crc32_error;
    uint32_t cnt_ht_crc32_ok;
    uint32_t cnt_vht_crc32_error;
    uint32_t cnt_vht_crc32_ok;
    uint32_t cnt_ofdm_crc32_error;
    uint32_t cnt_ofdm_crc32_ok;
    uint32_t cnt_cck_crc32_error;
    uint32_t cnt_cck_crc32_ok;
    uint32_t cnt_all;
    uint32_t cnt_cca_all;
  };
  FaCnt LastFaCnt() const;

private:
  void ThreadLoop();
  /* Port of `phydm_fa_cnt_statistics_ac` (phydm_dig.c:1421). Reads
   * OFDM/CCK FA + CCA + CRC32 counters from page-F BB registers. */
  void ReadFaCountersAc(FaCnt &out);
  /* Port of `phydm_false_alarm_counter_reg_reset` AC branch
   * (phydm_dig.c:1287-1298). Pulses BB reg toggles to clear the
   * counter latches so the next tick captures fresh-since-now
   * counts. */
  void ResetFaCountersAc();
  /* Port of `phydm_dig` (phydm_dig.c:1066) walking BB 0xc50/0xe50/
   * 0x1850/0x1a50 byte 0 (per-path IGI) based on the most recent
   * FA count. Always hits the !is_linked monitor-mode path: bounds
   * [DIG_MIN_COVERAGE=0x1c, DIG_MAX_OF_MIN_BALANCE_MODE=0x2a],
   * step={2,1,2}, FA thresholds={250,500,750}. */
  void DigInit();
  void DigTick(uint32_t fa_cnt);
  void DigWriteIgi(uint8_t igi);

  RtlAdapter _device;
  std::shared_ptr<EepromManager> _eepromManager;
  RadioManagementModule *_radio;
  Logger_t _logger;

  std::thread _thread;
  std::atomic<bool> _running{false};
  std::atomic<bool> _stop{false};

  /* Latest snapshot. mutable so const accessor is feasible without
   * dragging in a mutex; reader sees a torn-but-bounded copy. */
  mutable FaCnt _lastFaCnt{};

  /* DIG state, mirroring `struct phydm_dig_struct` minus the fields
   * we don't use (TDMA, damping check, antdiv override). All in
   * "monitor mode, never linked" semantics — we never look at
   * rssi_min / is_linked because devourer doesn't track them.
   * `_digInitialised` distinguishes the first tick (which reads
   * BB 0xc50 to seed cur_ig_value) from subsequent ticks (which
   * just walk based on FA count). */
  bool _digInitialised = false;
  uint8_t _cur_ig_value = 0x20;
  std::atomic<bool> _edcca_track{false};
  uint8_t _edcca_last_l2h = 0x7f; /* parked sentinel — first tick writes */
  uint8_t _dm_dig_max = 0x26;       /* DIG_MAX_COVERAGR */
  uint8_t _dm_dig_min = 0x1c;       /* DIG_MIN_COVERAGE */
  uint8_t _dig_max_of_min = 0x2a;   /* DIG_MAX_OF_MIN_BALANCE_MODE */
  uint8_t _rx_gain_range_max = 0x2a;
  uint8_t _rx_gain_range_min = 0x1c;
};

/* Vendor adaptivity operating point for the 11AC dies (phydm_adaptivity.c
 * phydm_adaptivity / phydm_set_l2h_th_ini): the BB EDCCA low-to-high
 * threshold is IGI-coupled — L2H = th_l2h_ini + (igi_base 0x32 − IGI),
 * upper-clamped to 10; H2L = L2H − 7. th_l2h_ini is −14 (0xf2) on the
 * 8814A, −17 (0xef) on the 8812A/8811A/8821A. Shared by the one-shot
 * SetCcaMode apply (static IGI, watchdog off — the default config) and the
 * per-tick re-track (watchdog on). */
inline int8_t jaguar1_edcca_l2h(int8_t th_l2h_ini, uint8_t igi) {
  const int l2h = th_l2h_ini + (0x32 - static_cast<int>(igi));
  return static_cast<int8_t>(l2h > 10 ? 10 : l2h);
}

#endif /* PHYDM_WATCHDOG_H */
