#ifndef PHYDM_WATCHDOG_H
#define PHYDM_WATCHDOG_H

#include "EepromManager.h"
#include "RtlUsbAdapter.h"
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
 * Out of scope for the first cut (added as separate ports):
 *   - DIG (Dynamic Initial Gain) — walks BB 0xc50[7:0] based on FA
 *     count + RSSI. Substantial port (~400 LOC of helpers).
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
  PhydmWatchdog(RtlUsbAdapter device,
                std::shared_ptr<EepromManager> eepromManager,
                RadioManagementModule *radio, Logger_t logger);
  ~PhydmWatchdog();

  /* Spawn the watchdog thread. Idempotent. */
  void Start();
  /* Signal stop + join. Idempotent. Called from destructor. */
  void Stop();
  /* Run one watchdog cycle synchronously on the calling thread. */
  void TickOnce();

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

  RtlUsbAdapter _device;
  std::shared_ptr<EepromManager> _eepromManager;
  RadioManagementModule *_radio;
  Logger_t _logger;

  std::thread _thread;
  std::atomic<bool> _running{false};
  std::atomic<bool> _stop{false};

  /* Latest snapshot. mutable so const accessor is feasible without
   * dragging in a mutex; reader sees a torn-but-bounded copy. */
  mutable FaCnt _lastFaCnt{};
};

#endif /* PHYDM_WATCHDOG_H */
