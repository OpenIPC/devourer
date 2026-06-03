#include "PhydmWatchdog.h"

#include "RadioManagementModule.h"

#include <chrono>

namespace {

/* Phydm AC FA counter register addresses, from
 * `hal/phydm/phydm_regdefine11ac.h`. */
constexpr uint16_t kRegOfdmFaType1 = 0xFCC; /* fast_fsync hi 16 */
constexpr uint16_t kRegOfdmFaType2 = 0xFD0; /* sb_search_fail lo 16 */
constexpr uint16_t kRegOfdmFaType3 = 0xFBC; /* parity_fail / rate_illegal */
constexpr uint16_t kRegOfdmFaType4 = 0xFC0; /* crc8_fail / mcs_fail */
constexpr uint16_t kRegOfdmFaType5 = 0xFC4; /* vht_crc8_fail */
constexpr uint16_t kRegOfdmFaType6 = 0xFC8; /* vht_mcs_fail */
constexpr uint16_t kRegOfdmFail = 0xF48;    /* OFDM FA count */
constexpr uint16_t kRegCckFa = 0xA5C;       /* CCK FA count */
constexpr uint16_t kRegCckCcaCnt = 0xF08;   /* CCK/OFDM CCA count */
constexpr uint16_t kRegCckCrc32Cnt = 0xF04;
constexpr uint16_t kRegVhtCrc32Cnt = 0xF0c;
constexpr uint16_t kRegHtCrc32Cnt = 0xF10;
constexpr uint16_t kRegOfdmCrc32Cnt = 0xF14;
constexpr uint16_t kRegBbRxPath = 0x808; /* BIT(28) = CCK enable */

constexpr uint32_t kMaskDWord = 0xFFFFFFFF;
constexpr uint32_t kMaskLWord = 0x0000FFFF;
constexpr uint32_t kBit28 = 1u << 28;
constexpr uint32_t kBit17 = 1u << 17;
constexpr uint32_t kBit15 = 1u << 15;
constexpr uint32_t kBit0 = 1u << 0;

/* Watchdog tick interval. Upstream uses 2s on Linux (CE) per
 * `ADAPTIVITY_INTERVAL` / phydm_interface.c. */
constexpr auto kTickInterval = std::chrono::seconds(2);

} // namespace

PhydmWatchdog::PhydmWatchdog(RtlUsbAdapter device,
                             std::shared_ptr<EepromManager> eepromManager,
                             RadioManagementModule *radio, Logger_t logger)
    : _device(device), _eepromManager(eepromManager), _radio(radio),
      _logger(logger) {}

PhydmWatchdog::~PhydmWatchdog() { Stop(); }

void PhydmWatchdog::Start() {
  bool expected = false;
  if (!_running.compare_exchange_strong(expected, true)) {
    return; /* already running */
  }
  _stop.store(false);
  _thread = std::thread([this]() { ThreadLoop(); });
}

void PhydmWatchdog::Stop() {
  bool expected = true;
  if (!_running.compare_exchange_strong(expected, false)) {
    return; /* not running */
  }
  _stop.store(true);
  if (_thread.joinable()) {
    _thread.join();
  }
}

void PhydmWatchdog::ThreadLoop() {
  /* Wake every kTickInterval and run TickOnce. Interruptible via
   * _stop. Use short polled sleeps (200ms) so Stop() returns
   * quickly even mid-interval. */
  auto next_tick = std::chrono::steady_clock::now() + kTickInterval;
  while (!_stop.load()) {
    if (std::chrono::steady_clock::now() >= next_tick) {
      TickOnce();
      next_tick = std::chrono::steady_clock::now() + kTickInterval;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

void PhydmWatchdog::TickOnce() {
  /* Read FA counters (no-op if BB isn't running yet — counters
   * read zero). Then reset the counter latches so the next tick
   * captures only the delta. */
  FaCnt fa{};
  ReadFaCountersAc(fa);
  _lastFaCnt = fa;
  ResetFaCountersAc();
}

void PhydmWatchdog::ReadFaCountersAc(FaCnt &out) {
  /* Port of `phydm_fa_cnt_statistics_ac` (phydm_dig.c:1421).
   * Reads OFDM/CCK FA + CCA + CRC32 counters from page-F BB
   * registers. Fields not needed for the watchdog's own logic are
   * skipped; the canonical phydm struct populates ~14 counters
   * total and we cover the same set. */
  uint32_t v = 0;

  /* OFDM FA breakdown — kept for diagnostics, not used by the
   * current minimal watchdog logic. */
  (void)_radio->phy_query_bb_reg_public(kRegOfdmFaType1, kMaskDWord);
  (void)_radio->phy_query_bb_reg_public(kRegOfdmFaType2, kMaskDWord);
  (void)_radio->phy_query_bb_reg_public(kRegOfdmFaType3, kMaskDWord);
  (void)_radio->phy_query_bb_reg_public(kRegOfdmFaType4, kMaskDWord);
  (void)_radio->phy_query_bb_reg_public(kRegOfdmFaType5, kMaskDWord);
  (void)_radio->phy_query_bb_reg_public(kRegOfdmFaType6, kMaskDWord);

  /* OFDM/CCK FA counts. */
  out.cnt_ofdm_fail =
      _radio->phy_query_bb_reg_public(kRegOfdmFail, kMaskLWord);
  out.cnt_cck_fail =
      _radio->phy_query_bb_reg_public(kRegCckFa, kMaskLWord);

  /* CCA counts. */
  v = _radio->phy_query_bb_reg_public(kRegCckCcaCnt, kMaskDWord);
  out.cnt_ofdm_cca = (v & 0xffff0000) >> 16;
  out.cnt_cck_cca = v & 0xffff;

  /* CRC32 counters. */
  v = _radio->phy_query_bb_reg_public(kRegCckCrc32Cnt, kMaskDWord);
  out.cnt_cck_crc32_error = (v & 0xffff0000) >> 16;
  out.cnt_cck_crc32_ok = v & 0xffff;

  v = _radio->phy_query_bb_reg_public(kRegOfdmCrc32Cnt, kMaskDWord);
  out.cnt_ofdm_crc32_error = (v & 0xffff0000) >> 16;
  out.cnt_ofdm_crc32_ok = v & 0xffff;

  v = _radio->phy_query_bb_reg_public(kRegHtCrc32Cnt, kMaskDWord);
  out.cnt_ht_crc32_error = (v & 0xffff0000) >> 16;
  out.cnt_ht_crc32_ok = v & 0xffff;

  v = _radio->phy_query_bb_reg_public(kRegVhtCrc32Cnt, kMaskDWord);
  out.cnt_vht_crc32_error = (v & 0xffff0000) >> 16;
  out.cnt_vht_crc32_ok = v & 0xffff;

  /* Cumulative FA + CCA. CCK bits live in 0x808 BIT(28); if set,
   * BB is using CCK so total includes CCK + OFDM. Otherwise OFDM
   * only. */
  const bool cck_enable =
      _radio->phy_query_bb_reg_public(kRegBbRxPath, kBit28) != 0;
  if (cck_enable) {
    out.cnt_all = out.cnt_ofdm_fail + out.cnt_cck_fail;
    out.cnt_cca_all = out.cnt_ofdm_cca + out.cnt_cck_cca;
  } else {
    out.cnt_all = out.cnt_ofdm_fail;
    out.cnt_cca_all = out.cnt_ofdm_cca;
  }
}

void PhydmWatchdog::ResetFaCountersAc() {
  /* Port of `phydm_false_alarm_counter_reg_reset` AC branch
   * (phydm_dig.c:1287-1298). Pulses the OFDM/CCK FA counter
   * reset bits, then resets the PMAC/PHY counter via
   * `phydm_reset_bb_hw_cnt` (R_0xb58 BIT(0)). */
  /* OFDM FA counter reset (R_0x9a4 BIT(17) toggle). */
  _device.phy_set_bb_reg(0x9a4, kBit17, 1);
  _device.phy_set_bb_reg(0x9a4, kBit17, 0);

  /* CCK FA counter reset (R_0xa2c BIT(15) toggle). */
  _device.phy_set_bb_reg(0xa2c, kBit15, 0);
  _device.phy_set_bb_reg(0xa2c, kBit15, 1);

  /* All-counter reset (R_0xb58 BIT(0) toggle). */
  _device.phy_set_bb_reg(0xb58, kBit0, 1);
  _device.phy_set_bb_reg(0xb58, kBit0, 0);
}

PhydmWatchdog::FaCnt PhydmWatchdog::LastFaCnt() const { return _lastFaCnt; }
