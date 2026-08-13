#ifndef HALMAC_8733B_MAC_H
#define HALMAC_8733B_MAC_H

#include <array>
#include <cstddef>
#include <cstdint>

#include "RtlAdapter.h"
#include "logger.h"

namespace rtl8733b {

inline constexpr size_t kPhysicalEfuseSize = 512;
inline constexpr size_t kProtectedEfuseSize = 124;
inline constexpr size_t kReservedCsEfuseSize = 24;
inline constexpr size_t kReservedEfuseSize = 16;
inline constexpr size_t kLogicalEfuseSize = 768;
inline constexpr size_t kTxPowerEfuseOffset8733b = 0x10;
inline constexpr size_t kTxPowerCalibrateOffset8733b = 0xc8;

/* Device-side RX aggregate ceiling programmed by configure_monitor_rx (HALMAC
 * RXDMA_AGG size field, 4 KiB pages -> 0x03). An aggregate must never span two
 * bulk-IN URBs — the descriptor tail would land in one completion and its body
 * in the next — so the RX loop floors its URB size at this value. Keep the two
 * in step: raising either alone reintroduces the straddle. */
inline constexpr int kRxAggregateBytes8733b = 12 * 1024;

enum class TxPowerPgMode8733b {
  DirectIndex,
  TssiOffset,
  Unknown,
};

struct DirectTxPowerPathInfo8733b {
  std::array<uint8_t, 6> cck_base_2g{};
  std::array<uint8_t, 5> bw40_base_2g{};
  std::array<uint8_t, 14> bw40_base_5g{};
  int8_t ofdm_diff_2g = 0;
  int8_t bw20_diff_2g = 0;
  int8_t ofdm_diff_5g = 0;
  int8_t bw20_diff_5g = 0;
  bool valid_2g = false;
  bool valid_5g = false;
};

struct DirectTxPowerInfo8733b {
  std::array<DirectTxPowerPathInfo8733b, 2> path{};

  /* The vendor HAL advertises two 2.4 GHz register paths but only one 5 GHz
   * RF path. Path-B 5 GHz bytes exist in the common layout but are not a
   * prerequisite for this device profile. */
  bool usable() const {
    return path[0].valid_2g && path[0].valid_5g && path[1].valid_2g;
  }
};

struct TssiPowerInfo8733b {
  std::array<int8_t, 25> path_a_de{};
  std::array<int8_t, 11> path_b_de{};
  std::array<std::array<int8_t, 2>, 8> trim{};
  bool path_a_programmed = false;
  bool path_b_programmed = false;
  bool trim_programmed = false;
};

struct EfuseInfo {
  std::array<uint8_t, kPhysicalEfuseSize> physical{};
  std::array<uint8_t, kLogicalEfuseSize> logical{};
  std::array<uint8_t, 6> mac{};
  size_t used_bytes = 0;
  uint16_t id = 0xffff;
  uint16_t vid = 0xffff;
  uint16_t pid = 0xffff;
  uint8_t rfe_type = 0xff;
  uint8_t xtal = 0xff;
  uint8_t thermal = 0xff;
  uint8_t trx_path = 0xff;
  uint8_t tx_power_calibrate = 0xff;
  uint8_t tx_power_tracking_mode = 0x0f;
  TxPowerPgMode8733b tx_power_mode = TxPowerPgMode8733b::Unknown;
  DirectTxPowerInfo8733b direct_tx_power{};
  bool direct_tx_power_parsed = false;
  TssiPowerInfo8733b tssi_power{};
  bool tssi_power_parsed = false;
  bool valid = false;

  bool mac_valid() const;
};

struct MacState {
  uint16_t pq_map = 0;
  uint32_t rqpn_hlpq = 0;
  uint32_t rqpn_npq = 0;
  uint8_t reserved_boundary = 0;
  uint16_t rx_boundary = 0;
  uint8_t cr = 0;
  uint8_t rxdma_mode = 0;
  uint16_t rx_agg = 0;
  uint32_t rcr = 0;
  uint16_t ldpc_control = 0;

  bool matches_normal_usb3out() const;
};

/* RTL8733B HALMAC 87xx MAC/EFUSE plane: physical OTP, logical-map decode,
 * normal-mode queue/page allocation, protocol/EDCA/WMAC, USB RX-DMA, and TRX
 * lifecycle. The probe and production device deliberately share this path. */
class Halmac8733bMac {
public:
  Halmac8733bMac(RtlAdapter device, Logger_t logger);

  bool read_efuse(EfuseInfo &out);
  bool initialize(const EfuseInfo &efuse);
  bool configure_monitor_rx(bool keep_corrupted);
  void stop();
  MacState read_mac_state();

  static bool parse_physical_efuse(const uint8_t *physical, size_t physical_len,
                                   uint8_t *logical, size_t logical_len,
                                   size_t *used_bytes = nullptr);
  static TxPowerPgMode8733b tx_power_pg_mode(uint8_t calibrate);
  static bool parse_direct_tx_power_efuse(const uint8_t *logical,
                                          size_t logical_len,
                                          DirectTxPowerInfo8733b &out);
  static bool parse_tssi_power_efuse(const uint8_t *logical,
                                     size_t logical_len,
                                     const uint8_t *physical,
                                     size_t physical_len,
                                     TssiPowerInfo8733b &out);

private:
  bool read_physical_efuse(std::array<uint8_t, kPhysicalEfuseSize> &map);
  bool read_physical_byte(uint16_t address, uint8_t &value);
  bool init_trx();
  bool init_queues();
  void init_protocol();
  void init_edca();
  void init_wmac();
  void init_usb();
  void program_mac(const std::array<uint8_t, 6> &mac);

  RtlAdapter _device;
  Logger_t _logger;
};

} // namespace rtl8733b

#endif /* HALMAC_8733B_MAC_H */
