#ifndef RTL8733B_BRINGUP_H
#define RTL8733B_BRINGUP_H

#include <cstddef>
#include <cstdint>

#include "RtlAdapter.h"
#include "logger.h"

namespace rtl8733b {

struct ChipInfo {
  uint32_t sys_cfg1 = 0;
  uint8_t die_id = 0;
  uint8_t cut = 0;
  uint8_t secure_ctrl = 0;
  uint8_t cr = 0;
  uint32_t sys_status1 = 0;
  uint16_t mcufw_ctrl = 0;
  bool efuse_autoload_ok = false;

  bool matches() const { return die_id == 0x16; }
};

struct RegisterSnapshot {
  uint16_t sys_func_en = 0;
  uint8_t rf_ctrl = 0;
  uint8_t cr = 0;
  uint16_t mcufw_ctrl = 0;
  uint32_t sys_status1 = 0;
  uint32_t ext_sys_func_en = 0;
};

/* Focused RTL8733B USB bring-up: identity, card-enable, and WLAN firmware
 * download only. It deliberately does not claim full device support; the
 * chip-specific MAC/BB/RF tables and RF calibration remain to be ported. */
class Rtl8733bBringup {
public:
  Rtl8733bBringup(RtlAdapter device, Logger_t logger);

  ChipInfo read_chip_info();
  RegisterSnapshot read_register_snapshot();
  bool power_on();
  bool power_off();
  bool download_default_firmware(uint8_t cut);

  bool checksum_ok() const { return _checksum_ok; }
  bool ready_ok() const { return _ready_ok; }
  bool download_attempted() const { return _download_attempted; }

private:
  void pre_init_system_cfg();
  void init_system_cfg();
  bool download_firmware(const uint8_t *fw, size_t size);
  bool check_fw_size(const uint8_t *fw, size_t size);
  bool start_dlfw(const uint8_t *fw);
  bool dlfw_to_mem(const uint8_t *fw, uint32_t src, uint32_t dest,
                   uint32_t size);
  bool send_fw_page(uint16_t page, const uint8_t *data, uint32_t size);
  bool iddma(uint32_t src, uint32_t dest, uint32_t len, bool first);
  bool check_checksum(uint32_t dest);
  bool finish_dlfw();
  void wlan_cpu_enable(bool enable);
  void platform_reset();

  uint8_t r8(uint16_t reg);
  uint16_t r16(uint16_t reg);
  uint32_t r32(uint16_t reg);
  void w8(uint16_t reg, uint8_t value);
  void w16(uint16_t reg, uint16_t value);
  void w32(uint16_t reg, uint32_t value);

  RtlAdapter _device;
  Logger_t _logger;
  bool _powered = false;
  bool _download_attempted = false;
  bool _checksum_ok = false;
  bool _ready_ok = false;
};

} // namespace rtl8733b

#endif /* RTL8733B_BRINGUP_H */
