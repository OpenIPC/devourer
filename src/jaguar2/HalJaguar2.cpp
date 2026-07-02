#include "HalJaguar2.h"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

namespace jaguar2 {

namespace {
/* RTL8822B USB power sequence, transcribed from the halmac WLAN_PWR_CFG flow
 * (reference/rtl88x2bu/hal/halmac/halmac_88xx/halmac_8822b/halmac_pwr_seq_8822b.c),
 * keeping the USB- and ALL-interface entries and dropping the SDIO/PCI-only
 * steps. Each step is a byte read-modify-write, a byte poll, or a delay. The
 * 0x10A8..0x10AA writes are cut-C-only in halmac; production RTL8822BU is C-cut
 * (SYS_CFG1 cut field = 2), so they are included. */
enum PwrCmd { PC_WRITE, PC_POLL, PC_DELAY, PC_END };
struct PwrCfg { uint16_t off; uint8_t cmd; uint8_t msk; uint8_t val; };
constexpr uint8_t B(int n) { return static_cast<uint8_t>(1u << n); }

/* card_en_flow_8822b = CARDDIS_TO_CARDEMU + CARDEMU_TO_ACT (USB/ALL). */
const PwrCfg kPwrOn8822bUsb[] = {
    /* --- card-disable -> card-emulation --- */
    {0x004A, PC_WRITE, B(0), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(3) | B(4) | B(7)), 0},
    /* --- card-emulation -> active --- */
    {0xFF0A, PC_WRITE, 0xFF, 0},
    {0xFF0B, PC_WRITE, 0xFF, 0},
    {0x0012, PC_WRITE, B(1), 0},
    {0x0012, PC_WRITE, B(0), B(0)},
    {0x0020, PC_WRITE, B(0), B(0)},
    {0x0001, PC_DELAY, 0, 1}, /* 1 ms */
    {0x0000, PC_WRITE, B(5), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(4) | B(3) | B(2)), 0},
    {0x0006, PC_POLL, B(1), B(1)},
    {0xFF1A, PC_WRITE, 0xFF, 0},
    {0x0006, PC_WRITE, B(0), B(0)},
    {0x0005, PC_WRITE, B(7), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(4) | B(3)), 0},
    {0x10C3, PC_WRITE, B(0), B(0)},
    {0x0005, PC_WRITE, B(0), B(0)},
    {0x0005, PC_POLL, B(0), 0},
    {0x0020, PC_WRITE, B(3), B(3)},
    {0x10A8, PC_WRITE, 0xFF, 0},    /* cut-C */
    {0x10A9, PC_WRITE, 0xFF, 0xef}, /* cut-C */
    {0x10AA, PC_WRITE, 0xFF, 0x0c}, /* cut-C */
    {0x0029, PC_WRITE, 0xFF, 0xF9},
    {0x0024, PC_WRITE, B(2), 0},
    {0x00AF, PC_WRITE, B(5), B(5)},
    {0, PC_END, 0, 0},
};

/* card_dis_flow_8822b = ACT_TO_CARDEMU + CARDEMU_TO_CARDDIS (USB/ALL). */
const PwrCfg kPwrOff8822bUsb[] = {
    /* --- active -> card-emulation --- */
    {0x0093, PC_WRITE, 0xFF, 0xC4},
    {0x001F, PC_WRITE, 0xFF, 0},
    {0x00EF, PC_WRITE, 0xFF, 0},
    {0xFF1A, PC_WRITE, 0xFF, 0x30},
    {0x0049, PC_WRITE, B(1), 0},
    {0x0006, PC_WRITE, B(0), B(0)},
    {0x0002, PC_WRITE, B(1), 0},
    {0x10C3, PC_WRITE, B(0), 0},
    {0x0005, PC_WRITE, B(1), B(1)},
    {0x0005, PC_POLL, B(1), 0},
    {0x0020, PC_WRITE, B(3), 0},
    {0x0000, PC_WRITE, B(5), B(5)},
    /* --- card-emulation -> card-disable --- */
    {0x0007, PC_WRITE, 0xFF, 0x20},
    {0x0067, PC_WRITE, B(5), 0},
    {0x004A, PC_WRITE, B(0), 0},
    {0x0081, PC_WRITE, static_cast<uint8_t>(B(7) | B(6)), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(3) | B(4)), B(3)},
    {0x0090, PC_WRITE, B(1), 0},
    {0, PC_END, 0, 0},
};

void run_pwr_seq(RtlUsbAdapter &dev, const PwrCfg *seq, uint32_t poll_max,
                 bool poll_fatal) {
  for (const PwrCfg *p = seq; p->cmd != PC_END; ++p) {
    if (p->cmd == PC_WRITE) {
      uint8_t v = dev.rtw_read8(p->off);
      v = static_cast<uint8_t>((v & ~p->msk) | (p->val & p->msk));
      dev.rtw_write8(p->off, v);
    } else if (p->cmd == PC_DELAY) {
      std::this_thread::sleep_for(std::chrono::milliseconds(p->val));
    } else { /* PC_POLL */
      uint32_t cnt = poll_max;
      while ((dev.rtw_read8(p->off) & p->msk) != (p->val & p->msk)) {
        if (--cnt == 0) {
          if (poll_fatal)
            throw std::runtime_error(
                "Jaguar2: power-on poll timeout (chip not responding)");
          break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
    }
  }
}
} /* namespace */

HalJaguar2::HalJaguar2(RtlUsbAdapter device, Logger_t logger)
    : _device{std::move(device)}, _logger{std::move(logger)} {}

void HalJaguar2::power_off() {
  run_pwr_seq(_device, kPwrOff8822bUsb, /*poll_max=*/2000, /*poll_fatal=*/false);
  _logger->info("Jaguar2: power-off (card-disable) sequence applied");
}

void HalJaguar2::power_on() {
  power_off(); /* reset from any prior (kernel-left active) state first */
  run_pwr_seq(_device, kPwrOn8822bUsb, /*poll_max=*/5000, /*poll_fatal=*/true);
  _logger->info("Jaguar2: power-on sequence complete (card active)");
}

void HalJaguar2::read_chip_version() {
  constexpr uint16_t REG_SYS_CFG1_8822B = 0x00F0;
  uint32_t v = _device.rtw_read32(REG_SYS_CFG1_8822B);

  _ver.test_chip = (v & (1u << 23)) != 0;
  _ver.cut = static_cast<uint8_t>((v >> 12) & 0xf);
  _ver.rf_2t2r = (v & (1u << 27)) ? 1 : 0;
  uint8_t vend = static_cast<uint8_t>(((v >> 16) & 0xf) >> 2);
  _ver.vendor = (vend <= 2) ? vend : 0;

  static const char *vn[] = {"TSMC", "SMIC", "UMC"};
  _logger->info("Jaguar2 chip: 8822B cut={} ({}{}) {} (SYS_CFG1=0x{:08x})",
                _ver.cut, vn[_ver.vendor], _ver.test_chip ? ",test" : "",
                _ver.rf_2t2r ? "2T2R" : "1T1R", v);
}

} /* namespace jaguar2 */
