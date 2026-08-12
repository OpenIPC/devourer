#include "Rtl8733bBringup.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "hal8733b_fw.h"

namespace rtl8733b {
namespace {

constexpr uint16_t R8733_SYS_FUNC_EN = 0x0002;
constexpr uint16_t R8733_9346CR = 0x000a;
constexpr uint16_t R8733_SECURE_CTRL = 0x0014;
constexpr uint16_t R8733_REG_ACCESS_CTRL = 0x001c;
constexpr uint16_t R8733_RF_CTRL = 0x001f;
constexpr uint16_t R8733_GPIO_MUXCFG = 0x0040;
constexpr uint16_t R8733_LED_CFG = 0x004c;
constexpr uint16_t R8733_PAD_CTRL1 = 0x0064;
constexpr uint16_t R8733_SYS_SDIO_CTRL = 0x0070;
constexpr uint16_t R8733_HCI_OPT_CTRL = 0x0074;
constexpr uint16_t R8733_MCUFW_CTRL = 0x0080;
constexpr uint16_t R8733_SYS_CFG1 = 0x00f0;
constexpr uint16_t R8733_SYS_STATUS1 = 0x00f4;
constexpr uint16_t R8733_SYS_CFG2 = 0x00fc;
constexpr uint16_t R8733_CR = 0x0100;
constexpr uint16_t R8733_TXDMA_PQ_MAP = 0x010c;
constexpr uint16_t R8733_RQPN_CTRL_HLPQ = 0x0200;
constexpr uint16_t R8733_DWBCN0_CTRL = 0x0208;
constexpr uint16_t R8733_TXDMA_STATUS = 0x0210;
constexpr uint16_t R8733_FWHW_TXQ_CTRL = 0x0420;
constexpr uint16_t R8733_BCN_CTRL = 0x0550;
constexpr uint16_t R8733_EXT_SYS_FUNC_EN = 0x1000;
constexpr uint16_t R8733_EXT_SYS_CLK_CTRL = 0x1008;
constexpr uint16_t R8733_FW_DBG7 = 0x10ac;
constexpr uint16_t R8733_CR_EXT = 0x1100;
constexpr uint16_t R8733_DDMA_CH0SA = 0x1200;
constexpr uint16_t R8733_DDMA_CH0DA = 0x1204;
constexpr uint16_t R8733_DDMA_CH0CTRL = 0x1208;

constexpr uint32_t OCPBASE_TXBUF = 0x18780000;
constexpr uint32_t OCPBASE_DMEM = 0x14200000;
constexpr uint32_t TXDESC_SIZE = 40;
constexpr uint32_t FW_HEADER_SIZE = 64;
constexpr uint32_t FW_CHECKSUM_SIZE = 8;
constexpr uint32_t FW_PACKET_SIZE = 8192;

constexpr uint32_t DDMA_OWN = 1u << 31;
constexpr uint32_t DDMA_CHECKSUM_EN = 1u << 29;
constexpr uint32_t DDMA_CHECKSUM_STATUS = 1u << 27;
constexpr uint32_t DDMA_RESET_CHECKSUM = 1u << 25;
constexpr uint32_t DDMA_CHECKSUM_CONT = 1u << 24;
constexpr uint32_t DDMA_LENGTH_MASK = 0x3ffff;

uint32_t le32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

void set_bits(uint8_t *p, unsigned bit, unsigned width, uint32_t value) {
  uint32_t word = le32(p);
  const uint32_t mask = width == 32 ? ~0u : ((1u << width) - 1u) << bit;
  word = (word & ~mask) | ((value << bit) & mask);
  p[0] = static_cast<uint8_t>(word);
  p[1] = static_cast<uint8_t>(word >> 8);
  p[2] = static_cast<uint8_t>(word >> 16);
  p[3] = static_cast<uint8_t>(word >> 24);
}

uint16_t le16(const uint8_t *p) {
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

void fill_rsvd_page_txdesc(std::array<uint8_t, TXDESC_SIZE> &storage,
                           uint16_t size) {
  uint8_t *desc = storage.data();
  set_bits(desc + 0x00, 0, 16, size);       // TXPKTSIZE
  set_bits(desc + 0x00, 16, 8, TXDESC_SIZE); // OFFSET
  set_bits(desc + 0x04, 8, 5, 0x10);       // QSEL_BEACON
  set_bits(desc + 0x1c, 0, 16, 0);         // checksum field

  uint16_t checksum = 0;
  for (unsigned i = 0; i < 8; ++i)
    checksum ^= static_cast<uint16_t>(le16(desc + i * 4) ^
                                      le16(desc + i * 4 + 2));
  checksum ^= 0xffff; // RTL8733B differs from the 88xx Jaguar descriptor
  set_bits(desc + 0x1c, 0, 16, checksum);
}

void delay_us(unsigned us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

} // namespace

Rtl8733bBringup::Rtl8733bBringup(RtlAdapter device, Logger_t logger)
    : _device(std::move(device)), _logger(std::move(logger)) {}

uint8_t Rtl8733bBringup::r8(uint16_t reg) { return _device.rtw_read8(reg); }
uint16_t Rtl8733bBringup::r16(uint16_t reg) { return _device.rtw_read16(reg); }
uint32_t Rtl8733bBringup::r32(uint16_t reg) { return _device.rtw_read32(reg); }
void Rtl8733bBringup::w8(uint16_t reg, uint8_t value) {
  _device.rtw_write8(reg, value);
}
void Rtl8733bBringup::w16(uint16_t reg, uint16_t value) {
  _device.rtw_write16(reg, value);
}
void Rtl8733bBringup::w32(uint16_t reg, uint32_t value) {
  _device.rtw_write32(reg, value);
}

ChipInfo Rtl8733bBringup::read_chip_info() {
  ChipInfo info;
  info.sys_cfg1 = r32(R8733_SYS_CFG1);
  info.die_id = r8(R8733_SYS_CFG2);
  info.cut = static_cast<uint8_t>((info.sys_cfg1 >> 12) & 0xf);
  info.secure_ctrl = r8(R8733_SECURE_CTRL);
  info.cr = r8(R8733_CR);
  info.sys_status1 = r32(R8733_SYS_STATUS1);
  info.mcufw_ctrl = r16(R8733_MCUFW_CTRL);
  info.efuse_autoload_ok = (r8(R8733_9346CR) & (1u << 5)) != 0;
  return info;
}

RegisterSnapshot Rtl8733bBringup::read_register_snapshot() {
  RegisterSnapshot snapshot;
  snapshot.sys_func_en = r16(R8733_SYS_FUNC_EN);
  snapshot.rf_ctrl = r8(R8733_RF_CTRL);
  snapshot.cr = r8(R8733_CR);
  snapshot.mcufw_ctrl = r16(R8733_MCUFW_CTRL);
  snapshot.sys_status1 = r32(R8733_SYS_STATUS1);
  snapshot.ext_sys_func_en = r32(R8733_EXT_SYS_FUNC_EN);
  return snapshot;
}

void Rtl8733bBringup::pre_init_system_cfg() {
  w8(R8733_REG_ACCESS_CTRL, 0);
  if (r8(R8733_SYS_CFG2 + 3) == 0x20)
    w8(0xfe5b, static_cast<uint8_t>(r8(0xfe5b) | (1u << 4)));

  w32(R8733_PAD_CTRL1, r32(R8733_PAD_CTRL1) | (3u << 28));
  w32(R8733_LED_CFG, r32(R8733_LED_CFG) & ~(3u << 25));
  w32(R8733_GPIO_MUXCFG, r32(R8733_GPIO_MUXCFG) | (1u << 2));

  // HALMAC_HW_EN_BB_RF(false), exactly as the vendor pre-init path.
  w8(R8733_SYS_FUNC_EN,
     static_cast<uint8_t>(r8(R8733_SYS_FUNC_EN) & ~0x03u));
  w8(R8733_RF_CTRL, static_cast<uint8_t>(r8(R8733_RF_CTRL) & ~0x07u));
  w32(R8733_HCI_OPT_CTRL, r32(R8733_HCI_OPT_CTRL) & ~(7u << 24));

  if (r8(R8733_SYS_CFG1 + 2) & (1u << 4))
    throw std::runtime_error("RTL8733B is strapped in test mode");
}

void Rtl8733bBringup::init_system_cfg() {
  w32(R8733_EXT_SYS_FUNC_EN,
      r32(R8733_EXT_SYS_FUNC_EN) | (1u << 16) | (1u << 17));
  w8(R8733_SYS_FUNC_EN + 1,
     static_cast<uint8_t>(r8(R8733_SYS_FUNC_EN + 1) | 0xd8));
  w8(R8733_CR_EXT + 3,
     static_cast<uint8_t>((r8(R8733_CR_EXT + 3) & 0xf0) | 0x0c));
  w8(R8733_SYS_SDIO_CTRL + 3, 0x04); // vendor Wi-Fi/BT PTA setup
}

bool Rtl8733bBringup::power_on() {
  pre_init_system_cfg();

  if (r16(R8733_MCUFW_CTRL) == 0xc078) {
    const uint8_t rpwm = r8(0xfe58);
    w8(0xfe58, static_cast<uint8_t>((rpwm ^ 0x80) & 0x80));
  }

  const bool already_on =
      r8(R8733_CR) != 0xea && (r8(R8733_SYS_STATUS1 + 1) & 0x01) == 0;
  if (!already_on) {
    auto rmw = [this](uint16_t reg, uint8_t mask, uint8_t value) {
      w8(reg, static_cast<uint8_t>((r8(reg) & ~mask) | (value & mask)));
    };
    auto poll = [this](uint16_t reg, uint8_t mask, uint8_t value) {
      for (unsigned count = 5000; count != 0; --count) {
        if ((r8(reg) & mask) == (value & mask))
          return true;
        delay_us(10);
      }
      return false;
    };

    // card-disable -> card-emulation -> active, USB/all-interface entries.
    rmw(0x0005, 1u << 3, 0);
    rmw(0x004a, 1u << 0, 0);
    if (!poll(0x0006, 1u << 1, 1u << 1)) {
      _logger->error("RTL8733B power: 0x0006[1] ready poll timed out");
      return false;
    }
    rmw(0x0006, 1u << 0, 1u << 0);
    rmw(0x0005, 1u << 0, 1u << 0);
    if (!poll(0x0005, 1u << 0, 0)) {
      _logger->error("RTL8733B power: 0x0005[0] auto-clear timed out");
      return false;
    }
    rmw(0x1002, 1u << 0, 1u << 0);

    for (unsigned i = 0; i < 3; ++i) {
      rmw(0x0002, 0x03, 0x00);
      rmw(0x0002, 0x03, 0x03);
    }
    for (unsigned i = 0; i < 2; ++i) {
      w8(0x001f, 0x00);
      w8(0x0077, 0x00);
      w8(0x001f, 0x87);
      w8(0x0077, 0x87);
    }
    w8(0x001f, 0x00);
    w8(0x0077, 0x00);
    w8(0x001f, 0x87);
    w8(0x0077, 0x87);
    w8(R8733_SYS_STATUS1 + 1,
       static_cast<uint8_t>(r8(R8733_SYS_STATUS1 + 1) & ~0x01u));
  }

  init_system_cfg();
  _powered = r8(R8733_CR) != 0xea && (r8(R8733_SYS_STATUS1 + 1) & 1u) == 0;
  /* Report the computed verdict, not the intent: this line is read during
   * exactly the triage where the readback failed, so claiming "card active"
   * one line before the caller throws "card-enable sequence failed" is worse
   * than useless. */
  if (_powered)
    _logger->info(
        "RTL8733B power: card active, CR=0x{:02x} SYS_STATUS1=0x{:08x}",
        r8(R8733_CR), r32(R8733_SYS_STATUS1));
  else
    _logger->error(
        "RTL8733B power: card NOT active, CR=0x{:02x} SYS_STATUS1=0x{:08x}",
        r8(R8733_CR), r32(R8733_SYS_STATUS1));
  return _powered;
}

bool Rtl8733bBringup::power_off() {
  auto rmw = [this](uint16_t reg, uint8_t mask, uint8_t value) {
    w8(reg, static_cast<uint8_t>((r8(reg) & ~mask) | (value & mask)));
  };
  auto poll = [this](uint16_t reg, uint8_t mask, uint8_t value) {
    for (unsigned count = 5000; count != 0; --count) {
      if ((r8(reg) & mask) == (value & mask))
        return true;
      delay_us(10);
    }
    return false;
  };

  /* HALMAC card_dis_flow_8733b, USB entries only. This is deliberately the
   * full card-disable flow rather than CR=0: the latter stops DMA but leaves
   * the BB/RF and analog power domains enabled after a userspace process
   * releases the interface. */
  rmw(0x00cd, 1u << 0, 0); // release SYS PMC from firmware control
  for (unsigned i = 0; i < 2; ++i) {
    w8(0x001f, 0x00);
    w8(0x0077, 0x00);
    w8(0x001f, 0x87);
    w8(0x0077, 0x87);
  }
  w8(0x001f, 0x00);
  w8(0x0077, 0x00);
  for (unsigned i = 0; i < 2; ++i) {
    rmw(0x0002, 0x03, 0x00);
    rmw(0x0002, 0x03, 0x03);
  }
  rmw(0x0002, 0x03, 0x00);
  rmw(0x0049, 1u << 3, 0);
  rmw(0x0006, 1u << 0, 1u << 0);
  rmw(0x0091, 1u << 2, 1u << 2);
  rmw(0x0005, 1u << 1, 1u << 1);
  if (!poll(0x0005, 1u << 1, 0)) {
    _logger->error("RTL8733B power-off: active-to-card-emulation poll timed out");
    return false;
  }

  // card-emulation -> card-disabled, USB-specific entries.
  rmw(0x0005, 1u << 3, 1u << 3);
  rmw(0x0006, (1u << 7) | (1u << 6), 0);
  w8(0x0007, 0x10);
  rmw(0x1004, 0x0f, 0x02);
  rmw(0x0024, 0x0f, 0);
  rmw(0x0025, 0xf0, 0x10);
  rmw(0x0021, 0xf0, 0x70);
  rmw(0x004a, 1u << 0, 1u << 0);

  _powered = false;
  _checksum_ok = false;
  _ready_ok = false;
  const RegisterSnapshot state = read_register_snapshot();
  const bool off = state.cr == 0xea ||
                   (static_cast<uint8_t>(state.sys_status1 >> 8) & 1u) != 0;
  _logger->info(
      "RTL8733B power-off: ready={} CR=0x{:02x} SYS_FUNC_EN=0x{:04x} "
      "RF_CTRL=0x{:02x} SYS_STATUS1=0x{:08x}",
      off, state.cr, state.sys_func_en, state.rf_ctrl, state.sys_status1);
  return off;
}

bool Rtl8733bBringup::check_fw_size(const uint8_t *fw, size_t size) {
  if (size < FW_HEADER_SIZE)
    return false;
  uint32_t dmem = le32(fw + 36) + FW_CHECKSUM_SIZE;
  uint32_t imem = le32(fw + 48) + FW_CHECKSUM_SIZE;
  uint32_t emem = 0;
  if (fw[24] & (1u << 4))
    emem = le32(fw + 52) + FW_CHECKSUM_SIZE;
  const size_t computed = FW_HEADER_SIZE + dmem + imem + emem;
  if (computed != size) {
    _logger->error("RTL8733B DLFW: image size {} != header size {}", size,
                   computed);
    return false;
  }
  return true;
}

void Rtl8733bBringup::wlan_cpu_enable(bool enable) {
  uint8_t value = r8(R8733_SYS_FUNC_EN + 1);
  value = enable ? static_cast<uint8_t>(value | (1u << 2))
                 : static_cast<uint8_t>(value & ~(1u << 2));
  w8(R8733_SYS_FUNC_EN + 1, value);
}

void Rtl8733bBringup::platform_reset() {
  w8(R8733_EXT_SYS_FUNC_EN + 2,
     static_cast<uint8_t>(r8(R8733_EXT_SYS_FUNC_EN + 2) & ~0x01u));
  w8(R8733_EXT_SYS_FUNC_EN + 2,
     static_cast<uint8_t>(r8(R8733_EXT_SYS_FUNC_EN + 2) | 0x01u));
}

bool Rtl8733bBringup::send_fw_page(uint16_t page, const uint8_t *data,
                                   uint32_t size) {
  std::vector<uint8_t> payload(data, data + size);
  if (((size + TXDESC_SIZE) & 511u) == 0)
    payload.push_back(0); // vendor USB max-packet boundary workaround

  const uint8_t old_cr1 = r8(R8733_CR + 1);
  const uint8_t old_txq2 = r8(R8733_FWHW_TXQ_CTRL + 2);
  w8(R8733_DWBCN0_CTRL + 1, static_cast<uint8_t>(page & 0xff));
  w8(R8733_DWBCN0_CTRL + 2,
     static_cast<uint8_t>(r8(R8733_DWBCN0_CTRL + 2) | 0x01));
  w8(R8733_CR + 1, static_cast<uint8_t>(old_cr1 | 0x01));
  w8(R8733_FWHW_TXQ_CTRL + 2, static_cast<uint8_t>(old_txq2 & ~(1u << 6)));

  std::array<uint8_t, TXDESC_SIZE> desc{};
  fill_rsvd_page_txdesc(desc, static_cast<uint16_t>(payload.size()));
  std::vector<uint8_t> packet(TXDESC_SIZE + payload.size(), 0);
  std::memcpy(packet.data(), desc.data(), desc.size());
  std::memcpy(packet.data() + TXDESC_SIZE, payload.data(), payload.size());
  const int sent = _device.bulk_send_sync_ep(
      _device.first_bulk_out_ep(), packet.data(), packet.size(), 1000);
  bool ok = sent == static_cast<int>(packet.size());
  if (!ok)
    _logger->error("RTL8733B DLFW: bulk OUT failed/short ({}/{})", sent,
                   packet.size());
  if (ok) {
    ok = false;
    for (unsigned count = 1000; count != 0; --count) {
      if (r8(R8733_DWBCN0_CTRL + 2) & 0x01) {
        ok = true;
        break;
      }
      delay_us(10);
    }
    if (!ok)
      _logger->error("RTL8733B DLFW: reserved-page valid poll timed out");
  }

  w8(R8733_DWBCN0_CTRL + 1, 0); // initial HALMAC reserved boundary
  w8(R8733_DWBCN0_CTRL + 2,
     static_cast<uint8_t>(r8(R8733_DWBCN0_CTRL + 2) | 0x01));
  w8(R8733_FWHW_TXQ_CTRL + 2, old_txq2);
  w8(R8733_CR + 1, old_cr1);
  return ok;
}

bool Rtl8733bBringup::iddma(uint32_t src, uint32_t dest, uint32_t len,
                            bool first) {
  for (unsigned count = 1000; r32(R8733_DDMA_CH0CTRL) & DDMA_OWN; --count) {
    if (count == 0) {
      _logger->error("RTL8733B DLFW: DDMA channel remained owned");
      return false;
    }
  }
  uint32_t ctrl = DDMA_CHECKSUM_EN | DDMA_OWN | (len & DDMA_LENGTH_MASK);
  if (!first)
    ctrl |= DDMA_CHECKSUM_CONT;
  w32(R8733_DDMA_CH0SA, src);
  w32(R8733_DDMA_CH0DA, dest);
  w32(R8733_DDMA_CH0CTRL, ctrl);
  for (unsigned count = 1000; r32(R8733_DDMA_CH0CTRL) & DDMA_OWN; --count) {
    if (count == 0) {
      _logger->error("RTL8733B DLFW: DDMA transfer timed out");
      return false;
    }
  }
  return true;
}

bool Rtl8733bBringup::check_checksum(uint32_t dest) {
  uint8_t ctrl = r8(R8733_MCUFW_CTRL);
  const bool imem = dest < OCPBASE_DMEM;
  if (r32(R8733_DDMA_CH0CTRL) & DDMA_CHECKSUM_STATUS) {
    ctrl |= imem ? (1u << 3) : (1u << 5);
    ctrl &= static_cast<uint8_t>(~(imem ? (1u << 4) : (1u << 6)));
    w8(R8733_MCUFW_CTRL, ctrl);
    _logger->error("RTL8733B DLFW: {} checksum failed", imem ? "IMEM" : "DMEM");
    return false;
  }
  ctrl |= imem ? static_cast<uint8_t>((1u << 3) | (1u << 4))
               : static_cast<uint8_t>((1u << 5) | (1u << 6));
  w8(R8733_MCUFW_CTRL, ctrl);
  return true;
}

bool Rtl8733bBringup::dlfw_to_mem(const uint8_t *fw, uint32_t src,
                                  uint32_t dest, uint32_t size) {
  w32(R8733_DDMA_CH0CTRL, r32(R8733_DDMA_CH0CTRL) | DDMA_RESET_CHECKSUM);
  uint32_t offset = 0;
  bool first = true;
  while (offset < size) {
    const uint32_t chunk = (std::min)(FW_PACKET_SIZE, size - offset);
    if (!send_fw_page(static_cast<uint16_t>(src >> 7), fw + offset, chunk))
      return false;
    if (!iddma(OCPBASE_TXBUF + src + TXDESC_SIZE, dest + offset, chunk,
               first))
      return false;
    first = false;
    offset += chunk;
  }
  return check_checksum(dest);
}

bool Rtl8733bBringup::start_dlfw(const uint8_t *fw) {
  const uint32_t dmem = le32(fw + 36) + FW_CHECKSUM_SIZE;
  const uint32_t imem = le32(fw + 48) + FW_CHECKSUM_SIZE;
  const uint32_t emem = (fw[24] & (1u << 4))
                            ? le32(fw + 52) + FW_CHECKSUM_SIZE
                            : 0;
  w16(R8733_MCUFW_CTRL,
      static_cast<uint16_t>((r16(R8733_MCUFW_CTRL) & 0x3800) | 1));

  const uint8_t *cur = fw + FW_HEADER_SIZE;
  if (!dlfw_to_mem(cur, 0, le32(fw + 32) & ~(1u << 31), dmem))
    return false;
  cur += dmem;
  if (!dlfw_to_mem(cur, 0, le32(fw + 60) & ~(1u << 31), imem))
    return false;
  cur += imem;
  if (emem && !dlfw_to_mem(cur, 0, le32(fw + 56) & ~(1u << 31), emem))
    return false;
  return true;
}

bool Rtl8733bBringup::finish_dlfw() {
  w32(R8733_TXDMA_STATUS, 1u << 2);
  uint16_t ctrl = r16(R8733_MCUFW_CTRL);
  if ((ctrl & 0x50) != 0x50) {
    _logger->error("RTL8733B DLFW: checksum-ready bits missing (0x{:04x})",
                   ctrl);
    return false;
  }
  _checksum_ok = true;
  w16(R8733_MCUFW_CTRL, static_cast<uint16_t>((ctrl | (1u << 14)) & ~1u));
  wlan_cpu_enable(true);

  for (unsigned count = 5000; count != 0; --count) {
    ctrl = r16(R8733_MCUFW_CTRL);
    if ((ctrl & 0xc078) == 0xc078) {
      _ready_ok = true;
      _logger->info("RTL8733B DLFW: firmware ready, MCUFW_CTRL=0x{:04x}", ctrl);
      return true;
    }
    delay_us(50);
  }
  _logger->error(
      "RTL8733B DLFW: firmware did not boot "
      "(MCUFW_CTRL=0x{:04x}, DBG7=0x{:08x})",
      r16(R8733_MCUFW_CTRL), r32(R8733_FW_DBG7));
  return false;
}

bool Rtl8733bBringup::download_firmware(const uint8_t *fw, size_t size) {
  _download_attempted = false;
  _checksum_ok = false;
  _ready_ok = false;
  if (!_powered && !power_on())
    return false;
  if ((r16(R8733_MCUFW_CTRL) & 0xc078) == 0xc078) {
    _checksum_ok = true;
    _ready_ok = true;
    _logger->info("RTL8733B DLFW: firmware was already running");
    return true;
  }
  if ((r8(R8733_SECURE_CTRL) & 0x03) == 0) {
    _logger->error("RTL8733B DLFW: secure-download silicon is not implemented");
    return false;
  }
  if (!check_fw_size(fw, size))
    return false;
  _download_attempted = true;

  w8(R8733_EXT_SYS_CLK_CTRL,
     static_cast<uint8_t>(r8(R8733_EXT_SYS_CLK_CTRL) | 0x02));
  w32(R8733_EXT_SYS_FUNC_EN,
      (r32(R8733_EXT_SYS_FUNC_EN) | 0x00003000u) & 0xffffff3fu);
  wlan_cpu_enable(false);

  struct Backup {
    uint16_t reg;
    uint8_t value;
  };
  std::array<Backup, 6> backup{{
      {static_cast<uint16_t>(R8733_TXDMA_PQ_MAP + 1),
       r8(R8733_TXDMA_PQ_MAP + 1)},
      {R8733_CR, r8(R8733_CR)},
      {R8733_RQPN_CTRL_HLPQ, r8(R8733_RQPN_CTRL_HLPQ)},
      {static_cast<uint16_t>(R8733_RQPN_CTRL_HLPQ + 1),
       r8(R8733_RQPN_CTRL_HLPQ + 1)},
      {static_cast<uint16_t>(R8733_RQPN_CTRL_HLPQ + 2),
       r8(R8733_RQPN_CTRL_HLPQ + 2)},
      {R8733_BCN_CTRL, r8(R8733_BCN_CTRL)},
  }};

  w8(R8733_TXDMA_PQ_MAP + 1, 3u << 6);
  w8(R8733_CR, (1u << 0) | (1u << 2));
  w8(R8733_RQPN_CTRL_HLPQ, 0xd0);
  w8(R8733_RQPN_CTRL_HLPQ + 1, 0x00);
  w8(R8733_RQPN_CTRL_HLPQ + 2, 0x20);
  w8(R8733_RQPN_CTRL_HLPQ + 3, 0x80);
  w8(R8733_BCN_CTRL,
     static_cast<uint8_t>((r8(R8733_BCN_CTRL) & ~(1u << 3)) | (1u << 4)));
  platform_reset();

  bool ok = start_dlfw(fw);
  for (const Backup &item : backup)
    w8(item.reg, item.value);
  if (ok)
    ok = finish_dlfw();
  if (!ok) {
    w8(R8733_MCUFW_CTRL, static_cast<uint8_t>(r8(R8733_MCUFW_CTRL) & ~1u));
    wlan_cpu_enable(true);
  }
  return ok;
}

bool Rtl8733bBringup::download_default_firmware(uint8_t cut) {
  if (cut == 2) {
    _logger->info("RTL8733B DLFW: selecting cut-2 CCV NIC image ({} bytes)",
                  ccv_array_mp_8733b_fw_nic_len);
    return download_firmware(ccv_array_mp_8733b_fw_nic,
                             ccv_array_mp_8733b_fw_nic_len);
  }
  _logger->info("RTL8733B DLFW: selecting normal NIC image ({} bytes)",
                array_mp_8733b_fw_nic_len);
  return download_firmware(array_mp_8733b_fw_nic,
                           array_mp_8733b_fw_nic_len);
}

} // namespace rtl8733b
