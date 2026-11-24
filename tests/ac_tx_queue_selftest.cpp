/* Headless guard for src/AcTxQueue.h, the register arithmetic behind
 * DeviceConfig::Tx::be_fast_retry and ac_lifetime_us:
 *   - edca_fast_retry: TXOP limit cleared, CWmax pulled down to CWmin,
 *     AIFS and CWmin kept, on the vendor BE words of every generation that
 *     applies it;
 *   - pkt_life_time_reg: 256 us units rounded up, both halves equal,
 *     clamped to the 16-bit field;
 *   - apply_ac_tx_queue through an RtlAdapter on a fake transport (the
 *     ack_responder_selftest shape): nothing touched with both options off,
 *     REG_LIFETIME_EN's other bits preserved, a failed write reported;
 *   - dot11_is_data on the Frame Control type bits.
 * Prints the failing case and exits nonzero. */
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>

#include "AcTxQueue.h"
#include "RtlAdapter.h"

static int g_fail = 0;

#define CHECK(cond, ...)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      ++g_fail;                                                                \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__);                         \
      std::printf(__VA_ARGS__);                                                \
      std::printf("\n");                                                       \
    }                                                                          \
  } while (0)

namespace {

/* A byte-addressed register file, as in tests/ack_responder_selftest.cpp. */
class FakeRegs final : public devourer::ITransport {
public:
  std::map<uint16_t, uint8_t> mem;
  bool fail_writes = false;
  bool fail_writes_after_store = false;
  bool throw_writes = false;
  int fail_first_writes = 0; /* the first N write8 calls fail, later ones succeed */
  int write_calls = 0;

  bool is_usb() const override { return true; }
  uint8_t read8(uint16_t a) override { return mem.count(a) ? mem[a] : 0; }
  uint16_t read16(uint16_t a) override {
    return static_cast<uint16_t>(read8(a) | (read8(a + 1) << 8));
  }
  uint32_t read32(uint16_t a) override {
    return static_cast<uint32_t>(read16(a)) |
           (static_cast<uint32_t>(read16(a + 2)) << 16);
  }
  bool write8(uint16_t a, uint8_t v) override {
    ++write_calls;
    if (throw_writes) throw std::runtime_error("injected write failure");
    if (fail_writes || write_calls <= fail_first_writes) return false;
    mem[a] = v;
    if (fail_writes_after_store) return false;
    return true;
  }
  bool write16(uint16_t a, uint16_t v) override {
    return write8(a, static_cast<uint8_t>(v)) &&
           write8(a + 1, static_cast<uint8_t>(v >> 8));
  }
  bool write32(uint16_t a, uint32_t v) override {
    return write16(a, static_cast<uint16_t>(v)) &&
           write16(a + 2, static_cast<uint16_t>(v >> 16));
  }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return true; }
  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override { return true; }
  int tx_sync(uint8_t, uint8_t *, size_t len, int) override {
    return static_cast<int>(len);
  }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}
};

} // namespace

static void test_edca() {
  using devourer::edca_fast_retry;
  /* RTL8733B / Jaguar1 / Jaguar3 vendor BE: TXOP 0x5e, CW 15..1023, AIFS
   * 0x2b. Jaguar2's power-on BE (TXOP already 0). Jaguar3 5 MHz BE. */
  CHECK(edca_fast_retry(0x005ea42bu) == 0x0000442bu, "8733B/J3 BE -> 0x%08x",
        edca_fast_retry(0x005ea42bu));
  CHECK(edca_fast_retry(0x0000a42bu) == 0x0000442bu, "J2 BE -> 0x%08x",
        edca_fast_retry(0x0000a42bu));
  CHECK(edca_fast_retry(0x005ea48eu) == 0x0000448eu, "J3 5M BE -> 0x%08x",
        edca_fast_retry(0x005ea48eu));
  /* Every word: TXOP 0, ECWmax == ECWmin, AIFS and ECWmin unchanged. */
  for (uint32_t w : {0x00000000u, 0xffffffffu, 0x002fa226u, 0x0000a44fu,
                     0x07ff3f01u}) {
    const uint32_t f = edca_fast_retry(w);
    CHECK((f >> 16) == 0, "0x%08x: TXOP left 0x%x", w, f >> 16);
    CHECK(((f >> 12) & 0xf) == ((w >> 8) & 0xf), "0x%08x: ECWmax 0x%x", w,
          (f >> 12) & 0xf);
    CHECK((f & 0xfff) == (w & 0xfff), "0x%08x: AIFS/ECWmin changed", w);
  }
}

static void test_lifetime() {
  using devourer::pkt_life_time_reg;
  CHECK(pkt_life_time_reg(30000) == 0x00760076u, "30 ms -> 0x%08x",
        pkt_life_time_reg(30000)); /* 117.19 units, rounded up */
  CHECK(pkt_life_time_reg(256) == 0x00010001u, "256 us -> 0x%08x",
        pkt_life_time_reg(256));
  CHECK(pkt_life_time_reg(1) == 0x00010001u, "1 us rounds up to one unit");
  CHECK(pkt_life_time_reg(0) == 0, "0 us -> 0x%08x", pkt_life_time_reg(0));
  CHECK(pkt_life_time_reg(0xffffu * 256) == 0xffffffffu, "max field");
  CHECK(pkt_life_time_reg(0xffffffffu) == 0xffffffffu, "clamped, no wrap");
}

static void test_apply() {
  auto logger = std::make_shared<Logger>();
  {
    auto regs = std::make_shared<FakeRegs>();
    RtlAdapter dev(regs, logger);
    devourer::DeviceConfig::Tx off;
    off.data_on_be = true; /* the endpoint/QSEL knob alone touches no register */
    CHECK(devourer::apply_ac_tx_queue(dev, off), "both off: reported failure");
    CHECK(regs->write_calls == 0, "both off: %d register writes",
          regs->write_calls);
  }
  {
    auto regs = std::make_shared<FakeRegs>();
    RtlAdapter dev(regs, logger);
    regs->write32(devourer::kRegEdcaBeParam, 0x005ea42bu);
    regs->write8(devourer::kRegLifetimeEn, 0x30); /* bits 4, 5: others' */
    devourer::DeviceConfig::Tx on;
    on.be_fast_retry = true;
    on.ac_lifetime_us = 60000;
    CHECK(devourer::apply_ac_tx_queue(dev, on), "on: reported failure");
    CHECK(regs->read32(devourer::kRegEdcaBeParam) == 0x0000442bu, "BE 0x%08x",
          regs->read32(devourer::kRegEdcaBeParam));
    CHECK(regs->read8(devourer::kRegLifetimeEn) == 0x3f, "LIFETIME_EN 0x%02x",
          regs->read8(devourer::kRegLifetimeEn));
    CHECK(regs->read32(devourer::kRegPktLifeTime) == 0x00eb00ebu,
          "PKT_LIFE_TIME 0x%08x", regs->read32(devourer::kRegPktLifeTime));
    /* Lifetime alone leaves the EDCA word alone. */
    regs->write32(devourer::kRegEdcaBeParam, 0x005ea42bu);
    devourer::DeviceConfig::Tx lt;
    lt.ac_lifetime_us = 1000;
    CHECK(devourer::apply_ac_tx_queue(dev, lt), "lifetime: reported failure");
    CHECK(regs->read32(devourer::kRegEdcaBeParam) == 0x005ea42bu,
          "lifetime alone moved BE to 0x%08x",
          regs->read32(devourer::kRegEdcaBeParam));
  }
  {
    auto regs = std::make_shared<FakeRegs>();
    RtlAdapter dev(regs, logger);
    regs->fail_writes = true;
    devourer::DeviceConfig::Tx on;
    on.be_fast_retry = true;
    CHECK(!devourer::apply_ac_tx_queue(dev, on), "failed write not reported");
  }
  {
    /* The lifetime write fails, a later write would succeed: the enable must
     * not be written, or the AC queues would age on a stale lifetime. */
    auto regs = std::make_shared<FakeRegs>();
    RtlAdapter dev(regs, logger);
    regs->mem[devourer::kRegLifetimeEn] = 0x30;
    regs->fail_first_writes = 1;
    devourer::DeviceConfig::Tx on;
    on.ac_lifetime_us = 1000;
    CHECK(!devourer::apply_ac_tx_queue(dev, on),
          "failed lifetime write not reported");
    CHECK(regs->read8(devourer::kRegLifetimeEn) == 0x30,
          "enable written after a failed lifetime write: 0x%02x",
          regs->read8(devourer::kRegLifetimeEn));
  }
}

static void test_is_data() {
  CHECK(devourer::dot11_is_data(0x08), "Data");
  CHECK(devourer::dot11_is_data(0x88), "QoS Data");
  CHECK(!devourer::dot11_is_data(0x80), "Beacon is management");
  CHECK(!devourer::dot11_is_data(0xd4), "ACK is control");
}

int main() {
  test_edca();
  test_lifetime();
  test_apply();
  test_is_data();
  if (g_fail) {
    std::printf("ac_tx_queue_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("ac_tx_queue_selftest: OK\n");
  return 0;
}
