/* Headless guard for the ACK-responder recipe (src/AckResponder.h): the arm,
 * the gate-only disarm staying gate-only, and the retarget that the RTL8733B
 * disarm is built on.
 *
 * Why retarget() exists. On the RTL8733B the net_type field is inert and the
 * engine matches MACID; the measured truth table is owned by AdapterCaps.h.
 * The identity is therefore the only thing a disarm can move, and
 * Rtl8733bDevice::disarm_ack_responder moves it back to the adapter's own MAC.
 *
 * The restore address is deliberately NOT zero: many Realtek MAC TX paths
 * refuse to schedule a frame when the MAC ID is zero (src/jaguar1/HalModule.cpp),
 * and zero would not remove the match anyway — 00:00:00:00:00:00 has the I/G
 * bit clear, so is_unicast() accepts it. Both properties are pinned below.
 *
 * This test does NOT cover silicon behaviour or the device-layer composition
 * in Rtl8733bDevice (nothing here instantiates it). Those are covered on
 * hardware by ack_txreport_matrix.sh's RTL8733B-only `disarmed` phase; its
 * legacy `off` phase is still only a never-armed control. */
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>

#include "AckResponder.h"
#include "RtlAdapter.h"

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

namespace {

/* A byte-addressed register file. Only the fields the recipe touches matter,
 * but modelling the whole space keeps the test honest about WHERE it writes:
 * an implementation that hit the wrong offset would read back zero here rather
 * than quietly passing. */
class FakeRegs final : public devourer::ITransport {
public:
  std::map<uint16_t, uint8_t> mem;
  bool fail_writes = false;
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
    if (fail_writes) return false;
    mem[a] = v;
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

constexpr uint16_t kNetType = 0x0102;
constexpr uint16_t kMacId = 0x0610;

}  // namespace

int main() {
  auto logger = std::make_shared<Logger>();
  const uint8_t mac[6] = {0x02, 0x12, 0x34, 0x56, 0x78, 0x9a};
  /* Stands in for the adapter's own EFUSE MAC — what MAC bring-up programmed
   * and what the disarm restores. Distinct from the responder MAC. */
  const uint8_t own[6] = {0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0x01};

  {
    /* Arm: MACID programmed, gate open, verify() agrees. The raw register
     * values are asserted independently of the readback helpers, so a
     * self-consistent endianness change in both would still be caught. */
    auto regs = std::make_shared<FakeRegs>();
    RtlAdapter dev(regs, logger);
    CHECK(devourer::ack::enable(dev, mac));
    CHECK(regs->read32(kMacId) == 0x56341202u);
    CHECK(regs->read16(kMacId + 4) == 0x9a78u);
    CHECK((regs->read8(kNetType) & 0x03u) == 0x03u);
    CHECK(devourer::ack::verify(dev, mac));
    CHECK(devourer::ack::macid_is(dev, mac));
    CHECK(!devourer::ack::is_disabled(dev));
  }
  {
    /* One packing, one register map: verify() and macid_is() must agree with
     * the bytes actually in the file. This is the property the helpers exist
     * for — a width or endianness fix that reached only one of them would
     * break here rather than silently diverge (they used to be four copies). */
    auto regs = std::make_shared<FakeRegs>();
    RtlAdapter dev(regs, logger);
    CHECK(devourer::ack::enable(dev, mac));
    CHECK(regs->read32(kMacId) == devourer::ack::macid_lo(mac));
    CHECK(regs->read16(kMacId + 4) == devourer::ack::macid_hi(mac));
    CHECK(devourer::ack::macid_is(dev, mac));
    CHECK(!devourer::ack::macid_is(dev, own));
  }
  {
    /* The portable disarm is gate-only and MUST stay that way: it is what the
     * other generations use, and moving their MACID is a change none of them
     * has a bench cell for. */
    auto regs = std::make_shared<FakeRegs>();
    RtlAdapter dev(regs, logger);
    CHECK(devourer::ack::enable(dev, mac));
    CHECK(devourer::ack::disable_verified(dev));
    CHECK((regs->read8(kNetType) & 0x03u) == 0);
    CHECK(regs->read32(kMacId) == 0x56341202u); /* identity left standing */
    CHECK(devourer::ack::macid_is(dev, mac));
    CHECK(!devourer::ack::verify(dev, mac));    /* gate shut, so not armed */
  }
  {
    /* The composed disarm the RTL8733B uses: gate closed, then the identity
     * moved off the responder address and onto the adapter's own. */
    auto regs = std::make_shared<FakeRegs>();
    RtlAdapter dev(regs, logger);
    CHECK(devourer::ack::enable(dev, mac));
    CHECK(devourer::ack::disable_verified(dev));
    CHECK(devourer::ack::retarget(dev, own));
    CHECK(devourer::ack::macid_is(dev, own));
    CHECK(!devourer::ack::macid_is(dev, mac)); /* no longer the responder */
    CHECK((regs->read8(kNetType) & 0x03u) == 0);
    CHECK(regs->read32(kMacId) == 0xccbbaa02u);
    CHECK(regs->read16(kMacId + 4) == 0x01ddu);
    /* NOT zero — the T1 state that stops MAC TX scheduling. */
    CHECK(regs->read32(kMacId) != 0 || regs->read16(kMacId + 4) != 0);
  }
  {
    /* retarget() does not touch the gate: composition is the caller's, so a
     * die that wants the identity moved without reopening anything gets that. */
    auto regs = std::make_shared<FakeRegs>();
    RtlAdapter dev(regs, logger);
    CHECK(devourer::ack::enable(dev, mac));
    const uint8_t before = regs->read8(kNetType);
    CHECK(devourer::ack::retarget(dev, own));
    CHECK(regs->read8(kNetType) == before);
  }
  {
    /* Re-arm after a composed disarm restores the responder identity: the live
     * toggle loop must work in both directions without a re-init. */
    auto regs = std::make_shared<FakeRegs>();
    RtlAdapter dev(regs, logger);
    CHECK(devourer::ack::enable(dev, mac));
    CHECK(devourer::ack::disable_verified(dev));
    CHECK(devourer::ack::retarget(dev, own));
    CHECK(devourer::ack::enable(dev, mac));
    CHECK(devourer::ack::verify(dev, mac));
  }
  {
    /* A failing transport must not report a successful retarget, and must have
     * ATTEMPTED both MACID halves — the reason retarget() sequences its writes
     * into locals instead of short-circuiting on `&&`. Note this counts
     * operations without identifying addresses, and its strength depends on
     * FakeRegs::write8 returning BEFORE it stores (so write16/write32
     * short-circuit to one call each): making the fake attempt both bytes
     * would turn this into a tautology. */
    auto regs = std::make_shared<FakeRegs>();
    RtlAdapter dev(regs, logger);
    CHECK(devourer::ack::enable(dev, mac));
    regs->fail_writes = true;
    const int before = regs->write_calls;
    CHECK(!devourer::ack::retarget(dev, own));
    CHECK(regs->write_calls - before == 2); /* both halves attempted */
    regs->fail_writes = false;
  }
  {
    /* Zero is a UNICAST address by the I/G bit, which is exactly why it is not
     * a safe "no match" value: is_unicast() accepts it, so a peer could
     * solicit it. Pins the reasoning behind restoring a real MAC. */
    const uint8_t zero[6] = {0, 0, 0, 0, 0, 0};
    const uint8_t group[6] = {0x57, 0x42, 0x00, 0x00, 0x13, 0x00};
    CHECK(devourer::ack::is_unicast(mac));
    CHECK(devourer::ack::is_unicast(zero));
    CHECK(!devourer::ack::is_unicast(group));
  }

  if (failures) {
    std::fprintf(stderr, "ack_responder_selftest: %d failure(s)\n", failures);
    return 1;
  }
  std::printf("ack_responder_selftest: OK\n");
  return 0;
}
