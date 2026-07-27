/* Headless guard for the TX quiesce seam (IRtlTransport::quiesce_tx,
 * RtlAdapter::quiesce_tx).
 *
 * What this covers: that the quiesce call reaches the transport through the
 * adapter, that a transport which has been quiesced refuses further sends and
 * the refusal is visible to the caller as `send_packet() == false`, that the
 * call is idempotent, and that the interface default is inert (the shape the
 * synchronous transports rely on).
 *
 * What it does NOT cover: UsbTransport's implementation — cancelling live
 * URBs, draining completions and owning the payload buffer. That needs a real
 * libusb handle, so it is validated on hardware under ASan (the max-duty
 * gap-0 teardown run), not here. This test exists so the wiring around it
 * cannot be quietly removed. */
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

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

/* Models the asynchronous half of a USB transport: a send is accepted while
 * running and refused once quiesced, and quiesce is what drains. */
class FakeAsyncTransport final : public devourer::IRtlTransport {
public:
  int accepted = 0;
  int refused = 0;
  int quiesce_calls = 0;
  int inflight = 0;

  bool is_usb() const override { return true; }
  uint8_t read8(uint16_t) override { return 0; }
  uint16_t read16(uint16_t) override { return 0; }
  uint32_t read32(uint16_t) override { return 0; }
  bool write8(uint16_t, uint8_t) override { return true; }
  bool write16(uint16_t, uint16_t) override { return true; }
  bool write32(uint16_t, uint32_t) override { return true; }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return true; }

  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override {
    if (_shutdown) {
      ++refused;
      return false;
    }
    ++accepted;
    ++inflight;
    return true;
  }
  int tx_sync(uint8_t, uint8_t *, size_t len, int) override {
    return static_cast<int>(len);
  }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}

  void quiesce_tx() override {
    ++quiesce_calls;
    if (_shutdown)
      return; /* idempotent: the real one latches a flag the same way */
    _shutdown = true;
    inflight = 0;
  }

private:
  bool _shutdown = false;
};

/* A transport whose TX is synchronous has nothing outstanding, so it inherits
 * the interface's no-op — the property the HalMAC generations depend on. */
class SyncOnlyTransport final : public devourer::IRtlTransport {
public:
  bool is_usb() const override { return false; }
  uint8_t read8(uint16_t) override { return 0; }
  uint16_t read16(uint16_t) override { return 0; }
  uint32_t read32(uint16_t) override { return 0; }
  bool write8(uint16_t, uint8_t) override { return true; }
  bool write16(uint16_t, uint16_t) override { return true; }
  bool write32(uint16_t, uint32_t) override { return true; }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return true; }
  bool tx_async(uint8_t, uint8_t *, size_t len, unsigned) override {
    return tx_sync(0, nullptr, len, 0) > 0;
  }
  int tx_sync(uint8_t, uint8_t *, size_t len, int) override {
    return static_cast<int>(len);
  }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}
};

} /* namespace */

int main() {
  auto logger = std::make_shared<Logger>();
  std::vector<uint8_t> frame(64, 0xA5);

  {
    auto fake = std::make_shared<FakeAsyncTransport>();
    RtlAdapter adapter(fake, logger);

    CHECK(adapter.send_packet(frame.data(), frame.size()));
    CHECK(adapter.send_packet(frame.data(), frame.size()));
    CHECK(fake->accepted == 2);
    CHECK(fake->inflight == 2);

    adapter.quiesce_tx();
    CHECK(fake->quiesce_calls == 1);
    CHECK(fake->inflight == 0);

    /* Post-quiesce sends are refused, not queued: teardown has begun and
     * nothing may re-enter the bus layer. */
    CHECK(!adapter.send_packet(frame.data(), frame.size()));
    CHECK(fake->refused == 1);
    CHECK(fake->accepted == 2);

    /* Stop() then the device destructor both quiesce — the second must be a
     * no-op rather than a second cancel/drain pass. */
    adapter.quiesce_tx();
    CHECK(fake->quiesce_calls == 2);
    CHECK(fake->inflight == 0);
  }

  {
    auto sync = std::make_shared<SyncOnlyTransport>();
    RtlAdapter adapter(sync, logger);
    CHECK(adapter.send_packet(frame.data(), frame.size()));
    adapter.quiesce_tx(); /* default no-op — must not disturb the transport */
    CHECK(adapter.send_packet(frame.data(), frame.size()));
  }

  if (failures != 0) {
    std::fprintf(stderr, "tx_quiesce_selftest: %d failure(s)\n", failures);
    return 1;
  }
  std::printf("tx_quiesce_selftest: OK\n");
  return 0;
}
