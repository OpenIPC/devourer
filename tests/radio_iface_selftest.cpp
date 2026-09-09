/* Headless guard for the radio contract (src/IRadio.h, src/IRtlRadio.h).
 *
 * NullRadio implements only IRadio's pure-virtual core. That it compiles is the
 * proof that the vendor-neutral contract needs no Realtek type; the runtime
 * checks pin the not-ported defaults a non-Realtek backend inherits and the
 * dynamic_cast convention callers use to reach the Realtek-only members. */
#include "IRadio.h"
#include "IRtlRadio.h"

#include <cstdio>
#include <memory>

namespace {

struct NullRadio final : IRadio {
  SelectedChannel ch_{};
  void Init(Action_ParsedRadioPacket, SelectedChannel c) override { ch_ = c; }
  void InitWrite(SelectedChannel c) override { ch_ = c; }
  void StartRxLoop(Action_ParsedRadioPacket) override {}
  void SetMonitorChannel(SelectedChannel c) override { ch_ = c; }
  bool send_packet(const uint8_t *, size_t) override { return false; }
  SelectedChannel GetSelectedChannel() override { return ch_; }
  void SetCcaMode(bool) override {}
};

int fails = 0;
void check(bool ok, const char *what) {
  if (!ok) {
    std::fprintf(stderr, "radio_iface: FAIL %s\n", what);
    fails++;
  }
}

} // namespace

int main() {
  std::unique_ptr<IRadio> r = std::make_unique<NullRadio>();

  check(dynamic_cast<IRtlRadio *>(r.get()) == nullptr,
        "a neutral radio is not an IRtlRadio");
  check(!r->GetAdapterCaps().supported, "GetAdapterCaps default is unsupported");
  check(!r->GetRxQuality().valid, "GetRxQuality default is invalid");
  check(!r->GetFwBootStatus().supported, "GetFwBootStatus default is unsupported");
  check(!r->GetTxPowerCaps().supported, "GetTxPowerCaps default is unsupported");
  check(!r->SetAckResponder(devourer::MacAddr{}), "SetAckResponder default refuses");
  check(r->ReadTsf() == 0, "ReadTsf default is 0");

  r->FastRetune(6);
  check(r->GetSelectedChannel().Channel == 6,
        "FastRetune default falls back to SetMonitorChannel");

  if (fails) return 1;
  std::puts("radio_iface: PASS");
  return 0;
}
