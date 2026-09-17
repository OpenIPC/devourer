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

/* The Realtek extension with nothing of its own implemented — the shape of a
 * backend that derives from IRtlRadio but has not ported an optional member.
 * RTL8733B is the live example: it overrides SetCcaMode and inherits the
 * carrier-sense gate split. */
struct NullRtlRadio final : IRtlRadio {
  SelectedChannel ch_{};
  void Init(Action_ParsedRadioPacket, SelectedChannel c) override { ch_ = c; }
  void InitWrite(SelectedChannel c) override { ch_ = c; }
  void StartRxLoop(Action_ParsedRadioPacket) override {}
  void SetMonitorChannel(SelectedChannel c) override { ch_ = c; }
  bool send_packet(const uint8_t *, size_t) override { return false; }
  SelectedChannel GetSelectedChannel() override { return ch_; }
  void SetCcaMode(bool) override {}
};

/* A Realtek backend that HAS ported its energy reader. Proves the neutral
 * GetChannelBusy implemented once on IRtlRadio actually reaches a derived
 * class's GetRxEnergy — the whole point of putting it there rather than
 * overriding it five times. */
struct ClmRtlRadio final : IRtlRadio {
  void Init(Action_ParsedRadioPacket, SelectedChannel) override {}
  void InitWrite(SelectedChannel) override {}
  void StartRxLoop(Action_ParsedRadioPacket) override {}
  void SetMonitorChannel(SelectedChannel) override {}
  bool send_packet(const uint8_t *, size_t) override { return false; }
  SelectedChannel GetSelectedChannel() override { return {}; }
  void SetCcaMode(bool) override {}

  RxEnergy GetRxEnergy(bool) override {
    RxEnergy e;
    e.valid_clm = true;
    e.clm_ratio_pct = 42;
    e.clm_period = 500; /* 4 us ticks -> a 2000 us window */
    e.valid_nhm = true;
    e.nhm_env_ratio_pct = 7;
    return e;
  }
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
  /* The neutral frame-free reading. That this compiles against a radio with no
   * Realtek type in sight is the proof GetChannelBusy sits on IRadio. */
  const devourer::ChannelBusy busy = r->GetChannelBusy();
  check(!busy.valid && !busy.valid_busy && !busy.valid_energy,
        "GetChannelBusy default reports no reading");
  check(busy.source == devourer::BusySource::None,
        "GetChannelBusy default names no source");
  check(!r->GetAdapterCaps().busy_airtime_ok,
        "busy_airtime_ok defaults false");
  check(!r->GetAdapterCaps().busy_airtime_measured,
        "busy_airtime_measured defaults false");
  check(!r->GetAdapterCaps().rx_energy_ok, "rx_energy_ok defaults false");
  check(!r->GetFwBootStatus().supported, "GetFwBootStatus default is unsupported");
  check(!r->GetTxPowerCaps().supported, "GetTxPowerCaps default is unsupported");
  check(!r->SetAckResponder(devourer::MacAddr{}), "SetAckResponder default refuses");
  check(r->ReadTsf() == 0, "ReadTsf default is 0");
  check(!r->WriteTsf(123456789ull), "WriteTsf default refuses and reports false");
  check(!r->GetAdapterCaps().tsf_write_ok, "tsf_write_ok default is false");

  r->FastRetune(6);
  check(r->GetSelectedChannel().Channel == 6,
        "FastRetune default falls back to SetMonitorChannel");

  /* The Realtek-only members a backend may leave unported. The rule is that
   * an unsupported optional member refuses instead of answering: a caller
   * cannot tell a fabricated reading from a real one, so `false` is the only
   * honest return. The gate split is the case with out-parameters, where
   * refusing also means leaving the caller's variables alone. */
  std::unique_ptr<IRadio> rtl_owner = std::make_unique<NullRtlRadio>();
  auto *rtl = dynamic_cast<IRtlRadio *>(rtl_owner.get());
  check(rtl != nullptr, "a Realtek radio is reachable by dynamic_cast");

  check(rtl->SetXtalCap(0) == -1, "SetXtalCap default refuses");
  check(rtl->GetXtalCap() == -1, "GetXtalCap default refuses");
  const RxEnergy energy = rtl->GetRxEnergy(false);
  check(!energy.valid_fa && !energy.valid_igi && !energy.valid_nhm,
        "GetRxEnergy default reports every field invalid");
  check(!rtl->ProbeEfuseStability().supported,
        "ProbeEfuseStability default is unsupported");
  /* The RTL8733B shape: derives from IRtlRadio, ports no energy reader. The
   * inherited GetChannelBusy must report nothing rather than a zero reading —
   * this is the false positive the dynamic_cast used to produce. */
  check(!rtl->GetChannelBusy().valid,
        "a Realtek radio with no GetRxEnergy reports no channel-busy reading");

  check(!rtl->SetCcaGates(true, true), "SetCcaGates default refuses");
  bool primary = true, edcca = true; /* poison: a refusal must not write */
  check(!rtl->GetCcaGates(primary, edcca), "GetCcaGates default refuses");
  check(primary && edcca, "GetCcaGates leaves its out-params alone when it refuses");

  /* A Realtek backend that HAS ported GetRxEnergy: the shared IRtlRadio
   * implementation must translate its CLM/NHM-env into the neutral reading. */
  std::unique_ptr<IRadio> clm_owner = std::make_unique<ClmRtlRadio>();
  const devourer::ChannelBusy clm = clm_owner->GetChannelBusy();
  check(clm.valid, "a Realtek radio with CLM reports a reading");
  check(clm.source == devourer::BusySource::Clm, "the reading names CLM");
  check(clm.valid_busy && clm.busy_pct == 42, "busy airtime passes through");
  check(clm.window_us == 2000, "window is the CLM period in 4 us ticks");
  check(clm.valid_energy && clm.energy_pct == 7, "NHM-env passes through");

  if (fails) return 1;
  std::puts("radio_iface: PASS");
  return 0;
}
