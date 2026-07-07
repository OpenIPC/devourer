/* Selftest for the AdapterHealth classifier (src/AdapterHealth.h) — pure
 * logic, no hardware. The cases encode the verdicts measured on real units
 * during the #205 investigation: a dying 8812AU (stochastic EFUSE garbage +
 * 8051 never booting) vs its healthy twin, plus the soft/inconclusive edges
 * (RF-quiet room, blank dev-board efuse, transient FW hiccup). */
#include "AdapterHealth.h"

#include <cstdio>

using namespace devourer;

static int failures = 0;

static void expect(bool cond, const char *what) {
  if (!cond) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    failures++;
  }
}

static AdapterHealthInput healthy_base() {
  AdapterHealthInput in;
  in.init_completed = true;
  in.efuse.supported = true;
  in.efuse.reads = 4;
  in.efuse.eeprom_id = kRtlEepromId;
  in.fw.supported = true;
  in.fw.attempted = true;
  in.fw.checksum_ok = true;
  in.fw.ready_ok = true;
  in.rx_checked = true;
  in.rx_frames_ok = 1000;
  return in;
}

int main() {
  uint32_t r = 0;

  /* Healthy twin: everything clean, frames heard. */
  {
    auto in = healthy_base();
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Healthy,
           "clean unit → Healthy");
    expect(r == 0, "clean unit → no reasons");
  }

  /* The dying unit, exactly as measured: unstable EFUSE + FW never ready +
   * deaf against a vouched flood. */
  {
    auto in = healthy_base();
    in.efuse.mismatched_reads = 3;
    in.efuse.invalid_id_reads = 4;
    in.efuse.eeprom_id = 0x1029;
    in.fw.checksum_ok = false;
    in.fw.ready_ok = false;
    in.rx_frames_ok = 0;
    in.rx_traffic_expected = true;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Failing,
           "dying unit → Failing");
    expect((r & kAdapterEfuseUnstable) != 0, "dying unit flags EfuseUnstable");
    expect((r & kAdapterFwBootFailed) != 0, "dying unit flags FwBootFailed");
    expect((r & kAdapterRxDeafToTraffic) != 0, "dying unit flags DeafToTraffic");
  }

  /* EFUSE instability ALONE is conclusive (healthy silicon never varies). */
  {
    auto in = healthy_base();
    in.efuse.mismatched_reads = 1;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Failing,
           "any cross-read mismatch → Failing");
  }

  /* Stable-but-corrupt ID + FW fail = two independent subsystems → Failing.
   * (The warm phase of the dying unit: ID settles wrong, MCU still dead.) */
  {
    auto in = healthy_base();
    in.efuse.invalid_id_reads = 4;
    in.efuse.eeprom_id = 0x8100;
    in.fw.ready_ok = false;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Failing,
           "corrupt-id + fw-dead → Failing");
  }

  /* Stable corrupt ID alone → Suspect (one anomaly; re-run). */
  {
    auto in = healthy_base();
    in.efuse.invalid_id_reads = 4;
    in.efuse.eeprom_id = 0x8100;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Suspect,
           "corrupt-id alone → Suspect");
  }

  /* Blank map (all-0xFF autoload fail) → Suspect, flagged as blank. */
  {
    auto in = healthy_base();
    in.efuse.invalid_id_reads = 4;
    in.efuse.eeprom_id = 0xFFFF;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Suspect,
           "blank efuse → Suspect");
    expect((r & kAdapterEfuseBlank) != 0, "blank efuse flags EfuseBlank");
    expect((r & kAdapterEfuseIdInvalid) == 0,
           "blank efuse is not also flagged corrupt-id");
  }

  /* FW hiccup alone → Suspect (could be transient warm state). */
  {
    auto in = healthy_base();
    in.fw.ready_ok = false;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Suspect,
           "fw-fail alone → Suspect");
  }

  /* Heard nothing in an unvouched (possibly RF-quiet) room → Suspect. */
  {
    auto in = healthy_base();
    in.rx_frames_ok = 0;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Suspect,
           "silent unvouched → Suspect");
    expect((r & kAdapterRxSilent) != 0, "silent unvouched flags RxSilent");
  }

  /* Same silence with a vouched traffic source → Failing. */
  {
    auto in = healthy_base();
    in.rx_frames_ok = 0;
    in.rx_traffic_expected = true;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Failing,
           "deaf to vouched traffic → Failing");
  }

  /* Bring-up abort short-circuits to Failing. */
  {
    AdapterHealthInput in;
    in.init_completed = false;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Failing,
           "init abort → Failing");
    expect((r & kAdapterInitFailed) != 0, "init abort flags InitFailed");
  }

  /* Fatal-DLFW generations (Jaguar2/3): the init abort carries the fw stage
   * that died — checksum passed, MCU never booted. */
  {
    AdapterHealthInput in;
    in.init_completed = false;
    in.fw.supported = true;
    in.fw.attempted = true;
    in.fw.checksum_ok = true;
    in.fw.ready_ok = false;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Failing,
           "fatal fw abort → Failing");
    expect((r & kAdapterInitFailed) != 0 && (r & kAdapterFwBootFailed) != 0,
           "fatal fw abort flags InitFailed + FwBootFailed");
  }

  /* Nothing checked (e.g. probes unsupported, rx skipped) → Unknown. */
  {
    AdapterHealthInput in;
    in.init_completed = true;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Unknown,
           "no checks → Unknown");
  }

  /* Corrupt frames alone never grade the verdict (distant ambient traffic
   * legitimately decodes mostly-corrupt). */
  {
    auto in = healthy_base();
    in.rx_frames_ok = 3;
    in.rx_frames_crc = 5000;
    expect(ClassifyAdapterHealth(in, r) == AdapterVerdict::Healthy,
           "high corrupt ratio with decodes → still Healthy");
  }

  /* ProbeEfuseStabilityImpl: mismatch detection + first-diff offset. */
  {
    int pass = 0;
    auto read_map = [&](uint8_t *buf) {
      buf[0] = 0x29;
      buf[1] = 0x81;
      buf[2] = (pass++ == 2) ? 0xAA : 0x55; /* corrupt 3rd read at offset 2 */
      buf[3] = 0x00;
      return true;
    };
    auto st = ProbeEfuseStabilityImpl(read_map, 4, 4);
    expect(st.supported && st.reads == 4, "impl runs all reads");
    expect(st.mismatched_reads == 1, "impl counts the one corrupt read");
    expect(st.first_mismatch_off == 2, "impl records first mismatch offset");
    expect(st.invalid_id_reads == 0 && st.eeprom_id == kRtlEepromId,
           "impl parses the LE eeprom id");
  }

  if (failures == 0)
    std::printf("adapter_health_selftest: all OK\n");
  return failures == 0 ? 0 : 1;
}
