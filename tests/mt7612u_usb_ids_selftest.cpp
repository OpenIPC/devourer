/* Headless guard for the MediaTek USB-id gate (src/mt7612u/Mt7612uUsbIds.h,
 * used by WiFiDriver::CreateRadio before the Realtek SYS_CFG2 read).
 *
 * The gate exists because a MediaTek adapter stalls read_chip_id(), which
 * returns 0, which matches no Realtek chip-id, which lands the adapter in the
 * unconditional Jaguar1 fallback. Refusing it depends on one property: the
 * MediaTek vid:pid set must never claim a device some Realtek backend owns.
 * That is what this test pins — a future id addition on either side that breaks
 * it fails here rather than on a user's desk.
 *
 * It also records WHY the gate is vid:pid and not vid: several vendor ids ship
 * both silicon families. That half is reported, not asserted, so removing an
 * OEM entry cannot fail the build — only a genuine pair collision can. */
#include "kestrel/KestrelUsbIds.h"
#include "mt7612u/Mt7612uUsbIds.h"
#include "rtl8733b/Rtl8733bUsbIds.h"

#include <cstdio>

namespace {

int fails = 0;
void check(bool ok, const char *what) {
  if (!ok) {
    std::fprintf(stderr, "mt7612u_usb_ids: FAIL %s\n", what);
    fails++;
  }
}

} // namespace

int main() {
  /* Every tabled MediaTek id is claimed. A table entry the lookup misses would
   * silently reopen the Jaguar1 misroute for that adapter. */
  for (const mt7612u::UsbId &id : mt7612u::kUsbIds) {
    if (!mt7612u::is_usb_id(id.vid, id.pid)) {
      std::fprintf(stderr, "mt7612u_usb_ids: FAIL %04x:%04x tabled but not claimed\n",
                   id.vid, id.pid);
      fails++;
    }
  }

  /* THE load-bearing property: the MediaTek gate must not claim any device a
   * Realtek backend owns. It runs before the SYS_CFG2 read, so a false claim
   * here is unrecoverable — the Realtek adapter is refused outright. */
  for (const rtl8733b::UsbId &id : rtl8733b::kUsbIds) {
    if (mt7612u::is_usb_id(id.vid, id.pid)) {
      std::fprintf(stderr,
                   "mt7612u_usb_ids: FAIL %04x:%04x is RTL8733B but the "
                   "MediaTek gate claims it\n",
                   id.vid, id.pid);
      fails++;
    }
  }
  for (const auto &id : kestrel::kKestrelUsbIds) {
    if (mt7612u::is_usb_id(id.vid, id.pid)) {
      std::fprintf(stderr,
                   "mt7612u_usb_ids: FAIL %04x:%04x is Kestrel but the "
                   "MediaTek gate claims it\n",
                   id.vid, id.pid);
      fails++;
    }
  }

  /* Negative control: the most common Realtek adapter devourer serves, and the
   * one the Jaguar1 fallback is FOR. If the gate ever claimed this, every
   * RTL8812AU would stop working — so a test that only checked tabled ids
   * passing would not be enough. 0bda:8812 is dispatched by chip-id, not by any
   * table above, so it is not covered by the loops. */
  check(!mt7612u::is_usb_id(0x0bda, 0x8812), "RTL8812AU is not claimed");
  check(!mt7612u::is_usb_id(0x0bda, 0x8813), "RTL8814AU is not claimed");

  /* A MediaTek id one digit off must NOT be claimed: the gate is an exact pair
   * match, not a prefix or a vendor test. */
  check(mt7612u::is_usb_id(0x0e8d, 0x7612), "the verified Alfa AWUS036ACM is claimed");
  check(!mt7612u::is_usb_id(0x0e8d, 0x7613), "an untabled MediaTek pid is not claimed");
  check(!mt7612u::is_usb_id(0x0bda, 0x7612), "the pid alone does not claim");

  /* Reported, not asserted — the rationale for pair-matching. Shared vendor ids
   * are why a "not a Realtek vendor id" gate would refuse working adapters. */
  int shared_vids = 0;
  for (const mt7612u::UsbId &m : mt7612u::kUsbIds) {
    bool shared = false;
    for (const rtl8733b::UsbId &r : rtl8733b::kUsbIds)
      shared = shared || r.vid == m.vid;
    for (const auto &k : kestrel::kKestrelUsbIds)
      shared = shared || k.vid == m.vid;
    if (shared)
      shared_vids++;
  }
  std::printf("mt7612u_usb_ids: %d MediaTek ids, %d share a vendor id with a "
              "Realtek table (why the gate matches pairs, not vendors)\n",
              (int)(sizeof(mt7612u::kUsbIds) / sizeof(mt7612u::kUsbIds[0])),
              shared_vids);

  if (fails) {
    std::fprintf(stderr, "mt7612u_usb_ids: %d failure(s)\n", fails);
    return 1;
  }
  std::printf("mt7612u_usb_ids: OK\n");
  return 0;
}
