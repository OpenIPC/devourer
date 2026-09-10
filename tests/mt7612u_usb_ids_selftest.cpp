/* Headless guard for the MediaTek USB-id gate (src/mt7612u/Mt7612uUsbIds.h,
 * used by WiFiDriver::CreateRadio before the Realtek SYS_CFG2 read).
 *
 * The gate exists because a MediaTek adapter stalls read_chip_id(), which fails,
 * which used to leave the adapter in the unconditional Jaguar1 fallback.
 * Refusing it depends on one property: the MediaTek vid:pid set must never claim
 * a device a Realtek backend owns. That is what this pins — a future id addition
 * on either side that breaks it fails here rather than on a user's desk.
 *
 * The cross-check MUST cover Jaguar. Jaguar1/2/3 dispatch by chip-id and have no
 * id table in devourer, so a check limited to KestrelUsbIds.h and
 * Rtl8733bUsbIds.h sees only 27 of the 91 ids devourer can serve — and every
 * vendor id where the two silicon families interleave lives in the 64 it would
 * miss. An earlier version of this test did exactly that, and adding
 * 0b05:17d2 (ASUS USB-AC56, an RTL8812AU) to the MediaTek table passed it while
 * refusing a real Jaguar1 adapter. Hence kJaguarWitness below.
 *
 * It also records WHY the gate is vid:pid and not vid. That half is reported,
 * not asserted, so removing an OEM entry cannot fail the build — only a genuine
 * pair collision can. */
#include "kestrel/KestrelUsbIds.h"
#include "mt7612u/Mt7612uUsbIds.h"
#include "rtl8733b/Rtl8733bUsbIds.h"

#include <cstdint>
#include <cstdio>

namespace {

int fails = 0;
void check(bool ok, const char *what) {
  if (!ok) {
    std::fprintf(stderr, "mt7612u_usb_ids: FAIL %s\n", what);
    fails++;
  }
}

/* Every USB id the Jaguar1/2/3 vendor driver claims, from
 * reference/rtl8812au/os_dep/linux/usb_intf.c (the tree devourer's Jaguar port
 * came from). Embedded rather than read from that submodule, because it is not
 * fetched in a default clone. Regenerate with:
 *
 *   grep -oE 'USB_DEVICE(_AND_INTERFACE_INFO)?\(0x[0-9A-Fa-f]{4}, *0x[0-9A-Fa-f]{4}' \
 *       reference/rtl8812au/os_dep/linux/usb_intf.c |
 *     grep -oE '0x[0-9A-Fa-f]{4}, *0x[0-9A-Fa-f]{4}' | tr -d ' ' | sort -u
 *
 * These are witnesses, not a dispatch table: devourer identifies this family by
 * chip-id, and nothing here should ever be used to route a device. */
constexpr mt7612u::UsbId kJaguarWitness[] = {
    {0x0409, 0x0408}, {0x0411, 0x0242}, {0x0411, 0x025d}, {0x0411, 0x029b},
    {0x04bb, 0x0952}, {0x04bb, 0x0953}, {0x050d, 0x1106}, {0x050d, 0x1109},
    {0x056e, 0x4007}, {0x056e, 0x400b}, {0x056e, 0x400d}, {0x056e, 0x400e},
    {0x056e, 0x400f}, {0x056e, 0x4010}, {0x0586, 0x3426}, {0x0789, 0x016e},
    {0x07b8, 0x8179}, {0x07b8, 0x8812}, {0x0846, 0x9051}, {0x0846, 0x9052},
    {0x0846, 0x9054}, {0x0b05, 0x17d2}, {0x0b05, 0x1817}, {0x0b05, 0x1852},
    {0x0b05, 0x1853}, {0x0df6, 0x0074}, {0x0e66, 0x0022}, {0x0e66, 0x0023},
    {0x0e66, 0x0026}, {0x1058, 0x0632}, {0x13b1, 0x003f}, {0x148f, 0x9097},
    {0x1740, 0x0100}, {0x2001, 0x330e}, {0x2001, 0x3313}, {0x2001, 0x3314},
    {0x2001, 0x3315}, {0x2001, 0x3316}, {0x2001, 0x3318}, {0x2001, 0x331a},
    {0x2019, 0xab30}, {0x2019, 0xab32}, {0x20f4, 0x805b}, {0x20f4, 0x809a},
    {0x20f4, 0x809b}, {0x2357, 0x0101}, {0x2357, 0x0103}, {0x2357, 0x0106},
    {0x2357, 0x010d}, {0x2357, 0x010e}, {0x2357, 0x010f}, {0x2357, 0x011e},
    {0x2357, 0x011f}, {0x2357, 0x0120}, {0x2357, 0x0122}, {0x2604, 0x0012},
    {0x3823, 0x6249}, {0x7392, 0xa811}, {0x7392, 0xa812}, {0x7392, 0xa813},
    {0x7392, 0xa822}, {0x7392, 0xa833}, {0x7392, 0xa834}, {0x7392, 0xb611},
};

} // namespace

int main() {
  /* Every tabled MediaTek id is claimed. A table entry the lookup misses would
   * silently reopen the Jaguar1 misroute for that adapter. */
  for (const mt7612u::UsbId &id : mt7612u::kUsbIds)
    if (!mt7612u::is_usb_id(id.vid, id.pid)) {
      std::fprintf(stderr,
                   "mt7612u_usb_ids: FAIL %04x:%04x tabled but not claimed\n",
                   id.vid, id.pid);
      fails++;
    }

  /* THE load-bearing property: the MediaTek gate must not claim any device a
   * Realtek backend owns. It runs before the SYS_CFG2 read, so a false claim
   * here is unrecoverable — the Realtek adapter is refused outright. */
  int checked = 0;
  auto reject = [&](uint16_t vid, uint16_t pid, const char *family) {
    checked++;
    if (mt7612u::is_usb_id(vid, pid)) {
      std::fprintf(stderr,
                   "mt7612u_usb_ids: FAIL %04x:%04x is %s but the MediaTek "
                   "gate claims it\n",
                   vid, pid, family);
      fails++;
    }
  };
  for (const mt7612u::UsbId &id : kJaguarWitness)
    reject(id.vid, id.pid, "Jaguar1/2/3");
  for (const rtl8733b::UsbId &id : rtl8733b::kUsbIds)
    reject(id.vid, id.pid, "RTL8733B");
  for (const auto &id : kestrel::kKestrelUsbIds)
    reject(id.vid, id.pid, "Kestrel");

  /* The coverage itself is an invariant: if someone trims the witness list, the
   * blindness this test exists to prevent comes back silently. */
  check(checked >= 91, "cross-checked against at least 91 Realtek ids");

  /* The exact id the earlier, Jaguar-blind version of this test let through. */
  check(!mt7612u::is_usb_id(0x0b05, 0x17d2),
        "ASUS USB-AC56 (RTL8812AU) is not claimed");
  check(!mt7612u::is_usb_id(0x0bda, 0x8812), "RTL8812AU reference id not claimed");
  check(!mt7612u::is_usb_id(0x0bda, 0x8813), "RTL8814AU reference id not claimed");

  /* Netgear interleaves the two families in adjacent product ids, so this is
   * where an off-by-one transcription would land. */
  check(mt7612u::is_usb_id(0x0846, 0x9053), "Netgear A6210 (MediaTek) is claimed");
  check(!mt7612u::is_usb_id(0x0846, 0x9052), "Netgear A6100 (RTL8821) is not claimed");
  check(!mt7612u::is_usb_id(0x0846, 0x9054), "Netgear A7000 (RTL8814A) is not claimed");

  /* Exact pair match, not a prefix or a vendor test. */
  check(mt7612u::is_usb_id(0x0e8d, 0x7612), "the verified Alfa AWUS036ACM is claimed");
  check(!mt7612u::is_usb_id(0x0e8d, 0x7613), "an untabled MediaTek pid is not claimed");
  check(!mt7612u::is_usb_id(0x0bda, 0x7612), "the pid alone does not claim");

  /* Reported, not asserted — the rationale for pair-matching. */
  int shared_vids = 0;
  for (const mt7612u::UsbId &m : mt7612u::kUsbIds) {
    bool shared = false;
    for (const mt7612u::UsbId &j : kJaguarWitness)
      shared = shared || j.vid == m.vid;
    for (const rtl8733b::UsbId &r : rtl8733b::kUsbIds)
      shared = shared || r.vid == m.vid;
    for (const auto &k : kestrel::kKestrelUsbIds)
      shared = shared || k.vid == m.vid;
    if (shared)
      shared_vids++;
  }
  std::printf("mt7612u_usb_ids: %d MediaTek ids vs %d Realtek ids; %d MediaTek "
              "ids share a vendor id with a Realtek one (why the gate matches "
              "pairs, not vendors)\n",
              (int)(sizeof(mt7612u::kUsbIds) / sizeof(mt7612u::kUsbIds[0])),
              checked, shared_vids);

  if (fails) {
    std::fprintf(stderr, "mt7612u_usb_ids: %d failure(s)\n", fails);
    return 1;
  }
  std::printf("mt7612u_usb_ids: OK\n");
  return 0;
}
