/* Headless guard for the MT76x0U USB-id gate (src/mt76x0u/Mt76x0uUsbIds.h)
 * and the MediaTek chip-family router (src/MediatekProbe.h), both consulted by
 * WiFiDriver::CreateRadio before the Realtek SYS_CFG2 read.
 *
 * Three properties, each of which would misroute a real adapter if broken:
 *
 *  1. The MT76x0U pair set never claims a device a Realtek backend owns. Like
 *     the MT7612U cell, the cross-check covers all 91 Realtek ids including
 *     the 64 Jaguar ids that have no table in devourer - and this table sits
 *     at product-id distance one from five of them.
 *  2. The overlap with the MT7612U table is exactly the ids flagged
 *     kFlagSharedWithMt7612u, so every other id routes without a register
 *     read and the shared one never routes on the id alone.
 *  3. family_of() is mt76x02.h's is_mt76x0()/is_mt76x2(), so a shared id
 *     reaches the backend that will accept the chip. */
#include "MediatekProbe.h"
#include "kestrel/KestrelUsbIds.h"
#include "mt7612u/Mt7612uUsbIds.h"
#include "mt76x0u/Mt76x0uUsbIds.h"
#include "rtl8733b/Rtl8733bUsbIds.h"

#include <cstdint>
#include <cstdio>

namespace {

int fails = 0;
void check(bool ok, const char *what) {
  if (!ok) {
    std::fprintf(stderr, "mt76x0u_usb_ids: FAIL %s\n", what);
    fails++;
  }
}

struct Pair {
  uint16_t vid, pid;
};

/* Every USB id the Jaguar1/2/3 vendor driver claims - the same witness list
 * tests/mt7612u_usb_ids_selftest.cpp embeds, from
 * reference/rtl8812au/os_dep/linux/usb_intf.c (regeneration command there).
 * Witnesses only: devourer routes this family by chip-id. */
constexpr Pair kJaguarWitness[] = {
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
  using devourer::mediatek::Family;
  using devourer::mediatek::family_of;

  /* Every tabled id is claimed, and the lookup returns its own entry. */
  for (const mt76x0u::UsbId &id : mt76x0u::kUsbIds) {
    const mt76x0u::UsbId *found = mt76x0u::find_usb_id(id.vid, id.pid);
    if (!found || found->flags != id.flags) {
      std::fprintf(stderr, "mt76x0u_usb_ids: FAIL %04x:%04x tabled but not "
                           "claimed as itself\n", id.vid, id.pid);
      fails++;
    }
  }

  /* 1. No Realtek id is claimed. */
  int checked = 0;
  auto reject = [&](uint16_t vid, uint16_t pid, const char *family) {
    checked++;
    if (mt76x0u::is_usb_id(vid, pid)) {
      std::fprintf(stderr, "mt76x0u_usb_ids: FAIL %04x:%04x is %s but the "
                           "MT76x0U gate claims it\n", vid, pid, family);
      fails++;
    }
  };
  for (const Pair &id : kJaguarWitness)
    reject(id.vid, id.pid, "Jaguar1/2/3");
  for (const rtl8733b::UsbId &id : rtl8733b::kUsbIds)
    reject(id.vid, id.pid, "RTL8733B");
  for (const auto &id : kestrel::kKestrelUsbIds)
    reject(id.vid, id.pid, "Kestrel");
  check(checked >= 91, "cross-checked against at least 91 Realtek ids");

  /* The distance-one neighbours, by name: an off-by-one transcription lands
   * on one of these. */
  check(mt76x0u::is_usb_id(0x0586, 0x3425) && !mt76x0u::is_usb_id(0x0586, 0x3426),
        "ZyXEL 3425 (MediaTek) claimed, 3426 (Realtek) not");
  check(mt76x0u::is_usb_id(0x2019, 0xab31) && !mt76x0u::is_usb_id(0x2019, 0xab30) &&
            !mt76x0u::is_usb_id(0x2019, 0xab32),
        "Planex ab31 claimed, ab30/ab32 not");
  check(mt76x0u::is_usb_id(0x13b1, 0x003e) && !mt76x0u::is_usb_id(0x13b1, 0x003f),
        "Linksys 003e claimed, 003f not");
  check(mt76x0u::is_usb_id(0x2357, 0x0123) && !mt76x0u::is_usb_id(0x2357, 0x0122),
        "TP-Link 0123 claimed, 0122 not");
  check(!mt76x0u::is_usb_id(0x0bda, 0x8812), "RTL8812AU reference id not claimed");

  /* 2. Overlap with the MT7612U table == the flagged set, exactly. */
  int shared = 0;
  for (const mt76x0u::UsbId &id : mt76x0u::kUsbIds) {
    const bool in_7612 = mt7612u::is_usb_id(id.vid, id.pid);
    const bool flagged = (id.flags & mt76x0u::kFlagSharedWithMt7612u) != 0;
    if (in_7612 != flagged) {
      std::fprintf(stderr, "mt76x0u_usb_ids: FAIL %04x:%04x is %sin the MT7612U "
                           "table but %sflagged shared\n", id.vid, id.pid,
                   in_7612 ? "" : "not ", flagged ? "" : "not ");
      fails++;
    }
    shared += in_7612;
  }
  check(shared == 1, "exactly one id is shared with MT7612U");
  check(!mt76x0u::is_exclusive_usb_id(0x7392, 0xb711),
        "7392:b711 is not routed on the id alone");
  check(mt76x0u::is_exclusive_usb_id(0x0e8d, 0x7610),
        "0e8d:7610 (the development target) routes on the id");
  check(!mt7612u::is_usb_id(0x0e8d, 0x7610), "0e8d:7610 is not an MT7612U id");

  /* The kernel quirk: only the Archer T1U. */
  int no2g = 0;
  for (const mt76x0u::UsbId &id : mt76x0u::kUsbIds)
    no2g += (id.flags & mt76x0u::kFlagNo2Ghz) != 0;
  check(no2g == 1 && mt76x0u::usb_id_has_no_2ghz(0x2357, 0x0105),
        "no_2ghz is set for 2357:0105 and nothing else");

  /* 3. Family routing matches mt76x02.h. */
  check(family_of(0x76100044) == Family::Mt76x0, "0x7610 is mt76x0");
  check(family_of(0x76300000) == Family::Mt76x0, "0x7630 is mt76x0");
  check(family_of(0x76500000) == Family::Mt76x0, "0x7650 is mt76x0");
  check(family_of(0x76120044) == Family::Mt76x2, "0x7612 is mt76x2");
  check(family_of(0x76320000) == Family::Mt76x2, "0x7632 is mt76x2");
  check(family_of(0x76620000) == Family::Mt76x2, "0x7662 is mt76x2");
  check(family_of(0x76020000) == Family::Mt76x2, "0x7602 is mt76x2");
  check(family_of(0x76010000) == Family::Unknown, "0x7601 (MT7601U) is neither");
  check(family_of(0xffffffff) == Family::Unknown, "an all-ones read is neither");
  check(family_of(0) == Family::Unknown, "a zero read is neither");

  std::printf("mt76x0u_usb_ids: %d MT76x0U ids vs %d Realtek ids, %d shared "
              "with MT7612U\n",
              (int)(sizeof(mt76x0u::kUsbIds) / sizeof(mt76x0u::kUsbIds[0])),
              checked, shared);
  if (fails) {
    std::fprintf(stderr, "mt76x0u_usb_ids: %d failure(s)\n", fails);
    return 1;
  }
  std::printf("mt76x0u_usb_ids: OK\n");
  return 0;
}
