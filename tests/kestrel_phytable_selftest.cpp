/* Headless guard for the Kestrel halbb/halrf table walker
 * (src/kestrel/PhyTableLoaderKestrel.h): headline {rfe,cut} selection and the
 * IF/ELSE_IF/CHK/ELSE/END conditional opcodes. Pure logic — the applied
 * register writes are captured into a vector, no hardware. */
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include "kestrel/PhyTableLoaderKestrel.h"

using namespace kestrel;

static int failures = 0;
#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

static std::vector<std::pair<uint32_t, uint32_t>>
run(const std::vector<uint32_t> &t, uint32_t rfe, uint32_t cut) {
  std::vector<std::pair<uint32_t, uint32_t>> out;
  apply_phy_table(t.data(), t.size(), rfe, cut,
                  [&](uint32_t a, uint32_t v) { out.emplace_back(a, v); });
  return out;
}

int main() {
  /* --- plain unconditional writes (no headline, no opcodes) --- */
  {
    auto o = run({0x4000, 0x11, 0x4004, 0x22, 0x4008, 0x33}, 1, 1);
    CHECK(o.size() == 3);
    CHECK(o[0] == std::make_pair(0x4000u, 0x11u));
    CHECK(o[2] == std::make_pair(0x4008u, 0x33u));
  }

  /* --- IF/CHK matching a headline-derived cfg_target --- */
  {
    /* headline: variant {rfe=1,cut=0} -> target 0x00010000. */
    const uint32_t H = (uint32_t(PHY_OP_HEADLINE) << 28);
    const uint32_t IF = (uint32_t(PHY_OP_IF) << 28);
    const uint32_t CHK = (uint32_t(PHY_OP_CHK) << 28);
    const uint32_t END = (uint32_t(PHY_OP_END) << 28);
    std::vector<uint32_t> t = {
        H | 0x00010000, 0x0,           /* headline0 rfe1/cut0 */
        H | 0x00020000, 0x0,           /* headline1 rfe2/cut0 */
        0x1000,         0xAA,          /* unconditional */
        IF | 0x00010000, 0x0,          /* if rfe1 */
        CHK,             0x0,          /* check */
        0x2000,         0xBB,          /* only when rfe1 matches */
        END,             0x0,
        0x3000,         0xCC,          /* unconditional after block */
    };
    /* rfe=1 -> cfg_target=0x00010000 (headline0) -> the IF block applies. */
    auto o = run(t, 1, 0);
    CHECK(o.size() == 3);
    CHECK(o[0] == std::make_pair(0x1000u, 0xAAu));
    CHECK(o[1] == std::make_pair(0x2000u, 0xBBu)); /* conditional taken */
    CHECK(o[2] == std::make_pair(0x3000u, 0xCCu));

    /* rfe=2 -> cfg_target=0x00020000 -> the IF(rfe1) block is skipped. */
    auto o2 = run(t, 2, 0);
    CHECK(o2.size() == 2);
    CHECK(o2[0] == std::make_pair(0x1000u, 0xAAu));
    CHECK(o2[1] == std::make_pair(0x3000u, 0xCCu)); /* 0x2000 skipped */
  }

  /* --- ELSE closes a matched branch (vendor semantics: ELSE turns matching
   * off; it is a terminator after a branch matched, not a fallback that
   * applies). --- */
  {
    const uint32_t H = (uint32_t(PHY_OP_HEADLINE) << 28);
    const uint32_t IF = (uint32_t(PHY_OP_IF) << 28);
    const uint32_t CHK = (uint32_t(PHY_OP_CHK) << 28);
    const uint32_t ELSE = (uint32_t(PHY_OP_ELSE) << 28);
    const uint32_t END = (uint32_t(PHY_OP_END) << 28);
    std::vector<uint32_t> t = {
        H | 0x00050000, 0x0,           /* variant rfe5 */
        IF | 0x00050000, 0x0,          /* if rfe5 (matches) */
        CHK,             0x0,
        0x2000,         0xBB,          /* applies (matched) */
        ELSE,            0x0,
        0x2004,         0xEE,          /* skipped (ELSE turned matching off) */
        END,             0x0,
        0x3000,         0xFF,          /* unconditional after END */
    };
    auto o = run(t, 5, 0);
    CHECK(o.size() == 2);
    CHECK(o[0] == std::make_pair(0x2000u, 0xBBu));
    CHECK(o[1] == std::make_pair(0x3000u, 0xFFu));
  }

  if (failures == 0)
    std::printf("kestrel_phytable_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
