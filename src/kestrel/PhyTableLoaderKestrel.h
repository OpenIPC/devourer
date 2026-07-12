#ifndef KESTREL_PHY_TABLE_LOADER_H
#define KESTREL_PHY_TABLE_LOADER_H

#include <cstddef>
#include <cstdint>
#include <functional>

/* Runtime walker for the RTL8852B halbb/halrf register tables (extracted by
 * tools/extract_8852b_phy.py). Transcribed from the vendor walker in
 * reference/rtl8852bu phl/hal_g6/phy/bb/halbb_hwimg_8852b.c +
 * halbb_hw_cfg.c (halbb_sel_headline / halbb_cfg_bb_phy) — the same
 * conditional-table shape devourer already walks for phydm (PhyTableLoader /
 * PhyTableLoaderJaguar3), with the G6 opcode encoding.
 *
 * Table layout: an array of u32 pairs {v1, v2}. A leading run of "headline"
 * entries (top nibble 0xF) declares the {rfe,cut} variants; the walker picks
 * the one matching the running chip and derives cfg_target. The body then
 * mixes plain register writes (top nibble 0: v1=addr, v2=value) with
 * conditional opcodes in v1's top nibble:
 *   IF (0x8) / ELSE_IF (0x9)  set the pending condition (rfe/cut params)
 *   CHK (0x4)                 evaluate it against cfg_target
 *   ELSE (0xA)                take the fallback branch
 *   END (0xB)                 close the conditional block
 * A register write is emitted (via the caller's callback) only while the
 * current branch matches. Header-only + callback-based so BB (write32) and RF
 * (RF-serial write) share it, and so it unit-tests without hardware. */

namespace kestrel {

/* Opcode nibbles (halbb_hwimg_8852b.h). */
enum : uint8_t {
  PHY_OP_CHK = 0x4,
  PHY_OP_IF = 0x8,
  PHY_OP_ELSE_IF = 0x9,
  PHY_OP_ELSE = 0xA,
  PHY_OP_END = 0xB,
  PHY_OP_HEADLINE = 0xF,
};

/* Apply a halbb/halrf table. `emit(addr, val)` performs one register write
 * (the caller handles the 0xf9..0xfe delay pseudo-addresses and the actual
 * write mechanism). `rfe_type` / `cut` select the headline variant. */
inline void apply_phy_table(const uint32_t *arr, size_t len, uint32_t rfe_type,
                            uint32_t cut,
                            const std::function<void(uint32_t, uint32_t)> &emit) {
  if (arr == nullptr || len < 2)
    return;

  /* Headline scan: leading entries with top nibble 0xF. */
  size_t h_size = 0;
  while (h_size + 1 < len && (arr[h_size] >> 28) == PHY_OP_HEADLINE)
    h_size += 2;

  uint32_t cfg_target = 0;
  if (h_size != 0) {
    /* Prefer the {rfe match, cut match} headline; the vendor's fuller
     * fallback ladder (rfe-only / cut-only / don't-care) collapses to this
     * exact-match plus "first headline" default, which covers the stock
     * single-variant adapters. */
    const uint32_t want = ((rfe_type & 0xff) << 16) | (cut & 0xff);
    size_t h_idx = 0; /* default: first headline */
    for (size_t i = 0; i < h_size; i += 2) {
      if ((arr[i] & 0x0fffffff) == want) {
        h_idx = i >> 1;
        break;
      }
    }
    cfg_target = arr[h_idx << 1] & 0x0fffffff;
  }

  bool is_matched = true;  /* default flags (halbb_flag_2_default) */
  bool find_target = false;
  uint32_t cfg_para = 0;
  for (size_t i = h_size; i + 1 < len; i += 2) {
    const uint32_t v1 = arr[i];
    const uint32_t v2 = arr[i + 1];
    switch (v1 >> 28) {
    case PHY_OP_IF:
    case PHY_OP_ELSE_IF:
      cfg_para = v1 & 0x0fffffff;
      break;
    case PHY_OP_ELSE:
      is_matched = false;
      break;
    case PHY_OP_END:
      is_matched = true;
      find_target = false;
      break;
    case PHY_OP_CHK:
      if (find_target) {
        is_matched = false;
      } else if (cfg_para == cfg_target) {
        is_matched = true;
        find_target = true;
      } else {
        is_matched = false;
        find_target = false;
      }
      break;
    default: /* top nibble 0 — a register write */
      if (is_matched)
        emit(v1, v2);
      break;
    }
  }
}

/* Apply a halrf *radio* table (array_mp_8852b_radioa/radiob). The RF tables use
 * a DIFFERENT conditional encoding from the halbb tables above — transcribed
 * verbatim from halrf_config_8852b_radio_a_reg (halrf_hwimg_8852b.c): entries
 * with bit31 set are IF/ELSE_IF/ELSE/END that stash the {rfe,cut} condition
 * (rfe = v1[23:16], cut = v1[7:0], 0xFF = don't-care); a bit30 entry is the
 * CHK that commits the match; a plain entry (top bits clear) is a register
 * write, but only while matched — with a "highest matching cut wins" fallback
 * (cv_max / latest-match replay) for the else branch. `emit(addr,val)` performs
 * one RF write. */
inline void apply_rf_table(const uint32_t *arr, size_t len, uint32_t rfe,
                           uint32_t cut,
                           const std::function<void(uint32_t, uint32_t)> &emit) {
  if (arr == nullptr || len < 2)
    return;
  constexpr uint32_t DONT_CARE = 0xff;
  bool is_matched = true, is_skipped = false, is_rfe_match = false;
  bool is_cart_match = false, is_else_case = false, is_rfe_ever_match = false;
  uint32_t cv = 0, cv_max = 0, latest = 0;

  for (size_t i = 0; i + 1 < len; i += 2) {
    const uint32_t v1 = arr[i];
    const uint32_t v2 = arr[i + 1];
    if (v1 & (1u << 31)) {
      const uint32_t c = v1 >> 28;
      if (c == 0xb) { /* END */
        is_matched = true;
        is_skipped = false;
      } else { /* IF (0x8) / ELSE_IF (0x9) / ELSE (0xa) [/ 0xf header] */
        const uint32_t rt = (v1 >> 16) & 0xff;
        is_rfe_match = (rt == DONT_CARE) ? true : (rt == rfe);
        const uint32_t ct = v1 & 0xff;
        if (ct == DONT_CARE) {
          is_cart_match = true;
        } else {
          cv = ct;
          is_cart_match = (ct == cut);
        }
        if (c == 0xa) { /* ELSE */
          is_else_case = !is_skipped;
          if (!is_rfe_ever_match)
            is_matched = !is_skipped;
        }
      }
    } else if (v1 & (1u << 30)) { /* CHK */
      if (is_skipped) {
        is_matched = false;
      } else if (is_rfe_match && is_cart_match) {
        is_matched = true;
        is_skipped = true;
      } else {
        is_matched = false;
        is_skipped = false;
      }
    } else { /* register write */
      if (is_matched) {
        emit(v1, v2);
        is_rfe_match = false;
        is_else_case = false;
        is_rfe_ever_match = false;
      } else if (is_rfe_match) {
        if (cv >= cv_max) { /* keep the highest matching cut */
          cv_max = cv;
          is_rfe_ever_match = true;
          latest = static_cast<uint32_t>(i);
        }
        is_rfe_match = false;
      } else if (is_else_case) { /* replay the latest matched block */
        is_else_case = false;
        is_rfe_ever_match = false;
        is_rfe_match = false;
        size_t j = latest;
        do {
          emit(arr[j], arr[j + 1]);
          j += 2;
        } while (j + 1 < len && !(arr[j] & 0xC0000000u));
      }
    }
  }
}

} /* namespace kestrel */

#endif /* KESTREL_PHY_TABLE_LOADER_H */
