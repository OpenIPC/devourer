/* The halmac read_buf_88xx debug-window arithmetic behind
 * IRtlRadio::ReadPacketBuffer (Jaguar2 and Jaguar3). Pure, header-only, so
 * tests/txqueue_selftest.cpp can pin the bounds without hardware.
 *
 * The chip's packet memory is read through a 4 KiB window at 0x8000..0x8FFF;
 * the window is selected by the low 12 bits of REG_PKTBUF_DBG_CTRL (0x0140),
 * counting from a per-memory base (TX FIFO 0x780, LLT 0x650). A read of `n`
 * bytes at byte `offset` walks windows base + (offset >> 12) up to the window
 * holding its LAST byte, base + ((offset + n - 1) >> 12); that index must fit
 * the 12-bit field, or the select would spill into the high nibble of 0x0140
 * that the caller preserves. Reads are 32-bit, so offset and n must be dword
 * aligned: an unaligned offset issues reads past 0x8FFF at a window's end. */
#ifndef DEVOURER_PKTBUF_WINDOW_H
#define DEVOURER_PKTBUF_WINDOW_H

#include <cstddef>
#include <cstdint>

namespace devourer {

inline constexpr uint32_t kPktBufWindowFieldMax = 0xFFF;

/* True when [offset, offset + n) can be read through windows that all fit the
 * 12-bit field. n == 0 is valid and reads nothing (the caller returns without
 * touching the window). The size of the selected memory itself is NOT checked
 * here - see IRtlRadio::ReadPacketBuffer. */
constexpr bool pktbuf_read_fits(uint32_t base, uint32_t offset, size_t n) {
  if (offset % 4 != 0 || n % 4 != 0)
    return false;
  if (base > kPktBufWindowFieldMax)
    return false;
  if (n == 0)
    return true;
  /* Overflow-safe: no request longer than the whole window space can fit, so
   * refuse it before forming offset + n - 1. */
  constexpr uint64_t kSpace = uint64_t(kPktBufWindowFieldMax + 1) << 12;
  if (static_cast<uint64_t>(n) > kSpace)
    return false;
  const uint64_t last = static_cast<uint64_t>(offset) + n - 1;
  return static_cast<uint64_t>(base) + (last >> 12) <= kPktBufWindowFieldMax;
}

} // namespace devourer

#endif /* DEVOURER_PKTBUF_WINDOW_H */
