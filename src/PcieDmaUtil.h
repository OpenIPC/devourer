#pragma once

/* Small helpers shared by the PCIe DMA planes. */

#include <chrono>
#include <cstddef>
#include <thread>

namespace devourer::pcie_dma {

constexpr size_t kPageSize = 4096;
inline size_t page_align(size_t v) {
  return (v + kPageSize - 1) & ~(kPageSize - 1);
}
inline void sleep_us(unsigned us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

} /* namespace devourer::pcie_dma */
