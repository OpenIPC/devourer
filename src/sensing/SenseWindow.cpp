#include "sensing/SenseWindow.h"

namespace devourer {
namespace sensing {

int64_t steady_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

} /* namespace sensing */
} /* namespace devourer */
