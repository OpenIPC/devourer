#ifndef SWEEP_SPEC_H
#define SWEEP_SPEC_H

#include <cstdlib>
#include <string>
#include <vector>

#include "ChannelFreq.h"

/* Sweep/hop bin-list grammar shared by DEVOURER_RX_SWEEP (demo) and
 * DEVOURER_HOP_CHANNELS (txdemo). Comma-separated tokens, each either
 *
 *   <ch>            a channel number                        "1,6,11"
 *   <a>-<b>[/<s>]   an inclusive range with step s (def. 1) "36-48/4"
 *   <fa>-<fb>[/<s>] a centre-frequency range in MHz         "5170-5250/5"
 *                   (values >= 1000 are MHz; step in MHz — 5 MHz per channel
 *                   number, so "2412-2462/25" = every 5th 2.4 GHz channel)
 *
 * A frequency range maps through devourer::freq_to_chan (5 GHz channels sit on
 * the 5 MHz grid, so 5 MHz-wide narrowband bins get one bin per channel
 * number). Malformed / out-of-band tokens are dropped. */

namespace devourer {

inline std::vector<int> parse_sweep_spec(const char *spec) {
  std::vector<int> out;
  if (spec == nullptr || *spec == '\0')
    return out;
  const std::string s = spec;
  size_t pos = 0;
  while (pos < s.size()) {
    const size_t comma = s.find(',', pos);
    std::string tok = s.substr(
        pos, comma == std::string::npos ? std::string::npos : comma - pos);
    pos = (comma == std::string::npos) ? s.size() : comma + 1;
    if (tok.empty())
      continue;

    long step = 1;
    const size_t slash = tok.find('/');
    if (slash != std::string::npos) {
      step = std::strtol(tok.c_str() + slash + 1, nullptr, 0);
      tok = tok.substr(0, slash);
    }
    if (step < 1)
      step = 1;

    /* A '-' past position 0 splits a range (a leading '-' would just make the
     * token malformed; channels are positive). */
    const size_t dash = tok.find('-', 1);
    if (dash == std::string::npos) {
      const int ch = static_cast<int>(std::strtol(tok.c_str(), nullptr, 0));
      if (ch > 0)
        out.push_back(ch);
      continue;
    }
    const long a = std::strtol(tok.substr(0, dash).c_str(), nullptr, 0);
    const long b = std::strtol(tok.substr(dash + 1).c_str(), nullptr, 0);
    if (a <= 0 || b < a)
      continue;
    /* MHz ranges live on the 5 MHz channel grid — a finer step would only
     * emit duplicate bins. */
    if (a >= 1000 && step < 5)
      step = 5;
    for (long v = a; v <= b; v += step) {
      const int ch = (a >= 1000)
                         ? freq_to_chan(static_cast<uint16_t>(v))
                         : static_cast<int>(v);
      if (ch > 0)
        out.push_back(ch);
    }
  }
  return out;
}

} /* namespace devourer */

#endif /* SWEEP_SPEC_H */
