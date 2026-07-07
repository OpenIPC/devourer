/* BfReportDetect — event-stream detector for beamforming self-sounding
 * reports, shared by rxdemo (2-adapter rig: separate capture radio) and
 * txdemo (single-radio: the sounder captures its own reports via
 * StartRxLoop). Header-only so the demos stay identical consumers.
 *
 * A VHT/HT Compressed Beamforming report is a management Action frame —
 * subtype 0xD0 (Action) or 0xE0 (Action No-Ack; CSI reports are sent No-Ack)
 * — whose body begins with a category + action of VHT cat 0x15 act 0x00, or
 * HT cat 0x07 act 0x00. When an armed beamformee replies to our NDPA+NDP the
 * report's SA is the beamformee MAC, the decode-level confirmation the
 * unassociated responder produced CSI.
 *
 * Emission goes through `bf_events` — a module-level EventSink pointer the
 * hosting demo sets right after constructing its Logger
 * (`devourer::bf::bf_events = &logger->events();`). Unset = detector inert.
 *
 * DEVOURER_BF_DETECT_REPORT modes:
 *   1  `bf.report` summary event (Nc/Nr/BW/Ng, SA) per report
 *   2  `bf.any` FC/category event for EVERY frame (subtype survey)
 *   3  mode 1 + capped CSI-payload hexdump (`bf.csi`, first frames)
 *   4  mode 1 + `bf.report_raw` full frame hex (for the decoder,
 *      tools/bf_report_decode.py) */
#ifndef BF_REPORT_DETECT_H
#define BF_REPORT_DETECT_H

#include <cstdio>
#include <cstdlib>

#include "Event.h"
#include "RxPacket.h"

namespace devourer::bf {

/* Event sink for every emission in this header. The demo points it at its
 * Logger's sink before the RX loop starts; nullptr disables the detector's
 * output entirely. */
inline devourer::EventSink *bf_events = nullptr;

inline void detect_report(const Packet &packet) {
  const char *mode_s = std::getenv("DEVOURER_BF_DETECT_REPORT");
  if (!mode_s || packet.Data.size() < 27 || bf_events == nullptr)
    return;
  const char mode = mode_s[0];
  const uint8_t *d = packet.Data.data();
  const uint8_t cat = d[24], act = d[25];
  const bool vht = (cat == 0x15 && act == 0x00);
  const bool ht = (cat == 0x07 && act == 0x00);
  if (mode == '2') {
    static int any = 0;
    if (++any <= 4000) {
      const unsigned fc16 = (unsigned(d[0]) << 8) | d[1];
      devourer::Ev(*bf_events, "bf.any")
          .hexf("fc", fc16, 4)
          .hexf("cat", cat, 2)
          .hexf("act", act, 2)
          .f("crc", packet.RxAtrib.crc_err ? 1 : 0)
          .f("len", packet.Data.size());
    }
  }
  const uint8_t sub = d[0] & 0xF0;
  if ((sub == 0xD0 || sub == 0xE0) && (vht || ht)) {
    static int rpt = 0;
    ++rpt;
    const uint8_t *mc = d + 26; /* VHT MIMO control field */
    unsigned nc = (mc[0] & 0x07) + 1, nr = ((mc[0] >> 3) & 0x07) + 1;
    unsigned chw = (mc[0] >> 6) & 0x03, ng = mc[1] & 0x03;
    char sa[18];
    std::snprintf(sa, sizeof(sa), "%02x:%02x:%02x:%02x:%02x:%02x", d[10],
                  d[11], d[12], d[13], d[14], d[15]);
    devourer::Ev(*bf_events, "bf.report")
        .f("kind", vht ? "VHT" : "HT")
        .f("n", rpt)
        .f("sa", sa)
        .f("nc", nc)
        .f("nr", nr)
        .f("bw", chw)
        .f("ng", ng)
        .f("len", packet.Data.size());
    if (mode == '3' && rpt <= 6) {
      size_t off = 26 + 3; /* hdr(24)+cat+act+mimoctrl(3) */
      size_t end = packet.Data.size() >= 4 ? packet.Data.size() - 4 : off;
      size_t n = end > off ? end - off : 0;
      devourer::Ev(*bf_events, "bf.csi")
          .f("len", n)
          .hex("csi", d + off, n < 40 ? n : 40);
    }
    if (mode == '4' && rpt <= 200) {
      /* Full-frame hex — consumed by tools/bf_report_decode.py. */
      devourer::Ev(*bf_events, "bf.report_raw")
          .hex("frame", d, packet.Data.size());
    }
  }
}

} // namespace devourer::bf

#endif /* BF_REPORT_DETECT_H */
