/* BfReportDetect — console detector for beamforming self-sounding reports,
 * shared by WiFiDriverDemo (2-adapter rig: separate capture radio) and
 * WiFiDriverTxDemo (single-radio: the sounder captures its own reports via
 * StartRxLoop). Header-only so the demos stay identical consumers.
 *
 * A VHT/HT Compressed Beamforming report is a management Action frame —
 * subtype 0xD0 (Action) or 0xE0 (Action No-Ack; CSI reports are sent No-Ack)
 * — whose body begins with a category + action of VHT cat 0x15 act 0x00, or
 * HT cat 0x07 act 0x00. When an armed beamformee replies to our NDPA+NDP the
 * report's SA is the beamformee MAC, the decode-level confirmation the
 * unassociated responder produced CSI.
 *
 * DEVOURER_BF_DETECT_REPORT modes:
 *   1  <devourer-bf-report> summary (Nc/Nr/BW/Ng, SA) per report
 *   2  <devourer-bf-any> FC/category of EVERY frame (subtype survey)
 *   3  mode 1 + capped CSI-payload hexdump (first frames)
 *   4  mode 1 + <devourer-bf-report-raw> full frame hex (for the decoder,
 *      tools/bf_report_decode.py) */
#ifndef BF_REPORT_DETECT_H
#define BF_REPORT_DETECT_H

#include <cstdio>
#include <cstdlib>

#include "RxPacket.h"

namespace devourer::bf {

inline void detect_report(const Packet &packet) {
  const char *mode_s = std::getenv("DEVOURER_BF_DETECT_REPORT");
  if (!mode_s || packet.Data.size() < 27)
    return;
  const char mode = mode_s[0];
  const uint8_t *d = packet.Data.data();
  const uint8_t cat = d[24], act = d[25];
  const bool vht = (cat == 0x15 && act == 0x00);
  const bool ht = (cat == 0x07 && act == 0x00);
  if (mode == '2') {
    static int any = 0;
    if (++any <= 4000)
      printf("<devourer-bf-any>fc=%02x%02x cat=%02x act=%02x crc=%u "
             "len=%zu\n", d[0], d[1], cat, act,
             packet.RxAtrib.crc_err ? 1u : 0u, packet.Data.size());
  }
  const uint8_t sub = d[0] & 0xF0;
  if ((sub == 0xD0 || sub == 0xE0) && (vht || ht)) {
    static int rpt = 0;
    ++rpt;
    const uint8_t *mc = d + 26; /* VHT MIMO control field */
    unsigned nc = (mc[0] & 0x07) + 1, nr = ((mc[0] >> 3) & 0x07) + 1;
    unsigned chw = (mc[0] >> 6) & 0x03, ng = mc[1] & 0x03;
    printf("<devourer-bf-report>%s n=%d sa=%02x:%02x:%02x:%02x:%02x:%02x "
           "Nc=%u Nr=%u BW=%u Ng=%u len=%zu\n",
           vht ? "VHT" : "HT", rpt, d[10], d[11], d[12], d[13], d[14],
           d[15], nc, nr, chw, ng, packet.Data.size());
    if (mode == '3' && rpt <= 6) {
      size_t off = 26 + 3; /* hdr(24)+cat+act+mimoctrl(3) */
      size_t end = packet.Data.size() >= 4 ? packet.Data.size() - 4 : off;
      printf("  csi[%zu]:", end - off);
      for (size_t i = off; i < end && i < off + 40; ++i)
        printf(" %02x", d[i]);
      printf("\n");
    }
    if (mode == '4' && rpt <= 200) {
      printf("<devourer-bf-report-raw>");
      for (size_t i = 0; i < packet.Data.size(); ++i)
        printf("%02x", d[i]);
      printf("\n");
    }
    fflush(stdout);
  }
}

} // namespace devourer::bf

#endif /* BF_REPORT_DETECT_H */
