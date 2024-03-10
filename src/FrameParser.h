#ifndef FRAMEPARSER_H
#define FRAMEPARSER_H

#include "logger.h"
#include <span>

enum class RX_PACKET_TYPE {
  NORMAL_RX,  /* Normal rx packet */
  TX_REPORT1, /* CCX */
  TX_REPORT2, /* TX RPT */
  HIS_REPORT, /* USB HISR RPT */
  C2H_PACKET
};

struct rx_pkt_attrib {
  uint16_t pkt_len;
  bool physt;
  uint8_t drvinfo_sz;
  uint8_t shift_sz;
  bool qos;
  uint8_t priority;
  bool mdata;
  uint16_t seq_num;
  uint8_t frag_num;
  bool mfrag;
  bool bdecrypted;
  uint8_t encrypt; /* when 0 indicate no encrypt. when non-zero, indicate the
                      encrypt algorith */
  bool crc_err;
  bool icv_err;
  uint8_t data_rate;
  uint8_t bw;
  uint8_t stbc;
  uint8_t ldpc;
  uint8_t sgi;
  RX_PACKET_TYPE pkt_rpt_type;
};

struct Packet {
  rx_pkt_attrib RxAtrib;
  std::span<uint8_t> Data;
};

class FrameParser {
  Logger_t _logger;

public:
  FrameParser(Logger_t logger);
  std::vector<Packet> recvbuf2recvframe(std::span<uint8_t> ptr);
};

#endif /* FRAMEPARSER_H */
