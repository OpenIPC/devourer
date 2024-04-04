#include "FrameParser.h"

#define CONFIG_USB_RX_AGGREGATION 1

#define RXDESC_SIZE 24

#include "basic_types.h"
#include "rtl8812a_recv.h"
#include <vector>

FrameParser::FrameParser(Logger_t logger) : _logger{logger} {}

static rx_pkt_attrib rtl8812_query_rx_desc_status(uint8_t *pdesc) {
  auto pattrib = rx_pkt_attrib{};

  /* Offset 0 */
  pattrib.pkt_len = GET_RX_STATUS_DESC_PKT_LEN_8812(pdesc);
  pattrib.crc_err = GET_RX_STATUS_DESC_CRC32_8812(pdesc);
  pattrib.icv_err = GET_RX_STATUS_DESC_ICV_8812(pdesc);
  pattrib.drvinfo_sz =
      (uint8_t)(GET_RX_STATUS_DESC_DRVINFO_SIZE_8812(pdesc) * 8);
  pattrib.encrypt = GET_RX_STATUS_DESC_SECURITY_8812(pdesc);
  pattrib.qos = GET_RX_STATUS_DESC_QOS_8812(pdesc);
  /* Qos data, wireless
                                                          lan header length is
                                                          26 */
  pattrib.shift_sz = GET_RX_STATUS_DESC_SHIFT_8812(
      pdesc); /* ((le32_to_cpu(pdesc.rxdw0) >> 24) & 0x3); */
  pattrib.physt = GET_RX_STATUS_DESC_PHY_STATUS_8812(
      pdesc); /* ((le32_to_cpu(pdesc.rxdw0) >> 26) & 0x1); */
  pattrib.bdecrypted = !GET_RX_STATUS_DESC_SWDEC_8812(
      pdesc); /* (le32_to_cpu(pdesc.rxdw0) & BIT(27))? 0:1; */

  /* Offset 4 */
  pattrib.priority = GET_RX_STATUS_DESC_TID_8812(pdesc);
  pattrib.mdata = GET_RX_STATUS_DESC_MORE_DATA_8812(pdesc);
  pattrib.mfrag = GET_RX_STATUS_DESC_MORE_FRAG_8812(pdesc);
  /* more fragment bit */

  /* Offset 8 */
  pattrib.seq_num = GET_RX_STATUS_DESC_SEQ_8812(pdesc);
  pattrib.frag_num = GET_RX_STATUS_DESC_FRAG_8812(pdesc);
  /* fragmentation number */

  if (GET_RX_STATUS_DESC_RPT_SEL_8812(pdesc)) {
    pattrib.pkt_rpt_type = RX_PACKET_TYPE::C2H_PACKET;
  } else {
    pattrib.pkt_rpt_type = RX_PACKET_TYPE::NORMAL_RX;
  }

  /* Offset 12 */
  pattrib.data_rate = GET_RX_STATUS_DESC_RX_RATE_8812(pdesc);
  /* Offset 16 */
  pattrib.sgi = GET_RX_STATUS_DESC_SPLCP_8812(pdesc);
  pattrib.ldpc = GET_RX_STATUS_DESC_LDPC_8812(pdesc);
  pattrib.stbc = GET_RX_STATUS_DESC_STBC_8812(pdesc);
  pattrib.bw = GET_RX_STATUS_DESC_BW_8812(pdesc);

  /* Offset 20 */
  /* pattrib.tsfl=(byte)GET_RX_STATUS_DESC_TSFL_8812(pdesc); */

  return pattrib;
}

__inline static u32 _RND8(u32 sz) {
  u32 val;

  val = ((sz >> 3) + ((sz & 7) ? 1 : 0)) << 3;

  return val;
}

std::vector<Packet> FrameParser::recvbuf2recvframe(std::span<uint8_t> ptr) {
  auto pbuf = ptr;
  auto pkt_cnt = GET_RX_STATUS_DESC_USB_AGG_PKTNUM_8812(pbuf.data());
  //_logger->info("pkt_cnt == {}", pkt_cnt);

  auto ret = std::vector<Packet>{};

  do {
    auto pattrib = rtl8812_query_rx_desc_status(pbuf.data());

    if ((pattrib.crc_err) || (pattrib.icv_err)) {
      _logger->info("RX Warning! crc_err={} "
                    "icv_err={}, skip!",
                    pattrib.crc_err, pattrib.icv_err);
      break;
    }

    auto pkt_offset = RXDESC_SIZE + pattrib.drvinfo_sz + pattrib.shift_sz +
                      pattrib.pkt_len; // this is offset for next package

    if ((pattrib.pkt_len <= 0) || (pkt_offset > pbuf.size())) {
      _logger->warn(
          "RX Warning!,pkt_len <= 0 or pkt_offset > transfer_len; pkt_len: "
          "{}, pkt_offset: {}, transfer_len: {}",
          pattrib.pkt_len, pkt_offset, pbuf.size());
      break;
    }

    if (pattrib.mfrag) {
      // !!! We skips this packages because ohd not use fragmentation
      _logger->warn("mfrag scipping");

      // if (rtw_os_alloc_recvframe(precvframe, pbuf.Slice(pattrib.shift_sz +
      // pattrib.drvinfo_sz + RXDESC_SIZE), pskb) == false)
      //{
      //     return false;
      // }
    }

    // recvframe_put(precvframe, pattrib.pkt_len);
    /* recvframe_pull(precvframe, drvinfo_sz + RXDESC_SIZE); */

    if (pattrib.pkt_rpt_type ==
        RX_PACKET_TYPE::NORMAL_RX) /* Normal rx packet */
    {
      ret.push_back({pattrib, pbuf.subspan(pattrib.shift_sz +
                                               pattrib.drvinfo_sz + RXDESC_SIZE,
                                           pattrib.pkt_len)});
      // pre_recv_entry(precvframe, pattrib.physt ? pbuf.Slice(RXDESC_OFFSET) :
      // null);
    } else {
      /* pkt_rpt_type == TX_REPORT1-CCX, TX_REPORT2-TX RTP,HIS_REPORT-USB HISR
       * RTP */
      if (pattrib.pkt_rpt_type == RX_PACKET_TYPE::C2H_PACKET) {
        _logger->info("RX USB C2H_PACKET");
        // rtw_hal_c2h_pkt_pre_hdl(padapter, precvframe.u.hdr.rx_data,
        // pattrib.pkt_len);
      } else if (pattrib.pkt_rpt_type == RX_PACKET_TYPE::HIS_REPORT) {
        _logger->info("RX USB HIS_REPORT");
      }
    }

    /* jaguar 8-byte alignment */
    pkt_offset = (uint16_t)_RND8(pkt_offset);
    // pkt_cnt--;

    if (pkt_offset >= pbuf.size()) {
      break;
    }
    pbuf = pbuf.subspan(pkt_offset, pbuf.size() - pkt_offset);
  } while (pbuf.size() > 0);

  if (pkt_cnt != 0) {
    _logger->info("Unprocessed packets: {}", pkt_cnt);
  }
  //_logger->info("{} received in frame", ret.size());

  return ret;
}
