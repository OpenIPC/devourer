#include "KestrelFw.h"

#include <chrono>
#include <cstring>
#include <thread>
#include <utility>

#include "MacRegAx.h"
#include "hal8852b_fw.h"

namespace kestrel {

namespace {
namespace r = kestrel::reg;

void delay_us(uint32_t us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

uint32_t le32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

void put_le32(uint8_t *p, uint32_t v) {
  p[0] = v & 0xFF;
  p[1] = (v >> 8) & 0xFF;
  p[2] = (v >> 16) & 0xFF;
  p[3] = (v >> 24) & 0xFF;
}

} /* namespace */

KestrelFw::KestrelFw(RtlAdapter device, Logger_t logger, ChipVariant variant)
    : _device{std::move(device)}, _logger{std::move(logger)},
      _variant{variant} {
  /* CH12 (H2C/FWDL) endpoint = BULKOUTID2 = the 3rd bulk-OUT in descriptor
   * order (vendor RtOutPipe[2]); on the 8852BU that is 0x07. */
  _ch12_ep = _device.nth_bulk_out_ep(r::BULKOUTID_H2C);
}

void KestrelFw::set32(uint16_t reg, uint32_t bits) {
  _device.rtw_write32(reg, _device.rtw_read32(reg) | bits);
}
void KestrelFw::clr32(uint16_t reg, uint32_t bits) {
  _device.rtw_write32(reg, _device.rtw_read32(reg) & ~bits);
}

bool KestrelFw::poll_wcpu(uint32_t mask, uint32_t expect, const char *what) {
  for (uint32_t cnt = r::FWDL_WAIT_CNT; cnt != 0; --cnt) {
    if ((_device.rtw_read32(r::R_AX_WCPU_FW_CTRL) & mask) == expect)
      return true;
    delay_us(1);
  }
  _logger->error("Kestrel FWDL: {} timeout (WCPU_FW_CTRL=0x{:08x})", what,
                 _device.rtw_read32(r::R_AX_WCPU_FW_CTRL));
  return false;
}

bool KestrelFw::hci_func_en() {
  /* hci_func_en (init.c:161): 8852A/8852B/8851B/8852BT take the FIRST branch —
   * RXDMA|TXDMA on R_AX_HCI_FUNC_EN (0x8380) ONLY. The _V1 register (0x7880) is
   * the ELSE branch for other chips; on the 8852B it is UNMAPPED (reads back
   * 0xdeadbeef), and writing there corrupts the HCI state so the running fw
   * faults on the AHB->HCI bus ~1 ms after boot (HALT_C2H = L2_ERR_AH_HCI). */
  set32(r::R_AX_HCI_FUNC_EN, r::B_AX_HCI_RXDMA_EN | r::B_AX_HCI_TXDMA_EN);
  return true;
}

bool KestrelFw::chk_dle_rdy(uint16_t status_reg, uint32_t rdy_bits,
                            const char *what) {
  for (uint32_t cnt = r::DLE_WAIT_CNT; cnt != 0; --cnt) {
    if ((_device.rtw_read32(status_reg) & rdy_bits) == rdy_bits)
      return true;
    delay_us(r::DLE_WAIT_US);
  }
  _logger->error("Kestrel FWDL: {} not ready (0x{:04x}=0x{:08x})", what,
                 status_reg, _device.rtw_read32(status_reg));
  return false;
}

bool KestrelFw::dle_init_dlfw() {
  /* dle_init(MAC_AX_QTA_DLFW) for 8852B USB: config wde_size9/ple_size8,
   * wde_qt4(wcpu=6 SCC override)/ple_qt13(c2h=16,h2c=48). */
  auto field = [](uint32_t v, uint32_t val, uint32_t msk, uint8_t sh) {
    return r::set_clr_word(v, val, msk, sh);
  };

  /* dle_func_en(DIS). */
  clr32(r::R_AX_DMAC_FUNC_EN, r::B_AX_DLE_WDE_EN | r::B_AX_DLE_PLE_EN);

  /* dle_mix_cfg: WDE 64B page, bound 0, free=lnk_pge_num(0). */
  uint32_t v = _device.rtw_read32(r::R_AX_WDE_PKTBUF_CFG);
  v = field(v, r::S_AX_WDE_PAGE_SEL_64, r::B_AX_WDE_PAGE_SEL_MSK,
            r::B_AX_WDE_PAGE_SEL_SH);
  v = field(v, 0, r::B_AX_WDE_START_BOUND_MSK, r::B_AX_WDE_START_BOUND_SH);
  v = field(v, r::DLFW_WDE_FREE_PAGE, r::B_AX_WDE_FREE_PAGE_NUM_MSK,
            r::B_AX_WDE_FREE_PAGE_NUM_SH);
  _device.rtw_write32(r::R_AX_WDE_PKTBUF_CFG, v);

  /* PLE 128B page, bound = (lnk+unlnk)*pge_size/DLE_BOUND_UNIT of the WDE
   * region = (0+1024)*64/8192 = 8; free=lnk_pge_num(64). */
  const uint32_t bound =
      (r::DLFW_WDE_FREE_PAGE + r::DLFW_WDE_UNLNK_PAGE) * 64u / r::DLE_BOUND_UNIT;
  v = _device.rtw_read32(r::R_AX_PLE_PKTBUF_CFG);
  v = field(v, r::S_AX_PLE_PAGE_SEL_128, r::B_AX_PLE_PAGE_SEL_MSK,
            r::B_AX_PLE_PAGE_SEL_SH);
  v = field(v, bound, r::B_AX_PLE_START_BOUND_MSK, r::B_AX_PLE_START_BOUND_SH);
  v = field(v, r::DLFW_PLE_FREE_PAGE, r::B_AX_PLE_FREE_PAGE_NUM_MSK,
            r::B_AX_PLE_FREE_PAGE_NUM_SH);
  _device.rtw_write32(r::R_AX_PLE_PKTBUF_CFG, v);

  /* quota cfg: min in [11:0], max in [27:16]. */
  auto qta = [&](uint16_t reg, uint16_t mn, uint16_t mx) {
    _device.rtw_write32(
        reg, (static_cast<uint32_t>(mn & r::QTA_SIZE_MSK) << r::QTA_MIN_SH) |
                 (static_cast<uint32_t>(mx & r::QTA_SIZE_MSK) << r::QTA_MAX_SH));
  };
  qta(r::R_AX_WDE_QTA0_CFG, 0, 0);                     /* hif */
  qta(r::R_AX_WDE_QTA1_CFG, r::DLFW_WDE_WCPU_MIN, 0);  /* wcpu=6 */
  qta(r::R_AX_WDE_QTA3_CFG, 0, 0);                     /* pkt_in */
  qta(r::R_AX_WDE_QTA4_CFG, 0, 0);                     /* cpu_io */
  /* PLE QTA0..QTA10: only c2h(Q2)=16 and h2c(Q3)=48 are nonzero. */
  for (uint16_t reg = r::R_AX_PLE_QTA0_CFG; reg <= r::R_AX_PLE_QTA10_CFG;
       reg += 4) {
    if (reg == r::R_AX_PLE_QTA2_CFG)
      qta(reg, r::DLFW_PLE_C2H, r::DLFW_PLE_C2H);
    else if (reg == r::R_AX_PLE_QTA3_CFG)
      qta(reg, r::DLFW_PLE_H2C, r::DLFW_PLE_H2C);
    else
      qta(reg, 0, 0);
  }

  /* dle_func_en(EN) + ready polls. */
  set32(r::R_AX_DMAC_FUNC_EN, r::B_AX_DLE_WDE_EN | r::B_AX_DLE_PLE_EN);
  if (!chk_dle_rdy(r::R_AX_WDE_INI_STATUS,
                   r::B_AX_WDE_Q_MGN_INI_RDY | r::B_AX_WDE_BUF_MGN_INI_RDY,
                   "WDE"))
    return false;
  if (!chk_dle_rdy(r::R_AX_PLE_INI_STATUS,
                   r::B_AX_PLE_Q_MGN_INI_RDY | r::B_AX_PLE_BUF_MGN_INI_RDY,
                   "PLE"))
    return false;
  return true;
}

bool KestrelFw::hfc_init_dlfw() {
  /* hfc_init(rst=1, en=0, h2c_en=1): reset is software-only (no register
   * writes for the h2c-only path); then set_fc_func_en(0,0), set_fc_h2c,
   * set_fc_func_en(0,1). */
  /* set_fc_func_en(0, 0): clear FC_EN + CH12_EN. */
  clr32(r::R_AX_HCI_FC_CTRL, r::B_AX_HCI_FC_EN | r::B_AX_HCI_FC_CH12_EN);
  /* set_fc_h2c: CH_PAGE_CTRL = h2c_prec<<16; HCI_FC_CTRL CH12_FULL_COND=X2. */
  _device.rtw_write32(
      r::R_AX_CH_PAGE_CTRL,
      (static_cast<uint32_t>(r::HFC_USB_H2C_PREC_8852B & r::B_AX_PREC_PAGE_CH12_MSK)
       << r::B_AX_PREC_PAGE_CH12_SH));
  _device.rtw_write32(
      r::R_AX_HCI_FC_CTRL,
      r::set_clr_word(_device.rtw_read32(r::R_AX_HCI_FC_CTRL),
                      r::HFC_FULL_COND_X2, r::B_AX_HCI_FC_CH12_FULL_COND_MSK,
                      r::B_AX_HCI_FC_CH12_FULL_COND_SH));
  /* set_fc_func_en(0, 1): clear FC_EN, set CH12_EN. */
  clr32(r::R_AX_HCI_FC_CTRL, r::B_AX_HCI_FC_EN);
  set32(r::R_AX_HCI_FC_CTRL, r::B_AX_HCI_FC_CH12_EN);
  return true;
}

bool KestrelFw::dmac_pre_init() {
  /* dmac_func_pre_en (init_8852b.c). */
  set32(r::R_AX_DMAC_FUNC_EN, r::B_AX_MAC_FUNC_EN | r::B_AX_DMAC_FUNC_EN |
                                  r::B_AX_DISPATCHER_EN | r::B_AX_PKT_BUF_EN);
  if (!dle_init_dlfw())
    return false;
  if (!hfc_init_dlfw())
    return false;
  return true;
}

bool KestrelFw::disable_cpu() {
  /* mac_disable_cpu. */
  clr32(r::R_AX_PLATFORM_ENABLE, r::B_AX_WCPU_EN);
  clr32(r::R_AX_WCPU_FW_CTRL,
        r::B_AX_WCPU_FWDL_EN | r::B_AX_H2C_PATH_RDY | r::B_AX_FWDL_PATH_RDY);
  clr32(r::R_AX_SYS_CLK_CTRL, r::B_AX_CPU_CLK_EN);
  /* 8852B needs the APB-wrap reset (resets CPU-local CR incl. WDT). */
  clr32(r::R_AX_PLATFORM_ENABLE, r::B_AX_APB_WRAP_EN);
  set32(r::R_AX_PLATFORM_ENABLE, r::B_AX_APB_WRAP_EN);
  return true;
}

bool KestrelFw::enable_cpu(uint8_t boot_reason) {
  /* mac_enable_cpu(boot_reason, dlfw=1). */
  if (_device.rtw_read32(r::R_AX_PLATFORM_ENABLE) & r::B_AX_WCPU_EN) {
    _logger->error("Kestrel FWDL: WCPU already enabled");
    return false;
  }
  _device.rtw_write32(r::R_AX_LDM, 0);      /* trim FW debug log */
  (void)_device.rtw_read32(r::R_AX_UDM0);
  /* clear SER status */
  _device.rtw_write32(r::R_AX_HALT_H2C_CTRL, 0);
  _device.rtw_write32(r::R_AX_HALT_C2H_CTRL, 0);
  _device.rtw_write32(r::R_AX_HALT_H2C, 0);
  _device.rtw_write32(r::R_AX_HALT_C2H, 0);
  /* write-1-clear HISR0 */
  _device.rtw_write32(r::R_AX_HISR0, _device.rtw_read32(r::R_AX_HISR0));
  set32(r::R_AX_SYS_CLK_CTRL, r::B_AX_CPU_CLK_EN);

  uint32_t v = _device.rtw_read32(r::R_AX_WCPU_FW_CTRL);
  v &= ~(r::B_AX_WCPU_FWDL_EN | r::B_AX_H2C_PATH_RDY | r::B_AX_FWDL_PATH_RDY);
  v = r::set_clr_word(v, r::FWDL_INITIAL_STATE, r::B_AX_WCPU_FWDL_STS_MSK,
                      r::B_AX_WCPU_FWDL_STS_SH);
  v |= r::B_AX_WCPU_FWDL_EN; /* dlfw=1 */
  _device.rtw_write32(r::R_AX_WCPU_FW_CTRL, v);

  uint16_t v16 = _device.rtw_read16(r::R_AX_BOOT_REASON);
  v16 = static_cast<uint16_t>(r::set_clr_word(v16, boot_reason,
                                              r::B_AX_BOOT_REASON_MSK,
                                              r::B_AX_BOOT_REASON_SH));
  _device.rtw_write16(r::R_AX_BOOT_REASON, v16);

  /* fwdl_precheck: on 8852B/MIPS/USB the CH12 index check is skipped; the PLE
   * queue-empty check (dle_dfi_qempty) is a defensive guard that always holds
   * on a fresh cold power-on (nothing queued yet), so it is intentionally
   * omitted here — see docs; if a warm re-init path is added it must return. */

  set32(r::R_AX_PLATFORM_ENABLE, r::B_AX_WCPU_EN);
  return true;
}

bool KestrelFw::send_fwdl_packet(const uint8_t *payload, uint32_t payload_len,
                                 bool is_header, uint8_t seq) {
  /* Packet = [WD body 24B] (+ [fwcmd_hdr 8B] for the header) + payload.
   * data_len (what the WD TXPKTSIZE and H2C total-len count) = the bytes after
   * the WD: fwcmd_hdr + payload for the header, or payload for a section. */
  const uint32_t data_len =
      is_header ? (r::FWCMD_HDR_LEN + payload_len) : payload_len;
  _txbuf.assign(r::WD_BODY_LEN + data_len, 0);
  uint8_t *wd = _txbuf.data();

  /* WD body dword0: CH_DMA = H2C(12); FWDL_EN for section packets. */
  uint32_t dw0 = static_cast<uint32_t>(r::MAC_AX_DMA_H2C & r::AX_TXD_CH_DMA_MSK)
                 << r::AX_TXD_CH_DMA_SH;
  if (!is_header)
    dw0 |= r::AX_TXD_FWDL_EN;
  put_le32(wd + 0, dw0);
  put_le32(wd + 8, (data_len & r::AX_TXD_TXPKTSIZE_MSK) << r::AX_TXD_TXPKTSIZE_SH);
  /* dword1,3,4,5 already zero. */

  uint8_t *after = wd + r::WD_BODY_LEN;
  if (is_header) {
    /* fwcmd_hdr (8B): hdr0 = cat|class|func|type|seq; hdr1 = total_len.
     * TOTAL_LEN counts the fwcmd header + payload (h2cb->len) — the vendor's
     * FWDL-header packet on the wire carries 0x58 = 80B header + 8. */
    uint32_t hdr0 =
        (static_cast<uint32_t>(r::FWCMD_H2C_CAT_MAC) << r::H2C_HDR_CAT_SH) |
        (static_cast<uint32_t>(r::FWCMD_H2C_CL_FWDL) << r::H2C_HDR_CLASS_SH) |
        (static_cast<uint32_t>(r::FWCMD_H2C_FUNC_FWHDR_DL) << r::H2C_HDR_FUNC_SH) |
        (static_cast<uint32_t>(r::FWCMD_TYPE_H2C) << r::H2C_HDR_DEL_TYPE_SH) |
        (static_cast<uint32_t>(seq) << r::H2C_HDR_H2C_SEQ_SH);
    put_le32(after + 0, hdr0);
    put_le32(after + 4, data_len << r::H2C_HDR_TOTAL_LEN_SH);
    std::memcpy(after + r::FWCMD_HDR_LEN, payload, payload_len);
  } else {
    std::memcpy(after, payload, payload_len);
  }

  /* tx_sync returns bytes-transferred on success (>=0), negative libusb rc on
   * error. */
  int rc = _device.bulk_send_sync_ep(_ch12_ep, _txbuf.data(),
                                     _txbuf.size(), 1000);
  if (rc < 0 || static_cast<size_t>(rc) != _txbuf.size()) {
    _logger->error("Kestrel FWDL: bulk-out to ep 0x{:02x} failed (rc={}, "
                   "wanted {})",
                   _ch12_ep, rc, _txbuf.size());
    return false;
  }
  return true;
}

bool KestrelFw::send_h2c_cmd(uint8_t cat, uint8_t h2c_class, uint8_t func,
                             const uint8_t *content, uint32_t len) {
  /* Packet = [WD 24B][fwcmd_hdr 8B][content]. Same framing as the FWDL header
   * path but with a caller-supplied cat/class/func and the content inline
   * (no FWDL_EN). */
  _txbuf.assign(r::WD_BODY_LEN + r::FWCMD_HDR_LEN + len, 0);
  uint8_t *wd = _txbuf.data();
  const uint32_t data_len = r::FWCMD_HDR_LEN + len;

  uint32_t dw0 = static_cast<uint32_t>(r::MAC_AX_DMA_H2C & r::AX_TXD_CH_DMA_MSK)
                 << r::AX_TXD_CH_DMA_SH;
  put_le32(wd + 0, dw0);
  put_le32(wd + 8,
           (data_len & r::AX_TXD_TXPKTSIZE_MSK) << r::AX_TXD_TXPKTSIZE_SH);

  uint8_t *hdr = wd + r::WD_BODY_LEN;
  /* Per-H2C sequence number (fwinfo->h2c_seq): 8-bit rolling counter
   * (H2C_HDR_H2C_SEQ [31:24], msk 0xff), incremented per runtime H2C across
   * ALL classes (h2c_pkt_set_hdr). The golden capture counts 0,1,2,... from
   * the first runtime H2C. */
  const uint8_t seq = _h2c_seq;
  _h2c_seq++;
  uint32_t hdr0 = (static_cast<uint32_t>(cat) << r::H2C_HDR_CAT_SH) |
                  (static_cast<uint32_t>(h2c_class) << r::H2C_HDR_CLASS_SH) |
                  (static_cast<uint32_t>(func) << r::H2C_HDR_FUNC_SH) |
                  (static_cast<uint32_t>(r::FWCMD_TYPE_H2C) << r::H2C_HDR_DEL_TYPE_SH) |
                  (static_cast<uint32_t>(seq) << r::H2C_HDR_H2C_SEQ_SH);
  put_le32(hdr + 0, hdr0);
  /* TOTAL_LEN counts the fwcmd header + content (h2cb->len in the vendor
   * h2c_pkt_set_hdr), NOT just the content — the fw's H2C-queue parser uses it
   * to find the next packet boundary; an 8-byte-short value desyncs the queue. */
  put_le32(hdr + 4, data_len << r::H2C_HDR_TOTAL_LEN_SH);
  std::memcpy(hdr + r::FWCMD_HDR_LEN, content, len);

  /* One-shot hexdump of the first FW_OFLD packet for golden byte-diff vs the
   * vendor .ko cmd_ofld capture (WD 24B + fwcmd hdr 8B + command dwords). */
  static bool dumped = false;
  if (!dumped && h2c_class == r::FWCMD_H2C_CL_FW_OFLD) {
    dumped = true;
    std::string hx;
    const size_t n = std::min<size_t>(_txbuf.size(), 64);
    char b[4];
    for (size_t i = 0; i < n; ++i) {
      std::snprintf(b, sizeof(b), "%02x", _txbuf[i]);
      hx += b;
      if ((i & 3) == 3) hx += ' ';
    }
    _logger->info("Kestrel cmd_ofld pkt[{}B] first64: {}", _txbuf.size(), hx);
  }

  int rc = _device.bulk_send_sync_ep(_ch12_ep, _txbuf.data(), _txbuf.size(), 1000);
  if (rc < 0 || static_cast<size_t>(rc) != _txbuf.size()) {
    _logger->error("Kestrel H2C: bulk-out to ep 0x{:02x} failed (rc={}, class={}"
                   " func={})",
                   _ch12_ep, rc, h2c_class, func);
    return false;
  }
  return true;
}

void KestrelFw::ofld_begin() {
  _ofld_buf.clear();
  _ofld_cmd_num = 0;
  _ofld_accu_delay_us = 0;
}

void KestrelFw::ofld_append(uint8_t src, uint8_t type, uint8_t path,
                            uint16_t offset, uint32_t value, uint32_t mask,
                            bool lc) {
  /* add_cmd (fwofld.c:609): one 16-byte fwcmd_cmd_ofld. RF_DDIE is sent as
   * SRC_RF with dword1 base-offset = 1. */
  const uint8_t rf_ddie = (src == r::OFLD_SRC_RF_DDIE) ? 1 : 0;
  const uint8_t real_src = (src == r::OFLD_SRC_RF_DDIE) ? r::OFLD_SRC_RF : src;

  uint32_t dw0 =
      (static_cast<uint32_t>(real_src) << r::CMD_OFLD_SRC_SH) |
      (static_cast<uint32_t>(type) << r::CMD_OFLD_TYPE_SH) |
      (lc ? r::CMD_OFLD_LC : 0) |
      (static_cast<uint32_t>(path & 0x3) << r::CMD_OFLD_PATH_SH) |
      (static_cast<uint32_t>(_ofld_cmd_num & 0x7f) << r::CMD_OFLD_CMD_NUM_SH) |
      (static_cast<uint32_t>(offset) << r::CMD_OFLD_OFFSET_SH);
  /* dword1: id (0) | the "base offset" field at the same sh16: RF serial = 0,
   * RF D-die = 1; otherwise GET_FIELD(offset, OFFSET) = the high 16 offset
   * bits (0 for a 16-bit BB/MAC addr). */
  uint32_t off2 = (real_src == r::OFLD_SRC_RF)
                      ? rf_ddie
                      : (static_cast<uint32_t>(offset) >> 16);
  uint32_t dw1 = off2 << r::CMD_OFLD_OFFSET_SH;

  size_t o = _ofld_buf.size();
  _ofld_buf.resize(o + r::CMD_OFLD_SIZE);
  uint8_t *p = _ofld_buf.data() + o;
  put_le32(p + 0, dw0);
  put_le32(p + 4, dw1);
  put_le32(p + 8, value);
  put_le32(p + 12, mask);
  ++_ofld_cmd_num;
  if (type == r::OFLD_TYPE_DELAY)
    _ofld_accu_delay_us += value;
}

bool KestrelFw::ofld_send_batch() {
  /* Per-batch trace: decode the first + last command's (src,offset) so a fw
   * SER mid-table can be pinned to a register range. */
  if (!_ofld_buf.empty()) {
    auto decode = [&](size_t o) {
      uint32_t d0 = le32(_ofld_buf.data() + o);
      uint8_t src = d0 & 0x3;
      uint16_t off = static_cast<uint16_t>((d0 >> r::CMD_OFLD_OFFSET_SH) & 0xffff);
      return std::pair<uint8_t, uint16_t>(src, off);
    };
    auto [fs, fo] = decode(0);
    auto [ls, lo] = decode(_ofld_buf.size() - r::CMD_OFLD_SIZE);
    _logger->debug("Kestrel cmd_ofld batch: {} cmds, first src{}@0x{:04x} "
                   "last src{}@0x{:04x}",
                   _ofld_cmd_num, fs, fo, ls, lo);
  }
  bool ok = send_h2c_cmd(r::FWCMD_H2C_CAT_MAC, r::FWCMD_H2C_CL_FW_OFLD,
                         r::FWCMD_H2C_FUNC_CMD_OFLD_REG, _ofld_buf.data(),
                         static_cast<uint32_t>(_ofld_buf.size()));
  /* proc_cmd_ofld: send -> host-sleep the accumulated DELAY_OFLD time ->
   * chk_cmd_ofld (c2hreg poll while RX is down). The result ack releases the
   * fw to return the H2C page; without it the pool leaks dry and CH12 jams. */
  if (ok) {
    if (_ofld_accu_delay_us)
      delay_us(_ofld_accu_delay_us);
    poll_cmd_ofld_result();
  }
  _ofld_buf.clear();
  _ofld_cmd_num = 0;
  _ofld_accu_delay_us = 0;
  return ok;
}

void KestrelFw::ofld_write(uint8_t src, uint8_t type, uint8_t path,
                           uint16_t offset, uint32_t value, uint32_t mask) {
  /* ofld_incompatible_full_cmd (fwofld.c:1026): when the 2000-byte buffer is
   * full, force LC on the LAST buffered command and send — then buffer the
   * new command into a fresh batch. */
  if (_ofld_buf.size() + r::CMD_OFLD_SIZE > r::CMD_OFLD_MAX_LEN) {
    uint8_t *last = _ofld_buf.data() + (_ofld_buf.size() - r::CMD_OFLD_SIZE);
    put_le32(last, le32(last) | r::CMD_OFLD_LC);
    ofld_send_batch();
  }
  ofld_append(src, type, path, offset, value, mask, /*lc=*/false);
}

bool KestrelFw::ofld_flush(OfldFlush style) {
  /* Section-end flush: append the vendor's harmless LC sentinel — halbb ends
   * with a BB write to 0x1a24 (mask 0xff, val 0) (halbb_fwofld.c:93); halrf
   * ends with a 1 us DELAY_OFLD (halrf_write_fwofld_trigger). Sent even when
   * the buffer is empty (vendor emits the sentinel unconditionally). */
  if (_ofld_buf.size() + r::CMD_OFLD_SIZE > r::CMD_OFLD_MAX_LEN) {
    uint8_t *last = _ofld_buf.data() + (_ofld_buf.size() - r::CMD_OFLD_SIZE);
    put_le32(last, le32(last) | r::CMD_OFLD_LC);
    if (!ofld_send_batch())
      return false;
  }
  if (style == OfldFlush::BB)
    ofld_append(r::OFLD_SRC_BB, r::OFLD_TYPE_WRITE, 0, r::BB_OFLD_FLUSH_ADDR,
                0, r::BB_OFLD_FLUSH_MASK, /*lc=*/true);
  else
    ofld_append(r::OFLD_SRC_BB /* cmd = {0}: src stays 0 */, r::OFLD_TYPE_DELAY,
                0, 0, /*value=*/1, 0, /*lc=*/true);
  return ofld_send_batch();
}

bool KestrelFw::poll_cmd_ofld_result() {
  /* poll_c2hreg + __recv_c2hreg: poll R_AX_C2HREG_CTRL for the TRIGGER bit,
   * drain data0..3, then clear TRIGGER (the ack). */
  for (uint32_t cnt = r::CMD_OFLD_POLL_CNT; cnt != 0; --cnt) {
    if (_device.rtw_read8(r::R_AX_C2HREG_CTRL) & r::B_AX_C2HREG_TRIGGER) {
      const uint32_t d0 = _device.rtw_read32(r::R_AX_C2HREG_DATA0);
      const uint32_t d1 = _device.rtw_read32(r::R_AX_C2HREG_DATA1);
      const uint32_t d2 = _device.rtw_read32(r::R_AX_C2HREG_DATA2);
      const uint32_t d3 = _device.rtw_read32(r::R_AX_C2HREG_DATA3);
      uint8_t ctrl = _device.rtw_read8(r::R_AX_C2HREG_CTRL);
      _device.rtw_write8(r::R_AX_C2HREG_CTRL,
                         static_cast<uint8_t>(ctrl & ~r::B_AX_C2HREG_TRIGGER));
      /* chk_cmd_ofld_reg: dword0 carries RET[23:16] (0=ok) and the failing
       * CMD_NUM[31:24]; dword1=offset, dword2=exp val, dword3=read val. A
       * nonzero RET means the fw REJECTED a command in the batch (e.g. an
       * unwritable/guarded register) — the batch did not fully apply. */
      const uint8_t ret = static_cast<uint8_t>((d0 >> 16) & 0xff);
      const uint8_t cmd_num = static_cast<uint8_t>((d0 >> 24) & 0xff);
      if (ret != 0) {
        _logger->warn("Kestrel H2C: cmd_ofld FW REJECT ret={} cmd_num={} "
                      "offset=0x{:04x} exp=0x{:08x} read=0x{:08x}",
                      ret, cmd_num, d1 & 0xffff, d2, d3);
        return false;
      }
      _logger->debug("Kestrel H2C: cmd_ofld result ok (d0=0x{:08x})", d0);
      return true;
    }
    delay_us(r::CMD_OFLD_POLL_US);
  }
  /* Distinguish "fw ignores the batch" from "fw crashed/SER". The SER error is
   * only valid when HALT_C2H_CTRL is set (mac_get_err_status gates on it);
   * HALT_C2H then carries the mac_ax_err_info code (0x2010 = L2_ERR_AH_HCI,
   * 0x1000 = L1_ERR_DMAC, ...). */
  const uint32_t ctrl = _device.rtw_read32(r::R_AX_HALT_C2H_CTRL);
  const uint32_t err = _device.rtw_read32(r::R_AX_HALT_C2H);
  _logger->warn("Kestrel H2C: cmd_ofld C2H-result poll timeout — "
                "HALT_C2H_CTRL=0x{:08x} HALT_C2H=0x{:08x}{} UDM0=0x{:08x} "
                "WCPU=0x{:08x} BOOT_DBG=0x{:08x}",
                ctrl, err, ctrl ? " (SER latched)" : " (stale)",
                _device.rtw_read32(r::R_AX_UDM0),
                _device.rtw_read32(r::R_AX_WCPU_FW_CTRL),
                _device.rtw_read32(r::R_AX_BOOT_DBG));
  return false;
}

bool KestrelFw::radio_page_to_fw(uint8_t cls, uint8_t page,
                                 const uint32_t *packed, uint16_t count) {
  std::vector<uint8_t> buf(static_cast<size_t>(count) * 4);
  for (uint16_t i = 0; i < count; ++i)
    put_le32(buf.data() + i * 4, packed[i]);
  bool ok = send_h2c_cmd(r::FWCMD_H2C_CAT_OUTSRC, cls, page, buf.data(),
                         static_cast<uint32_t>(buf.size()));
  delay_us(200);
  return ok;
}

bool KestrelFw::check_fw_rdy() {
  /* Poll WCPU_FWDL_STS [7:5] until FW_INIT_RDY(7); surface fail codes. */
  for (uint32_t cnt = r::FWDL_WAIT_CNT; cnt != 0; --cnt) {
    uint8_t sts = static_cast<uint8_t>(
        (_device.rtw_read8(r::R_AX_WCPU_FW_CTRL) >> r::B_AX_WCPU_FWDL_STS_SH) &
        r::B_AX_WCPU_FWDL_STS_MSK);
    if (sts == r::FWDL_WCPU_FW_INIT_RDY)
      return true;
    if (sts == r::FWDL_CHECKSUM_FAIL) {
      _logger->error("Kestrel FWDL: checksum fail");
      return false;
    }
    if (sts == r::FWDL_SECURITY_FAIL) {
      _logger->error("Kestrel FWDL: security fail");
      return false;
    }
    if (sts == r::FWDL_CUT_NOT_MATCH) {
      _logger->error("Kestrel FWDL: cut mismatch (wrong FW image)");
      return false;
    }
    delay_us(1);
  }
  _logger->error("Kestrel FWDL: fw-ready poll timeout (boot_dbg=0x{:08x})",
                 _device.rtw_read32(r::R_AX_BOOT_DBG));
  return false;
}

void KestrelFw::idmem_share_mode_check() {
  /* idmem_share_mode_check (fwdl.c:280), non-secure branch: force the SEC_CTRL
   * IDMEM-share field to the AX-MIPS default (0x1). Golden capture:
   * 0x0C00 -> 0x0001c01f. */
  uint32_t v = _device.rtw_read32(r::R_AX_SEC_CTRL);
  uint32_t cur = (v >> r::B_SEC_IDMEM_SIZE_CONFIG_SH) & r::B_SEC_IDMEM_SIZE_CONFIG_MSK;
  if (cur != r::FWDL_IDMEM_SHARE_DEFAULT_MODE) {
    v = r::set_clr_word(v, r::FWDL_IDMEM_SHARE_DEFAULT_MODE,
                        r::B_SEC_IDMEM_SIZE_CONFIG_MSK,
                        r::B_SEC_IDMEM_SIZE_CONFIG_SH);
    _device.rtw_write32(r::R_AX_SEC_CTRL, v);
    _logger->info("Kestrel FWDL: IDMEM share -> default (SEC_CTRL=0x{:08x})", v);
  }
}

void KestrelFw::fwdl_patch_fw_delay() {
  /* fwdl_patch_fw_delay (fwdl.c:1176), non-secure branch: the fw's own CPU-clk
   * setting is wrong, which makes its secure-checksum delay (and every other
   * timed HCI access) inaccurate — the running fw then faults on the AHB->HCI
   * bus (HALT_C2H = L2_ERR_AH_HCI). Fix it by poking the correct clk value into
   * IDMEM via the indirect-access window. Golden: 0x0C04 -> 0x18e0c3d8, then
   * 0x40000 (wIndex=4) -> 0x0000000a. */
  _device.rtw_write32(r::R_AX_FILTER_MODEL_ADDR, r::FW_CPU_CLK_ADDR_8852B);
  _device.rtw_write32_wide(r::R_AX_INDIR_ACCESS_ENTRY, r::FW_FAKE_CPU_CLK_8852B);
  _logger->info("Kestrel FWDL: fw CPU-clk patch (IDMEM 0x{:08x}=0x{:x})",
                r::FW_CPU_CLK_ADDR_8852B, r::FW_FAKE_CPU_CLK_8852B);
}

bool KestrelFw::mac_fwdl(const uint8_t *fw, uint32_t len, uint8_t mss_idx) {
  _device.rtw_write32(r::R_AX_UDM1, 0);
  if (len < r::FWHDR_HDR_LEN) {
    _logger->error("Kestrel FWDL: fw too short ({})", len);
    return false;
  }

  /* Parse the fw header (fwhdr_hdr_parser). */
  const uint32_t sec_num =
      (le32(fw + 24) >> r::FWHDR_SEC_NUM_SH) & r::FWHDR_SEC_NUM_MSK; /* dword6 */
  const uint32_t hdr_len =
      (le32(fw + 12) >> r::FWHDR_FWHDR_SZ_SH) & r::FWHDR_FWHDR_SZ_MSK; /* dw3 */
  const uint32_t dyn_hdr_en =
      (le32(fw + 28) >> r::FWHDR_FW_DYN_HDR_SH) & r::FWHDR_FW_DYN_HDR_MSK; /*dw7*/
  const uint32_t dynamic_hdr_len =
      dyn_hdr_en ? (hdr_len - (r::FWHDR_HDR_LEN + sec_num * r::FWHDR_SECTION_LEN))
                 : 0;
  _logger->info("Kestrel FWDL: sec_num={} hdr_len={} dyn={} mss_idx={}",
                sec_num, hdr_len, dyn_hdr_en, mss_idx);

  /* Parse section headers. Each section's payload follows the full header
   * block contiguously (bin_ptr starts at fw + hdr_len). A type-9 security
   * section with mssc>0 carries mssc appended SIGLEN signatures at the tail of
   * the image (outside the sections); the selected one is patched into the
   * security-section body before download (fwhdr_parser MSS handling). */
  struct Sec {
    const uint8_t *addr;
    uint32_t len;
    bool security;
  };
  std::vector<Sec> secs;
  uint32_t total_mss_bytes = 0; /* trailing signature bytes (not downloaded) */
  const uint8_t *bin_ptr = fw + hdr_len;
  const uint8_t *sh = fw + r::FWHDR_HDR_LEN;
  const uint8_t *sec_body = nullptr; /* the security section's downloaded body */
  for (uint32_t i = 0; i < sec_num; ++i, sh += r::FWHDR_SECTION_LEN) {
    uint32_t dw1 = le32(sh + 4);
    uint32_t sec_len =
        (dw1 >> r::SECTION_INFO_SEC_SIZE_SH) & r::SECTION_INFO_SEC_SIZE_MSK;
    if (dw1 & r::SECTION_INFO_CHECKSUM)
      sec_len += r::FWDL_SECTION_CHKSUM_LEN;
    uint8_t type =
        (dw1 >> r::SECTION_INFO_SECTIONTYPE_SH) & r::SECTION_INFO_SECTIONTYPE_MSK;
    bool security = (type == r::FWDL_SECURITY_SECTION_TYPE);
    if (security) {
      sec_len = 2048; /* 8852B workaround: forced length */
      total_mss_bytes += le32(sh + 8) * r::FWDL_SECURITY_SIGLEN; /* mssc*512 */
      sec_body = bin_ptr;
    }
    secs.push_back({bin_ptr, sec_len, security});
    bin_ptr += sec_len;
  }
  /* Image = header + sections + trailing MSS signatures. */
  if (static_cast<uint32_t>(bin_ptr - fw) + total_mss_bytes != len) {
    _logger->error("Kestrel FWDL: sections 0x{:x} + mss 0x{:x} != image 0x{:x}",
                   static_cast<uint32_t>(bin_ptr - fw), total_mss_bytes, len);
    return false;
  }

  /* Patch the selected MSS signature into a mutable copy of the security
   * section body. The trailing signatures start at bin_ptr (right after the
   * last section); the selected one is at +mss_idx*SIGLEN. It is copied to
   * offset FWDL_SECURITY_SECTION_CONSTANT within the section body. */
  std::vector<uint8_t> sec_patched;
  if (sec_body && total_mss_bytes) {
    constexpr uint32_t kSecConst = 64 + (6 * 32 * 2); /* =448 (SECTION_MAX=6) */
    const uint8_t *sig = bin_ptr + mss_idx * r::FWDL_SECURITY_SIGLEN;
    sec_patched.assign(sec_body, sec_body + 2048);
    std::memcpy(sec_patched.data() + kSecConst, sig, r::FWDL_SECURITY_SIGLEN);
    for (Sec &s : secs)
      if (s.security)
        s.addr = sec_patched.data();
  }

  /* Retry loop (FWDL_TRY_CNT = 3 for AX MIPS). */
  for (uint8_t attempt = 0; attempt < r::FWDL_TRY_CNT_MIPS; ++attempt) {
    /* phase0: wait H2C path ready. */
    if (!poll_wcpu(r::B_AX_H2C_PATH_RDY, r::B_AX_H2C_PATH_RDY, "H2C_PATH_RDY"))
      goto retry;
    /* fwdl_patch_fw_delay (fwdl.c:1260): between phase0 and phase1, on a
     * non-secure IC, patch the fw CPU clock. */
    if (!_is_sec_ic)
      fwdl_patch_fw_delay();
    /* phase1: download the fw header (minus the dynamic header) as one H2C —
     * fwdl_phase1 is called with (hdr_len - dynamic_hdr_len) in the vendor
     * (fwdl.c:1685/1746); the dynamic header is host-side cap metadata, not
     * part of the FWHDR_DL. */
    if (!send_fwdl_packet(fw, hdr_len - dynamic_hdr_len, /*is_header=*/true, 0))
      goto retry;
    if (!poll_wcpu(r::B_AX_FWDL_PATH_RDY, r::B_AX_FWDL_PATH_RDY, "FWDL_PATH_RDY"))
      goto retry;
    _device.rtw_write32(r::R_AX_HALT_H2C_CTRL, 0);
    _device.rtw_write32(r::R_AX_HALT_C2H_CTRL, 0);
    /* phase2: download each section in <=2020-byte chunks. */
    {
      bool ok = true;
      for (const Sec &s : secs) {
        uint32_t off = 0;
        while (off < s.len && ok) {
          uint32_t chunk = s.len - off;
          if (chunk > r::FWDL_SECTION_PER_PKT_LEN)
            chunk = r::FWDL_SECTION_PER_PKT_LEN;
          ok = send_fwdl_packet(s.addr + off, chunk, /*is_header=*/false, 0);
          off += chunk;
        }
        if (!ok)
          break;
      }
      if (!ok)
        goto retry;
    }
    if (!check_fw_rdy())
      goto retry;
    _logger->info("Kestrel FWDL: firmware booted (attempt {})", attempt + 1);
    return true;
  retry:
    _logger->warn("Kestrel FWDL: attempt {} failed", attempt + 1);
    if (attempt + 1 < r::FWDL_TRY_CNT_MIPS) {
      disable_cpu();
      if (!enable_cpu(r::AX_BOOT_REASON_PWR_ON))
        return false;
    }
  }
  _logger->error("Kestrel FWDL: exhausted {} attempts", r::FWDL_TRY_CNT_MIPS);
  return false;
}

bool KestrelFw::fw_pre_init() {
  /* mac_hal_init pre-FWDL: hci_func_en (init.c:406) + dmac_pre_init (DLFW
   * DLE/HFC, init.c:414). The caller runs usb_pre_init (intf_pre_init) after
   * this, then download_firmware — vendor order. */
  if (_ch12_ep == 0) {
    _logger->error("Kestrel FWDL: no CH12 bulk-OUT endpoint (need >=3 bulk-out; "
                   "found {})",
                   _device.bulk_out_ep_count());
    return false;
  }
  return hci_func_en() && dmac_pre_init();
}

bool KestrelFw::download_firmware(uint8_t cut, uint8_t mss_idx, bool is_sec_ic) {
  _is_sec_ic = is_sec_ic;
  /* NICCE image, cut-selected: CBV(1) -> u2, CCV+(>=2) -> u3. */
  const uint8_t *fw;
  uint32_t len;
  if (cut >= 2) {
    fw = array_8852b_u3_nicce;
    len = array_8852b_u3_nicce_len;
  } else {
    fw = array_8852b_u2_nicce;
    len = array_8852b_u2_nicce_len;
  }
  _logger->info("Kestrel FWDL: image {} ({} bytes) via ep 0x{:02x}",
                cut >= 2 ? "u3_nicce" : "u2_nicce", len, _ch12_ep);

  /* mac_hal_init WDT block (init.c:470, right before the CPU enable): no
   * WDT wake on USB/PCIE, WDT platform-reset disabled (wdt_plt_rst_en=0).
   * Golden capture: 0x0170 -> 0x00000000, 0x01e0 keeps bit16 clear. */
  clr32(r::R_AX_SYS_CFG5, r::B_AX_WDT_WAKE_PCIE_EN | r::B_AX_WDT_WAKE_USB_EN);
  clr32(r::R_AX_WCPU_FW_CTRL, r::B_AX_WDT_PLT_RST_EN);

  if (!disable_cpu())
    return false;
  if (!enable_cpu(r::AX_BOOT_REASON_PWR_ON))
    return false;
  return mac_fwdl(fw, len, mss_idx);
}

} /* namespace kestrel */
