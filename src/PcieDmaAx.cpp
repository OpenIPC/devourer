#include "PcieDmaAx.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "PcieDmaUtil.h"
#include "PcieTransport.h"
#include "kestrel/FrameParserKestrel.h"

namespace devourer {

namespace {

constexpr uint32_t IDX_MASK = 0xFFF;
constexpr uint32_t BD_SZ = 8;

/* ---- 8852C HAXI ring registers (vendor _pcie_8852c.c get_txbd/rxbd_reg +
 * mac_bdram_tbl_8852c). num/idx are shared with the 8852A/B layout except
 * CH10/11; bdram/desa are the _V1 bank. ---- */
struct TxChRegs {
  uint16_t num, idx, bdram, desa_l;
  uint8_t bd_start, bd_max, bd_min; /* BDRAM table */
};
constexpr TxChRegs kTxCh[PcieDmaAx::kTxChannels] = {
    {0x1024, 0x1058, 0x1300, 0x1230, 0, 5, 2},   /* ACH0 */
    {0x1026, 0x105C, 0x1304, 0x1238, 5, 5, 2},   /* ACH1 */
    {0x1028, 0x1060, 0x1308, 0x1240, 10, 5, 2},  /* ACH2 */
    {0x102A, 0x1064, 0x130C, 0x1248, 15, 5, 2},  /* ACH3 */
    {0x102C, 0x1068, 0x1310, 0x1250, 20, 5, 2},  /* ACH4 */
    {0x102E, 0x106C, 0x1314, 0x1258, 25, 5, 2},  /* ACH5 */
    {0x1030, 0x1070, 0x1318, 0x1260, 30, 5, 2},  /* ACH6 */
    {0x1032, 0x1074, 0x131C, 0x1268, 35, 5, 2},  /* ACH7 */
    {0x1034, 0x1078, 0x1320, 0x1270, 40, 5, 1},  /* CH8  B0MG */
    {0x1036, 0x107C, 0x1324, 0x1278, 45, 5, 1},  /* CH9  B0HI */
    {0x1438, 0x11D0, 0x1420, 0x1458, 50, 5, 1},  /* CH10 B1MG */
    {0x143A, 0x11D4, 0x1424, 0x1460, 55, 5, 1},  /* CH11 B1HI */
    {0x1038, 0x1080, 0x1328, 0x1280, 60, 4, 1},  /* CH12 FWCMD */
};
constexpr uint16_t R_RXQ_NUM = 0x1210, R_RXQ_IDX = 0x1218, R_RXQ_DESA_L = 0x1220;
constexpr uint16_t R_RPQ_NUM = 0x1212, R_RPQ_IDX = 0x121C, R_RPQ_DESA_L = 0x1228;
/* Interrupt masks this plane could enable (unused: polling). */
constexpr uint16_t R_AX_HAXI_HIMR00 = 0x10B0;
constexpr uint16_t R_AX_PCIE_HIMR00_V1 = 0x30B0;

/* ---- 8852B PCIe DMA ring registers (vendor _pcie_8852b.c get_txbd_reg /
 * get_rxbd_reg + mac_bdram_tbl_8852b). Only ACH0-3, CH8, CH9 and CH12 exist
 * (get_txbd_reg returns MACNOTSUP for the rest); BDRAM_CTRL is the _V2 bank
 * at 0x1200 and the TXBD DESA bank starts at 0x1110. A zero row = channel
 * absent on this die. ---- */
constexpr TxChRegs kTxCh8852b[PcieDmaAx::kTxChannels] = {
    {0x1024, 0x1058, 0x1200, 0x1110, 0, 5, 2},   /* ACH0 */
    {0x1026, 0x105C, 0x1204, 0x1118, 5, 5, 2},   /* ACH1 */
    {0x1028, 0x1060, 0x1208, 0x1120, 10, 5, 2},  /* ACH2 */
    {0x102A, 0x1064, 0x120C, 0x1128, 15, 5, 2},  /* ACH3 */
    {0, 0, 0, 0, 0, 0, 0},                       /* ACH4 (absent) */
    {0, 0, 0, 0, 0, 0, 0},                       /* ACH5 (absent) */
    {0, 0, 0, 0, 0, 0, 0},                       /* ACH6 (absent) */
    {0, 0, 0, 0, 0, 0, 0},                       /* ACH7 (absent) */
    {0x1034, 0x1078, 0x1220, 0x1150, 20, 4, 1},  /* CH8  B0MG */
    {0x1036, 0x107C, 0x1224, 0x1158, 24, 4, 1},  /* CH9  B0HI */
    {0, 0, 0, 0, 0, 0, 0},                       /* CH10 B1MG (absent) */
    {0, 0, 0, 0, 0, 0, 0},                       /* CH11 B1HI (absent) */
    {0x1038, 0x1080, 0x1228, 0x1160, 28, 4, 1},  /* CH12 FWCMD */
};
constexpr uint16_t R_RXQ_NUM_8852B = 0x1020, R_RXQ_IDX_8852B = 0x1050,
                   R_RXQ_DESA_L_8852B = 0x1100;
constexpr uint16_t R_RPQ_NUM_8852B = 0x1022, R_RPQ_IDX_8852B = 0x1054,
                   R_RPQ_DESA_L_8852B = 0x1108;
constexpr uint16_t R_AX_PCIE_HIMR00_8852B = 0x10B0;

/* TXBD / addr_info / rxbd_info / RPP field encodings (rtw89 pci.h — the
 * host-side byte formats the vendor drops lack). */
constexpr uint16_t TXBD_OPTION_LS = 1u << 14;
constexpr uint16_t TXWP_VALID = 1u << 15;
constexpr uint32_t ADDR_INFO_NR_MAX = 10;        /* RTW89_TXADDR_INFO_NR_V1 */
constexpr uint32_t ADDR_INFO_LEN_MAX = 2044;     /* TXADDR_INFO_LENTHG_V1_MAX */
constexpr uint16_t ADDR_INFO_LS = 1u << 15;      /* B_PCIADDR_LS_V1_MASK */
constexpr uint32_t ADDR_INFO_SZ = 6;
/* 8852B: one 8-byte addr_info_32 {len16, option16, dma32} per page
 * (rtw89_pci_fill_txaddr_info): option = MSDU_LS | NUM(1) (| dma[39:32] in
 * [13:6], always 0 here — the slab sits below 4 GiB). */
constexpr uint32_t ADDR_INFO32_SZ = 8;
constexpr uint16_t ADDR_INFO32_MSDU_LS = 1u << 15; /* RTW89_PCI_ADDR_MSDU_LS */
constexpr uint16_t ADDR_INFO32_NUM1 = 1;           /* RTW89_PCI_ADDR_NUM(1) */
constexpr uint32_t WP_INFO_SZ = 8;
constexpr uint32_t RXBD_INFO_FS = 1u << 15;
constexpr uint32_t RXBD_INFO_LS = 1u << 14;
constexpr uint32_t RXBD_INFO_LEN_MSK = 0x3FFF;
constexpr uint32_t RPP_SEQ_SH = 16, RPP_SEQ_MSK = 0x7FFF;
constexpr uint32_t RPP_STATUS_SH = 13, RPP_STATUS_MSK = 0x7;

/* wd_body dword0/1 bits the plane owns (txdesc.h AX_TXD_*). */
constexpr uint32_t WD_BODY0_WD_PAGE = 1u << 7;
constexpr uint32_t WD_BODY0_STF_MODE = 1u << 10; /* USB store-and-forward */
constexpr uint32_t WD_BODY0_WDINFO_EN = 1u << 22;
constexpr uint32_t WD_BODY1_ADDR_INFO_NUM_SH = 26;
constexpr uint32_t WD_BODY1_ADDR_INFO_NUM_MSK = 0x3F;
constexpr uint32_t WD_BODY_LEN_V1 = 32; /* 8852C wd_body_t_v1 */
constexpr uint32_t WD_BODY_LEN_8852B = 24; /* 8852B wd_body_t (no ADDR_INFO_NUM) */
constexpr uint32_t WD_INFO_LEN = 24;

constexpr size_t PAGE_SZ = pcie_dma::kPageSize;
using pcie_dma::page_align;
using pcie_dma::sleep_us;

inline uint32_t rd_le32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
inline void wr_le32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}
inline void wr_le16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
/* 8-byte BD {le16 a, le16 b, le32 dma} as volatile byte stores. */
inline void bd_write(volatile uint8_t *e, uint16_t a, uint16_t b, uint32_t dma) {
  e[0] = static_cast<uint8_t>(a);
  e[1] = static_cast<uint8_t>(a >> 8);
  e[2] = static_cast<uint8_t>(b);
  e[3] = static_cast<uint8_t>(b >> 8);
  e[4] = static_cast<uint8_t>(dma);
  e[5] = static_cast<uint8_t>(dma >> 8);
  e[6] = static_cast<uint8_t>(dma >> 16);
  e[7] = static_cast<uint8_t>(dma >> 24);
}

/* Slab layout offsets (page-aligned sections). */
struct Layout {
  size_t txbd, rxq_bd, rpq_bd, rxq_buf, rpq_buf, wd, payload, fwcmd, total;
};
Layout layout() {
  Layout l{};
  l.txbd = 0;
  l.rxq_bd = l.txbd + page_align(PcieDmaAx::kTxChannels * PcieDmaAx::kBdLen * BD_SZ);
  l.rpq_bd = l.rxq_bd + page_align(PcieDmaAx::kBdLen * BD_SZ);
  l.rxq_buf = l.rpq_bd + page_align(PcieDmaAx::kBdLen * BD_SZ);
  l.rpq_buf = l.rxq_buf + page_align(static_cast<size_t>(PcieDmaAx::kBdLen) *
                                     PcieDmaAx::kRxBufSize);
  l.wd = l.rpq_buf + page_align(static_cast<size_t>(PcieDmaAx::kBdLen) *
                                PcieDmaAx::kRxBufSize);
  l.payload = l.wd + page_align(static_cast<size_t>(PcieDmaAx::kWdPages) *
                                PcieDmaAx::kWdPageSize);
  l.fwcmd = l.payload + page_align(static_cast<size_t>(PcieDmaAx::kWdPages) *
                                   PcieDmaAx::kPayloadSize);
  l.total = l.fwcmd + page_align(static_cast<size_t>(PcieDmaAx::kBdLen) *
                                 PcieDmaAx::kFwcmdSlotSize);
  return l;
}

} /* namespace */

PcieDmaAx::PcieDmaAx(PcieTransport &t, Logger_t logger, int rx_poll_us,
                     AxDie die)
    : _t(t), _logger(std::move(logger)), _rx_poll_us(rx_poll_us), _die(die),
      _wd_body_len(die == AxDie::C8852C ? WD_BODY_LEN_V1 : WD_BODY_LEN_8852B),
      _drv_info_unit(die == AxDie::C8852C ? 16 : 8) {
  static_assert(WD_BODY_LEN_V1 + WD_INFO_LEN + WP_INFO_SZ +
                        ADDR_INFO_NR_MAX * ADDR_INFO_SZ <=
                    kWdPageSize,
                "WD page overflow");
  static_assert(ADDR_INFO_NR_MAX * ADDR_INFO_LEN_MAX >= kPayloadSize,
                "payload bounce not addressable by addr_info");
  static_assert(kPayloadSize <= 0xFFFF,
                "8852B addr_info_32 length is 16-bit");
}

size_t PcieDmaAx::slab_bytes() const { return layout().total; }

bool PcieDmaAx::attach(uint8_t *slab, uint64_t slab_iova) {
  const Layout l = layout();
  const bool c = _die == AxDie::C8852C;
  const TxChRegs *tab = c ? kTxCh : kTxCh8852b;
  for (uint32_t ch = 0; ch < kTxChannels; ch++) {
    TxRing &r = _tx[ch];
    const size_t off = l.txbd + static_cast<size_t>(ch) * kBdLen * BD_SZ;
    r.bd = slab + off;
    r.bd_iova = slab_iova + off;
    r.wp = 0;
    r.present = tab[ch].num != 0;
    r.reg_num = tab[ch].num;
    r.reg_idx = tab[ch].idx;
    r.reg_bdram = tab[ch].bdram;
    r.reg_desa_l = tab[ch].desa_l;
    r.bdram = tab[ch].bd_start | (static_cast<uint32_t>(tab[ch].bd_max) << 8) |
              (static_cast<uint32_t>(tab[ch].bd_min) << 16);
  }
  _rxq.bd = slab + l.rxq_bd;
  _rxq.bd_iova = slab_iova + l.rxq_bd;
  _rxq.bufs = slab + l.rxq_buf;
  _rxq.bufs_iova = slab_iova + l.rxq_buf;
  _rxq.reg_num = c ? R_RXQ_NUM : R_RXQ_NUM_8852B;
  _rxq.reg_idx = c ? R_RXQ_IDX : R_RXQ_IDX_8852B;
  _rxq.reg_desa_l = c ? R_RXQ_DESA_L : R_RXQ_DESA_L_8852B;
  _rpq.bd = slab + l.rpq_bd;
  _rpq.bd_iova = slab_iova + l.rpq_bd;
  _rpq.bufs = slab + l.rpq_buf;
  _rpq.bufs_iova = slab_iova + l.rpq_buf;
  _rpq.reg_num = c ? R_RPQ_NUM : R_RPQ_NUM_8852B;
  _rpq.reg_idx = c ? R_RPQ_IDX : R_RPQ_IDX_8852B;
  _rpq.reg_desa_l = c ? R_RPQ_DESA_L : R_RPQ_DESA_L_8852B;
  for (uint32_t i = 0; i < kBdLen; i++) {
    arm_rx_bd(_rxq, i);
    arm_rx_bd(_rpq, i);
  }
  _wd_pages = slab + l.wd;
  _wd_pages_iova = slab_iova + l.wd;
  _payload = slab + l.payload;
  _payload_iova = slab_iova + l.payload;
  _fwcmd = slab + l.fwcmd;
  _fwcmd_iova = slab_iova + l.fwcmd;
  _free_pages.clear();
  _free_pages.reserve(kWdPages);
  for (uint32_t i = kWdPages; i-- > 0;)
    _free_pages.push_back(static_cast<uint16_t>(i));
  _page_busy.fill(false);
  return true;
}

void PcieDmaAx::arm_rx_bd(RxRing &r, uint32_t idx) {
  bd_write(r.bd + static_cast<size_t>(idx) * BD_SZ,
           static_cast<uint16_t>(kRxBufSize), 0,
           static_cast<uint32_t>(r.bufs_iova +
                                 static_cast<uint64_t>(idx) * kRxBufSize));
}

uint32_t PcieDmaAx::hw_idx(uint16_t reg_idx) {
  /* A ring index register reads back its 12-bit hardware index in [27:16].
   * All-ones is the PCIe "nobody answered" value (device in reset / off the
   * bus), never a valid index for a 256-entry ring: report it as kBdLen so
   * every caller's `wp != hw` loop terminates instead of spinning forever. */
  const uint32_t v = _t.mmio_read<uint32_t>(reg_idx);
  if (v == 0xFFFFFFFFu) {
    if (!_dead_logged.exchange(true))
      _logger->error("PcieDmaAx: ring index 0x{:04x} reads all-ones — the "
                     "device stopped answering (reset / link down)",
                     reg_idx);
    return kBdLen;
  }
  const uint32_t hw = (v >> 16) & IDX_MASK;
  return hw < kBdLen ? hw : kBdLen;
}

void PcieDmaAx::setup_rings() {
  /* rtw89_pci_reset_trx_rings: per TX channel num + BDRAM + desa_l (desa_h
   * never written), per RX ring num + desa_l; host indices to 0. The index
   * registers themselves were cleared by the HAL's clr_idx_all just before. */
  std::lock_guard<std::mutex> lk(_pool_mu);
  uint32_t n_tx = 0;
  for (uint32_t ch = 0; ch < kTxChannels; ch++) {
    TxRing &r = _tx[ch];
    r.wp = 0;
    if (!r.present)
      continue;
    n_tx++;
    _t.mmio_write<uint16_t>(r.reg_num, static_cast<uint16_t>(kBdLen & IDX_MASK));
    _t.mmio_write<uint32_t>(r.reg_bdram, r.bdram);
    _t.mmio_write<uint32_t>(r.reg_desa_l, static_cast<uint32_t>(r.bd_iova));
  }
  for (RxRing *r : {&_rxq, &_rpq}) {
    r->wp = 0;
    for (uint32_t i = 0; i < kBdLen; i++)
      arm_rx_bd(*r, i);
    _t.mmio_write<uint16_t>(r->reg_num, static_cast<uint16_t>(kBdLen & IDX_MASK));
    _t.mmio_write<uint32_t>(r->reg_desa_l, static_cast<uint32_t>(r->bd_iova));
  }
  /* Every page is free again: the DMA engine was stopped + index-cleared
   * before this, so nothing in flight can still name an old page. */
  _free_pages.clear();
  for (uint32_t i = kWdPages; i-- > 0;)
    _free_pages.push_back(static_cast<uint16_t>(i));
  _page_busy.fill(false);
  _rx_assembling = false;
  _rx_assembly.clear();
  std::atomic_thread_fence(std::memory_order_seq_cst);
  _logger->info("PcieDmaAx: rings programmed ({} TX × {} BDs, RXQ/RPQ {} × "
                "{} B, {} WD pages @ IOVA 0x{:x})",
                n_tx, kBdLen, kBdLen, kRxBufSize, kWdPages, _wd_pages_iova);
}

bool PcieDmaAx::ring_has_room(const TxRing &r, int ch) {
  /* Submissions a timed-out wait left unconsumed stay in the ring. The
   * hardware index can only be trusted as "caught up" while fewer than
   * kBdLen-1 entries are outstanding; at that point refuse rather than let a
   * wrapped write index meet a stalled hardware index and read as success. */
  const uint32_t hw = hw_idx(r.reg_idx);
  if (hw >= kBdLen)
    return false; /* device gone — hw_idx logged it */
  const uint32_t outstanding = (r.wp + kBdLen - hw) % kBdLen;
  if (outstanding >= kBdLen - 1) {
    _logger->error("PcieDmaAx: CH{} ring stalled ({} unconsumed entries, hw={} "
                   "wp={}) — refusing the submission",
                   ch, outstanding, hw, r.wp);
    return false;
  }
  return true;
}

bool PcieDmaAx::wait_consumed(TxRing &r, int ch, int timeout_ms) {
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 20);
  for (;;) {
    const uint32_t hw = hw_idx(r.reg_idx);
    if (hw == r.wp)
      return true;
    if (hw >= kBdLen)
      return false; /* device gone — hw_idx logged it */
    if (std::chrono::steady_clock::now() > deadline) {
      _logger->error("PcieDmaAx: CH{} TXBD not consumed (wp={} hw={}) within "
                     "{} ms",
                     ch, r.wp, hw, timeout_ms > 0 ? timeout_ms : 20);
      if (!_stall_dumped.exchange(true)) {
        /* One engine snapshot per session, for the first stall: the DMA
         * gates, busy/error flags and the LBC watchdog (8852B bank) or the
         * HAXI equivalents. */
        struct { const char *n; uint16_t reg; } regs_b[] = {
            {"INIT_CFG1", 0x1000}, {"PS_CTRL", 0x1008},   {"DMA_STOP1", 0x1010},
            {"DMA_BUSY1", 0x101C}, {"HISR00", 0x10B4},    {"PCIE_INFO", 0x10F8},
            {"DBG_ERR_FLAG", 0x11C4}, {"LBC_WATCHDOG", 0x11D8},
            {"LTR_CTRL_0", 0x8410}, {"HCI_FUNC_EN", 0x8380}};
        struct { const char *n; uint16_t reg; } regs_c[] = {
            {"HAXI_INIT_CFG1", 0x1000}, {"HAXI_DMA_STOP1", 0x1010},
            {"HAXI_DMA_BUSY1", 0x101C}, {"HAXI_DMA_BUSY2", 0x11C8},
            {"HAXI_DMA_BUSY3", 0x1208}, {"HAXI_HISR00", 0x10B4}};
        if (_die == AxDie::C8852B)
          for (const auto &x : regs_b)
            _logger->error("  stall dump {:>13} 0x{:04x} = 0x{:08x}", x.n, x.reg,
                           _t.mmio_read<uint32_t>(x.reg));
        else
          for (const auto &x : regs_c)
            _logger->error("  stall dump {:>13} 0x{:04x} = 0x{:08x}", x.n, x.reg,
                           _t.mmio_read<uint32_t>(x.reg));
        _logger->error("  stall dump CH{} num=0x{:04x} idx=0x{:08x} bdram=0x{:08x} "
                       "desa=0x{:08x}",
                       ch, _t.mmio_read<uint16_t>(r.reg_num),
                       _t.mmio_read<uint32_t>(r.reg_idx),
                       _t.mmio_read<uint32_t>(r.reg_bdram),
                       _t.mmio_read<uint32_t>(r.reg_desa_l));
      }
      return false;
    }
    sleep_us(10);
  }
}

int PcieDmaAx::tx_submit(uint8_t hint, uint8_t *buf, size_t len,
                         int timeout_ms) {
  if (hint >= kTxChannels || !_tx[hint].present) {
    _logger->error("PcieDmaAx: tx on DMA channel {} this die does not have",
                   hint);
    return -1;
  }
  /* One lock per channel: the rings are independent (CH12 fwcmd vs a data or
   * mgmt frame in flight), only the WD page pool is shared and it has its
   * own mutex. */
  std::lock_guard<std::mutex> lk(_tx_mu[hint]);
  if (hint == kFwcmdCh)
    return submit_fwcmd(buf, len, timeout_ms);
  return submit_wd(hint, buf, len, timeout_ms);
}

int PcieDmaAx::submit_fwcmd(uint8_t *buf, size_t len, int timeout_ms) {
  /* rtw89_pci_fwcmd_submit: the whole [rxd_short 16 B][payload] buffer is the
   * DMA source; one TXBD with LS. The slot is reused once the hardware index
   * has passed it, which the synchronous wait below guarantees. */
  if (len < 16 || len > kFwcmdSlotSize) {
    _logger->error("PcieDmaAx: fwcmd len {} out of range (16..{})", len,
                   kFwcmdSlotSize);
    return -1;
  }
  TxRing &r = _tx[kFwcmdCh];
  if (!ring_has_room(r, kFwcmdCh))
    return -1;
  const uint32_t slot = r.wp;
  uint8_t *dst = _fwcmd + static_cast<size_t>(slot) * kFwcmdSlotSize;
  std::memcpy(dst, buf, len);
  const uint64_t iova = _fwcmd_iova + static_cast<uint64_t>(slot) * kFwcmdSlotSize;
  bd_write(r.bd + static_cast<size_t>(slot) * BD_SZ, static_cast<uint16_t>(len),
           TXBD_OPTION_LS, static_cast<uint32_t>(iova));
  std::atomic_thread_fence(std::memory_order_seq_cst);
  r.wp = (r.wp + 1) % kBdLen;
  _t.mmio_write<uint16_t>(r.reg_idx, static_cast<uint16_t>(r.wp & IDX_MASK));
  return wait_consumed(r, kFwcmdCh, timeout_ms) ? static_cast<int>(len) : -1;
}

int PcieDmaAx::submit_wd(int ch, uint8_t *buf, size_t len, int timeout_ms) {
  /* The HAL hands us [wd_body][wd_info if WDINFO_EN][frame] exactly as it
   * would bulk-OUT it on USB (32-byte wd_body_v1 on the 8852C, 24-byte
   * wd_body on the 8852B). */
  if (len < _wd_body_len) {
    _logger->error("PcieDmaAx: WD buffer too short ({})", len);
    return -1;
  }
  uint32_t dw0 = rd_le32(buf);
  const uint32_t wd_len =
      _wd_body_len + ((dw0 & WD_BODY0_WDINFO_EN) ? WD_INFO_LEN : 0);
  if (len < wd_len) {
    _logger->error("PcieDmaAx: WD buffer shorter than its descriptor ({} < {})",
                   len, wd_len);
    return -1;
  }
  const uint32_t flen = static_cast<uint32_t>(len - wd_len);
  if (flen == 0 || flen > kPayloadSize) {
    _logger->error("PcieDmaAx: frame of {} B exceeds the {} B payload bounce",
                   flen, kPayloadSize);
    return -1;
  }
  if (!ring_has_room(_tx[ch], ch))
    return -1;

  /* A TX-only session has nobody servicing the RXQ, and the MAC keeps
   * delivering ambient frames into it. On the 8852B's engine a full RXQ
   * (DBG_ERR_FLAG PCIE_RXQ_FULL, HISR RDU) halts the TX channels as well —
   * measured: B0MG stalled after 57-195 frames on a busy channel — so the
   * TX path re-arms the RXQ itself, discarding what it reaps, whenever the
   * RX loop is not running. */
  /* Not on every frame — the send path owes no register read per frame: a
   * drain every 16 submissions or 50 ms (both decided from host state)
   * keeps well ahead of the 256-buffer ring at any ambient rate seen (the
   * measured fill took ~300 ms on a busy channel). Ownership is decided
   * under _rxq_mu so an RX loop starting in between cannot lose frames. */
  {
    const auto now = std::chrono::steady_clock::now();
    const bool due = (++_tx_since_drain >= 16) ||
                     (now - _last_drain) > std::chrono::milliseconds(50);
    if (due) {
      _tx_since_drain = 0;
      _last_drain = now;
      std::lock_guard<std::mutex> lk(_rxq_mu);
      if (!_rx_loop_active.load())
        _rx_txonly_dropped += reap_rxq(nullptr);
    }
  }

  /* A free page, reaping release reports first; starve → wait on the RPQ. */
  uint16_t page;
  {
    std::unique_lock<std::mutex> lk(_pool_mu);
    reap_rpq_locked();
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 20);
    while (_free_pages.empty()) {
      lk.unlock();
      if (std::chrono::steady_clock::now() > deadline) {
        _logger->error("PcieDmaAx: CH{} no free WD page ({} in flight, RPQ "
                       "released none within the timeout)",
                       ch, kWdPages);
        return -1;
      }
      sleep_us(50);
      lk.lock();
      reap_rpq_locked();
    }
    page = _free_pages.back();
    _free_pages.pop_back();
    _page_busy[page] = true;
  }

  uint8_t *pg = _wd_pages + static_cast<size_t>(page) * kWdPageSize;
  const uint64_t pg_iova = _wd_pages_iova + static_cast<uint64_t>(page) * kWdPageSize;
  uint8_t *pl = _payload + static_cast<size_t>(page) * kPayloadSize;
  const uint64_t pl_iova = _payload_iova + static_cast<uint64_t>(page) * kPayloadSize;

  std::memcpy(pl, buf + wd_len, flen);
  std::memset(pg, 0, kWdPageSize);
  std::memcpy(pg, buf, wd_len);

  /* wp_info: seq0 = page index | VALID; seq1..3 = 0 (rtw89_pci_txwd_submit). */
  uint8_t *wp = pg + wd_len;
  wr_le16(wp + 0, static_cast<uint16_t>(page | TXWP_VALID));
  wr_le16(wp + 2, 0);
  wr_le16(wp + 4, 0);
  wr_le16(wp + 6, 0);

  uint8_t *ai = wp + WP_INFO_SZ;
  uint32_t page_len;
  if (_die == AxDie::C8852C) {
    /* addr_info_v1 ×n over the payload bounce (rtw89_pci_fill_txaddr_info_v1). */
    uint32_t remain = flen, n = 0;
    uint64_t dma = pl_iova;
    for (; n < ADDR_INFO_NR_MAX && remain; n++) {
      const uint32_t chunk = remain > ADDR_INFO_LEN_MAX ? ADDR_INFO_LEN_MAX : remain;
      remain -= chunk;
      const uint16_t len_opt = static_cast<uint16_t>(
          (chunk & 0x7FF) | (remain == 0 ? ADDR_INFO_LS : 0));
      wr_le16(ai + 0, len_opt);
      wr_le16(ai + 2, static_cast<uint16_t>(dma & 0xFFFF));
      wr_le16(ai + 4, static_cast<uint16_t>((dma >> 16) & 0xFFFF));
      dma += chunk;
      ai += ADDR_INFO_SZ;
    }
    page_len = wd_len + WP_INFO_SZ + n * ADDR_INFO_SZ;
    /* wd_body_v1 dword1 carries the addr_info count (AX_TXD_ADDR_INFO_NUM);
     * the 8852B body has no such field (its dword1 is PLD / DMA_TXAGG_NUM /
     * SHCUT_CAMID) and is left alone. */
    uint32_t dw1 = rd_le32(pg + 4);
    dw1 = (dw1 & ~(WD_BODY1_ADDR_INFO_NUM_MSK << WD_BODY1_ADDR_INFO_NUM_SH)) |
          ((n & WD_BODY1_ADDR_INFO_NUM_MSK) << WD_BODY1_ADDR_INFO_NUM_SH);
    wr_le32(pg + 4, dw1);
  } else {
    /* One addr_info_32 for the whole frame (rtw89_pci_fill_txaddr_info). */
    wr_le16(ai + 0, static_cast<uint16_t>(flen));
    wr_le16(ai + 2, static_cast<uint16_t>(ADDR_INFO32_MSDU_LS | ADDR_INFO32_NUM1));
    wr_le32(ai + 4, static_cast<uint32_t>(pl_iova));
    page_len = wd_len + WP_INFO_SZ + ADDR_INFO32_SZ;
  }

  /* wd_body patch: this is a WD page (not an inline frame) and no USB
   * store-and-forward. WP_OFFSET stays as the HAL set it (rtw89 uses a
   * non-zero offset only for a security header). */
  dw0 = (dw0 | WD_BODY0_WD_PAGE) & ~WD_BODY0_STF_MODE;
  wr_le32(pg + 0, dw0);

  TxRing &r = _tx[ch];
  bd_write(r.bd + static_cast<size_t>(r.wp) * BD_SZ,
           static_cast<uint16_t>(page_len), TXBD_OPTION_LS,
           static_cast<uint32_t>(pg_iova));
  std::atomic_thread_fence(std::memory_order_seq_cst);
  r.wp = (r.wp + 1) % kBdLen;
  _t.mmio_write<uint16_t>(r.reg_idx, static_cast<uint16_t>(r.wp & IDX_MASK));
  /* The page stays busy either way: a timed-out fetch may still complete,
   * and only its RPP can prove the hardware is done with the buffers. */
  return wait_consumed(r, ch, timeout_ms) ? static_cast<int>(len) : -1;
}

void PcieDmaAx::release_page(uint32_t seq, uint32_t status) {
  if (seq >= kWdPages || !_page_busy[seq]) {
    _rpp.unknown_seq++;
    return;
  }
  _page_busy[seq] = false;
  _free_pages.push_back(static_cast<uint16_t>(seq));
  _rpp.pages_freed++;
  switch (status) {
  case 0: _rpp.tx_done++; break;
  case 1: _rpp.retry_limit++; break;
  case 2: _rpp.lifetime++; break;
  case 3: _rpp.macid_drop++; break;
  default: break;
  }
}

uint32_t PcieDmaAx::reap_rpq_locked() {
  const uint32_t hw = hw_idx(_rpq.reg_idx);
  if (hw == _rpq.wp || hw >= kBdLen)
    return 0;
  std::atomic_thread_fence(std::memory_order_acquire);
  uint32_t n = 0;
  while (_rpq.wp != hw && n < kBdLen) {
    const uint8_t *buf = _rpq.bufs + static_cast<size_t>(_rpq.wp) * kRxBufSize;
    const uint32_t info = rd_le32(buf);
    const uint32_t blen = info & RXBD_INFO_LEN_MSK;
    const bool fs = info & RXBD_INFO_FS, ls = info & RXBD_INFO_LS;
    if (fs && ls && blen > 4 && blen <= kRxBufSize) {
      /* rxd (16/32 B) + drv_info, then 4-byte RPP words to the end. */
      kestrel::KestrelRxFrame f{};
      if (kestrel::parse_rx_8852b(buf + 4, blen - 4, f, _drv_info_unit)) {
        for (uint32_t off = 0; off + 4 <= f.payload_len; off += 4) {
          const uint32_t rpp = rd_le32(f.payload + off);
          release_page((rpp >> RPP_SEQ_SH) & RPP_SEQ_MSK,
                       (rpp >> RPP_STATUS_SH) & RPP_STATUS_MSK);
        }
      } else {
        _logger->warn("PcieDmaAx: RPQ buffer {} unparseable (len {})", _rpq.wp,
                      blen);
      }
    } else {
      _logger->warn("PcieDmaAx: RPQ buffer {} not FS+LS (info 0x{:08x})",
                    _rpq.wp, info);
    }
    arm_rx_bd(_rpq, _rpq.wp);
    _rpq.wp = (_rpq.wp + 1) % kBdLen;
    n++;
  }
  std::atomic_thread_fence(std::memory_order_release);
  _t.mmio_write<uint16_t>(_rpq.reg_idx, static_cast<uint16_t>(_rpq.wp & IDX_MASK));
  return n;
}

uint32_t PcieDmaAx::reap_rxq(
    const std::function<void(const uint8_t *, int)> &on_data) {
  const uint32_t hw = hw_idx(_rxq.reg_idx);
  if (hw == _rxq.wp || hw >= kBdLen)
    return 0;
  std::atomic_thread_fence(std::memory_order_acquire);
  uint32_t n = 0;
  while (_rxq.wp != hw && n < kBdLen) {
    const uint8_t *buf = _rxq.bufs + static_cast<size_t>(_rxq.wp) * kRxBufSize;
    const uint32_t info = rd_le32(buf);
    const uint32_t blen = info & RXBD_INFO_LEN_MSK;
    const bool fs = info & RXBD_INFO_FS, ls = info & RXBD_INFO_LS;
    if (!on_data) {
      /* TX-only drain: re-arm and count, deliver nothing, keep the delivery
       * statistics honest. */
    } else if (blen > 4 && blen <= kRxBufSize) {
      const uint8_t *data = buf + 4;
      const int dlen = static_cast<int>(blen - 4);
      if (fs && dlen >= 4) {
        const uint32_t t = (rd_le32(data) >> 24) & 0xf;
        _rx_types[t]++;
      }
      if (fs && ls) {
        on_data(data, dlen);
        _rx_delivered++;
      } else if (fs) {
        _rx_assembly.assign(data, data + dlen);
        _rx_assembling = true;
      } else if (_rx_assembling) {
        _rx_assembly.insert(_rx_assembly.end(), data, data + dlen);
        if (ls) {
          on_data(_rx_assembly.data(), static_cast<int>(_rx_assembly.size()));
          _rx_delivered++;
          _rx_assembling = false;
          _rx_assembly.clear();
        }
      } else {
        _logger->warn("PcieDmaAx: RXQ continuation without a first segment "
                      "(info 0x{:08x})",
                      info);
      }
    } else {
      _rx_bad_len++;
    }
    arm_rx_bd(_rxq, _rxq.wp);
    _rxq.wp = (_rxq.wp + 1) % kBdLen;
    n++;
  }
  std::atomic_thread_fence(std::memory_order_release);
  _t.mmio_write<uint16_t>(_rxq.reg_idx, static_cast<uint16_t>(_rxq.wp & IDX_MASK));
  return n;
}

void PcieDmaAx::rx_loop(
    const std::function<void(const uint8_t *, int)> &on_data,
    const std::function<bool()> &should_stop) {
  _logger->info("PcieDmaAx: RX loop started (polled, {} us; RXQ + RPQ)",
                _rx_poll_us);
  uint64_t rxq_bufs = 0, rpq_bufs = 0;
  {
    /* Take RXQ ownership under the same mutex the TX-only drain checks. */
    std::lock_guard<std::mutex> lk(_rxq_mu);
    _rx_loop_active = true;
  }
  struct ActiveGuard {
    std::atomic<bool> &f;
    std::mutex &mu;
    ~ActiveGuard() {
      std::lock_guard<std::mutex> lk(mu);
      f = false;
    }
  } active_guard{_rx_loop_active, _rxq_mu};
  while (!should_stop()) {
    uint32_t n;
    {
      std::lock_guard<std::mutex> lk(_rxq_mu);
      n = reap_rxq(on_data);
    }
    uint32_t m;
    {
      std::lock_guard<std::mutex> lk(_pool_mu);
      m = reap_rpq_locked();
    }
    rxq_bufs += n;
    rpq_bufs += m;
    if (n == 0 && m == 0)
      sleep_us(static_cast<unsigned>(_rx_poll_us > 0 ? _rx_poll_us : 200));
  }
  const RppStats s = rpp_stats();
  std::string types;
  for (size_t t = 0; t < _rx_types.size(); t++)
    if (_rx_types[t])
      types += " t" + std::to_string(t) + "=" + std::to_string(_rx_types[t]);
  _logger->info("PcieDmaAx: RX loop exited ({} RXQ bufs / {} packets, {} RPQ "
                "bufs, {} bad-len; rpkt types:{}; RPP done={} rty={} life={} "
                "drop={} unknown={})",
                rxq_bufs, _rx_delivered, rpq_bufs, _rx_bad_len, types,
                s.tx_done, s.retry_limit, s.lifetime, s.macid_drop,
                s.unknown_seq);
}

void PcieDmaAx::irq_mask() {
  /* Teardown-time hook (the transport masks before dropping the vector):
   * the one place a TX-only session's RXQ drain can be reported. */
  if (_rx_txonly_dropped)
    _logger->info("PcieDmaAx: TX-only RXQ drain re-armed {} buffers (ambient "
                  "RX discarded while no RX loop ran)",
                  _rx_txonly_dropped);
  if (_die == AxDie::C8852C) {
    _t.mmio_write<uint32_t>(R_AX_PCIE_HIMR00_V1, 0);
    _t.mmio_write<uint32_t>(R_AX_HAXI_HIMR00, 0);
  } else {
    _t.mmio_write<uint32_t>(R_AX_PCIE_HIMR00_8852B, 0);
  }
}

PcieDmaAx::RppStats PcieDmaAx::rpp_stats() const {
  std::lock_guard<std::mutex> lk(_pool_mu); /* written under it by release_page */
  return _rpp;
}

} /* namespace devourer */
