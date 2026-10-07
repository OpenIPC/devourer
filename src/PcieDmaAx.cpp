#include "PcieDmaAx.h"

#include <chrono>
#include <cstring>
#include <thread>

#include "PcieTransport.h"
#include "kestrel/FrameParserKestrel.h"

namespace devourer {

namespace {

constexpr uint32_t IDX_MASK = 0xFFF;
constexpr uint32_t BD_SZ = 8;

/* ---- 8852C HAXI ring registers (vendor _pcie_8852c.c get_txbd/rxbd_reg;
 * rtw89 rtw89_pci_ch_dma_addr_set_v1). num/idx are shared with the 8852A/B
 * layout except CH10/11; bdram/desa are the _V1 bank. ---- */
struct TxChRegs {
  uint16_t num, idx, bdram, desa_l;
  uint8_t bd_start, bd_max, bd_min; /* BDRAM table (rtw89_bd_ram_table_dual) */
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

/* TXBD / addr_info / rxbd_info / RPP field encodings (rtw89 pci.h). */
constexpr uint16_t TXBD_OPTION_LS = 1u << 14;
constexpr uint16_t TXWP_VALID = 1u << 15;
constexpr uint32_t ADDR_INFO_NR_MAX = 10;        /* RTW89_TXADDR_INFO_NR_V1 */
constexpr uint32_t ADDR_INFO_LEN_MAX = 2044;     /* TXADDR_INFO_LENTHG_V1_MAX */
constexpr uint16_t ADDR_INFO_LS = 1u << 15;      /* B_PCIADDR_LS_V1_MASK */
constexpr uint32_t ADDR_INFO_SZ = 6;
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
constexpr uint32_t WD_INFO_LEN = 24;
constexpr uint16_t DRV_INFO_UNIT_8852C = 16;

constexpr size_t PAGE_SZ = 4096;
inline size_t page_align(size_t v) { return (v + PAGE_SZ - 1) & ~(PAGE_SZ - 1); }

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
void sleep_us(unsigned us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
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

PcieDmaAx::PcieDmaAx(PcieTransport &t, Logger_t logger, int rx_poll_us)
    : _t(t), _logger(std::move(logger)), _rx_poll_us(rx_poll_us) {
  static_assert(WD_BODY_LEN_V1 + WD_INFO_LEN + WP_INFO_SZ +
                        ADDR_INFO_NR_MAX * ADDR_INFO_SZ <=
                    kWdPageSize,
                "WD page overflow");
  static_assert(ADDR_INFO_NR_MAX * ADDR_INFO_LEN_MAX >= kPayloadSize,
                "payload bounce not addressable by addr_info");
}

size_t PcieDmaAx::slab_bytes() const { return layout().total; }

bool PcieDmaAx::attach(uint8_t *slab, uint64_t slab_iova) {
  const Layout l = layout();
  for (uint32_t ch = 0; ch < kTxChannels; ch++) {
    TxRing &r = _tx[ch];
    const size_t off = l.txbd + static_cast<size_t>(ch) * kBdLen * BD_SZ;
    r.bd = slab + off;
    r.bd_iova = slab_iova + off;
    r.wp = 0;
    r.reg_num = kTxCh[ch].num;
    r.reg_idx = kTxCh[ch].idx;
    r.reg_bdram = kTxCh[ch].bdram;
    r.reg_desa_l = kTxCh[ch].desa_l;
    r.bdram = kTxCh[ch].bd_start | (static_cast<uint32_t>(kTxCh[ch].bd_max) << 8) |
              (static_cast<uint32_t>(kTxCh[ch].bd_min) << 16);
  }
  _rxq.bd = slab + l.rxq_bd;
  _rxq.bd_iova = slab_iova + l.rxq_bd;
  _rxq.bufs = slab + l.rxq_buf;
  _rxq.bufs_iova = slab_iova + l.rxq_buf;
  _rxq.reg_num = R_RXQ_NUM;
  _rxq.reg_idx = R_RXQ_IDX;
  _rxq.reg_desa_l = R_RXQ_DESA_L;
  _rpq.bd = slab + l.rpq_bd;
  _rpq.bd_iova = slab_iova + l.rpq_bd;
  _rpq.bufs = slab + l.rpq_buf;
  _rpq.bufs_iova = slab_iova + l.rpq_buf;
  _rpq.reg_num = R_RPQ_NUM;
  _rpq.reg_idx = R_RPQ_IDX;
  _rpq.reg_desa_l = R_RPQ_DESA_L;
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
  return (_t.mmio_read<uint32_t>(reg_idx) >> 16) & IDX_MASK;
}

void PcieDmaAx::setup_rings() {
  /* rtw89_pci_reset_trx_rings: per TX channel num + BDRAM + desa_l (desa_h
   * never written), per RX ring num + desa_l; host indices to 0. The index
   * registers themselves were cleared by the HAL's clr_idx_all just before. */
  std::lock_guard<std::mutex> lk(_pool_mu);
  for (uint32_t ch = 0; ch < kTxChannels; ch++) {
    TxRing &r = _tx[ch];
    r.wp = 0;
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
  _logger->info("PcieDmaAx: rings programmed (13 TX × {} BDs, RXQ/RPQ {} × "
                "{} B, {} WD pages @ IOVA 0x{:x})",
                kBdLen, kBdLen, kRxBufSize, kWdPages, _wd_pages_iova);
}

bool PcieDmaAx::wait_consumed(TxRing &r, int ch, int timeout_ms) {
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 20);
  for (;;) {
    const uint32_t hw = hw_idx(r.reg_idx);
    if (hw == r.wp)
      return true;
    if (std::chrono::steady_clock::now() > deadline) {
      _logger->error("PcieDmaAx: CH{} TXBD not consumed (wp={} hw={}) within "
                     "{} ms",
                     ch, r.wp, hw, timeout_ms > 0 ? timeout_ms : 20);
      return false;
    }
    sleep_us(10);
  }
}

int PcieDmaAx::tx_submit(uint8_t hint, uint8_t *buf, size_t len,
                         int timeout_ms) {
  if (hint >= kTxChannels) {
    _logger->error("PcieDmaAx: tx on unknown DMA channel {}", hint);
    return -1;
  }
  std::lock_guard<std::mutex> lk(_tx_mu);
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
  /* The HAL hands us [wd_body_v1][wd_info if WDINFO_EN][frame] exactly as it
   * would bulk-OUT it on USB. */
  if (len < WD_BODY_LEN_V1) {
    _logger->error("PcieDmaAx: WD buffer too short ({})", len);
    return -1;
  }
  uint32_t dw0 = rd_le32(buf);
  const uint32_t wd_len =
      WD_BODY_LEN_V1 + ((dw0 & WD_BODY0_WDINFO_EN) ? WD_INFO_LEN : 0);
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

  /* addr_info_v1 ×n over the payload bounce (rtw89_pci_fill_txaddr_info_v1). */
  uint8_t *ai = wp + WP_INFO_SZ;
  uint32_t remain = flen, n = 0;
  uint64_t dma = pl_iova;
  for (; n < ADDR_INFO_NR_MAX && remain; n++) {
    const uint32_t chunk = remain > ADDR_INFO_LEN_MAX ? ADDR_INFO_LEN_MAX : remain;
    remain -= chunk;
    const uint16_t len_opt =
        static_cast<uint16_t>((chunk & 0x7FF) | (remain == 0 ? ADDR_INFO_LS : 0));
    wr_le16(ai + 0, len_opt);
    wr_le16(ai + 2, static_cast<uint16_t>(dma & 0xFFFF));
    wr_le16(ai + 4, static_cast<uint16_t>((dma >> 16) & 0xFFFF));
    dma += chunk;
    ai += ADDR_INFO_SZ;
  }
  const uint32_t page_len = wd_len + WP_INFO_SZ + n * ADDR_INFO_SZ;

  /* wd_body patch: this is a WD page (not an inline frame), no USB
   * store-and-forward, and the addr_info count. WP_OFFSET stays as the HAL
   * set it (rtw89 uses a non-zero offset only for a security header). */
  dw0 = (dw0 | WD_BODY0_WD_PAGE) & ~WD_BODY0_STF_MODE;
  wr_le32(pg + 0, dw0);
  uint32_t dw1 = rd_le32(pg + 4);
  dw1 = (dw1 & ~(WD_BODY1_ADDR_INFO_NUM_MSK << WD_BODY1_ADDR_INFO_NUM_SH)) |
        ((n & WD_BODY1_ADDR_INFO_NUM_MSK) << WD_BODY1_ADDR_INFO_NUM_SH);
  wr_le32(pg + 4, dw1);

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
  if (hw == _rpq.wp)
    return 0;
  std::atomic_thread_fence(std::memory_order_acquire);
  uint32_t n = 0;
  while (_rpq.wp != hw) {
    const uint8_t *buf = _rpq.bufs + static_cast<size_t>(_rpq.wp) * kRxBufSize;
    const uint32_t info = rd_le32(buf);
    const uint32_t blen = info & RXBD_INFO_LEN_MSK;
    const bool fs = info & RXBD_INFO_FS, ls = info & RXBD_INFO_LS;
    if (fs && ls && blen > 4 && blen <= kRxBufSize) {
      /* rxd (16/32 B) + drv_info, then 4-byte RPP words to the end. */
      kestrel::KestrelRxFrame f{};
      if (kestrel::parse_rx_8852b(buf + 4, blen - 4, f, DRV_INFO_UNIT_8852C)) {
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
  if (hw == _rxq.wp)
    return 0;
  std::atomic_thread_fence(std::memory_order_acquire);
  uint32_t n = 0;
  while (_rxq.wp != hw) {
    const uint8_t *buf = _rxq.bufs + static_cast<size_t>(_rxq.wp) * kRxBufSize;
    const uint32_t info = rd_le32(buf);
    const uint32_t blen = info & RXBD_INFO_LEN_MSK;
    const bool fs = info & RXBD_INFO_FS, ls = info & RXBD_INFO_LS;
    if (blen > 4 && blen <= kRxBufSize) {
      const uint8_t *data = buf + 4;
      const int dlen = static_cast<int>(blen - 4);
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
  while (!should_stop()) {
    uint32_t n = reap_rxq(on_data);
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
  _logger->info("PcieDmaAx: RX loop exited ({} RXQ bufs / {} packets, {} RPQ "
                "bufs; RPP done={} rty={} life={} drop={} unknown={})",
                rxq_bufs, _rx_delivered, rpq_bufs, s.tx_done, s.retry_limit,
                s.lifetime, s.macid_drop, s.unknown_seq);
}

void PcieDmaAx::irq_mask() {
  _t.mmio_write<uint32_t>(R_AX_PCIE_HIMR00_V1, 0);
  _t.mmio_write<uint32_t>(R_AX_HAXI_HIMR00, 0);
}

PcieDmaAx::RppStats PcieDmaAx::rpp_stats() const { return _rpp; }

} /* namespace devourer */
