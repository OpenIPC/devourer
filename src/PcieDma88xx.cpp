#include "PcieDma88xx.h"

#include <chrono>
#include <cstring>
#include <thread>

#include <poll.h>
#include <unistd.h>

#include "PcieDmaUtil.h"
#include "PcieTransport.h"

namespace devourer {

namespace {

/* ---- 88xx PCIe TRX ring register map (rtw88 pci.h, v6.12) ---- */
constexpr uint16_t RTK_PCI_CTRL = 0x300;
constexpr uint32_t BIT_RST_TRXDMA_INTF = 1u << 20;
constexpr uint32_t BIT_RX_TAG_EN = 1u << 15;

constexpr uint16_t RTK_PCI_TXBD_DESA_BCNQ = 0x308;
constexpr uint16_t RTK_PCI_TXBD_DESA_H2CQ = 0x1320;
constexpr uint16_t RTK_PCI_TXBD_DESA_MGMTQ = 0x310;
constexpr uint16_t RTK_PCI_TXBD_DESA_BKQ = 0x330;
constexpr uint16_t RTK_PCI_TXBD_DESA_BEQ = 0x328;
constexpr uint16_t RTK_PCI_TXBD_DESA_VIQ = 0x320;
constexpr uint16_t RTK_PCI_TXBD_DESA_VOQ = 0x318;
constexpr uint16_t RTK_PCI_TXBD_DESA_HI0Q = 0x340;
constexpr uint16_t RTK_PCI_RXBD_DESA_MPDUQ = 0x338;

constexpr uint16_t RTK_PCI_TXBD_NUM_H2CQ = 0x1328;
constexpr uint16_t RTK_PCI_TXBD_NUM_MGMTQ = 0x380;
constexpr uint16_t RTK_PCI_TXBD_NUM_BKQ = 0x38A;
constexpr uint16_t RTK_PCI_TXBD_NUM_BEQ = 0x388;
constexpr uint16_t RTK_PCI_TXBD_NUM_VIQ = 0x386;
constexpr uint16_t RTK_PCI_TXBD_NUM_VOQ = 0x384;
constexpr uint16_t RTK_PCI_TXBD_NUM_HI0Q = 0x38C;
constexpr uint16_t RTK_PCI_RXBD_NUM_MPDUQ = 0x382;

constexpr uint16_t RTK_PCI_TXBD_IDX_H2CQ = 0x132C;
constexpr uint16_t RTK_PCI_TXBD_IDX_MGMTQ = 0x3B0;
constexpr uint16_t RTK_PCI_TXBD_IDX_BKQ = 0x3AC;
constexpr uint16_t RTK_PCI_TXBD_IDX_BEQ = 0x3A8;
constexpr uint16_t RTK_PCI_TXBD_IDX_VIQ = 0x3A4;
constexpr uint16_t RTK_PCI_TXBD_IDX_VOQ = 0x3A0;
constexpr uint16_t RTK_PCI_TXBD_IDX_HI0Q = 0x3B8;
constexpr uint16_t RTK_PCI_RXBD_IDX_MPDUQ = 0x3B4;

constexpr uint16_t RTK_PCI_TXBD_RWPTR_CLR = 0x39C;
constexpr uint16_t RTK_PCI_TXBD_H2CQ_CSR = 0x1330;
constexpr uint32_t BIT_CLR_H2CQ_HOST_IDX = 1u << 16;
constexpr uint32_t BIT_CLR_H2CQ_HW_IDX = 1u << 8;

constexpr uint16_t RTK_PCI_TXBD_BCN_WORK = 0x383;
constexpr uint8_t BIT_PCI_BCNQ_FLAG = 1u << 4;

/* Interrupt registers (RX-relevant subset). */
constexpr uint16_t RTK_PCI_HIMR0 = 0x0B0;
constexpr uint16_t RTK_PCI_HISR0 = 0x0B4;
constexpr uint32_t IMR_ROK = 1u << 0; /* RX DMA OK */
constexpr uint32_t IMR_RDU = 1u << 1; /* RX descriptor unavailable */

constexpr uint32_t TRX_BD_IDX_MASK = 0xFFF;

/* 8821C (and all wcpu-11ac rtw88 chips we care about): 48-byte tx pkt desc,
 * 16-byte TX BD slot (a PAIR of 8-byte entries), 8-byte RX BD. */
constexpr uint32_t TX_PKT_DESC_SZ = 48;
constexpr uint32_t TX_BD_SLOT_SZ = 16;
constexpr uint32_t RX_BD_SZ = 8;

constexpr uint32_t RING_LEN_DEFAULT = 128; /* RTK_DEFAULT_TX_DESC_NUM */
constexpr uint32_t RING_LEN_BE = 256;      /* RTK_BEQ_TX_DESC_NUM */
constexpr uint32_t RING_LEN_BCN = 1;
constexpr uint32_t TX_BOUNCE_SZ = 32 * 1024;

constexpr size_t PAGE_SZ = pcie_dma::kPageSize;
using pcie_dma::page_align;
using pcie_dma::sleep_us;

/* 8-byte buffer-descriptor entry accessors (volatile LE stores). */
inline void bd_write(volatile uint8_t *e, uint16_t buf_size, uint16_t psb_len,
                     uint32_t dma) {
  e[0] = static_cast<uint8_t>(buf_size);
  e[1] = static_cast<uint8_t>(buf_size >> 8);
  e[2] = static_cast<uint8_t>(psb_len);
  e[3] = static_cast<uint8_t>(psb_len >> 8);
  e[4] = static_cast<uint8_t>(dma);
  e[5] = static_cast<uint8_t>(dma >> 8);
  e[6] = static_cast<uint8_t>(dma >> 16);
  e[7] = static_cast<uint8_t>(dma >> 24);
}


struct QueueRegs {
  uint32_t len;
  uint16_t desa, num, idx;
};
const QueueRegs kQ[PcieDma88xx::Q_MAX] = {
    /* Q_BCN  */ {RING_LEN_BCN, RTK_PCI_TXBD_DESA_BCNQ, 0, 0},
    /* Q_MGMT */
    {RING_LEN_DEFAULT, RTK_PCI_TXBD_DESA_MGMTQ, RTK_PCI_TXBD_NUM_MGMTQ,
     RTK_PCI_TXBD_IDX_MGMTQ},
    /* Q_VO   */
    {RING_LEN_DEFAULT, RTK_PCI_TXBD_DESA_VOQ, RTK_PCI_TXBD_NUM_VOQ,
     RTK_PCI_TXBD_IDX_VOQ},
    /* Q_VI   */
    {RING_LEN_DEFAULT, RTK_PCI_TXBD_DESA_VIQ, RTK_PCI_TXBD_NUM_VIQ,
     RTK_PCI_TXBD_IDX_VIQ},
    /* Q_BE   */
    {RING_LEN_BE, RTK_PCI_TXBD_DESA_BEQ, RTK_PCI_TXBD_NUM_BEQ,
     RTK_PCI_TXBD_IDX_BEQ},
    /* Q_BK   */
    {RING_LEN_DEFAULT, RTK_PCI_TXBD_DESA_BKQ, RTK_PCI_TXBD_NUM_BKQ,
     RTK_PCI_TXBD_IDX_BKQ},
    /* Q_HI0  */
    {RING_LEN_DEFAULT, RTK_PCI_TXBD_DESA_HI0Q, RTK_PCI_TXBD_NUM_HI0Q,
     RTK_PCI_TXBD_IDX_HI0Q},
    /* Q_H2C  */
    {RING_LEN_DEFAULT, RTK_PCI_TXBD_DESA_H2CQ, RTK_PCI_TXBD_NUM_H2CQ,
     RTK_PCI_TXBD_IDX_H2CQ},
};

} /* namespace */

PcieDma88xx::PcieDma88xx(PcieTransport &t, Logger_t logger,
                         uint32_t rx_ring_len, uint32_t rx_buf_size,
                         int rx_poll_us)
    : _t(t), _logger(std::move(logger)), _rx_ring_len(rx_ring_len),
      _rx_buf_size(rx_buf_size), _rx_poll_us(rx_poll_us) {}

size_t PcieDma88xx::slab_bytes() const {
  /* Slab layout (page-aligned sections):
   *   [0]  8 × TX BD rings   (4 KiB each — BE at 256×16 = 4 KiB is the max)
   *   [1]  8 × TX bounce      (32 KiB each; sync TX = one in flight per queue)
   *   [2]  RX BD ring         (rx_ring_len × 8)
   *   [3]  RX buffers         (rx_ring_len × rx_buf_size) */
  const uint32_t rxn = _rx_ring_len;
  size_t off_txbd = 0;
  size_t off_bounce = off_txbd + Q_MAX * PAGE_SZ;
  size_t off_rxbd = off_bounce + Q_MAX * TX_BOUNCE_SZ;
  size_t off_rxbuf = off_rxbd + page_align(rxn * RX_BD_SZ);
  return page_align(off_rxbuf + static_cast<size_t>(rxn) * _rx_buf_size);
}

bool PcieDma88xx::attach(uint8_t *slab, uint64_t slab_iova) {
  const uint32_t rxn = _rx_ring_len;
  size_t off_txbd = 0;
  size_t off_bounce = off_txbd + Q_MAX * PAGE_SZ;
  size_t off_rxbd = off_bounce + Q_MAX * TX_BOUNCE_SZ;
  size_t off_rxbuf = off_rxbd + page_align(rxn * RX_BD_SZ);

  auto va = [&](size_t off) { return slab + off; };
  auto iova = [&](size_t off) { return slab_iova + off; };

  for (int q = 0; q < Q_MAX; q++) {
    size_t bd_off = off_txbd + static_cast<size_t>(q) * PAGE_SZ;
    size_t bo_off = off_bounce + static_cast<size_t>(q) * TX_BOUNCE_SZ;
    _tx[q].bd = va(bd_off);
    _tx[q].bd_iova = iova(bd_off);
    _tx[q].bounce = va(bo_off);
    _tx[q].bounce_iova = iova(bo_off);
    _tx[q].bounce_len = TX_BOUNCE_SZ;
    _tx[q].len = kQ[q].len;
    _tx[q].wp = 0;
    _tx[q].reg_desa = kQ[q].desa;
    _tx[q].reg_num = kQ[q].num;
    _tx[q].reg_idx = kQ[q].idx;
  }

  _rx.bd = va(off_rxbd);
  _rx.bd_iova = iova(off_rxbd);
  _rx.bufs = va(off_rxbuf);
  _rx.bufs_iova = iova(off_rxbuf);
  _rx.len = rxn;
  _rx.rp = 0;
  for (uint32_t i = 0; i < rxn; i++)
    arm_rx_bd(i);
  return true;
}

void PcieDma88xx::arm_rx_bd(uint32_t idx) {
  /* {buf_size, total_pkt_size = 0 (HW write-back), dma} — port of
   * rtw_pci_reset_rx_desc. */
  bd_write(_rx.bd + static_cast<size_t>(idx) * RX_BD_SZ,
           static_cast<uint16_t>(_rx_buf_size), 0,
           static_cast<uint32_t>(_rx.bufs_iova +
                                 static_cast<uint64_t>(idx) * _rx_buf_size));
}

void PcieDma88xx::setup_rings() {
  /* Port of rtw_pci_reset_buf_desc — EXACT order (register-level quirks:
   * 0x300+3 |= 0xf7 first, RWPTR clear + H2CQ CSR clear last), then
   * rtw_pci_dma_reset. */
  _t.mmio_write<uint8_t>(
      RTK_PCI_CTRL + 3,
      static_cast<uint8_t>(_t.mmio_read<uint8_t>(RTK_PCI_CTRL + 3) | 0xf7));

  /* BCN has no NUM register ("specialized for rsvd page"). */
  _t.mmio_write<uint32_t>(RTK_PCI_TXBD_DESA_BCNQ,
                          static_cast<uint32_t>(_tx[Q_BCN].bd_iova));

  /* H2C (wcpu-11ac chips). */
  _tx[Q_H2C].wp = 0;
  _t.mmio_write<uint16_t>(
      RTK_PCI_TXBD_NUM_H2CQ,
      static_cast<uint16_t>(_tx[Q_H2C].len & TRX_BD_IDX_MASK));
  _t.mmio_write<uint32_t>(RTK_PCI_TXBD_DESA_H2CQ,
                          static_cast<uint32_t>(_tx[Q_H2C].bd_iova));

  for (int q : {Q_BK, Q_BE, Q_VO, Q_VI, Q_MGMT, Q_HI0}) {
    _tx[q].wp = 0;
    _t.mmio_write<uint16_t>(
        _tx[q].reg_num, static_cast<uint16_t>(_tx[q].len & TRX_BD_IDX_MASK));
    _t.mmio_write<uint32_t>(_tx[q].reg_desa,
                            static_cast<uint32_t>(_tx[q].bd_iova));
  }

  _rx.rp = 0;
  for (uint32_t i = 0; i < _rx.len; i++)
    arm_rx_bd(i);
  _t.mmio_write<uint16_t>(RTK_PCI_RXBD_NUM_MPDUQ,
                          static_cast<uint16_t>(_rx.len & TRX_BD_IDX_MASK));
  _t.mmio_write<uint32_t>(RTK_PCI_RXBD_DESA_MPDUQ,
                          static_cast<uint32_t>(_rx.bd_iova));

  /* reset read/write pointers */
  _t.mmio_write<uint32_t>(RTK_PCI_TXBD_RWPTR_CLR, 0xffffffff);
  /* reset H2C queue indices in a single write (wcpu-11ac) */
  _t.mmio_write<uint32_t>(RTK_PCI_TXBD_H2CQ_CSR,
                          _t.mmio_read<uint32_t>(RTK_PCI_TXBD_H2CQ_CSR) |
                              BIT_CLR_H2CQ_HOST_IDX | BIT_CLR_H2CQ_HW_IDX);

  /* rtw_pci_dma_reset */
  _t.mmio_write<uint32_t>(RTK_PCI_CTRL, _t.mmio_read<uint32_t>(RTK_PCI_CTRL) |
                                            BIT_RST_TRXDMA_INTF |
                                            BIT_RX_TAG_EN);
  _logger->info("PcieTransport: TRX rings programmed (RXBD {} slots @ IOVA "
                "0x{:x})",
                _rx.len, _rx.bd_iova);
}

int PcieDma88xx::tx_submit(uint8_t hint, uint8_t *buf, size_t len,
                           int timeout_ms) {
  (void)hint; /* USB addressing; the ring is chosen by descriptor QSEL */
  if (len < 48)
    return -1;
  const uint8_t qsel = buf[5] & 0x1F;
  int queue;
  switch (qsel) {
  case 0x10: /* QSEL_BEACON — rsvd page / DLFW */
    queue = Q_BCN;
    break;
  case 0x11: /* high */
    queue = Q_HI0;
    break;
  case 0x12: /* mgmt (the monitor-inject descriptor) */
    queue = Q_MGMT;
    break;
  case 0x13: /* h2c command */
    queue = Q_H2C;
    break;
  default: /* AC data */
    queue = Q_BE;
    break;
  }
  return tx_submit_sync(queue, buf, len, timeout_ms);
}

int PcieDma88xx::tx_submit_sync(int queue, const uint8_t *buf, size_t len,
                                int timeout_ms) {
  if (queue < 0 || queue >= Q_MAX)
    return -1;
  TxRing &r = _tx[queue];
  if (len < TX_PKT_DESC_SZ || len > r.bounce_len) {
    _logger->error("PcieTransport: tx_submit_sync len {} out of range", len);
    return -1;
  }

  memcpy(r.bounce, buf, len);

  /* BD slot = a pair of 8-byte entries: entry0 -> the 48-byte tx desc (with
   * psb_len = total length in 128-byte units, plus OWN on the BCN queue),
   * entry1 -> the payload. Port of rtw_pci_tx_write_data. */
  uint16_t psb_len = static_cast<uint16_t>((len - 1) / 128 + 1);
  if (queue == Q_BCN)
    psb_len |= 1u << 15; /* RTK_PCI_TXBD_OWN_OFFSET */

  volatile uint8_t *slot =
      r.bd + static_cast<size_t>(queue == Q_BCN ? 0 : r.wp) * TX_BD_SLOT_SZ;
  bd_write(slot, TX_PKT_DESC_SZ, psb_len, static_cast<uint32_t>(r.bounce_iova));
  bd_write(slot + RX_BD_SZ, static_cast<uint16_t>(len - TX_PKT_DESC_SZ), 0,
           static_cast<uint32_t>(r.bounce_iova + TX_PKT_DESC_SZ));

  std::atomic_thread_fence(std::memory_order_seq_cst);

  if (queue == Q_BCN) {
    /* Kick the beacon queue; completion is the caller-polled bcn-valid latch
     * (REG_FIFOPAGE_CTRL_2+1 bit7), same as the USB rsvd-page contract. */
    _t.mmio_write<uint8_t>(
        RTK_PCI_TXBD_BCN_WORK,
        static_cast<uint8_t>(_t.mmio_read<uint8_t>(RTK_PCI_TXBD_BCN_WORK) |
                             BIT_PCI_BCNQ_FLAG));
    return static_cast<int>(len);
  }

  r.wp = (r.wp + 1) % r.len;
  _t.mmio_write<uint16_t>(r.reg_idx,
                          static_cast<uint16_t>(r.wp & TRX_BD_IDX_MASK));

  /* Sync semantics: wait for the hardware read pointer to consume the slot. */
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 20);
  for (;;) {
    uint32_t idx = _t.mmio_read<uint32_t>(r.reg_idx);
    uint32_t hw_rp = (idx >> 16) & TRX_BD_IDX_MASK;
    if (hw_rp == r.wp)
      return static_cast<int>(len);
    if (std::chrono::steady_clock::now() > deadline) {
      _logger->error("PcieTransport: TX q{} completion timeout (wp={} hw_rp={})",
                     queue, r.wp, hw_rp);
      return -1;
    }
    sleep_us(20);
  }
}

void PcieDma88xx::rx_loop(
    const std::function<void(const uint8_t *, int)> &on_data,
    const std::function<bool()> &should_stop) {
  const int msi_evt = _t.msi_fd();
  const bool msi = msi_evt >= 0;
  _logger->info("PcieTransport: RX loop started ({})",
                msi ? "MSI+eventfd, 100 ms safety timeout" : "polled");
  if (msi) {
    /* Clear any latched status, then unmask RX-OK + ring-underrun. */
    _t.mmio_write<uint32_t>(RTK_PCI_HISR0, _t.mmio_read<uint32_t>(RTK_PCI_HISR0));
    _t.mmio_write<uint32_t>(RTK_PCI_HIMR0, IMR_ROK | IMR_RDU);
  }
  uint64_t reaped = 0;
  while (!should_stop()) {
    uint32_t v = _t.mmio_read<uint32_t>(RTK_PCI_RXBD_IDX_MPDUQ);
    uint32_t hw_wp = (v >> 16) & TRX_BD_IDX_MASK;
    if (hw_wp == _rx.rp) {
      if (msi) {
        /* Wait for the MSI edge. 100 ms timeout = safety net against a lost
         * edge (the ring index re-check above makes a spurious/late wake
         * harmless); always drain the eventfd counter and W1C the HISR so the
         * next frame generates a fresh edge. */
        struct pollfd pfd {msi_evt, POLLIN, 0};
        (void)::poll(&pfd, 1, 100);
        uint64_t cnt;
        while (read(msi_evt, &cnt, sizeof(cnt)) == sizeof(cnt)) {
        }
        _t.mmio_write<uint32_t>(RTK_PCI_HISR0,
                                _t.mmio_read<uint32_t>(RTK_PCI_HISR0));
        continue;
      }
      sleep_us(static_cast<unsigned>(_rx_poll_us));
      continue;
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    while (_rx.rp != hw_wp && !should_stop()) {
      const uint8_t *buf = _rx.bufs + static_cast<size_t>(_rx.rp) * _rx_buf_size;
      /* Exactly one MPDU per RX BD on PCIe (no USB aggregation): usable length
       * = 24-byte rx desc + drvinfo + shift + pkt_len, computed from the
       * descriptor itself (the BD's total_pkt_size write-back carries the DMA
       * tag when RX_TAG_EN is set, not a length). */
      uint32_t d0 = static_cast<uint32_t>(buf[0]) | (buf[1] << 8) |
                    (buf[2] << 16) | (static_cast<uint32_t>(buf[3]) << 24);
      uint32_t pkt_len = d0 & 0x3FFF;
      uint32_t drvinfo = ((d0 >> 16) & 0xF) * 8;
      uint32_t shift = (d0 >> 24) & 0x3;
      uint32_t used = 24 + drvinfo + shift + pkt_len;
      if (pkt_len != 0 && used <= _rx_buf_size)
        on_data(buf, static_cast<int>(used));

      arm_rx_bd(_rx.rp); /* return the slot to hardware */
      _rx.rp = (_rx.rp + 1) % _rx.len;
      ++reaped;
      std::atomic_thread_fence(std::memory_order_release);
      _t.mmio_write<uint16_t>(RTK_PCI_RXBD_IDX_MPDUQ,
                              static_cast<uint16_t>(_rx.rp & TRX_BD_IDX_MASK));
    }
  }
  if (msi)
    _t.mmio_write<uint32_t>(RTK_PCI_HIMR0, 0); /* re-mask on exit */
  _logger->info("PcieTransport: RX loop exited ({} BDs reaped)", reaped);
}

void PcieDma88xx::irq_mask() { _t.mmio_write<uint32_t>(RTK_PCI_HIMR0, 0); }

} /* namespace devourer */
