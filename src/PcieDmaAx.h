#pragma once

/* PcieDmaAx — the AX-generation (Wi-Fi 6, rtw89) PCIe DMA plane for the
 * RTL8852CE: the HAXI host interface. Register-level contract from the vendor
 * mac_ax `_pcie_8852c.c` (ring register map, BDRAM table) and the host-side
 * ring/descriptor contract from rtw89 pci.{c,h} (the vendor USB drop does not
 * ship its PCIe ring driver).
 *
 * TX: 13 channels (ACH0..7 data, CH8 B0MG mgmt, CH9 B0HI, CH10/11 band 1,
 * CH12 FWCMD), each a 256-entry ring of 8-byte TXBDs {len, option(LS), dma}.
 *   - CH12 carries FWDL + H2C: the HAL's [16-byte rxd_short][payload] buffer
 *     is DMA'd as-is from a bounce slot; completion = the CH12 hardware index
 *     passing the slot. No WD page, no release report.
 *   - Data/mgmt channels point the TXBD at a 128-byte WD page: the HAL's
 *     [wd_body_v1 32 B][wd_info 24 B][frame] is split into the page
 *     (body + info + an 8-byte wp_info {seq|VALID} + up to 10 six-byte
 *     addr_info_v1 entries referencing a per-page payload bounce) and the
 *     payload bounce. The TXBD being consumed only means the WD was fetched:
 *     the payload is DMA'd later through the addr_info, so page + bounce are
 *     freed by the release report (RPP) that names the page's seq — never by
 *     the TXBD index.
 * RX: two rings of 8-byte RXBDs {buf_size, rsvd, dma} over 11494-byte
 * buffers. RXQ delivers frames / PPDU status / C2H, one packet per BD in
 * RXBD_PKT mode (FS/LS spanning handled anyway); RPQ delivers release
 * packets: an rx descriptor followed by 4-byte RPP words {POLLUTED 31,
 * SEQ[30:16], TX_STATUS[15:13], QSEL[12:8], MACID[7:0]}. Each buffer begins
 * with a 4-byte rxbd_info {FS 15, LS 14, len[13:0] incl. itself, tag[28:16]}.
 *
 * Indices: the hardware read index is rd32(idx)[27:16], the host write index
 * is wr16(idx), both 12-bit. desa_h is never written: every ring, page and
 * buffer IOVA is below 4 GiB (the transport's slab guarantees it).
 *
 * The HAL's queue handle passed as `hint` IS the AX DMA channel (0 ACH0,
 * 8 B0MG, 12 FWCMD). The pre/post-init register sequences that surround
 * setup_rings() (HAXI mode, DMA stop/start, LTR) are the HAL's
 * (HalKestrel::pcie_pre_init / pcie_init) — they are MAC registers, not ring
 * memory. */

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

#include "PcieDmaPlane.h"
#include "logger.h"

namespace devourer {

class PcieTransport;

class PcieDmaAx final : public IPcieDmaPlane {
public:
  static constexpr uint32_t kTxChannels = 13;
  static constexpr uint32_t kFwcmdCh = 12;
  static constexpr uint32_t kBdLen = 256;        /* RTW89_PCI_TXBD/RXBD_NUM_MAX */
  static constexpr uint32_t kWdPages = 512;      /* RTW89_PCI_TXWD_NUM_MAX */
  static constexpr uint32_t kWdPageSize = 128;   /* RTW89_PCI_TXWD_PAGE_SIZE */
  static constexpr uint32_t kRxBufSize = 11496;  /* RTW89_PCI_RX_BUF_SIZE 11494, 8-aligned */
  static constexpr uint32_t kPayloadSize = 4096; /* per-page frame bounce */
  static constexpr uint32_t kFwcmdSlotSize = 2304; /* rxd 16 + fwcmd hdr 8 + 2020 section, rounded */

  PcieDmaAx(PcieTransport &t, Logger_t logger, int rx_poll_us);

  const char *name() const override { return "ax-haxi"; }
  size_t slab_bytes() const override;
  bool attach(uint8_t *slab_va, uint64_t slab_iova) override;
  void setup_rings() override;
  int tx_submit(uint8_t hint, uint8_t *buf, size_t len,
                int timeout_ms) override;
  void rx_loop(const std::function<void(const uint8_t *, int)> &on_data,
               const std::function<bool()> &should_stop) override;
  void irq_mask() override;

  /* Release-report accounting (RPP tx_status), for diagnostics. */
  struct RppStats {
    uint64_t tx_done = 0, retry_limit = 0, lifetime = 0, macid_drop = 0;
    uint64_t unknown_seq = 0, pages_freed = 0;
  };
  RppStats rpp_stats() const;

private:
  struct TxRing {
    volatile uint8_t *bd = nullptr; /* kBdLen × 8 B */
    uint64_t bd_iova = 0;
    uint32_t wp = 0;
    uint16_t reg_num = 0, reg_idx = 0, reg_bdram = 0, reg_desa_l = 0;
    uint32_t bdram = 0;
  };
  struct RxRing {
    volatile uint8_t *bd = nullptr; /* kBdLen × 8 B */
    uint64_t bd_iova = 0;
    uint8_t *bufs = nullptr; /* kBdLen × kRxBufSize */
    uint64_t bufs_iova = 0;
    uint32_t wp = 0;
    uint16_t reg_num = 0, reg_idx = 0, reg_desa_l = 0;
  };

  void arm_rx_bd(RxRing &r, uint32_t idx);
  uint32_t hw_idx(uint16_t reg_idx);
  bool wait_consumed(TxRing &r, int ch, int timeout_ms);
  int submit_fwcmd(uint8_t *buf, size_t len, int timeout_ms);
  int submit_wd(int ch, uint8_t *buf, size_t len, int timeout_ms);
  /* Reap the RPQ ring: free the WD pages the release reports name. Returns
   * the number of RPQ buffers consumed. Caller holds _pool_mu. */
  uint32_t reap_rpq_locked();
  void release_page(uint32_t seq, uint32_t status);
  /* Reap the RXQ ring, delivering packets. Returns buffers consumed. */
  uint32_t reap_rxq(const std::function<void(const uint8_t *, int)> &on_data);

  PcieTransport &_t;
  Logger_t _logger;
  int _rx_poll_us;

  std::array<TxRing, kTxChannels> _tx{};
  RxRing _rxq{}, _rpq{};

  /* WD page pool: page VA/IOVA + its payload bounce; freed by RPP seq. */
  uint8_t *_wd_pages = nullptr;
  uint64_t _wd_pages_iova = 0;
  uint8_t *_payload = nullptr;
  uint64_t _payload_iova = 0;
  std::vector<uint16_t> _free_pages;       /* LIFO of free page indices */
  std::array<bool, kWdPages> _page_busy{}; /* in flight until its RPP */
  std::mutex _pool_mu;                     /* pool + RPQ ring state */
  std::mutex _tx_mu;                       /* serializes submitters */

  uint8_t *_fwcmd = nullptr; /* kBdLen × kFwcmdSlotSize bounce ring (CH12) */
  uint64_t _fwcmd_iova = 0;

  std::vector<uint8_t> _rx_assembly; /* FS..LS spanning reassembly */
  bool _rx_assembling = false;

  RppStats _rpp{};
  uint64_t _rx_delivered = 0;
  /* RXQ diagnostics: per rpkt_type packet counts (rxd dword0 [27:24]) and
   * BDs dropped for an implausible rxbd_info length. */
  std::array<uint64_t, 16> _rx_types{};
  uint64_t _rx_bad_len = 0;
  std::atomic<bool> _dead_logged{false};
};

} /* namespace devourer */
