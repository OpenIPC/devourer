#include "PcieTransport.h"

/* Linux-only (vfio). Compiled only when DEVOURER_PCIE=ON — the CMake option is
 * gated on Linux. */

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

#include <fcntl.h>
#include <linux/vfio.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>

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

constexpr size_t PAGE_SZ = 4096;
inline size_t page_align(size_t v) { return (v + PAGE_SZ - 1) & ~(PAGE_SZ - 1); }

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

void sleep_us(unsigned us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

} /* namespace */

std::shared_ptr<PcieTransport> PcieTransport::Open(const std::string &bdf,
                                                   Logger_t logger) {
  return Open(bdf, std::move(logger), Config{});
}

std::shared_ptr<PcieTransport> PcieTransport::Open(const std::string &bdf,
                                                   Logger_t logger,
                                                   const Config &cfg) {
  std::shared_ptr<PcieTransport> t(new PcieTransport(logger, cfg));
  t->_bdf = bdf;
  if (!t->open_vfio(bdf))
    return nullptr;
  if (!t->map_bar2())
    return nullptr;
  if (!t->setup_config_space())
    return nullptr;
  if (!t->init_dma())
    return nullptr;
  if (cfg.use_msi && !t->setup_msi())
    logger->warn("PcieTransport: MSI setup failed — RX falls back to polling");
  logger->info("PcieTransport: {} ready (BAR2 {} KiB, DMA slab {} KiB @ IOVA "
               "0x{:x}, RX {})",
               bdf, t->_mmio_len / 1024, t->_slab_len / 1024, t->_cfg.iova_base,
               t->_msi_evt >= 0 ? "MSI+eventfd" : "polled");
  return t;
}

bool PcieTransport::setup_msi() {
  struct vfio_irq_info info{};
  info.argsz = sizeof(info);
  info.index = VFIO_PCI_MSI_IRQ_INDEX;
  if (ioctl(_device, VFIO_DEVICE_GET_IRQ_INFO, &info) < 0 || info.count < 1) {
    _logger->warn("PcieTransport: no MSI IRQ available");
    return false;
  }
  int evt = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (evt < 0)
    return false;
  /* One MSI vector -> the eventfd. */
  char buf[sizeof(struct vfio_irq_set) + sizeof(int32_t)] = {};
  auto *is = reinterpret_cast<struct vfio_irq_set *>(buf);
  is->argsz = sizeof(buf);
  is->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
  is->index = VFIO_PCI_MSI_IRQ_INDEX;
  is->start = 0;
  is->count = 1;
  memcpy(is->data, &evt, sizeof(int32_t));
  if (ioctl(_device, VFIO_DEVICE_SET_IRQS, is) < 0) {
    _logger->warn("PcieTransport: VFIO_DEVICE_SET_IRQS(MSI) failed: {}",
                  strerror(errno));
    close(evt);
    return false;
  }
  _msi_evt = evt;
  return true;
}

PcieTransport::~PcieTransport() {
  if (_msi_evt >= 0) {
    if (_mmio)
      mw<uint32_t>(RTK_PCI_HIMR0, 0); /* mask before dropping the vector */
    struct vfio_irq_set off{};
    off.argsz = sizeof(off);
    off.flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER;
    off.index = VFIO_PCI_MSI_IRQ_INDEX;
    off.count = 0;
    ioctl(_device, VFIO_DEVICE_SET_IRQS, &off);
    close(_msi_evt);
  }
  if (_mmio)
    munmap(const_cast<uint8_t *>(_mmio), _mmio_len);
  if (_slab && _container >= 0) {
    struct vfio_iommu_type1_dma_unmap um{};
    um.argsz = sizeof(um);
    um.iova = _cfg.iova_base;
    um.size = _slab_len;
    ioctl(_container, VFIO_IOMMU_UNMAP_DMA, &um);
  }
  if (_slab)
    munmap(_slab, _slab_len);
  if (_device >= 0)
    close(_device);
  if (_group >= 0)
    close(_group);
  if (_container >= 0)
    close(_container);
}

bool PcieTransport::open_vfio(const std::string &bdf) {
  /* IOMMU group number from sysfs. */
  std::string link = "/sys/bus/pci/devices/" + bdf + "/iommu_group";
  char buf[256];
  ssize_t n = readlink(link.c_str(), buf, sizeof(buf) - 1);
  if (n <= 0) {
    _logger->error("PcieTransport: readlink({}) failed: {}", link,
                   strerror(errno));
    return false;
  }
  buf[n] = 0;
  const char *slash = strrchr(buf, '/');
  std::string group_num = slash ? slash + 1 : buf;

  _container = open("/dev/vfio/vfio", O_RDWR);
  if (_container < 0) {
    _logger->error("PcieTransport: open /dev/vfio/vfio failed: {} (modprobe "
                   "vfio-pci?)",
                   strerror(errno));
    return false;
  }
  if (ioctl(_container, VFIO_GET_API_VERSION) != VFIO_API_VERSION) {
    _logger->error("PcieTransport: VFIO API version mismatch");
    return false;
  }
  int iommu_type = 0;
  if (ioctl(_container, VFIO_CHECK_EXTENSION, VFIO_TYPE1v2_IOMMU) == 1)
    iommu_type = VFIO_TYPE1v2_IOMMU;
  else if (ioctl(_container, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU) == 1)
    iommu_type = VFIO_TYPE1_IOMMU;
  else {
    _logger->error("PcieTransport: no Type1 IOMMU support");
    return false;
  }

  std::string group_path = "/dev/vfio/" + group_num;
  _group = open(group_path.c_str(), O_RDWR);
  if (_group < 0) {
    _logger->error("PcieTransport: open {} failed: {} (device bound to "
                   "vfio-pci? permissions?)",
                   group_path, strerror(errno));
    return false;
  }
  struct vfio_group_status st{};
  st.argsz = sizeof(st);
  if (ioctl(_group, VFIO_GROUP_GET_STATUS, &st) < 0 ||
      !(st.flags & VFIO_GROUP_FLAGS_VIABLE)) {
    _logger->error("PcieTransport: IOMMU group {} not viable (all devices in "
                   "the group must be bound to vfio-pci)",
                   group_num);
    return false;
  }
  if (ioctl(_group, VFIO_GROUP_SET_CONTAINER, &_container) < 0) {
    _logger->error("PcieTransport: GROUP_SET_CONTAINER failed: {}",
                   strerror(errno));
    return false;
  }
  if (ioctl(_container, VFIO_SET_IOMMU, iommu_type) < 0) {
    _logger->error("PcieTransport: SET_IOMMU failed: {}", strerror(errno));
    return false;
  }
  _device = ioctl(_group, VFIO_GROUP_GET_DEVICE_FD, bdf.c_str());
  if (_device < 0) {
    _logger->error("PcieTransport: GET_DEVICE_FD({}) failed: {}", bdf,
                   strerror(errno));
    return false;
  }
  _logger->info("PcieTransport: vfio group {} opened for {}", group_num, bdf);
  return true;
}

bool PcieTransport::map_bar2() {
  struct vfio_region_info reg{};
  reg.argsz = sizeof(reg);
  reg.index = VFIO_PCI_BAR2_REGION_INDEX;
  if (ioctl(_device, VFIO_DEVICE_GET_REGION_INFO, &reg) < 0) {
    _logger->error("PcieTransport: BAR2 region info failed: {}",
                   strerror(errno));
    return false;
  }
  if (!(reg.flags & VFIO_REGION_INFO_FLAG_MMAP) || reg.size == 0) {
    _logger->error("PcieTransport: BAR2 not mmap-able (size={} flags={:#x})",
                   (unsigned long long)reg.size, reg.flags);
    return false;
  }
  void *p = mmap(nullptr, reg.size, PROT_READ | PROT_WRITE, MAP_SHARED,
                 _device, reg.offset);
  if (p == MAP_FAILED) {
    _logger->error("PcieTransport: BAR2 mmap failed: {}", strerror(errno));
    return false;
  }
  _mmio = static_cast<volatile uint8_t *>(p);
  _mmio_len = reg.size;
  return true;
}

bool PcieTransport::cfg_read(uint32_t off, void *buf, size_t len) {
  return pread(_device, buf, len, _cfg_region_off + off) ==
         static_cast<ssize_t>(len);
}

bool PcieTransport::cfg_write(uint32_t off, const void *buf, size_t len) {
  return pwrite(_device, buf, len, _cfg_region_off + off) ==
         static_cast<ssize_t>(len);
}

bool PcieTransport::setup_config_space() {
  struct vfio_region_info reg{};
  reg.argsz = sizeof(reg);
  reg.index = VFIO_PCI_CONFIG_REGION_INDEX;
  if (ioctl(_device, VFIO_DEVICE_GET_REGION_INFO, &reg) < 0) {
    _logger->error("PcieTransport: config region info failed: {}",
                   strerror(errno));
    return false;
  }
  _cfg_region_off = reg.offset;
  _cfg_region_len = reg.size;

  uint16_t vid = 0, did = 0;
  cfg_read(0x00, &vid, 2);
  cfg_read(0x02, &did, 2);
  if (vid == 0xFFFF) {
    _logger->error("PcieTransport: config space reads 0xFFFF — link down?");
    return false;
  }
  _logger->info("PcieTransport: PCI device {:04x}:{:04x}", vid, did);

  /* Memory + bus-master enable. Without bus-master every register read works
   * but no DMA moves — the classic silent-failure trap. */
  uint16_t cmd = 0;
  cfg_read(0x04, &cmd, 2);
  cmd |= 0x2 /* MEMORY */ | 0x4 /* MASTER */;
  cfg_write(0x04, &cmd, 2);

  /* Find the PCI Express capability (id 0x10) for LNKCTL / DEVCTL2. */
  uint8_t cap_ptr = 0;
  cfg_read(0x34, &cap_ptr, 1);
  uint32_t pcie_cap = 0;
  for (int guard = 0; cap_ptr && guard < 48; guard++) {
    uint8_t id = 0, next = 0;
    cfg_read(cap_ptr, &id, 1);
    cfg_read(cap_ptr + 1, &next, 1);
    if (id == 0x10) {
      pcie_cap = cap_ptr;
      break;
    }
    cap_ptr = next;
  }
  if (pcie_cap) {
    /* Clear ASPM (LNKCTL[1:0]) during bring-up — L1 entry on a half-configured
     * link is a known hang source. */
    uint16_t lnkctl = 0;
    cfg_read(pcie_cap + 0x10, &lnkctl, 2);
    if (lnkctl & 0x3) {
      lnkctl &= ~0x3;
      cfg_write(pcie_cap + 0x10, &lnkctl, 2);
      _logger->info("PcieTransport: ASPM disabled for bring-up");
    }
    /* Disable completion timeout (DEVCTL2 bit4) — rtw88 does this specifically
     * for the 8821C (rtw_pci_phy_cfg). */
    uint16_t devctl2 = 0;
    cfg_read(pcie_cap + 0x28, &devctl2, 2);
    devctl2 |= 1u << 4;
    cfg_write(pcie_cap + 0x28, &devctl2, 2);
  } else {
    _logger->warn("PcieTransport: PCIe capability not found — skipping "
                  "ASPM/completion-timeout config");
  }
  return true;
}

bool PcieTransport::init_dma() {
  /* Slab layout (page-aligned sections):
   *   [0]  8 × TX BD rings   (4 KiB each — BE at 256×16 = 4 KiB is the max)
   *   [1]  8 × TX bounce      (32 KiB each; sync TX = one in flight per queue)
   *   [2]  RX BD ring         (rx_ring_len × 8)
   *   [3]  RX buffers         (rx_ring_len × rx_buf_size) */
  const uint32_t rxn = _cfg.rx_ring_len;
  size_t off_txbd = 0;
  size_t off_bounce = off_txbd + Q_MAX * PAGE_SZ;
  size_t off_rxbd = off_bounce + Q_MAX * TX_BOUNCE_SZ;
  size_t off_rxbuf = off_rxbd + page_align(rxn * RX_BD_SZ);
  _slab_len = page_align(off_rxbuf + static_cast<size_t>(rxn) * _cfg.rx_buf_size);

  void *p = mmap(nullptr, _slab_len, PROT_READ | PROT_WRITE,
                 MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  if (p == MAP_FAILED) {
    _logger->error("PcieTransport: DMA slab mmap({} KiB) failed: {}",
                   _slab_len / 1024, strerror(errno));
    return false;
  }
  _slab = static_cast<uint8_t *>(p);
  memset(_slab, 0, _slab_len);

  if (_cfg.iova_base + _slab_len > (1ull << 32)) {
    _logger->error("PcieTransport: IOVA base 0x{:x} + slab exceeds 4 GiB (the "
                   "descriptor dma fields are 32-bit)",
                   _cfg.iova_base);
    return false;
  }
  struct vfio_iommu_type1_dma_map m{};
  m.argsz = sizeof(m);
  m.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE;
  m.vaddr = reinterpret_cast<uint64_t>(_slab);
  m.iova = _cfg.iova_base;
  m.size = _slab_len;
  if (ioctl(_container, VFIO_IOMMU_MAP_DMA, &m) < 0) {
    _logger->error("PcieTransport: VFIO_IOMMU_MAP_DMA failed: {}",
                   strerror(errno));
    return false;
  }

  auto va = [&](size_t off) { return _slab + off; };
  auto iova = [&](size_t off) { return _cfg.iova_base + off; };

  static const struct {
    uint32_t len;
    uint16_t desa, num, idx;
  } kQ[Q_MAX] = {
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

void PcieTransport::arm_rx_bd(uint32_t idx) {
  /* {buf_size, total_pkt_size = 0 (HW write-back), dma} — port of
   * rtw_pci_reset_rx_desc. */
  bd_write(_rx.bd + static_cast<size_t>(idx) * RX_BD_SZ,
           static_cast<uint16_t>(_cfg.rx_buf_size), 0,
           static_cast<uint32_t>(_rx.bufs_iova +
                                 static_cast<uint64_t>(idx) * _cfg.rx_buf_size));
}

void PcieTransport::setup_trx_rings() {
  /* Port of rtw_pci_reset_buf_desc — EXACT order (register-level quirks:
   * 0x300+3 |= 0xf7 first, RWPTR clear + H2CQ CSR clear last), then
   * rtw_pci_dma_reset. */
  mw<uint8_t>(RTK_PCI_CTRL + 3,
              static_cast<uint8_t>(mr<uint8_t>(RTK_PCI_CTRL + 3) | 0xf7));

  /* BCN has no NUM register ("specialized for rsvd page"). */
  mw<uint32_t>(RTK_PCI_TXBD_DESA_BCNQ, static_cast<uint32_t>(_tx[Q_BCN].bd_iova));

  /* H2C (wcpu-11ac chips). */
  _tx[Q_H2C].wp = 0;
  mw<uint16_t>(RTK_PCI_TXBD_NUM_H2CQ,
               static_cast<uint16_t>(_tx[Q_H2C].len & TRX_BD_IDX_MASK));
  mw<uint32_t>(RTK_PCI_TXBD_DESA_H2CQ, static_cast<uint32_t>(_tx[Q_H2C].bd_iova));

  for (int q : {Q_BK, Q_BE, Q_VO, Q_VI, Q_MGMT, Q_HI0}) {
    _tx[q].wp = 0;
    mw<uint16_t>(_tx[q].reg_num,
                 static_cast<uint16_t>(_tx[q].len & TRX_BD_IDX_MASK));
    mw<uint32_t>(_tx[q].reg_desa, static_cast<uint32_t>(_tx[q].bd_iova));
  }

  _rx.rp = 0;
  for (uint32_t i = 0; i < _rx.len; i++)
    arm_rx_bd(i);
  mw<uint16_t>(RTK_PCI_RXBD_NUM_MPDUQ,
               static_cast<uint16_t>(_rx.len & TRX_BD_IDX_MASK));
  mw<uint32_t>(RTK_PCI_RXBD_DESA_MPDUQ, static_cast<uint32_t>(_rx.bd_iova));

  /* reset read/write pointers */
  mw<uint32_t>(RTK_PCI_TXBD_RWPTR_CLR, 0xffffffff);
  /* reset H2C queue indices in a single write (wcpu-11ac) */
  mw<uint32_t>(RTK_PCI_TXBD_H2CQ_CSR,
               mr<uint32_t>(RTK_PCI_TXBD_H2CQ_CSR) | BIT_CLR_H2CQ_HOST_IDX |
                   BIT_CLR_H2CQ_HW_IDX);

  /* rtw_pci_dma_reset */
  mw<uint32_t>(RTK_PCI_CTRL, mr<uint32_t>(RTK_PCI_CTRL) | BIT_RST_TRXDMA_INTF |
                                 BIT_RX_TAG_EN);
  _logger->info("PcieTransport: TRX rings programmed (RXBD {} slots @ IOVA "
                "0x{:x})",
                _rx.len, _rx.bd_iova);
}

int PcieTransport::tx_submit_sync(int queue, const uint8_t *buf, size_t len,
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
    mw<uint8_t>(RTK_PCI_TXBD_BCN_WORK,
                static_cast<uint8_t>(mr<uint8_t>(RTK_PCI_TXBD_BCN_WORK) |
                                     BIT_PCI_BCNQ_FLAG));
    return static_cast<int>(len);
  }

  r.wp = (r.wp + 1) % r.len;
  mw<uint16_t>(r.reg_idx, static_cast<uint16_t>(r.wp & TRX_BD_IDX_MASK));

  /* Sync semantics: wait for the hardware read pointer to consume the slot. */
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 20);
  for (;;) {
    uint32_t idx = mr<uint32_t>(r.reg_idx);
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

void PcieTransport::rx_poll_loop(
    const std::function<void(const uint8_t *, int)> &on_data,
    const std::function<bool()> &should_stop) {
  const bool msi = _msi_evt >= 0;
  _logger->info("PcieTransport: RX loop started ({})",
                msi ? "MSI+eventfd, 100 ms safety timeout"
                    : "polled");
  if (msi) {
    /* Clear any latched status, then unmask RX-OK + ring-underrun. */
    mw<uint32_t>(RTK_PCI_HISR0, mr<uint32_t>(RTK_PCI_HISR0));
    mw<uint32_t>(RTK_PCI_HIMR0, IMR_ROK | IMR_RDU);
  }
  uint64_t reaped = 0;
  while (!should_stop()) {
    uint32_t v = mr<uint32_t>(RTK_PCI_RXBD_IDX_MPDUQ);
    uint32_t hw_wp = (v >> 16) & TRX_BD_IDX_MASK;
    if (hw_wp == _rx.rp) {
      if (msi) {
        /* Wait for the MSI edge. 100 ms timeout = safety net against a lost
         * edge (the ring index re-check above makes a spurious/late wake
         * harmless); always drain the eventfd counter and W1C the HISR so the
         * next frame generates a fresh edge. */
        struct pollfd pfd {_msi_evt, POLLIN, 0};
        (void)::poll(&pfd, 1, 100);
        uint64_t cnt;
        while (read(_msi_evt, &cnt, sizeof(cnt)) == sizeof(cnt)) {
        }
        mw<uint32_t>(RTK_PCI_HISR0, mr<uint32_t>(RTK_PCI_HISR0));
        continue;
      }
      sleep_us(static_cast<unsigned>(_cfg.rx_poll_us));
      continue;
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    while (_rx.rp != hw_wp && !should_stop()) {
      const uint8_t *buf = _rx.bufs + static_cast<size_t>(_rx.rp) * _cfg.rx_buf_size;
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
      if (pkt_len != 0 && used <= _cfg.rx_buf_size)
        on_data(buf, static_cast<int>(used));

      arm_rx_bd(_rx.rp); /* return the slot to hardware */
      _rx.rp = (_rx.rp + 1) % _rx.len;
      ++reaped;
      std::atomic_thread_fence(std::memory_order_release);
      mw<uint16_t>(RTK_PCI_RXBD_IDX_MPDUQ,
                   static_cast<uint16_t>(_rx.rp & TRX_BD_IDX_MASK));
    }
  }
  if (msi)
    mw<uint32_t>(RTK_PCI_HIMR0, 0); /* re-mask on exit */
  _logger->info("PcieTransport: RX loop exited ({} BDs reaped)", reaped);
}

} /* namespace devourer */
