#include "PcieTransport.h"

/* Linux-only (vfio). Compiled only when DEVOURER_PCIE=ON — the CMake option is
 * gated on Linux. */

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <linux/vfio.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "Event.h"
#if defined(DEVOURER_HAVE_JAGUAR2_8821C)
#include "PcieDma88xx.h"
#endif
#if defined(DEVOURER_HAVE_KESTREL_8852C)
#include "PcieDmaAx.h"
#endif

namespace devourer {

namespace {
constexpr uint16_t kPciDidRtl8852ce = 0xC852;
constexpr uint16_t kPciDidRtl8852be = 0xB852;
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
  if (!t->select_dma_plane())
    return nullptr;
  if (!t->init_dma())
    return nullptr;
  if (cfg.use_msi && !t->setup_msi())
    logger->warn("PcieTransport: MSI setup failed — RX falls back to polling");
  logger->info("PcieTransport: {} ready ({} plane, BAR2 {} KiB, DMA slab {} "
               "KiB @ IOVA 0x{:x}, RX {})",
               bdf, t->_dma->name(), t->_mmio_len / 1024, t->_slab_len / 1024,
               t->_cfg.iova_base,
               t->_msi_evt >= 0 ? "MSI+eventfd" : "polled");
  return t;
}

bool PcieTransport::select_dma_plane() {
  switch (_pci_did) {
  case kPciDidRtl8852ce:
#if defined(DEVOURER_HAVE_KESTREL_8852C)
    _dma = std::make_unique<PcieDmaAx>(*this, _logger, _cfg.rx_poll_us);
    return true;
#else
    _logger->error("PcieTransport: RTL8852CE ({:04x}:{:04x}) needs "
                   "DEVOURER_KESTREL_8852C=ON",
                   _pci_vid, _pci_did);
    return false;
#endif
  case kPciDidRtl8852be:
    _logger->error("PcieTransport: RTL8852BE ({:04x}:{:04x}) is not ported "
                   "(its PCIe power sequence and single-BDRAM ring table are "
                   "unported; the 8852CE is)",
                   _pci_vid, _pci_did);
    return false;
  default:
#if defined(DEVOURER_HAVE_JAGUAR2_8821C)
    /* The HalMAC 88xx ring plane; the factory still checks the chip-id. */
    _dma = std::make_unique<PcieDma88xx>(*this, _logger, _cfg.rx_ring_len,
                                         _cfg.rx_buf_size, _cfg.rx_poll_us);
    return true;
#else
    _logger->error("PcieTransport: PCI device {:04x}:{:04x} has no DMA plane "
                   "in this build (DEVOURER_JAGUAR2_8821C=OFF)",
                   _pci_vid, _pci_did);
    return false;
#endif
  }
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
    if (_mmio && _dma)
      _dma->irq_mask(); /* mask before dropping the vector */
    struct vfio_irq_set off{};
    off.argsz = sizeof(off);
    off.flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER;
    off.index = VFIO_PCI_MSI_IRQ_INDEX;
    off.count = 0;
    ioctl(_device, VFIO_DEVICE_SET_IRQS, &off);
    close(_msi_evt);
  }
  _dma.reset();
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
  _pci_vid = vid;
  _pci_did = did;
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
  _slab_len = _dma->slab_bytes();
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
  return _dma->attach(_slab, _cfg.iova_base);
}

void PcieTransport::warn_oob(uint32_t off) {
  if (!_warned_oob.exchange(true))
    _logger->warn("PcieTransport: register 0x{:x} is past the {} KiB BAR2 — "
                  "access dropped (further ones silent)",
                  off, _mmio_len / 1024);
}

bool PcieTransport::write32_wide(uint32_t addr, uint32_t v) {
  if (addr + 4 > _mmio_len) {
    warn_oob(addr);
    return false;
  }
  *reinterpret_cast<volatile uint32_t *>(_mmio + addr) = v;
  return true;
}

uint32_t PcieTransport::read32_wide(uint32_t addr) {
  if (addr + 4 > _mmio_len) {
    warn_oob(addr);
    return 0;
  }
  return *reinterpret_cast<volatile uint32_t *>(_mmio + addr);
}

int PcieTransport::tx_sync(uint8_t ep, uint8_t *buf, size_t len,
                           int timeout_ms) {
  _tx_submitted.fetch_add(1, std::memory_order_relaxed);
  int rc = _dma->tx_submit(ep, buf, len, timeout_ms);
  if (rc < 0) {
    _tx_failed.fetch_add(1, std::memory_order_relaxed);
    _tx_last_rc.store(rc, std::memory_order_relaxed);
    devourer::Ev(_logger->events(), "tx.fail").f("rc", rc).f("timeout", true);
  }
  return rc;
}

devourer::TxStats PcieTransport::tx_stats() const {
  devourer::TxStats s;
  s.submitted = _tx_submitted.load(std::memory_order_relaxed);
  s.failed = _tx_failed.load(std::memory_order_relaxed);
  s.last_error_rc = _tx_last_rc.load(std::memory_order_relaxed);
  s.last_was_timeout = s.failed != 0;
  return s;
}

} /* namespace devourer */
