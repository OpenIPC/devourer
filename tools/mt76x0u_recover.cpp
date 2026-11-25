/*
 * mt76x0u_recover - step-by-step reset probe for an MT7610U whose firmware
 * reports running (MT_MCU_COM_REG0 == 1) but does not answer MCU commands.
 *
 * Each argument is one step, run in order; the chip state is printed after
 * every step. Nothing runs implicitly, so a session can stop at the first step
 * that changes something.
 *
 *   state              print the registers below (also printed after each step)
 *   ping               Q_SELECT through the MCU with a response wait
 *   intlevel           MT_MCU_INT_LEVEL = 1, 30 ms (mt76x02_mcu_cleanup)
 *   fwreset            DEV_MODE wValue 1 (mt76x02u_mcu_fw_reset), 10 ms
 *   pbfreset           pulse MT_PBF_SYS_CTRL MCU|DMA|MAC|PBF|ASY reset
 *                      (mt76x2/pci_init.c mt76x2_reset_wlan pattern)
 *   dmareset           pulse MT_PBF_SYS_CTRL DMA|PBF reset (MCU untouched)
 *   mcureset           pulse MT_PBF_SYS_CTRL MCU_RESET only
 *   chipoff            mt76x0_chip_onoff(false, false)
 *   chipon             mt76x0_chip_onoff(true, false)
 *   chipreset          mt76x0_chip_onoff(true, true): the WLAN reset pulse,
 *                      which only fires when WLAN_EN is already set
 *   clearcom0          MT_MCU_COM_REG0 = 0
 *   upload DIR         mt76x0u_load_firmware(force) from DIR
 *   bringup DIR        mt76x0u_bring_up(DIR, no force)
 *   macstop            mt76x0u_stop (MAC TX/RX off), as Mt76x0uRadio::Stop()
 *   mculog FILE        log every MCU command sent (cmd, seq, payload)
 *   drain              read EP 0x84 until empty (clears USB_DMA_CFG RX_BUSY
 *                      left by a process killed with its receiver on)
 *   rx CH SECS        set channel CH (20 MHz), RX ring, MAC start, then per
 *                      second: frames over USB and the MAC's RX error counters
 *   usbreset           libusb_reset_device + re-claim, as devourer's open
 *                      (exits if the device re-enumerated)
 *   sleep MS
 *
 * Build (no CMake target; standalone against the backend sources):
 *   g++ -std=c++20 -O1 -Isrc/mt76x0u -Isrc/mt76x0u/include \
 *       $(pkg-config --cflags libusb-1.0) tools/mt76x0u_recover.cpp \
 *       src/mt76x0u/{usb,async,mcu,fw,eeprom,init,phy,chan,mac,rx}.cpp \
 *       $(pkg-config --libs libusb-1.0) -lpthread -o mt76x0u_recover
 */
#include <libusb.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "internal.h"
#include "regs.h"
#include "mt76x0u/mt76x0u.h"

namespace {

constexpr uint32_t kPbfMcuReset = 1u << 0;
constexpr uint32_t kPbfAllReset = 0x1f; /* MCU DMA MAC PBF ASY */

void sink(void *, char level, const char *line) {
  std::printf("    [%c] %s\n", level, line);
}

void reg(mt76x0u_dev *d, const char *name, uint32_t addr) {
  uint32_t v = 0;
  if (mt0_rr_chk(d, addr, &v))
    std::printf("  %-16s 0x%04x = READ FAILED\n", name, addr);
  else
    std::printf("  %-16s 0x%04x = 0x%08x\n", name, addr, v);
}

void state(mt76x0u_dev *d) {
  mt0_io_clear(d);
  reg(d, "COM_REG0", MT_MCU_COM_REG0);
  reg(d, "WLAN_FUN_CTRL", MT_WLAN_FUN_CTRL);
  reg(d, "CMB_CTRL", MT_CMB_CTRL);
  reg(d, "USB_DMA_CFG", MT_USB_DMA_CFG);
  reg(d, "PBF_SYS_CTRL", MT_PBF_SYS_CTRL);
  reg(d, "FCE_PSE_CTRL", MT_FCE_PSE_CTRL);
  reg(d, "MAC_SYS_CTRL", MT_MAC_SYS_CTRL);
  reg(d, "MAC_CSR0", MT_MAC_CSR0);
  /* The in-band command path, EP 8 -> FCE -> MCU (programmed by the kernel
   * only in the firmware upload), and the PBF queue page counts. */
  reg(d, "FCE_BASE_PTR", MT_TX_CPU_FROM_FCE_BASE_PTR);
  reg(d, "FCE_MAX_COUNT", MT_TX_CPU_FROM_FCE_MAX_COUNT);
  reg(d, "FCE_CPU_DESC_IDX", MT_TX_CPU_FROM_FCE_CPU_DESC_IDX);
  reg(d, "FCE_PDMA_GLOBAL", MT_FCE_PDMA_GLOBAL_CONF);
  reg(d, "FCE_SKIP_FS", MT_FCE_SKIP_FS);
  reg(d, "PBF_0438", 0x0438);
  reg(d, "PBF_0A30", 0x0a30);
  reg(d, "PBF_0A34", 0x0a34);
}

void pulse_pbf(mt76x0u_dev *d, uint32_t bits) {
  uint32_t v = 0;
  if (mt0_rr_chk(d, MT_PBF_SYS_CTRL, &v)) {
    std::printf("  PBF_SYS_CTRL unreadable, skipped\n");
    return;
  }
  mt0_wr(d, MT_PBF_SYS_CTRL, v | bits);
  mt0_usleep(1000);
  mt0_wr(d, MT_PBF_SYS_CTRL, v & ~bits);
  mt0_usleep(50000);
}

struct RxCount {
  std::atomic<unsigned> frames{0}, crc{0};
};

void count_cb(void *user, const void *, size_t, const mt76x0u_rx_info *info) {
  auto *c = static_cast<RxCount *>(user);
  c->frames++;
  if (info && info->crc_err)
    c->crc++;
}

/* MT_RX_STAT_0..2 (mt76x02_regs.h), clear-on-read: 0 = CRC | PHY errors,
 * 1 = false CCA | PLCP errors, 2 = duplicate | overflow. */
void rx_stats(mt76x0u_dev *d, unsigned frames, unsigned crc) {
  uint32_t s0 = 0, s1 = 0, s2 = 0;
  mt0_rr_chk(d, 0x1700, &s0);
  mt0_rr_chk(d, 0x1704, &s1);
  mt0_rr_chk(d, 0x1708, &s2);
  std::printf("  usb frames %5u (crc %u) | MAC crc %u phy %u falsecca %u plcp %u dup %u ovf %u\n",
              frames, crc, s0 & 0xffff, s0 >> 16, s1 & 0xffff, s1 >> 16, s2 & 0xffff,
              s2 >> 16);
}

/* Channel, RX ring, MAC start, then per-second counts. */
int rx_test(mt76x0u_dev *d, int ch, int secs) {
  RxCount c;
  if (mt76x0u_set_channel(d, static_cast<uint8_t>(ch), 0)) {
    std::printf("  set_channel %d failed\n", ch);
    return -1;
  }
  if (mt76x0u_rx_start(d, count_cb, &c) || mt76x0u_start(d, 1)) {
    std::printf("  rx_start/start failed\n");
    return -1;
  }
  uint32_t sys = 0;
  mt0_rr_chk(d, MT_MAC_SYS_CTRL, &sys);
  std::printf("  channel %d, MAC_SYS_CTRL 0x%08x after start\n", ch, sys);
  rx_stats(d, 0, 0); /* clear the counters */
  unsigned last_f = 0, last_c = 0;
  for (int s = 0; s < secs; ++s) {
    mt0_usleep(1000000);
    const unsigned f = c.frames, cr = c.crc;
    rx_stats(d, f - last_f, cr - last_c);
    last_f = f;
    last_c = cr;
  }
  mt76x0u_rx_quiesce(d);
  mt76x0u_rx_stop(d);
  return c.frames ? 0 : 1;
}

int ping(mt76x0u_dev *d) {
  d->mcu_running = 1;
  d->q_selected = 0;
  const int rc = mt0_mcu_function_select(d, Q_SELECT, 1);
  std::printf("  ping (Q_SELECT): %s\n", rc ? "NO RESPONSE" : "OK");
  return rc;
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s STEP [STEP...] (see source header)\n", argv[0]);
    return 2;
  }
  mt76x0u_set_log_sink(sink, nullptr);

  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx)) {
    std::fprintf(stderr, "libusb_init failed\n");
    return 1;
  }
  libusb_device_handle *h = libusb_open_device_with_vid_pid(ctx, MT76X0U_VID, MT76X0U_PID);
  if (!h) {
    std::fprintf(stderr, "0e8d:7610 not found or not openable\n");
    return 1;
  }
  libusb_set_auto_detach_kernel_driver(h, 1);
  if (int r = libusb_claim_interface(h, 0)) {
    std::fprintf(stderr, "claim interface 0: %s\n", libusb_error_name(r));
    return 1;
  }
  const char *err = nullptr;
  mt76x0u_dev *d = mt76x0u_open_handle(h, ctx, 0, &err);
  if (!d) {
    std::fprintf(stderr, "open: %s\n", err ? err : "?");
    return 1;
  }
  std::printf("== open: ASIC 0x%08x MAC 0x%08x\n", mt76x0u_asic_version(d),
              mt76x0u_mac_version(d));
  state(d);

  int rc = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string s = argv[i];
    std::printf("== %s\n", s.c_str());
    mt0_io_clear(d);
    if (s == "state") {
      /* printed below */
    } else if (s == "ping") {
      rc = ping(d);
    } else if (s == "intlevel") {
      mt0_wr(d, MT_MCU_INT_LEVEL, 1);
      mt0_usleep(30000);
    } else if (s == "fwreset") {
      const int r = mt0_vendor_req(d, MT_VEND_DEV_MODE,
                                   static_cast<uint8_t>(LIBUSB_ENDPOINT_OUT) |
                                       static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR) |
                                       static_cast<uint8_t>(LIBUSB_RECIPIENT_DEVICE),
                                   0x1, 0, nullptr, 0);
      std::printf("  DEV_MODE 1: %d\n", r);
      mt0_usleep(10000);
    } else if (s == "pbfreset") {
      pulse_pbf(d, kPbfAllReset);
    } else if (s == "dmareset") {
      pulse_pbf(d, (1u << 1) | (1u << 3)); /* DMA_RESET | PBF_RESET, MCU untouched */
    } else if (s == "mcureset") {
      pulse_pbf(d, kPbfMcuReset);
    } else if (s == "chipoff") {
      mt0_chip_onoff(d, 0, 0);
    } else if (s == "chipon") {
      mt0_chip_onoff(d, 1, 0);
    } else if (s == "chipreset") {
      mt0_chip_onoff(d, 1, 1);
    } else if (s == "clearcom0") {
      mt0_wr(d, MT_MCU_COM_REG0, 0);
    } else if ((s == "upload" || s == "bringup") && i + 1 < argc) {
      const char *dir = argv[++i];
      int up = 0;
      rc = s == "upload" ? mt76x0u_load_firmware(d, dir, 1, &up)
                         : mt76x0u_bring_up(d, dir, 0);
      std::printf("  %s: rc=%d%s\n", s.c_str(), rc,
                  s == "upload" ? (up ? " (uploaded)" : " (not uploaded)") : "");
    } else if (s == "fcefix") {
      /* After a USB reset the FCE writes in-band commands from slot 0 while
       * the MCU's index is wherever the last session left it. Point it at
       * the last slot, so the MCU's next read is slot 0. Experiment. */
      uint32_t m = 0;
      if (!mt0_rr_chk(d, MT_TX_CPU_FROM_FCE_MAX_COUNT, &m) && m > 0)
        mt0_wr(d, MT_TX_CPU_FROM_FCE_CPU_DESC_IDX, m - 1);
    } else if (s == "fceidx" && i + 1 < argc) {
      mt0_wr(d, MT_TX_CPU_FROM_FCE_CPU_DESC_IDX,
             static_cast<uint32_t>(std::atoi(argv[++i])));
    } else if (s == "macstop") {
      rc = mt76x0u_stop(d); /* what Mt76x0uRadio::Stop() runs before close */
      std::printf("  mt76x0u_stop: %d\n", rc);
    } else if (s == "mculog" && i + 1 < argc) {
      d->mculog = std::fopen(argv[++i], "w"); /* closed by mt76x0u_close */
      std::printf("  mculog %s\n", d->mculog ? "on" : "FAILED");
    } else if (s == "drain") {
      /* Read EP 0x84 until it stays empty for 200 ms (or 2000 reads). */
      static uint8_t buf[16384];
      int reads = 0, bytes = 0, got = 0;
      while (reads < 2000 &&
             mt0_bulk(d, 0x84, buf, sizeof buf, &got, 200) == 0 && got > 0) {
        ++reads;
        bytes += got;
      }
      std::printf("  drained %d transfer(s), %d bytes from EP 0x84\n", reads, bytes);
    } else if (s == "rx" && i + 2 < argc) {
      const int ch = std::atoi(argv[++i]);
      const int secs = std::atoi(argv[++i]);
      rc = rx_test(d, ch, secs);
    } else if (s == "usbreset") {
      /* As devourer's open (UsbOpen.cpp): reset, then re-claim. */
      const int r = libusb_reset_device(h);
      std::printf("  libusb_reset_device: %s\n", libusb_error_name(r));
      if (r == LIBUSB_ERROR_NOT_FOUND)
        return 0; /* re-enumerated: the handle is stale; skip the close path */
      std::printf("  re-claim: %s\n", libusb_error_name(libusb_claim_interface(h, 0)));
    } else if (s == "sleep" && i + 1 < argc) {
      mt0_usleep(static_cast<unsigned>(std::atoi(argv[++i])) * 1000u);
    } else {
      std::fprintf(stderr, "unknown step '%s'\n", s.c_str());
      rc = 2;
      break;
    }
    std::printf("  io errors this step: %u\n", mt0_io_errors(d));
    state(d);
  }

  mt76x0u_close(d);
  libusb_release_interface(h, 0);
  libusb_close(h);
  libusb_exit(ctx);
  return rc ? 1 : 0;
}
