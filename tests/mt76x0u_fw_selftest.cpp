/* Headless guard for the MT76x0U firmware loader and MCU register-pair
 * encoding (src/mt76x0u/fw.cpp, src/mt76x0u/mcu.cpp) - the parts that are pure
 * byte layout and so can be pinned to the kernel source without a radio:
 *
 *  - mt0_fw_parse: mt76x0u_load_firmware's checks (ILM larger than the IVB,
 *    size == 32 + ilm + dlm, no wrap on a hostile header) and the IVB split
 *    (the ILM's first 64 bytes start the chip, the rest uploads to 0x40);
 *  - mt0_fw_build_chunk: __mt76x02u_mcu_fw_send_data's framing (info word
 *    with the UNROUNDED length, payload, zero tail, bulk length rounded);
 *  - mt0_mcu_pack_rp / mt0_mcu_parse_rp: CMD_RANDOM_* payloads, and a reply
 *    that disagrees with the request is refused rather than half-applied;
 *  - the guards: an MCU random read/write before Q_SELECT, a read of more
 *    than one command's worth, and RF access with the MCU down are refused
 *    without touching USB;
 *  - mt0_fw_wait_running, through the mt0_io_hook seam: reads that time out
 *    while the firmware boots (USB/IP) are waited out, not counted as I/O
 *    errors, and a chip that never comes up still fails at the deadline.
 *
 * Optionally (argv[1] = a directory holding mt7610e.bin / mt7610u.bin, e.g.
 * from linux-firmware) it parses the real images and checks the chunk plan
 * they produce. Without one that part is reported as skipped. */
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "internal.h"

namespace {

int fails;

void expect(const char *what, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "mt76x0u_fw: FAIL %s\n", what);
    fails++;
  }
}

void put32(std::vector<uint8_t> &b, size_t off, uint32_t v) {
  for (int i = 0; i < 4; i++)
    b[off + i] = (uint8_t)(v >> (8 * i));
}

uint32_t get32(const uint8_t *p) {
  return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
         (uint32_t)p[3] << 24;
}

std::vector<uint8_t> image(uint32_t ilm, uint32_t dlm, size_t extra = 0) {
  std::vector<uint8_t> b(32 + ilm + dlm + extra);
  put32(b, 0, ilm);
  put32(b, 4, dlm);
  b[8] = 0x40; b[9] = 0x76;   /* build_ver 0x7640 */
  b[10] = 0x00; b[11] = 0x01; /* fw_ver 0.1.00 */
  std::memcpy(&b[16], "2014040110180000", 16);
  for (size_t i = 32; i < b.size(); i++)
    b[i] = (uint8_t)i;
  return b;
}

std::vector<uint8_t> read_file(const std::string &path) {
  std::vector<uint8_t> out;
  FILE *f = std::fopen(path.c_str(), "rb");
  if (!f) return out;
  int c;
  while ((c = std::fgetc(f)) != EOF)
    out.push_back((uint8_t)c);
  std::fclose(f);
  return out;
}

/* The chunk plan fw_send_data() walks: (destination, length) per chunk. */
std::vector<std::pair<uint32_t, uint32_t>> plan(uint32_t len, uint32_t dst) {
  std::vector<std::pair<uint32_t, uint32_t>> out;
  const uint32_t max = MT0_FW_URB_MAX_PAYLOAD - 8;
  for (uint32_t pos = 0; pos < len;) {
    const uint32_t n = len - pos < max ? len - pos : max;
    out.emplace_back(dst + pos, n);
    pos += n;
  }
  return out;
}

void real_image(const std::string &dir, const char *name) {
  const std::vector<uint8_t> b = read_file(dir + "/" + name);
  if (b.empty()) {
    std::printf("mt76x0u_fw: %s/%s not present, skipped\n", dir.c_str(), name);
    return;
  }
  struct mt0_fw_image img;
  const char *err = nullptr;
  const bool ok = mt0_fw_parse(b.data(), b.size(), &img, &err) == 0;
  expect((std::string(name) + " parses").c_str(), ok);
  if (!ok) return;
  expect((std::string(name) + " IVB is the ILM's first 64 bytes").c_str(),
         img.ivb == b.data() + 32 && img.ilm == b.data() + 32 + 64);
  const auto ilm = plan(img.ilm_len, MT_MCU_IVB_SIZE);
  const auto dlm = plan(img.dlm_len, MT_MCU_DLM_OFFSET);
  uint64_t sum = 0;
  for (const auto &c : ilm) sum += c.second;
  for (const auto &c : dlm) sum += c.second;
  expect((std::string(name) + " chunks cover ILM-IVB + DLM exactly").c_str(),
         sum == (uint64_t)img.ilm_len + img.dlm_len &&
             sum + 64 + 32 == b.size());
  expect((std::string(name) + " ILM starts at 0x40, DLM at 0x80000").c_str(),
         ilm.front().first == 0x40 && dlm.front().first == 0x80000);
  std::printf("mt76x0u_fw: %s: fw %u.%u.%02u build %x %.16s; ILM %u B in %zu "
              "chunks from 0x40, DLM %u B in %zu chunks from 0x80000\n",
              name, (img.fw_ver >> 12) & 0xf, (img.fw_ver >> 8) & 0xf,
              img.fw_ver & 0xf, img.build_ver, img.build_time, img.ilm_len,
              ilm.size(), img.dlm_len, dlm.size());
}

/* A chip behind the mt0_io_hook seam for the firmware-start wait: identifies
 * as an MT7610U, and MT_MCU_COM_REG0 reads fail (LIBUSB_ERROR_TIMEOUT) for
 * the first `timeouts` EP0 attempts, then read 0 for `zeros` reads, then 1. */
struct FakeBoot {
  int timeouts = 0;
  int zeros = 0;
  int reads = 0;
  int unstick_writes = 0;
  int failed_reads = 0;
};

int fake_boot_ctrl(void *user, uint8_t type, uint8_t, uint16_t val,
                   uint16_t idx, uint8_t *data, uint16_t len) {
  auto *f = static_cast<FakeBoot *>(user);
  if (!(type & 0x80)) { /* writes succeed */
    if (((uint32_t)val << 16 | idx) == MT_FCE_PSE_CTRL)
      f->unstick_writes++;
    return 0;
  }
  const uint32_t addr = (uint32_t)val << 16 | idx;
  uint32_t v = 0;
  if (addr == MT_ASIC_VERSION) {
    v = 0x76100002;
  } else if (addr == MT_MCU_COM_REG0) {
    if (f->timeouts > 0) {
      f->timeouts--;
      f->failed_reads++;
      return LIBUSB_ERROR_TIMEOUT;
    }

    f->reads++;
    v = f->zeros-- > 0 ? 0 : 1;
  }
  for (int i = 0; i < 4 && i < len; i++)
    data[i] = (uint8_t)(v >> (8 * i));
  return len;
}

void fw_wait_case(const char *what, int timeouts, int zeros, int timeout_us,
                  int want) {
  FakeBoot f;
  f.timeouts = timeouts;
  f.zeros = zeros;
  const mt0_io_hook hook = {fake_boot_ctrl, nullptr, &f};
  const char *err = nullptr;
  struct mt76x0u_dev *d = mt0_open_hooked(&hook, 0, &err);
  expect((std::string(what) + ": hooked open").c_str(), d != nullptr);
  if (!d) return;
  mt0_io_restore(d, 3); /* failures counted earlier in the caller's sequence */
  const int rc = mt0_fw_wait_running(d, timeout_us);
  expect((std::string(what) + ": result").c_str(), rc == want);
  expect((std::string(what) + ": earlier I/O errors kept, its own dropped").c_str(),
         mt0_io_errors(d) == 3);
  expect((std::string(what) + ": EP0 logging and tries back to default").c_str(),
         d->ep0_quiet == 0 && d->ep0_tries == 0);
  expect((std::string(what) + ": one MT_FCE_PSE_CTRL write per read with no answer").c_str(),
         f.unstick_writes == f.failed_reads);
  mt0_close(d);
  delete d;
}

} // namespace

int main(int argc, char **argv) {
  struct mt0_fw_image img;
  const char *err = nullptr;

  /* --- parse ---------------------------------------------------------- */
  {
    auto b = image(100, 20);
    expect("valid image parses", mt0_fw_parse(b.data(), b.size(), &img, &err) == 0);
    expect("IVB at payload start", img.ivb == b.data() + 32);
    expect("ILM after the IVB, 100 - 64 bytes",
           img.ilm == b.data() + 32 + 64 && img.ilm_len == 36);
    expect("DLM after the whole ILM", img.dlm == b.data() + 32 + 100 &&
                                          img.dlm_len == 20);
    expect("versions", img.build_ver == 0x7640 && img.fw_ver == 0x0100);
    expect("build time", std::strcmp(img.build_time, "2014040110180000") == 0);
  }
  {
    auto b = image(100, 20);
    expect("short header refused", mt0_fw_parse(b.data(), 31, &img, &err) != 0);
    expect("NULL refused", mt0_fw_parse(nullptr, 0, &img, &err) != 0);
  }
  {
    auto b = image(64, 20);
    expect("ILM == IVB refused", mt0_fw_parse(b.data(), b.size(), &img, &err) != 0);
  }
  {
    auto b = image(100, 20, 1);
    expect("one trailing byte refused",
           mt0_fw_parse(b.data(), b.size(), &img, &err) != 0);
    expect("one byte short refused",
           mt0_fw_parse(b.data(), b.size() - 2, &img, &err) != 0);
  }
  {
    /* ilm + dlm wraps to 120 in 32 bits: must still be refused. */
    auto b = image(100, 20);
    put32(b, 0, 0xffffffffu);
    put32(b, 4, 121);
    expect("wrapping header refused",
           mt0_fw_parse(b.data(), b.size(), &img, &err) != 0);
  }

  /* --- chunk framing ---------------------------------------------------- */
  {
    std::vector<uint8_t> scratch(MT0_FW_URB_MAX_PAYLOAD, 0xee);
    std::vector<uint8_t> src(MT0_FW_URB_MAX_PAYLOAD, 0x5a);
    const int n = mt0_fw_build_chunk(scratch.data(), src.data(), 5);
    const uint32_t info = get32(scratch.data());
    expect("5-byte chunk: bulk length 4 + 8 + 4", n == 16);
    expect("info LEN is the unrounded 5", FIELD_GET(MT_MCU_MSG_LEN, info) == 5);
    expect("info PORT is CPU_TX_PORT", FIELD_GET(MT_MCU_MSG_PORT, info) == CPU_TX_PORT);
    expect("info TYPE_CMD", (info & MT_MCU_MSG_TYPE_CMD) != 0);
    expect("payload copied", std::memcmp(&scratch[4], src.data(), 5) == 0);
    bool zero = true;
    for (int i = 4 + 5; i < n; i++)
      zero = zero && scratch[i] == 0;
    expect("rounding gap and trailer are zero, not stale scratch", zero);

    const int max = MT0_FW_URB_MAX_PAYLOAD - 8;
    const int m = mt0_fw_build_chunk(scratch.data(), src.data(), max);
    expect("a full chunk fits the kernel's max_payload buffer",
           m == MT0_FW_URB_MAX_PAYLOAD);
  }

  /* --- register pairs --------------------------------------------------- */
  {
    struct mt0_reg_pair p[2] = {{MT_RF(0, 3), 0x73}, {MT_RF(5, 2), 0x0c}};
    uint8_t buf[16];
    expect("pack: 8 bytes per pair",
           mt0_mcu_pack_rp(buf, MT_MCU_MEMMAP_RF, p, 2) == 16);
    expect("pack: address is base + reg",
           get32(buf) == (MT_MCU_MEMMAP_RF | MT_RF(0, 3)) &&
               get32(buf + 8) == (MT_MCU_MEMMAP_RF | MT_RF(5, 2)));
    expect("pack: value follows", get32(buf + 4) == 0x73 && get32(buf + 12) == 0x0c);

    /* A reply echoes the addresses with the values read. */
    uint8_t reply[16];
    std::memcpy(reply, buf, sizeof reply);
    reply[4] = 0x11;
    reply[12] = 0x22;
    struct mt0_reg_pair q[2] = {{MT_RF(0, 3), 0}, {MT_RF(5, 2), 0}};
    expect("parse: matching reply accepted",
           mt0_mcu_parse_rp(reply, 16, MT_MCU_MEMMAP_RF, q, 2) == 0 &&
               q[0].value == 0x11 && q[1].value == 0x22);

    struct mt0_reg_pair r[2] = {{MT_RF(0, 3), 0xaa}, {MT_RF(5, 2), 0xbb}};
    expect("parse: short reply refused",
           mt0_mcu_parse_rp(reply, 8, MT_MCU_MEMMAP_RF, r, 2) != 0);
    reply[8] ^= 1; /* second echoed address now wrong */
    expect("parse: wrong echoed address refused",
           mt0_mcu_parse_rp(reply, 16, MT_MCU_MEMMAP_RF, r, 2) != 0);
    expect("parse: a refused reply changes nothing",
           r[0].value == 0xaa && r[1].value == 0xbb);
    expect("one command carries 24 pairs", MT0_RP_MAX_PER_CMD == 24);
  }

  /* --- guards (no handle: any USB access here would crash the cell) ----- */
  {
    struct mt76x0u_dev d{};
    struct mt0_reg_pair p = {MT_RF(0, 3), 0};
    d.mcu_running = 1;
    d.q_selected = 0;
    expect("random write before Q_SELECT refused",
           mt0_mcu_wr_rp(&d, MT_MCU_MEMMAP_RF, &p, 1) != 0);
    expect("random read before Q_SELECT refused",
           mt0_mcu_rd_rp(&d, MT_MCU_MEMMAP_RF, &p, 1) != 0);
    std::vector<mt0_reg_pair> many(MT0_RP_MAX_PER_CMD + 1, p);
    d.q_selected = 1;
    expect("random read of 25 pairs refused",
           mt0_mcu_rd_rp(&d, MT_MCU_MEMMAP_RF, many.data(), (int)many.size()) != 0);
    d.mcu_running = 0;
    expect("RF write with the MCU down refused", mt0_rf_wr(&d, MT_RF(0, 3), 1) != 0);
    expect("RF read with the MCU down refused", mt0_rf_rr(&d, MT_RF(0, 3)) == -1);
    expect("zero pairs is a no-op", mt0_mcu_wr_rp(&d, 0, nullptr, 0) == 0 &&
                                        mt0_mcu_rd_rp(&d, 0, nullptr, 0) == 0);
  }

  /* --- firmware-start wait (USB/IP: no EP0 answer while booting) ------- */
  /* Each read is one attempt; after one with no answer the wait writes
   * MT_FCE_PSE_CTRL (unsticks EP0 over usbipd) and reads again. */
  fw_wait_case("boot: two reads time out, then running", 2, 0, 2000000, 0);
  fw_wait_case("boot: bit 0 clear for 5 reads, then running", 0, 5, 2000000, 0);
  fw_wait_case("boot: running at once", 0, 0, 2000000, 0);
  fw_wait_case("boot: never answers -> fails at the deadline", 1 << 30, 0, 100000, -1);
  fw_wait_case("boot: one dropped read (native and USB/IP), then running", 1, 0, 2000000, 0);
  fw_wait_case("boot: answers 0 forever -> fails at the deadline", 0, 1 << 30, 20000, -1);
  {
    /* ep0_tries: 1 means one attempt and no retry (the wait relies on it);
     * 0 keeps mt76's 10 attempts for every other access. */
    FakeBoot f;
    f.timeouts = 100;
    const mt0_io_hook hook = {fake_boot_ctrl, nullptr, &f};
    const char *err = nullptr;
    struct mt76x0u_dev *d = mt0_open_hooked(&hook, 0, &err);
    expect("ep0_tries: hooked open", d != nullptr);
    if (d) {
      uint32_t v;
      d->ep0_quiet = 1;
      d->ep0_tries = 1;
      expect("ep0_tries 1: the read fails", mt0_rr_chk(d, MT_MCU_COM_REG0, &v) != 0);
      expect("ep0_tries 1: exactly one attempt", f.timeouts == 99);
      d->ep0_tries = 0;
      mt0_rr_chk(d, MT_MCU_COM_REG0, &v);
      expect("ep0_tries 0: the default 10 attempts", f.timeouts == 89);
      mt0_close(d);
      delete d;
    }
  }

  /* --- real images (optional) ------------------------------------------- */
  if (argc > 1 && argv[1][0]) {
    real_image(argv[1], "mt7610e.bin");
    real_image(argv[1], "mt7610u.bin");
  } else {
    std::printf("mt76x0u_fw: no firmware directory given, real-image checks "
                "skipped (set DEVOURER_MT76X0U_TEST_FW_DIR)\n");
  }

  if (fails) {
    std::fprintf(stderr, "mt76x0u_fw: %d failure(s)\n", fails);
    return 1;
  }
  std::printf("mt76x0u_fw: ok\n");
  return 0;
}
