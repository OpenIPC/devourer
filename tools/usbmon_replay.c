/* usbmon_replay — replay a binary URB script verbatim via USBDEVFS.
 *
 * Why this exists: prior bisect work narrowed RTL8814AU's userspace TX
 * gate to "path-A radio table application" by removing kernel-side init
 * blocks. That's an indirect argument. The decisive test is:
 *
 *     Feed the kernel's exact URB sequence into the chip from userspace.
 *     If on-air TX appears, the gate is at the URB layer (and we know
 *     which URBs devourer is missing or getting wrong). If TX still
 *     fails, the gate is below the URB layer (xhci scheduling, DMA
 *     coherency, IRQ-EP polling cadence) — no userspace fix exists
 *     without a kernel shim.
 *
 * Input: a binary URB script produced by tools/pcapng_to_urbscript.py
 * from a tshark capture of usbmonN during a working kernel-driver
 * session.
 *
 * What this tool does NOT do (deliberately):
 *   - It does NOT try to reproduce kernel scheduling jitter at sub-100µs
 *     granularity. Inter-URB gaps are capped at 100ms and floored at 0.
 *   - It does NOT clear halts between URBs. If a stall happens during
 *     replay, that is itself a divergence worth reporting.
 *   - It does NOT inject extra TX traffic at the end. The kernel capture
 *     already contains the first TX URBs that produced on-air emission;
 *     replay them as part of the script.
 *
 * Build: g++ -O2 -Wall -Wextra -o build/usbmon_replay tools/usbmon_replay.c
 *   (or wire into CMake; standalone single-file for portability).
 *
 * Run:
 *   sudo ./build/usbmon_replay \
 *       --device /dev/bus/usb/004/003 \
 *       --urbs /tmp/cap-kernel.urbs \
 *       --interface 0
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <linux/usbdevice_fs.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/* URB script format — keep in sync with tools/pcapng_to_urbscript.py. */
#define URBSCRIPT_MAGIC "URBS"
#define URBSCRIPT_VERSION 1u

struct __attribute__((packed)) urbs_header {
  char magic[4];
  uint32_t version;
  uint32_t urb_count;
};

struct __attribute__((packed)) urbs_record_fixed {
  uint8_t xfer_type;     /* 1=INTR, 2=CTRL, 3=BULK */
  uint8_t ep_with_dir;   /* bit 7 = IN, bits 3..0 = ep number */
  uint16_t reserved;
  uint32_t inter_urb_gap_us;
  uint8_t setup[8];
  /* transfer_length: bytes to send (OUT) or expect (IN) on the wire.
   * data_bytes: actual bytes following this header on disk. For OUT they
   * are equal; for IN data_bytes is always 0 — the chip provides the
   * bytes at replay time and we allocate transfer_length of buffer. */
  uint32_t transfer_length;
  uint32_t data_bytes;
};
_Static_assert(sizeof(struct urbs_record_fixed) == 24, "record size drift");

/* CLI-configurable. */
static int g_iface = 0;
static int g_max_gap_us = 100000; /* cap each inter-URB sleep at 100ms */
static int g_dry_run = 0;
static int g_verbose = 0;
static int g_disconnect = 0; /* USBDEVFS_DISCONNECT before claim — for chips
                                bound by a kernel driver we want kicked out. */

/* Per-xfer-type counters. */
struct stats {
  unsigned submits;
  unsigned ok;
  unsigned err_submit;
  unsigned err_reap;
  unsigned err_status;
  unsigned ctrl, bulk, intr, in_urbs, out_urbs;
};

static void sleep_us(unsigned us) {
  if (us == 0) return;
  if ((int)us > g_max_gap_us) us = g_max_gap_us;
  struct timespec req = {.tv_sec = us / 1000000, .tv_nsec = (us % 1000000) * 1000};
  nanosleep(&req, NULL);
}

static const char *xfer_name(uint8_t t) {
  switch (t) {
    case 1: return "INTR";
    case 2: return "CTRL";
    case 3: return "BULK";
    default: return "????";
  }
}

/* Build a usbdevfs_urb for one record, submit it, reap the completion. */
static int replay_one(int fd, const struct urbs_record_fixed *rec,
                      const uint8_t *data, struct stats *st, unsigned idx) {
  uint8_t type;
  switch (rec->xfer_type) {
    case 1: type = USBDEVFS_URB_TYPE_INTERRUPT; break;
    case 2: type = USBDEVFS_URB_TYPE_CONTROL; break;
    case 3: type = USBDEVFS_URB_TYPE_BULK; break;
    default:
      fprintf(stderr, "[%u] unsupported xfer_type=%u, skipping\n",
              idx, rec->xfer_type);
      return -1;
  }

  /* Buffer layout:
   *   CTRL: setup[8] || payload[transfer_length]
   *         buffer_length = 8 + transfer_length
   *   BULK/INTR: payload[transfer_length]
   *         buffer_length = transfer_length (min 1 for the kernel's sake)
   * For OUT URBs the source bytes are at `data` (data_bytes ==
   * transfer_length). For IN URBs `data` is empty (data_bytes == 0) and
   * the buffer is zero-filled — the chip fills it on completion. */
  size_t buf_len;
  if (type == USBDEVFS_URB_TYPE_CONTROL) {
    buf_len = 8 + rec->transfer_length;
  } else {
    buf_len = rec->transfer_length;
    if (buf_len == 0) buf_len = 1; /* libusb doesn't accept a null buffer */
  }

  uint8_t *buf = (uint8_t *)calloc(1, buf_len);
  if (!buf) {
    fprintf(stderr, "[%u] OOM allocating %zu byte buffer\n", idx, buf_len);
    return -1;
  }
  int is_in = (rec->ep_with_dir & 0x80) != 0;
  if (type == USBDEVFS_URB_TYPE_CONTROL) {
    memcpy(buf, rec->setup, 8);
    if (rec->data_bytes > 0 && data && !is_in) {
      memcpy(buf + 8, data, rec->data_bytes);
    }
  } else if (rec->data_bytes > 0 && data && !is_in) {
    memcpy(buf, data, rec->data_bytes);
  }

  struct usbdevfs_urb urb;
  memset(&urb, 0, sizeof(urb));
  urb.type = type;
  urb.endpoint = rec->ep_with_dir; /* high bit already set for IN */
  urb.buffer = buf;
  urb.buffer_length = (int)buf_len;
  /* Match kernel rtl8814au TX-OUT behaviour where applicable: */
  if (type == USBDEVFS_URB_TYPE_BULK && !(rec->ep_with_dir & 0x80)) {
    urb.flags = USBDEVFS_URB_ZERO_PACKET;
  }
  urb.usercontext = (void *)(uintptr_t)idx;

  st->submits++;
  if (rec->ep_with_dir & 0x80) st->in_urbs++; else st->out_urbs++;
  if (type == USBDEVFS_URB_TYPE_CONTROL) st->ctrl++;
  if (type == USBDEVFS_URB_TYPE_BULK) st->bulk++;
  if (type == USBDEVFS_URB_TYPE_INTERRUPT) st->intr++;

  if (g_dry_run) {
    if (g_verbose) {
      fprintf(stderr, "[%u] dry-run %s ep=0x%02x xfer_len=%u disk=%u\n",
              idx, xfer_name(rec->xfer_type), rec->ep_with_dir,
              rec->transfer_length, rec->data_bytes);
    }
    free(buf);
    st->ok++;
    return 0;
  }

  if (ioctl(fd, USBDEVFS_SUBMITURB, &urb) < 0) {
    int e = errno;
    fprintf(stderr, "[%u] SUBMITURB %s ep=0x%02x failed: %s\n",
            idx, xfer_name(rec->xfer_type), rec->ep_with_dir, strerror(e));
    st->err_submit++;
    free(buf);
    return -1;
  }

  /* Reap. Block until completion. */
  struct usbdevfs_urb *reaped = NULL;
  if (ioctl(fd, USBDEVFS_REAPURB, &reaped) < 0) {
    int e = errno;
    fprintf(stderr, "[%u] REAPURB failed: %s\n", idx, strerror(e));
    st->err_reap++;
    free(buf);
    return -1;
  }
  if (reaped != &urb) {
    fprintf(stderr, "[%u] reaped unexpected URB pointer\n", idx);
    st->err_reap++;
    free(buf);
    return -1;
  }
  if (reaped->status != 0) {
    /* Non-zero status: -EPIPE = stall, -ETIMEDOUT = timed out, -ENOENT =
     * cancelled. We do NOT clear-halt here — a stall during replay is
     * itself a divergence and we want it counted. */
    if (g_verbose) {
      fprintf(stderr, "[%u] %s ep=0x%02x completed with status=%d actual=%d\n",
              idx, xfer_name(rec->xfer_type), rec->ep_with_dir,
              reaped->status, reaped->actual_length);
    }
    st->err_status++;
    free(buf);
    return 0;
  }
  st->ok++;
  if (g_verbose) {
    fprintf(stderr, "[%u] OK %s ep=0x%02x actual=%d\n",
            idx, xfer_name(rec->xfer_type), rec->ep_with_dir,
            reaped->actual_length);
  }
  free(buf);
  return 0;
}

static int parse_header(FILE *f, uint32_t *urb_count_out) {
  struct urbs_header hdr;
  if (fread(&hdr, sizeof(hdr), 1, f) != 1) {
    fprintf(stderr, "failed to read URB script header\n");
    return -1;
  }
  if (memcmp(hdr.magic, URBSCRIPT_MAGIC, 4) != 0) {
    fprintf(stderr, "URB script magic mismatch (got %.4s, want %s)\n",
            hdr.magic, URBSCRIPT_MAGIC);
    return -1;
  }
  if (hdr.version != URBSCRIPT_VERSION) {
    fprintf(stderr, "URB script version %u != supported %u\n",
            hdr.version, URBSCRIPT_VERSION);
    return -1;
  }
  *urb_count_out = hdr.urb_count;
  return 0;
}

static int open_and_claim(const char *device) {
  int fd = open(device, O_RDWR);
  if (fd < 0) {
    fprintf(stderr, "open(%s) failed: %s\n", device, strerror(errno));
    return -1;
  }
  if (g_disconnect) {
    /* DISCONNECT_CLAIM yanks the kernel driver and claims the interface
     * in one ioctl. Equivalent to detach_kernel_driver+claim. */
    struct usbdevfs_disconnect_claim dc;
    memset(&dc, 0, sizeof(dc));
    dc.interface = g_iface;
    dc.flags = USBDEVFS_DISCONNECT_CLAIM_IF_DRIVER;
    strncpy(dc.driver, "usbfs", sizeof(dc.driver) - 1);
    if (ioctl(fd, USBDEVFS_DISCONNECT_CLAIM, &dc) < 0 && errno != ENODATA) {
      int e = errno;
      fprintf(stderr,
              "DISCONNECT_CLAIM iface=%d failed: %s — falling back to plain CLAIM\n",
              g_iface, strerror(e));
      /* Fall through to plain claim. */
    } else {
      return fd;
    }
  }
  unsigned ifnum = g_iface;
  if (ioctl(fd, USBDEVFS_CLAIMINTERFACE, &ifnum) < 0) {
    fprintf(stderr, "CLAIMINTERFACE %u failed: %s (try --disconnect)\n",
            ifnum, strerror(errno));
    close(fd);
    return -1;
  }
  return fd;
}

static void release_and_close(int fd) {
  if (fd < 0) return;
  unsigned ifnum = g_iface;
  if (ioctl(fd, USBDEVFS_RELEASEINTERFACE, &ifnum) < 0 && errno != ENOENT) {
    fprintf(stderr, "RELEASEINTERFACE %u failed: %s\n",
            ifnum, strerror(errno));
  }
  close(fd);
}

static void usage(const char *argv0) {
  fprintf(stderr,
          "usage: %s --device PATH --urbs FILE [--interface N] [--disconnect]\n"
          "          [--max-gap-us US] [--dry-run] [-v]\n"
          "\n"
          "  --device PATH      e.g. /dev/bus/usb/004/003\n"
          "  --urbs FILE        URB script from tools/pcapng_to_urbscript.py\n"
          "  --interface N      USB interface to claim (default 0)\n"
          "  --disconnect       kick the kernel driver off the interface first\n"
          "  --max-gap-us US    cap each inter-URB sleep at US (default 100000)\n"
          "  --dry-run          parse the script but don't talk to the chip\n"
          "  -v                 verbose (one line per URB)\n",
          argv0);
}

int main(int argc, char **argv) {
  const char *device = NULL;
  const char *urbs_path = NULL;
  static const struct option opts[] = {
      {"device", required_argument, 0, 'd'},
      {"urbs", required_argument, 0, 'u'},
      {"interface", required_argument, 0, 'i'},
      {"disconnect", no_argument, 0, 'D'},
      {"max-gap-us", required_argument, 0, 'g'},
      {"dry-run", no_argument, 0, 'n'},
      {"help", no_argument, 0, 'h'},
      {0, 0, 0, 0},
  };
  int c;
  while ((c = getopt_long(argc, argv, "d:u:i:Dg:nvh", opts, NULL)) != -1) {
    switch (c) {
      case 'd': device = optarg; break;
      case 'u': urbs_path = optarg; break;
      case 'i': g_iface = atoi(optarg); break;
      case 'D': g_disconnect = 1; break;
      case 'g': g_max_gap_us = atoi(optarg); break;
      case 'n': g_dry_run = 1; break;
      case 'v': g_verbose = 1; break;
      case 'h': default: usage(argv[0]); return c == 'h' ? 0 : 2;
    }
  }
  if (!device || !urbs_path) {
    usage(argv[0]);
    return 2;
  }

  FILE *f = fopen(urbs_path, "rb");
  if (!f) {
    fprintf(stderr, "open URB script %s: %s\n", urbs_path, strerror(errno));
    return 1;
  }
  uint32_t urb_count = 0;
  if (parse_header(f, &urb_count) < 0) {
    fclose(f);
    return 1;
  }
  fprintf(stderr, "URB script: %u URBs\n", urb_count);

  int fd = -1;
  if (!g_dry_run) {
    fd = open_and_claim(device);
    if (fd < 0) {
      fclose(f);
      return 1;
    }
  }

  struct stats st;
  memset(&st, 0, sizeof(st));

  /* Allocate per-URB. Data sizes are bounded by RF-table replay sizes (a
   * few KB at most for any single transfer). If a single URB carries
   * more than 64KB the chip and the script have other problems. */
  uint8_t *databuf = (uint8_t *)malloc(65536);
  if (!databuf) {
    fprintf(stderr, "OOM allocating record buffer\n");
    if (fd >= 0) release_and_close(fd);
    fclose(f);
    return 1;
  }

  for (uint32_t i = 0; i < urb_count; i++) {
    struct urbs_record_fixed rec;
    if (fread(&rec, sizeof(rec), 1, f) != 1) {
      fprintf(stderr, "[%u] short read in URB script\n", i);
      break;
    }
    if (rec.data_bytes > 65536) {
      fprintf(stderr, "[%u] data_bytes=%u exceeds buffer (skipping)\n",
              i, rec.data_bytes);
      if (fseek(f, rec.data_bytes, SEEK_CUR) != 0) break;
      continue;
    }
    if (rec.data_bytes > 0 && fread(databuf, rec.data_bytes, 1, f) != 1) {
      fprintf(stderr, "[%u] short read on data (%u bytes)\n", i, rec.data_bytes);
      break;
    }
    sleep_us(rec.inter_urb_gap_us);
    replay_one(fd, &rec, databuf, &st, i);
  }

  free(databuf);
  if (fd >= 0) release_and_close(fd);
  fclose(f);

  fprintf(stderr,
          "replay done: %u submits, ok=%u err_submit=%u err_reap=%u err_status=%u\n",
          st.submits, st.ok, st.err_submit, st.err_reap, st.err_status);
  fprintf(stderr,
          "  by kind: ctrl=%u bulk=%u intr=%u | by dir: in=%u out=%u\n",
          st.ctrl, st.bulk, st.intr, st.in_urbs, st.out_urbs);
  return (st.err_submit > 0 || st.err_reap > 0) ? 1 : 0;
}
