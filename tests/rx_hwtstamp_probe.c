// rx_hwtstamp_probe.c — read RX HARDWARE timestamps (SO_TIMESTAMPING) off a
// Wi-Fi interface driven by the CONFIG_RTW_HWTSTAMP-patched vendor driver.
//
// This is the userspace half of the "honest apples-vs-apples" NTP/PTP-vs-TSF
// comparison: the patched reference/rtl88x2cu driver attaches the MAC RX TSF
// (RxPacket.tsfl, latched in hardware below the CSMA/queueing layer) to each
// skb, and this program reads it back through the *standard kernel API* — the
// same SOF_TIMESTAMPING_RAW_HARDWARE path a real PTP NIC would use. Any gap
// versus devourer's own tsfl fit is then the API path, not the clock.
//
// It binds an AF_PACKET socket to the interface (so it sees every delivered
// frame regardless of L3), enables RX hardware timestamping, and prints, per
// frame: the raw hardware timestamp (ns; a free-running TSF, ~µs resolution)
// and the delta from the previous one. A monotonic ~µs-granular series that
// advances with real airtime is the pass signal; all-zero means the driver
// isn't stamping (unpatched, or the descriptor carried no TSF).
//
// Build: cc -O2 -o build/rx_hwtstamp_probe tests/rx_hwtstamp_probe.c
// Run:   sudo ./build/rx_hwtstamp_probe <iface> [n_frames]
//
// The interface must be UP and receiving frames (associated managed mode with
// traffic, or any mode delivering data frames up the normal path). Monitor mode
// uses a different (radiotap) delivery path and is not what this probes.
#include <arpa/inet.h>
#include <errno.h>
#include <linux/errqueue.h>
#include <linux/if_ether.h>
#include <linux/net_tstamp.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <iface> [n_frames]\n", argv[0]);
    return 2;
  }
  const char *iface = argv[1];
  int want = argc > 2 ? atoi(argv[2]) : 20;

  int fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
  if (fd < 0) { perror("socket(AF_PACKET)"); return 1; }

  unsigned ifindex = if_nametoindex(iface);
  if (!ifindex) { fprintf(stderr, "no such iface %s\n", iface); return 1; }
  struct sockaddr_ll sll = {0};
  sll.sll_family = AF_PACKET;
  sll.sll_protocol = htons(ETH_P_ALL);
  sll.sll_ifindex = ifindex;
  if (bind(fd, (struct sockaddr *)&sll, sizeof sll) < 0) { perror("bind"); return 1; }

  // Ask the kernel to report the hardware RX timestamp (and raw hw clock value)
  // on the socket error/ancillary queue — the same request tcpdump/ptp4l make.
  int flags = SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE |
              SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
  if (setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPING, &flags, sizeof flags) < 0) {
    perror("setsockopt(SO_TIMESTAMPING)");
    return 1;
  }

  printf("probing RX hardware timestamps on %s (want %d frames)\n", iface, want);
  uint8_t buf[2048];
  char ctrl[512];
  int seen = 0, hw_nonzero = 0;
  uint64_t prev_hw = 0;
  while (seen < want) {
    struct iovec iov = {buf, sizeof buf};
    struct msghdr msg = {0};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = ctrl;
    msg.msg_controllen = sizeof ctrl;
    ssize_t n = recvmsg(fd, &msg, 0);
    if (n < 0) { if (errno == EINTR) continue; perror("recvmsg"); break; }

    struct scm_timestamping *ts = NULL;
    for (struct cmsghdr *cm = CMSG_FIRSTHDR(&msg); cm; cm = CMSG_NXTHDR(&msg, cm)) {
      if (cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SO_TIMESTAMPING)
        ts = (struct scm_timestamping *)CMSG_DATA(cm);
    }
    seen++;
    if (!ts) { printf("frame %3d: len=%zd  (no SO_TIMESTAMPING cmsg)\n", seen, n); continue; }

    // ts[0] = software, ts[2] = raw hardware (the MAC TSF, ns). We want ts[2].
    uint64_t hw = (uint64_t)ts->ts[2].tv_sec * 1000000000ull + ts->ts[2].tv_nsec;
    uint64_t sw = (uint64_t)ts->ts[0].tv_sec * 1000000000ull + ts->ts[0].tv_nsec;
    long long d = prev_hw ? (long long)(hw - prev_hw) : 0;
    if (hw) hw_nonzero++;
    printf("frame %3d: len=%4zd  hw_raw=%llu ns  d=%+lld ns  sw=%llu ns\n",
           seen, n, (unsigned long long)hw, d, (unsigned long long)sw);
    if (hw) prev_hw = hw;
  }

  printf("== %d frames, %d with a nonzero raw-hardware timestamp ==\n", seen, hw_nonzero);
  printf("%s\n", hw_nonzero > 1 ? "PASS: driver surfaces the MAC RX TSF via SO_TIMESTAMPING"
                                : "FAIL: no hardware RX timestamps (driver unpatched?)");
  close(fd);
  return hw_nonzero > 1 ? 0 : 1;
}
