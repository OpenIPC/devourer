/* Flood MCS<idx>/20MHz 802.11 data frames out a monitor-mode netdev via a raw
 * AF_PACKET socket, as fast as the kernel driver accepts them. The C twin of
 * kernel_tx_inject.py — same frame bytes — so the CPU-per-Mbit harness has a
 * feeder whose own overhead is negligible (matching devourer's compiled txdemo
 * feeder) and that runs on Python-less targets (OpenIPC IP cameras).
 *
 *   cc -O2 -o raw_inject tests/raw_inject.c
 *   sudo ./raw_inject <monitor_iface> [mcs] [payload_bytes] [secs]
 *
 * Prints "injected N frames in X.Xs (Y fps) ..." — the exact line the harness
 * (tests/cpu_per_mbit.py) greps for.
 */
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static double now_s(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec / 1e9;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <iface> [mcs] [payload] [secs]\n", argv[0]);
    return 2;
  }
  const char *iface = argv[1];
  int mcs = argc > 2 ? atoi(argv[2]) : 7;
  int payload = argc > 3 ? atoi(argv[3]) : 1500;
  double secs = argc > 4 ? atof(argv[4]) : 10.0;

  /* radiotap: present bit 19 (MCS), known = bw+mcs (0x03), flags bw20 = 0. */
  unsigned char rt[] = {0, 0, 11, 0, 0x00, 0x00, 0x08, 0x00,
                        0x03, 0x00, (unsigned char)(mcs & 0xff)};
  /* 802.11 data: FC=data, dur, A1=bcast, A2/A3 = canonical SA, seq. */
  const unsigned char sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  unsigned char dot11[24];
  size_t o = 0;
  dot11[o++] = 0x08; dot11[o++] = 0x00; dot11[o++] = 0x00; dot11[o++] = 0x00;
  memset(dot11 + o, 0xff, 6); o += 6;          /* A1 broadcast */
  memcpy(dot11 + o, sa, 6); o += 6;            /* A2 */
  memcpy(dot11 + o, sa, 6); o += 6;            /* A3 */
  dot11[o++] = 0x00; dot11[o++] = 0x00;        /* seq */

  size_t flen = sizeof(rt) + o + (size_t)payload;
  unsigned char *frame = calloc(1, flen);
  if (!frame) { perror("calloc"); return 1; }
  memcpy(frame, rt, sizeof(rt));
  memcpy(frame + sizeof(rt), dot11, o);        /* payload stays zeroed */

  int s = socket(AF_PACKET, SOCK_RAW, 0);
  if (s < 0) { perror("socket"); return 1; }
  struct sockaddr_ll sll;
  memset(&sll, 0, sizeof(sll));
  sll.sll_family = AF_PACKET;
  sll.sll_ifindex = if_nametoindex(iface);
  if (!sll.sll_ifindex) { fprintf(stderr, "no iface %s\n", iface); return 1; }
  if (bind(s, (struct sockaddr *)&sll, sizeof(sll)) < 0) { perror("bind"); return 1; }

  unsigned long long n = 0;
  double t0 = now_s();
  while (now_s() - t0 < secs) {
    if (send(s, frame, flen, 0) >= 0)
      n++;
  }
  double dt = now_s() - t0;
  printf("injected %llu frames in %.1fs (%.0f fps) mcs=%d payload=%d\n",
         n, dt, n / dt, mcs, payload);
  free(frame);
  close(s);
  return 0;
}
