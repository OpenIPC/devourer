// pcie_phc_lat.c — measure the PHC gettime floor of /dev/ptpN via the
// PTP_SYS_OFFSET_EXTENDED ioctl. Each sample brackets the driver's device read
// with a system-clock pre/post timestamp; (post - pre) is the time the PHC
// spends reading its hardware clock — for the RTL8821CE TSF PHC that is a BAR2
// MMIO load. This is the number that makes a PCIe PHC "tractable": ~us and low
// jitter, versus a ~68 us jittery USB control transfer.
//
// Build: cc -O2 -o pcie_phc_lat pcie_phc_lat.c
// Run:   sudo ./pcie_phc_lat /dev/ptpN [n_samples]
#include <fcntl.h>
#include <linux/ptp_clock.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

static uint64_t ns(struct ptp_clock_time t) { return t.sec * 1000000000ull + t.nsec; }
static int cmp(const void *a, const void *b) {
  uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
  return x < y ? -1 : x > y ? 1 : 0;
}

int main(int argc, char **argv) {
  const char *dev = argc > 1 ? argv[1] : "/dev/ptp0";
  int n = argc > 2 ? atoi(argv[2]) : 25;
  if (n < 1) n = 1;
  if (n > PTP_MAX_SAMPLES) n = PTP_MAX_SAMPLES;

  int fd = open(dev, O_RDONLY);
  if (fd < 0) { perror("open ptp"); return 1; }

  struct ptp_sys_offset_extended off = {0};
  off.n_samples = n;
  if (ioctl(fd, PTP_SYS_OFFSET_EXTENDED, &off) < 0) { perror("PTP_SYS_OFFSET_EXTENDED"); return 1; }

  uint64_t *win = malloc(sizeof(uint64_t) * n);
  double mean = 0;
  for (int i = 0; i < n; i++) {
    uint64_t pre = ns(off.ts[i][0]);   // system before device read
    uint64_t post = ns(off.ts[i][2]);  // system after device read
    win[i] = post - pre;
    mean += win[i];
  }
  mean /= n;
  qsort(win, n, sizeof(uint64_t), cmp);
  printf("PHC gettime read window over %d samples (%s):\n", n, dev);
  printf("  mean=%.0f ns  p50=%llu ns  p99=%llu ns  min=%llu ns  max=%llu ns\n",
         mean, (unsigned long long)win[n / 2], (unsigned long long)win[n * 99 / 100],
         (unsigned long long)win[0], (unsigned long long)win[n - 1]);
  // Also show the device (PHC) time from the middle sample, for sanity.
  printf("  phc time (mid sample) = %llu ns\n", (unsigned long long)ns(off.ts[n / 2][1]));
  return 0;
}
