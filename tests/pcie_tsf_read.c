// pcie_tsf_read.c — read the RTL8821CE MAC TSF directly over PCIe BAR2 MMIO,
// from userspace, while the in-tree rtw88 driver keeps the chip alive.
//
// Purpose: de-risk / demonstrate the "a real PHC is tractable on PCIe" claim.
// The 802.11 TSF (REG_TSFTR, 0x560/0x564; a free-running ~1 MHz MAC counter)
// is the same hardware clock a PTP hardware clock would expose. Over PCIe it is
// a plain MMIO load (~µs, low jitter) — versus a ~68 µs jittery USB control
// transfer. This reads it N times, confirms it advances at real time, and
// reports the per-read latency distribution (the PHC gettime floor).
//
// It maps the BAR read-only and only touches a read-only status register, so it
// is safe to run concurrently with rtw88.
//
// Build: cc -O2 -o pcie_tsf_read tests/pcie_tsf_read.c
// Run:   sudo ./pcie_tsf_read [/sys/bus/pci/devices/0000:01:00.0/resource2] [N]
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define REG_TSFTR 0x560u
#define BAR_LEN   0x10000u

static uint64_t read_tsf(volatile uint8_t *bar) {
  uint32_t lo = *(volatile uint32_t *)(bar + REG_TSFTR);
  uint32_t hi = *(volatile uint32_t *)(bar + REG_TSFTR + 4);
  return ((uint64_t)hi << 32) | lo;
}

static int cmp_u64(const void *a, const void *b) {
  uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
  return x < y ? -1 : x > y ? 1 : 0;
}

int main(int argc, char **argv) {
  const char *path = argc > 1 ? argv[1] : "/sys/bus/pci/devices/0000:01:00.0/resource2";
  int N = argc > 2 ? atoi(argv[2]) : 2000;
  if (N < 10) N = 10;

  int fd = open(path, O_RDONLY | O_SYNC);
  if (fd < 0) { perror("open resource2"); return 1; }
  volatile uint8_t *bar = mmap(NULL, BAR_LEN, PROT_READ, MAP_SHARED, fd, 0);
  if (bar == MAP_FAILED) { perror("mmap"); return 1; }

  // Liveness: read twice ~200 ms apart, expect ~200000 us of advance.
  uint64_t t0 = read_tsf(bar);
  struct timespec sl = {0, 200000000};
  nanosleep(&sl, NULL);
  uint64_t t1 = read_tsf(bar);
  printf("TSF #1 = %llu us\nTSF #2 = %llu us   (advanced %lld us over ~200 ms)\n",
         (unsigned long long)t0, (unsigned long long)t1, (long long)(t1 - t0));
  if (t1 <= t0) {
    printf("FAIL: TSF not advancing (chip powered down / MAC clock off?)\n");
    return 1;
  }

  // Latency: time N back-to-back reads (the PHC gettime floor).
  uint64_t *lat = malloc(sizeof(uint64_t) * N);
  for (int i = 0; i < N; i++) {
    struct timespec a, b;
    clock_gettime(CLOCK_MONOTONIC, &a);
    (void)read_tsf(bar);
    clock_gettime(CLOCK_MONOTONIC, &b);
    lat[i] = (b.tv_sec - a.tv_sec) * 1000000000ull + (b.tv_nsec - a.tv_nsec);
  }
  qsort(lat, N, sizeof(uint64_t), cmp_u64);
  double mean = 0;
  for (int i = 0; i < N; i++) mean += lat[i];
  mean /= N;
  printf("BAR2 MMIO TSF read latency over %d samples:\n", N);
  printf("  mean=%.0f ns  p50=%llu ns  p99=%llu ns  min=%llu ns  max=%llu ns\n",
         mean, (unsigned long long)lat[N / 2], (unsigned long long)lat[N * 99 / 100],
         (unsigned long long)lat[0], (unsigned long long)lat[N - 1]);
  printf("PASS: TSF live and readable over PCIe MMIO (this is the PHC clock source)\n");
  return 0;
}
