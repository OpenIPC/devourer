/* rxq_preempt — an intermittent, real-time-priority CPU preemptor that models a
 * VR compositor (or any high-priority periodic task) starving a background USB
 * pump thread. Pins itself to CORE, raises to SCHED_FIFO, then repeats:
 * busy-hog for ON_US, sleep for OFF_US. During each hog the FIFO priority fully
 * preempts the normal-priority devourer RX pump on that core — reproducing the
 * transient ring starvation of issue #330 on a host that otherwise has CPU to
 * spare. SCHED_FIFO needs privilege (run under sudo); it falls back to a plain
 * busy-hog (weaker, ~50%% share) if setscheduler fails.
 *
 * Build:  cc -O2 -o build/rxq_preempt tests/rxq_preempt.c -lpthread
 * Usage:  rxq_preempt CORE ON_US OFF_US
 */
#define _GNU_SOURCE
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static long now_us(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec * 1000000L + t.tv_nsec / 1000;
}

int main(int argc, char **argv) {
  if (argc < 4) {
    fprintf(stderr, "usage: rxq_preempt CORE ON_US OFF_US\n");
    return 2;
  }
  int core = atoi(argv[1]);
  long on_us = atol(argv[2]);
  long off_us = atol(argv[3]);

  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(core, &set);
  if (sched_setaffinity(0, sizeof(set), &set) != 0)
    perror("sched_setaffinity");

  struct sched_param sp = {.sched_priority = 50};
  if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
    perror("sched_setscheduler(FIFO) — falling back to normal priority");

  volatile unsigned long sink = 0;
  for (;;) {
    long deadline = now_us() + on_us;
    while (now_us() < deadline)
      sink += 1; /* burn the core at RT priority */
    struct timespec ts = {off_us / 1000000, (off_us % 1000000) * 1000};
    nanosleep(&ts, NULL);
  }
  return (int)sink; /* unreachable */
}
