/* Minimal libusb probe: read Realtek SYS_CFG1 (0x00F0) + SYS_CFG2 (0x00FC) off a
 * given VID:PID and print them, without any chip bring-up. Used to verify the
 * real RTL8821C SYS_CFG2 chip-id byte the devourer factory must branch on
 * (the src/WiFiDriver.cpp comment claiming 0x09 is an unverified guess).
 *
 * Build: cc -o chipid_probe chipid_probe.c $(pkg-config --libs libusb-1.0)
 * Run:   ./chipid_probe 0x0bda 0xc811
 */
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>

#define VENQT_READ 0xC0 /* REALTEK_USB_VENQT_READ */
#define TIMEOUT 500

static int rd(libusb_device_handle *h, uint16_t off, uint8_t *buf, uint16_t n) {
  return libusb_control_transfer(h, VENQT_READ, 5, off, 0, buf, n, TIMEOUT);
}

int main(int argc, char **argv) {
  uint16_t vid = 0x0bda, pid = 0xc811;
  if (argc >= 3) {
    vid = (uint16_t)strtol(argv[1], NULL, 0);
    pid = (uint16_t)strtol(argv[2], NULL, 0);
  }
  if (libusb_init(NULL) < 0) { fprintf(stderr, "libusb_init failed\n"); return 1; }

  libusb_device_handle *h = libusb_open_device_with_vid_pid(NULL, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x failed (plugged in? driver bound?)\n", vid, pid); libusb_exit(NULL); return 2; }

  libusb_set_auto_detach_kernel_driver(h, 1);
  int r = libusb_claim_interface(h, 0);
  if (r < 0) { fprintf(stderr, "claim_interface failed: %s\n", libusb_error_name(r)); libusb_close(h); libusb_exit(NULL); return 3; }

  uint8_t cfg2 = 0;            /* 0x00FC low byte = chip-id */
  uint8_t cfg1[4] = {0,0,0,0}; /* 0x00F0 dword = cut/vendor/rf_type bits */
  int r2 = rd(h, 0x00FC, &cfg2, 1);
  int r1 = rd(h, 0x00F0, cfg1, 4);

  if (r2 < 0) fprintf(stderr, "read 0x00FC failed: %s\n", libusb_error_name(r2));
  if (r1 < 0) fprintf(stderr, "read 0x00F0 failed: %s\n", libusb_error_name(r1));

  uint32_t v1 = (uint32_t)cfg1[0] | ((uint32_t)cfg1[1] << 8) |
                ((uint32_t)cfg1[2] << 16) | ((uint32_t)cfg1[3] << 24);
  printf("VID:PID          = %04x:%04x\n", vid, pid);
  printf("SYS_CFG2 (0x00FC) chip-id = 0x%02x\n", cfg2);
  printf("SYS_CFG1 (0x00F0)         = 0x%08x\n", v1);
  /* Decode per vendor rtl8821c_ops.c read_chip_version */
  printf("  cut  (CHIP_VER [15:12]) = %u\n", (v1 >> 12) & 0xF);
  printf("  vend (VENDOR   [19:16]) = %u (0=TSMC,1=SMIC,2=UMC)\n", (v1 >> 16) & 0xF);
  printf("  RF_TYPE (bit27)         = %u (0=1T1R,1=2T2R)\n", (v1 >> 27) & 0x1);
  printf("  RTL_ID  (bit23,testchip)= %u\n", (v1 >> 23) & 0x1);

  libusb_release_interface(h, 0);
  libusb_close(h);
  libusb_exit(NULL);
  return 0;
}
