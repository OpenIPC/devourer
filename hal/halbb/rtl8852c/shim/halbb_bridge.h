/* halbb-G6 8852C shim bridge — the register/OS plane the vendored halbb C
 * bottoms out in, routed to devourer's RtlAdapter. C ABI, shared by the shim'd
 * vendor C and devourer's C++ (RtlKestrelDevice owns the bridge instance and
 * installs the callbacks). See hal/halbb/rtl8852c/shim/ and tools/vendor_halbb_8852c.sh.
 *
 * The vendor code calls hal_write32(bb->hal_com, addr, val) etc.; hal_com's
 * drv_priv points at a kestrel_halbb_bridge whose fn pointers reach the device.
 */
#ifndef KESTREL_HALBB_BRIDGE_H
#define KESTREL_HALBB_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque device cookie (a devourer RtlKestrelDevice* on the C++ side). */
struct kestrel_halbb_bridge {
  void *dev; /* passed back to every callback */
  unsigned int (*read32)(void *dev, unsigned int addr);
  void (*write32)(void *dev, unsigned int addr, unsigned int val);
  /* MAC power-register plane (rtw_hal_mac_*_pwr_reg): field/shift write. */
  unsigned int (*read_pwr)(void *dev, unsigned int addr);
  void (*write_pwr)(void *dev, unsigned int addr, unsigned int val);
  void (*delay_us)(void *dev, unsigned int us);
  /* diagnostic sink (BB_DBG/BB_WARNING) — may be NULL for no-op. */
  void (*logline)(void *dev, const char *msg);
};

#ifdef __cplusplus
}
#endif
#endif /* KESTREL_HALBB_BRIDGE_H */
