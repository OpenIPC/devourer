/* halbb-G6 shim bridge — the register/OS plane the vendored halbb/halrf C
 * bottoms out in, routed to devourer's RtlAdapter. C ABI, shared by the shim'd
 * vendor C and devourer's C++ (RtlKestrelDevice owns the bridge instance and
 * installs the callbacks). See hal/halbb/g6/shim/ and tools/vendor_halbb_*.sh.
 *
 * The vendor code calls hal_write32(bb->hal_com, addr, val) etc.; hal_com's
 * drv_priv points at a kestrel_halbb_bridge whose fn pointers reach the device.
 */
#ifndef KESTREL_HALBB_BRIDGE_H
#define KESTREL_HALBB_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Which Kestrel chip the glue drives — selects ic_type/chip_id for the
 * vendored cores' own runtime dispatch. */
enum kestrel_chip {
  KESTREL_CHIP_8852B = 0,
  KESTREL_CHIP_8852C = 1,
};

/* Opaque device cookie (a devourer RtlKestrelDevice* on the C++ side). */
struct kestrel_halbb_bridge {
  void *dev; /* passed back to every callback */
  unsigned int (*read32)(void *dev, unsigned int addr);
  void (*write32)(void *dev, unsigned int addr, unsigned int val);
  /* MAC power-register plane (rtw_hal_mac_*_pwr_reg): field/shift write. */
  unsigned int (*read_pwr)(void *dev, unsigned int addr);
  void (*write_pwr)(void *dev, unsigned int addr, unsigned int val);
  /* RF-register plane (rtw_hal_*_rf_reg): 3-wire LSSI, masked field read/write
   * per RF path. May be NULL when only the BB plane is used (halbb). */
  unsigned int (*read_rf)(void *dev, unsigned int path, unsigned int addr,
                          unsigned int mask);
  void (*write_rf)(void *dev, unsigned int path, unsigned int addr,
                   unsigned int mask, unsigned int val);
  void (*delay_us)(void *dev, unsigned int us);
  /* Logical-efuse field read: devourer parses `id` (enum rtw_efuse_info) out of
   * its efuse shadow via the vendored 8852c map, writes `size` bytes to value.
   * Returns nonzero on success. NULL on halbb-only / no-efuse builds. */
  int (*efuse_get_info)(void *dev, unsigned int id, void *value,
                        unsigned int size);
  /* diagnostic sink (BB_DBG/BB_WARNING) — may be NULL for no-op. */
  void (*logline)(void *dev, const char *msg);
  /* XTAL-SI plane (rtw_hal_mac_get/set_xsi): the 8852B's a-die SI reset
   * (halrf_arfc_si_reset_8852b, on the halrf_dm_init RFK prologue) needs it.
   * NULL-able: the shim falls back to a no-op (fine for the 8852C paths). */
  unsigned char (*read_xsi)(void *dev, unsigned char offset);
  void (*write_xsi)(void *dev, unsigned char offset, unsigned char val);
  /* fwcmd H2C plane (rtw_hal_mac_send_h2c): `data` = content dwords,
   * `len_bytes` = content length. The C++ side forwards ONLY the OUTSRC
   * radio-page classes (8 = radio A, 9 = radio B, func = page#) into its
   * mac_ax H2C encoder — every other halrf H2C stays inert, matching the
   * validated bring-up. Returns 0 on success (and for ignored classes).
   * NULL-able: the shim then reports success without sending. */
  int (*send_h2c)(void *dev, unsigned char h2c_class, unsigned char h2c_func,
                  const unsigned int *data, unsigned short len_bytes);
  /* The live struct bb_info* (set by kestrel_halbb_create). The shim's
   * halrf->halbb cross-plane calls (rtw_hal_bb_backup_info / restore /
   * tx_mode_switch — hal_api_bb.c in the vendor) resolve the bb instance
   * through here, since the halrf ctx owns a different rtw_hal_com_t. */
  void *bb_info;
};

#ifdef __cplusplus
}
#endif
#endif /* KESTREL_HALBB_BRIDGE_H */
