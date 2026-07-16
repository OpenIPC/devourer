#ifndef DEVOURER_USB_OPEN_H
#define DEVOURER_USB_OPEN_H

#include <memory>
#include <string>

#include "logger.h"

struct libusb_device_handle;
struct libusb_context;

namespace devourer {

class UsbDeviceLock;

/* Return the vendor Wi-Fi interface that owns bulk IN and OUT endpoints. This
 * must run before claim_interface_then_reset(): composite RTL8822BU adapters
 * expose Bluetooth on interfaces 0/1 and Wi-Fi on interface 2. Fall back to
 * interface 0 for legacy single-interface adapters whose descriptor does not
 * identify a vendor bulk interface. */
int find_wifi_interface(libusb_device_handle *handle);

/* Prepare an already-opened USB handle for use with the OS-friendly ordering —
 * *lock and claim before reset*:
 *
 *   1. take the advisory exclusive UsbDeviceLock (see UsbDeviceLock.h) FIRST,
 *      before touching the device. This is the cross-platform gate: on Windows
 *      (WinUSB) libusb_claim_interface is not a kernel arbitration point and
 *      does not report BUSY, so the lock — not the claim — is what turns a
 *      second devourer away there, and taking it up front means that second
 *      process resets nothing. `out_lock` receives it; the caller must keep it
 *      alive for as long as it drives the adapter (hand it to CreateRtlDevice,
 *      which then does NOT re-acquire);
 *   2. detach an attached kernel driver on `iface`;
 *   3. set configuration 1 (a cold, never-kernel-configured chip sits in config
 *      0, where claim / bulk I/O fail with ESRCH); no-op if already set;
 *   4. libusb_claim_interface(iface) — the kernel's exclusive per-interface lock
 *      (this reports BUSY on Linux usbfs, a redundant second gate);
 *   5. only now, holding both the lock and the interface, and only if `do_reset`,
 *      libusb_reset_device() — safe, since we are the sole owner — then re-claim.
 *
 * Returns 0 on success; a libusb error code otherwise (already logged via
 * `logger`). LIBUSB_ERROR_BUSY means "adapter in use by another process" (from
 * either the lock or the claim) — the caller should close the handle and exit
 * without touching the device. On success `out_lock` holds the exclusive lock
 * (or is null if the lock could only degrade to a warning — e.g. a read-only
 * tmpdir — in which case the kernel claim still guards). `lock_dir` is the
 * lock-file directory (empty = "/tmp"; see UsbDeviceLock::try_acquire). */
int claim_interface_then_reset(libusb_device_handle *handle, int iface,
                               const Logger_t &logger, bool do_reset,
                               std::shared_ptr<UsbDeviceLock> &out_lock,
                               const std::string &lock_dir = {});

/* claim_interface_then_reset plus transparent recovery from the reset
 * re-enumeration: on some chips (Kestrel 35bc:xxxx) libusb_reset_device on a
 * warm adapter drops the firmware back to ROM and the device re-enumerates —
 * the handle goes stale (LIBUSB_ERROR_NOT_FOUND) and, on the way through ROM,
 * the dongle can transiently surface as its ZeroCD id (0bda:1a2b DISK). This
 * wrapper closes the stale handle, waits (bounded) for the SAME physical
 * device — bus + port path — to come back under its original VID:PID, then
 * re-opens and re-claims WITHOUT a second reset: the re-enumeration itself
 * reloaded the firmware, so its state is fresh. `handle` is replaced in place
 * on that path (the old handle is closed); `iface` is re-resolved on the new
 * handle via find_wifi_interface(). All other outcomes are exactly
 * claim_interface_then_reset(). `ctx` must be the context `handle` was opened
 * on. */
int claim_interface_reset_reopen(libusb_context *ctx,
                                 libusb_device_handle *&handle,
                                 const Logger_t &logger, bool do_reset,
                                 std::shared_ptr<UsbDeviceLock> &out_lock,
                                 const std::string &lock_dir = {},
                                 unsigned reopen_timeout_ms = 30000);

} // namespace devourer

#endif /* DEVOURER_USB_OPEN_H */
