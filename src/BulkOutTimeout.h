#ifndef DEVOURER_BULK_OUT_TIMEOUT_H
#define DEVOURER_BULK_OUT_TIMEOUT_H

#include <cstddef>

namespace devourer {

/* The libusb timeout a bulk-OUT data send of `len` bytes gets under
 * DeviceConfig::Tx::no_cancel_multipkt (applied by
 * UsbTransport::tx_sync_data only).
 *
 * libusb cancels a transfer on timeout, and a transfer longer than one USB
 * packet can be cancelled after the device already accepted some of its
 * packets. With several sender threads queued on one endpoint, the next queued
 * transfer then streams straight in as the rest of the half-received packet:
 * the Realtek TXDMA misparses it (TXDMA_STATUS PAYLOAD_UDN), keeps its pages,
 * NAKs every later bulk-OUT, and TX is dead until re-init (observed on an
 * 8822EU with 4 sender threads under a carrier-sense-free jam: `rc=-7 got
 * 1536/4439`, then nothing; finishing the cancelled tail afterwards does not
 * help, the splice has already happened). So a multi-packet transfer gets no
 * timeout: it waits for the chip, the same backpressure tx_async (timeout 0)
 * relies on. The cost: a frame the chip NAKs indefinitely blocks its sender
 * indefinitely, and nothing short of the chip draining ends that wait. A
 * transfer of at most one packet is accepted whole or not at all, so it keeps
 * the caller's timeout.
 *
 * `max_packet` 0 = unknown: assume 64 (the smallest bulk packet), i.e. err
 * towards never cancelling. */
inline int bulk_out_timeout_ms(size_t len, unsigned max_packet, int requested) {
  const size_t mps = max_packet ? max_packet : 64;
  return len > mps ? 0 : requested;
}

} // namespace devourer

#endif /* DEVOURER_BULK_OUT_TIMEOUT_H */
