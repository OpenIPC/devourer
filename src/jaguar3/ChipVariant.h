#ifndef JAGUAR3_CHIP_VARIANT_H
#define JAGUAR3_CHIP_VARIANT_H

namespace jaguar3 {

/* Which Jaguar3 MAC/DLFW fork a device is. Both variants share the rtl8822c PHY
 * generation (BB/AGC/RF tables, IQK/DACK, channel/bandwidth, 5/10 MHz
 * narrowband); they differ only in the firmware blob — and, if the bench ever
 * proves it, the power-on sequence / MAC-init (currently verified identical).
 *
 *   C8822C — rtl8822c: RTL8812CU / RTL8822CU  (SYS_CFG2 chip-id 0x13)
 *   C8822E — rtl8822e: RTL8812EU / RTL8822EU  (SYS_CFG2 chip-id 0x17)
 *
 * Resolved at construction in WiFiDriver::CreateRtlDevice from the SYS_CFG2
 * (0x00FC) chip-id byte and threaded down into HalmacJaguar3Fw for blob select. */
enum class ChipVariant { C8822C, C8822E };

} /* namespace jaguar3 */

#endif /* JAGUAR3_CHIP_VARIANT_H */
