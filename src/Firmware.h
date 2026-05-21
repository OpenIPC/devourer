#ifndef FIRMWARE_H
#define FIRMWARE_H

#include <cstddef>
#include <cstdint>

#include "HalVerDef.h"

#define CONFIG_RTL8812A
#define CONFIG_RTL8814A
#define LOAD_FW_HEADER_FROM_DRIVER
#define ODM_WIN 1
#define DM_ODM_SUPPORT_TYPE ODM_WIN
typedef uint8_t u8;
typedef uint32_t u32;
extern "C" {
#include "hal8812a_fw.h"
#include "hal8814a_fw.h"
}

struct FirmwareBlob {
  const u8 *buf;
  std::size_t len;
};

/* Pick the right firmware NIC blob for the given chip. CHIP_8814A has its own
 * compiled-in array; everything else in the Jaguar family rides on the 8812
 * blob. */
inline FirmwareBlob PickFirmwareForChip(HAL_IC_TYPE_E ic_type) {
  if (ic_type == CHIP_8814A) {
    return {array_mp_8814a_fw_nic, sizeof(array_mp_8814a_fw_nic)};
  }
  return {array_mp_8812a_fw_nic, sizeof(array_mp_8812a_fw_nic)};
}

#endif /* FIRMWARE_H */
