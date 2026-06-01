#ifndef EEPROMMANAGER_H
#define EEPROMMANAGER_H

#include "PhyTableLoader.h"
#include "RtlUsbAdapter.h"
#include "logger.h"

#include "HalVerDef.h"
#include "phydm_pre_define.h"
#include "rtl8812a_hal.h"

typedef unsigned short ushort;

class EepromManager {
  RtlUsbAdapter _device;
  Logger_t _logger;

  uint8_t efuse_eeprom_data[EFUSE_MAP_LEN_JAGUAR]; /*92C:256bytes, 88E:512bytes,
                                     we use union set (512bytes)*/
  uint8_t EEPROMVersion;
  uint8_t EEPROMRegulatory;
  bool EEPROMBluetoothCoexist;
  uint8_t eeprom_thermal_meter;
  uint8_t PAType_2G;
  uint8_t PAType_5G;
  uint8_t LNAType_2G;
  uint8_t LNAType_5G;
  uint8_t external_pa_5g;
  uint8_t external_lna_5g;

public:
  EepromManager(RtlUsbAdapter device, Logger_t logger);
  uint8_t GetBoardType();
  void efuse_ShadowRead1Byte(uint16_t Offset, uint8_t *Value);

  /* 8814AU only: read EFUSE and populate rfe_type, PA/LNA types, crystal cap,
   * etc. Must be called AFTER firmware download (pre-fwdl EFUSE access
   * leaves the chip in a state where it never acks RSVD-page bulk OUTs and
   * fwdl times out). Safe to call once fw is running. */
  void LateInitFor8814A();

  /* Build the minimal phydm-context shim PhyTableLoader needs to evaluate
   * card-cut conditionals on the 8814AU BB/AGC/MAC tables. The fields are
   * populated from EFUSE values already read into this manager. */
  JaguarPhyContext GetPhyContext() const;

  /* Read the chip's MAC address from the EFUSE shadow. Returns true if a
   * non-empty (not all-0xFF / not all-0x00) MAC was found. EFUSE offsets per
   * upstream `hal_pg.h`: 0xD7 for 8812AU, 0xD8 for 8814AU, 0x107 for 8821AU.
   * Caller is expected to fall back to a hardcoded MAC if this returns
   * false (the T1 canary diff against `aircrack-ng/88XXau` showed
   * `REG_MACID @ 0x610` was unprogrammed on 8812AU because devourer never
   * wrote it — many Realtek MAC-TX paths refuse to schedule frames if the
   * MAC ID is zero). */
  bool GetMacAddress(uint8_t out[6]) const;

  /* Parse the EFUSE TX-power PG block at offset 0x10 into the per-channel
   * per-path Index{24G,5G}_*_Base and *_Diff tables. Port of upstream
   * `hal_load_pg_txpwr_info` + `hal_load_pg_txpwr_info_path_{2,5}g` from
   * `hal_com_phycfg.c`. Called once during init after the EFUSE shadow is
   * populated. Once loaded, `RadioManagementModule::PHY_GetTxPowerIndexBase`
   * computes the per-rate per-channel power index instead of using the
   * uniform `SetTxPower(N)` shortcut (T1 root cause for the 0xc20..0xc40
   * TX-AGC divergence + the 0xc54 TxPwrTraing divergence at ch6). */
  void LoadTxPowerInfo();

  /* Returns the chip's TX-power index for (path, rate, ntx, bw, channel),
   * mirroring upstream `PHY_GetTxPowerIndexBase` (sans the by-rate /
   * regulatory-limit / tracking-offset overlay — those layers come later).
   * Returns 0 if `LoadTxPowerInfo()` hasn't run yet. */
  uint8_t GetTxPowerIndexBase(uint8_t path, uint8_t rate, uint8_t ntx_idx,
                              uint8_t bandwidth, uint8_t channel) const;

  HAL_VERSION version_id;
  odm_cut_version_e cut_version;
  uint8_t crystal_cap;
  uint16_t TypeGPA;
  uint16_t TypeAPA;
  uint16_t TypeGLNA;
  uint16_t TypeALNA;
  uint8_t numTotalRfPath;
  uint8_t maxSpatialStreams = 2;
  bool ExternalLNA_2G;
  bool ExternalPA_2G;
  HAL_RF_TYPE_E rf_type;
  uint16_t rfe_type;

  /* Per-channel per-path TX-power tables parsed from the EFUSE PG block.
   * Mirrors `HAL_DATA_TYPE::Index{24G,5G}_*_Base` and `*_Diff` arrays from
   * upstream `aircrack-ng/rtl8812au/include/hal_data.h`. Populated by
   * `LoadTxPowerInfo()` post-EFUSE-read; consumed by
   * `RadioManagementModule::PHY_GetTxPowerIndexBase8812` to compute the
   * per-rate per-channel TX power index instead of using the historical
   * uniform `SetTxPower(N)` shortcut.
   *
   * Dimensions match upstream: MAX_RF_PATH = 4, MAX_TX_COUNT = 4,
   * CENTER_CH_2G_NUM = 14, CENTER_CH_5G_ALL_NUM = 65. */
  static constexpr int kMaxRfPath = 4;
  static constexpr int kMaxTxCount = 4;
  static constexpr int kCenterCh2gNum = 14;
  static constexpr int kCenterCh5gAllNum = 65;
  uint8_t Index24G_CCK_Base[kMaxRfPath][kCenterCh2gNum]{};
  uint8_t Index24G_BW40_Base[kMaxRfPath][kCenterCh2gNum]{};
  uint8_t Index5G_BW40_Base[kMaxRfPath][kCenterCh5gAllNum]{};
  int8_t CCK_24G_Diff[kMaxRfPath][kMaxTxCount]{};
  int8_t OFDM_24G_Diff[kMaxRfPath][kMaxTxCount]{};
  int8_t BW20_24G_Diff[kMaxRfPath][kMaxTxCount]{};
  int8_t BW40_24G_Diff[kMaxRfPath][kMaxTxCount]{};
  int8_t OFDM_5G_Diff[kMaxRfPath][kMaxTxCount]{};
  int8_t BW20_5G_Diff[kMaxRfPath][kMaxTxCount]{};
  int8_t BW40_5G_Diff[kMaxRfPath][kMaxTxCount]{};
  int8_t BW80_5G_Diff[kMaxRfPath][kMaxTxCount]{};
  bool TxPowerInfoLoaded = false;

private:
  void read_chip_version_8812a(RtlUsbAdapter device);
  void dump_chip_info(HAL_VERSION ChipVersion);
  void rtw_hal_config_rftype();
  void hal_InitPGData_8812A();
  void EFUSE_ShadowMapUpdate(uint8_t efuseType);
  void Efuse_ReadAllMap(uint8_t efuseType, uint8_t *Efuse);
  void EfusePowerSwitch8812A(bool bWrite, bool pwrState);
  bool IsEfuseTxPowerInfoValid(uint8_t *efuseEepromData);
  void efuse_ReadEFuse(uint8_t efuseType, uint16_t _offset, uint16_t _size_byte,
                       uint8_t *pbuf);
  void Hal_EfuseReadEFuse8812A(uint16_t _offset, uint16_t _size_byte,
                               uint8_t *pbuf);
  void rtw_dump_cur_efuse();
  void Hal_EfuseParseIDCode8812A();
  uint8_t Hal_ReadPROMVersion8812A(RtlUsbAdapter device,
                                   uint8_t *efuse_eeprom_data);
  uint8_t Hal_ReadTxPowerInfo8812A(RtlUsbAdapter device,
                                   uint8_t *efuse_eeprom_data);
  void Hal_EfuseParseBTCoexistInfo8812A();
  void Hal_EfuseParseXtal_8812A();
  void Hal_ReadThermalMeter_8812A();
  void Hal_ReadAmplifierType_8812A();
  void hal_ReadPAType_8812A();
  void Hal_ReadRFEType_8812A();
  void hal_ReadUsbType_8812AU();
};

#endif /* EEPROMMANAGER_H */
