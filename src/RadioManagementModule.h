#ifndef RADIOMANAGEMENTMODULE_H
#define RADIOMANAGEMENTMODULE_H

#include <cstdint>

#include "EepromManager.h"
#include "RfPath.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"
#include "logger.h"

#define HAL_PRIME_CHNL_OFFSET_DONT_CARE 0
#define HAL_PRIME_CHNL_OFFSET_LOWER 1
#define HAL_PRIME_CHNL_OFFSET_UPPER 2

enum class HwPort { HW_PORT0, HW_PORT1 };

enum class BandType {
  BAND_ON_2_4G = 0,
  BAND_ON_5G = 1,
  BAND_ON_BOTH = 2,
  BAND_MAX = 3,
};

class RadioManagementModule {
  RtlUsbAdapter _device;
  std::shared_ptr<EepromManager> _eepromManager;
  Logger_t _logger;
  HwPort _hwPort = HwPort::HW_PORT0;
  bool _needIQK = false;
  ChannelWidth_t _currentChannelBw;
  uint8_t _currentChannel;
  BandType current_band_type;
  bool _swChannel = false;
  bool _channelBwInitialized = false;
  bool _setChannelBw = false;
  uint8_t _cur40MhzPrimeSc;
  uint8_t _cur80MhzPrimeSc;
  uint8_t _currentCenterFrequencyIndex;

public:
  RadioManagementModule(RtlUsbAdapter device,
                        std::shared_ptr<EepromManager> eepromManager,
                        Logger_t logger);
  void hw_var_rcr_config(uint32_t rcr);
  void SetMonitorMode();
  void set_channel_bwmode(uint8_t channel, uint8_t channel_offset,
                          ChannelWidth_t bwmode);
  void phy_set_rf_reg(RfPath eRFPath, uint16_t RegAddr, uint32_t BitMask,
                      uint32_t Data);
  void init_hw_mlme_ext(SelectedChannel pmlmeext);
  void rtw_hal_set_chnl_bw(uint8_t channel, ChannelWidth_t Bandwidth,
                           uint8_t Offset40, uint8_t Offset80);
  void PHY_SwitchWirelessBand8812(BandType Band);

private:
  void rtw_hal_set_msr(uint8_t net_type);
  void hw_var_set_monitor();
  void PHY_SetSwChnlBWMode8812(uint8_t channel, ChannelWidth_t Bandwidth,
                               uint8_t Offset40, uint8_t Offset80);
  void PHY_HandleSwChnlAndSetBW8812(bool bSwitchChannel, bool bSetBandWidth,
                                    uint8_t ChannelNum,
                                    ChannelWidth_t ChnlWidth,
                                    uint8_t ChnlOffsetOf40MHz,
                                    uint8_t ChnlOffsetOf80MHz,
                                    uint8_t CenterFrequencyIndex1);
  void phy_SwChnlAndSetBwMode8812();
  uint32_t phy_RFSerialRead(RfPath eRFPath, uint32_t Offset);
  void phy_RFSerialWrite(RfPath eRFPath, uint32_t Offset, uint32_t Data);
  void phy_SetRFEReg8812(BandType Band);
  void phy_SetBBSwingByBand_8812A(BandType Band);
  uint32_t phy_get_tx_bb_swing_8812a(BandType Band, RfPath RFPath);
  void Set_HW_VAR_ENABLE_RX_BAR(bool val);
};

#endif /* RADIOMANAGEMENTMODULE_H */
