#include "RadioManagementModule.h"
#include "Hal8812PhyReg.h"
#include "registry_priv.h"

#include <map>
#include <stdexcept>
#include <thread>

RadioManagementModule::RadioManagementModule(
    RtlUsbAdapter device, std::shared_ptr<EepromManager> eepromManager,
    Logger_t logger)
    : _device{device}, _eepromManager{eepromManager}, _logger{logger} {}

void RadioManagementModule::hw_var_rcr_config(uint32_t rcr) {
  _device.rtw_write32(REG_RCR, rcr);
}

#define _HW_STATE_NOLINK_ 0x00

void RadioManagementModule::SetMonitorMode() {
  rtw_hal_set_msr(_HW_STATE_NOLINK_);
  hw_var_set_monitor();
}

void RadioManagementModule::rtw_hal_set_msr(uint8_t net_type) {
  switch (_hwPort) {
  case HwPort::HW_PORT0: {
    /*REG_CR - BIT[17:16]-Network Type for port 0*/
    auto val8 = _device.rtw_read8(MSR) & 0x0C;
    val8 |= net_type;
    _device.rtw_write8(MSR, val8);
    break;
  }
  case HwPort::HW_PORT1: {
    /*REG_CR - BIT[19:18]-Network Type for port 1*/
    auto val8 = _device.rtw_read8(MSR) & 0x03;
    val8 |= net_type << 2;
    _device.rtw_write8(MSR, val8);
    break;
  }

  default:
    throw std::logic_error("not yet implemented");
    break;
  }
}

void RadioManagementModule::hw_var_set_monitor() {
  /* Receive all type */
  uint32_t rcr_bits = RCR_AAP | RCR_APM | RCR_AM | RCR_AB | RCR_APWRMGT |
                      RCR_ADF | RCR_ACF | RCR_AMF | RCR_APP_PHYST_RXFF;

  /* Append FCS */
  rcr_bits |= RCR_APPFCS;

  // rtw_hal_get_hwreg(adapterState, HW_VAR_RCR, pHalData.rcr_backup);
  hw_var_rcr_config(rcr_bits);

  /* Receive all data frames */
  uint16_t value_rxfltmap2 = 0xFFFF;
  _device.rtw_write16(REG_RXFLTMAP2, value_rxfltmap2);
}

static uint8_t rtw_get_center_ch(uint8_t channel, ChannelWidth_t chnl_bw,
                                 uint8_t chnl_offset) {
  uint8_t center_ch = channel;

  if (chnl_bw == ChannelWidth_t::CHANNEL_WIDTH_80) {
    if (channel == 36 || channel == 40 || channel == 44 || channel == 48)
      center_ch = 42;
    else if (channel == 52 || channel == 56 || channel == 60 || channel == 64)
      center_ch = 58;
    else if (channel == 100 || channel == 104 || channel == 108 ||
             channel == 112)
      center_ch = 106;
    else if (channel == 116 || channel == 120 || channel == 124 ||
             channel == 128)
      center_ch = 122;
    else if (channel == 132 || channel == 136 || channel == 140 ||
             channel == 144)
      center_ch = 138;
    else if (channel == 149 || channel == 153 || channel == 157 ||
             channel == 161)
      center_ch = 155;
    else if (channel == 165 || channel == 169 || channel == 173 ||
             channel == 177)
      center_ch = 171;
    else if (channel <= 14)
      center_ch = 7;
  } else if (chnl_bw == ChannelWidth_t::CHANNEL_WIDTH_40) {
    if (chnl_offset == HAL_PRIME_CHNL_OFFSET_LOWER) {
      center_ch = (uint8_t)(channel + 2);
    } else {
      center_ch = (uint8_t)(channel - 2);
    }
  } else if (chnl_bw == ChannelWidth_t::CHANNEL_WIDTH_20) {
    center_ch = channel;
  } else {
    throw std::logic_error("not yet implemented");
  }

  return center_ch;
}

void RadioManagementModule::set_channel_bwmode(uint8_t channel,
                                               uint8_t channel_offset,
                                               ChannelWidth_t bwmode) {
  uint8_t center_ch, chnl_offset80 = HAL_PRIME_CHNL_OFFSET_DONT_CARE;

  _logger->info("[{}] ch = {}, offset = {}, bwmode = {}", __func__, channel,
                channel_offset, (int)bwmode);

  center_ch = rtw_get_center_ch(channel, bwmode, channel_offset);
  if (bwmode == ChannelWidth_t::CHANNEL_WIDTH_80) {
    if (center_ch > channel) {
      chnl_offset80 = HAL_PRIME_CHNL_OFFSET_LOWER;
    } else if (center_ch < channel) {
      chnl_offset80 = HAL_PRIME_CHNL_OFFSET_UPPER;
    } else {
      chnl_offset80 = HAL_PRIME_CHNL_OFFSET_DONT_CARE;
    }
  }

  rtw_hal_set_chnl_bw(center_ch, bwmode, channel_offset,
                      chnl_offset80); /* set center channel */
}

void RadioManagementModule::rtw_hal_set_chnl_bw(uint8_t channel,
                                                ChannelWidth_t Bandwidth,
                                                uint8_t Offset40,
                                                uint8_t Offset80) {
  PHY_SetSwChnlBWMode8812(channel, Bandwidth, Offset40, Offset80);
}

void RadioManagementModule::PHY_SetSwChnlBWMode8812(uint8_t channel,
                                                    ChannelWidth_t Bandwidth,
                                                    uint8_t Offset40,
                                                    uint8_t Offset80) {
  PHY_HandleSwChnlAndSetBW8812(true, true, channel, Bandwidth, Offset40,
                               Offset80, channel);
}

void RadioManagementModule::PHY_HandleSwChnlAndSetBW8812(
    bool bSwitchChannel, bool bSetBandWidth, uint8_t ChannelNum,
    ChannelWidth_t ChnlWidth, uint8_t ChnlOffsetOf40MHz,
    uint8_t ChnlOffsetOf80MHz, uint8_t CenterFrequencyIndex1) {
  _logger->info(
      "=> PHY_HandleSwChnlAndSetBW8812: bSwitchChannel {}, bSetBandWidth * {}",
      bSwitchChannel, bSetBandWidth);

  /* check is swchnl or setbw */
  if (!bSwitchChannel && !bSetBandWidth) {
    _logger->error("PHY_HandleSwChnlAndSetBW8812:  not switch channel and "
                   "not set bandwidth");
    return;
  }

  /* skip change for channel or bandwidth is the same */
  if (bSwitchChannel) {
    if (_currentChannel != ChannelNum) {
      _swChannel = true;
    }
  }

  if (bSetBandWidth) {
    if (_channelBwInitialized == false) {
      _channelBwInitialized = true;
      _setChannelBw = true;
    } else if ((_currentChannelBw != ChnlWidth) ||
               (_cur40MhzPrimeSc != ChnlOffsetOf40MHz) ||
               (_cur80MhzPrimeSc != ChnlOffsetOf80MHz) ||
               (_currentCenterFrequencyIndex != CenterFrequencyIndex1)) {
      _setChannelBw = true;
    }
  }

  if (!_setChannelBw && !_swChannel && _needIQK != true) {
    _logger->error("<= PHY_HandleSwChnlAndSetBW8812: SwChnl {}, "
                   "_setChannelBw {}",
                   _swChannel, _setChannelBw);
    return;
  }

  if (_swChannel) {
    _currentChannel = ChannelNum;
    _currentCenterFrequencyIndex = ChannelNum;
  }

  if (_setChannelBw) {
    _currentChannelBw = ChnlWidth;
    _cur40MhzPrimeSc = ChnlOffsetOf40MHz;
    _cur80MhzPrimeSc = ChnlOffsetOf80MHz;
    _currentCenterFrequencyIndex = CenterFrequencyIndex1;
  }

  /* Switch workitem or set timer to do switch channel or setbandwidth operation
   */
  phy_SwChnlAndSetBwMode8812();
}

void RadioManagementModule::phy_SwChnlAndSetBwMode8812() {
#if 0
  if (_swChannel) {
    phy_SwChnl8812();
    _swChannel = false;
  }

  if (_setChannelBw) {
    phy_PostSetBwMode8812();
    _setChannelBw = false;
  }

  PHY_SetTxPowerLevel8812(_currentChannel);

  _needIQK = false;
#endif
}

void RadioManagementModule::phy_set_rf_reg(RfPath eRFPath, uint16_t RegAddr,
                                           uint32_t BitMask, uint32_t Data) {
  uint data = Data;
  _logger->debug("RFREG;{};{:X};{:X};{:X}", (uint8_t)eRFPath, (uint)RegAddr,
                 BitMask, data);
  if (BitMask == 0) {
    return;
  }

  /* RF data is 20 bits only */
  if (BitMask != bLSSIWrite_data_Jaguar) {
    uint32_t Original_Value, BitShift;
    Original_Value = phy_RFSerialRead(eRFPath, RegAddr);
    BitShift = PHY_CalculateBitShift(BitMask);
    data = ((Original_Value) & (~BitMask)) | (data << (int)BitShift);
  }

  phy_RFSerialWrite(eRFPath, RegAddr, data);
}

struct BbRegisterDefinition {
  /// LSSI data
  uint32_t Rf3WireOffset;

  /// wire parameter control2
  uint32_t RfHSSIPara2;

  /// LSSI RF readback data SI mode
  uint16_t RfLSSIReadBack;

  /// LSSI RF readback data PI mode 0x8b8-8bc for Path A and B
  uint16_t RfLSSIReadBackPi;
};

std::map<RfPath, BbRegisterDefinition> PhyRegDef = {
    {RfPath::RF_PATH_A,
     {
         .Rf3WireOffset = rA_LSSIWrite_Jaguar,
         .RfHSSIPara2 = rHSSIRead_Jaguar,
         .RfLSSIReadBack = rA_SIRead_Jaguar,
         .RfLSSIReadBackPi = rA_PIRead_Jaguar,
     }},
    {RfPath::RF_PATH_B,
     {
         .Rf3WireOffset = rB_LSSIWrite_Jaguar,
         .RfHSSIPara2 = rHSSIRead_Jaguar,
         .RfLSSIReadBack = rB_SIRead_Jaguar,
         .RfLSSIReadBackPi = rB_PIRead_Jaguar,
     }}};

uint32_t RadioManagementModule::phy_RFSerialRead(RfPath eRFPath,
                                                 uint32_t Offset) {
  uint32_t retValue;

  _logger->error("TODO: phy_RFSerialRead");

#if 0
  BbRegisterDefinition pPhyReg = PhyRegDef[eRFPath];
  bool bIsPIMode = false;

  /* <20120809, Kordan> CCA OFF(when entering), asked by James to avoid reading
   * the wrong value. */
  /* <20120828, Kordan> Toggling CCA would affect RF 0x0, skip it! */
  if (Offset != 0x0 && !(_eepromManager.Version.IS_C_CUT())) {
    _device.phy_set_bb_reg(rCCAonSec_Jaguar, 0x8, 1);
  }

  Offset &= 0xff;

  if (eRFPath == RfPath::RF_PATH_A) {
    bIsPIMode = phy_query_bb_reg(0xC00, 0x4) != 0;
  } else if (eRFPath == RfPath::RF_PATH_B) {
    bIsPIMode = phy_query_bb_reg(0xE00, 0x4) != 0;
  }

  _device.phy_set_bb_reg((ushort)pPhyReg.RfHSSIPara2, bHSSIRead_addr_Jaguar,
                         Offset);

  if (_eepromManager.Version.IS_C_CUT()) {
    Thread.Sleep(20);
  }

  if (bIsPIMode) {
    retValue = phy_query_bb_reg(pPhyReg.RfLSSIReadBackPi, rRead_data_Jaguar);
    /* RTW_INFO("[PI mode] RFR-%d Addr[0x%x]=0x%x\n", eRFPath,
     * pPhyReg.rfLSSIReadBackPi, retValue); */
  } else {
    retValue = phy_query_bb_reg(pPhyReg.RfLSSIReadBack, rRead_data_Jaguar);
    /* RTW_INFO("[SI mode] RFR-%d Addr[0x%x]=0x%x\n", eRFPath,
     * pPhyReg.RfLSSIReadBack, retValue); */
  }

  /* <20120809, Kordan> CCA ON(when exiting), asked by James to avoid reading
   * the wrong value. */
  /* <20120828, Kordan> Toggling CCA would affect RF 0x0, skip it! */
  if (Offset != 0x0 && !(_eepromManager.Version.IS_C_CUT())) {
    _device.phy_set_bb_reg(rCCAonSec_Jaguar, 0x8, 0);
  }
#endif
  return retValue;
}

void RadioManagementModule::phy_RFSerialWrite(RfPath eRFPath, uint32_t Offset,
                                              uint32_t Data) {
  BbRegisterDefinition pPhyReg = PhyRegDef[eRFPath];

  Offset &= 0xff;
  /* Shadow Update */
  /* PHY_RFShadowWrite(adapterState, eRFPath, Offset, Data); */
  /* Put write addr in [27:20]  and write data in [19:00] */
  auto dataAndAddr = ((Offset << 20) | (Data & 0x000fffff)) & 0x0fffffff;

  /* Write Operation */
  /* TODO: Dynamically determine whether using PI or SI to write RF registers.
   */
  _device.phy_set_bb_reg((ushort)pPhyReg.Rf3WireOffset, bMaskDWord,
                         dataAndAddr);
  /* RTW_INFO("RFW-%d Addr[0x%x]=0x%x\n", eRFPath, pPhyReg.Rf3WireOffset,
   * DataAndAddr); */
}

void RadioManagementModule::PHY_SwitchWirelessBand8812(BandType Band) {
  ChannelWidth_t current_bw = _currentChannelBw;
  bool eLNA_2g = _eepromManager->ExternalLNA_2G;

  _logger->info("==>PHY_SwitchWirelessBand8812() %s",
                ((Band == BandType::BAND_ON_2_4G) ? "2.4G" : "5G"));

  current_band_type = Band;

  if (Band == BandType::BAND_ON_2_4G) {
    /* 2.4G band */

    _device.phy_set_bb_reg(rOFDMCCKEN_Jaguar, bOFDMEN_Jaguar | bCCKEN_Jaguar,
                           0x03);

    /* <20131128, VincentL> Remove 0x830[3:1] setting when switching 2G/5G,
     * requested by Yn. */
    _device.phy_set_bb_reg(rBWIndication_Jaguar, 0x3,
                           0x1); /* 0x834[1:0] = 0x1 */
    /* set PD_TH_20M for BB Yn user guide R27 */
    _device.phy_set_bb_reg(rPwed_TH_Jaguar,
                           BIT13 | BIT14 | BIT15 | BIT16 | BIT17,
                           0x17); /* 0x830[17:13]=5'b10111 */

    /* set PWED_TH for BB Yn user guide R29 */

    if (current_bw == ChannelWidth_t::CHANNEL_WIDTH_20 &&
        _eepromManager->version_id.RFType == RF_TYPE_1T1R && eLNA_2g == false) {
      /* 0x830[3:1]=3'b010 */
      _device.phy_set_bb_reg(rPwed_TH_Jaguar, BIT1 | BIT2 | BIT3, 0x02);
    } else {
      /* 0x830[3:1]=3'b100 */
      _device.phy_set_bb_reg(rPwed_TH_Jaguar, BIT1 | BIT2 | BIT3, 0x04);
    }

    /* AGC table select */
    _device.phy_set_bb_reg(rAGC_table_Jaguar, 0x3, 0); /* 0x82C[1:0] = 2b'00 */

    phy_SetRFEReg8812(Band);

    /* <20131106, Kordan> Workaround to fix CCK FA for scan issue. */
    /* if( pHalData.bMPMode == FALSE) */

    _device.phy_set_bb_reg(rTxPath_Jaguar, 0xf0, 0x1);
    _device.phy_set_bb_reg(rCCK_RX_Jaguar, 0x0f000000, 0x1);

    /* CCK_CHECK_en */
    _device.rtw_write8(
        REG_CCK_CHECK_8812,
        (uint8_t)(_device.rtw_read8(REG_CCK_CHECK_8812) & (~BIT7)));
  } else {
    /* 5G band */

    /* CCK_CHECK_en */
    _device.rtw_write8(REG_CCK_CHECK_8812,
                       (uint8_t)(_device.rtw_read8(REG_CCK_CHECK_8812) | BIT7));

    uint16_t count = 0;
    uint16_t reg41A = _device.rtw_read16(REG_TXPKT_EMPTY);
    /* RTW_INFO("Reg41A value %d", reg41A); */
    reg41A &= 0x30;
    while ((reg41A != 0x30) && (count < 50)) {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(50ms);
      /* RTW_INFO("Delay 50us\n"); */

      reg41A = _device.rtw_read16(REG_TXPKT_EMPTY);
      reg41A &= 0x30;
      count++;
      /* RTW_INFO("Reg41A value %d", reg41A); */
    }

    if (count != 0) {
      _logger->info("PHY_SwitchWirelessBand8812(): Switch to 5G Band. Count = "
                    "{:4X} reg41A={:4X}",
                    count, reg41A);
    }

    /* 2012/02/01, Sinda add registry to switch workaround without long-run
     * verification for scan issue. */
    _device.phy_set_bb_reg(rOFDMCCKEN_Jaguar, bOFDMEN_Jaguar | bCCKEN_Jaguar,
                           0x03);

    /* <20131128, VincentL> Remove 0x830[3:1] setting when switching 2G/5G,
     * requested by Yn. */
    _device.phy_set_bb_reg(rBWIndication_Jaguar, 0x3,
                           0x2); /* 0x834[1:0] = 0x2 */
    /* set PD_TH_20M for BB Yn user guide R27 */
    _device.phy_set_bb_reg(rPwed_TH_Jaguar,
                           BIT13 | BIT14 | BIT15 | BIT16 | BIT17,
                           0x15); /* 0x830[17:13]=5'b10101 */

    /* set PWED_TH for BB Yn user guide R29 */
    /* 0x830[3:1]=3'b100 */
    _device.phy_set_bb_reg(rPwed_TH_Jaguar, BIT1 | BIT2 | BIT3, 0x04);

    /* AGC table select */
    _device.phy_set_bb_reg(rAGC_table_Jaguar, 0x3, 1); /* 0x82C[1:0] = 2'b00 */

    phy_SetRFEReg8812(Band);

    /* <20131106, Kordan> Workaround to fix CCK FA for scan issue. */
    /* if( pHalData.bMPMode == FALSE) */
    _device.phy_set_bb_reg(rTxPath_Jaguar, 0xf0, 0x0);
    _device.phy_set_bb_reg(rCCK_RX_Jaguar, 0x0f000000, 0xF);
  }

  phy_SetBBSwingByBand_8812A(Band);
}

void RadioManagementModule::phy_SetRFEReg8812(BandType Band) {
  uint u1tmp = 0;

  if (Band == BandType::BAND_ON_2_4G) {
    switch (_eepromManager->rfe_type) {
    case 0:
    case 2:
      _device.phy_set_bb_reg(rA_RFE_Pinmux_Jaguar, bMaskDWord, 0x77777777);
      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x77777777);
      _device.phy_set_bb_reg(rA_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x000);
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x000);
      break;
    case 1: {
      _device.phy_set_bb_reg(rA_RFE_Pinmux_Jaguar, bMaskDWord, 0x77777777);
      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x77777777);
      _device.phy_set_bb_reg(rA_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x000);
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x000);
    } break;
    case 3:
      _device.phy_set_bb_reg(rA_RFE_Pinmux_Jaguar, bMaskDWord, 0x54337770);
      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x54337770);
      _device.phy_set_bb_reg(rA_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x010);
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x010);
      _device.phy_set_bb_reg(r_ANTSEL_SW_Jaguar, 0x00000303, 0x1);
      break;
    case 4:
      _device.phy_set_bb_reg(rA_RFE_Pinmux_Jaguar, bMaskDWord, 0x77777777);
      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x77777777);
      _device.phy_set_bb_reg(rA_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x001);
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x001);
      break;
    case 5:
      _device.rtw_write8(rA_RFE_Pinmux_Jaguar + 2, 0x77);

      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x77777777);
      u1tmp = _device.rtw_read8(rA_RFE_Inv_Jaguar + 3);
      u1tmp &= ~BIT0;
      _device.rtw_write8(rA_RFE_Inv_Jaguar + 3, (uint8_t)(u1tmp));
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x000);
      break;
    case 6:
      _device.phy_set_bb_reg(rA_RFE_Pinmux_Jaguar, bMaskDWord, 0x07772770);
      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x07772770);
      _device.phy_set_bb_reg(rA_RFE_Inv_Jaguar, bMaskDWord, 0x00000077);
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMaskDWord, 0x00000077);
      break;
    default:
      break;
    }
  } else {
    switch (_eepromManager->rfe_type) {
    case 0:
      _device.phy_set_bb_reg(rA_RFE_Pinmux_Jaguar, bMaskDWord, 0x77337717);
      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x77337717);
      _device.phy_set_bb_reg(rA_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x010);
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x010);
      break;
    case 1: {
      _device.phy_set_bb_reg(rA_RFE_Pinmux_Jaguar, bMaskDWord, 0x77337717);
      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x77337717);
      _device.phy_set_bb_reg(rA_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x000);
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x000);
    } break;
    case 2:
    case 4:
      _device.phy_set_bb_reg(rA_RFE_Pinmux_Jaguar, bMaskDWord, 0x77337777);
      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x77337777);
      _device.phy_set_bb_reg(rA_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x010);
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x010);
      break;
    case 3:
      _device.phy_set_bb_reg(rA_RFE_Pinmux_Jaguar, bMaskDWord, 0x54337717);
      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x54337717);
      _device.phy_set_bb_reg(rA_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x010);
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x010);
      _device.phy_set_bb_reg(r_ANTSEL_SW_Jaguar, 0x00000303, 0x1);
      break;
    case 5:
      _device.rtw_write8(rA_RFE_Pinmux_Jaguar + 2, 0x33);
      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x77337777);
      u1tmp = _device.rtw_read8(rA_RFE_Inv_Jaguar + 3);
      _device.rtw_write8(rA_RFE_Inv_Jaguar + 3, (uint8_t)(u1tmp |= BIT0));
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x010);
      break;
    case 6:
      _device.phy_set_bb_reg(rA_RFE_Pinmux_Jaguar, bMaskDWord, 0x07737717);
      _device.phy_set_bb_reg(rB_RFE_Pinmux_Jaguar, bMaskDWord, 0x07737717);
      _device.phy_set_bb_reg(rA_RFE_Inv_Jaguar, bMaskDWord, 0x00000077);
      _device.phy_set_bb_reg(rB_RFE_Inv_Jaguar, bMaskDWord, 0x00000077);
      break;
    default:
      break;
    }
  }
}

void RadioManagementModule::phy_SetBBSwingByBand_8812A(BandType Band) {
  _device.phy_set_bb_reg(
      rA_TxScale_Jaguar, 0xFFE00000,
      phy_get_tx_bb_swing_8812a(Band, RfPath::RF_PATH_A)); /* 0xC1C[31:21] */
  _device.phy_set_bb_reg(
      rB_TxScale_Jaguar, 0xFFE00000,
      phy_get_tx_bb_swing_8812a(Band, RfPath::RF_PATH_B)); /* 0xE1C[31:21] */
}

#define EEPROM_TX_BBSWING_2G_8812 0xC6
#define EEPROM_TX_BBSWING_5G_8812 0xC7

uint32_t RadioManagementModule::phy_get_tx_bb_swing_8812a(BandType Band,
                                                          RfPath RFPath) {
  int8_t bbSwing_2G = (int8_t)(-1 * registry_priv::TxBBSwing_2G);
  int8_t bbSwing_5G = (int8_t)(-1 * registry_priv::TxBBSwing_5G);
  uint32_t _out = 0x200;
  const int8_t AUTO = -1;

  if (_device.AutoloadFailFlag) {
    if (Band == BandType::BAND_ON_2_4G) {
      if (bbSwing_2G == 0)
        _out = 0x200; /* 0 dB */
      else if (bbSwing_2G == -3)
        _out = 0x16A; /* -3 dB */
      else if (bbSwing_2G == -6)
        _out = 0x101; /* -6 dB */
      else if (bbSwing_2G == -9)
        _out = 0x0B6; /* -9 dB */
      else {
        if (_eepromManager->ExternalPA_2G) {
          _out = 0x16A;
        } else {
          _out = 0x200;
        }
      }
    } else if (Band == BandType::BAND_ON_5G) {
      if (bbSwing_5G == 0)
        _out = 0x200; /* 0 dB */

      else if (bbSwing_5G == -3)
        _out = 0x16A; /* -3 dB */

      else if (bbSwing_5G == -6)
        _out = 0x101; /* -6 dB */

      else if (bbSwing_5G == -9)
        _out = 0x0B6; /* -9 dB */

      else {
        _out = 0x200;
      }
    } else {
      _out = 0x16A; /* -3 dB */
    }
  } else {
    uint8_t swing = 0;
    int8_t onePathSwing = 0;

    if (Band == BandType::BAND_ON_2_4G) {
      if (registry_priv::TxBBSwing_2G == AUTO) {
        _eepromManager->efuse_ShadowRead1Byte(EEPROM_TX_BBSWING_2G_8812,
                                              &swing);
        swing = (swing == 0xFF) ? (uint8_t)0x00 : swing;
      } else if (bbSwing_2G == 0)
        swing = 0x00; /* 0 dB */
      else if (bbSwing_2G == -3)
        swing = 0x05; /* -3 dB */
      else if (bbSwing_2G == -6)
        swing = 0x0A; /* -6 dB */
      else if (bbSwing_2G == -9)
        swing = 0xFF; /* -9 dB */
      else
        swing = 0x00;
    } else {
      if (registry_priv::TxBBSwing_5G == AUTO) {
        _eepromManager->efuse_ShadowRead1Byte(EEPROM_TX_BBSWING_5G_8812,
                                              &swing);
        swing = (swing == 0xFF) ? (uint8_t)0x00 : swing;
      } else if (bbSwing_5G == 0)
        swing = 0x00; /* 0 dB */
      else if (bbSwing_5G == -3)
        swing = 0x05; /* -3 dB */
      else if (bbSwing_5G == -6)
        swing = 0x0A; /* -6 dB */
      else if (bbSwing_5G == -9)
        swing = 0xFF; /* -9 dB */
      else
        swing = 0x00;
    }

    if (RFPath == RfPath::RF_PATH_A) {
      onePathSwing = (uint8_t)((swing & 0x3) >> 0); /* 0xC6/C7[1:0] */
    } else if (RFPath == RfPath::RF_PATH_B) {
      onePathSwing = (uint8_t)((swing & 0xC) >> 2); /* 0xC6/C7[3:2] */
    }

    if (onePathSwing == 0x0) {
      _out = 0x200; /* 0 dB */
    } else if (onePathSwing == 0x1) {
      _out = 0x16A; /* -3 dB */
    } else if (onePathSwing == 0x2) {
      _out = 0x101; /* -6 dB */
    } else if (onePathSwing == 0x3) {
      _out = 0x0B6; /* -9 dB */
    }
  }

  /* RTW_INFO("<=== phy_get_tx_bb_swing_8812a, out = 0x%X\n", out); */

  return _out;
}

void RadioManagementModule::init_hw_mlme_ext(SelectedChannel pmlmeext) {
  /* Modify to make sure first time change channel(band) would be done properly
   */
  _currentChannel = 0;
  _currentChannelBw = ChannelWidth_t::CHANNEL_WIDTH_MAX;
  current_band_type = BandType::BAND_MAX;

  /* set_opmode_cmd(padapter, infra_client_with_mlme); */ /* removed */
  Set_HW_VAR_ENABLE_RX_BAR(true);
  set_channel_bwmode(pmlmeext.Channel, pmlmeext.ChannelOffset,
                     pmlmeext.ChannelWidth);
}

void RadioManagementModule::Set_HW_VAR_ENABLE_RX_BAR(bool val) {
  if (val) {
    /* enable RX BAR */
    uint32_t val16 = _device.rtw_read16(REG_RXFLTMAP1);

    val16 |= BIT8;
    _device.rtw_write16(REG_RXFLTMAP1, (uint16_t)val16);
  } else {
    /* disable RX BAR */
    uint32_t val16 = _device.rtw_read16(REG_RXFLTMAP1);

    val16 &= ~BIT8;
    _device.rtw_write16(REG_RXFLTMAP1, (uint16_t)val16);
  }

  _logger->info("[HW_VAR_ENABLE_RX_BAR] 0x{:4X}=0x{:4X}", REG_RXFLTMAP1,
                _device.rtw_read16(REG_RXFLTMAP1));
}
