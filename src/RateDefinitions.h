#ifndef DEVOURER_RATE_DEFINITIONS_H
#define DEVOURER_RATE_DEFINITIONS_H

/* Realtek MGN_* rate enumeration — the driver-internal rate index used across
 * both chip generations (Jaguar1 RadioManagementModule and Jaguar3
 * RadioManagementJaguar3). Kept in a neutral header so neither generation's
 * files depend on the other's. */
enum MGN_RATE {
  MGN_1M = 0x02,
  MGN_2M = 0x04,
  MGN_5_5M = 0x0B,
  MGN_6M = 0x0C,
  MGN_9M = 0x12,
  MGN_11M = 0x16,
  MGN_12M = 0x18,
  MGN_18M = 0x24,
  MGN_24M = 0x30,
  MGN_36M = 0x48,
  MGN_48M = 0x60,
  MGN_54M = 0x6C,
  MGN_MCS32 = 0x7F,
  MGN_MCS0,
  MGN_MCS1,
  MGN_MCS2,
  MGN_MCS3,
  MGN_MCS4,
  MGN_MCS5,
  MGN_MCS6,
  MGN_MCS7,
  MGN_MCS8,
  MGN_MCS9,
  MGN_MCS10,
  MGN_MCS11,
  MGN_MCS12,
  MGN_MCS13,
  MGN_MCS14,
  MGN_MCS15,
  MGN_MCS16,
  MGN_MCS17,
  MGN_MCS18,
  MGN_MCS19,
  MGN_MCS20,
  MGN_MCS21,
  MGN_MCS22,
  MGN_MCS23,
  MGN_MCS24,
  MGN_MCS25,
  MGN_MCS26,
  MGN_MCS27,
  MGN_MCS28,
  MGN_MCS29,
  MGN_MCS30,
  MGN_MCS31,
  MGN_VHT1SS_MCS0,
  MGN_VHT1SS_MCS1,
  MGN_VHT1SS_MCS2,
  MGN_VHT1SS_MCS3,
  MGN_VHT1SS_MCS4,
  MGN_VHT1SS_MCS5,
  MGN_VHT1SS_MCS6,
  MGN_VHT1SS_MCS7,
  MGN_VHT1SS_MCS8,
  MGN_VHT1SS_MCS9,
  MGN_VHT2SS_MCS0,
  MGN_VHT2SS_MCS1,
  MGN_VHT2SS_MCS2,
  MGN_VHT2SS_MCS3,
  MGN_VHT2SS_MCS4,
  MGN_VHT2SS_MCS5,
  MGN_VHT2SS_MCS6,
  MGN_VHT2SS_MCS7,
  MGN_VHT2SS_MCS8,
  MGN_VHT2SS_MCS9,
  MGN_VHT3SS_MCS0,
  MGN_VHT3SS_MCS1,
  MGN_VHT3SS_MCS2,
  MGN_VHT3SS_MCS3,
  MGN_VHT3SS_MCS4,
  MGN_VHT3SS_MCS5,
  MGN_VHT3SS_MCS6,
  MGN_VHT3SS_MCS7,
  MGN_VHT3SS_MCS8,
  MGN_VHT3SS_MCS9,
  MGN_VHT4SS_MCS0,
  MGN_VHT4SS_MCS1,
  MGN_VHT4SS_MCS2,
  MGN_VHT4SS_MCS3,
  MGN_VHT4SS_MCS4,
  MGN_VHT4SS_MCS5,
  MGN_VHT4SS_MCS6,
  MGN_VHT4SS_MCS7,
  MGN_VHT4SS_MCS8,
  MGN_VHT4SS_MCS9,
  MGN_UNKNOWN
};

/* TX-descriptor RATE_ID (RA group) for a forced-rate injected frame, from the
 * frame's rate family + NSS + band — the 8822B/8822C-era RATEID_IDX_* table
 * (vendor ieee80211.h). The group defines the rate-space the firmware retry
 * ladder walks, and a family-mismatched group is exactly how retries of an
 * MCS frame end up on the legacy chain (54M..6M on the 8822C, VHT wandering
 * on the 8822B) — witness-measured per copy, tests/retry_ladder_probe.sh:
 * with the family-correct group the 8822C fw steps MCSx -> MCS(x-1) -> 6M
 * floor at 5 GHz (VHT: M7 -> M4 -> M1 -> M0 -> 6M), and at 2.4 GHz the
 * B-containing group adds the CCK floor. 2SS group rows follow the vendor
 * table symmetrically (unmeasured — no 2SS retry cell has run). */
inline unsigned char rateid_for_mgn(unsigned char mgn, bool band5g) {
  if (mgn >= MGN_VHT1SS_MCS0)                     /* VHT: 10 rates per NSS */
    return mgn >= MGN_VHT2SS_MCS0 ? 9 /* VHT_2SS */ : 10 /* VHT_1SS */;
  if (mgn >= MGN_MCS0)                            /* HT */
    return mgn >= MGN_MCS8 ? (band5g ? 4 /* GN_N2SS */ : 2 /* BGN_20M_2SS */)
                           : (band5g ? 5 /* GN_N1SS */ : 3 /* BGN_20M_1SS */);
  if (mgn == MGN_MCS32) /* HT duplicate mode — 0x7F sorts BELOW MGN_MCS0 */
    return band5g ? 5 : 3;
  if (mgn == MGN_1M || mgn == MGN_2M || mgn == MGN_5_5M || mgn == MGN_11M)
    return 8; /* B */
  return band5g ? 7 /* G */ : 6 /* BG */;
}

#endif /* DEVOURER_RATE_DEFINITIONS_H */
