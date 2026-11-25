/* Headless guard for the MT76x0U channel math (src/mt76x0u/chan.cpp) - the
 * part of the channel set that is pure, and the part a wrong guess would
 * break silently on air (the receiver tunes, just to the wrong place):
 *
 *  - every 2.4/5 GHz entry of a real monitoring application's hop list
 *    (iw-style "36 ht40+", "40 ht40-", ..., "165") is accepted, and its RF
 *    center channel is in the kernel's frequency plan;
 *  - HT40+/- map to mt76x0_phy_set_channel's ch_group_index 0/1, the center
 *    channel is primary +/- 2, and the EXT_CCA value is the kernel's table;
 *  - pairs that do not exist, channel 14 at 40 MHz and channels beyond the
 *    plan are refused;
 *  - the radar flag that gates the DFS AGC adjust;
 *  - the monitor RX filter value: the kernel's monitor semantics, and with
 *    control frames refused it reduces to wifit3's measured 0x00017f93. */
#include <cstdio>

#include "internal.h"
#include "phy.h"

namespace {

int fails;

void expect(const char *what, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "mt76x0u_chan: FAIL %s\n", what);
    fails++;
  }
}

bool shape(uint8_t ch, int ht40, mt0_chan_shape &s) {
  const char *err = nullptr;
  return mt0_chan_shape_for(ch, ht40, &s, &err) == 0;
}

} // namespace

int main() {
  mt0_chan_shape s{};

  /* A monitoring application's hop list, 2.4 and 5 GHz entries. */
  const struct { uint8_t ch; int ht40; } hops[] = {
      {1, 1},    {2, 1},    {3, 1},    {4, 1},    {5, 1},    {5, -1},
      {6, 1},    {6, -1},   {7, 1},    {7, -1},   {8, 1},    {8, -1},
      {9, -1},   {9, 1},    {10, -1},  {11, -1},  {12, -1},  {13, -1},
      {36, 1},   {40, -1},  {44, 1},   {48, -1},  {52, 1},   {56, -1},
      {60, 1},   {64, -1},  {100, 1},  {104, -1}, {108, 1},  {112, -1},
      {116, 1},  {120, -1}, {124, 1},  {128, -1}, {132, 1},  {136, -1},
      {140, 1},  {144, -1}, {149, 1},  {153, -1}, {157, 1},  {161, -1},
      {165, 0},
  };
  int ok = 0;
  for (const auto &e : hops) {
    if (shape(e.ch, e.ht40, s) && s.center == e.ch + (e.ht40 > 0 ? 2 : e.ht40 < 0 ? -2 : 0))
      ok++;
    else
      std::fprintf(stderr, "mt76x0u_chan: hop-list entry %u %+d refused or mis-centred\n",
                   e.ch, e.ht40);
  }
  expect("all 43 hop-list entries tune", ok == 43);

  expect("36 HT40+", shape(36, 1, s) && s.center == 38 && s.ch_group == 0 &&
                         s.width == 40 && s.rf_bw_band == (RF_A_BAND | RF_BW_40));
  expect("40 HT40- shares the 38 center", shape(40, -1, s) && s.center == 38 &&
                                              s.ch_group == 1);
  expect("6 HT40- centers on 4", shape(6, -1, s) && s.center == 4 &&
                                     s.rf_bw_band == (RF_G_BAND | RF_BW_40));
  expect("6 at 20 MHz", shape(6, 0, s) && s.center == 6 && s.width == 20 &&
                            s.ch_group == 0 && s.rf_bw_band == (RF_G_BAND | RF_BW_20));
  expect("165 HT40+ centers on 167 (in the plan)", shape(165, 1, s) && s.center == 167);
  expect("144 HT40- centers on 142", shape(144, -1, s) && s.center == 142);

  expect("1 HT40- refused", !shape(1, -1, s));
  expect("13 HT40+ refused", !shape(13, 1, s));
  expect("14 at 40 MHz refused", !shape(14, 1, s) && !shape(14, -1, s));
  expect("14 at 20 MHz accepted", shape(14, 0, s));
  expect("40 HT40+ refused (not a pair start)", !shape(40, 1, s));
  expect("144 HT40+ refused", !shape(144, 1, s));
  expect("177 refused (beyond the plan)", !shape(177, 0, s));
  expect("37 refused (not a 20 MHz raster channel)", !shape(37, 0, s));
  expect("6 GHz channel numbers refused", !shape(233, 0, s));

  expect("EXT_CCA group 0", shape(36, 1, s) && s.ext_cca == 0x1e4);
  expect("EXT_CCA group 1", shape(40, -1, s) && s.ext_cca == 0x2e1);

  expect("52 is radar", shape(52, 0, s) && s.radar);
  expect("144 is radar", shape(144, 0, s) && s.radar);
  expect("36 is not radar", shape(36, 0, s) && !s.radar);
  expect("149 is not radar", shape(149, 0, s) && !s.radar);
  expect("6 is not radar", shape(6, 0, s) && !s.radar);

  /* MT_RX_FILTR_CFG bits are DROP bits; 0x00017f97 is the init table's. */
  expect("monitor filter, control frames kept",
         mt0_monitor_filter_value(0x00017f97, 1) == 0x00001093);
  expect("monitor filter, control refused = wifit3's measured 0x00017f93",
         mt0_monitor_filter_value(0x00017f97, 0) == 0x00017f93);
  expect("CRC and PHY errors stay dropped",
         (mt0_monitor_filter_value(0x00017f97, 1) & 0x3) == 0x3);

  if (fails) {
    std::fprintf(stderr, "mt76x0u_chan: %d failure(s)\n", fails);
    return 1;
  }
  std::printf("mt76x0u_chan: ok\n");
  return 0;
}
