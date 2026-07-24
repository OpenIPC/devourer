#pragma once
#include <cstdint>
namespace apfpv {
enum class StationFrameKind { Mgmt, EapolData, CcmpData, BroadcastMgmt, Beacon };
void FillStationTxDesc(uint8_t* txdesc, uint16_t payloadLen, uint8_t descOffset,
                       uint8_t macId, StationFrameKind kind, uint8_t rateId, uint8_t txRate);
}
