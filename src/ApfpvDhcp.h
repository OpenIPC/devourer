#pragma once
#include <cstdint>
#include <array>
#include <vector>
#include <functional>
namespace apfpv {
using Mac = std::array<uint8_t,6>;
class ApfpvDhcp {
public:
    using SendUdpFn = std::function<bool(const std::vector<uint8_t>&)>;
    struct Lease { uint32_t ip=0, netmask=0, server=0; bool valid=false; };
    ApfpvDhcp(const Mac& self, SendUdpFn send);
    bool start();
    void retransmit();   // resend the CURRENT pending message (DISCOVER or REQUEST), no reset
    void onBootpReply(const uint8_t* bootp, size_t len);
    const Lease& lease() const { return _lease; }
    // Skip DHCP and bind a fixed lease (static-IP option). netmask 0 -> /24, gateway 0 -> subnet .1.
    void claimStatic(uint32_t ip, uint32_t netmask = 0, uint32_t gateway = 0);
    void claimStatic_192_168_0_10() { claimStatic(0xC0A8000A, 0xFFFFFF00, 0xC0A80001); }
private:
    enum class St { Init, Discover, Request, Bound };
    std::vector<uint8_t> buildDiscover();
    std::vector<uint8_t> buildRequest(uint32_t offeredIp, uint32_t serverId);
    Mac _self; SendUdpFn _send; St _state = St::Init; Lease _lease; uint32_t _xid = 0;
    uint32_t _offerIp = 0, _offerSid = 0;   // remembered from the OFFER for REQUEST retransmits
};
}
