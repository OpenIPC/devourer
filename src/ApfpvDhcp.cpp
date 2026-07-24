// ============================================================================
//  ApfpvDhcp.cpp — minimal DHCP client, route/DNS-suppressed (impl)
//  Mirrors the real VRX udhcpc.apfpv.script ("not add routes and dns for apfpv
//  interfaces"): take the IP, ignore router(opt 3)/DNS(opt 6). Declarations in
//  ApfpvDhcp.h. APFPV reliably hands 192.168.0.10.
// ============================================================================
#include "ApfpvDhcp.h"
#include <cstring>

namespace apfpv {

static void put32(std::vector<uint8_t>& v, uint32_t x) {
    v.push_back(x>>24); v.push_back(x>>16); v.push_back(x>>8); v.push_back(x);
}

ApfpvDhcp::ApfpvDhcp(const Mac& self, SendUdpFn send) : _self(self), _send(std::move(send)) {}

// Build a BOOTP/DHCP message (op=1 BOOTREQUEST). Minimal but valid.
static std::vector<uint8_t> bootp(const Mac& self, uint32_t xid, uint8_t msgType,
                                  uint32_t reqIp, uint32_t serverId) {
    std::vector<uint8_t> p;
    p.push_back(1); p.push_back(1); p.push_back(6); p.push_back(0);  // op,htype,hlen,hops
    put32(p, xid);
    p.push_back(0); p.push_back(0);                  // secs
    p.push_back(0x80); p.push_back(0x00);            // flags = broadcast
    put32(p, 0); put32(p, 0); put32(p, 0); put32(p, 0); // ci,yi,si,gi addr
    p.insert(p.end(), self.begin(), self.end()); // chaddr
    p.resize(p.size() + 10, 0);                      // chaddr pad
    p.resize(236, 0);                                // sname+file
    put32(p, 0x63825363);                            // magic cookie
    // option 53: DHCP message type
    p.push_back(53); p.push_back(1); p.push_back(msgType);
    if (reqIp) { p.push_back(50); p.push_back(4); put32(p, reqIp); }      // requested IP
    if (serverId) { p.push_back(54); p.push_back(4); put32(p, serverId); } // server id
    // option 55: param request list — deliberately request ONLY subnet mask,
    // NOT router(3)/DNS(6), matching udhcpc.apfpv.script intent.
    p.push_back(55); p.push_back(1); p.push_back(1);
    p.push_back(255);                                // end
    return p;
}

std::vector<uint8_t> ApfpvDhcp::buildDiscover() { return bootp(_self, _xid, 1, 0, 0); }
std::vector<uint8_t> ApfpvDhcp::buildRequest(uint32_t ip, uint32_t sid) { return bootp(_self, _xid, 3, ip, sid); }

bool ApfpvDhcp::start() {
    _xid = 0x41504650;                  // "APFP"
    _state = St::Discover;
    return _send(buildDiscover());
}

void ApfpvDhcp::onBootpReply(const uint8_t* p, size_t len) {
    if (len < 240) return;
    if ((p[236]<<24|p[237]<<16|p[238]<<8|p[239]) != 0x63825363) return; // cookie
    uint32_t yi = (p[16]<<24)|(p[17]<<16)|(p[18]<<8)|p[19];
    // parse options for msg type (53), netmask (1), server id (54). IGNORE 3/6.
    uint8_t msgType = 0; uint32_t mask = 0, sid = 0;
    size_t i = 240;
    while (i + 2 <= len) {
        uint8_t opt = p[i], olen = p[i+1];
        if (opt == 255) break;
        if (opt == 0) { i++; continue; }
        if (i + 2 + olen > len) break;
        const uint8_t* d = p + i + 2;
        if (opt == 53 && olen == 1) msgType = d[0];
        else if (opt == 1 && olen == 4) mask = (d[0]<<24)|(d[1]<<16)|(d[2]<<8)|d[3];
        else if (opt == 54 && olen == 4) sid = (d[0]<<24)|(d[1]<<16)|(d[2]<<8)|d[3];
        // opt 3 (router) and opt 6 (DNS) intentionally skipped.
        i += 2 + olen;
    }
    if (_state == St::Discover && msgType == 2) {      // OFFER -> REQUEST
        _state = St::Request;
        _offerIp = yi; _offerSid = sid;                // remember for REQUEST retransmits
        _send(buildRequest(yi, sid));
    } else if (_state == St::Request && msgType == 5) { // ACK -> bound
        _lease.ip = yi; _lease.netmask = mask; _lease.server = sid; _lease.valid = true;
        _state = St::Bound;
    }
}

void ApfpvDhcp::retransmit() {
    // Resend whichever message we're waiting on a reply for, WITHOUT resetting the state
    // machine — so a missed ACK retries the REQUEST (not a fresh DISCOVER that restarts DORA).
    if (_state == St::Discover || _state == St::Init) _send(buildDiscover());
    else if (_state == St::Request) _send(buildRequest(_offerIp, _offerSid));
}

void ApfpvDhcp::claimStatic(uint32_t ip, uint32_t netmask, uint32_t gateway) {
    if (netmask == 0) netmask = 0xFFFFFF00;            // default /24
    if (gateway == 0) gateway = (ip & netmask) | 1;   // default = subnet's .1
    _lease.ip = ip; _lease.netmask = netmask; _lease.server = gateway;
    _lease.valid = true; _state = St::Bound;
}

} // namespace apfpv
