// ============================================================================
//  Wpa2Authenticator.cpp — AP-side WPA2-PSK 4-way (the mirror of Wpa2Supplicant).
//  Sends M1(ANonce) + M3(GTK, KEK-wrapped); verifies the station's M2/M4 MICs.
// ============================================================================
#include "Wpa2Authenticator.h"
#include "Dot11Frames.h"
#include "crypto/AesCcm.h"
#include <cstring>
#include <cstdio>

#ifdef _WIN32
extern "C" __declspec(dllimport) long __stdcall
    BCryptGenRandom(void*, unsigned char*, unsigned long, unsigned long);
#endif
static void ap_secure_rand(uint8_t* b, size_t n) {
#ifdef _WIN32
    if (BCryptGenRandom(nullptr, b, (unsigned long)n, 0x00000002u) == 0) return;
#else
    FILE* u = fopen("/dev/urandom","rb"); if (u) { size_t r=fread(b,1,n,u); fclose(u); if (r==n) return; }
#endif
    for (size_t i=0;i<n;i++) b[i]=(uint8_t)((i*131)^0x5a);
}

namespace apfpv {

Wpa2Authenticator::Wpa2Authenticator(Crypto c, SendFn send)
    : _c(std::move(c)), _send(std::move(send)) {}

void Wpa2Authenticator::begin(const std::array<uint8_t,32>& pmk, const Mac& ap, const Mac& sta,
                              const std::array<uint8_t,16>& gtk, uint8_t gtkKeyId) {
    _pmk = pmk; _ap = ap; _sta = sta; _gtk = gtk; _gtkKeyId = gtkKeyId;
    _state = State::Idle; _replay = 0;
}

// Identical PRF input to Wpa2Supplicant::derivePtk (min/max MAC, min/max nonce) so keys agree.
void Wpa2Authenticator::derivePtk(const uint8_t* sNonce) {
    std::memcpy(_sNonce.data(), sNonce, 32);
    uint8_t data[76];
    const Mac& aa = _ap; const Mac& sa = _sta;
    bool aaFirst = std::memcmp(aa.data(), sa.data(), 6) < 0;
    std::memcpy(data + 0, (aaFirst?aa:sa).data(), 6);
    std::memcpy(data + 6, (aaFirst?sa:aa).data(), 6);
    bool aFirst = std::memcmp(_aNonce.data(), _sNonce.data(), 32) < 0;
    std::memcpy(data + 12, aFirst?_aNonce.data():_sNonce.data(), 32);
    std::memcpy(data + 44, aFirst?_sNonce.data():_aNonce.data(), 32);
    auto ptk = _c.prf(_pmk.data(), 32, "Pairwise key expansion", data, sizeof(data), 48);
    std::memcpy(_kck.data(), ptk.data() + 0,  16);
    std::memcpy(_kek.data(), ptk.data() + 16, 16);
    std::memcpy(_tk.data(),  ptk.data() + 32, 16);
}

std::vector<uint8_t> Wpa2Authenticator::buildM1() {
    std::vector<uint8_t> kb(99, 0);
    kb[0]=0x02; kb[1]=0x03;
    uint16_t bl=(uint16_t)(kb.size()-4); kb[2]=bl>>8; kb[3]=bl&0xff;
    kb[4]=0x02;                                  // RSN descriptor
    uint16_t info=0x008A; kb[5]=info>>8; kb[6]=info&0xff;   // ver2 + pairwise + ACK (no MIC)
    kb[7]=0x00; kb[8]=0x10;                       // key length 16
    for(int i=0;i<8;i++) kb[9+i]=(uint8_t)(_replay>>((7-i)*8));   // replay counter (BE)
    std::memcpy(kb.data()+17, _aNonce.data(), 32);
    return BuildEapolKeyFromAp(_ap, _sta, kb);
}

std::vector<uint8_t> Wpa2Authenticator::buildM3() {
    // Key Data = RSN IE (matches the beacon) + GTK KDE, then AES-key-wrapped under the KEK.
    std::vector<uint8_t> kd;
    static const uint8_t rsn[] = {0x30,0x14, 0x01,0x00, 0x00,0x0f,0xac,0x04,
        0x01,0x00, 0x00,0x0f,0xac,0x04, 0x01,0x00, 0x00,0x0f,0xac,0x02, 0x00,0x00};
    kd.insert(kd.end(), rsn, rsn+sizeof(rsn));                          // 22
    kd.push_back(0xdd); kd.push_back(22);                               // GTK KDE: dd len
    kd.push_back(0x00); kd.push_back(0x0f); kd.push_back(0xac); kd.push_back(0x01);
    kd.push_back(_gtkKeyId & 0x03); kd.push_back(0x00);                 // keyid | reserved
    kd.insert(kd.end(), _gtk.begin(), _gtk.end());                      // +24 => 46
    if (kd.size() % 8 != 0) { kd.push_back(0xdd); while (kd.size() % 8) kd.push_back(0x00); }
    std::vector<uint8_t> wrapped(kd.size()+8);
    crypto::aes_key_wrap(_kek.data(), kd.data(), kd.size(), wrapped.data());

    std::vector<uint8_t> kb(99 + wrapped.size(), 0);
    kb[0]=0x02; kb[1]=0x03;
    uint16_t bl=(uint16_t)(kb.size()-4); kb[2]=bl>>8; kb[3]=bl&0xff;
    kb[4]=0x02;
    uint16_t info=0x134A; kb[5]=info>>8; kb[6]=info&0xff;  // ver2+pairwise+install+ack+mic+secure+enc
    kb[7]=0x00; kb[8]=0x10;
    for(int i=0;i<8;i++) kb[9+i]=(uint8_t)(_replay>>((7-i)*8));
    std::memcpy(kb.data()+17, _aNonce.data(), 32);
    uint16_t kdl=(uint16_t)wrapped.size(); kb[97]=kdl>>8; kb[98]=kdl&0xff;
    std::memcpy(kb.data()+99, wrapped.data(), wrapped.size());
    auto mic = _c.eapol_mic(_kck.data(), kb.data(), kb.size());        // MIC over the frame (MIC=0)
    std::memcpy(kb.data()+81, mic.data(), 16);
    return BuildEapolKeyFromAp(_ap, _sta, kb);
}

bool Wpa2Authenticator::verifyMic(const uint8_t* body, size_t len) const {
    if (len < 97) return false;                        // need the 16-byte MIC field at [81..96]
    std::vector<uint8_t> m(body, body+len);
    uint8_t rx[16]; std::memcpy(rx, body+81, 16);
    std::memset(m.data()+81, 0, 16);
    auto calc = _c.eapol_mic(_kck.data(), m.data(), m.size());
    return std::memcmp(calc.data(), rx, 16) == 0;
}

void Wpa2Authenticator::startHandshake() {
    ap_secure_rand(_aNonce.data(), 32);
    _replay = 1;
    _send(buildM1());
    _state = State::WaitM2;
    fprintf(stderr, "[ap-wpa] sent M1 (ANonce) -> WaitM2\n");
}

bool Wpa2Authenticator::onEapolKey(const uint8_t* body, size_t len) {
    if (len < 95) return false;
    uint16_t info = (body[5]<<8)|body[6];
    bool hasMic = info & 0x0100, secure = info & 0x0200;
    if (_state == State::WaitM2 && hasMic && !secure) {        // M2: SNonce + MIC
        derivePtk(body + 17);
        if (!verifyMic(body, len)) { fprintf(stderr, "[ap-wpa] M2 MIC mismatch\n"); return false; }
        _replay++;                                             // M3 uses the next counter
        _send(buildM3());
        _state = State::WaitM4;
        fprintf(stderr, "[ap-wpa] M2 ok -> sent M3\n");
        return false;
    }
    if (_state == State::WaitM4 && hasMic && secure) {         // M4: confirm
        if (!verifyMic(body, len)) { fprintf(stderr, "[ap-wpa] M4 MIC mismatch\n"); return false; }
        _state = State::Done;
        fprintf(stderr, "[ap-wpa] M4 ok -> 4-way COMPLETE (station authenticated)\n");
        return true;
    }
    return false;
}

}  // namespace apfpv
