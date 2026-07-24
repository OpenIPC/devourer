#pragma once
#include <cstdint>
#include <vector>
#include <array>
#include <functional>
#include "Wpa2Crypto.h"
namespace apfpv {
using Mac = std::array<uint8_t,6>;
// AP-side WPA2-PSK 4-way handshake (Authenticator) — the mirror of Wpa2Supplicant. One
// instance per associating station. The AP sends M1 (ANonce) + M3 (GTK), and verifies the
// station's M2/M4 MICs. Same crypto (PRF-SHA1 PTK, HMAC-SHA1 MIC) so keys agree with any
// standard supplicant.
class Wpa2Authenticator {
public:
    using SendFn = std::function<bool(const std::vector<uint8_t>&)>;
    Wpa2Authenticator(Crypto c, SendFn send);
    void begin(const std::array<uint8_t,32>& pmk, const Mac& ap, const Mac& sta,
               const std::array<uint8_t,16>& gtk, uint8_t gtkKeyId);
    void startHandshake();                              // generate ANonce + send M1
    bool onEapolKey(const uint8_t* body, size_t len);   // M2 -> M3, M4 -> Done (returns true on M4)
    bool ready() const { return _state == State::Done; }
private:
    enum class State { Idle, WaitM2, WaitM4, Done };
    void derivePtk(const uint8_t* sNonce);
    std::vector<uint8_t> buildM1();
    std::vector<uint8_t> buildM3();
    bool verifyMic(const uint8_t* body, size_t len) const;
    Crypto _c; SendFn _send;
    Mac _ap{}, _sta{};
    std::array<uint8_t,32> _pmk{}, _aNonce{}, _sNonce{};
    std::array<uint8_t,16> _kck{}, _kek{}, _tk{}, _gtk{};
    uint8_t _gtkKeyId = 1;
    State _state = State::Idle;
    uint64_t _replay = 0;
};
}
