"""Cut a real WPA2 four-way out of a capture into C test vectors.

Like tests/ccmp_extract_vectors.py, this does NO crypto. It finds the four
EAPOL-Key frames, copies their bytes, and writes down the PTK and GTK that
wpa_supplicant and hostapd logged. Everything the test then asserts is
something one of those two produced.

That is the point. tests/supplicant_selftest.cpp drives src/sta/Supplicant.h
with a fixture authenticator written by the same author from the same reading
of the same clause; it can pin the state machine and cannot pin the PRF, the
MIC or the KDE layout. These frames were produced by hostapd and answered by
wpa_supplicant.

NOT REPRODUCIBLE FROM THE TREE ALONE, and so there is no --check mode: the
input is a pcap plus the keys hostapd and wpa_supplicant logged during a
mac80211_hwsim run (tests/eapol_capture_vectors.sh, root + hwsim), none of
which is checked in - and a fresh run draws fresh nonces, so it would produce
a different, equally valid four-way. The checked-in
tests/eapol_kernel_vectors.h is the artifact; this script documents how it was
cut.
"""
import struct
import sys

pcap = sys.argv[1]
ptk_hex = sys.argv[2]
gtk_hex = sys.argv[3]
psk = sys.argv[4]
ssid = sys.argv[5]
out_path = sys.argv[6]


def radiotap_flags(pkt):
    """The radiotap Flags octet, or None. Only one field can precede it."""
    rt_len = struct.unpack('<H', pkt[2:4])[0]
    words, off = [], 4
    while True:
        w = struct.unpack('<I', pkt[off:off + 4])[0]
        words.append(w)
        off += 4
        if not (w & 0x80000000):
            break
    if not (words[0] & 0x02):
        return None
    if words[0] & 0x01:
        off += (-off) % 8 + 8          # TSFT: 8 bytes, 8-aligned
    return pkt[off] if off < rt_len else None


blob = open(pcap, 'rb').read()
magic = blob[:4]
endian = '<' if magic == b'\xd4\xc3\xb2\xa1' else '>'
if magic not in (b'\xd4\xc3\xb2\xa1', b'\xa1\xb2\xc3\xd4'):
    raise SystemExit('not a classic pcap')
if struct.unpack(endian + 'I', blob[20:24])[0] != 127:
    raise SystemExit('expected DLT_IEEE802_11_RADIOTAP')

off = 24
frames = []
while off + 16 <= len(blob):
    _, _, incl, _ = struct.unpack(endian + 'IIII', blob[off:off + 16])
    off += 16
    pkt = blob[off:off + incl]
    off += incl
    if len(pkt) < 8:
        continue
    rt_len = struct.unpack('<H', pkt[2:4])[0]
    flags = radiotap_flags(pkt)
    assert flags is not None and not (flags & 0x10), 'capture carries an FCS'
    m = pkt[rt_len:]
    if len(m) < 24 + 8:
        continue
    fc0, fc1 = m[0], m[1]
    if fc0 != 0x08 and fc0 != 0x88:        # data / QoS data
        continue
    if fc1 & 0x40:                          # protected: not the four-way
        continue
    hlen = 24 + (2 if fc0 == 0x88 else 0)
    if (fc1 & 0x03) == 0x03:
        hlen += 6
    if len(m) < hlen + 8 + 99:
        continue
    llc = m[hlen:hlen + 8]
    if llc[0] != 0xaa or llc[6] != 0x88 or llc[7] != 0x8e:
        continue
    e = m[hlen + 8:]
    if e[1] != 3:                           # EAPOL-Key
        continue
    body = (e[2] << 8) | e[3]
    e = e[:4 + body]
    to_ds = bool(fc1 & 0x01)
    frames.append(dict(eapol=e, to_ds=to_ds,
                       a1=m[4:10], a2=m[10:16], a3=m[16:22]))

# The first four distinct EAPOL-Key frames of one exchange, in order. A
# retransmission repeats a key-info/replay pair, so drop duplicates.
seen, four = set(), []
for f in frames:
    e = f['eapol']
    key = (e[5], e[6], bytes(e[9:17]))
    if key in seen:
        continue
    seen.add(key)
    four.append(f)
    if len(four) == 4:
        break
if len(four) != 4:
    raise SystemExit('wanted four EAPOL-Key frames, got %d' % len(four))

# Direction tells us which is which without reading the key-info bits - which
# is deliberate, because the key-info interpretation is one of the things
# under test.
if [f['to_ds'] for f in four] != [False, True, False, True]:
    raise SystemExit('the four frames are not AP,STA,AP,STA: %s'
                     % [f['to_ds'] for f in four])
aa = four[0]['a2']        # the authenticator: addr2 of a from-DS frame
spa = four[1]['a2']       # the supplicant: addr2 of a to-DS frame


def carr(b, indent='    '):
    toks = ['0x%02x' % x for x in b]
    lines, cur = [], indent
    for t in toks:
        if len(cur) + len(t) + 2 > 76:
            lines.append(cur.rstrip())
            cur = indent
        cur += t + ', '
    lines.append(cur.rstrip().rstrip(','))
    return '\n'.join(lines)


with open(out_path, 'w') as f:
    f.write('''/* eapol_kernel_vectors.h - a real WPA2-PSK four-way, off the air.
 *
 * GENERATED, do not edit by hand. See tests/eapol_capture_vectors.sh: two
 * mac80211_hwsim radios, hostapd and wpa_supplicant, and the four EAPOL-Key
 * frames they actually exchanged. No hardware.
 *
 * WHY. tests/supplicant_selftest.cpp drives src/sta/Supplicant.h against a
 * fixture authenticator written by the same author from the same reading of
 * the same clause. That pins the state machine and cannot pin the PRF, the
 * EAPOL-Key MIC or the GTK KDE layout - a shared misreading is invisible to
 * it, the same way a zero CCM nonce flags octet is invisible to a CCMP
 * vector generator written from the same reading.
 *
 * The PTK below is the one WPA_SUPPLICANT derived and logged; the GTK is the
 * one HOSTAPD generated and logged. Nothing here was computed by this
 * repository.
 *
 * KEY MATERIAL IS FROM A THROWAWAY LAB PSK on virtual radios and protects
 * nothing.
 */
#ifndef DEVOURER_TEST_EAPOL_KERNEL_VECTORS_H
#define DEVOURER_TEST_EAPOL_KERNEL_VECTORS_H

#include <cstddef>
#include <cstdint>

namespace devourer {
namespace test {

''')
    f.write('static const char kHostapdPassphrase[] = "%s";\n' % psk)
    f.write('static const char kHostapdSsid[] = "%s";\n\n' % ssid)
    f.write('/* The authenticator (the AP) and the supplicant (the station). */\n')
    f.write('static const uint8_t kEapolAa[6] = {\n%s\n};\n\n' % carr(aa))
    f.write('static const uint8_t kEapolSpa[6] = {\n%s\n};\n\n' % carr(spa))
    f.write('/* wpa_supplicant derived and logged this. */\n')
    f.write('static const uint8_t kSupplicantPtk[48] = {\n%s\n};\n\n'
            % carr(bytes.fromhex(ptk_hex)))
    f.write('/* hostapd generated and logged this. */\n')
    f.write('static const uint8_t kHostapdGtk[16] = {\n%s\n};\n\n'
            % carr(bytes.fromhex(gtk_hex)))
    for i, fr in enumerate(four, 1):
        f.write('/* message %d, %s */\n'
                % (i, 'station -> AP' if fr['to_ds'] else 'AP -> station'))
        f.write('static const uint8_t kEapolMsg%d[] = {\n%s\n};\n\n'
                % (i, carr(fr['eapol'])))
    f.write('''struct EapolFrameVector {
  const uint8_t* eapol;
  size_t len;
};

static const EapolFrameVector kEapolFourWay[4] = {
''')
    for i, fr in enumerate(four, 1):
        f.write('    {kEapolMsg%d, sizeof kEapolMsg%d},\n' % (i, i))
    f.write('''};

}  // namespace test
}  // namespace devourer

#endif /* DEVOURER_TEST_EAPOL_KERNEL_VECTORS_H */
''')
print('wrote a four-way (%d bytes of EAPOL) -> %s'
      % (sum(len(f['eapol']) for f in four), out_path))
