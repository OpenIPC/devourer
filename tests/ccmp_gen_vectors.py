#!/usr/bin/env python3
"""Generate the CCMP known-answer vectors in tests/ccmp_vectors.h.

WHAT THESE VECTORS ACTUALLY PIN — read this before trusting them.

The ciphertext and MIC come from python-cryptography's AESCCM, which is a
genuinely third-party cipher. The 802.11 FRAMING below, however, is a
transcription of the same rules src/sta/Ccmp.h implements, written by the same
author from the same reading of 802.11-2016 12.5.3.3. It is NOT an
independent check of the framing: the masks, their order and even the idioms
match the C line for line, so a misreading shared by both passes here.

So, honestly:

  * These vectors DO pin the cipher plumbing - that the AAD and nonce this
    module builds reach AES-CCM intact and unmutated, and that ciphertext,
    MIC, PN and header packing round-trip byte-exactly. A regression in any of
    that turns the cell red.
  * These vectors DO NOT independently verify the framing RULES. A shared
    misreading of the standard passes. What pins the rules is the direct
    assertions in tests/ccmp_selftest.cpp (one rule at a time, written against
    the standard's text) and tests/ccmp_kernel_vectors.h (frames the Linux
    kernel encrypted, at all eight TIDs, with the MIC as the oracle).

THE REMAINING GAP. The IEEE 802.11-2016 Annex J.4 CCMP test vector - an
answer computed by people who were not reading this code, rather than an
interop reference - is not in the tree, and the selftest is shaped to take
it. It must be copied from the standard, never transcribed from memory: a
known-answer test whose answer is remembered rather than sourced is a second
guess by the same author, and if it disagreed the natural move would be to
"correct" it until it matched, which is how a wrong implementation acquires a
passing test. hostapd's wlantest vectors are the next-best source.

Regenerate:  python3 tests/ccmp_gen_vectors.py > tests/ccmp_vectors.h
Verify:      python3 tests/ccmp_gen_vectors.py --check
             (regenerates in memory and byte-compares with the checked-in
             header; exit 0 match, 1 mismatch, 77 = python-cryptography
             missing, which ctest reports as skipped. Registered as the
             ccmp_vectors_generated ctest cell.)
Deterministic - every input below is fixed.
"""
import io
import os
import sys

CHECK = '--check' in sys.argv[1:]
TARGET = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      'ccmp_vectors.h')

try:
    from cryptography.hazmat.primitives.ciphers.aead import AESCCM
except ImportError:
    if CHECK:
        print('ccmp_gen_vectors --check: python-cryptography is not '
              'installed; skipped', file=sys.stderr)
        sys.exit(77)
    sys.exit("needs python-cryptography (pip install cryptography)")

if CHECK:
    _stdout = sys.stdout
    sys.stdout = io.StringIO()


def is_qos(fc0):
    return (fc0 & 0x8C) == 0x88


def hdr_len_of(hdr):
    # Mirrors src/sta/Dot11.h data_hdr_len(), INCLUDING the HT Control field a
    # QoS frame with the Order bit carries. Being a mirror, it shares any
    # omission with the C - which is why the qos_order_htc case below exists
    # and why the kernel vectors, not this file, are the independent check.
    n = 24
    if (hdr[1] & 0x03) == 0x03:
        n += 6
    if is_qos(hdr[0]):
        n += 2
        if hdr[1] & 0x80:
            n += 4
    return n


def ccmp_aad(hdr):
    """802.11-2016 12.5.3.3.3."""
    four = (hdr[1] & 0x03) == 0x03
    qos = is_qos(hdr[0])
    mgmt = (hdr[0] & 0x0C) == 0x00
    fc = hdr[0] | (hdr[1] << 8)
    if not mgmt:
        fc &= ~0x0070          # subtype masked only for non-management
    fc &= ~0x0800              # retry
    fc &= ~0x1000              # pwr mgmt
    fc &= ~0x2000              # more data
    if qos:
        fc &= ~0x8000          # +HTC/Order, masked for QoS data only
    fc |= 0x4000               # protected
    seq = (hdr[22] | (hdr[23] << 8)) & 0x000F
    aad = bytes([fc & 0xFF, fc >> 8]) + bytes(hdr[4:22]) + \
        bytes([seq & 0xFF, seq >> 8])
    if four:
        aad += bytes(hdr[24:30])
    if qos:
        q = 30 if four else 24
        aad += bytes([hdr[q] & 0x0F, 0x00])
    return aad


def ccmp_nonce(hdr, a2, pn):
    """802.11-2016 12.5.3.3.4: Nonce Flags | A2 | PN(6, big-endian).

    The flags octet is Priority (b0..b3) | Management (b4) - NOT zero. A zero
    octet here and in src/sta/Ccmp.h would agree with each other and keep
    these vectors green: independence of implementation (python-cryptography
    vs OpenSSL) is not independence of interpretation.
    """
    four_addr = (hdr[1] & 0x03) == 0x03
    flags = 0
    if is_qos(hdr[0]):
        flags = hdr[30 if four_addr else 24] & 0x0F
    if (hdr[0] & 0x0C) == 0x00:          # type 0 = management
        flags |= 0x10
    return bytes([flags]) + bytes(a2) + bytes((pn >> (8 * (5 - i))) & 0xFF
                                              for i in range(6))


def ccmp_hdr(pn, key_id):
    return bytes([
        pn & 0xFF, (pn >> 8) & 0xFF, 0x00,
        0x20 | ((key_id & 3) << 6),
        (pn >> 16) & 0xFF, (pn >> 24) & 0xFF,
        (pn >> 32) & 0xFF, (pn >> 40) & 0xFF,
    ])


def c_array(name, data):
    body = ',\n    '.join(
        ', '.join('0x%02x' % b for b in data[i:i + 12])
        for i in range(0, len(data), 12))
    return 'static const uint8_t %s[%d] = {\n    %s\n};\n' % (
        name, len(data), body)


# Each header below is a REAL frame of its shape - the right length for what
# its frame control claims. A "QoS" header of 24 bytes would put the CCMP
# header where the QoS Control field belongs (a frame that cannot exist on
# air), and a to-DS+from-DS header is a 4-address frame with a 4-address AAD.
CASES = [
    # 3-address data, from-DS. The baseline shape both AP harnesses air.
    ('basic', bytes(range(0x10, 0x20)),
     bytes.fromhex('02424475d600'), 0x000000000001, 0,
     bytes.fromhex('0842000002aabbccddee02424475d60002424475d6000000'),
     bytes.fromhex('aaaa030000000800') + b'devourer station mode'),

    # RETRY SET, plus pwr-mgmt and more-data, on a plain 3-address to-DS frame.
    # Retry must actually be set here, or no vector can detect a missing
    # Retry mask - the classic slip. fc1 = 0x39: to-DS, retry, pwr-mgmt,
    # more-data. Subtype bits set in fc0 too.
    ('masked_fc', bytes.fromhex('c97c1f67ce371185514a8a19f2bdd52f'),
     bytes.fromhex('001122334455'), 0x0b5039768834, 2,
     bytes.fromhex('7839') + bytes.fromhex('1234') +
     bytes.fromhex('001122334455') + bytes.fromhex('66778899aabb') +
     bytes.fromhex('ccddeeff0011') + bytes.fromhex('3412'),
     b'\x01\x02\x03\x04'),

    # Non-zero fragment number: the AAD keeps it while masking the sequence
    # number, the half of that rule most likely to be dropped.
    ('fragment', bytes(16),
     bytes.fromhex('020000000001'), 0xffffffffffff, 1,
     bytes.fromhex('0842000002000000000202000000000102000000000175f0'),
     b'x' * 64),

    # A REAL QoS data MPDU: 26-byte header with the QoS Control field at bytes
    # 24-25, TID 5. The AAD gains the TID octets. A station omitting them
    # against a peer that includes them produces a MIC failure with no
    # diagnostic at either end, and a station's data plane is QoS against any
    # 802.11n+ AP - so this is the shape a station's data plane depends on.
    ('qos_tid5', bytes.fromhex('0f0e0d0c0b0a09080706050403020100'),
     bytes.fromhex('aabbccddeeff'), 0x000000abcdef, 0,
     bytes.fromhex('8841000002424475d600aabbccddeeff02424475d6002010') +
     bytes.fromhex('0500'),
     bytes.fromhex('aaaa030000000806') + b'qos-tid-5-body'),

    # QoS WITH THE ORDER BIT (+HTC): fc1 = 0xC1, so an HT Control field
    # follows the QoS Control and data_hdr_len() reports 30. The AAD must MASK
    # bit 15 here - it does not on a non-QoS frame, where the same bit is the
    # strictly-ordered service class. Without the mask every +HTC frame fails
    # its MIC against any conforming peer, and no other case here sets the
    # bit.
    ('qos_order_htc', bytes.fromhex('101112131415161718191a1b1c1d1e1f'),
     bytes.fromhex('aabbccddeeff'), 0x0000000000aa, 5,
     bytes.fromhex('88c1000002424475d600aabbccddeeff02424475d6003050') +
     bytes.fromhex('0300') + bytes.fromhex('00000000'),
     bytes.fromhex('aaaa030000000800') + b'htc-ordered'),

    # 4-address QoS: the AAD gains A4 as well as the TID, 30 octets in all.
    # Nothing in the tree builds one yet, which is exactly why it is here -
    # the AAD length branch is otherwise dead code nobody would notice broken.
    ('four_addr_qos', bytes.fromhex('00112233445566778899aabbccddeeff'),
     bytes.fromhex('020000000011'), 0x0000000000ff, 3,
     bytes.fromhex('8803000002000000001102000000001102000000002201f0') +
     bytes.fromhex('020000000033') + bytes.fromhex('0700'),
     b'four-address-qos'),
]

print('/* GENERATED by tests/ccmp_gen_vectors.py - do not edit.')
print(' *')
print(' * CCMP known-answer vectors. The ciphertext and MIC come from')
print(" * python-cryptography's AESCCM; the 802.11 framing is transcribed in")
print(' * that script from the same reading of the standard as src/sta/Ccmp.h,')
print(' * so these pin the CIPHER PLUMBING and NOT the framing rules. The')
print(' * script header explains what that does and does not buy; the direct')
print(' * assertions in tests/ccmp_selftest.cpp are what pin the rules. */')
print('#ifndef DEVOURER_TESTS_CCMP_VECTORS_H')
print('#define DEVOURER_TESTS_CCMP_VECTORS_H')
print()
print('#include <cstdint>')
print('#include <cstddef>')
print()
print('struct CcmpVector {')
print('  const char* name;')
print('  const uint8_t* tk;')
print('  const uint8_t* a2;')
print('  uint64_t pn;')
print('  uint8_t key_id;')
print('  const uint8_t* hdr;    size_t hdr_len;')
print('  const uint8_t* plain;  size_t plain_len;')
print('  const uint8_t* mpdu;   size_t mpdu_len;   /* hdr|ccmp|ct|mic */')
print('  size_t aad_len;        /* 22, +6 four-addr, +2 QoS */')
print('};')
print()

entries = []
for label, tk, a2, pn, kid, hdr, plain in CASES:
    hl = hdr_len_of(hdr)
    assert len(hdr) == hl, (label, len(hdr), hl)
    aad = ccmp_aad(hdr)
    nonce = ccmp_nonce(hdr, a2, pn)
    blob = AESCCM(tk, tag_length=8).encrypt(nonce, plain, aad)
    prot = bytearray(hdr)
    prot[1] |= 0x40
    mpdu = bytes(prot) + ccmp_hdr(pn, kid) + blob
    print(c_array('kTk_' + label, tk))
    print(c_array('kA2_' + label, a2))
    print(c_array('kHdr_' + label, hdr))
    print(c_array('kPlain_' + label, plain))
    print(c_array('kMpdu_' + label, mpdu))
    entries.append('  {"%s", kTk_%s, kA2_%s, 0x%012xULL, %d,\n'
                   '   kHdr_%s, sizeof kHdr_%s,\n'
                   '   kPlain_%s, sizeof kPlain_%s,\n'
                   '   kMpdu_%s, sizeof kMpdu_%s, %d},'
                   % (label, label, label, pn, kid, label, label, label,
                      label, label, label, len(aad)))

print('static const CcmpVector kCcmpVectors[] = {')
print('\n'.join(entries))
print('};')
print('static const size_t kCcmpVectorCount =')
print('    sizeof kCcmpVectors / sizeof kCcmpVectors[0];')
print()
print('#endif /* DEVOURER_TESTS_CCMP_VECTORS_H */')

if CHECK:
    got = sys.stdout.getvalue()
    sys.stdout = _stdout
    with open(TARGET, newline='') as f:
        # A Windows checkout may have converted line endings; the content is
        # what is compared, not the platform's newline convention.
        want = f.read().replace('\r\n', '\n')
    if got != want:
        g, w = got.splitlines(), want.splitlines()
        first = next((i for i in range(min(len(g), len(w))) if g[i] != w[i]),
                     min(len(g), len(w)))
        print('ccmp_gen_vectors --check: tests/ccmp_vectors.h does NOT match '
              'the generator (first difference at line %d)' % (first + 1),
              file=sys.stderr)
        sys.exit(1)
    print('ccmp_gen_vectors --check: tests/ccmp_vectors.h reproduced byte '
          'for byte')
