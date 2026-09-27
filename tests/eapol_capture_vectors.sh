#!/bin/sh
# Regenerate tests/eapol_kernel_vectors.h from a mac80211_hwsim rig.
#
# The sibling of tests/ccmp_capture_vectors.sh, and for the same reason. That
# one made the kernel encrypt CCMP frames so our framing could be checked
# against something that did not share our reading of the standard. This one
# makes hostapd and wpa_supplicant run a four-way handshake so the PRF, the
# PTK derivation, the EAPOL-Key MIC and the GTK KDE layout can be checked the
# same way.
#
# NO HARDWARE. Two virtual radios, so the bench is untouched. Needs root and
# hostapd, wpa_supplicant and tcpdump on PATH.
#
# -K is what makes both daemons log their derived keys. Without it there is
# nothing to compare against and the capture is just four opaque frames.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
WORK=${WORK:-$(mktemp -d /tmp/eapolvec.XXXXXX)}
NS=eapolvec_sta
PSK=eapolvectors123
SSID=eapolvec

[ "$(id -u)" = 0 ] || { echo "this needs root"; exit 1; }

cleanup() {
    # BY PID, and only the processes this run started. A name or command-line
    # match would also hit somebody else's capture or hostapd, and the host's
    # own wpa_supplicant: `ip netns exec` does not change the PID namespace.
    for p in ${CAP_PID:-} ${HOSTAPD_PID:-} ${WPA_PID:-}; do
        kill "$p" 2>/dev/null || true
    done
    [ "${NS_OURS:-no}" = yes ] && ip netns del "$NS" 2>/dev/null || true
    [ "${HWSIM_OURS:-no}" = yes ] && rmmod mac80211_hwsim 2>/dev/null || true
}
trap cleanup EXIT

echo "--- two virtual radios"
# REFUSE, do not reuse: an already-loaded hwsim is somebody else's rig (or a
# leftover), and if it is in use the rmmod fails, modprobe is a no-op, and
# "the two highest phys" can then include a REAL adapter - which would be
# moved into the namespace and destroyed with it.
if [ -d /sys/module/mac80211_hwsim ]; then
    echo "mac80211_hwsim is already loaded - refusing (unload it if it is yours)"
    exit 1
fi
if ip netns list 2>/dev/null | awk '{print $1}' | grep -qx "$NS"; then
    echo "netns $NS already exists - refusing to use or delete it"
    exit 1
fi
PHYS_BEFORE=$(ls /sys/class/ieee80211 2>/dev/null || true)
modprobe mac80211_hwsim radios=2
HWSIM_OURS=yes
sleep 2
# The phys hwsim just CREATED - the set difference against the listing
# before the modprobe - and there must be exactly two.
PHYS=$(ls /sys/class/ieee80211 | grep -vxF "$PHYS_BEFORE" | sed 's/phy//' | sort -n)
[ "$(echo "$PHYS" | grep -c .)" = 2 ] || { echo "expected 2 new hwsim phys, got: $PHYS"; exit 1; }
AP_PHY=phy$(echo "$PHYS" | head -1)
STA_PHY=phy$(echo "$PHYS" | tail -1)
if_for_phy() {
    for l in /sys/class/net/*/phy80211; do
        [ -e "$l" ] || continue
        if [ "$(basename "$(readlink -f "$l")")" = "$1" ]; then
            basename "$(dirname "$l")"
            return
        fi
    done
}
AP_IF=$(if_for_phy "$AP_PHY")
STA_IF=$(if_for_phy "$STA_PHY")
[ -n "$AP_IF" ] && [ -n "$STA_IF" ] || { echo "no hwsim interfaces"; exit 1; }
echo "    AP  $AP_PHY/$AP_IF   STA $STA_PHY/$STA_IF"
nmcli dev set "$AP_IF" managed no 2>/dev/null || true
nmcli dev set "$STA_IF" managed no 2>/dev/null || true

ip netns add "$NS"
NS_OURS=yes
iw phy "$STA_PHY" set netns name "$NS"
ip netns exec "$NS" ip link set "$STA_IF" up

cat > "$WORK/hostapd.conf" <<EOF
interface=$AP_IF
driver=nl80211
ssid=$SSID
hw_mode=g
channel=1
auth_algs=1
wpa=2
wpa_key_mgmt=WPA-PSK
wpa_pairwise=CCMP
rsn_pairwise=CCMP
wpa_passphrase=$PSK
EOF
cat > "$WORK/wpa.conf" <<EOF
network={
	ssid="$SSID"
	psk="$PSK"
	key_mgmt=WPA-PSK
	proto=RSN
	pairwise=CCMP
	group=CCMP
}
EOF

echo "--- capture, AP, station"
ip link set hwsim0 up
ip link set "$AP_IF" up
setsid tcpdump -i hwsim0 -s 0 -w "$WORK/cap.pcap" -U >"$WORK/tcpdump.log" 2>&1 &
CAP_PID=$!
sleep 2
# $! is the daemon itself: a background child of a non-interactive shell is
# not a process-group leader, so setsid (and `ip netns exec`) exec in place.
setsid hostapd -dd -K -t "$WORK/hostapd.conf" >"$WORK/hostapd.log" 2>&1 &
HOSTAPD_PID=$!
sleep 4
grep -q AP-ENABLED "$WORK/hostapd.log" || { echo "hostapd did not come up"; exit 1; }
ip netns exec "$NS" setsid wpa_supplicant -i "$STA_IF" -c "$WORK/wpa.conf" \
    -dd -K -t -f "$WORK/wpa.log" >/dev/null 2>&1 &
WPA_PID=$!
i=0
while [ $i -lt 30 ]; do
    grep -q CTRL-EVENT-CONNECTED "$WORK/wpa.log" 2>/dev/null && break
    i=$((i + 1)); sleep 1
done
grep -q CTRL-EVENT-CONNECTED "$WORK/wpa.log" || { echo "no association"; exit 1; }
sleep 1
kill "$CAP_PID" 2>/dev/null || true
sleep 1

PTK=$(grep -m1 'WPA: PTK - hexdump' "$WORK/wpa.log" | sed 's/.*): //; s/ //g')
[ ${#PTK} = 96 ] || { echo "could not read wpa_supplicant's 48-byte PTK"; exit 1; }
# THE GTK COMES FROM THE SUPPLICANT'S LOG, NOT THE AUTHENTICATOR'S.
#
# hostapd generates a GTK when it starts and then throws it away - "WPA:
# Re-initialize GMK/Counter on first station" - so its log carries two, and
# the first `GTK - hexdump(len=16)` line is the one that never reached the
# air. Taking it produced a vector file that failed on the first run, which is
# the good outcome and cost an hour.
#
# wpa_supplicant's "WPA: Group Key" line is the key it took OUT of message 3
# and installed. That is unambiguous, and it is the receiving end's own record
# - which is what our extraction has to agree with.
GTK=$(grep -m1 'WPA: Group Key - hexdump(len=16)' "$WORK/wpa.log" \
        | sed 's/.*): //; s/ //g')
[ ${#GTK} = 32 ] || { echo "could not read the installed 16-byte GTK"; exit 1; }
echo "    associated; wpa_supplicant's PTK and the installed GTK captured"

python3 "$HERE/eapol_extract_vectors.py" "$WORK/cap.pcap" "$PTK" "$GTK" \
    "$PSK" "$SSID" "$HERE/eapol_kernel_vectors.h"

BUILD=${BUILD:-$HERE/../build}
if [ -f "$BUILD/CMakeCache.txt" ]; then
    echo "--- rebuilding and running SupplicantSelftest against the new vectors"
    cmake --build "$BUILD" --target SupplicantSelftest >/dev/null 2>&1 \
        || { echo "FAIL: SupplicantSelftest does not build"; exit 1; }
    "$BUILD/SupplicantSelftest" || { echo "FAIL: the new vectors do not verify"; exit 1; }
else
    echo "--- no build tree at $BUILD; build and run SupplicantSelftest yourself"
fi
echo "--- done"
