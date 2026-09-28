"""Send one UDP datagram per 802.11 user priority.

`ping -Q` does not give exact TID control: the TOS byte reaches
cfg80211_classify8021d() only when skb->priority has not already been set, and
setting IP_TOS sets sk_priority through ip_tos2prio[] first.  SO_PRIORITY in
the range 256..263 is special-cased by that same function as "user priority
0..7, use it verbatim", which is the only way to address all eight TIDs.
"""
import socket
import sys
import time

dst = sys.argv[1]
tag = (sys.argv[2] if len(sys.argv) > 2 else "").encode()
for tid in range(8):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_PRIORITY, 256 + tid)
    # A payload that says which TID it was sent with, so a decrypted vector can
    # be checked against the TID in the header it was encrypted under.
    s.sendto(b"devourer-ccmp-vector tid=%d %s" % (tid, tag), (dst, 9999))
    s.close()
    time.sleep(0.15)
print("sent 8 datagrams, one per TID")
