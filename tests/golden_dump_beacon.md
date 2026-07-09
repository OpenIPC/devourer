# Golden dump: rtw88_8822c IBSS beacon-enable vs devourer (8822C, ch6 capture)
# Captured 20738 URBs; 53 beacon re-downloads in ~5s (~94ms = beacon interval).
## Kernel does (that devourer does NOT):
1. Re-downloads the beacon EVERY beacon interval (~94ms) — FW/driver-orchestrated, not static HW-TBTT.
2. Sends H2C RSVD_PAGE (REG_HMEBOX0 0x1d0 = 0x690c0100, cmd 0x00) telling the FW the rsvd-page locations.
3. BCNQ_BDNY (0x0424) = 0x9207 -> boundary page 0x207 (519); devourer computes rsvd_boundary=1938.
4. BCN_CTRL (0x0550) cycles 0x10/0x14/0x18/0x1c across the download bracket (devourer used static 0x1e).
5. Downloads MULTIPLE rsvd pages (beacon+probe-rsp+null) at heads 0x207/0x287/0xa287/0xab87 (0x0204|BIT15).
## Download bracket (recurring, per beacon interval):
   0204=0080 ; 0101(CR+1)=01 ; 0550=14 ; 0204=9287(arm head+valid) ; [bulk beacon] ; 0550=14 ; 0101=00 ; 0204=0080
