// ap_wpa2.cpp — devourer as a WPA2-PSK AP. STATUS: the RSN 4-way handshake
// COMPLETES with a real Linux station — wpa_supplicant reports "WPA: Key
// negotiation completed ... [PTK=CCMP GTK=CCMP]" + CTRL-EVENT-CONNECTED, and the
// AP logs msg2-MIC-verified -> msg3 -> msg4-OK. This is the deepest driver-protocol
// milestone (WPA2 key negotiation). It needs no CCMP (EAPOL-Key frames are
// cleartext, MIC-protected), so it works WITHOUT the hardware crypto engine — the
// crypto (PBKDF2/PRF/HMAC-MIC/AES-key-wrap) is openssl in userspace. Association
// reuses the open-AP path (probe/auth/assoc); the beacon/probe/assoc carry an RSN
// IE (WPA2-PSK-CCMP); after assoc the AP runs the authenticator: msg1(ANonce) ->
// msg2(SNonce,MIC) -> derive PTK, verify MIC -> msg3(GTK,MIC) -> msg4.
//
// ENCRYPTED DATA plane too: after the handshake the AP decrypts the station's CCMP
// data frames (software AES-CCM with the TK = PTK[32:48]) and answers ARP + ICMP +
// DHCP ENCRYPTED, so a real station leases 192.168.99.2 over encrypted DHCP (dhcpcd:
// "leased") AND pings the AP over WPA2/CCMP at 0% loss (~2.5 ms RTT). The station
// runs hardware CCMP, the AP software CCMP — they interoperate. So this is a COMPLETE
// zero-config WPA2-PSK AP: associate -> 4-way -> encrypted DHCP -> encrypted IP.
// (HW CCMP offload would need porting the J3 security TX/RX descriptor fields.)
//
// Details that mattered: (1) msg3 key-data pad is 0xDD then 0x00s (a 2nd 0xDD
// mis-parses as a KDE and the station rejects msg3); (2) a prior WRONG_KEY failure
// temp-disables the SSID in wpa_supplicant — cold-cycle the station for a clean run;
// (3) CCMP AAD masks FC subtype/retry/pm/md + sets protected, and masks the seq
// number (keep frag); the nonce is 0|A2|PN(6, big-endian).
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/ap_wpa2.cpp \
//   examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lcrypto -lpthread -o build/ap_wpa2
// Run: sudo DEVOURER_VID=0x2357 DEVOURER_PID=0x012d DEVOURER_CHANNEL=6 \
//   DEVOURER_WPA2_PSK=devourer123 DEVOURER_BCN_TU=25 DEVOURER_TX_WITH_RX=thread \
//   build/ap_wpa2 [sec]
#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <unistd.h>
#include <libusb.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/rand.h>
#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "SelectedChannel.h"
#include "TxMode.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

static const uint8_t kBssid[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x00};
static const char* kSsid = "devourerAP";
static IRtlDevice* g_dev = nullptr;
static std::vector<uint8_t> g_rt;
static uint8_t g_chan = 6;
static const char* g_psk = "devourer123";
static std::atomic<uint64_t> g_sent{0};
static std::mutex g_q_mu;
static std::vector<std::vector<uint8_t>> g_q;

// WPA2-PSK / CCMP RSN IE (group=CCMP, pairwise=CCMP, akm=PSK).
static const uint8_t kRsn[] = {0x30, 0x14, 0x01,0x00,
    0x00,0x0f,0xac,0x04, 0x01,0x00, 0x00,0x0f,0xac,0x04,
    0x01,0x00, 0x00,0x0f,0xac,0x02, 0x00,0x00};

// Per-station 4-way state (single client for the demo).
static uint8_t g_anonce[32], g_snonce[32], g_ptk[48], g_gtk[16];
static uint8_t g_replay[8];
static uint8_t g_sta[6];
static int g_state = 0;  // 0 idle, 1 sent msg1, 2 done

static void enqueue(std::vector<uint8_t> mpdu) {
  std::vector<uint8_t> f; f.reserve(g_rt.size() + mpdu.size());
  f.insert(f.end(), g_rt.begin(), g_rt.end());
  f.insert(f.end(), mpdu.begin(), mpdu.end());
  std::lock_guard<std::mutex> lk(g_q_mu);
  if (g_q.size() < 128) g_q.push_back(std::move(f));
}
static std::vector<uint8_t> mgmt_hdr(uint8_t fc, const uint8_t* sta) {
  return {fc,0,0,0, sta[0],sta[1],sta[2],sta[3],sta[4],sta[5],
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5], 0,0};
}
static void append_ies(std::vector<uint8_t>& m, bool ssid) {
  if (ssid) { m.insert(m.end(), {0x00,(uint8_t)strlen(kSsid)});
    m.insert(m.end(), kSsid, kSsid+strlen(kSsid)); }
  m.insert(m.end(), {0x01,0x08,0x82,0x84,0x8b,0x96,0x24,0x30,0x48,0x6c});
  m.insert(m.end(), {0x03,0x01,g_chan});
  m.insert(m.end(), kRsn, kRsn+sizeof(kRsn));           // RSN IE -> advertise WPA2
}

// --- WPA2 crypto (openssl) --------------------------------------------------
static void prf(const uint8_t* key, int klen, const char* label,
                const uint8_t* data, int dlen, uint8_t* out, int olen) {
  int ll = (int)strlen(label);
  for (int i = 0, gen = 0; gen < olen; ++i, gen += 20) {
    std::vector<uint8_t> b(ll + 1 + dlen + 1);
    memcpy(b.data(), label, ll); b[ll] = 0;
    memcpy(b.data()+ll+1, data, dlen); b[ll+1+dlen] = (uint8_t)i;
    unsigned int l; uint8_t d[20];
    HMAC(EVP_sha1(), key, klen, b.data(), b.size(), d, &l);
    int c = (olen-gen < 20) ? olen-gen : 20; memcpy(out+gen, d, c);
  }
}
static void compute_ptk() {
  const uint8_t *aa = kBssid, *sa = g_sta;
  uint8_t b[76]; int p = 0;
  const uint8_t* mn = memcmp(aa,sa,6) < 0 ? aa : sa;
  const uint8_t* mx = memcmp(aa,sa,6) < 0 ? sa : aa;
  memcpy(b+p, mn, 6); p+=6; memcpy(b+p, mx, 6); p+=6;
  const uint8_t* nn = memcmp(g_anonce,g_snonce,32) < 0 ? g_anonce : g_snonce;
  const uint8_t* nx = memcmp(g_anonce,g_snonce,32) < 0 ? g_snonce : g_anonce;
  memcpy(b+p, nn, 32); p+=32; memcpy(b+p, nx, 32); p+=32;
  uint8_t pmk[32];
  PKCS5_PBKDF2_HMAC(g_psk, strlen(g_psk), (const unsigned char*)kSsid,
                    strlen(kSsid), 4096, EVP_sha1(), 32, pmk);
  prf(pmk, 32, "Pairwise key expansion", b, p, g_ptk, 48);
}
// MIC over the EAPOL frame with the MIC field (offset 81, 16 bytes) zeroed.
static void set_mic(std::vector<uint8_t>& e) {
  memset(e.data()+81, 0, 16);
  unsigned int l; uint8_t d[20];
  HMAC(EVP_sha1(), g_ptk, 16 /*KCK*/, e.data(), e.size(), d, &l);
  memcpy(e.data()+81, d, 16);
}
static bool check_mic(const uint8_t* e, int len) {
  std::vector<uint8_t> t(e, e+len);
  uint8_t got[16]; memcpy(got, t.data()+81, 16);
  memset(t.data()+81, 0, 16);
  unsigned int l; uint8_t d[20];
  HMAC(EVP_sha1(), g_ptk, 16, t.data(), t.size(), d, &l);
  return memcmp(got, d, 16) == 0;
}
// AES key wrap (RFC 3394) with the KEK (PTK bytes 16..31), for msg3 key data.
static int aes_wrap(const uint8_t* kek, const uint8_t* in, int inlen, uint8_t* out) {
  EVP_CIPHER_CTX* c = EVP_CIPHER_CTX_new();
  EVP_CIPHER_CTX_set_flags(c, EVP_CIPHER_CTX_FLAG_WRAP_ALLOW);
  EVP_EncryptInit_ex(c, EVP_aes_128_wrap(), NULL, kek, NULL);
  int ol = 0, tmp = 0;
  EVP_EncryptUpdate(c, out, &ol, in, inlen);
  EVP_EncryptFinal_ex(c, out+ol, &tmp); ol += tmp;
  EVP_CIPHER_CTX_free(c);
  return ol;
}

// Build an EAPOL-Key data frame (from-DS) to the station.
static std::vector<uint8_t> eapol_frame(uint16_t keyinfo, const uint8_t* nonce,
                                        const uint8_t* keydata, int kdlen, bool mic) {
  std::vector<uint8_t> e(99, 0);
  e[0]=2; e[1]=3;                                       // EAPOL v2, type Key
  int blen = 95 + kdlen; e[2]=blen>>8; e[3]=blen&0xff;
  e[4]=2;                                               // RSN key descriptor
  e[5]=keyinfo>>8; e[6]=keyinfo&0xff;
  e[7]=0; e[8]=16;                                      // key length 16
  memcpy(e.data()+9, g_replay, 8);
  if (nonce) memcpy(e.data()+17, nonce, 32);
  e[97]=kdlen>>8; e[98]=kdlen&0xff;
  if (keydata && kdlen) e.insert(e.end(), keydata, keydata+kdlen);
  if (mic) set_mic(e);
  // wrap in 802.11 data (from-DS) + LLC/SNAP ethertype 0x888e
  std::vector<uint8_t> m = {0x08,0x02,0,0,
      g_sta[0],g_sta[1],g_sta[2],g_sta[3],g_sta[4],g_sta[5],
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5], 0,0,
      0xaa,0xaa,0x03,0x00,0x00,0x00, 0x88,0x8e};
  m.insert(m.end(), e.begin(), e.end());
  return m;
}
static void send_msg1() {
  RAND_bytes(g_anonce, 32);
  for (int i=7;i>=0;--i) if (++g_replay[i]) break;      // bump replay counter
  enqueue(eapol_frame(0x008a, g_anonce, nullptr, 0, false));  // ver2|pair|ack
  g_state = 1;
  fprintf(stderr, "  WPA2: sent msg1 (ANonce) to %02x:%02x:%02x:%02x:%02x:%02x\n",
          g_sta[0],g_sta[1],g_sta[2],g_sta[3],g_sta[4],g_sta[5]);
}
static void send_msg3() {
  RAND_bytes(g_gtk, 16);
  // key data = RSN IE + GTK KDE, padded to /8, then AES-wrapped with the KEK.
  std::vector<uint8_t> kd(kRsn, kRsn+sizeof(kRsn));
  uint8_t gtkkde[24] = {0xdd,0x16,0x00,0x0f,0xac,0x01,0x01,0x00};
  memcpy(gtkkde+8, g_gtk, 16);
  kd.insert(kd.end(), gtkkde, gtkkde+24);
  if (kd.size() % 8) {                                  // 802.11i pad: 0xDD then 0x00s
    kd.push_back(0xdd);
    while (kd.size() % 8) kd.push_back(0x00);
  }
  std::vector<uint8_t> wrapped(kd.size()+8);
  int wl = aes_wrap(g_ptk+16, kd.data(), kd.size(), wrapped.data());
  for (int i=7;i>=0;--i) if (++g_replay[i]) break;
  enqueue(eapol_frame(0x13ca, g_anonce, wrapped.data(), wl, true));  // install|ack|mic|secure|enc
  fprintf(stderr, "  WPA2: sent msg3 (GTK, MIC) — 4-way in progress\n");
}

// --- CCMP data plane (software AES-CCM) so the station pings encrypted --------
static const uint8_t kApIp[4] = {192, 168, 99, 1};
static uint64_t g_txpn = 1;                              // AP outbound packet number
static uint16_t csum16(const uint8_t* d, int len) {
  uint32_t s = 0; for (int i=0;i+1<len;i+=2) s += (d[i]<<8)|d[i+1];
  if (len&1) s += d[len-1]<<8; while (s>>16) s=(s&0xffff)+(s>>16); return (uint16_t)~s;
}
// CCMP AAD + nonce from the 802.11 header (802.11i 8.3.3.3.2/.3).
static void ccmp_aad_nonce(const uint8_t* hdr, uint64_t pn, const uint8_t* a2,
                           uint8_t* aad, int* aadlen, uint8_t* nonce) {
  uint16_t fc = hdr[0] | (hdr[1] << 8);
  fc &= ~0x0070; fc &= ~(0x0800|0x1000|0x2000); fc |= 0x4000;  // mask subtype/retry/pm/md, set prot
  aad[0]=fc&0xff; aad[1]=fc>>8;
  memcpy(aad+2, hdr+4, 18);                              // addr1,2,3
  uint16_t seq = (hdr[22]|(hdr[23]<<8)) & 0x000f;        // keep frag, mask seqnum
  aad[20]=seq&0xff; aad[21]=seq>>8; *aadlen=22;
  nonce[0]=0; memcpy(nonce+1, a2, 6);
  for (int i=0;i<6;i++) nonce[7+i] = (pn >> (8*(5-i))) & 0xff;
}
static bool ccm(bool enc, const uint8_t* key, const uint8_t* nonce, const uint8_t* aad,
                int aadlen, const uint8_t* in, int inlen, uint8_t* out, uint8_t* tag) {
  EVP_CIPHER_CTX* c = EVP_CIPHER_CTX_new(); int l; bool ok=true;
  if (enc) {
    EVP_EncryptInit_ex(c, EVP_aes_128_ccm(), 0,0,0);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_AEAD_SET_IVLEN, 13, 0);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_AEAD_SET_TAG, 8, 0);
    EVP_EncryptInit_ex(c, 0,0,key,nonce);
    EVP_EncryptUpdate(c, 0,&l,0,inlen); EVP_EncryptUpdate(c,0,&l,aad,aadlen);
    EVP_EncryptUpdate(c, out,&l,in,inlen); EVP_EncryptFinal_ex(c,out+l,&l);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_AEAD_GET_TAG, 8, tag);
  } else {
    EVP_DecryptInit_ex(c, EVP_aes_128_ccm(), 0,0,0);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_AEAD_SET_IVLEN, 13, 0);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_AEAD_SET_TAG, 8, tag);
    EVP_DecryptInit_ex(c, 0,0,key,nonce);
    EVP_DecryptUpdate(c, 0,&l,0,inlen); EVP_DecryptUpdate(c,0,&l,aad,aadlen);
    ok = EVP_DecryptUpdate(c, out,&l,in,inlen) > 0;
  }
  EVP_CIPHER_CTX_free(c); return ok;
}
// Encrypt an AP->STA payload (LLC/SNAP+eth+data) into a CCMP data frame.
static std::vector<uint8_t> ccmp_tx(const uint8_t* sta, uint16_t eth,
                                    const uint8_t* pl, int plen) {
  std::vector<uint8_t> pt = {0xaa,0xaa,0x03,0,0,0,(uint8_t)(eth>>8),(uint8_t)(eth&0xff)};
  pt.insert(pt.end(), pl, pl+plen);
  std::vector<uint8_t> hdr = {0x08,0x42,0,0,             // data, from-DS + protected
      sta[0],sta[1],sta[2],sta[3],sta[4],sta[5],
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5], 0,0};
  uint64_t pn = g_txpn++;
  uint8_t aad[32], nonce[13], mic[8]; int aadlen;
  ccmp_aad_nonce(hdr.data(), pn, kBssid, aad, &aadlen, nonce);
  std::vector<uint8_t> ct(pt.size());
  ccm(true, g_ptk+32, nonce, aad, aadlen, pt.data(), pt.size(), ct.data(), mic);
  uint8_t ch8[8] = {(uint8_t)(pn&0xff),(uint8_t)((pn>>8)&0xff),0,0x20,
      (uint8_t)((pn>>16)&0xff),(uint8_t)((pn>>24)&0xff),(uint8_t)((pn>>32)&0xff),(uint8_t)((pn>>40)&0xff)};
  std::vector<uint8_t> m = hdr;
  m.insert(m.end(), ch8, ch8+8); m.insert(m.end(), ct.begin(), ct.end());
  m.insert(m.end(), mic, mic+8);
  return m;
}
// DHCP OFFER/ACK payload (IP+UDP+BOOTP) leasing 192.168.99.2 — encrypted by ccmp_tx.
static const uint8_t kLeaseIp[4] = {192,168,99,2};
static std::vector<uint8_t> dhcp_payload(const uint8_t* sta, const uint8_t* xid, uint8_t mt) {
  std::vector<uint8_t> b(236, 0);
  b[0]=2; b[1]=1; b[2]=6; memcpy(&b[4],xid,4);
  memcpy(&b[16],kLeaseIp,4); memcpy(&b[20],kApIp,4); memcpy(&b[28],sta,6);
  const uint8_t opt[]={0x63,0x82,0x53,0x63, 53,1,mt, 54,4,kApIp[0],kApIp[1],kApIp[2],kApIp[3],
      51,4,0,1,0x51,0x80, 1,4,255,255,255,0, 3,4,kApIp[0],kApIp[1],kApIp[2],kApIp[3],
      6,4,kApIp[0],kApIp[1],kApIp[2],kApIp[3], 255};
  b.insert(b.end(), opt, opt+sizeof(opt));
  int ul=8+(int)b.size();
  std::vector<uint8_t> pl(28,0);
  pl[0]=0x45; int tot=20+ul; pl[2]=tot>>8; pl[3]=tot&0xff; pl[8]=64; pl[9]=17;
  memcpy(&pl[12],kApIp,4); pl[16]=pl[17]=pl[18]=pl[19]=0xff;
  uint16_t ic=csum16(pl.data(),20); pl[10]=ic>>8; pl[11]=ic&0xff;
  pl[20]=0; pl[21]=67; pl[22]=0; pl[23]=68; pl[24]=ul>>8; pl[25]=ul&0xff;
  pl.insert(pl.end(), b.begin(), b.end());
  return pl;
}
// Handle a decrypted L2 payload (LLC/SNAP + eth): answer ARP + ICMP + DHCP, encrypted.
static void handle_plain(const uint8_t* sta, const uint8_t* d, int len) {
  if (len < 8 || d[0]!=0xaa) return;                    // not LLC/SNAP (e.g. IPv6 ND)
  uint16_t eth = (d[6]<<8)|d[7]; const uint8_t* pl = d+8; int pllen = len-8;
  if (eth==0x0806 && pllen>=28) {                        // ARP
    if (((pl[6]<<8)|pl[7])==1 && memcmp(pl+24,kApIp,4)==0) {
      uint8_t a[28]={0,1,8,0,6,4,0,2, kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
        kApIp[0],kApIp[1],kApIp[2],kApIp[3], pl[8],pl[9],pl[10],pl[11],pl[12],pl[13],
        pl[14],pl[15],pl[16],pl[17]};
      enqueue(ccmp_tx(sta,0x0806,a,28));
    }
  } else if (eth==0x0800 && pllen>=28) {                 // IPv4/ICMP
    const uint8_t* ip=pl; int ihl=(ip[0]&0x0f)*4;
    if (ip[9]==1 && pllen>=ihl+8 && memcmp(ip+16,kApIp,4)==0 && ip[ihl]==8) {  // ICMP echo
      std::vector<uint8_t> r(pl, pl+pllen);
      memcpy(r.data()+12,kApIp,4); memcpy(r.data()+16,ip+12,4);
      r[10]=r[11]=0; uint16_t ic=csum16(r.data(),ihl); r[10]=ic>>8; r[11]=ic&0xff;
      r[ihl]=0; r[ihl+2]=r[ihl+3]=0;
      uint16_t cc=csum16(r.data()+ihl,pllen-ihl); r[ihl+2]=cc>>8; r[ihl+3]=cc&0xff;
      enqueue(ccmp_tx(sta,0x0800,r.data(),pllen));
    } else if (ip[9]==17 && pllen>=ihl+8+240) {          // UDP -> DHCP
      const uint8_t* udp=ip+ihl;
      if (((udp[2]<<8)|udp[3])==67) {
        const uint8_t* dh=udp+8; const uint8_t* end=pl+pllen; uint8_t m=0;
        for (const uint8_t* o=dh+240; o+1<end && *o!=0xff; ) {
          if (*o==0){o++;continue;} if (*o==53 && o+2<end) m=o[2]; o+=2+o[1]; }
        uint8_t reply = (m==1)?2 : (m==3)?5 : 0;
        if (reply) { auto dp=dhcp_payload(sta, dh+4, reply);
          enqueue(ccmp_tx(sta,0x0800,dp.data(),(int)dp.size())); }
      }
    }
  }
}

static void on_rx(const Packet& p) {
  if (p.Data.size() < 24 || p.RxAtrib.crc_err) return;
  const uint8_t fc0 = p.Data[0], fc1 = p.Data[1];
  const uint8_t* a1 = p.Data.data() + 4;
  const uint8_t* sta = p.Data.data() + 10;
  bool to_us = std::memcmp(a1, kBssid, 6) == 0;
  bool bcast = (a1[0] & 0x01) != 0;

  if (fc0 == 0x40) {                                    // probe-req
    if (!bcast && !to_us) return;
    auto m = mgmt_hdr(0x50, sta);
    m.insert(m.end(), {0,0,0,0,0,0,0,0, 0x64,0x00, 0x11,0x00});  // cap: ESS+Privacy
    append_ies(m, true); enqueue(std::move(m));
  } else if (fc0 == 0xb0 && to_us) {                    // auth
    auto m = mgmt_hdr(0xb0, sta);
    m.insert(m.end(), {0,0, 0x02,0x00, 0,0}); enqueue(std::move(m));
    fprintf(stderr, "  AUTH from %02x:%02x:%02x:%02x:%02x:%02x\n",
            sta[0],sta[1],sta[2],sta[3],sta[4],sta[5]);
  } else if ((fc0 == 0x00 || fc0 == 0x20) && to_us) {   // (re)assoc
    auto m = mgmt_hdr(0x10, sta);
    m.insert(m.end(), {0x11,0x00, 0x00,0x00, 0x01,0xc0});
    append_ies(m, false); enqueue(std::move(m));
    memcpy(g_sta, sta, 6); memset(g_replay, 0, 8); g_state = 0;
    fprintf(stderr, "  ASSOC from %02x:%02x:%02x:%02x:%02x:%02x -> start 4-way\n",
            sta[0],sta[1],sta[2],sta[3],sta[4],sta[5]);
    send_msg1();
  } else if ((fc0 == 0x08 || fc0 == 0x88) && (fc1 & 0x01) && to_us) {  // data to-DS
    int hlen = 24 + (fc0 == 0x88 ? 2 : 0);
    if ((fc1 & 0x40) && g_state == 2) {                 // PROTECTED (CCMP) data
      int len = (int)p.Data.size();
      if (len < hlen + 8 + 8) return;                   // hdr + CCMP hdr + MIC
      const uint8_t* d = p.Data.data();
      const uint8_t* cc = d + hlen;                     // CCMP header
      uint64_t pn = cc[0] | (cc[1]<<8) | ((uint64_t)cc[4]<<16) | ((uint64_t)cc[5]<<24)
                  | ((uint64_t)cc[6]<<32) | ((uint64_t)cc[7]<<40);
      int ctlen = len - hlen - 8 - 8;
      const uint8_t* ct = d + hlen + 8; const uint8_t* mic = ct + ctlen;
      uint8_t aad[32], nonce[13], tag[8]; int aadlen;
      ccmp_aad_nonce(d, pn, sta, aad, &aadlen, nonce);   // A2 = station
      memcpy(tag, mic, 8);
      std::vector<uint8_t> pt(ctlen);
      if (ccm(false, g_ptk+32, nonce, aad, aadlen, ct, ctlen, pt.data(), tag))
        handle_plain(sta, pt.data(), ctlen);             // decrypted -> ARP/ICMP
      return;
    }
    if ((int)p.Data.size() < hlen + 8) return;
    const uint8_t* llc = p.Data.data() + hlen;
    if (!(llc[0]==0xaa && llc[6]==0x88 && llc[7]==0x8e)) return;  // EAPOL
    const uint8_t* e = llc + 8; int elen = (int)p.Data.size() - (hlen + 8);
    if (elen < 99 || e[1] != 3) return;                 // EAPOL-Key
    uint16_t ki = (e[5]<<8) | e[6];
    if ((ki & 0x0008) && (ki & 0x0100) && !(ki & 0x0040) && !(ki & 0x0200)) {
      // msg2: pairwise + MIC, no install/secure -> SNonce + MIC
      memcpy(g_snonce, e+17, 32);
      compute_ptk();
      if (!check_mic(e, elen)) { fprintf(stderr, "  WPA2: msg2 MIC FAIL\n"); return; }
      fprintf(stderr, "  WPA2: msg2 OK (SNonce, MIC verified) — PTK derived\n");
      send_msg3();
    } else if ((ki & 0x0100) && (ki & 0x0200)) {        // msg4: MIC + secure
      if (check_mic(e, elen)) {
        g_state = 2;
        fprintf(stderr, "  WPA2: msg4 OK — 4-WAY HANDSHAKE COMPLETE (station keyed)\n");
      }
    }
  }
}

int main(int argc, char** argv) {
  int sec = argc > 1 ? atoi(argv[1]) : 60;
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) g_chan = (uint8_t)atoi(c);
  if (const char* k = std::getenv("DEVOURER_WPA2_PSK")) g_psk = k;
  auto logger = std::make_shared<Logger>(); apply_logging_env(*logger);
  libusb_context* ctx = nullptr; libusb_init(&ctx);
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid = 0x0bda, pid = 0xc812;
  if (const char* v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x fail\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, devourer::find_wifi_interface(h), logger, true, lk) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lk, devourer_config_from_env());
  g_dev = dev.get(); if (!g_dev) return 1;
  g_rt = devourer::build_stream_radiotap(devourer::parse_tx_mode_str("6M"));
  g_dev->InitWrite(SelectedChannel{g_chan, 0, CHANNEL_WIDTH_20});
  int tu = 25; if (const char* i = std::getenv("DEVOURER_BCN_TU")) tu = atoi(i);
  std::vector<uint8_t> bcn = {0,0,0x0a,0,0,0x80,0,0,0x08,0,
      0x80,0,0,0, 0xff,0xff,0xff,0xff,0xff,0xff,
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
      0,0, 0,0,0,0,0,0,0,0, (uint8_t)(tu&0xff),(uint8_t)(tu>>8), 0x11,0x00};
  append_ies(bcn, true);
  bool bok = g_dev->StartBeacon(bcn.data(), bcn.size(), tu);
  std::thread rx([&]{ g_dev->StartRxLoop(on_rx); });
  fprintf(stderr, "ap_wpa2 up: SSID %s WPA2-PSK '%s' ch%d beacon=%s\n",
          kSsid, g_psk, g_chan, bok ? "OK" : "FAIL");
  auto end = std::chrono::steady_clock::now() + std::chrono::seconds(sec);
  while (std::chrono::steady_clock::now() < end) {
    std::vector<std::vector<uint8_t>> batch;
    { std::lock_guard<std::mutex> l(g_q_mu); batch.swap(g_q); }
    for (auto& f : batch) if (g_dev->send_packet(f.data(), f.size())) g_sent.fetch_add(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  fprintf(stderr, "sent=%llu 4way_state=%d\n", (unsigned long long)g_sent.load(), g_state);
  _exit(0);
}
