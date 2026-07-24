// Self-contained AES-128 + CCM (CCMP: M=8, L=2). Public-domain style AES core.
#include "AesCcm.h"
#include <cstring>
#include <cstdlib>   // getenv (DEVOURER_FORCE_SW_AES A/B switch)
// x86 AES-NI intrinsics (must be included at file scope, NOT inside a namespace).
// Gives native Windows/Linux x86 hosts the same hardware AES the ARM phone gets
// from ARM-CE — without it, software AES runs ~8x slower (355us vs ~10us/pkt) and
// caps CCMP-decrypt throughput well below line rate.
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
#define APFPV_X86_AESNI 1
#if defined(__GNUC__)
#include <cpuid.h>   // __get_cpuid — reliable AES-NI detection on mingw/GCC
#endif
#endif
namespace apfpv { namespace crypto {

static const uint8_t S[256]={
0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16};
static uint8_t xt(uint8_t x){ return (x<<1)^((x>>7)*0x1b); }
static void aes_key(Aes& a,const uint8_t k[16]){
    memcpy(a.rk,k,16); uint8_t rc=1;
    for(int i=16;i<176;i+=4){
        uint8_t t[4]; memcpy(t,a.rk+i-4,4);
        if(i%16==0){ uint8_t tmp=t[0];t[0]=S[t[1]]^rc;t[1]=S[t[2]];t[2]=S[t[3]];t[3]=S[tmp]; rc=xt(rc); }
        for(int j=0;j<4;j++) a.rk[i+j]=a.rk[i-16+j]^t[j];
    }
}
static void aes_enc(const Aes& a,const uint8_t in[16],uint8_t out[16]){
    uint8_t s[16]; memcpy(s,in,16); for(int i=0;i<16;i++)s[i]^=a.rk[i];
    for(int r=1;r<10;r++){
        uint8_t t[16];
        for(int i=0;i<16;i++) t[i]=S[s[i]];
        uint8_t u[16];
        u[0]=t[0];u[4]=t[4];u[8]=t[8];u[12]=t[12];
        u[1]=t[5];u[5]=t[9];u[9]=t[13];u[13]=t[1];
        u[2]=t[10];u[6]=t[14];u[10]=t[2];u[14]=t[6];
        u[3]=t[15];u[7]=t[3];u[11]=t[7];u[15]=t[11];
        for(int c=0;c<4;c++){ uint8_t* col=u+c*4; uint8_t a0=col[0],a1=col[1],a2=col[2],a3=col[3];
            col[0]=xt(a0)^(xt(a1)^a1)^a2^a3; col[1]=a0^xt(a1)^(xt(a2)^a2)^a3;
            col[2]=a0^a1^xt(a2)^(xt(a3)^a3); col[3]=(xt(a0)^a0)^a1^a2^xt(a3); }
        for(int i=0;i<16;i++) s[i]=u[i]^a.rk[r*16+i];
    }
    uint8_t t[16]; for(int i=0;i<16;i++)t[i]=S[s[i]];
    uint8_t u[16];
    u[0]=t[0];u[4]=t[4];u[8]=t[8];u[12]=t[12];
    u[1]=t[5];u[5]=t[9];u[9]=t[13];u[13]=t[1];
    u[2]=t[10];u[6]=t[14];u[10]=t[2];u[14]=t[6];
    u[3]=t[15];u[7]=t[3];u[11]=t[7];u[15]=t[11];
    for(int i=0;i<16;i++) out[i]=u[i]^a.rk[160+i];
}
// CCM core: CBC-MAC over (B0|AAD|payload), CTR for encryption. M=8,L=2.
static void ccm_core(const Aes& a,const uint8_t* nonce,size_t nlen,
                     const uint8_t* aad,size_t aadlen,
                     const uint8_t* in,size_t inlen,uint8_t* out,uint8_t mic[8],bool enc){
    uint8_t L=2; uint8_t B[16]={0};
    B[0]=(uint8_t)((aadlen>0?0x40:0)|(((8-2)/2)<<3)|(L-1));
    memcpy(B+1,nonce,nlen);
    B[14]=inlen>>8;B[15]=inlen&0xff;
    uint8_t X[16]; aes_enc(a,B,X);
    if(aadlen>0){ uint8_t hdr[16]={0}; size_t o=0; hdr[0]=aadlen>>8;hdr[1]=aadlen&0xff;o=2;
        size_t i=0; while(i<aadlen){ while(o<16&&i<aadlen)hdr[o++]=aad[i++]; for(int j=0;j<16;j++)X[j]^=hdr[j]; aes_enc(a,X,X); o=0; memset(hdr,0,16);} }
    // payload into MAC
    { size_t i=0; uint8_t blk[16];
      while(i<inlen){ size_t k=inlen-i<16?inlen-i:16; memset(blk,0,16); memcpy(blk,in+i,k);
          for(int j=0;j<16;j++)X[j]^=blk[j]; aes_enc(a,X,X); i+=k; } }
    // CTR
    uint8_t A[16]={0}; A[0]=L-1; memcpy(A+1,nonce,nlen); A[14]=0;A[15]=0;
    uint8_t S0[16]; aes_enc(a,A,S0);
    for(int i=0;i<8;i++) mic[i]=X[i]^S0[i];
    size_t i=0; uint16_t ctr=1;
    while(i<inlen){ A[14]=ctr>>8;A[15]=ctr&0xff; uint8_t Sx[16]; aes_enc(a,A,Sx);
        size_t k=inlen-i<16?inlen-i:16; for(size_t j=0;j<k;j++) out[i+j]=in[i+j]^Sx[j]; i+=k; ctr++; }
    (void)enc;
}
void aes_ccm_encrypt(const uint8_t key[16],const uint8_t* nonce,size_t nlen,
                     const uint8_t* aad,size_t aadlen,const uint8_t* pt,size_t ptlen,uint8_t* out){
    Aes a; aes_key(a,key); uint8_t mic[8];
    ccm_core(a,nonce,nlen,aad,aadlen,pt,ptlen,out,mic,true);
    memcpy(out+ptlen,mic,8);
}
bool aes_ccm_decrypt(const uint8_t key[16],const uint8_t* nonce,size_t nlen,
                     const uint8_t* aad,size_t aadlen,const uint8_t* ct,size_t ctlen,uint8_t* out){
    if(ctlen<8) return false; size_t ptlen=ctlen-8;
    Aes a; aes_key(a,key);
    // decrypt: CTR first to recover plaintext, then MAC-verify
    uint8_t A[16]={0}; A[0]=1; memcpy(A+1,nonce,nlen);
    uint16_t ctr=1; size_t i=0;
    while(i<ptlen){ A[14]=ctr>>8;A[15]=ctr&0xff; uint8_t Sx[16]; aes_enc(a,A,Sx);
        size_t k=ptlen-i<16?ptlen-i:16; for(size_t j=0;j<k;j++) out[i+j]=ct[i+j]^Sx[j]; i+=k; ctr++; }
    uint8_t mic[8],calc[16]; (void)calc;
    uint8_t tmp[2048]; if(ptlen>sizeof(tmp)) return false;
    ccm_core(a,nonce,nlen,aad,aadlen,out,ptlen,tmp,mic,false);
    return memcmp(mic,ct+ptlen,8)==0;
}
// AES key cache support: export expanded key + cached-decrypt overload
void aes_key_expand(const uint8_t key[16], Aes& a) { aes_key(a, key); }
bool aes_ccm_decrypt(const uint8_t key[16],const uint8_t* nonce,size_t nlen,
                     const uint8_t* aad,size_t aadlen,const uint8_t* ct,size_t ctlen,uint8_t* out,
                     const Aes* aes_cache){
    if(ctlen<8) return false; size_t ptlen=ctlen-8;
    const Aes* a = aes_cache;
    Aes local; if(!a){ aes_key(local,key); a=&local; }
    uint8_t A[16]={0}; A[0]=1; memcpy(A+1,nonce,nlen);
    uint16_t ctr=1; size_t i=0;
    while(i<ptlen){ A[14]=ctr>>8;A[15]=ctr&0xff; uint8_t Sx[16]; aes_enc(*a,A,Sx);
        size_t k=ptlen-i<16?ptlen-i:16; for(size_t j=0;j<k;j++) out[i+j]=ct[i+j]^Sx[j]; i+=k; ctr++; }
    uint8_t mic[8],tmp[2048]; if(ptlen>sizeof(tmp)) return false;
    ccm_core(*a,nonce,nlen,aad,aadlen,out,ptlen,tmp,mic,false);
    return memcmp(mic,ct+ptlen,8)==0;
}

#if defined(__aarch64__)
#define APFPV_HAVE_AES_HW 1
// ARM-CE accelerated decrypt — data RX only (NOT handshake).
// Uses inline ASM AESE/AESMC. Separate from SW AES above so the
// EAPOL/4-way handshake always uses the proven SW implementation.
struct AesCeKey { uint8_t rk[176]; };
static void aes_ce_expand(const uint8_t k[16], AesCeKey& ce){
    static const uint8_t Rcon[11]={0,1,2,4,8,0x10,0x20,0x40,0x80,0x1b,0x36};
    uint8_t* rk=ce.rk; memcpy(rk,k,16);
    for(int r=1;r<=10;r++){
        uint8_t prev[16]; memcpy(prev,rk+(r-1)*16,16);
        uint8_t tmp[4]; tmp[0]=S[prev[13]]; tmp[1]=S[prev[14]]; tmp[2]=S[prev[15]]; tmp[3]=S[prev[12]];
        tmp[0]^=Rcon[r];
        for(int i=0;i<4;i++) rk[r*16+i]=rk[(r-1)*16+i]^tmp[i];
        for(int i=4;i<16;i++) rk[r*16+i]=rk[(r-1)*16+i]^rk[r*16+i-4];
    }
}
static void aes_enc_ce(const AesCeKey& ce,const uint8_t in[16],uint8_t out[16]){
    const uint8_t* rk=ce.rk;
    __asm__ __volatile__(
        ".arch_extension crypto\n\t"
        "ld1 {v0.16b}, [%[in]]\n\t"
        "ld1 {v1.16b}, [%[pk]], #16\n\t"
        "aese v0.16b, v1.16b\n\t" "aesmc v0.16b, v0.16b\n\t"
        "ld1 {v1.16b}, [%[pk]], #16\n\t"
        "aese v0.16b, v1.16b\n\t" "aesmc v0.16b, v0.16b\n\t"
        "ld1 {v1.16b}, [%[pk]], #16\n\t"
        "aese v0.16b, v1.16b\n\t" "aesmc v0.16b, v0.16b\n\t"
        "ld1 {v1.16b}, [%[pk]], #16\n\t"
        "aese v0.16b, v1.16b\n\t" "aesmc v0.16b, v0.16b\n\t"
        "ld1 {v1.16b}, [%[pk]], #16\n\t"
        "aese v0.16b, v1.16b\n\t" "aesmc v0.16b, v0.16b\n\t"
        "ld1 {v1.16b}, [%[pk]], #16\n\t"
        "aese v0.16b, v1.16b\n\t" "aesmc v0.16b, v0.16b\n\t"
        "ld1 {v1.16b}, [%[pk]], #16\n\t"
        "aese v0.16b, v1.16b\n\t" "aesmc v0.16b, v0.16b\n\t"
        "ld1 {v1.16b}, [%[pk]], #16\n\t"
        "aese v0.16b, v1.16b\n\t" "aesmc v0.16b, v0.16b\n\t"
        "ld1 {v1.16b}, [%[pk]], #16\n\t"
        "aese v0.16b, v1.16b\n\t" "aesmc v0.16b, v0.16b\n\t"
        "ld1 {v1.16b}, [%[pk]], #16\n\t"
        "aese v0.16b, v1.16b\n\t"                         // round 10: SubBytes+ShiftRows, no MixColumns
        "ld1 {v1.16b}, [%[pk]]\n\t"                       // rk[10]
        "eor v0.16b, v0.16b, v1.16b\n\t"                  // FINAL AddRoundKey — was MISSING (broke every block)
        "st1 {v0.16b}, [%[out]]\n\t"
        : [pk] "+r"(rk)
        : [in] "r"(in), [out] "r"(out)
        : "v0","v1","v16","cc","memory"
    );
}
#elif defined(APFPV_X86_AESNI)
#define APFPV_HAVE_AES_HW 1
// x86 AES-NI accelerated decrypt — data RX only (NOT handshake), mirrors the
// ARM-CE path. Same CCM logic; only the block cipher uses AES-NI intrinsics.
#if defined(__GNUC__)
#define APFPV_AES_TGT __attribute__((target("aes,sse2")))
#else
#define APFPV_AES_TGT
#endif
struct AesCeKey { __m128i rk[11]; };
APFPV_AES_TGT static inline __m128i x86_kexp(__m128i key, __m128i kga){
    kga = _mm_shuffle_epi32(kga, _MM_SHUFFLE(3,3,3,3));
    key = _mm_xor_si128(key, _mm_slli_si128(key,4));
    key = _mm_xor_si128(key, _mm_slli_si128(key,4));
    key = _mm_xor_si128(key, _mm_slli_si128(key,4));
    return _mm_xor_si128(key, kga);
}
APFPV_AES_TGT static void aes_ce_expand(const uint8_t k[16], AesCeKey& ce){
    ce.rk[0]=_mm_loadu_si128(reinterpret_cast<const __m128i*>(k));
    ce.rk[1]=x86_kexp(ce.rk[0],_mm_aeskeygenassist_si128(ce.rk[0],0x01));
    ce.rk[2]=x86_kexp(ce.rk[1],_mm_aeskeygenassist_si128(ce.rk[1],0x02));
    ce.rk[3]=x86_kexp(ce.rk[2],_mm_aeskeygenassist_si128(ce.rk[2],0x04));
    ce.rk[4]=x86_kexp(ce.rk[3],_mm_aeskeygenassist_si128(ce.rk[3],0x08));
    ce.rk[5]=x86_kexp(ce.rk[4],_mm_aeskeygenassist_si128(ce.rk[4],0x10));
    ce.rk[6]=x86_kexp(ce.rk[5],_mm_aeskeygenassist_si128(ce.rk[5],0x20));
    ce.rk[7]=x86_kexp(ce.rk[6],_mm_aeskeygenassist_si128(ce.rk[6],0x40));
    ce.rk[8]=x86_kexp(ce.rk[7],_mm_aeskeygenassist_si128(ce.rk[7],0x80));
    ce.rk[9]=x86_kexp(ce.rk[8],_mm_aeskeygenassist_si128(ce.rk[8],0x1b));
    ce.rk[10]=x86_kexp(ce.rk[9],_mm_aeskeygenassist_si128(ce.rk[9],0x36));
}
APFPV_AES_TGT static void aes_enc_ce(const AesCeKey& ce,const uint8_t in[16],uint8_t out[16]){
    __m128i m=_mm_loadu_si128(reinterpret_cast<const __m128i*>(in));
    m=_mm_xor_si128(m,ce.rk[0]);
    m=_mm_aesenc_si128(m,ce.rk[1]); m=_mm_aesenc_si128(m,ce.rk[2]);
    m=_mm_aesenc_si128(m,ce.rk[3]); m=_mm_aesenc_si128(m,ce.rk[4]);
    m=_mm_aesenc_si128(m,ce.rk[5]); m=_mm_aesenc_si128(m,ce.rk[6]);
    m=_mm_aesenc_si128(m,ce.rk[7]); m=_mm_aesenc_si128(m,ce.rk[8]);
    m=_mm_aesenc_si128(m,ce.rk[9]);
    m=_mm_aesenclast_si128(m,ce.rk[10]);
    _mm_storeu_si128(reinterpret_cast<__m128i*>(out),m);
}
#endif

#ifdef APFPV_HAVE_AES_HW
// Data RX only: hardware-AES CCMP decrypt (ARM-CE or x86 AES-NI). Returns false
// to fall back to the SW path (e.g. an x86 CPU lacking AES-NI). Only aes_enc_ce
// differs per-arch; the CCM (CTR + CBC-MAC) logic below is shared.
bool aes_ccm_decrypt_ce(const uint8_t key[16],const uint8_t* nonce,size_t nlen,
                        const uint8_t* aad,size_t aadlen,
                        const uint8_t* ct,size_t ctlen,uint8_t* out){
    if(ctlen<8) return false;
    // A/B knob: DEVOURER_FORCE_SW_AES=1 forces the software-AES fallback (the pre-fix path)
    // so the AES-NI win can be measured back-to-back on the same link.
    static const bool kForceSw = std::getenv("DEVOURER_FORCE_SW_AES") != nullptr;
    if (kForceSw) return false;
#if defined(APFPV_X86_AESNI) && defined(__GNUC__)
    // Robust AES-NI detection. __builtin_cpu_supports("aes") returns FALSE on mingw when
    // __cpu_model isn't initialized -> silent software fallback (~290us/pkt) even on a CPU
    // that HAS AES-NI -> single RX worker CPU-bound at ~39 Mbps. Use CPUID directly:
    // leaf 1, ECX bit 25 = AES-NI. (This was the APFPV-vs-wfb RX gap: wfb has no CCMP.)
    static const bool kHasAesNi = [](){
        unsigned a,b,c,d;
        if (__get_cpuid(1,&a,&b,&c,&d)) return (c & (1u<<25)) != 0;
        return false;
    }();
    if(!kHasAesNi) return false;   // genuinely no AES-NI -> SW fallback
#endif
    // TLS cache per worker thread
    static thread_local uint8_t ck[16]={}; static thread_local AesCeKey ce; static thread_local bool ok=false;
    if(!ok||memcmp(ck,key,16)!=0){ aes_ce_expand(key,ce); memcpy(ck,key,16); ok=true; }
    size_t ptlen=ctlen-8;
    // CTR decrypt
    uint8_t A[16]={0}; A[0]=1; memcpy(A+1,nonce,nlen);
    uint16_t ctr=1; size_t i=0;
    while(i<ptlen){ A[14]=ctr>>8;A[15]=ctr&0xff; uint8_t Sx[16]; aes_enc_ce(ce,A,Sx);
        size_t k=ptlen-i<16?ptlen-i:16; for(size_t j=0;j<k;j++) out[i+j]=ct[i+j]^Sx[j]; i+=k; ctr++; }
    // CBC-MAC verify
    uint8_t L=2,B[16]={0}; B[0]=(uint8_t)((aadlen>0?0x40:0)|(((8-2)/2)<<3)|(L-1));
    memcpy(B+1,nonce,nlen); B[14]=ptlen>>8;B[15]=ptlen&0xff;
    uint8_t X[16]; aes_enc_ce(ce,B,X);
    if(aadlen>0){ uint8_t hdr[16]={0}; size_t o=0; hdr[0]=aadlen>>8;hdr[1]=aadlen&0xff;o=2;
        size_t j=0; while(j<aadlen){ while(o<16&&j<aadlen)hdr[o++]=aad[j++];
            for(int v=0;v<16;v++)X[v]^=hdr[v]; aes_enc_ce(ce,X,X); o=0; memset(hdr,0,16);} }
    { size_t j=0; uint8_t blk[16];
      while(j<ptlen){ size_t k=ptlen-j<16?ptlen-j:16; memset(blk,0,16); memcpy(blk,out+j,k);
          for(int v=0;v<16;v++)X[v]^=blk[v]; aes_enc_ce(ce,X,X); j+=k; } }
    uint8_t NA[16]={0}; NA[0]=L-1; memcpy(NA+1,nonce,nlen);
    uint8_t S0[16]; aes_enc_ce(ce,NA,S0);
    uint8_t mic[8]; for(int v=0;v<8;v++) mic[v]=X[v]^S0[v];
    return memcmp(mic,ct+ptlen,8)==0;
}
#endif
}}

// ---- AES-128 ECB encrypt/decrypt + RFC 3394 key unwrap (for GTK) -----------
namespace apfpv { namespace crypto {

// inverse S-box
static uint8_t ISB[256];
static bool isb_init=false;
static void build_isb(){ if(isb_init)return; for(int i=0;i<256;i++) ISB[S[i]]=i; isb_init=true; }
static uint8_t mul(uint8_t a,uint8_t b){ uint8_t p=0; for(int i=0;i<8;i++){ if(b&1)p^=a; uint8_t hi=a&0x80; a<<=1; if(hi)a^=0x1b; b>>=1;} return p; }

void aes128_ecb_encrypt(const uint8_t key[16],const uint8_t in[16],uint8_t out[16]){
    Aes a; aes_key(a,key); aes_enc(a,in,out);
}
void aes128_ecb_decrypt(const uint8_t key[16],const uint8_t in[16],uint8_t out[16]){
    build_isb(); Aes a; aes_key(a,key);
    uint8_t s[16]; for(int i=0;i<16;i++) s[i]=in[i]^a.rk[160+i];
    for(int r=9;r>=1;r--){
        // inv shiftrows
        uint8_t t[16];
        t[0]=s[0];t[4]=s[4];t[8]=s[8];t[12]=s[12];
        t[1]=s[13];t[5]=s[1];t[9]=s[5];t[13]=s[9];
        t[2]=s[10];t[6]=s[14];t[10]=s[2];t[14]=s[6];
        t[3]=s[7];t[7]=s[11];t[11]=s[15];t[15]=s[3];
        for(int i=0;i<16;i++) t[i]=ISB[t[i]];            // inv subbytes
        for(int i=0;i<16;i++) t[i]^=a.rk[r*16+i];        // addroundkey
        // inv mixcolumns
        for(int c=0;c<4;c++){ uint8_t* col=t+c*4; uint8_t a0=col[0],a1=col[1],a2=col[2],a3=col[3];
            col[0]=mul(a0,14)^mul(a1,11)^mul(a2,13)^mul(a3,9);
            col[1]=mul(a0,9)^mul(a1,14)^mul(a2,11)^mul(a3,13);
            col[2]=mul(a0,13)^mul(a1,9)^mul(a2,14)^mul(a3,11);
            col[3]=mul(a0,11)^mul(a1,13)^mul(a2,9)^mul(a3,14); }
        for(int i=0;i<16;i++) s[i]=t[i];
    }
    uint8_t t[16];
    t[0]=s[0];t[4]=s[4];t[8]=s[8];t[12]=s[12];
    t[1]=s[13];t[5]=s[1];t[9]=s[5];t[13]=s[9];
    t[2]=s[10];t[6]=s[14];t[10]=s[2];t[14]=s[6];
    t[3]=s[7];t[7]=s[11];t[11]=s[15];t[15]=s[3];
    for(int i=0;i<16;i++) out[i]=ISB[t[i]]^a.rk[i];
}

// RFC 3394 AES Key Unwrap. wrapped = (n+1)*8 bytes. out = n*8.
bool aes_key_unwrap(const uint8_t kek[16],const uint8_t* wrapped,size_t wlen,uint8_t* out){
    if(wlen<16 || (wlen%8)!=0) return false;
    size_t n=(wlen/8)-1;
    uint8_t A[8]; memcpy(A,wrapped,8);
    memcpy(out,wrapped+8,n*8);
    for(int j=5;j>=0;j--){
        for(size_t i=n;i>=1;i--){
            uint8_t blk[16]; memcpy(blk,A,8); memcpy(blk+8,out+(i-1)*8,8);
            uint64_t t=n*(uint64_t)j+i;
            for(int k=0;k<8;k++) blk[7-k]^=(uint8_t)(t>>(k*8));
            uint8_t dec[16]; aes128_ecb_decrypt(kek,blk,dec);
            memcpy(A,dec,8); memcpy(out+(i-1)*8,dec+8,8);
            if(i==1) {} // continue
        }
    }
    static const uint8_t IV[8]={0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6};
    return memcmp(A,IV,8)==0;
}

// RFC 3394 AES Key Wrap (inverse of unwrap) — wraps plen bytes (multiple of 8, >=16) into
// out[plen+8]. Used by the AP-side 4-way (M3) to deliver the GTK encrypted under the KEK.
void aes_key_wrap(const uint8_t kek[16],const uint8_t* plain,size_t plen,uint8_t* out){
    size_t n=plen/8;
    static const uint8_t IV[8]={0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6};
    uint8_t A[8]; memcpy(A,IV,8);
    memcpy(out+8,plain,plen);                       // R[1..n] = P
    for(int j=0;j<=5;j++){
        for(size_t i=1;i<=n;i++){
            uint8_t blk[16]; memcpy(blk,A,8); memcpy(blk+8,out+i*8,8);
            uint8_t enc[16]; aes128_ecb_encrypt(kek,blk,enc);
            memcpy(A,enc,8);
            uint64_t t=n*(uint64_t)j+i;
            for(int k=0;k<8;k++) A[7-k]^=(uint8_t)(t>>(k*8));   // A = MSB64(B) XOR t
            memcpy(out+i*8,enc+8,8);                            // R[i] = LSB64(B)
        }
    }
    memcpy(out,A,8);                                 // C[0] = A
}
}}
