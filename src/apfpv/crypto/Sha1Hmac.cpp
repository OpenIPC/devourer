#include "Sha1Hmac.h"
#include <cstring>
namespace apfpv { namespace crypto {
struct Sha1Ctx { uint32_t h[5]; uint64_t len; uint8_t buf[64]; size_t n; };
static inline uint32_t rol(uint32_t v,int b){ return (v<<b)|(v>>(32-b)); }
static void sha1_init(Sha1Ctx& c){ c.h[0]=0x67452301;c.h[1]=0xEFCDAB89;c.h[2]=0x98BADCFE;c.h[3]=0x10325476;c.h[4]=0xC3D2E1F0;c.len=0;c.n=0; }
static void sha1_block(Sha1Ctx& c,const uint8_t* p){
    uint32_t w[80];
    for(int i=0;i<16;i++) w[i]=(p[i*4]<<24)|(p[i*4+1]<<16)|(p[i*4+2]<<8)|p[i*4+3];
    for(int i=16;i<80;i++) w[i]=rol(w[i-3]^w[i-8]^w[i-14]^w[i-16],1);
    uint32_t a=c.h[0],b=c.h[1],d=c.h[2],e=c.h[3],f=c.h[4];
    for(int i=0;i<80;i++){ uint32_t k,t;
        if(i<20){k=0x5A827999;t=(b&d)|((~b)&e);}
        else if(i<40){k=0x6ED9EBA1;t=b^d^e;}
        else if(i<60){k=0x8F1BBCDC;t=(b&d)|(b&e)|(d&e);}
        else{k=0xCA62C1D6;t=b^d^e;}
        uint32_t tmp=rol(a,5)+t+f+k+w[i]; f=e;e=d;d=rol(b,30);b=a;a=tmp; }
    c.h[0]+=a;c.h[1]+=b;c.h[2]+=d;c.h[3]+=e;c.h[4]+=f;
}
static void sha1_update(Sha1Ctx& c,const uint8_t* p,size_t len){
    c.len+=len;
    while(len){ size_t k=64-c.n; if(k>len)k=len; memcpy(c.buf+c.n,p,k); c.n+=k;p+=k;len-=k;
        if(c.n==64){sha1_block(c,c.buf);c.n=0;} }
}
static void sha1_final(Sha1Ctx& c,uint8_t out[20]){
    uint64_t bits=c.len*8; uint8_t pad=0x80; sha1_update(c,&pad,1);
    uint8_t z=0; while(c.n!=56) sha1_update(c,&z,1);
    uint8_t lb[8]; for(int i=0;i<8;i++) lb[i]=(uint8_t)(bits>>(56-i*8)); sha1_update(c,lb,8);
    for(int i=0;i<5;i++){out[i*4]=c.h[i]>>24;out[i*4+1]=c.h[i]>>16;out[i*4+2]=c.h[i]>>8;out[i*4+3]=c.h[i];}
}
void sha1(const uint8_t* d,size_t l,uint8_t o[20]){ Sha1Ctx c; sha1_init(c); sha1_update(c,d,l); sha1_final(c,o); }
void hmac_sha1(const uint8_t* key,size_t kl,const uint8_t* msg,size_t ml,uint8_t out[20]){
    uint8_t k[64]={0}; if(kl>64){ sha1(key,kl,k);} else memcpy(k,key,kl);
    uint8_t ip[64],op[64]; for(int i=0;i<64;i++){ip[i]=k[i]^0x36;op[i]=k[i]^0x5c;}
    uint8_t ih[20]; Sha1Ctx c; sha1_init(c); sha1_update(c,ip,64); sha1_update(c,msg,ml); sha1_final(c,ih);
    sha1_init(c); sha1_update(c,op,64); sha1_update(c,ih,20); sha1_final(c,out);
}
void pbkdf2_sha1(const char* pass,const uint8_t* salt,size_t sl,unsigned iters,uint8_t* out,size_t outlen){
    size_t pl=strlen(pass); unsigned block=1;
    while(outlen){ uint8_t u[20],t[20],s2[256]; size_t s2l=sl;
        memcpy(s2,salt,sl); s2[s2l++]=block>>24;s2[s2l++]=block>>16;s2[s2l++]=block>>8;s2[s2l++]=block;
        hmac_sha1((const uint8_t*)pass,pl,s2,s2l,u); memcpy(t,u,20);
        for(unsigned i=1;i<iters;i++){ hmac_sha1((const uint8_t*)pass,pl,u,20,u); for(int j=0;j<20;j++)t[j]^=u[j]; }
        size_t k=outlen<20?outlen:20; memcpy(out,t,k); out+=k; outlen-=k; block++; }
}
}}
