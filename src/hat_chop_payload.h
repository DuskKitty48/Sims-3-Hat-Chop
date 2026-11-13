/* Packed TS3 WritePartVerts payload interpolation for a new bisect vertex.
 * Verified EA 1.69 live capture, 0x005D2210:
 * 0x005D2281..22AC consumes uint32 usage words from vector(begin,end).
 * 0x005D2450 dispatches EXACT usage 1..7/10; there is no high-bit mask.
 * 0x005D2640 normal and 0x005D295F tangent write Z,Y,X,0x80 bytes.
 * 0x005D268F UV writes signed SHORT2; the first-UV transform flag is LOCAL
 * byte ESP+17 (cleared 0x005D2612), not part of a declaration usage word.
 * 0x005D285D bones: a13!=0 => four bytes, else four uint16 palette indices.
 * 0x005D2895 weights: bytes [w2,w1,w0,w3], correction to w0 makes sum255.
 * 0x005D29A8 color: four bytes; usage10 at 0x005D29CB writes a SEPARATE ID
 * stream and consumes no VB bytes. Morph/vertex-ID provenance is NOT here.
 * 0x005D2A97 output sizes: pos8, bones4/8, other known VB attributes4.
 * Source: .../decompiled/functions/005D/005D2210_FUN_005d2210.cpp + .asm
 * capture manifest SHA256 0460d42c9c423d7eb2eb422766555078d12a503c1608ee7cfd69f5c2b008bebf.
 * Caller must validate readable stride-sized A/B and writable dst. This
 * pure helper validates the declaration and all calculations before commit.
 */
#ifndef HAT_CHOP_PAYLOAD_H
#define HAT_CHOP_PAYLOAD_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#define HC_PAYLOAD_MAX_STRIDE 256
#define HC_PAYLOAD_MAX_ATTR 32

typedef struct { uint16_t offset; uint8_t usage,size; } HCPayloadAttr;
typedef struct {
    int stride,nattr,boneBytes,hasVertexId;
    int pos,bones,weights;
    HCPayloadAttr attr[HC_PAYLOAD_MAX_ATTR];
} HCPayloadLayout;

/* words is a SNAPSHOT of the pre-o_write declaration vector. Original
 * WritePartVerts erases its front words; never inspect it after that call.
 * boneBytes is 1 when a13!=0, otherwise 2. On failure *out is unchanged. */
static int hc_payload_layout(const uint32_t *words,size_t count,int stride,
                             int boneBytes,HCPayloadLayout *out)
{
    HCPayloadLayout q; size_t i; int offset=0;
    if(!words || !out || count==0 || count>HC_PAYLOAD_MAX_ATTR ||
       stride<8 || stride>HC_PAYLOAD_MAX_STRIDE || (boneBytes!=1 && boneBytes!=2)) return 0;
    memset(&q,0,sizeof(q)); q.pos=q.bones=q.weights=-1;
    q.stride=stride; q.boneBytes=boneBytes;
    for(i=0;i<count;i++) {
        uint32_t usage=words[i]; int size;
        if(usage==10) { if(q.hasVertexId) return 0; q.hasVertexId=1; continue; }
        if(usage<1 || usage>7) return 0;
        size=usage==1?8:(usage==4?4*boneBytes:4);
        if(offset>stride-size) return 0;
        if(usage==1) { if(q.pos>=0) return 0; q.pos=offset; }
        if(usage==4) { if(q.bones>=0) return 0; q.bones=offset; }
        if(usage==5) { if(q.weights>=0) return 0; q.weights=offset; }
        q.attr[q.nattr].offset=(uint16_t)offset;
        q.attr[q.nattr].usage=(uint8_t)usage;
        q.attr[q.nattr].size=(uint8_t)size;
        q.nattr++; offset+=size;
    }
    if(q.pos!=0 || offset!=stride || ((q.bones<0)!=(q.weights<0))) return 0;
    *out=q; return 1;
}

static int hc_payload_valid(const HCPayloadLayout *layout)
{
    uint32_t words[HC_PAYLOAD_MAX_ATTR]; HCPayloadLayout parsed; int i;
    if(!layout || layout->nattr<=0 || layout->nattr>HC_PAYLOAD_MAX_ATTR ||
       layout->hasVertexId<0 || layout->hasVertexId>1) return 0;
    for(i=0;i<layout->nattr;i++) words[i]=layout->attr[i].usage;
    if(!hc_payload_layout(words,(size_t)layout->nattr,layout->stride,layout->boneBytes,&parsed)) return 0;
    if(parsed.pos!=layout->pos || parsed.bones!=layout->bones || parsed.weights!=layout->weights) return 0;
    for(i=0;i<layout->nattr;i++) if(parsed.attr[i].offset!=layout->attr[i].offset ||
        parsed.attr[i].size!=layout->attr[i].size) return 0;
    return 1;
}

static int16_t hc_payload_s16(const unsigned char *p)
{ int16_t value; memcpy(&value,p,2); return value; }
static void hc_payload_put16(unsigned char *p,int value)
{ int16_t v=(int16_t)value; memcpy(p,&v,2); }
static unsigned hc_payload_bone(const unsigned char *p,int bytes,int k)
{
    uint16_t value;
    if(bytes==1) return p[k];
    memcpy(&value,p+2*k,2); return value;
}
static void hc_payload_putbone(unsigned char *p,int bytes,int k,unsigned value)
{
    uint16_t v=(uint16_t)value;
    if(bytes==1) p[k]=(unsigned char)value;
    else memcpy(p+2*k,&v,2);
}
static int hc_payload_round(double x) { return (int)floor(x+.5); }

static int hc_payload_position(unsigned char *out,const unsigned char *base,
                                const float P[3])
{
    int w=hc_payload_s16(base+6),sign=w<0?-1:1,k,q[3]; double largest=0;
    if(w==0 || !P) return 0;
    for(k=0;k<3;k++) {
        double value=P[k];
        if(!(value>=-32765.0 && value<=32765.0)) return 0;
        if(fabs(value)>largest) largest=fabs(value);
    }
    for(k=0;k<3;k++) if((double)P[k]*w < -32767.0 || (double)P[k]*w >32767.0) break;
    if(k<3) {
        int divisor=(int)ceil(largest);
        if(divisor<=0) divisor=1;
        w=sign*(32765/divisor); if(w==0) return 0;
    }
    for(k=0;k<3;k++) {
        q[k]=hc_payload_round((double)P[k]*w);
        if(q[k]<-32767 || q[k]>32767) return 0;
    }
    for(k=0;k<3;k++) hc_payload_put16(out+2*k,q[k]);
    hc_payload_put16(out+6,w); return 1;
}

static int hc_payload_vector(unsigned char *out,const unsigned char *a,
                              const unsigned char *b,double t)
{
    double v[3],length=0; int k;
    if(a[3]!=128 || b[3]!=128) return 0;
    for(k=0;k<3;k++) {v[k]=(2.0*((1-t)*a[k]+t*b[k])/255.0)-1;length+=v[k]*v[k];}
    if(length<.0025) {
        const unsigned char *fallback=t<=.5?a:b;
        length=0;
        for(k=0;k<3;k++) {v[k]=2.0*fallback[k]/255.0-1;length+=v[k]*v[k];}
    }
    if(length<.0025) return 0;
    length=sqrt(length);
    for(k=0;k<3;k++) {
        int value=hc_payload_round((v[k]/length+1)*127.5);
        if(value<0) value=0;
        if(value>255) value=255;
        out[k]=(unsigned char)value;
    }
    out[3]=128; return 1;
}

static int hc_payload_skin(unsigned char *out,const unsigned char *a,
                            const unsigned char *b,double t,const HCPayloadLayout *layout)
{
    static const int order[4]={2,1,0,3};
    unsigned ids[8]; double weight[8]={0},fraction[4];
    int quantized[4]={0},n=0,side,k,j,total=0,keep;
    for(side=0;side<2;side++) {
        const unsigned char *src=side?b:a; double sum=0,factor=side?t:1-t;
        for(k=0;k<4;k++) sum+=src[layout->weights+order[k]];
        if(sum<=0) return 0;
        for(k=0;k<4;k++) {
            unsigned id=hc_payload_bone(src+layout->bones,layout->boneBytes,k);
            double amount=src[layout->weights+order[k]]*factor/sum;
            if(amount<=0) continue;
            for(j=0;j<n && ids[j]!=id;j++) {}
            if(j==n) {if(n>=8)return 0;ids[n++]=id;}
            weight[j]+=amount;
        }
    }
    for(k=1;k<n;k++) {
        unsigned id=ids[k];double w=weight[k];j=k;
        while(j>0 && (weight[j-1]<w || (weight[j-1]==w && ids[j-1]>id))) {
            ids[j]=ids[j-1];weight[j]=weight[j-1];j--;
        }
        ids[j]=id;weight[j]=w;
    }
    keep=n<4?n:4;
    if(keep<=0) return 0;
    {
        double sum=0;
        for(k=0;k<keep;k++) sum+=weight[k];
        if(!(sum>0)) return 0;
        for(k=0;k<keep;k++) {
            double value=255*weight[k]/sum;
            quantized[k]=(int)floor(value);fraction[k]=value-quantized[k];total+=quantized[k];
        }
    }
    while(total<255) {
        int best=0;
        for(k=1;k<keep;k++) if(fraction[k]>fraction[best]) best=k;
        quantized[best]++;fraction[best]=-1;total++;
    }
    if(total!=255) return 0;
    for(k=0;k<4;k++) {
        hc_payload_putbone(out+layout->bones,layout->boneBytes,k,k<keep?ids[k]:0);
        out[layout->weights+order[k]]=(unsigned char)quantized[k];
    }
    return 1;
}

/* Both inputs are ORIGINAL packed vertices. t is the ORIGINAL source-edge
 * intersection parameter; newP is the shaped/clipped position. All channels
 * interpolate, bones merge by global drawable palette ID, top4 renormalize.
 * Exact endpoints preserve their original non-position bytes. New position
 * keeps that endpoint's W if representable, otherwise recomputes signed W.
 * dst may alias either input; all failures leave it byte-for-byte unchanged.
 * This updates neither external vertex IDs nor any GPU morph-delta stream. */
static int hc_payload_interpolate(void *dst,const void *constA,const void *constB,
                                  double t,const float pos[3],const HCPayloadLayout *layout)
{
    const unsigned char *a=(const unsigned char *)constA,*b=(const unsigned char *)constB;
    unsigned char temporary[HC_PAYLOAD_MAX_STRIDE];int i,k;
    if(!dst || !a || !b || !pos || !(t>=0 && t<=1) || !hc_payload_valid(layout)) return 0;
    if(!hc_payload_s16(a+layout->pos+6) || !hc_payload_s16(b+layout->pos+6)) return 0;
    memcpy(temporary,t==1?b:a,(size_t)layout->stride);
    if(t>0 && t<1) {
        for(i=0;i<layout->nattr;i++) {
            int usage=layout->attr[i].usage,offset=layout->attr[i].offset;
            if(usage==2 || usage==6) {
                if(!hc_payload_vector(temporary+offset,a+offset,b+offset,t)) return 0;
            } else if(usage==3) {
                for(k=0;k<2;k++) {
                    double value=(1-t)*hc_payload_s16(a+offset+2*k)+t*hc_payload_s16(b+offset+2*k);
                    hc_payload_put16(temporary+offset+2*k,hc_payload_round(value));
                }
            } else if(usage==7) {
                for(k=0;k<4;k++) temporary[offset+k]=(unsigned char)hc_payload_round((1-t)*a[offset+k]+t*b[offset+k]);
            }
        }
        if(layout->bones>=0 && !hc_payload_skin(temporary,a,b,t,layout)) return 0;
    }
    if(!hc_payload_position(temporary+layout->pos,(t==1?b:a)+layout->pos,pos)) return 0;
    memcpy(dst,temporary,(size_t)layout->stride);return 1;
}
#endif
