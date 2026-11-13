/* Version5 post-FillDrawable transaction; include AFTER Ctx/hk_write.
 * Source proof: 005D4700 calls WritePartVerts at005D4820, copies ushort
 * source indices via005CA330 at005D4841, sorts whole triples005D2B60, then
 * 005CA290 copies the whole primary VB and negates its four position shorts.
 * Original locked IB/VB bases are restored at005D48C8/CB; caller unlocks only
 * after hk_fill returns. See corresponding .cpp/.asm in the verified live
 * capture6707155C. No source GEOM resource or external morph-ID stream changes.
 *
 * Every original final index must match the complete ordered source-triangle
 * multiset BEFORE shaping/cutting. Duplicate keys use occurrence cursors so
 * each original source slot maps to exactly one sorted destination slot,
 * including removed-top face slots reclaimed for a clipped quad's extra face.
 * Both full output VB and IB are staged, then committed together only after
 * ALL parts succeed. Failure leaves both original destination buffers intact.
 */
#ifndef HAT_CHOP_PROFILE_INDICES_H
#define HAT_CHOP_PROFILE_INDICES_H

#include "hat_chop_profile_apply.h"

typedef struct { uint64_t key; uint32_t source; } HCProfileFace;
typedef struct {
    uint32_t geom,base,first;
    int count,nt,hair;
    const HCPayloadLayout *layout;
} HCProfilePart;

static uint32_t hc_profile_hash(uint32_t hash,uint32_t value)
{ return (hash^value)*16777619u; }

static int hc_profile_ranges_overlap(uint32_t a,size_t an,uint32_t b,size_t bn)
{
    return (uint64_t)a<(uint64_t)b+bn && (uint64_t)b<(uint64_t)a+an;
}

static void hc_profile_capture(int geom,int elems,int stride,int vb,int boneByteFlag)
{
    Ctx *c=(Ctx *)TlsGetValue(g_tls);
    uint32_t begin,end,words[HC_PAYLOAD_MAX_ATTR];size_t count;int i;
    HCCapturedLayout value;GeomInfo gi;
    if(!c || !c->mb || !c->hats || !g_enabled || c->captureFailed) return;
    if(!geom || (geom&3) || !geom_info((void *)(uint32_t)geom,&gi)) goto bad;
    /* Only hair payloads are rewritten. Unrelated part declarations must not
     * disable the hair transaction or consume its capture slots. */
    if(gi.shader!=SHADER_SIMHAIR) return;
    if(c->captureCount<0 || c->captureCount>=256 || !readable((void *)(uint32_t)elems,8)) goto bad;
    begin=U32((void *)(uint32_t)elems,0);end=U32((void *)(uint32_t)elems,4);
    if((begin&3) || end<begin || (end-begin)%4 || !begin ||
       end-begin>4*HC_PAYLOAD_MAX_ATTR || !readable((void *)begin,end-begin)) goto bad;
    count=(end-begin)/4;
    memcpy(words,(void *)begin,count*4);
    memset(&value,0,sizeof(value));value.geom=(uint32_t)geom;value.vb=(uint32_t)vb;
    value.count=gi.count;value.stride=stride&0xffff;
    if(!hc_payload_layout(words,count,value.stride,(boneByteFlag&255)?1:2,&value.layout)) goto bad;
    for(i=0;i<c->captureCount;i++) if(c->captured[i].geom==value.geom && c->captured[i].vb==value.vb) goto bad;
    c->captured[c->captureCount++]=value;return;
bad:
    c->captureFailed=1;
}

/* Actual head floor includes Face(Head), Scalp(HeadNew/HeadDome). Scalp has
 * NO Head joint in verified EA resources, so checking only Head loses its top.
 * Tight triangle bounds exclude bodies and hat accessories. Bind positions
 * cannot reflect later GPU slider/morph deltas; that limitation is unchanged.
 */
#ifdef HC_HAT_FIT
/* v7.1: top of the face part (the only SimSkin part on all three head joints;
 * any head-joint part if none). -HUGE_VAL when nothing is found. */
static double hc_profile_face_top(uint32_t db,uint32_t de)
{
    uint32_t d,pb,pe,p;double faceTop=-HUGE_VAL,anyTop=-HUGE_VAL;
    for(d=db;d<de;d+=DRAW_SIZE) {
        pb=U32(d,DRAW_PARTS_BEGIN);pe=U32(d,DRAW_PARTS_END);
        if(pe<pb || (pe-pb)%PART_SIZE || pe-pb>PART_SIZE*256 || (pe>pb && !readable((void *)pb,pe-pb)))continue;
        for(p=pb;p<pe;p+=PART_SIZE) {
            uint32_t geom=U32(p,PART_GEOM);GeomInfo gi;int j,mask=0;double ymax=-HUGE_VAL;
            if(!geom || (geom&3) || !geom_info((void *)geom,&gi))continue;
            if(gi.shader!=0x548394B9u || is_hat(&gi))continue;
            for(j=0;j<gi.njoints;j++) {
                if(gi.joints[j]==0x0F97B21Bu)mask|=1;
                if(gi.joints[j]==0x6E191ACBu)mask|=2;
                if(gi.joints[j]==0x9D5460CEu)mask|=4;
            }
            if(!mask)continue;
            for(j=0;j<gi.count;j++) {
                const float *q=(const float *)(gi.data+(size_t)j*gi.stride+gi.posOff);
                if(hc_gr_finite(q[1]) && q[1]>ymax)ymax=q[1];
            }
            if(ymax>anyTop)anyTop=ymax;
            if(mask==7 && ymax>faceTop)faceTop=ymax;
        }
    }
    return faceTop>-HUGE_VAL?faceTop:anyTop;
}
#endif

#ifdef HC_HAT_FIT
/* v7.2 chop type 3 "bald" (full hat; types 1 and 2 are the cap and beanie cuts): every scalp-hair triangle
 * of the drawable becomes degenerate in the final index buffer
 * (the hair resources are untouched; taking the hat off rebuilds the Sim with the full hairstyle).
 * Hair = SimHair parts reaching the top of the head (a beard part stays). The part order and vertex counts
 * give each part's index range; every hair vertex is first checked against its GEOM position, so a layout
 * this code does not know changes nothing. -> hidden triangle count, 0 nothing to do, -1 left alone. */
static int hc_full_hat_hide_hair(Ctx *c,void *self,void *info)
{
    uint32_t db,de,di,draw,pb,pe,part,nv,ni,stride,ib,vb,base=0,i,lo[64],hi[64];
    int nranges=0,hidden=0,k,doubled;double faceTop;size_t vbbytes;
    if(!c || !readable(self,MB_DRAW_END+4) || !readable(info,0x46) || !*((uint8_t *)info+0x42))return 0;
    db=U32(self,MB_DRAW_BEGIN);de=U32(self,MB_DRAW_END);di=U32(info,0x20);
    if(de<db || (de-db)%DRAW_SIZE || de-db>DRAW_SIZE*64 || di>=(de-db)/DRAW_SIZE || !readable((void *)db,de-db))goto bad;
    draw=db+di*DRAW_SIZE;nv=U32(draw,0);ni=U32(draw,4);
    pb=U32(draw,DRAW_PARTS_BEGIN);pe=U32(draw,DRAW_PARTS_END);
    stride=*(const uint16_t *)((uint8_t *)info+0x40);ib=U32(info,0x34);vb=U32(info,0x38);
    doubled=*((uint8_t *)info+0x44)!=0;
    if(!nv || nv>65535 || !ni || ni%3 || ni>3000000 || stride<8 || stride>1024 ||
       pe<pb || (pe-pb)%PART_SIZE || pe-pb>PART_SIZE*256 || !readable((void *)pb,pe-pb))goto bad;
    vbbytes=(size_t)nv*(doubled+1)*stride;
    if(!memory_range((void *)ib,(size_t)ni*2,1) || !memory_range((void *)vb,vbbytes,0))goto bad;
    faceTop=hc_profile_face_top(db,de);
    for(part=pb;part<pe;part+=PART_SIZE) {
        uint32_t geom=U32(part,PART_GEOM),j;GeomInfo gi;
        if(!geom || (geom&3) || !geom_info((void *)geom,&gi) || (uint32_t)gi.count>nv-base)goto bad;
        if(gi.shader==SHADER_SIMHAIR) {
            double ymax=-HUGE_VAL;
            for(j=0;j<(uint32_t)gi.count;j++) {
                const float *src=(const float *)(gi.data+(size_t)j*gi.stride+gi.posOff);
                const int16_t *packed=(const int16_t *)(vb+(size_t)(base+j)*stride);float p[3];
                if(packed[3]<=0)goto bad;
                decode(packed,p);
                for(k=0;k<3;k++)if(!hc_gr_finite(src[k]) || !hc_gr_finite(p[k]) || fabs(p[k]-src[k])>.002)goto bad;
                if(src[1]>ymax)ymax=src[1];
            }
            if(!(faceTop>-1e30) || ymax>faceTop-.03) {
                if(nranges>=64)goto bad;
                lo[nranges]=base;hi[nranges]=base+(uint32_t)gi.count;nranges++;
            }
        }
        base+=(uint32_t)gi.count;
    }
    if(base!=nv)goto bad;
    if(!nranges)return 0;
    for(i=0;i<ni/3;i++) {
        uint16_t *f=(uint16_t *)ib+3*i;
        for(k=0;k<nranges;k++)
            if(f[0]>=lo[k] && f[0]<hi[k] && f[1]>=lo[k] && f[1]<hi[k] && f[2]>=lo[k] && f[2]<hi[k]) {
                if(f[1]!=f[0] || f[2]!=f[0]){f[1]=f[2]=f[0];hidden++;}
                break;
            }
    }
    if(g_logParts || c->fullLogs<40) {
        c->fullLogs++;
        hc_log("chop type 3 (bald, full hat): all hair hidden (%d triangle(s) of %d hair part(s))",hidden,nranges);
    }
    return hidden;
bad:
    if(g_logParts || c->fullLogs<40) {
        c->fullLogs++;
        hc_log("chop type 3 (bald, full hat): hair left as it is (the drawable layout is not the supported one)");
    }
    return -1;
}
#endif

static void hc_profile_prepare_runtime(Ctx *c,const void *mb)
{
    uint32_t db,de,d,pb,pe,p,hash=2166136261u;double *tri=NULL;int nt=0,capacity=0;
    uint32_t *F=NULL;
    /* Head part / triangle cutoffs; adult values unless v7.1 finds a kid face. */
    double partMin=1.70,triMin=1.50,triXZ=.25;
    if(!c || !mb || !c->hats || !c->shell.ok || !readable(mb,MB_DRAW_END+4)) goto bad;
    db=U32(mb,MB_DRAW_BEGIN);de=U32(mb,MB_DRAW_END);
    if(de<db || (de-db)%DRAW_SIZE || de-db>DRAW_SIZE*64 || !readable((void *)db,de-db)) goto bad;
#ifdef HC_HAT_FIT
    {
        /* Child/toddler: the adult cutoffs sit .136 m and .336 m below the adult
         * face top (1.836); keep the same distances scaled to this head size. */
        double faceTop=hc_profile_face_top(db,de);
        if(faceTop>0.2 && faceTop<1.5) {
            HCAgeMap m;
            hc_age_class(&m,faceTop<HC_AGE_TODDLER_BELOW?2:1,0);
            partMin=faceTop-(HC_AGE_ADULT_FACE_TOP-1.70)*m.r[1];
            triMin=faceTop-(HC_AGE_ADULT_FACE_TOP-1.50)*m.r[1];
            triXZ=.25*.5*(m.r[0]+m.r[2]);
        }
    }
#endif
    for(d=db;d<de;d+=DRAW_SIZE) {
        pb=U32(d,DRAW_PARTS_BEGIN);pe=U32(d,DRAW_PARTS_END);
        if(pe<pb || (pe-pb)%PART_SIZE || pe-pb>PART_SIZE*256 || (pe>pb && !readable((void *)pb,pe-pb))) goto bad;
        for(p=pb;p<pe;p+=PART_SIZE) {
            uint32_t geom=U32(p,PART_GEOM);GeomInfo gi;int head=0,j,k,nf;double ymax=-HUGE_VAL;
            if(!geom || (geom&3) || !geom_info((void *)geom,&gi)) goto bad;
            if(gi.shader!=0x548394B9u || is_hat(&gi)) continue;
            for(j=0;j<gi.njoints;j++) if(gi.joints[j]==0x0F97B21Bu || gi.joints[j]==0x6E191ACBu || gi.joints[j]==0x9D5460CEu) head=1;
            if(!head) continue;
            for(j=0;j<gi.count;j++) {
                const float *q=(const float *)(gi.data+(size_t)j*gi.stride+gi.posOff);
                if(!hc_gr_finite(q[1])) goto bad;
                if(q[1]>ymax)ymax=q[1];
            }
            if(ymax<=partMin) continue;
            nf=geom_tris(&gi,&F);if(nf<0)goto bad;
            hash=hc_profile_hash(hash,geom);hash=hc_profile_hash(hash,(uint32_t)gi.count);
            hash=hc_profile_hash(hash,(uint32_t)nf);hash=hc_profile_hash(hash,(uint32_t)(uintptr_t)gi.data);
            for(j=0;j<nf;j++) {
                double points[9];int good=1;
                for(k=0;k<3;k++) {
                    const float *q=(const float *)(gi.data+(size_t)F[3*j+k]*gi.stride+gi.posOff);
                    if(!hc_gr_finite(q[0]) || !hc_gr_finite(q[1]) || !hc_gr_finite(q[2])) goto bad;
                    if(q[1]<=triMin || fabs(q[0])>=triXZ || fabs(q[2])>=triXZ)good=0;
                    points[3*k]=q[0];points[3*k+1]=q[1];points[3*k+2]=q[2];
                }
                if(!good)continue;
                if(nt>=200000)goto bad;
                if(nt==capacity) {
                    int next=capacity?capacity*2:2048;double *grown;
                    if(next>200000)next=200000;
                    grown=(double *)realloc(tri,(size_t)next*9*sizeof(double));if(!grown)goto bad;
                    tri=grown;capacity=next;
                }
                memcpy(tri+(size_t)nt*9,points,sizeof(points));nt++;
            }
            free(F);F=NULL;
        }
    }
    hash=hc_profile_hash(hash,(uint32_t)nt);
    if(c->shapeReady && c->headKey==hash) {free(tri);return;}
    c->shapeReady=0;
    if(!hc_shape_prepare(&c->shell,nt?tri:NULL,nt,&c->shape))goto bad;
    c->headKey=hash;c->shapeReady=1;
#ifdef HC_HAT_FIT
    c->shapeGeomHash=hc_shape_geometry_hash(&c->shape);
#endif
#ifdef HC_HAT_FIT
    if(g_logParts || g_builds<=60) hc_log("two-type profile: cut%d (%s, ray pick cut%d), lifted %.1f mm to the hat edge, "
        "%d/36 edge slices, coverage %d/%d, %d actual head triangle(s)",
        c->shape.cut,c->shape.rimSectors>=HC_FIT_MIN_SECTORS?"hat edge fit":"ray pick",c->shape.rayCut,
        c->shape.lift[c->shape.cut]*1000,c->shape.rimSectors,c->shape.uncovered[0],c->shape.uncovered[1],nt);
#else
    if(g_logParts || g_builds<=60) hc_log("two-type profile: cut%d, coverage %d/%d, %d actual head triangle(s)",
        c->shape.cut,c->shape.uncovered[0],c->shape.uncovered[1],nt);
#endif
    free(tri);return;
bad:
    free(F);free(tri);
    if(c)c->shapeReady=0;
}

static uint64_t hc_profile_face_key(uint32_t a,uint32_t b,uint32_t c)
{ return (uint64_t)a|((uint64_t)b<<16)|((uint64_t)c<<32); }
static int hc_profile_face_compare(const void *aa,const void *bb)
{
    const HCProfileFace *a=(const HCProfileFace *)aa,*b=(const HCProfileFace *)bb;
    if(a->key!=b->key)return a->key<b->key?-1:1;
    return a->source<b->source?-1:a->source>b->source;
}

static int chop_hidden_indices(Ctx *c,void *self,void *info)
{
    uint32_t db,de,di,draw,pb,pe,part,nv,ni,stride,ib,vb,base=0,nf=0,i;
    uint32_t *F=NULL,*sourceF=NULL,*outputF=NULL,*destination=NULL,*cursor=NULL;
    unsigned char *stageVB=NULL;uint16_t *stageIB=NULL;HCProfileFace *faces=NULL;
    HCProfilePart parts[256];int nparts=0,doubled,result=-1,hairs=0,generated=0,moved=0,fixedCuts=0,cacheHits=0,skinKept=0;
    DWORD started=GetTickCount();
    const char *why="drawable validation";
    size_t vbbytes;
    if(!c || c->mb!=self || !c->hats || c->chopFailed || !c->shell.ok || !g_enabled || g_layoutBad ||
       !readable(self,MB_DRAW_END+4) || !readable(info,0x46) || !*((uint8_t *)info+0x42))return 0;
    if(!c->shapeReady || !c->shape.ok || c->captureFailed || c->captureCount<0 || c->captureCount>256)goto done;
    db=U32(self,MB_DRAW_BEGIN);de=U32(self,MB_DRAW_END);di=U32(info,0x20);
    if(de<db || (de-db)%DRAW_SIZE || de-db>DRAW_SIZE*64 || di>=(de-db)/DRAW_SIZE || !readable((void *)db,de-db))goto done;
    draw=db+di*DRAW_SIZE;nv=U32(draw,0);ni=U32(draw,4);
    pb=U32(draw,DRAW_PARTS_BEGIN);pe=U32(draw,DRAW_PARTS_END);
    stride=*(const uint16_t *)((uint8_t *)info+0x40);ib=U32(info,0x34);vb=U32(info,0x38);
    doubled=*((uint8_t *)info+0x44)!=0;
    if(!nv || nv>65535 || !ni || ni%3 || ni>3000000 || stride<8 || stride>HC_PAYLOAD_MAX_STRIDE ||
       pe<pb || (pe-pb)%PART_SIZE || pe-pb>PART_SIZE*256 || !readable((void *)pb,pe-pb))goto done;
    vbbytes=(size_t)nv*(doubled+1)*stride;
    if(!memory_range((void *)ib,(size_t)ni*2,1) || !memory_range((void *)vb,vbbytes,1) ||
       hc_profile_ranges_overlap(ib,(size_t)ni*2,vb,vbbytes))goto done;
    sourceF=(uint32_t *)malloc((size_t)ni*sizeof(uint32_t));outputF=(uint32_t *)malloc((size_t)ni*sizeof(uint32_t));
    faces=(HCProfileFace *)malloc((size_t)(ni/3)*sizeof(*faces));
    destination=(uint32_t *)malloc((size_t)(ni/3)*sizeof(uint32_t));cursor=(uint32_t *)calloc(ni/3,sizeof(uint32_t));
    if(!sourceF || !outputF || !faces || !destination || !cursor)goto done;
    for(i=0;i<ni/3;i++)destination[i]=UINT32_MAX;
    for(part=pb;part<pe;part+=PART_SIZE) {
        uint32_t geom=U32(part,PART_GEOM),sub,j;GeomInfo gi;HCProfilePart *entry;int nt,k;
        if(nparts>=256 || !geom || (geom&3) || !geom_info((void *)geom,&gi) || (uint32_t)gi.count>nv-base)goto done;
        for(sub=gi.subs;sub<gi.subsEnd;sub+=12) {
            uint32_t indexData=U32((void *)sub,0),indexCount=U32((void *)sub,4);
            if(*(uint8_t *)(sub+8)!=2 ||
               hc_profile_ranges_overlap(vb,vbbytes,indexData,(size_t)indexCount*2) ||
               hc_profile_ranges_overlap(ib,(size_t)ni*2,indexData,(size_t)indexCount*2))goto done;
        }
        if(hc_profile_ranges_overlap(vb,vbbytes,(uint32_t)(uintptr_t)gi.data,(size_t)gi.count*gi.stride) ||
           hc_profile_ranges_overlap(ib,(size_t)ni*2,(uint32_t)(uintptr_t)gi.data,(size_t)gi.count*gi.stride))goto done;
        for(j=0;j<(uint32_t)gi.count;j++) {
            const float *src=(const float *)(gi.data+(size_t)j*gi.stride+gi.posOff);
            const int16_t *packed=(const int16_t *)(vb+(size_t)(base+j)*stride);float p[3];
            if(packed[3]<=0)goto done;
            decode(packed,p);
            for(k=0;k<3;k++)if(!hc_gr_finite(src[k]) || !hc_gr_finite(p[k]) || fabs(p[k]-src[k])>.002)goto done;
            if(doubled) {
                const int16_t *copy=(const int16_t *)(vb+(size_t)(base+j+nv)*stride);
                for(k=0;k<4;k++)if(packed[k]==INT16_MIN || copy[k]!=-packed[k])goto done;
                if(stride>8 && memcmp((const unsigned char *)packed+8,(const unsigned char *)copy+8,stride-8))goto done;
            }
        }
        nt=geom_tris(&gi,&F);if(nt<0 || (uint32_t)nt>ni/3-nf)goto done;
        entry=parts+nparts++;entry->geom=geom;entry->base=base;entry->first=nf;
        entry->count=gi.count;entry->nt=nt;entry->hair=gi.shader==SHADER_SIMHAIR;entry->layout=NULL;
        if(entry->hair && nt) {
            int matches=0;
            for(k=0;k<c->captureCount;k++) {
                const HCCapturedLayout *capture=c->captured+k;
                if(capture->geom==geom && capture->vb==vb+base*stride) {
                    if(capture->count!=gi.count || capture->stride!=(int)stride ||
                       capture->layout.stride!=(int)stride || !hc_payload_valid(&capture->layout))goto done;
                    entry->layout=&capture->layout;matches++;
                }
            }
            if(matches!=1)goto done;
        }
        for(j=0;j<(uint32_t)nt;j++) {
            const uint32_t *f=F+3*j;
            for(k=0;k<3;k++)sourceF[3*nf+k]=base+f[k];
            faces[nf].key=hc_profile_face_key(sourceF[3*nf],sourceF[3*nf+1],sourceF[3*nf+2]);
            faces[nf].source=nf;nf++;
        }
        free(F);F=NULL;base+=(uint32_t)gi.count;
    }
    if(base!=nv || nf!=ni/3)goto done;
    qsort(faces,nf,sizeof(*faces),hc_profile_face_compare);
    /* Full original IB multiset validation, one destination occurrence per
     * source occurrence. No geometry math or writes have happened yet. */
    for(i=0;i<nf;i++) {
        const uint16_t *f=(const uint16_t *)ib+3*i;uint64_t key;uint32_t lo=0,hi=nf,index;
        if(f[0]>=nv || f[1]>=nv || f[2]>=nv)goto done;
        key=hc_profile_face_key(f[0],f[1],f[2]);
        while(lo<hi) {uint32_t mid=lo+(hi-lo)/2;if(faces[mid].key<key)lo=mid+1;else hi=mid;}
        if(lo>=nf || faces[lo].key!=key || cursor[lo]>=nf-lo)goto done;
        index=lo+cursor[lo]++;
        if(faces[index].key!=key || destination[faces[index].source]!=UINT32_MAX)goto done;
        destination[faces[index].source]=i;
    }
    for(i=0;i<nf;i++)if(destination[i]==UINT32_MAX)goto done;
    stageVB=(unsigned char *)malloc(vbbytes);stageIB=(uint16_t *)malloc((size_t)ni*2);
    if(!stageVB || !stageIB)goto done;
    memcpy(stageVB,(void *)vb,vbbytes);memcpy(outputF,sourceF,(size_t)ni*sizeof(uint32_t));
    for(i=0;i<(uint32_t)nparts;i++) {
        const HCProfilePart *entry=parts+i;HCProfileApplyStats stats;uint32_t j;
        if(!entry->hair || !entry->nt)continue;
        F=(uint32_t *)malloc((size_t)entry->nt*3*sizeof(uint32_t));if(!F)goto done;
        for(j=0;j<(uint32_t)entry->nt*3;j++)F[j]=sourceF[(size_t)entry->first*3+j]-entry->base;
#ifdef HC_HAT_FIT
        {
            const unsigned char *partVB=(const unsigned char *)(vb+(size_t)entry->base*stride);
            uint64_t key=hc_cache_key(&c->shape,c->shapeGeomHash,partVB,entry->count,(int)stride,F,entry->nt,entry->layout);
            const HCCacheEntry *hit=hc_cache_find(&c->cache,key,entry->count,entry->nt,(int)stride);
            if(hit) {
                memcpy(stageVB+(size_t)entry->base*stride,hit->vb,(size_t)entry->count*stride);
                memcpy(outputF+(size_t)entry->first*3,hit->F,(size_t)entry->nt*3*sizeof(uint32_t));
                stats=hit->stats;cacheHits++;
            } else {
                if(!hc_profile_apply(&c->shape,partVB,entry->count,(int)stride,F,entry->nt,entry->layout,
                      stageVB+(size_t)entry->base*stride,outputF+(size_t)entry->first*3,&stats)) {
                    why=hc_apply_fail?hc_apply_fail:"hair cut";goto done;
                }
                hc_cache_store(&c->cache,key,stageVB+(size_t)entry->base*stride,entry->count,(int)stride,
                               outputF+(size_t)entry->first*3,entry->nt,&stats);
            }
        }
#else
        if(!hc_profile_apply(&c->shape,(const unsigned char *)(vb+(size_t)entry->base*stride),
              entry->count,(int)stride,F,entry->nt,entry->layout,stageVB+(size_t)entry->base*stride,
              outputF+(size_t)entry->first*3,&stats)) {
            why=hc_apply_fail?hc_apply_fail:"hair cut";goto done;
        }
#endif
        fixedCuts+=stats.fixedCut;skinKept+=stats.skinTolerated;
        for(j=0;j<(uint32_t)entry->nt*3;j++) {
            uint32_t *value=outputF+(size_t)entry->first*3+j;
            if(*value>=(uint32_t)entry->count)goto done;
            *value+=entry->base;
        }
        free(F);F=NULL;hairs++;generated+=stats.generated;moved+=stats.moved;
    }
    if(!hairs) {result=0;goto done;}
    for(i=0;i<nf;i++) {
        uint32_t j,slot=destination[i];
        for(j=0;j<3;j++) {
            uint32_t value=outputF[3*i+j];if(value>=nv)goto done;
            stageIB[3*slot+j]=(uint16_t)value;
        }
    }
    if(doubled)for(i=0;i<nv;i++) {
        int16_t *front=(int16_t *)(stageVB+(size_t)i*stride),*back=(int16_t *)(stageVB+(size_t)(i+nv)*stride);int k;
        memcpy(back,front,stride);
        for(k=0;k<4;k++) {if(front[k]==INT16_MIN)goto done;back[k]=(int16_t)-front[k];}
    }
    /* Recheck permissions immediately before the sole commit; game build is
     * serialized for this drawable, so source mapping cannot change midway. */
    if(!memory_range((void *)vb,vbbytes,1) || !memory_range((void *)ib,(size_t)ni*2,1))goto done;
    memcpy((void *)vb,stageVB,vbbytes);memcpy((void *)ib,stageIB,(size_t)ni*2);
    result=generated+1;
done:
    free(F);free(sourceF);free(outputF);free(destination);free(cursor);free(faces);free(stageVB);free(stageIB);
    if(result<0) {
        if(c)c->chopFailed=1;
        if(g_logParts || g_builds<=60)hc_log("two-type chop skipped: %s",why);
    } else if(result>0 && (g_logParts || g_builds<=60)) hc_log("two-type chop: %d hair part(s), %d vertices moved, %d exact edge vertices%s%s%s, %lu ms",
        hairs,moved,generated,fixedCuts?" (fallback cut on a hair with few spare vertices)":"",
        skinKept?" (kept a small front skin overlap, at most 3 mm)":"",
        cacheHits?" (reused from an identical earlier build)":"",(unsigned long)(GetTickCount()-started));
    return result;
}
#endif
