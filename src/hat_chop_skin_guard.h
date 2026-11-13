/* Local candidate guard for NEW front-of-head penetration after the approved
 * profile shaper/bisector. Include after hat_chop_profile_shape.h.
 *
 * This does not change the approved shaper, cut plane, topology, Y, or payload.
 * Only NEW or already shape-moved vertices in the frozen front sector
 * (z-centerZ)/radius > .5 may move, and movement is radially OUTWARD in X/Z.
 * Actual head geometry is queried at actual sample Y, using the OUTERMOST
 * intersection. First-hit face/scalp overlaps can lie well inside visible skin.
 *
 * New vertices get 2mm clearance. Retained moved vertices and face-interior
 * samples correct only negative clearance WORSENED from original provenance;
 * preexisting authored nose/face penetration is not inflated into new tabs.
 * Triangle samples cover three edge midpoints and the centroid. These finite
 * samples are a diagnostic constraint, not proof of continuous nonintersection.
 *
 * Y buckets only select exact triangle candidates: no sampled radius field or
 * interpolated head shape. Eligible coincident seams share outward movement.
 * Both inputs remain immutable; failures leave output/statistics unchanged.
 */
#ifndef HAT_CHOP_SKIN_GUARD_H
#define HAT_CHOP_SKIN_GUARD_H
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "hat_chop_ray_index.h"

#define HC_SKIN_Y_BUCKETS 128
#define HC_SKIN_MAX_BUCKET_ITEMS 4194304u
#define HC_SKIN_CLEARANCE .002
#define HC_SKIN_TOL .000002
#define HC_SKIN_MAX_OUTWARD .05
#define HC_SKIN_ITERATIONS 16
#define HC_SKIN_EDGE_CLEARANCE .0001
#define HC_SKIN_MAX_EDGE_KNOTS 256

typedef struct {
    int before,after,unresolved,moved,iterations,skippedNoHead;
    int beforeVertices,afterVertices,beforeFaces,afterFaces,eligible,faceSamples;
    int beforeEdges,afterEdges;
    uint32_t headQueries;
    double maxOutward,maxBefore,maxAfter;
    double radiusTests,edgeCalls,edgeTests;   /* v7 cost counters (diagnostics only) */
} HCSkinGuardStats;

typedef struct {
    const double *tri;
    int nt;
    double ymin,ymax,*low,*high;
    uint32_t start[HC_SKIN_Y_BUCKETS+1],*item;
    /* v7 speed-up for edge crossings: candidates from the Y buckets, tested in
     * ascending triangle order exactly like the old all-triangle loop */
    uint32_t *stamp,*cand,query;
    HCRayIndex2 ix2;                   /* v7: height x direction buckets around the head axis */
    double edgeTests;                  /* v7 cost counter (diagnostics only) */
} HCSkinHead;
typedef struct { uint32_t vertex,bits[3]; } HCSkinSeam;
typedef struct { uint32_t face; double baseline[4],target[4]; unsigned char front[4],active[4]; } HCSkinFace;

static int hc_skin_finite(double value)
{ return value==value && value>-1.0e100 && value<1.0e100; }
static int hc_skin_bin(const HCSkinHead *h,double y)
{
    int b=(int)floor((y-h->ymin)*(HC_SKIN_Y_BUCKETS/(h->ymax-h->ymin)));
    if(b<0)b=0;
    if(b>=HC_SKIN_Y_BUCKETS)b=HC_SKIN_Y_BUCKETS-1;
    return b;
}
static void hc_skin_head_free(HCSkinHead *h)
{ free(h->low);free(h->high);free(h->item);free(h->stamp);free(h->cand);hc_ray2_free(&h->ix2);memset(h,0,sizeof(*h)); }
static int hc_skin_u32_compare(const void *a,const void *b)
{ uint32_t x=*(const uint32_t *)a,y=*(const uint32_t *)b; return x<y?-1:x>y; }
static int hc_skin_head_build(const HCShapeState *s,HCSkinHead *h)
{
    uint32_t count[HC_SKIN_Y_BUCKETS]={0},cursor[HC_SKIN_Y_BUCKETS];int i,k,b;
    memset(h,0,sizeof(*h));h->tri=s->headTri;h->nt=s->nhead;h->ymin=1.0e100;h->ymax=-1.0e100;
    h->low=(double *)malloc((size_t)h->nt*sizeof(double));h->high=(double *)malloc((size_t)h->nt*sizeof(double));
    if(!h->low || !h->high)return 0;
    for(i=0;i<h->nt;i++) {
        const double *t=h->tri+(size_t)i*9;double lo=t[1],hi=t[1];
        for(k=0;k<9;k++)if(!hc_skin_finite(t[k]) || fabs(t[k])>10)return 0;
        for(k=1;k<3;k++){double y=t[3*k+1];if(y<lo)lo=y;if(y>hi)hi=y;}
        h->low[i]=lo;h->high[i]=hi;
        if(lo<h->ymin)h->ymin=lo;
        if(hi>h->ymax)h->ymax=hi;
    }
    if(!(h->ymax-h->ymin>1e-9))return 0;
    for(i=0;i<h->nt;i++) {
        int lo=hc_skin_bin(h,h->low[i]),hi=hc_skin_bin(h,h->high[i]);
        if(lo>0)lo--;
        if(hi<HC_SKIN_Y_BUCKETS-1)hi++;
        for(b=lo;b<=hi;b++)count[b]++;
    }
    for(b=0;b<HC_SKIN_Y_BUCKETS;b++) {
        if(count[b]>HC_SKIN_MAX_BUCKET_ITEMS-h->start[b])return 0;
        h->start[b+1]=h->start[b]+count[b];cursor[b]=h->start[b];
    }
    h->item=(uint32_t *)malloc((size_t)h->start[HC_SKIN_Y_BUCKETS]*sizeof(uint32_t));if(!h->item)return 0;
    for(i=0;i<h->nt;i++) {
        int lo=hc_skin_bin(h,h->low[i]),hi=hc_skin_bin(h,h->high[i]);
        if(lo>0)lo--;
        if(hi<HC_SKIN_Y_BUCKETS-1)hi++;
        for(b=lo;b<=hi;b++)h->item[cursor[b]++]=(uint32_t)i;
    }
#if defined(HC_HAT_FIT) && !defined(HC_NO_RAY_INDEX)
    /* optional: without these the edge test uses the full loop */
    h->stamp=(uint32_t *)calloc((size_t)h->nt,sizeof(uint32_t));
    h->cand=(uint32_t *)malloc((size_t)h->nt*sizeof(uint32_t));
    if(!h->stamp || !h->cand){free(h->stamp);free(h->cand);h->stamp=h->cand=NULL;}
    hc_ray2_build(&h->ix2,h->tri,h->nt,h->low,h->high,s->center[0],s->center[1]);
#endif
    return 1;
}

/* One head triangle of the outermost radius query; same text in both loops. */
#define HC_SKIN_RADIUS_TEST(i) { \
        const double *t;double e1[3],e2[3],hh[3],ss[3],qq[3],det,u,v,d; \
        if(y<h->low[i] || y>h->high[i])continue; \
        t=h->tri+(size_t)i*9; \
        e1[0]=t[3]-t[0];e1[1]=t[4]-t[1];e1[2]=t[5]-t[2]; \
        e2[0]=t[6]-t[0];e2[1]=t[7]-t[1];e2[2]=t[8]-t[2]; \
        hh[0]=-dz*e2[1];hh[1]=dz*e2[0]-dx*e2[2];hh[2]=dx*e2[1]; \
        det=e1[0]*hh[0]+e1[1]*hh[1]+e1[2]*hh[2];if(fabs(det)<=1e-12)continue; \
        ss[0]=cx-t[0];ss[1]=y-t[1];ss[2]=cz-t[2]; \
        u=(ss[0]*hh[0]+ss[1]*hh[1]+ss[2]*hh[2])/det;if(u<0 || u>1)continue; \
        qq[0]=ss[1]*e1[2]-ss[2]*e1[1];qq[1]=ss[2]*e1[0]-ss[0]*e1[2];qq[2]=ss[0]*e1[1]-ss[1]*e1[0]; \
        v=(dx*qq[0]+dz*qq[2])/det;if(v<0 || u+v>1)continue; \
        d=(e2[0]*qq[0]+e2[1]*qq[1]+e2[2]*qq[2])/det; \
        if(d>.005 && d>best)best=d; \
    }

static double hc_skin_radius(const HCSkinHead *h,double cx,double cz,double y,
                              double dx,double dz,HCSkinGuardStats *stats)
{
    double best=0;uint32_t j;int bin;
    stats->headQueries++;
    if(y<h->ymin || y>h->ymax)return 0;
    if(h->ix2.item && cx==h->ix2.cx && cz==h->ix2.cz) {
        uint32_t b=(uint32_t)(hc_ray2_ybin(&h->ix2,y)*HC_RAY2_A+hc_ray2_abin(dx,dz));
        stats->radiusTests+=h->ix2.start[b+1]-h->ix2.start[b];
        for(j=h->ix2.start[b];j<h->ix2.start[b+1];j++) {
            uint32_t i=h->ix2.item[j];
            HC_SKIN_RADIUS_TEST(i)
        }
        return best;
    }
    bin=hc_skin_bin(h,y);
    for(j=h->start[bin];j<h->start[bin+1];j++) {
        uint32_t i=h->item[j];
        HC_SKIN_RADIUS_TEST(i)
    }
    return best;
}

static double hc_skin_clearance(const HCSkinHead *head,const HCShapeState *state,
                                 const double p[3],HCSkinGuardStats *stats)
{
    double dx=p[0]-state->center[0],dz=p[2]-state->center[1],r=sqrt(dx*dx+dz*dz),skin;
    if(r<1e-9)return 1.0e99;
    skin=hc_skin_radius(head,state->center[0],state->center[1],p[1],dx/r,dz/r,stats);
    return skin>0?r-skin:1.0e99;
}
static int hc_skin_front(const HCShapeState *s,const double p[3])
{
    double dx=p[0]-s->center[0],dz=p[2]-s->center[1],r=sqrt(dx*dx+dz*dz);
    return r>1e-9 && dz/r>.5;
}
static int hc_skin_seam_compare(const void *aa,const void *bb)
{
    const HCSkinSeam *a=(const HCSkinSeam *)aa,*b=(const HCSkinSeam *)bb;int k;
    for(k=0;k<3;k++)if(a->bits[k]!=b->bits[k])return a->bits[k]<b->bits[k]?-1:1;
    return a->vertex<b->vertex?-1:a->vertex>b->vertex;
}
static void hc_skin_sample(const float *P,const uint32_t *f,int sample,double p[3])
{
    static const double w[4][3]={{.5,.5,0},{0,.5,.5},{.5,0,.5},{1.0/3,1.0/3,1.0/3}};int k,j;
    for(k=0;k<3;k++){p[k]=0;for(j=0;j<3;j++)p[k]+=w[sample][j]*P[(size_t)f[j]*3+k];}
}

static int hc_skin_knot(double *values,int *count,double value)
{
    int i,j;
    if(!(value>=0 && value<=1))return 1;
    for(i=0;i<*count;i++)if(fabs(values[i]-value)<1e-10)return 1;
    if(*count>=HC_SKIN_MAX_EDGE_KNOTS)return 0;
    for(i=0;i<*count && values[i]<value;i++) {}
    for(j=*count;j>i;j--)values[j]=values[j-1];
    values[i]=value;(*count)++;return 1;
}

/* Exact segment/head crossings split the segment into inside/outside spans.
 * Include frozen approved-front boundaries, then inspect each span midpoint.
 * The eighth-edge grid is supplemental; it alone can miss a thin head facet.
 * Y/AABB rejection is exact and avoids heavy tests against unrelated faces. */
static int hc_skin_edge_knots(const HCSkinHead *head,const HCShapeState *state,
        const float *a,const float *b,const float *approvedA,const float *approvedB,
        double knots[HC_SKIN_MAX_EDGE_KNOTS],int *count)
{
    double d[3],lo[3],hi[3];int i,k;
    *count=0;
    for(i=0;i<=8;i++)if(!hc_skin_knot(knots,count,i/8.0))return 0;
    for(k=0;k<3;k++){d[k]=(double)b[k]-a[k];lo[k]=a[k]<b[k]?a[k]:b[k];hi[k]=a[k]>b[k]?a[k]:b[k];}
    for(i=-1;i<=1;i+=2) {
        double va=(approvedA[0]-state->center[0])+i*1.7320508075688772935*(approvedA[2]-state->center[1]);
        double vb=(approvedB[0]-state->center[0])+i*1.7320508075688772935*(approvedB[2]-state->center[1]);
        if(fabs(vb-va)>1e-15 && !hc_skin_knot(knots,count,-va/(vb-va)))return 0;
    }
#define HC_SKIN_EDGE_TEST(i) { \
        const double *t;double e1[3],e2[3],hh[3],ss[3],qq[3],det,u,v,at; \
        if(hi[1]<head->low[i] || lo[1]>head->high[i])continue; \
        t=head->tri+(size_t)i*9; \
        for(k=0;k<3;k+=2) { \
            double mn=t[k],mx=t[k];int j; \
            for(j=1;j<3;j++){double q=t[3*j+k];if(q<mn)mn=q;if(q>mx)mx=q;} \
            if(hi[k]<mn || lo[k]>mx)break; \
        } \
        if(k<3)continue; \
        for(k=0;k<3;k++){e1[k]=t[3+k]-t[k];e2[k]=t[6+k]-t[k];ss[k]=a[k]-t[k];} \
        hh[0]=d[1]*e2[2]-d[2]*e2[1];hh[1]=d[2]*e2[0]-d[0]*e2[2];hh[2]=d[0]*e2[1]-d[1]*e2[0]; \
        det=e1[0]*hh[0]+e1[1]*hh[1]+e1[2]*hh[2];if(fabs(det)<=1e-12)continue; \
        u=(ss[0]*hh[0]+ss[1]*hh[1]+ss[2]*hh[2])/det;if(u< -1e-10 || u>1+1e-10)continue; \
        qq[0]=ss[1]*e1[2]-ss[2]*e1[1];qq[1]=ss[2]*e1[0]-ss[0]*e1[2];qq[2]=ss[0]*e1[1]-ss[1]*e1[0]; \
        v=(d[0]*qq[0]+d[1]*qq[1]+d[2]*qq[2])/det;if(v< -1e-10 || u+v>1+1e-10)continue; \
        at=(e2[0]*qq[0]+e2[1]*qq[1]+e2[2]*qq[2])/det; \
        if(!hc_skin_knot(knots,count,at))return 0; \
    }
    if(head->stamp && head->cand && head->ix2.item &&
       head->ix2.cx==state->center[0] && head->ix2.cz==state->center[1]) {
        /* Any crossing point lies in the edge's height range and in its direction
         * range seen from the head axis, so its triangle is listed in one of these
         * buckets. Candidates are then tested in ascending order, as before. */
        HCSkinHead *mh=(HCSkinHead *)head;uint32_t n=0,j,c;int yb,ab,y0,y1,a0=0,a1=0,all=0;
        double seg[9]={a[0],a[1],a[2],b[0],b[1],b[2],b[0],b[1],b[2]};
        if(++mh->query==0){memset(mh->stamp,0,(size_t)head->nt*sizeof(uint32_t));mh->query=1;}
        y0=hc_ray2_ybin(&head->ix2,lo[1]-HC_RAY_PAD);y1=hc_ray2_ybin(&head->ix2,hi[1]+HC_RAY_PAD);
        hc_ray2_span(seg,head->ix2.cx,head->ix2.cz,&all,&a0,&a1);
        for(yb=y0;yb<=y1;yb++)for(ab=all?0:a0;;ab=(ab+1)%HC_RAY2_A) {
            uint32_t bucket=(uint32_t)(yb*HC_RAY2_A+ab);
            for(j=head->ix2.start[bucket];j<head->ix2.start[bucket+1];j++) {
                uint32_t k2=head->ix2.item[j];
                if(mh->stamp[k2]!=mh->query){mh->stamp[k2]=mh->query;mh->cand[n++]=k2;}
            }
            if(all?ab==HC_RAY2_A-1:ab==a1)break;
        }
        mh->edgeTests+=n;
        if(n>32)qsort(mh->cand,n,sizeof(uint32_t),hc_skin_u32_compare);
        else for(c=1;c<n;c++){uint32_t v=mh->cand[c];j=c;while(j>0 && mh->cand[j-1]>v){mh->cand[j]=mh->cand[j-1];j--;}mh->cand[j]=v;}
        for(c=0;c<n;c++) {
            i=(int)head->cand[c];
            HC_SKIN_EDGE_TEST(i)
        }
        return 1;
    }
    for(i=0;i<head->nt;i++) HC_SKIN_EDGE_TEST(i)
    return 1;
}

/* Read-only when move==NULL. Otherwise stage minimum outward support for NEW
 * protected edge penetration. A 0.1mm edge margin fits SHORT4 quantization and
 * avoids imposing 2mm through a tiny coefficient near a frozen endpoint. */
static int hc_skin_edge_constraints(const HCSkinHead *head,const HCShapeState *state,
        const float *Q,const float *approved,const float *baseline,const HCSkinFace *faces,int nface,
        const uint32_t *F,const unsigned char *eligible,const double *direction,double *move,
        HCSkinGuardStats *stats,int *violations,double *maximum)
{
    int i,j,k;
    *violations=0;*maximum=0;
    for(i=0;i<nface;i++) {
        const uint32_t *face=F+(size_t)faces[i].face*3;
        for(j=0;j<3;j++) {
            uint32_t va=face[j],vb=face[(j+1)%3];double knots[HC_SKIN_MAX_EDGE_KNOTS];int count,sample;
            if(va==vb || (!eligible[va] && !eligible[vb]))continue;
            stats->edgeCalls++;
            if(!hc_skin_edge_knots(head,state,Q+(size_t)va*3,Q+(size_t)vb*3,
                  approved+(size_t)va*3,approved+(size_t)vb*3,knots,&count))return 0;
            for(sample=0;sample<count*2-1;sample++) {
                double t=sample%2?.5*(knots[sample/2]+knots[sample/2+1]):knots[sample/2];
                double p[3],frozen[3],base[3],clear,baseClear,target,deficit,dx,dz,r,ca,cb,denom;
                for(k=0;k<3;k++) {
                    p[k]=(1-t)*Q[(size_t)va*3+k]+t*Q[(size_t)vb*3+k];
                    frozen[k]=(1-t)*approved[(size_t)va*3+k]+t*approved[(size_t)vb*3+k];
                    base[k]=(1-t)*baseline[(size_t)va*3+k]+t*baseline[(size_t)vb*3+k];
                }
                if(!hc_skin_front(state,frozen))continue;
                clear=hc_skin_clearance(head,state,p,stats);if(clear>=-HC_SKIN_TOL)continue;
                baseClear=hc_skin_clearance(head,state,base,stats);if(clear>=baseClear-HC_SKIN_TOL)continue;
                target=baseClear<HC_SKIN_EDGE_CLEARANCE?baseClear:HC_SKIN_EDGE_CLEARANCE;
                deficit=target-clear;if(deficit<=HC_SKIN_TOL)continue;
                (*violations)++;if(deficit>*maximum)*maximum=deficit;
                if(!move)continue;
                dx=p[0]-state->center[0];dz=p[2]-state->center[1];r=sqrt(dx*dx+dz*dz);if(r<1e-9)continue;
                dx/=r;dz/=r;
                ca=eligible[va]?(1-t)*(dx*direction[2*va]+dz*direction[2*va+1]):0;
                cb=eligible[vb]?t*(dx*direction[2*vb]+dz*direction[2*vb+1]):0;
                if(ca<0)ca=0;
                if(cb<0)cb=0;
                denom=ca*ca+cb*cb;if(denom<1e-12)continue;
                if(ca>0 && deficit*ca/denom>move[va])move[va]=deficit*ca/denom;
                if(cb>0 && deficit*cb/denom>move[vb])move[vb]=deficit*cb/denom;
            }
        }
    }
    return 1;
}

/* originalP is untouched decoded input. baselineProvenance is a copy with
 * each generated donor slot set to (1-t)*originalP[a]+t*originalP[b]. newMask
 * flags those generated slots. Faces are the final fixed-capacity bisect F.
 * Q==approvedQ is supported; other output aliases must have stride-sized
 * accessible memory. A successful candidate may have unresolved finite-sample
 * constraints; caller must inspect statistics before deciding to use it.
 * Missing actual head is an unchanged success, explicitly skippedNoHead=1. */
static int hc_skin_guard(const HCShapeState *state,const float *originalP,
        const float *approvedQ,int nv,const uint32_t *F,int nt,const unsigned char *newMask,
        const float *baselineProvenance,float *outQ,HCSkinGuardStats *stats)
{
    static const double weights[4][3]={{.5,.5,0},{0,.5,.5},{.5,0,.5},{1.0/3,1.0/3,1.0/3}};
    HCSkinHead head;HCSkinGuardStats result;float *Q=NULL;
    unsigned char *eligible=NULL,*used=NULL;double *direction=NULL,*radius=NULL,*initial=NULL,*target=NULL,*move=NULL;
    HCSkinSeam *seam=NULL;HCSkinFace *faces=NULL;int nseam=0,nface=0,i,j,k,iteration,ok=0;
    memset(&head,0,sizeof(head));memset(&result,0,sizeof(result));
    if(!state || !state->ok || !originalP || !approvedQ || !F || !newMask || !baselineProvenance || !outQ ||
       nv<=0 || nv>200000 || nt<0 || nt>1000000 || !hc_skin_finite(state->center[0]) || !hc_skin_finite(state->center[1]))return 0;
    for(i=0;i<nv;i++)for(k=0;k<3;k++)if(!hc_skin_finite(originalP[3*i+k]) ||
        !hc_skin_finite(approvedQ[3*i+k]) || !hc_skin_finite(baselineProvenance[3*i+k]))return 0;
    for(i=0;i<nt*3;i++)if(F[i]>=(uint32_t)nv)return 0;
    if(!state->actualHead || !state->headTri || state->nhead<=0) {
        result.skippedNoHead=1;memmove(outQ,approvedQ,(size_t)nv*3*sizeof(float));if(stats)*stats=result;return 1;
    }
    if(state->nhead>200000 || !hc_skin_head_build(state,&head))goto done;
    Q=(float *)malloc((size_t)nv*3*sizeof(float));eligible=(unsigned char *)calloc(nv,1);used=(unsigned char *)calloc(nv,1);
    direction=(double *)calloc((size_t)nv*2,sizeof(double));radius=(double *)calloc(nv,sizeof(double));
    initial=(double *)calloc(nv,sizeof(double));target=(double *)malloc((size_t)nv*sizeof(double));move=(double *)calloc(nv,sizeof(double));
    seam=(HCSkinSeam *)malloc((size_t)nv*sizeof(*seam));faces=(HCSkinFace *)calloc((size_t)(nt?nt:1),sizeof(*faces));
    if(!Q || !eligible || !used || !direction || !radius || !initial || !target || !move || !seam || !faces)goto done;
    memcpy(Q,approvedQ,(size_t)nv*3*sizeof(float));for(i=0;i<nt*3;i++)used[F[i]]=1;
    for(i=0;i<nv;i++) {
        double p[3]={Q[3*i],Q[3*i+1],Q[3*i+2]},dx=p[0]-state->center[0],dz=p[2]-state->center[1];
        double r=sqrt(dx*dx+dz*dz),clear,base[3],baseClear;
        target[i]=-1.0e99;radius[i]=initial[i]=r;
        if(!used[i] || !hc_skin_front(state,p) || (!newMask[i] &&
           !memcmp(originalP+(size_t)i*3,approvedQ+(size_t)i*3,3*sizeof(float))))continue;
        eligible[i]=1;direction[2*i]=dx/r;direction[2*i+1]=dz/r;
        seam[nseam].vertex=(uint32_t)i;memcpy(seam[nseam].bits,Q+(size_t)i*3,12);nseam++;
        clear=hc_skin_clearance(&head,state,p,&result);
        if(clear>1e98)continue;
        if(newMask[i])target[i]=r-clear+HC_SKIN_CLEARANCE;
        else {
            for(k=0;k<3;k++)base[k]=originalP[3*i+k];
            baseClear=hc_skin_clearance(&head,state,base,&result);
            if(clear<0 && clear<baseClear-HC_SKIN_TOL)target[i]=r-clear+(baseClear<HC_SKIN_CLEARANCE?baseClear:HC_SKIN_CLEARANCE);
        }
        if(target[i]-r>HC_SKIN_TOL){result.beforeVertices++;if(target[i]-r>result.maxBefore)result.maxBefore=target[i]-r;}
    }
    result.eligible=nseam;qsort(seam,(size_t)nseam,sizeof(*seam),hc_skin_seam_compare);
    for(i=0;i<nt;i++) {
        const uint32_t *f=F+3*i;HCSkinFace *face;int any=0;
        if(f[0]==f[1] || f[1]==f[2] || f[2]==f[0])continue;
        for(j=0;j<3;j++)if(eligible[f[j]])any=1;
        if(!any)continue;
        face=faces+nface++;face->face=(uint32_t)i;
        for(j=0;j<4;j++) {
            double p[3],base[3],clear;
            hc_skin_sample(Q,f,j,p);if(!hc_skin_front(state,p))continue;
            face->front[j]=1;result.faceSamples++;hc_skin_sample(baselineProvenance,f,j,base);
            face->baseline[j]=hc_skin_clearance(&head,state,base,&result);
            clear=hc_skin_clearance(&head,state,p,&result);
            if(clear<0 && clear<face->baseline[j]-HC_SKIN_TOL) {
                double t=face->baseline[j]<HC_SKIN_CLEARANCE?face->baseline[j]:HC_SKIN_CLEARANCE;
                face->active[j]=1;face->target[j]=t;result.beforeFaces++;
                if(t-clear>result.maxBefore)result.maxBefore=t-clear;
            }
        }
    }
    result.before=result.beforeVertices+result.beforeFaces;
    for(iteration=0;iteration<HC_SKIN_ITERATIONS;iteration++) {
        int changed=0,edgeViolations;double edgeMaximum;
        memset(move,0,(size_t)nv*sizeof(double));
        for(i=0;i<nv;i++)if(eligible[i] && target[i]-radius[i]>HC_SKIN_TOL)move[i]=target[i]-radius[i];
        for(i=0;i<nface;i++) {
            HCSkinFace *face=faces+i;const uint32_t *f=F+(size_t)face->face*3;
            for(j=0;j<4;j++)if(face->front[j]) {
                double p[3],clear,deficit,rr,nx,nz,denom=0,coefficient[3];
                hc_skin_sample(Q,f,j,p);clear=hc_skin_clearance(&head,state,p,&result);
                if(!face->active[j] && clear<0 && clear<face->baseline[j]-HC_SKIN_TOL) {
                    face->active[j]=1;face->target[j]=face->baseline[j]<HC_SKIN_CLEARANCE?face->baseline[j]:HC_SKIN_CLEARANCE;
                }
                if(!face->active[j] || (deficit=face->target[j]-clear)<=HC_SKIN_TOL)continue;
                nx=p[0]-state->center[0];nz=p[2]-state->center[1];rr=sqrt(nx*nx+nz*nz);if(rr<1e-9)continue;
                nx/=rr;nz/=rr;
                for(k=0;k<3;k++) {
                    uint32_t v=f[k];double c=weights[j][k]*(nx*direction[2*v]+nz*direction[2*v+1]);
                    coefficient[k]=eligible[v] && c>1e-9?c:0;denom+=coefficient[k]*coefficient[k];
                }
                if(denom<=1e-12)continue;
                for(k=0;k<3;k++)if(coefficient[k]>0) {
                    uint32_t v=f[k];double amount=deficit*coefficient[k]/denom;if(amount>move[v])move[v]=amount;
                }
            }
        }
        if(!hc_skin_edge_constraints(&head,state,Q,approvedQ,baselineProvenance,faces,nface,F,
              eligible,direction,move,&result,&edgeViolations,&edgeMaximum))goto done;
        if(iteration==0) {
            result.beforeEdges=edgeViolations;result.before+=edgeViolations;
            if(edgeMaximum>result.maxBefore)result.maxBefore=edgeMaximum;
        }
        /* Identical approved coordinates on UV seams receive the same shift. */
        for(i=0;i<nseam;) {
            int end=i+1;double amount=move[seam[i].vertex];
            while(end<nseam && !memcmp(seam[end].bits,seam[i].bits,12)) {
                if(move[seam[end].vertex]>amount)amount=move[seam[end].vertex];
                end++;
            }
            for(j=i;j<end;j++)move[seam[j].vertex]=amount;
            i=end;
        }
        for(i=0;i<nv;i++)if(eligible[i] && move[i]>HC_SKIN_TOL) {
            double amount=move[i],room=HC_SKIN_MAX_OUTWARD-(radius[i]-initial[i]);
            if(amount>room)amount=room;
            if(amount<=HC_SKIN_TOL)continue;
            radius[i]+=amount;Q[3*i]=(float)(state->center[0]+direction[2*i]*radius[i]);
            Q[3*i+2]=(float)(state->center[1]+direction[2*i+1]*radius[i]);changed=1;
        }
        result.iterations=iteration+1;if(!changed)break;
    }
    for(i=0;i<nv;i++)if(eligible[i]) {
        double p[3]={Q[3*i],Q[3*i+1],Q[3*i+2]},r=hypot(p[0]-state->center[0],p[2]-state->center[1]);
        double delta=r-initial[i],deficit=target[i]-r;
        if(memcmp(Q+(size_t)i*3,approvedQ+(size_t)i*3,12))result.moved++;
        if(delta>result.maxOutward)result.maxOutward=delta;
        if(deficit>HC_SKIN_TOL){result.afterVertices++;if(deficit>result.maxAfter)result.maxAfter=deficit;}
    }
    for(i=0;i<nface;i++)for(j=0;j<4;j++)if(faces[i].active[j]) {
        double p[3],clear,deficit;hc_skin_sample(Q,F+(size_t)faces[i].face*3,j,p);
        clear=hc_skin_clearance(&head,state,p,&result);deficit=faces[i].target[j]-clear;
        if(deficit>HC_SKIN_TOL){result.afterFaces++;if(deficit>result.maxAfter)result.maxAfter=deficit;}
    }
    {
        double edgeMaximum;
        if(!hc_skin_edge_constraints(&head,state,Q,approvedQ,baselineProvenance,faces,nface,F,
              eligible,direction,NULL,&result,&result.afterEdges,&edgeMaximum))goto done;
        if(edgeMaximum>result.maxAfter)result.maxAfter=edgeMaximum;
    }
    result.after=result.unresolved=result.afterVertices+result.afterFaces+result.afterEdges;
    for(i=0;i<nv;i++) {
        if(!eligible[i] && memcmp(Q+(size_t)i*3,approvedQ+(size_t)i*3,12))goto done;
        if(memcmp(Q+(size_t)i*3+1,approvedQ+(size_t)i*3+1,sizeof(float)))goto done;
        for(k=0;k<3;k++)if(!hc_skin_finite(Q[3*i+k]))goto done;
    }
    result.edgeTests=head.edgeTests;
    memcpy(outQ,Q,(size_t)nv*3*sizeof(float));if(stats)*stats=result;ok=1;
done:
    hc_skin_head_free(&head);free(Q);free(eligible);free(used);free(direction);free(radius);free(initial);free(target);free(move);free(seam);free(faces);
    return ok;
}
#endif
