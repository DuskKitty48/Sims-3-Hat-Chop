/* Exact planar triangle clipping in fixed vertex/index capacities.
 * y=a*x+b*z+c; keep the nonpositive side. No game-hook integration here.
 *
 * New edge vertices use ONLY above vertices unreferenced by every source face
 * that is not fully above. All original below/on-plane positions stay exact.
 * Source index edges remain independent across UV seams. Endpoint-on-plane
 * intersections reuse that original endpoint. A clipped face's first triangle
 * stays in its original face slot; quad extras use fully-above face slots.
 * Remaining fully-above slots become degenerate. Unchanged face winding and
 * slots are preserved. No coordinate welding or extra buffer capacity is used.
 *
 * provenance: Q[dst]=(1-t)*P[a]+t*P[b], with original endpoint indices a<b.
 * Endpoint indices cannot be donor slots, so payload interpolation can use a
 * staged original VB without read-after-write hazards. Attributes are NOT
 * handled here. The adapter must interpolate them and commit VB+IB together.
 *
 * Return 1 on success, 0 on invalid input, allocation/capacity/alias failure.
 * ALL outputs, including count/stats, remain unchanged on failure. Q==P and
 * outF==F are supported; all other output aliases are rejected. Callers supply
 * accessible buffers. provenanceCapacity may be nv; count is required.
 */
#ifndef HAT_CHOP_BISECT_H
#define HAT_CHOP_BISECT_H
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#ifndef HC_BISECT_MAX_VERT
#define HC_BISECT_MAX_VERT 200000
#endif
#ifndef HC_BISECT_MAX_TRI
#define HC_BISECT_MAX_TRI 1000000
#endif
#ifndef HC_BISECT_ALLOC
#define HC_BISECT_ALLOC malloc
#endif
#ifndef HC_BISECT_FREE
#define HC_BISECT_FREE free
#endif

typedef struct { uint32_t dst, a, b; double t; } HCBisectVertex;
typedef struct {
    uint32_t above_vertices, fully_above_faces, clipped_faces, quad_faces;
    uint32_t available_vertex_slots, intersection_vertices, output_triangles;
    double max_plane_residual;
} HCBisectStats;
typedef struct { uint32_t a, b; } HCBisectIndexEdge;

static int hc_bisect_finite(double x) { return x==x && x<=DBL_MAX && x>=-DBL_MAX; }
static int hc_bisect_overlap(const void *a,size_t an,const void *b,size_t bn)
{
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
    if (!an || !bn) return 0;
    if (an>UINTPTR_MAX-x || bn>UINTPTR_MAX-y) return 1;
    return x<y+bn && y<x+an;
}
static int hc_bisect_edge_compare(const void *a,const void *b)
{
    const HCBisectIndexEdge *x=(const HCBisectIndexEdge *)a,*y=(const HCBisectIndexEdge *)b;
    if (x->a!=y->a) return x->a<y->a?-1:1;
    return x->b<y->b?-1:x->b>y->b;
}
static uint32_t hc_bisect_find(const HCBisectIndexEdge *e,size_t n,uint32_t a,uint32_t b)
{
    size_t lo=0,hi=n;
    if (a>b) { uint32_t t=a;a=b;b=t; }
    while (lo<hi) {
        size_t m=lo+(hi-lo)/2;
        if (e[m].a<a || (e[m].a==a && e[m].b<b)) lo=m+1; else hi=m;
    }
    return lo<n && e[lo].a==a && e[lo].b==b ? (uint32_t)lo : UINT32_MAX;
}

/* The token version plans polygon size before allocating destination slots.
 * High-bit tokens represent strict indexed edge intersections, not vertices. */
static int hc_bisect_polygon_tokens(const uint32_t *f,const double *d,uint64_t out[6])
{
    int k,n=0;
    for (k=0;k<3;k++) {
        uint32_t a=f[k],b=f[(k+1)%3];
        int inside_a=d[a]<=0,inside_b=d[b]<=0;
        uint64_t token;
        if (inside_a && (!n || out[n-1]!=a)) out[n++]=a;
        if (inside_a==inside_b) continue;
        if (d[a]==0) token=a;
        else if (d[b]==0) token=b;
        else token=UINT64_C(0x8000000000000000) | ((uint64_t)(a<b?a:b)<<32) | (a<b?b:a);
        if (!n || out[n-1]!=token) out[n++]=token;
    }
    if (n>1 && out[n-1]==out[0]) n--;
    return n;
}

static int hc_bisect_plane(const float *P,int nv,const uint32_t *F,int nt,
                            double a,double b,double c,float *Q,uint32_t *outF,
                            HCBisectVertex *provenance,int provenanceCapacity,
                            int *provenanceCount,HCBisectStats *stats)
{
    double *d=NULL,normal[3],offset,scale;
    unsigned char *used=NULL,*hidden=NULL;
    HCBisectIndexEdge *edges=NULL;
    HCBisectVertex *records=NULL;
    float *q=NULL; uint32_t *fout=NULL;
    HCBisectStats result;
    size_t pb,fb,rb,edge_count=0,unique=0,i,j,next_vertex=0,next_face=0;
    const void *outs[5];size_t sizes[5];int k,l,ok=0;
    if (!P || !Q || !provenanceCount || nv<=0 || nv>HC_BISECT_MAX_VERT || nt<0 || nt>HC_BISECT_MAX_TRI ||
        (nt && (!F || !outF)) || provenanceCapacity<0 || provenanceCapacity>HC_BISECT_MAX_VERT ||
        (provenanceCapacity && !provenance) || !hc_bisect_finite(a) || !hc_bisect_finite(b) || !hc_bisect_finite(c)) return 0;
    if ((size_t)nv>SIZE_MAX/(3*sizeof(double)) || (size_t)nt>SIZE_MAX/(3*sizeof(HCBisectIndexEdge)) ||
        (size_t)provenanceCapacity>SIZE_MAX/sizeof(HCBisectVertex)) return 0;
    pb=(size_t)nv*3*sizeof(float);fb=(size_t)nt*3*sizeof(uint32_t);
    rb=(size_t)provenanceCapacity*sizeof(HCBisectVertex);
    outs[0]=Q;sizes[0]=pb;outs[1]=outF;sizes[1]=fb;outs[2]=provenance;sizes[2]=rb;
    outs[3]=provenanceCount;sizes[3]=sizeof(*provenanceCount);outs[4]=stats;sizes[4]=stats?sizeof(*stats):0;
    for (k=0;k<5;k++) {
        for (l=0;l<k;l++) if (hc_bisect_overlap(outs[k],sizes[k],outs[l],sizes[l])) return 0;
        if (!(k==0 && Q==P) && hc_bisect_overlap(outs[k],sizes[k],P,pb)) return 0;
        if (!(k==1 && outF==F) && hc_bisect_overlap(outs[k],sizes[k],F,fb)) return 0;
    }
    for (i=0;i<(size_t)nv*3;i++) if (!hc_bisect_finite(P[i])) return 0;
    for (i=0;i<(size_t)nt*3;i++) if (F[i]>=(uint32_t)nv) return 0;
    memset(&result,0,sizeof(result));
    scale=fmax(1.0,fmax(fabs(a),fabs(b)));
    normal[0]=-a/scale;normal[1]=1.0/scale;normal[2]=-b/scale;offset=-c/scale;
    d=(double *)HC_BISECT_ALLOC((size_t)nv*sizeof(double));
    used=(unsigned char *)HC_BISECT_ALLOC((size_t)nv);
    if (nt) {
        hidden=(unsigned char *)HC_BISECT_ALLOC((size_t)nt);
        edges=(HCBisectIndexEdge *)HC_BISECT_ALLOC((size_t)nt*3*sizeof(*edges));
    }
    if (!d || !used || (nt && (!hidden || !edges))) goto done;
    memset(used,0,(size_t)nv);
    for (i=0;i<(size_t)nv;i++) {
        d[i]=normal[0]*P[3*i]+normal[1]*P[3*i+1]+normal[2]*P[3*i+2]+offset;
        if (!hc_bisect_finite(d[i])) goto done;
        result.above_vertices+=d[i]>0;
    }
    for (i=0;i<(size_t)nt;i++) {
        const uint32_t *f=F+3*i;uint64_t tokens[6];int n,above=0;
        for (k=0;k<3;k++) above+=d[f[k]]>0;
        hidden[i]=(unsigned char)(above==3);
        result.fully_above_faces+=above==3;
        if (above==3) continue;
        for (k=0;k<3;k++) used[f[k]]=1;
        if (!above) { result.output_triangles++;continue; }
        result.clipped_faces++;
        n=hc_bisect_polygon_tokens(f,d,tokens);
        if (n>=3) result.output_triangles+=(uint32_t)(n-2);
        result.quad_faces+=n==4;
        for (k=0;k<3;k++) {
            uint32_t x=f[k],y=f[(k+1)%3];
            if (!((d[x]>0 && d[y]<0) || (d[y]>0 && d[x]<0))) continue;
            edges[edge_count].a=x<y?x:y;edges[edge_count++].b=x<y?y:x;
        }
    }
    for (i=0;i<(size_t)nv;i++) result.available_vertex_slots+=d[i]>0 && !used[i];
    if (edge_count) qsort(edges,edge_count,sizeof(*edges),hc_bisect_edge_compare);
    for (i=0;i<edge_count;i++) {
        if (unique && edges[i].a==edges[unique-1].a && edges[i].b==edges[unique-1].b) continue;
        edges[unique++]=edges[i];
    }
    if (unique>result.available_vertex_slots || unique>(size_t)provenanceCapacity ||
        result.quad_faces>result.fully_above_faces) goto done;
    q=(float *)HC_BISECT_ALLOC(pb);
    if (nt) fout=(uint32_t *)HC_BISECT_ALLOC(fb);
    if (unique) records=(HCBisectVertex *)HC_BISECT_ALLOC(unique*sizeof(*records));
    if (!q || (nt && !fout) || (unique && !records)) goto done;
    memcpy(q,P,pb);
    if (nt) memcpy(fout,F,fb);
    for (i=0;i<unique;i++) {
        double da=fabs(d[edges[i].a]),db=fabs(d[edges[i].b]),t,residual;
        while (next_vertex<(size_t)nv && !(d[next_vertex]>0 && !used[next_vertex])) next_vertex++;
        if (next_vertex==(size_t)nv) goto done;
        if (da>db) t=1/(1+db/da);
        else { double ratio=da/db;t=ratio/(1+ratio); }
        if (!hc_bisect_finite(t) || t<0 || t>1) goto done;
        records[i].dst=(uint32_t)next_vertex++;records[i].a=edges[i].a;records[i].b=edges[i].b;records[i].t=t;
        for (k=0;k<3;k++) {
            double x=(double)P[(size_t)records[i].a*3+k]+t*((double)P[(size_t)records[i].b*3+k]-P[(size_t)records[i].a*3+k]);
            if (!hc_bisect_finite(x) || x < -FLT_MAX || x > FLT_MAX) goto done;
            q[(size_t)records[i].dst*3+k]=(float)x;
        }
        residual=fabs(normal[0]*q[(size_t)records[i].dst*3]+normal[1]*q[(size_t)records[i].dst*3+1]+
                      normal[2]*q[(size_t)records[i].dst*3+2]+offset);
        if (!hc_bisect_finite(residual)) goto done;
        if (residual>result.max_plane_residual) result.max_plane_residual=residual;
    }
    for (i=0;i<(size_t)nt;i++) if (hidden[i]) fout[3*i+1]=fout[3*i+2]=fout[3*i];
    for (i=0;i<(size_t)nt;i++) {
        uint64_t tokens[6];uint32_t polygon[6];int n;
        const uint32_t *source=F+3*i;
        if (hidden[i] || (d[source[0]]<=0 && d[source[1]]<=0 && d[source[2]]<=0)) continue;
        n=hc_bisect_polygon_tokens(source,d,tokens);
        for (k=0;k<n;k++) {
            if (!(tokens[k]&UINT64_C(0x8000000000000000))) polygon[k]=(uint32_t)tokens[k];
            else {
                uint32_t ea=(uint32_t)((tokens[k]>>32)&UINT64_C(0x7fffffff)),eb=(uint32_t)tokens[k];
                uint32_t ix=hc_bisect_find(edges,unique,ea,eb);
                if (ix==UINT32_MAX) goto done;
                polygon[k]=records[ix].dst;
            }
        }
        if (n<3) fout[3*i+1]=fout[3*i+2]=fout[3*i];
        else {
            fout[3*i]=polygon[0];fout[3*i+1]=polygon[1];fout[3*i+2]=polygon[2];
            if (n==4) {
                while (next_face<(size_t)nt && !hidden[next_face]) next_face++;
                if (next_face==(size_t)nt) goto done;
                j=next_face++;
                fout[3*j]=polygon[0];fout[3*j+1]=polygon[2];fout[3*j+2]=polygon[3];
            }
        }
    }
    result.intersection_vertices=(uint32_t)unique;
    memcpy(Q,q,pb);if (nt) memcpy(outF,fout,fb);
    if (unique) memcpy(provenance,records,unique*sizeof(*records));
    *provenanceCount=(int)unique;if (stats) *stats=result;ok=1;
done:
    HC_BISECT_FREE(d);HC_BISECT_FREE(used);HC_BISECT_FREE(hidden);HC_BISECT_FREE(edges);
    HC_BISECT_FREE(q);HC_BISECT_FREE(fout);HC_BISECT_FREE(records);
    return ok;
}
#endif
