/* Approved two-chop shaper: direct radial queries, cut-relative proportional
 * falloff, and a 2 mm head floor. Only X/Z change. State MUST start zeroed.
 * Owns headTri; borrows Shell hatTri. Release before rebuilding that Shell.
 * Preparation failure preserves the previous state and its owned geometry.
 */
#ifndef HAT_CHOP_PROFILE_SHAPE_H
#define HAT_CHOP_PROFILE_SHAPE_H
#include "hat_chop_gravity.h"
#include "hat_chop_profile_refs.h"
#include "hat_chop_profile_meshes.h"
#include "hat_chop_profile_fit.h"
#include "hat_chop_ray_index.h"
#include "hat_chop_age.h"
#ifdef HC_HAT_FIT
#include "hat_chop_hat_fit.h"
#endif

typedef struct {
    int ok, cut, uncovered[2], actualHead;
    int wallSamples, headWallConflicts;
    double maxHeadWallConflict;
    double scale[3], shift[3], center[2], plane[3], fadeDepth;
    double *headTri;
    int nhead;
    const double *hatTri;
    int nhat;
    /* v7 hat fit: per-cut lift that hides the cut inside the hat edge */
    int rayCut, rimSectors;
    double lift[2], rimWorst[2], rimMean[2];
    /* v7.3 update: the hat's lowest edge per 10-degree slice (HUGE_VAL = not measured),
     * slices about x = 0, z = rimZ; rimOk when enough slices were measured */
    double rimY[36], rimZ;
    int rimOk;
    /* v7 speed-up: height buckets for head, hat and reference ray queries
     * (empty = old all-triangle loop; results are the same either way) */
    HCRayIndex headIdx, hatIdx, refIdx[2], refHeadIdx;
    HCAgeMap age;                      /* v7.1: child/toddler mapping (kid 0 = adult) */
} HCShapeState;

static void hc_shape_release(HCShapeState *s)
{
    if(!s)return;
    free(s->headTri);
    hc_ray_index_free(&s->headIdx);hc_ray_index_free(&s->hatIdx);
    hc_ray_index_free(&s->refIdx[0]);hc_ray_index_free(&s->refIdx[1]);hc_ray_index_free(&s->refHeadIdx);
    memset(s,0,sizeof(*s));
}

/* One triangle of a horizontal ray query; identical text in both loops below. */
#define HC_SHAPE_RAY_TEST(t) { \
        double e1[3]={t[3]-t[0],t[4]-t[1],t[5]-t[2]}; \
        double e2[3]={t[6]-t[0],t[7]-t[1],t[8]-t[2]}; \
        double h[3]={-dz*e2[1],dz*e2[0]-dx*e2[2],dx*e2[1]}; \
        double det=e1[0]*h[0]+e1[1]*h[1]+e1[2]*h[2]; \
        double s[3],q[3],u,v,d; \
        if(fabs(det)<=1e-12)continue; \
        s[0]=cx-t[0];s[1]=y-t[1];s[2]=cz-t[2]; \
        u=(s[0]*h[0]+s[1]*h[1]+s[2]*h[2])/det; \
        if(u<0||u>1)continue; \
        q[0]=s[1]*e1[2]-s[2]*e1[1]; \
        q[1]=s[2]*e1[0]-s[0]*e1[2]; \
        q[2]=s[0]*e1[1]-s[1]*e1[0]; \
        v=(dx*q[0]+dz*q[2])/det; \
        if(v<0||u+v>1)continue; \
        d=(e2[0]*q[0]+e2[1]*q[1]+e2[2]*q[2])/det; \
        if(d>minimum&&((outermost&&d>best)||(!outermost&&d<best)))best=d; \
    }

/* Normalized horizontal direction. Zero means no valid radial hit. With a
 * built index for exactly these triangles, only the bucket at y is tested. */
static double hc_shape_ray_mode_ix(const HCRayIndex *ix,const double *tri,int nt,double cx,double cz,
        double y,double dx,double dz,double minimum,int outermost)
{
    int i;
    double best=outermost?0:HUGE_VAL;
    if(ix && ix->item && ix->nt==nt) {
        uint32_t j,b;
        if(!(y>=ix->ymin && y<=ix->ymax))return 0;
        b=(uint32_t)hc_ray_bucket(ix,y);
        for(j=ix->start[b];j<ix->start[b+1];j++) {
            const double *t=tri+(size_t)ix->item[j]*9;
            if(y<ix->lo[ix->item[j]] || y>ix->hi[ix->item[j]])continue;
            HC_SHAPE_RAY_TEST(t)
        }
        return hc_gr_finite(best)?best:0;
    }
    for(i=0;i<nt;i++) {
        const double *t=tri+(size_t)i*9;
        HC_SHAPE_RAY_TEST(t)
    }
    return hc_gr_finite(best)?best:0;
}

static double hc_shape_ray_mode(const double *tri,int nt,double cx,double cz,
        double y,double dx,double dz,double minimum,int outermost)
{
    return hc_shape_ray_mode_ix(NULL,tri,nt,cx,cz,y,dx,dz,minimum,outermost);
}

/* Inverse affine preserves nonuniform source fits exactly. */
static double hc_shape_source_ray_ix(const HCShapeState *s,const HCRayIndex *ix,const double *tri,int nt,
        double dx,double dz,double gameY,int outermost)
{
    double sx=dx/s->scale[0],sz=dz/s->scale[2];
    double inverseRadius=sqrt(sx*sx+sz*sz),value;
    if(!(inverseRadius>0))return 0;
    value=hc_shape_ray_mode_ix(ix,tri,nt,0,0,(gameY-s->shift[1])/s->scale[1],
                           sx/inverseRadius,sz/inverseRadius,.005*inverseRadius,outermost);
    return value>0?value/inverseRadius:0;
}

static double hc_shape_source_ray(const HCShapeState *s,const double *tri,int nt,
        double dx,double dz,double gameY,int outermost)
{
    return hc_shape_source_ray_ix(s,NULL,tri,nt,dx,dz,gameY,outermost);
}

static void hc_shape_update_plane(HCShapeState *s)
{
    s->plane[0]=s->scale[1]*hc_ref_source_plane[s->cut][0]/s->scale[0];
    s->plane[1]=s->scale[1]*hc_ref_source_plane[s->cut][1]/s->scale[2];
    s->plane[2]=s->scale[1]*hc_ref_source_plane[s->cut][2]+s->shift[1]-
        s->plane[0]*s->shift[0]-s->plane[1]*s->shift[2]+s->lift[s->cut];
}

#ifdef HC_HAT_FIT
/* Rim fit (see hat_chop_hat_fit.h). Needs scale/shift set; fills lift/rim
 * fields and returns the cut whose hidden plane follows the hat edge best,
 * or rayCut when the edge cannot be measured well enough. */
static int hc_shape_rim_pick(HCShapeState *st,const double *tri,int ntri,int rayCut)
{
    double rim[HC_FIT_SECTORS][3],score[2];int c;
    st->rayCut=rayCut;st->lift[0]=st->lift[1]=0;
    st->rimSectors=hc_fit_rim(tri,ntri,rim,&st->age);
    for(c=0;c<HC_FIT_SECTORS;c++)st->rimY[c]=rim[c][1];
    st->rimZ=hc_age_z(&st->age,HC_FIT_HEAD_Z);
    st->rimOk=st->rimSectors>=HC_FIT_MIN_SECTORS;
    if(st->rimSectors<HC_FIT_MIN_SECTORS)return rayCut;
    for(c=0;c<2;c++) {
        st->cut=c;st->lift[c]=0;hc_shape_update_plane(st);
        hc_fit_score(st->plane,rim,&st->rimWorst[c],&st->rimMean[c],&st->lift[c]);
        score[c]=st->rimMean[c]+st->lift[c];
    }
    return score[1]<score[0]?1:0;
}
#endif

/* Optional explicit affine overrides recognized-port calibration. age NULL =
 * adult; otherwise the child/toddler mapping used by the hat-edge slices. */
static int hc_shape_prepare_full(Shell *hat,const double *headTri,int nhead,
        const double *scale,const double *shift,const HCAgeMap *age,HCShapeState *out)
{
    HCShapeState state;
    double headLo=HUGE_VAL,headHi=-HUGE_VAL,fittedScale[3],fittedShift[3];
    int j,k;
    if(!hat||!hat->ok||!hat->tri||!out||nhead<0||nhead>200000||
       (nhead>0&&!headTri)||(!nhead&&headTri))return 0;
    memset(&state,0,sizeof(state));
    if(age && age->kid)state.age=*age;
    else state.age.r[0]=state.age.r[1]=state.age.r[2]=1;
    if(!scale&&!shift&&hc_profile_fit_lookup(hat->tri,hat->ntri,fittedScale,fittedShift)) {
        scale=fittedScale;shift=fittedShift;
    }
    for(k=0;k<3;k++) {
        state.scale[k]=scale?scale[k]:hc_ref_default_scale[k];
        state.shift[k]=shift?shift[k]:hc_ref_default_shift[k];
        if(!(state.scale[k]>.25&&state.scale[k]<4)||
           !hc_gr_finite(state.shift[k])||fabs(state.shift[k])>10)return 0;
    }
    for(j=0;j<nhead*9;j++)if(!hc_gr_finite(headTri[j])||fabs(headTri[j])>10)return 0;
    state.actualHead=nhead>0;
    if(nhead) {
        for(j=0;j<nhead*3;j++) {
            double y=headTri[(size_t)j*3+1];
            if(y<headLo)headLo=y;
            if(y>headHi)headHi=y;
        }
    } else {
        for(j=0;j<hc_ref_head_mesh_count*3;j++) {
            double y=hc_ref_head_mesh[(size_t)j*3+1]*state.scale[1]+state.shift[1];
            if(y<headLo)headLo=y;
            if(y>headHi)headHi=y;
        }
    }
    state.fadeDepth=.35*(headHi-headLo);
    if(!(state.fadeDepth>.01&&state.fadeDepth<.5))return 0;
    state.center[0]=state.shift[0];state.center[1]=state.shift[2];
    state.cut=hc_ref_select_cut(hat->tri,hat->ntri,state.scale,state.shift,state.uncovered);
#ifdef HC_HAT_FIT
    state.cut=hc_shape_rim_pick(&state,hat->tri,hat->ntri,state.cut);
#endif
    if(state.cut<0)return 0;
    hc_shape_update_plane(&state);
    if(nhead) {
        state.headTri=(double*)malloc((size_t)nhead*9*sizeof(double));
        if(!state.headTri)return 0;
        memcpy(state.headTri,headTri,(size_t)nhead*9*sizeof(double));
    }
    state.nhead=nhead;
    state.hatTri=hat->tri;state.nhat=hat->ntri;state.ok=1;
#if defined(HC_HAT_FIT) && !defined(HC_NO_RAY_INDEX)
    /* A failed build leaves that index empty: the full loop is used instead. */
    hc_ray_index_build(&state.hatIdx,state.hatTri,state.nhat);
    if(nhead)hc_ray_index_build(&state.headIdx,state.headTri,state.nhead);
    hc_ray_index_build(&state.refIdx[0],hc_ref_meshes[0],hc_ref_mesh_counts[0]);
    hc_ray_index_build(&state.refIdx[1],hc_ref_meshes[1],hc_ref_mesh_counts[1]);
    hc_ray_index_build(&state.refHeadIdx,hc_ref_head_mesh,hc_ref_head_mesh_count);
#endif
    hc_shape_release(out);
    *out=state;
    return 1;
}

static int hc_shape_prepare_affine(Shell *hat,const double *headTri,int nhead,
        const double *scale,const double *shift,HCShapeState *out)
{
    return hc_shape_prepare_full(hat,headTri,nhead,scale,shift,NULL,out);
}

/* Adults: unchanged. v7.1: a child or toddler head (scalp top under 1.55 m)
 * maps the adult TS4 calibration onto that head before everything else. */
static int hc_shape_prepare(Shell *hat,const double *headTri,int nhead,HCShapeState *out)
{
#ifdef HC_HAT_FIT
    HCAgeMap age;
    hc_age_from_head(headTri,nhead,&age);
    if(age.kid) {
        double s[3],t[3];
        hc_age_affine(&age,hc_ref_default_scale,hc_ref_default_shift,s,t);
        return hc_shape_prepare_full(hat,headTri,nhead,s,t,&age,out);
    }
#endif
    return hc_shape_prepare_affine(hat,headTri,nhead,NULL,NULL,out);
}

static int hc_shape_force_cut(HCShapeState *s,int cut)
{
    if(!s||!s->ok||cut<0||cut>1)return 0;
    s->cut=cut;
    hc_shape_update_plane(s);
    return 1;
}

/* Success is 1; invalid input returns 0 with Q untouched. Q==P supported.
 * Exact approved semantics: no field smoothing, crown/full-weight override,
 * radial plausibility filter, or rim-derived fade. Cutting is caller-owned.
 */
static int hc_shape_points(const HCShapeState *s,const float *P,int nv,float *Q)
{
    int i,k;
    if(!s||!s->ok||!s->hatTri||s->nhat<=0||!P||!Q||nv<=0||nv>HC_MAX_HAIR_VERT)return 0;
    for(i=0;i<nv;i++)for(k=0;k<3;k++)if(!hc_gr_finite(P[(size_t)i*3+k]))return 0;
    for(i=0;i<nv;i++) {
        const float *p=P+(size_t)i*3;
        float *q=Q+(size_t)i*3;
        double x=p[0],y=p[1],z=p[2],dx=x-s->center[0],dz=z-s->center[1];
        double radius=sqrt(dx*dx+dz*dz),weight,cut,probeY,head,reference,wall,target,newRadius;
        q[0]=p[0];q[1]=p[1];q[2]=p[2];
        cut=s->plane[0]*x+s->plane[1]*z+s->plane[2];
        weight=1-(cut-y)/s->fadeDepth;
        if(weight<=0||radius<1e-9)continue;
        if(weight>1)weight=1;
        weight=weight*weight*(3-2*weight);
        dx/=radius;dz/=radius;
        probeY=y<cut-.001?y:cut-.001;
        head=s->nhead?hc_shape_ray_mode_ix(&s->headIdx,s->headTri,s->nhead,s->center[0],s->center[1],probeY,dx,dz,.005,0):
            hc_shape_source_ray_ix(s,&s->refHeadIdx,hc_ref_head_mesh,hc_ref_head_mesh_count,dx,dz,probeY,0);
        if(!(head>0))continue;
        reference=hc_shape_source_ray_ix(s,&s->refIdx[s->cut],hc_ref_meshes[s->cut],hc_ref_mesh_counts[s->cut],dx,dz,probeY,1);
        wall=hc_shape_ray_mode_ix(&s->hatIdx,s->hatTri,s->nhat,s->center[0],s->center[1],cut+.001,dx,dz,.005,0);
        target=HUGE_VAL;
        if(reference>0)target=reference+.002;
        if(wall>0&&wall-.0005<target)target=wall-.0005;
        if(target<head+.002)target=head+.002;
        if(!hc_gr_finite(target)||radius<=target)continue;
        newRadius=radius+weight*(target-radius);
        q[0]=(float)(s->center[0]+dx*newRadius);
        q[2]=(float)(s->center[1]+dz*newRadius);
    }
    return 1;
}
#endif /* HAT_CHOP_PROFILE_SHAPE_H */
