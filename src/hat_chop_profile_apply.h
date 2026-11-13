/* Shared version-5 pipeline: shape, actual bisect, then packed interpolation.
 * Every input buffer is read-only. Callers commit outVB/outF only on success.
 */
#ifndef HAT_CHOP_PROFILE_APPLY_H
#define HAT_CHOP_PROFILE_APPLY_H
#include "hat_chop_profile_shape.h"
#include "hat_chop_bisect.h"
#include "hat_chop_payload.h"
#ifdef HC_FRONT_SKIN_GUARD
#include "hat_chop_skin_guard.h"
#endif
#ifdef HC_HAT_FIT
#include "hat_chop_fixed_cut.h"
#endif

/* v7.1: a few small front-skin leftovers (up to HC_SKIN_ACCEPT_SPOTS spots, each
 * at most HC_SKIN_ACCEPT m deep) keep the chop; skipping it would show the full
 * hair through the hat. Seen: adults 0.35-0.49 mm, child + long wavy hair 2.1 mm. */
#ifndef HC_SKIN_ACCEPT
#define HC_SKIN_ACCEPT 0.003
#endif
#ifndef HC_SKIN_ACCEPT_SPOTS
#define HC_SKIN_ACCEPT_SPOTS 3
#endif

/* Last failure step of hc_profile_apply, for the log only (best effort). */
static const char *volatile hc_apply_fail = 0;
#define HC_FAIL(stage) do { hc_apply_fail = (stage); goto done; } while (0)

typedef struct {
    int cut, generated, moved, fixedCut, skinTolerated;
    int frontKept;      /* fallback used when reshaped hair went into the face: v7.3 update: 1 = only hidden hair
                           touches, 2 = hair below the hat edge kept as made, 3 = front hair kept as made
                           (7.3's level 1), 4 = only the cut (7.3's level 2) */
    double maxPlaneError, maxWarpedBoundaryDeviation, maxPackedPositionError;
    HCBisectStats bisect;
#ifdef HC_FRONT_SKIN_GUARD
    HCSkinGuardStats skin;
#endif
} HCProfileApplyStats;

static int hc_profile_store_position(unsigned char *v, const float *p)
{
    int k;
    int16_t *s = (int16_t *)v;
    if (s[3] <= 0) return 0;
    for (k = 0; k < 3; k++) {
        double value = (double)p[k] * s[3];
        if (!hc_gr_finite(value) || value < INT16_MIN + .5 || value > INT16_MAX - .5) return 0;
    }
    for (k = 0; k < 3; k++) s[k] = (int16_t)lrint((double)p[k] * s[3]);
    return 1;
}

/* Output sizes equal their inputs. The bisector reuses ONLY vertices/faces
 * belonging to the removed top, and returns a source-edge interpolation map.
 * Unknown vertex declarations, insufficient spare slots, and invalid geometry
 * return zero without writing either caller output buffer or the statistics.
 */
static int hc_profile_apply(const HCShapeState *state, const unsigned char *vb,
                            int nv, int stride, const uint32_t *F, int nt,
                            const HCPayloadLayout *layout, unsigned char *outVB,
                            uint32_t *outF, HCProfileApplyStats *stats)
{
    float *P = NULL, *shaped = NULL, *Q = NULL, *boundary = NULL;
    uint32_t *indices = NULL;
    unsigned char *vertices = NULL, *isNew = NULL;
    HCBisectVertex *edges = NULL;
    HCProfileApplyStats result;
#ifdef HC_FRONT_SKIN_GUARD
    float *skinBaseline = NULL;
#ifdef HC_HAT_FIT
    float *approved = NULL;
    int level;
    HCSkinCover cover;
    const HCSkinCover *coverp = NULL;
#endif
#endif
    int i, k, count = 0, ok = 0;
    size_t bytes;
    if (!state || !vb || !F || !layout || !outVB || !outF ||
        nv <= 0 || nv > HC_MAX_HAIR_VERT || nt <= 0 || nt > HC_MAX_HAIR_TRI ||
        stride < 8 || stride > 1024 || layout->stride != stride || !hc_payload_valid(layout)) return 0;
    bytes = (size_t)nv * stride;
    P = (float *)malloc((size_t)nv * 3 * sizeof(float));
    shaped = (float *)malloc((size_t)nv * 3 * sizeof(float));
    Q = (float *)malloc((size_t)nv * 3 * sizeof(float));
    indices = (uint32_t *)malloc((size_t)nt * 3 * sizeof(uint32_t));
    vertices = (unsigned char *)malloc(bytes);
    isNew = (unsigned char *)calloc((size_t)nv, 1);
    edges = (HCBisectVertex *)malloc((size_t)nv * sizeof(*edges));
    boundary = (float *)malloc((size_t)nv * 3 * sizeof(float));
    hc_apply_fail = "memory";
    if (!P || !shaped || !Q || !indices || !vertices || !isNew || !edges || !boundary) goto done;
    memset(&result, 0, sizeof(result));
    for (i = 0; i < nv; i++) {
        const int16_t *v = (const int16_t *)(vb + (size_t)i * stride);
        if (v[3] <= 0) goto done;
        for (k = 0; k < 3; k++) P[3*i+k] = (float)v[k] / v[3];
    }
    if (!hc_shape_points(state, P, nv, shaped)) HC_FAIL("shaping");
    if (hc_bisect_plane(shaped, nv, F, nt, state->plane[0], state->plane[1], state->plane[2],
                        Q, indices, edges, nv, &count, &result.bisect)) {
        /* Match the approved reference: reshape the newly bisected border too.
         * This deliberately keeps its Y while moving X/Z toward the scalp, just
         * as proportional editing after bisect can bend the original cut line. */
        for (i = 0; i < count; i++)
            memcpy(boundary + (size_t)i * 3, Q + (size_t)edges[i].dst * 3, 3 * sizeof(float));
        if (count && !hc_shape_points(state, boundary, count, boundary)) HC_FAIL("border shaping");
        for (i = 0; i < count; i++)
            memcpy(Q + (size_t)edges[i].dst * 3, boundary + (size_t)i * 3, 3 * sizeof(float));
    } else {
#ifdef HC_HAT_FIT
        /* Too few spare slots for the exact cut: cut with the existing vertices
         * instead of skipping the whole chop (full hair through the hat). */
        uint32_t *movedIndex = (uint32_t *)malloc((size_t)nv * sizeof(uint32_t));
        int nm;
        if (!movedIndex) HC_FAIL("memory");
        nm = hc_fixed_cut(shaped, nv, F, nt, state->plane[0], state->plane[1], state->plane[2],
                          Q, indices, movedIndex);
        if (nm < 0) { free(movedIndex); HC_FAIL("fallback cut"); }
        for (i = 0; i < nm; i++)
            memcpy(boundary + (size_t)i * 3, Q + (size_t)movedIndex[i] * 3, 3 * sizeof(float));
        if (nm && !hc_shape_points(state, boundary, nm, boundary)) { free(movedIndex); HC_FAIL("border shaping"); }
        for (i = 0; i < nm; i++)
            memcpy(Q + (size_t)movedIndex[i] * 3, boundary + (size_t)i * 3, 3 * sizeof(float));
        free(movedIndex);
        count = 0;
        result.fixedCut = 1;
#else
        HC_FAIL("exact cut needs more spare vertices");
#endif
    }
    for (i = 0; i < count; i++) {
        const HCBisectVertex *e = edges + i;
        if (e->dst >= (uint32_t)nv || e->a >= (uint32_t)nv || e->b >= (uint32_t)nv || isNew[e->dst]) HC_FAIL("cut map");
        isNew[e->dst] = 1;
    }
#ifdef HC_FRONT_SKIN_GUARD
    /* Original-space provenance gives each new edge point a fair before/after
     * skin comparison; an unrelated donor vertex is never the baseline. */
    skinBaseline = (float *)malloc((size_t)nv * 3 * sizeof(float));
    if (!skinBaseline) goto done;
    memcpy(skinBaseline, P, (size_t)nv * 3 * sizeof(float));
    for (i = 0; i < count; i++) {
        const HCBisectVertex *e = edges + i;
        for (k = 0; k < 3; k++)
            skinBaseline[(size_t)e->dst*3+k] = (float)((1-e->t)*P[(size_t)e->a*3+k] + e->t*P[(size_t)e->b*3+k]);
    }
#ifdef HC_HAT_FIT
    approved = (float *)malloc((size_t)nv * 3 * sizeof(float));
    if (!approved) HC_FAIL("memory");
    memcpy(approved, Q, (size_t)nv * 3 * sizeof(float));
#endif
    if (!hc_skin_guard(state, P, Q, nv, indices, nt, isNew, skinBaseline, Q, &result.skin))
        HC_FAIL("front skin guard");
    if (result.skin.skippedNoHead) HC_FAIL("front skin guard: no head mesh");
#ifdef HC_HAT_FIT
    /* v7.3 (2026-11-13): bangs and front locks that the reshaping pulled into the
     * face no longer throw the whole chop away (the full hair showed through the
     * hat, and the failed chop was recalculated on every rebuild: lag). The
     * reshaping only moves X/Z, so putting a vertex's X/Z back where the hair's
     * maker placed it (baseline: the original position, or the original edge
     * point for new cut vertices) can never put it deeper into the face than the
     * hair itself was. Y stays, so the cut stays.
     * v7.3 update (same day): 7.3 put ALL front hair back, also the locks between the
     * cut and the hat edge, which then poked through a cap's front band. Hair
     * above the hat's lower edge is under the hat: the hat hides it touching the
     * forehead. So the fallback now checks only hair below the hat edge (the
     * visible part, over the face) and restores only that first.
     * Level 1: everything stays reshaped; only hair below the hat edge is checked.
     * Level 2: hair below the hat edge at the front goes back where it was made.
     * Level 3: all front hair goes back (7.3 level 1).
     * Level 4: no reshaping at all, only the cut (7.3 level 2).
     * Levels 1-2 need the measured hat edge; without it 3-4 run as in 7.3. */
    if (state->rimOk) {
        memcpy(cover.y, state->rimY, sizeof(cover.y));
        cover.z = state->rimZ;
        coverp = &cover;
    }
    for (level = 1; level <= 4 && result.skin.unresolved &&
         !(result.skin.unresolved <= HC_SKIN_ACCEPT_SPOTS && result.skin.maxAfter <= HC_SKIN_ACCEPT); level++) {
        if (level <= 2 && !coverp) continue;
        memcpy(Q, approved, (size_t)nv * 3 * sizeof(float));
        for (i = 0; level >= 2 && i < nv; i++) {
            /* full restore in the guard's front sector (dz/r > .5), fading out to the
             * reshaped hair by dz/r = .25 so the temples get no step */
            double dx = approved[3*i] - state->center[0], dz = approved[3*i+2] - state->center[1];
            double r = sqrt(dx*dx + dz*dz), w = 1.0;
            if (level <= 3) {
                double f = r > 1e-9 ? (dz / r - .25) / .25 : 0.0;
                if (f <= 0) continue;
                w = f >= 1 ? 1.0 : f * f * (3 - 2 * f);
            }
            if (level == 2) {
                /* only below the hat edge: full 4 mm under it, none from 2 mm above it */
                double p[3], edge, g;
                p[0] = approved[3*i]; p[1] = approved[3*i+1]; p[2] = approved[3*i+2];
                edge = cover.y[hc_skin_cover_sector(&cover, p)];
                if (edge < 1.0e99) {
                    g = (edge + .002 - p[1]) / .006;
                    if (g <= 0) continue;
                    if (g < 1) w *= g * g * (3 - 2 * g);
                }
            }
            if (w >= 1.0) {                 /* exact original X/Z (bit for bit) */
                Q[3*i]   = skinBaseline[3*i];
                Q[3*i+2] = skinBaseline[3*i+2];
                continue;
            }
            Q[3*i]   = (float)(approved[3*i]   + w * ((double)skinBaseline[3*i]   - approved[3*i]));
            Q[3*i+2] = (float)(approved[3*i+2] + w * ((double)skinBaseline[3*i+2] - approved[3*i+2]));
        }
        if (!hc_skin_guard_cover(state, P, Q, nv, indices, nt, isNew, skinBaseline, Q, &result.skin, coverp))
            HC_FAIL("front skin guard");
        result.frontKept = level;
    }
    /* v7: a sub-millimeter leftover no longer throws the whole chop away (which
     * showed the full hair through the hat); deeper leftovers still reject. */
    if (result.skin.unresolved && result.skin.unresolved <= HC_SKIN_ACCEPT_SPOTS && result.skin.maxAfter <= HC_SKIN_ACCEPT)
        result.skinTolerated = result.skin.unresolved;
    else
#endif
    if (result.skin.unresolved) {
        static char why[160];
        snprintf(why, sizeof(why), "front skin guard: hair still inside the face (%d spot(s), deepest %.2f mm)",
                 result.skin.unresolved, result.skin.maxAfter * 1000);
        HC_FAIL(why);
    }
#endif
    memcpy(vertices, vb, bytes);
    /* Interpolating uses the untouched payload at the original edge endpoints.
     * Donor slots are never used as interpolation sources. */
    for (i = 0; i < count; i++) {
        const HCBisectVertex *e = edges + i;
        if (isNew[e->dst] != 1) goto done;
        if (!hc_payload_interpolate(vertices + (size_t)e->dst * stride,
                                    vb + (size_t)e->a * stride, vb + (size_t)e->b * stride,
                                    e->t, Q + (size_t)e->dst * 3, layout)) HC_FAIL("vertex data");
        isNew[e->dst] = 2;
    }
    for (i = 0; i < nv; i++) {
        unsigned char *v = vertices + (size_t)i * stride;
        if (!isNew[i] && !hc_profile_store_position(v, Q + (size_t)i * 3)) HC_FAIL("packed position");
        if (memcmp(v, vb + (size_t)i * stride, 8)) result.moved++;
    }
    for (i = 0; i < nt * 3; i++) if (indices[i] >= (uint32_t)nv) goto done;
    result.maxPlaneError = result.bisect.max_plane_residual;
    for (i = 0; i < count; i++) {
        const int16_t *v = (const int16_t *)(vertices + (size_t)edges[i].dst * stride);
        const float *ideal = Q + (size_t)edges[i].dst * 3;
        double error = fabs((double)v[1]/v[3] - state->plane[0]*(double)v[0]/v[3] -
                            state->plane[1]*(double)v[2]/v[3] - state->plane[2]);
        if (error > result.maxWarpedBoundaryDeviation) result.maxWarpedBoundaryDeviation = error;
        for (k = 0; k < 3; k++) {
            double distance = fabs((double)v[k]/v[3] - ideal[k]);
            if (distance > result.maxPackedPositionError) result.maxPackedPositionError = distance;
            if (distance > .0001) HC_FAIL("packed cut edge");
        }
    }
    result.generated = count;
    result.cut = state->cut;
    memcpy(outVB, vertices, bytes);
    memcpy(outF, indices, (size_t)nt * 3 * sizeof(uint32_t));
    if (stats) *stats = result;
    hc_apply_fail = 0;
    ok = 1;
done:
    free(P); free(shaped); free(Q); free(boundary); free(indices); free(vertices); free(isNew); free(edges);
#ifdef HC_FRONT_SKIN_GUARD
    free(skinBaseline);
#ifdef HC_HAT_FIT
    free(approved);
#endif
#endif
    return ok;
}
#endif
