/* v7 speed-up: height buckets for the horizontal ray queries of the shaper.
 *
 * A horizontal ray at height y can only hit a triangle whose Y range contains y.
 * Each triangle is listed in every bucket its Y range (padded by HC_RAY_PAD,
 * far above rounding error) touches, so a query tests a superset of every
 * triangle the old all-triangle loop could hit. min/max over the same hits is
 * order independent: results are bit-identical, only far fewer tests run.
 */
#ifndef HAT_CHOP_RAY_INDEX_H
#define HAT_CHOP_RAY_INDEX_H
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define HC_RAY_BUCKETS 256
#define HC_RAY_PAD 1e-9

typedef struct {
    int nt;
    double ymin, ymax, perBucket;
    double *lo, *hi;
    uint32_t start[HC_RAY_BUCKETS + 1];
    uint32_t *item;
} HCRayIndex;

static void hc_ray_index_free(HCRayIndex *x)
{
    if (!x) return;
    free(x->lo); free(x->hi); free(x->item);
    memset(x, 0, sizeof(*x));
}

static int hc_ray_bucket(const HCRayIndex *x, double y)
{
    int b = (int)floor((y - x->ymin) * x->perBucket);
    if (b < 0) b = 0;
    if (b >= HC_RAY_BUCKETS) b = HC_RAY_BUCKETS - 1;
    return b;
}

/* tri: 9 doubles per triangle, borrowed (not stored). Returns 1 on success;
 * on failure the index stays empty and callers use the full loop. */
static int hc_ray_index_build(HCRayIndex *x, const double *tri, int nt)
{
    uint32_t count[HC_RAY_BUCKETS] = {0}, cursor[HC_RAY_BUCKETS];
    int i, b;
    memset(x, 0, sizeof(*x));
    if (!tri || nt <= 0 || nt > 1000000) return 0;
    x->lo = (double *)malloc((size_t)nt * sizeof(double));
    x->hi = (double *)malloc((size_t)nt * sizeof(double));
    if (!x->lo || !x->hi) goto fail;
    x->ymin = HUGE_VAL; x->ymax = -HUGE_VAL;
    for (i = 0; i < nt; i++) {
        const double *t = tri + (size_t)i * 9;
        double lo = t[1], hi = t[1];
        if (t[4] < lo) lo = t[4];
        if (t[4] > hi) hi = t[4];
        if (t[7] < lo) lo = t[7];
        if (t[7] > hi) hi = t[7];
        if (!(lo == lo) || !(hi == hi)) goto fail;
        x->lo[i] = lo - HC_RAY_PAD; x->hi[i] = hi + HC_RAY_PAD;
        if (x->lo[i] < x->ymin) x->ymin = x->lo[i];
        if (x->hi[i] > x->ymax) x->ymax = x->hi[i];
    }
    if (!(x->ymax - x->ymin > 1e-9) || !(x->ymax - x->ymin < 100)) goto fail;
    x->perBucket = HC_RAY_BUCKETS / (x->ymax - x->ymin);
    for (i = 0; i < nt; i++)
        for (b = hc_ray_bucket(x, x->lo[i]); b <= hc_ray_bucket(x, x->hi[i]); b++) count[b]++;
    for (b = 0; b < HC_RAY_BUCKETS; b++) {
        x->start[b + 1] = x->start[b] + count[b];
        cursor[b] = x->start[b];
        if (x->start[b + 1] > 64u * 1000000u) goto fail;
    }
    x->item = (uint32_t *)malloc((size_t)(x->start[HC_RAY_BUCKETS] ? x->start[HC_RAY_BUCKETS] : 1) * sizeof(uint32_t));
    if (!x->item) goto fail;
    for (i = 0; i < nt; i++)
        for (b = hc_ray_bucket(x, x->lo[i]); b <= hc_ray_bucket(x, x->hi[i]); b++) x->item[cursor[b]++] = (uint32_t)i;
    x->nt = nt;
    return 1;
fail:
    hc_ray_index_free(x);
    return 0;
}

/* Height x direction buckets for rays that start on a fixed vertical axis
 * (cx, cz) and run horizontally outward. A triangle whose top-down outline
 * does not contain the axis can only be hit inside the angle range of its
 * corners; it is listed there with one spare bucket on each side. Triangles
 * on or around the axis are listed in every direction. Superset only: the
 * caller still runs its own exact test on each listed triangle. */
#define HC_RAY2_Y 128
#define HC_RAY2_A 64
#define HC_TWO_PI 6.28318530717958647692

typedef struct {
    int nt;
    double cx, cz, ymin, ymax, perY;
    uint32_t *start;                   /* HC_RAY2_Y*HC_RAY2_A+1 */
    uint32_t *item;
} HCRayIndex2;

static void hc_ray2_free(HCRayIndex2 *x)
{
    if (!x) return;
    free(x->start); free(x->item);
    memset(x, 0, sizeof(*x));
}

static int hc_ray2_ybin(const HCRayIndex2 *x, double y)
{
    int b = (int)floor((y - x->ymin) * x->perY);
    if (b < 0) b = 0;
    if (b >= HC_RAY2_Y) b = HC_RAY2_Y - 1;
    return b;
}

static int hc_ray2_abin(double dx, double dz)
{
    int b = (int)floor((atan2(dx, dz) + 3.14159265358979323846) * (HC_RAY2_A / HC_TWO_PI));
    if (b < 0) b = 0;
    if (b >= HC_RAY2_A) b = HC_RAY2_A - 1;
    return b;
}

/* Angle bins of one triangle: *all=1, or the inclusive range [*a0, *a1] (may wrap). */
static void hc_ray2_span(const double *t, double cx, double cz, int *all, int *a0, int *a1)
{
    double ang[3], x[3], z[3], gap = -1, s1, s2, s3, binw = HC_TWO_PI / HC_RAY2_A;
    int k, j, startK = 0;
    *all = 0;
    for (k = 0; k < 3; k++) {
        x[k] = t[3*k] - cx; z[k] = t[3*k+2] - cz;
        if (x[k] * x[k] + z[k] * z[k] < 1e-12) { *all = 1; return; }
        ang[k] = atan2(x[k], z[k]) + 3.14159265358979323846;
    }
    /* axis inside or on the outline (2D signs, with tolerance): every direction */
    s1 = x[0] * z[1] - z[0] * x[1];
    s2 = x[1] * z[2] - z[1] * x[2];
    s3 = x[2] * z[0] - z[2] * x[0];
    if ((s1 >= -1e-12 && s2 >= -1e-12 && s3 >= -1e-12) || (s1 <= 1e-12 && s2 <= 1e-12 && s3 <= 1e-12)) { *all = 1; return; }
    /* the arc covering all corners is the circle minus its largest empty gap */
    for (k = 0; k < 3; k++) {
        double best = HC_TWO_PI;
        for (j = 0; j < 3; j++) if (j != k) {
            double g = ang[j] - ang[k];
            if (g <= 0) g += HC_TWO_PI;
            if (g < best) best = g;
        }
        if (best > gap) { gap = best; startK = k; }  /* empty gap runs from ang[k] forward */
    }
    if (gap < 3.14159265358979323846 + 2 * binw) { *all = 1; return; }
    /* covered arc: from the corner after the gap to ang[startK] */
    {
        double a1v = ang[startK], a0v = a1v - (HC_TWO_PI - gap);
        int b0 = (int)floor(a0v / binw) - 1, b1 = (int)floor(a1v / binw) + 1;
        *a0 = ((b0 % HC_RAY2_A) + HC_RAY2_A) % HC_RAY2_A;
        *a1 = ((b1 % HC_RAY2_A) + HC_RAY2_A) % HC_RAY2_A;
        if (b1 - b0 + 1 >= HC_RAY2_A) *all = 1;
    }
}

/* lo/hi: each triangle's Y range (borrowed); listed with a padded Y range. */
static int hc_ray2_build(HCRayIndex2 *x, const double *tri, int nt, const double *lo, const double *hi,
                         double cx, double cz)
{
    uint32_t *count = NULL, *cursor = NULL;
    int pass, i, yb, ab;
    memset(x, 0, sizeof(*x));
    if (!tri || !lo || !hi || nt <= 0 || nt > 1000000) return 0;
    x->cx = cx; x->cz = cz;
    x->ymin = HUGE_VAL; x->ymax = -HUGE_VAL;
    for (i = 0; i < nt; i++) {
        if (lo[i] - HC_RAY_PAD < x->ymin) x->ymin = lo[i] - HC_RAY_PAD;
        if (hi[i] + HC_RAY_PAD > x->ymax) x->ymax = hi[i] + HC_RAY_PAD;
    }
    if (!(x->ymax - x->ymin > 1e-9) || !(x->ymax - x->ymin < 100)) return 0;
    x->perY = HC_RAY2_Y / (x->ymax - x->ymin);
    count = (uint32_t *)calloc(HC_RAY2_Y * HC_RAY2_A, sizeof(uint32_t));
    cursor = (uint32_t *)malloc(HC_RAY2_Y * HC_RAY2_A * sizeof(uint32_t));
    x->start = (uint32_t *)calloc(HC_RAY2_Y * HC_RAY2_A + 1, sizeof(uint32_t));
    if (!count || !cursor || !x->start) goto fail;
    for (pass = 0; pass < 2; pass++) {
        for (i = 0; i < nt; i++) {
            int all, a0 = 0, a1 = 0, y0 = hc_ray2_ybin(x, lo[i] - HC_RAY_PAD), y1 = hc_ray2_ybin(x, hi[i] + HC_RAY_PAD);
            hc_ray2_span(tri + (size_t)i * 9, cx, cz, &all, &a0, &a1);
            for (yb = y0; yb <= y1; yb++) {
                if (all) {
                    for (ab = 0; ab < HC_RAY2_A; ab++) {
                        if (pass) x->item[cursor[yb * HC_RAY2_A + ab]++] = (uint32_t)i; else count[yb * HC_RAY2_A + ab]++;
                    }
                } else {
                    for (ab = a0;; ab = (ab + 1) % HC_RAY2_A) {
                        if (pass) x->item[cursor[yb * HC_RAY2_A + ab]++] = (uint32_t)i; else count[yb * HC_RAY2_A + ab]++;
                        if (ab == a1) break;
                    }
                }
            }
        }
        if (!pass) {
            int b;
            for (b = 0; b < HC_RAY2_Y * HC_RAY2_A; b++) {
                x->start[b + 1] = x->start[b] + count[b];
                cursor[b] = x->start[b];
                if (x->start[b + 1] > 64u * 1000000u) goto fail;
            }
            x->item = (uint32_t *)malloc((size_t)(x->start[HC_RAY2_Y * HC_RAY2_A] ? x->start[HC_RAY2_Y * HC_RAY2_A] : 1) * sizeof(uint32_t));
            if (!x->item) goto fail;
        }
    }
    free(count); free(cursor);
    x->nt = nt;
    return 1;
fail:
    free(count); free(cursor);
    hc_ray2_free(x);
    return 0;
}
#endif
