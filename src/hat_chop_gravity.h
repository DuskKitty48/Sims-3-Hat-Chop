/* Straight-down hat chopping shared by the game hook and offline mesh tests.
 * Derived from lowest_field / column_low / chop_gravity in hatchop_proto.py.
 * Only Y changes. This is geometric chopping, not a guarantee for every mesh.
 * A Shell must initially be zeroed; shell_build takes ownership of tri on
 * success AND failure. Shell queries mutate its stamp array: serialize users.
 */
#ifndef HAT_CHOP_GRAVITY_H
#define HAT_CHOP_GRAVITY_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

static double g_downClear = 0.0005;
static double g_gridStep = 0.002;

/* Limits bound temporary memory and processing of malformed game resources. */
#define HC_MAX_HAT_TRI 200000
#define HC_MAX_HAIR_VERT 200000
#define HC_MAX_HAIR_TRI 1000000
#define HC_MAX_LOW_CELLS 4194304u
#define HC_MAX_RASTER_WORK 67108864u
#define HC_MAX_BOUNDARY_CELLS 65536u
#define HC_MAX_GRID_CELLS 2097152u
#define HC_MAX_GRID_ITEMS 8388608u
#define HC_MAX_HIT_PAIRS 8388608u

typedef struct { double x, z, y; } HCGravityBoundary;

typedef struct {
    int ok, ntri;
    double *tri;                       /* 9 doubles per triangle, owned */
    double px, py, pz;                 /* diagnostic center only */
    double x0, z0, step, *low;
    int nx, nz;
    double cutMin;                    /* minimum finite field height, before clearance */
    HCGravityBoundary *boundary;
    int nboundary;
    double gmin[3], cell;
    int gn[3], *gstart, *gitem;
    uint32_t *stamp, query;
} Shell;

typedef struct { uint32_t a, b, tries; } HCGravityEdge;
typedef struct { uint32_t edge, tri; } HCGravityPair;

static int hc_gr_finite(double x)
{
    /* Also works on C runtimes whose isfinite spelling differs. */
    return x == x && x <= 1.7976931348623157e308 && x >= -1.7976931348623157e308;
}

static double hc_gr_min3(double a, double b, double c)
{
    double t = a < b ? a : b;
    return t < c ? t : c;
}

static double hc_gr_max3(double a, double b, double c)
{
    double t = a > b ? a : b;
    return t > c ? t : c;
}

static void shell_free(Shell *s)
{
    if (!s) return;
    free(s->tri);
    free(s->low);
    free(s->boundary);
    free(s->gstart);
    free(s->gitem);
    free(s->stamp);
    memset(s, 0, sizeof(*s));
}

/* Rasterized lowest surface in each X/Z column, with the prototype's
 * barycentric tolerance and HATCHOP_DILATE=0 (no extra occupied columns). */
static int hc_gr_low_build(Shell *s, const double *lo, const double *hi)
{
    double dx, dz;
    size_t nc, work = 0;
    int t, x, z;
    s->step = g_gridStep;
    s->x0 = lo[0] - 0.01;
    s->z0 = lo[2] - 0.01;
    dx = floor((hi[0] + 0.01 - s->x0) / s->step) + 1;
    dz = floor((hi[2] + 0.01 - s->z0) / s->step) + 1;
    if (dx < 1 || dz < 1 || dx > 4096 || dz > 4096) return 0;
    s->nx = (int)dx;
    s->nz = (int)dz;
    nc = (size_t)s->nx * s->nz;
    if (nc > HC_MAX_LOW_CELLS) return 0;
    s->low = (double *)malloc(nc * sizeof(double));
    if (!s->low) return 0;
    for (nc = 0; nc < (size_t)s->nx * s->nz; nc++) s->low[nc] = HUGE_VAL;
    for (t = 0; t < s->ntri; t++) {
        const double *p = s->tri + (size_t)9 * t;
        double xs[3], zs[3], den;
        int k, xa, xb, za, zb;
        for (k = 0; k < 3; k++) {
            xs[k] = (p[3 * k] - s->x0) / s->step;
            zs[k] = (p[3 * k + 2] - s->z0) / s->step;
        }
        xa = (int)floor(hc_gr_min3(xs[0], xs[1], xs[2]));
        xb = (int)ceil(hc_gr_max3(xs[0], xs[1], xs[2]));
        za = (int)floor(hc_gr_min3(zs[0], zs[1], zs[2]));
        zb = (int)ceil(hc_gr_max3(zs[0], zs[1], zs[2]));
        if (xa < 0) xa = 0;
        if (za < 0) za = 0;
        if (xb >= s->nx) xb = s->nx - 1;
        if (zb >= s->nz) zb = s->nz - 1;
        den = (zs[1] - zs[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (zs[0] - zs[2]);
        if (fabs(den) < 1e-12) continue;
        nc = (size_t)(xb - xa + 1) * (zb - za + 1);
        if (nc > HC_MAX_RASTER_WORK - work) return 0;
        work += nc;
        for (x = xa; x <= xb; x++)
            for (z = za; z <= zb; z++) {
                double gx = x + 0.5, gz = z + 0.5;
                double w0 = ((zs[1] - zs[2]) * (gx - xs[2]) + (xs[2] - xs[1]) * (gz - zs[2])) / den;
                double w1 = ((zs[2] - zs[0]) * (gx - xs[2]) + (xs[0] - xs[2]) * (gz - zs[2])) / den;
                double w2 = 1 - w0 - w1;
                if (w0 >= -0.02 && w1 >= -0.02 && w2 >= -0.02) {
                    size_t i = (size_t)x * s->nz + z;
                    double y = w0 * p[1] + w1 * p[4] + w2 * p[7];
                    if (y < s->low[i]) s->low[i] = y;
                }
            }
    }
    return 1;
}

/* Include diagonal neighbors as well as cardinal neighbors: at an empty
 * cell's exact corner, a diagonal occupied cell can tie the nearest sites.
 * An occupied cell with all eight neighbors occupied cannot be nearest to an
 * outside query. Keep sites in X-major/Z-minor order so all ties are stable.
 * This extends the actual local brim height, including tilts and holes;
 * it does not introduce a hair-specific height or horizontal deformation. */
static int hc_gr_is_boundary(const Shell *s, int x, int z)
{
    int dx, dz;
    for (dx = -1; dx <= 1; dx++)
        for (dz = -1; dz <= 1; dz++) {
            int a = x + dx, b = z + dz;
            if (!dx && !dz) continue;
            if (a < 0 || a >= s->nx || b < 0 || b >= s->nz ||
                !hc_gr_finite(s->low[(size_t)a * s->nz + b])) return 1;
        }
    return 0;
}

static int hc_gr_boundary_build(Shell *s)
{
    int x, z, count = 0;
    s->cutMin = HUGE_VAL;
    for (x = 0; x < s->nx; x++)
        for (z = 0; z < s->nz; z++) {
            size_t i = (size_t)x * s->nz + z;
            if (!hc_gr_finite(s->low[i])) continue;
            if (s->low[i] < s->cutMin) s->cutMin = s->low[i];
            if (hc_gr_is_boundary(s, x, z)) count++;
        }
    if (!count || (unsigned)count > HC_MAX_BOUNDARY_CELLS) return 0;
    s->boundary = (HCGravityBoundary *)malloc((size_t)count * sizeof(HCGravityBoundary));
    if (!s->boundary) return 0;
    s->nboundary = 0;
    for (x = 0; x < s->nx; x++)
        for (z = 0; z < s->nz; z++) {
            size_t i = (size_t)x * s->nz + z;
            HCGravityBoundary *b;
            if (!hc_gr_finite(s->low[i])) continue;
            if (!hc_gr_is_boundary(s, x, z)) continue;
            b = s->boundary + s->nboundary++;
            b->x = s->x0 + (x + 0.5) * s->step;
            b->z = s->z0 + (z + 0.5) * s->step;
            b->y = s->low[i];
        }
    return 1;
}

static void hc_gr_cell_of(const Shell *s, const double *p, int *c)
{
    int k;
    for (k = 0; k < 3; k++) {
        double v = floor((p[k] - s->gmin[k]) / s->cell);
        c[k] = v < 0 ? 0 : v >= s->gn[k] ? s->gn[k] - 1 : (int)v;
    }
}

static void hc_gr_tri_cells(const Shell *s, int t, int *a, int *b)
{
    const double *p = s->tri + (size_t)9 * t;
    double lo[3], hi[3];
    int k;
    for (k = 0; k < 3; k++) {
        lo[k] = hc_gr_min3(p[k], p[k + 3], p[k + 6]);
        hi[k] = hc_gr_max3(p[k], p[k + 3], p[k + 6]);
    }
    hc_gr_cell_of(s, lo, a);
    hc_gr_cell_of(s, hi, b);
}

/* Compact cell -> triangle lists. Coarsen only the acceleration grid if its
 * bounds would exceed the resource limit; the chopping grid stays exact. */
static int hc_gr_grid_build(Shell *s, const double *lo, const double *hi)
{
    size_t nc = 0, total = 0;
    int k, t, x, y, z, pass, *count = NULL;
    s->cell = 0.01;
    for (k = 0; k < 3; k++) s->gmin[k] = lo[k] - 0.02;
    for (pass = 0; pass < 24; pass++) {
        int valid = 1;
        nc = 1;
        for (k = 0; k < 3; k++) {
            double n = floor((hi[k] - lo[k] + 0.04) / s->cell) + 1;
            if (n < 1 || n > 512) { valid = 0; break; }
            s->gn[k] = (int)n;
            nc *= (size_t)s->gn[k];
        }
        if (valid && nc <= HC_MAX_GRID_CELLS) break;
        s->cell *= 2;
    }
    if (pass == 24) return 0;
    count = (int *)calloc(nc + 1, sizeof(int));
    s->gstart = (int *)calloc(nc + 1, sizeof(int));
    s->stamp = (uint32_t *)calloc((size_t)s->ntri, sizeof(uint32_t));
    if (!count || !s->gstart || !s->stamp) { free(count); return 0; }
    for (t = 0; t < s->ntri; t++) {
        int a[3], b[3];
        size_t add;
        hc_gr_tri_cells(s, t, a, b);
        add = (size_t)(b[0] - a[0] + 1) * (b[1] - a[1] + 1) * (b[2] - a[2] + 1);
        if (add > HC_MAX_GRID_ITEMS - total) { free(count); return 0; }
        total += add;
        for (x = a[0]; x <= b[0]; x++)
            for (y = a[1]; y <= b[1]; y++)
                for (z = a[2]; z <= b[2]; z++) count[(x * s->gn[1] + y) * s->gn[2] + z]++;
    }
    s->gitem = (int *)malloc((total ? total : 1) * sizeof(int));
    if (!s->gitem) { free(count); return 0; }
    for (nc = 0; nc < (size_t)s->gn[0] * s->gn[1] * s->gn[2]; nc++)
        s->gstart[nc + 1] = s->gstart[nc] + count[nc];
    memset(count, 0, ((size_t)s->gn[0] * s->gn[1] * s->gn[2] + 1) * sizeof(int));
    for (t = 0; t < s->ntri; t++) {
        int a[3], b[3];
        hc_gr_tri_cells(s, t, a, b);
        for (x = a[0]; x <= b[0]; x++)
            for (y = a[1]; y <= b[1]; y++)
                for (z = a[2]; z <= b[2]; z++) {
                    int ci = (x * s->gn[1] + y) * s->gn[2] + z;
                    s->gitem[s->gstart[ci] + count[ci]++] = t;
                }
    }
    free(count);
    return 1;
}

/* Build from hat triangles; ownership transfers even when validation fails. */
static int shell_build(Shell *s, double *tri, int ntri)
{
    double lo[3] = {HUGE_VAL, HUGE_VAL, HUGE_VAL};
    double hi[3] = {-HUGE_VAL, -HUGE_VAL, -HUGE_VAL};
    int i, k;
    if (!s) { free(tri); return 0; }
    shell_free(s);
    s->tri = tri;
    s->ntri = ntri;
    if (!tri || ntri <= 0 || ntri > HC_MAX_HAT_TRI ||
        !hc_gr_finite(g_gridStep) || g_gridStep < 0.0001 || g_gridStep > 0.05 ||
        !hc_gr_finite(g_downClear) || g_downClear < 0 || g_downClear > 0.05) goto fail;
    for (i = 0; i < ntri * 3; i++)
        for (k = 0; k < 3; k++) {
            double v = tri[(size_t)3 * i + k];
            if (!hc_gr_finite(v) || fabs(v) > 10000) goto fail;
            if (v < lo[k]) lo[k] = v;
            if (v > hi[k]) hi[k] = v;
        }
    s->px = (lo[0] + hi[0]) * 0.5;
    s->py = lo[1];
    s->pz = (lo[2] + hi[2]) * 0.5;
    if (!hc_gr_low_build(s, lo, hi) || !hc_gr_boundary_build(s) || !hc_gr_grid_build(s, lo, hi)) goto fail;
    s->ok = 1;
    return 1;
fail:
    shell_free(s);
    return 0;
}

/* Moller-Trumbore with the same strict segment endpoints as the prototype. */
static int hc_gr_tri_hit(const double *a, const double *d, const double *p)
{
    double e1[3] = {p[3] - p[0], p[4] - p[1], p[5] - p[2]};
    double e2[3] = {p[6] - p[0], p[7] - p[1], p[8] - p[2]};
    double h[3] = {d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2], d[0] * e2[1] - d[1] * e2[0]};
    double det = e1[0] * h[0] + e1[1] * h[1] + e1[2] * h[2], inv, o[3], u, q[3], v, t;
    if (fabs(det) <= 1e-12) return 0;
    inv = 1 / det;
    o[0] = a[0] - p[0]; o[1] = a[1] - p[1]; o[2] = a[2] - p[2];
    u = (o[0] * h[0] + o[1] * h[1] + o[2] * h[2]) * inv;
    if (u < 0 || u > 1) return 0;
    q[0] = o[1] * e1[2] - o[2] * e1[1];
    q[1] = o[2] * e1[0] - o[0] * e1[2];
    q[2] = o[0] * e1[1] - o[1] * e1[0];
    v = (d[0] * q[0] + d[1] * q[1] + d[2] * q[2]) * inv;
    if (v < 0 || u + v > 1) return 0;
    t = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) * inv;
    return t > 1e-6 && t < 1 - 1e-6;
}

/* Clip to the 3D grid and traverse only the cells the segment enters. The
 * triangle stamp suppresses duplicates; returned IDs retain the actual hit.
 * hits has ntri capacity. first_only is used by final unique-edge counts. */
static int hc_gr_segment_hits(Shell *s, const double *a, const double *b, int *hits, int first_only)
{
    double d[3], enter = 0, leave = 1, start[3], next[3], delta[3];
    int c[3], step[3], k, n = 0, walk, maxwalk;
    for (k = 0; k < 3; k++) {
        double gl = s->gmin[k], gh = gl + s->gn[k] * s->cell;
        d[k] = b[k] - a[k];
        if (d[k] == 0) {
            if (a[k] < gl || a[k] > gh) return 0;
        } else {
            double t0 = (gl - a[k]) / d[k], t1 = (gh - a[k]) / d[k], tmp;
            if (t0 > t1) { tmp = t0; t0 = t1; t1 = tmp; }
            if (t0 > enter) enter = t0;
            if (t1 < leave) leave = t1;
            if (enter > leave) return 0;
        }
    }
    for (k = 0; k < 3; k++) start[k] = a[k] + d[k] * enter;
    hc_gr_cell_of(s, start, c);
    for (k = 0; k < 3; k++) {
        if (d[k] > 0) {
            step[k] = 1;
            next[k] = (s->gmin[k] + (c[k] + 1) * s->cell - a[k]) / d[k];
            delta[k] = s->cell / d[k];
        } else if (d[k] < 0) {
            step[k] = -1;
            next[k] = (s->gmin[k] + c[k] * s->cell - a[k]) / d[k];
            delta[k] = -s->cell / d[k];
        } else {
            step[k] = 0;
            next[k] = delta[k] = HUGE_VAL;
        }
    }
    if (++s->query == 0) {
        memset(s->stamp, 0, (size_t)s->ntri * sizeof(uint32_t));
        s->query = 1;
    }
    maxwalk = s->gn[0] + s->gn[1] + s->gn[2] + 3;
    for (walk = 0; walk < maxwalk; walk++) {
        int ci = (c[0] * s->gn[1] + c[1]) * s->gn[2] + c[2], j;
        double nearest;
        for (j = s->gstart[ci]; j < s->gstart[ci + 1]; j++) {
            int t = s->gitem[j];
            if (s->stamp[t] == s->query) continue;
            s->stamp[t] = s->query;
            if (hc_gr_tri_hit(a, d, s->tri + (size_t)9 * t)) {
                if (first_only) return 1;
                hits[n++] = t;
            }
        }
        nearest = hc_gr_min3(next[0], next[1], next[2]);
        if (nearest > leave || !hc_gr_finite(nearest)) break;
        for (k = 0; k < 3; k++)
            if (next[k] <= nearest) { c[k] += step[k]; next[k] += delta[k]; }
        if (c[0] < 0 || c[0] >= s->gn[0] || c[1] < 0 || c[1] >= s->gn[1] || c[2] < 0 || c[2] >= s->gn[2]) break;
    }
    return n;
}

static int hc_gr_edge_cmp(const void *a, const void *b)
{
    const HCGravityEdge *x = (const HCGravityEdge *)a, *y = (const HCGravityEdge *)b;
    if (x->a != y->a) return x->a < y->a ? -1 : 1;
    return x->b < y->b ? -1 : x->b > y->b;
}

static int hc_gr_int_cmp(const void *a, const void *b)
{
    int x = *(const int *)a, y = *(const int *)b;
    return x < y ? -1 : x > y;
}

static int hc_gr_input_ok(const Shell *s, const float *P, int nv, const uint32_t *F, int nt)
{
    size_t i;
    if (!s || !s->ok || !s->tri || !s->low || !s->boundary || !s->nboundary || !s->gstart || !s->gitem || !s->stamp ||
        !P || nv <= 0 || nv > HC_MAX_HAIR_VERT || nt < 0 || nt > HC_MAX_HAIR_TRI || (nt && !F) ||
        !hc_gr_finite(g_downClear) || g_downClear < 0 || g_downClear > 0.05) return 0;
    for (i = 0; i < (size_t)nv * 3; i++) if (!hc_gr_finite(P[i])) return 0;
    for (i = 0; i < (size_t)nt * 3; i++) if (F[i] >= (uint32_t)nv) return 0;
    return 1;
}

/* Sort and deduplicate undirected edges, just like edges_of in the prototype.
 * Degenerate a==b edges cannot cross a triangle and can be discarded. */
static HCGravityEdge *hc_gr_edges(const uint32_t *F, int nt, size_t *ne)
{
    HCGravityEdge *E;
    size_t i, n = 0, out = 0;
    *ne = 0;
    E = (HCGravityEdge *)malloc((nt ? (size_t)nt * 3 : 1) * sizeof(HCGravityEdge));
    if (!E) return NULL;
    for (i = 0; i < (size_t)nt; i++) {
        int k;
        for (k = 0; k < 3; k++) {
            uint32_t a = F[3 * i + k], b = F[3 * i + (k + 1) % 3];
            if (a == b) continue;
            E[n].a = a < b ? a : b;
            E[n].b = a < b ? b : a;
            E[n++].tries = 0;
        }
    }
    qsort(E, n, sizeof(*E), hc_gr_edge_cmp);
    for (i = 0; i < n; i++)
        if (!out || E[i].a != E[out - 1].a || E[i].b != E[out - 1].b) E[out++] = E[i];
    *ne = out;
    return E;
}

static int hc_gr_count_edges(Shell *s, const double *P, const HCGravityEdge *E, size_t ne)
{
    size_t i;
    int n = 0;
    for (i = 0; i < ne; i++)
        n += hc_gr_segment_hits(s, P + (size_t)3 * E[i].a, P + (size_t)3 * E[i].b, NULL, 1);
    return n;
}

/* Exposed to the output codec: count the actual, decoded final coordinates.
 * Returns -1 for invalid data or resource allocation failure. */
static int gravity_crossing_count(Shell *s, const float *P, int nv, const uint32_t *F, int nt)
{
    HCGravityEdge *E;
    double *Q;
    size_t i, ne;
    int n;
    if (!hc_gr_input_ok(s, P, nv, F, nt)) return -1;
    E = hc_gr_edges(F, nt, &ne);
    Q = (double *)malloc((size_t)nv * 3 * sizeof(double));
    if (!E || !Q) { free(E); free(Q); return -1; }
    for (i = 0; i < (size_t)nv * 3; i++) Q[i] = P[i];
    n = hc_gr_count_edges(s, Q, E, ne);
    free(E);
    free(Q);
    return n;
}

static double hc_gr_column_low(const Shell *s, double x, double z)
{
    double i = floor((x - s->x0) / s->step), j = floor((z - s->z0) / s->step);
    if (i < 0 || i >= s->nx || j < 0 || j >= s->nz) return HUGE_VAL;
    return s->low[(size_t)(int)i * s->nz + (int)j];
}

/* Shared chop/cull limit, BEFORE clearance. Preserve every finite original
 * column exactly; outside the footprint use the nearest occupied cell center.
 * Query cost is O(boundary cells), bounded by HC_MAX_BOUNDARY_CELLS. */
static double hc_chop_limit(const Shell *s, double x, double z)
{
    double low, best = HUGE_VAL;
    int i;
    if (!s || !s->low || !hc_gr_finite(x) || !hc_gr_finite(z)) return HUGE_VAL;
    low = hc_gr_column_low(s, x, z);
    if (hc_gr_finite(low)) return low;
    if (!s->boundary || s->nboundary <= 0) return HUGE_VAL;
    for (i = 0; i < s->nboundary; i++) {
        const HCGravityBoundary *b = s->boundary + i;
        double dx = x - b->x, dz = z - b->z, distance = dx * dx + dz * dz;
        if (!i || distance < best) { best = distance; low = b->y; }
    }
    return low;
}

static double hc_gr_triangle_height(const double *T, double x, double z)
{
    double den = (T[5] - T[8]) * (T[0] - T[6]) + (T[6] - T[3]) * (T[2] - T[8]);
    double w0, w1, w2, sum;
    if (fabs(den) < 1e-12) return hc_gr_min3(T[1], T[4], T[7]);
    w0 = ((T[5] - T[8]) * (x - T[6]) + (T[6] - T[3]) * (z - T[8])) / den;
    w1 = ((T[8] - T[2]) * (x - T[6]) + (T[0] - T[6]) * (z - T[8])) / den;
    w2 = 1 - w0 - w1;
    if (w0 < 0) w0 = 0;
    if (w1 < 0) w1 = 0;
    if (w2 < 0) w2 = 0;
    sum = w0 + w1 + w2;
    return (w0 * T[1] + w1 * T[4] + w2 * T[7]) / sum;
}

/* The whole pass runs on a private copy. Invalid input or exhausted temporary
 * storage leaves P untouched and sets *left=-1. Successful output changes only
 * Y; *left counts unique edges crossing after the final float conversion. */
static int chop_mesh(Shell *s, float *P, int nv, const uint32_t *F, int nt, int *left)
{
    HCGravityEdge *E = NULL;
    HCGravityPair *pairs = NULL;
    double *Q = NULL;
    int *hits = NULL, moved = 0, it;
    size_t i, ne, npair, capacity = 0;
    if (left) *left = -1;
    if (!hc_gr_input_ok(s, P, nv, F, nt)) return 0;
    E = hc_gr_edges(F, nt, &ne);
    Q = (double *)malloc((size_t)nv * 3 * sizeof(double));
    hits = (int *)malloc((size_t)s->ntri * sizeof(int));
    if (!E || !Q || !hits) goto done;
    for (i = 0; i < (size_t)nv * 3; i++) Q[i] = P[i];
    for (i = 0; i < (size_t)nv; i++) {
        double y;
        if (Q[3 * i + 1] <= s->cutMin - g_downClear) continue;
        y = hc_chop_limit(s, Q[3 * i], Q[3 * i + 2]);
        if (hc_gr_finite(y) && Q[3 * i + 1] > y - g_downClear) Q[3 * i + 1] = y - g_downClear;
    }
    for (it = 0; it < 40; it++) {
        npair = 0;
        /* Collect before moving anything this round, and sort each edge's
         * triangle IDs to reproduce NumPy's edge-major crossing_pairs order. */
        for (i = 0; i < ne; i++) {
            int n = hc_gr_segment_hits(s, Q + (size_t)3 * E[i].a, Q + (size_t)3 * E[i].b, hits, 0), j;
            size_t need = npair + (size_t)n;
            if (need > HC_MAX_HIT_PAIRS) goto done;
            if (need > capacity) {
                size_t cap = capacity ? capacity : 1024;
                HCGravityPair *p;
                while (cap < need) cap = cap > HC_MAX_HIT_PAIRS / 2 ? HC_MAX_HIT_PAIRS : cap * 2;
                p = (HCGravityPair *)realloc(pairs, cap * sizeof(*p));
                if (!p) goto done;
                pairs = p;
                capacity = cap;
            }
            qsort(hits, (size_t)n, sizeof(int), hc_gr_int_cmp);
            for (j = 0; j < n; j++) {
                pairs[npair].edge = (uint32_t)i;
                pairs[npair++].tri = (uint32_t)hits[j];
            }
        }
        if (!npair) break;
        for (i = 0; i < npair; i++) {
            HCGravityEdge *e = E + pairs[i].edge;
            const double *T = s->tri + (size_t)9 * pairs[i].tri;
            int k;
            e->tries++;
            for (k = 0; k < 2; k++) {
                size_t v = (size_t)3 * (k ? e->b : e->a);
                double y = e->tries > 3 ? hc_gr_min3(T[1], T[4], T[7]) : hc_gr_triangle_height(T, Q[v], Q[v + 2]);
                if (Q[v + 1] > y - g_downClear) Q[v + 1] = y - g_downClear;
            }
        }
    }
    /* Count the coordinates that callers actually receive, not the last
     * iteration's stale hit list or higher-precision working coordinates. */
    for (i = 0; i < (size_t)nv; i++) Q[3 * i + 1] = (float)Q[3 * i + 1];
    if (left) *left = hc_gr_count_edges(s, Q, E, ne);
    for (i = 0; i < (size_t)nv; i++)
        if (P[3 * i + 1] != (float)Q[3 * i + 1]) {
            P[3 * i + 1] = (float)Q[3 * i + 1];
            moved++;
        }
done:
    free(E);
    free(Q);
    free(hits);
    free(pairs);
    return moved;
}

#endif /* HAT_CHOP_GRAVITY_H */
