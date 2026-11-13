/* v7 fallback cut that needs no spare vertices (same rule as fixed_cut() in
 * tools/hatchop_fixed_profile_demo.py). Used only when the exact bisect has too
 * few removed-top vertex/face slots, which used to skip the whole chop and leave
 * full hair poking through the hat.
 *
 * Faces fully above y=a*x+b*z+c become degenerate. Each above vertex of a cut
 * face moves to the average of its unique crossing-edge intersection points,
 * then its Y is set exactly on the plane. Below/on-plane vertices never move.
 * Vertex payloads (UVs, weights) stay with their vertex. Returns the moved
 * count (indices in moved[]), or -1 on invalid input/allocation failure.
 */
#ifndef HAT_CHOP_FIXED_CUT_H
#define HAT_CHOP_FIXED_CUT_H
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct { uint32_t up, down; } HCFixedEdge;

static int hc_fixed_edge_compare(const void *x, const void *y)
{
    const HCFixedEdge *p = (const HCFixedEdge *)x, *q = (const HCFixedEdge *)y;
    if (p->up != q->up) return p->up < q->up ? -1 : 1;
    return p->down < q->down ? -1 : p->down > q->down;
}

static int hc_fixed_cut(const float *P, int nv, const uint32_t *F, int nt,
                        double a, double b, double c, float *Q, uint32_t *outF, uint32_t *moved)
{
    double *s = NULL, *sum = NULL;
    uint32_t *count = NULL;
    HCFixedEdge *e = NULL;
    size_t ne = 0, i, k;
    int n = -1;
    if (!P || !Q || !F || !outF || !moved || nv <= 0 || nt <= 0) return -1;
    s = (double *)malloc((size_t)nv * sizeof(double));
    sum = (double *)calloc((size_t)nv * 3, sizeof(double));
    count = (uint32_t *)calloc((size_t)nv, sizeof(uint32_t));
    e = (HCFixedEdge *)malloc((size_t)nt * 3 * sizeof(*e));
    if (!s || !sum || !count || !e) goto done;
    for (i = 0; i < (size_t)nv; i++) {
        s[i] = P[3*i+1] - (a * P[3*i] + b * P[3*i+2] + c);
        if (!(s[i] == s[i])) goto done;
    }
    for (i = 0; i < (size_t)nt * 3; i++) if (F[i] >= (uint32_t)nv) goto done;
    for (i = 0; i < (size_t)nt; i++)
        for (k = 0; k < 3; k++) {
            uint32_t x = F[3*i+k], y = F[3*i+(k+1)%3];
            if ((s[x] > 0) == (s[y] > 0)) continue;
            e[ne].up = s[x] > 0 ? x : y;
            e[ne].down = s[x] > 0 ? y : x;
            ne++;
        }
    if (ne) qsort(e, ne, sizeof(*e), hc_fixed_edge_compare);
    for (i = 0; i < ne; i++) {
        uint32_t up = e[i].up, down = e[i].down;
        double t;
        if (i && e[i].up == e[i-1].up && e[i].down == e[i-1].down) continue;
        t = s[up] / (s[up] - s[down]);
        for (k = 0; k < 3; k++) sum[3*up+k] += P[3*up+k] + t * (P[3*down+k] - P[3*up+k]);
        count[up]++;
    }
    memcpy(Q, P, (size_t)nv * 3 * sizeof(float));
    memcpy(outF, F, (size_t)nt * 3 * sizeof(uint32_t));
    n = 0;
    for (i = 0; i < (size_t)nv; i++) {
        if (!count[i]) continue;
        Q[3*i]   = (float)(sum[3*i]   / count[i]);
        Q[3*i+2] = (float)(sum[3*i+2] / count[i]);
        Q[3*i+1] = (float)(a * Q[3*i] + b * Q[3*i+2] + c);
        moved[n++] = (uint32_t)i;
    }
    for (i = 0; i < (size_t)nt; i++) {
        const uint32_t *f = F + 3 * i;
        if (s[f[0]] > 0 && s[f[1]] > 0 && s[f[2]] > 0) outF[3*i+1] = outF[3*i+2] = outF[3*i];
    }
done:
    free(s); free(sum); free(count); free(e);
    return n;
}
#endif
