/* Shared runtime/offline predicate for removing the hidden top of a hair.
 * Include after hat_chop_gravity.h. The caller validates face vertex indices.
 * Mixed boundary triangles remain so their visible lower portion is retained.
 */
#ifndef HAT_CHOP_CUT_H
#define HAT_CHOP_CUT_H

static inline int hidden_triangle(Shell *s, const float *originalP, const uint32_t face[3])
{
    int k;
    if (!s || !s->ok || !originalP || !face) return 0;
    for (k = 0; k < 3; k++) {
        const float *p = originalP + (size_t)face[k] * 3;
        double low;
        if (!hc_gr_finite(p[0]) || !hc_gr_finite(p[1]) || !hc_gr_finite(p[2])) return 0;
        low = hc_chop_limit(s, p[0], p[2]);
        if (!hc_gr_finite(low) || p[1] <= low - g_downClear) return 0;
    }
    return 1;
}

/* Keep two synchronous rings of triangles that share a vertex INDEX with a
 * retained triangle. This preserves support at the hat boundary while still
 * removing the deep hidden top. Split UV-seam vertices remain independent;
 * no per-hair names, tuning, or coordinate welding is used.
 * On invalid input/allocation failure, out is left untouched.
 */
static int build_hidden_mask(Shell *s, const float *originalP, int nv,
                             const uint32_t *F, int nt, unsigned char *out)
{
    unsigned char *hidden = NULL, *boundary = NULL;
    int i, k, ring;
    if (!s || !s->ok || !originalP || nv <= 0 || nv > HC_MAX_HAIR_VERT ||
        nt < 0 || nt > HC_MAX_HAIR_TRI || (nt && (!F || !out))) return 0;
    for (i = 0; i < nv; i++)
        for (k = 0; k < 3; k++)
            if (!hc_gr_finite(originalP[(size_t)i * 3 + k])) return 0;
    for (i = 0; i < nt; i++)
        for (k = 0; k < 3; k++)
            if (F[(size_t)i * 3 + k] >= (uint32_t)nv) return 0;
    if (!nt) return 1;
    hidden = (unsigned char *)malloc((size_t)nt);
    boundary = (unsigned char *)malloc((size_t)nv);
    if (!hidden || !boundary) { free(hidden); free(boundary); return 0; }
    /* Evaluate the possibly extrapolated height once per vertex, then reuse
     * the same storage for synchronous boundary-ring propagation. */
    for (i = 0; i < nv; i++) {
        const float *p = originalP + (size_t)i * 3;
        double low = hc_chop_limit(s, p[0], p[2]);
        boundary[i] = (unsigned char)(hc_gr_finite(low) && p[1] > low - g_downClear);
    }
    for (i = 0; i < nt; i++) {
        const uint32_t *f = F + (size_t)i * 3;
        hidden[i] = (unsigned char)(boundary[f[0]] && boundary[f[1]] && boundary[f[2]]);
    }
    for (ring = 0; ring < 2; ring++) {
        memset(boundary, 0, (size_t)nv);
        for (i = 0; i < nt; i++) if (!hidden[i])
            for (k = 0; k < 3; k++) boundary[F[(size_t)i * 3 + k]] = 1;
        for (i = 0; i < nt; i++) if (hidden[i])
            for (k = 0; k < 3; k++) if (boundary[F[(size_t)i * 3 + k]]) { hidden[i] = 0; break; }
    }
    memcpy(out, hidden, (size_t)nt);
    free(hidden); free(boundary);
    return 1;
}

#endif
