/* v7.1 child and toddler support.
 *
 * Every adult-height number in Hat Chop (TS4 reference cut planes and hair
 * shape, hat-edge slices, crown disk, head collection) is mapped onto a child
 * or toddler head with a per-axis scale about the head top. Ratios are the
 * measured EA bind-pose head proportions (face + scalp: cuFace/cuScalp,
 * puFace/puScalp vs the af/am average). Adults (kid == 0) use the old numbers
 * unchanged, so adult results stay byte-identical.
 *
 *            head top  half-width  height   depth    z center
 *   adult    1.8750    0.0990      0.2558   0.2438   0.0080
 *   child    1.3112    0.0862      0.2155   0.2078   0.0064   ratios x .8699 y .8427 z .8525
 *   toddler  0.8494    0.0738      0.1717   0.1643  -0.0005   ratios x .7454 y .6712 z .6741
 */
#ifndef HAT_CHOP_AGE_H
#define HAT_CHOP_AGE_H

#define HC_AGE_ADULT_TOP 1.8750
#define HC_AGE_ADULT_ZC 0.0080
#define HC_AGE_ADULT_FACE_TOP 1.836   /* face mesh top (af/amFace): head-part cutoff 1.70 sits .136 below */
#define HC_AGE_KID_BELOW 1.55          /* head top (or face top) below this: child or toddler */
#define HC_AGE_TODDLER_BELOW 1.00

typedef struct {
    int kid;                           /* 0 adult/teen/elder, 1 child, 2 toddler */
    double r[3];                       /* x, y, z size ratio vs adult */
    double A[3], K[3];                 /* adult head-top anchor, this head's anchor */
} HCAgeMap;

static void hc_age_class(HCAgeMap *m, int kid, double top)
{
    static const double ratio[3][3] = {{1, 1, 1}, {.8699, .8427, .8525}, {.7454, .6712, .6741}};
    static const double zc[3] = {HC_AGE_ADULT_ZC, .0064, -.0005};
    int k;
    memset(m, 0, sizeof(*m));
    m->kid = kid;
    for (k = 0; k < 3; k++) m->r[k] = ratio[kid][k];
    m->A[0] = 0; m->A[1] = HC_AGE_ADULT_TOP; m->A[2] = HC_AGE_ADULT_ZC;
    m->K[0] = 0; m->K[1] = top; m->K[2] = zc[kid];
}

/* Age from the head triangles' top (scalp top). Adults: m->kid = 0. */
static void hc_age_from_head(const double *tri, int nt, HCAgeMap *m)
{
    double top = -HUGE_VAL;
    int i;
    memset(m, 0, sizeof(*m));
    m->r[0] = m->r[1] = m->r[2] = 1;
    if (!tri || nt <= 0) return;
    for (i = 0; i < nt * 3; i++) if (tri[(size_t)i * 3 + 1] > top) top = tri[(size_t)i * 3 + 1];
    if (!(top < HC_AGE_KID_BELOW) || !(top > 0.2)) return;
    hc_age_class(m, top < HC_AGE_TODDLER_BELOW ? 2 : 1, top);
}

/* Adult-space constant -> this head. Identity for adults (exact). */
static double hc_age_y(const HCAgeMap *m, double y) { return m && m->kid ? m->K[1] + (y - m->A[1]) * m->r[1] : y; }
static double hc_age_z(const HCAgeMap *m, double z) { return m && m->kid ? m->K[2] + (z - m->A[2]) * m->r[2] : z; }
static double hc_age_len(const HCAgeMap *m, double d) { return m && m->kid ? d * .5 * (m->r[0] + m->r[2]) : d; }

/* TS4 source -> game calibration for this head, from the adult one. */
static void hc_age_affine(const HCAgeMap *m, const double *s0, const double *t0, double *s, double *t)
{
    int k;
    for (k = 0; k < 3; k++) {
        s[k] = m->r[k] * s0[k];
        t[k] = m->r[k] * (t0[k] - m->A[k]) + m->K[k];
    }
}

/* Hat-only age class for the crown-cover gate (runs before the head is known):
 * adult hats reach above 1.55 m, toddler hats stay under 1.0 m. */
static int hc_age_class_of_hat(const double *tri, int nt)
{
    double top = -HUGE_VAL;
    int i;
    for (i = 0; i < nt * 3; i++) if (tri[(size_t)i * 3 + 1] > top) top = tri[(size_t)i * 3 + 1];
    return top >= HC_AGE_KID_BELOW ? 0 : top >= HC_AGE_TODDLER_BELOW ? 1 : 2;
}
#endif
