/* Version 7 hat fit: makes the chop follow each hat instead of one fixed height.
 *
 * 1. Crown cover: a hat-bone accessory only counts as a hat when straight-down
 *    rays over the scalp top hit it (>= 90%). Circlets, tiaras, animal ears,
 *    earmuffs and headbands no longer chop the hair under them.
 * 2. Rim fit: the hat's lowest point near the head is measured in 36 slices of
 *    10 degrees. For each of the two TS4 cuts, the plane is lifted until it sits
 *    HC_FIT_MARGIN above that edge in every slice (hidden inside the hat), and
 *    the cut whose lifted plane follows the edge most closely is used. This
 *    reproduces TS4's own RegionMap pick for all eight test hats (24 LODs).
 *    Planes already hidden are never moved; nothing is ever lowered.
 * Matches tools/hatchop_cc_hat_check.py (same samples, same sectors).
 */
#ifndef HAT_CHOP_HAT_FIT_H
#define HAT_CHOP_HAT_FIT_H
#include <math.h>
#include <float.h>
#include <string.h>
#include "hat_chop_age.h"

#define HC_FIT_HEAD_X 0.0
#define HC_FIT_HEAD_Z (-0.005)
#define HC_FIT_CROWN_R 0.075      /* scalp top disk sampled for cover */
#define HC_FIT_MIN_COVER 0.9
#define HC_FIT_RIM_R 0.125        /* hat geometry this close to the head axis is "on the head" */
#define HC_FIT_RIM_Y_MIN 1.55     /* ignore capes/drapes below the head */
#define HC_FIT_RIM_Y_MAX 1.85     /* a slice whose lowest point is above this has no band */
#define HC_FIT_SECTORS 36
#define HC_FIT_MIN_SECTORS 24     /* fewer measured slices: keep the old ray pick, no lift */
#define HC_FIT_MARGIN 0.003       /* cut sits 3 mm inside the hat edge */

/* Share of crown sample points under the triangles (xz point-in-triangle).
 * age NULL/adult: the original disk; child/toddler: the disk scaled to that head. */
static double hc_fit_crown_cover_age(const double *tri, int ntri, const HCAgeMap *age)
{
    int ring, k, i, hits = 0, total = 0;
    double crownR = hc_age_len(age, HC_FIT_CROWN_R), headZ = hc_age_z(age, HC_FIT_HEAD_Z);
    if (!tri || ntri <= 0) return 0;
    for (ring = 0; ring <= 5; ring++) {
        double r = crownR * ring / 5.0;
        int n = ring ? (int)(6 + ring / 5.0 * 18 + 1e-9) : 1;
        for (k = 0; k < n; k++) {
            double t = 2 * 3.14159265358979323846 * k / n;
            double x = HC_FIT_HEAD_X + r * sin(t), z = headZ + r * cos(t);
            int hit = 0;
            for (i = 0; i < ntri && !hit; i++) {
                const double *a = tri + (size_t)i * 9, *b = a + 3, *c = a + 6;
                double v0x = c[0] - a[0], v0z = c[2] - a[2], v1x = b[0] - a[0], v1z = b[2] - a[2];
                double v2x = x - a[0], v2z = z - a[2], den = v0x * v1z - v1x * v0z, u, v;
                if (fabs(den) <= 1e-14) continue;
                u = (v2x * v1z - v1x * v2z) / den;
                v = (v0x * v2z - v2x * v0z) / den;
                if (u >= 0 && v >= 0 && u + v <= 1) hit = 1;
            }
            hits += hit; total++;
        }
    }
    return total ? (double)hits / total : 0;
}

static double hc_fit_crown_cover(const double *tri, int ntri)
{
    return hc_fit_crown_cover_age(tri, ntri, NULL);
}

/* Crown cover for the in-game gate, before the head is known: adult hats use the
 * adult disk; a child-or-toddler-height hat counts if it covers either kid head. */
static double hc_fit_crown_cover_any_age(const double *tri, int ntri)
{
    HCAgeMap child, toddler;
    double a, b;
    if (hc_age_class_of_hat(tri, ntri) == 0) return hc_fit_crown_cover(tri, ntri);
    hc_age_class(&child, 1, 1.3112);
    hc_age_class(&toddler, 2, 0.8494);
    a = hc_fit_crown_cover_age(tri, ntri, &child);
    b = hc_fit_crown_cover_age(tri, ntri, &toddler);
    return a > b ? a : b;
}

/* Lowest dense sample per 10-degree slice (0 = front, +z). Returns measured slices.
 * age NULL/adult: the original constants; child/toddler: scaled to that head. */
static int hc_fit_rim(const double *tri, int ntri, double rim[HC_FIT_SECTORS][3], const HCAgeMap *age)
{
    int i, a, b, s, count = 0;
    double yMin = hc_age_y(age, HC_FIT_RIM_Y_MIN), yMax = hc_age_y(age, HC_FIT_RIM_Y_MAX);
    double rimR = hc_age_len(age, HC_FIT_RIM_R), headZ = hc_age_z(age, HC_FIT_HEAD_Z);
    for (s = 0; s < HC_FIT_SECTORS; s++) rim[s][1] = HUGE_VAL;
    if (!tri || ntri <= 0) return 0;
    for (i = 0; i < ntri; i++) {
        const double *t = tri + (size_t)i * 9;
        for (a = 0; a <= 6; a++) for (b = 0; b <= 6 - a; b++) {
            double w0 = a / 6.0, w1 = b / 6.0, w2 = 1 - w0 - w1, p[3], dx, dz, az;
            int k;
            for (k = 0; k < 3; k++) p[k] = w0 * t[k] + w1 * t[3 + k] + w2 * t[6 + k];
            if (!(p[1] > yMin)) continue;
            dx = p[0] - HC_FIT_HEAD_X; dz = p[2] - headZ;
            if (!(sqrt(dx * dx + dz * dz) <= rimR)) continue;
            az = atan2(dx, dz) * (180.0 / 3.14159265358979323846);
            az = fmod(az + 360.0, 360.0);
            s = (int)(az / 10.0);
            if (s < 0) s = 0;
            if (s >= HC_FIT_SECTORS) s = HC_FIT_SECTORS - 1;
            if (p[1] < rim[s][1]) { rim[s][0] = p[0]; rim[s][1] = p[1]; rim[s][2] = p[2]; }
        }
    }
    for (s = 0; s < HC_FIT_SECTORS; s++) {
        if (rim[s][1] > yMax) rim[s][1] = HUGE_VAL;
        else count++;
    }
    return count;
}

/* Per cut: lowest (plane - rim) over slices, mean, and the lift that hides the cut. */
static void hc_fit_score(const double plane[3], double rim[HC_FIT_SECTORS][3],
                         double *worst, double *mean, double *lift)
{
    int s, n = 0;
    double lo = HUGE_VAL, sum = 0;
    for (s = 0; s < HC_FIT_SECTORS; s++) {
        double d;
        if (!(rim[s][1] < HUGE_VAL)) continue;
        d = plane[0] * rim[s][0] + plane[1] * rim[s][2] + plane[2] - rim[s][1];
        if (d < lo) lo = d;
        sum += d; n++;
    }
    *worst = n ? lo : 0;
    *mean = n ? sum / n : 0;
    *lift = n && lo < HC_FIT_MARGIN ? HC_FIT_MARGIN - lo : 0;
}
#endif
